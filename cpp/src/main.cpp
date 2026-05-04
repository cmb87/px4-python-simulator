#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <cmath>
#include <memory>
#include <cstdlib>

#include "x8/parameters.hpp"
#include "x8/forces.hpp"
#include "iris/parameters.hpp"
#include "iris/forces.hpp"
#include "dynamics/dynamics.hpp"
#include "sensors/sensors.hpp"
#include "networking/websocket_publisher.hpp"
#include "networking/mavlink_interface.hpp"
#include "networking/mavlink_simulator.hpp"
#include "sensors/sensor_params.hpp"

using namespace std::chrono_literals;

namespace {

class VehicleModel {
public:
    virtual ~VehicleModel() = default;
    virtual VehicleParameters& get_params() = 0;
    virtual Eigen::VectorXd calculate_forces(double t, double dt, const Eigen::VectorXd& y, const Eigen::VectorXd& u, const Eigen::Matrix<double, 6, 1>& wind) = 0;
};

class X8Model : public VehicleModel {
public:
    VehicleParameters& get_params() override { return params; }
    Eigen::VectorXd calculate_forces(double t, double dt, const Eigen::VectorXd& y, const Eigen::VectorXd& u, const Eigen::Matrix<double, 6, 1>& wind) override {
        return x8::forces(t, y, u, wind, params);
    }
private:
    X8Parameters params;
};

class IrisModel : public VehicleModel {
public:
    IrisModel() {
        force_model.configure_from_parameters(params);
    }
    VehicleParameters& get_params() override { return params; }
    Eigen::VectorXd calculate_forces(double t, double dt, const Eigen::VectorXd& y, const Eigen::VectorXd& u, const Eigen::Matrix<double, 6, 1>& wind) override {
        return force_model.update(dt, y, u, wind, params);
    }
private:
    iris::IrisParameters params;
    iris::IrisQuadForceModel force_model;
};

Eigen::Vector4d rail_alignment_quaternion_wxyz(const VehicleParameters& params) {
    Eigen::Vector3d rail_dir = params.rail_dir_ned;
    double rail_norm = rail_dir.norm();
    if (rail_norm <= 1e-9) {
        return Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
    }
    rail_dir /= rail_norm;

    double yaw = std::atan2(rail_dir[1], rail_dir[0]);
    double horiz = std::hypot(rail_dir[0], rail_dir[1]);
    double pitch = std::atan2(-rail_dir[2], std::max(horiz, 1e-9));

    double cr = std::cos(0.0 * 0.5);
    double sr = std::sin(0.0 * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);

    Eigen::Vector4d q;
    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
    return q;
}

Eigen::VectorXd rail_forces(const Eigen::VectorXd& y, const VehicleParameters& params) {
    Eigen::Vector3d rail_dir = params.rail_dir_ned;
    double rail_norm = rail_dir.norm();
    if (rail_norm <= 1e-9) {
        return Eigen::VectorXd::Zero(6);
    }
    rail_dir /= rail_norm;

    Eigen::Vector3d pos = y.segment<3>(0);
    double rail_dist = (pos - params.rail_start_ned).dot(rail_dir);
    double rail_force_ned = params.rail_pull_max * params.gravity * (params.rail_length - rail_dist);
    if (rail_force_ned < 0.0) {
        rail_force_ned = 0.0;
    }

    Eigen::Vector4d quat = rail_alignment_quaternion_wxyz(params);
    Eigen::Matrix3d Mfg = dynamics::Mfg_from_quat(quat);
    Eigen::Vector3d force_body = Mfg * (rail_force_ned * rail_dir);

    Eigen::VectorXd out = Eigen::VectorXd::Zero(6);
    out.segment<3>(0) = force_body;
    return out;
}

}

int main() {
    std::string model_name = "x8";
    if (const char* env_p = std::getenv("SIM_VEHICLE_MODEL")) {
        model_name = env_p;
    }
    std::cout << "Starting C++ " << model_name << " Simulation..." << std::endl;

    std::unique_ptr<VehicleModel> vehicle;
    if (model_name == "iris") {
        vehicle = std::make_unique<IrisModel>();
    } else {
        vehicle = std::make_unique<X8Model>();
    }

    VehicleParameters& params = vehicle->get_params();
    dynamics::State state;
    if (params.rail_launch_enabled) {
        state.y.segment<3>(0) = params.rail_start_ned;
        state.y.segment<4>(3) = rail_alignment_quaternion_wxyz(params);
        state.y.segment<3>(10).setZero();
        params.left_rail = false;
    }
    Eigen::VectorXd u = Eigen::VectorXd::Zero(4);
    Eigen::Vector3d wind = Eigen::Vector3d::Zero();
    
    sensors::SensorSuite sensor_suite;

    auto it_lat = params.gps_origin.find("lat");
    auto it_lon = params.gps_origin.find("lon");
    auto it_alt = params.gps_origin.find("alt");

    networking::WebSocketPublisher ws_pub;
    ws_pub.start(8765);

    networking::MavlinkInterface mav_interface;
    networking::MavlinkSimulator mav_sim(mav_interface);
    std::atomic<bool> controls_received{false};
    std::atomic<bool> armed{false};
    std::atomic<bool> ever_armed{false};
    
    mav_interface.set_on_controls([&](const mavlink_hil_actuator_controls_t& controls) {
        u[0] = controls.controls[0]; 
        u[1] = controls.controls[1]; 
        u[2] = controls.controls[2]; 
        u[3] = controls.controls[3]; 
        
        armed = (controls.mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
        if (armed) {
            ever_armed = true;
        }
        controls_received = true;
    });

    try {
        mav_interface.listen("0.0.0.0", 4560);
    } catch (const std::exception& e) {
        std::cerr << "Failed to start MAVLink server: " << e.what() << std::endl;
        return 1;
    }

    // --- EXACT PYTHON SEQUENCE: WAIT FOR FIRST HEARTBEAT ---
    std::cout << "Waiting for MAVLink heartbeat from PX4..." << std::endl;
    while (!mav_interface.has_target()) {
        std::this_thread::sleep_for(10ms);
    }
    
    uint8_t px4_sysid = mav_interface.get_target_system();
    const uint8_t sim_sysid = 42;
    mav_interface.set_source_system(sim_sysid);
    std::cout << "PX4 detected! SYSID=" << (int)px4_sysid << ". Starting simulation loop." << std::endl;

    uint64_t sim_time_us = 0;
    const int check_factor = 2;
    const uint64_t physics_dt_us = 8000;
    const uint64_t loop_dt_us = 4000;
    const uint64_t gps_start_time_us = 1000000;

    uint64_t last_heartbeat_time_us = 0;
    uint64_t last_system_time_us = 0;
    uint64_t next_hil_state_time_us = 0;
    uint64_t slow_down_counter = 0;

    while (true) {
        if (!mav_interface.is_running()) {
            std::cout << "MAVLink interface stopped. Exiting." << std::endl;
            break;
        }

        auto step_start_time = std::chrono::steady_clock::now();
        
        bool io_run_only = (slow_down_counter % check_factor) != 0;

        if (!io_run_only) {
            sim_time_us += physics_dt_us;

            // 1. Calculate forces and ydot (before integrator step for temporal consistency)
            Eigen::Matrix<double, 6, 1> wind_6d = Eigen::Matrix<double, 6, 1>::Zero();
            Eigen::VectorXd tau = Eigen::VectorXd::Zero(6);
            Eigen::VectorXd ydot = Eigen::VectorXd::Zero(13);

            if (ever_armed) {
                tau = vehicle->calculate_forces(sim_time_us * 1e-6, 0.008, state.y, u, wind_6d);
                
                if (params.rail_launch_enabled && !params.left_rail) {
                    tau += rail_forces(state.y, params);
                    ydot = dynamics::rail_dynamics(sim_time_us * 1e-6, state.y, params, tau);
                    
                    auto rail_func = [&](double t, const Eigen::VectorXd& y) {
                        return dynamics::rail_dynamics(t, y, params, tau);
                    };
                    state.y = dynamics::IntegratorEuler::step(sim_time_us * 1e-6, 0.008, state.y, rail_func);
                    state.y.segment<4>(3) = rail_alignment_quaternion_wxyz(params);
                    state.y.segment<3>(10).setZero();
                } else {
                    ydot = dynamics::dynamics_6dof(sim_time_us * 1e-6, state.y, params, tau);
                    
                    auto dynamics_func = [&](double t, const Eigen::VectorXd& y) {
                        return dynamics::dynamics_6dof(t, y, params, tau);
                    };
                    state.y = dynamics::IntegratorEuler::step(sim_time_us * 1e-6, 0.008, state.y, dynamics_func);
                    state.y.segment<4>(3).normalize();
                }

                // Ground constraint
                if (state.y[2] >= 0.0) {
                    state.y[2] = 0.0;
                    Eigen::Matrix3d Mfg_tmp = dynamics::Mfg_from_quat(state.y.segment<4>(3));
                    Eigen::Matrix3d Mgf_tmp = Mfg_tmp.transpose();
                    Eigen::Vector3d vel_ned_tmp = Mgf_tmp * state.y.segment<3>(7);
                    if (vel_ned_tmp[2] > 0.0) {
                        vel_ned_tmp[2] = 0.0;
                        state.y.segment<3>(7) = Mfg_tmp * vel_ned_tmp;
                    }
                }
            }

            // 3. Update sensors
            sensor_suite.update((double)sim_time_us, false, state.y, ydot, wind, params);
            const auto& s_out = sensor_suite.get_outputs();

            // Ground-truth LLA for State and WebSocket
            double gt_lat = it_lat->second;
            double gt_lon = it_lon->second;
            double gt_alt = it_alt->second - state.y[2];
            {
                double x_rad = state.y[0] / R_EARTH;
                double y_rad = state.y[1] / R_EARTH;
                double c = std::sqrt(x_rad*x_rad + y_rad*y_rad);
                if (c > 0.0) {
                    double sin_c = std::sin(c), cos_c = std::cos(c);
                    double lat0_rad = it_lat->second * M_PI / 180.0;
                    double lon0_rad = it_lon->second * M_PI / 180.0;
                    double sin_lat0 = std::sin(lat0_rad), cos_lat0 = std::cos(lat0_rad);
                    double lat_rad = std::asin(cos_c * sin_lat0 + (x_rad * sin_c * cos_lat0) / c);
                    gt_lat = lat_rad * 180.0 / M_PI;
                    gt_lon = (lon0_rad + std::atan2(y_rad * sin_c, c * cos_lat0 * cos_c - x_rad * sin_lat0 * sin_c)) * 180.0 / M_PI;
                }
            }

            // Ground-truth NED velocity, airspeeds, and body acceleration
            Eigen::Matrix3d Mfg = dynamics::Mfg_from_quat(state.y.segment<4>(3));
            Eigen::Matrix3d Mgf = Mfg.transpose();
            Eigen::Vector3d vel_ned = Mgf * state.y.segment<3>(7);
            double va = state.y.segment<3>(7).norm();
            Eigen::Vector3d accel_body = ydot.segment<3>(7);

            // 4. Send MAVLink reports
            
            // HIL_SENSOR
            mav_sim.send_hil_sensor(sim_time_us, s_out, params.has_airspeed_sensor);

            // HIL_STATE_QUATERNION
            int32_t hil_state_interval = mav_interface.get_hil_state_interval_us();
            if (hil_state_interval > 0 && sim_time_us >= next_hil_state_time_us) {
                mav_sim.send_hil_state_quaternion(sim_time_us, state.y, vel_ned, va, va, gt_lat, gt_lon, gt_alt, accel_body);
                next_hil_state_time_us = sim_time_us + (hil_state_interval > 0 ? hil_state_interval : 0);
            }

            // HIL_GPS
            if (sim_time_us >= gps_start_time_us) {
                mav_sim.send_hil_gps(sim_time_us, s_out);
            }

            // 5. Publish to WebSocket (every 32ms sim time)
            if (sim_time_us % 32000 < 8000) {
                double alpha = 0.0, beta = 0.0;
                Eigen::Vector3d vel_rel = state.y.segment<3>(7); 
                double va_ws = vel_rel.norm();
                if (va_ws > 1e-6) {
                    alpha = std::atan2(vel_rel[2], vel_rel[0]) * 180.0 / M_PI;
                    beta = std::asin(vel_rel[1] / va_ws) * 180.0 / M_PI;
                }

                nlohmann::json j;
                j["system_id"] = (int)sim_sysid;
                j["time_usec"] = sim_time_us;
                j["u"] = {u[0], u[1], u[2], u[3], 0.0, 0.0, 0.0, 0.0}; 
                j["position_ned_m"] = {state.y[0], state.y[1], state.y[2]};
                j["quaternion_wxyz"] = {state.y[3], state.y[4], state.y[5], state.y[6]};
                j["velocity_body_mps"] = {state.y[7], state.y[8], state.y[9]};
                j["angular_rate_body_rps"] = {state.y[10], state.y[11], state.y[12]};
                j["lla"] = {{"lat_deg", gt_lat}, {"lon_deg", gt_lon}, {"alt_m", gt_alt}};
                j["aero"] = {{"alpha_deg", alpha}, {"beta_deg", beta}};
                
                // Add sensor data
                j["sensors"] = {
                    {"accel", {s_out.accel[0], s_out.accel[1], s_out.accel[2]}},
                    {"gyro", {s_out.gyro[0], s_out.gyro[1], s_out.gyro[2]}},
                    {"mag", {s_out.mag[0], s_out.mag[1], s_out.mag[2]}},
                    {"baro_alt", s_out.baro.pressure_alt},
                    {"gps_lla", {s_out.gps.lat_deg, s_out.gps.lon_deg, s_out.gps.alt_m}},
                    {"airspeed_ias", s_out.airspeed.ias}
                };
                j["euler_deg"] = {s_out.euler_deg[0], s_out.euler_deg[1], s_out.euler_deg[2]};

                ws_pub.publish(j);
            }
        } // end of if (!io_run_only)

        // ALWAYS (every 4ms):
        // Heartbeat and System Time logic
        if (sim_time_us >= last_heartbeat_time_us + 1000000) {
            mav_sim.send_heartbeat();
            last_heartbeat_time_us = sim_time_us;
        }

        if (sim_time_us >= last_system_time_us + 1000000) {
            mav_sim.send_system_time(sim_time_us);
            last_system_time_us = sim_time_us;
        }

        slow_down_counter++;

        // 6. Sleep to maintain the target simulation rate
        auto elapsed = std::chrono::steady_clock::now() - step_start_time;
        auto sleep_duration = std::chrono::microseconds(loop_dt_us) - elapsed;
        if (sleep_duration > 0ms) {
            std::this_thread::sleep_for(sleep_duration);
        }
    }


    return 0;
}


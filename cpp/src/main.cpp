#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <cmath>

#include "x8/parameters.hpp"
#include "x8/forces.hpp"
#include "dynamics/dynamics.hpp"
#include "sensors/sensors.hpp"
#include "networking/websocket_publisher.hpp"
#include "networking/mavlink_interface.hpp"

using namespace std::chrono_literals;

namespace {

Eigen::Vector4d rail_alignment_quaternion_wxyz(const X8Parameters& params) {
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

Eigen::VectorXd rail_forces(const Eigen::VectorXd& y, const X8Parameters& params) {
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
    std::cout << "Starting C++ x8 Simulation..." << std::endl;

    X8Parameters params;
    dynamics::State state;
    if (params.rail_launch_enabled) {
        state.y.segment<3>(0) = params.rail_start_ned;
        state.y.segment<4>(3) = rail_alignment_quaternion_wxyz(params);
        state.y.segment<3>(10).setZero();
        params.left_rail = false;
    }
    Eigen::VectorXd u = Eigen::VectorXd::Zero(4);
    Eigen::Vector3d wind = Eigen::Vector3d::Zero();
    
    sensors::IMUSensor imu;
    sensors::BarometerSensor baro;
    sensors::GPSSensor gps;
    sensors::MagnetometerSensor mag;

    // Match Python behavior: all sensors include noise.
    imu.set_noise(true);
    baro.set_noise(true);
    gps.set_noise(true);
    mag.set_noise(true);
    
    gps.set_home(params.gps_origin["lat"], params.gps_origin["lon"], params.gps_origin["alt"]);
    baro.set_home_altitude(params.gps_origin["alt"]);

    networking::WebSocketPublisher ws_pub;
    ws_pub.start(8765);

    networking::MavlinkInterface mav_interface;
    std::atomic<bool> controls_received{false};
    std::atomic<bool> armed{false};
    std::atomic<bool> ever_armed{false};
    
    mav_interface.set_on_controls([&](const mavlink_hil_actuator_controls_t& controls) {
        u[0] = controls.controls[0]; // Throttle
        u[1] = controls.controls[1]; // Left elevon
        u[2] = controls.controls[2]; // Right elevon
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
    
    uint8_t sysid = mav_interface.get_target_system();
    mav_interface.set_source_system(sysid);
    std::cout << "PX4 detected! SYSID=" << (int)sysid << ". Starting simulation loop." << std::endl;

    uint64_t sim_time_us = 0;
    const double dt = 0.004; // 250 Hz
    const uint64_t dt_us = 4000;
    const uint64_t gps_start_time_us = 1000000;

    auto last_heartbeat_time_us = sim_time_us;
    auto last_system_time_us = sim_time_us;
    uint64_t next_hil_state_time_us = 0;

    while (true) {
        if (!mav_interface.is_running()) {
            std::cout << "MAVLink interface stopped. Exiting." << std::endl;
            break;
        }

        auto step_start_time = std::chrono::steady_clock::now();

        // 1. Calculate forces
        Eigen::Matrix<double, 6, 1> wind_6d = Eigen::Matrix<double, 6, 1>::Zero();
        Eigen::VectorXd tau = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd ydot = Eigen::VectorXd::Zero(13);
        auto dynamics_func = [&](double t, const Eigen::VectorXd& y) {
            return dynamics::dynamics_6dof(t, y, params, tau);
        };

        if (ever_armed) {
            tau = x8::forces(sim_time_us * 1e-6, state.y, u, wind_6d, params);
            if (params.rail_launch_enabled && !params.left_rail) {
                state.y.segment<4>(3) = rail_alignment_quaternion_wxyz(params);
                state.y.segment<3>(10).setZero();
                tau += rail_forces(state.y, params);
                auto rail_func = [&](double t, const Eigen::VectorXd& y) {
                    return dynamics::rail_dynamics(t, y, params, tau);
                };
                state.y = dynamics::IntegratorRK4::step(sim_time_us * 1e-6, dt, state.y, rail_func);
                state.y.segment<4>(3) = rail_alignment_quaternion_wxyz(params);
                state.y.segment<3>(10).setZero();
            } else {
                state.y = dynamics::IntegratorRK4::step(sim_time_us * 1e-6, dt, state.y, dynamics_func);
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

            if (params.rail_launch_enabled && !params.left_rail) {
                ydot = dynamics::rail_dynamics((sim_time_us + dt_us) * 1e-6, state.y, params, tau);
            } else {
                ydot = dynamics_func((sim_time_us + dt_us) * 1e-6, state.y);
            }
        }

        sim_time_us += dt_us;

        // 3. Update sensors
        Eigen::Matrix3d Mfg = dynamics::Mfg_from_quat(state.y.segment<4>(3));
        Eigen::Matrix3d Mgf = Mfg.transpose();
        Eigen::Vector3d vel_ned = Mgf * state.y.segment<3>(7);

        imu.update(dt, ydot.segment<3>(7), state.y.segment<3>(7), state.y.segment<3>(10), state.y.segment<4>(3));
        double alt_amsl = params.gps_origin["alt"] - state.y[2];
        baro.update(dt, alt_amsl);
        gps.update(dt, state.y.segment<3>(0), vel_ned); 
        mag.update(dt, state.y.segment<4>(3), params.magnetic_ned);

        auto gps_data = gps.get_data();
        const double gps_vel_n = gps_data.vel[0];
        const double gps_vel_e = gps_data.vel[1];
        const double gps_vel_up = gps_data.vel[2];
        const double gps_vel_d = -gps_vel_up;

        // Ground-truth LLA for State and WebSocket
        double gt_lat = params.gps_origin["lat"];
        double gt_lon = params.gps_origin["lon"];
        double gt_alt = params.gps_origin["alt"] - state.y[2];
        {
            const double R_EARTH = 6371000.0;
            double x_rad = state.y[0] / R_EARTH;
            double y_rad = state.y[1] / R_EARTH;
            double c = std::sqrt(x_rad*x_rad + y_rad*y_rad);
            if (c > 0.0) {
                double sin_c = std::sin(c), cos_c = std::cos(c);
                double lat0_rad = params.gps_origin["lat"] * M_PI / 180.0;
                double lon0_rad = params.gps_origin["lon"] * M_PI / 180.0;
                double sin_lat0 = std::sin(lat0_rad), cos_lat0 = std::cos(lat0_rad);
                double lat_rad = std::asin(cos_c * sin_lat0 + (x_rad * sin_c * cos_lat0) / c);
                gt_lat = lat_rad * 180.0 / M_PI;
                gt_lon = (lon0_rad + std::atan2(y_rad * sin_c, c * cos_lat0 * cos_c - x_rad * sin_lat0 * sin_c)) * 180.0 / M_PI;
            }
        }

        // 4. Send MAVLink reports (ALWAYS sysid matched to PX4, compid 51)
        mavlink_message_t msg;
        auto acc = imu.get_accel();
        auto gyro = imu.get_gyro();
        auto mag_val = mag.get_mag();
        
        // HIL_SENSOR
        {
            // Match Python behavior exactly: mark all HIL_SENSOR fields updated,
            // only clear diff pressure bit if no airspeed sensor.
            uint32_t fields_updated = 8191;

            double diff_pressure_pa = 0.0;
            if (params.has_airspeed_sensor) {
                double va = state.y.segment<3>(7).norm();
                diff_pressure_pa = 0.5 * params.rho * va * va;
            } else {
                fields_updated &= ~(1u << 10);
            }

            mavlink_msg_hil_sensor_pack(sysid, 51, &msg,
                sim_time_us,
                acc[0], acc[1], acc[2],
                gyro[0], gyro[1], gyro[2],
                mag_val[0], mag_val[1], mag_val[2],
                baro.get_pressure_hpa(),
                diff_pressure_pa * 0.01, baro.get_altitude(), 15.0, fields_updated, 0
            );
            mav_interface.send_message(msg);
        }

        // HIL_STATE_QUATERNION
        int32_t hil_state_interval = mav_interface.get_hil_state_interval_us();
        if (hil_state_interval > 0 && sim_time_us >= next_hil_state_time_us) {
            float q[4] = {(float)state.y[3], (float)state.y[4], (float)state.y[5], (float)state.y[6]};
            float horiz_speed = std::sqrt(gps_vel_n * gps_vel_n + gps_vel_e * gps_vel_e);
            const double m_s2_to_mg = 1000.0 / 9.80665;
            mavlink_msg_hil_state_quaternion_pack(sysid, 51, &msg,
                sim_time_us, q, state.y[10], state.y[11], state.y[12],
                (int32_t)(gt_lat * 1e7), (int32_t)(gt_lon * 1e7), (int32_t)(gt_alt * 1000),
                (int16_t)(gps_vel_n * 100), (int16_t)(gps_vel_e * 100), (int16_t)(gps_vel_d * 100),
                (uint16_t)(horiz_speed * 100), (uint16_t)(horiz_speed * 100),
                (int16_t)(acc[0] * m_s2_to_mg), (int16_t)(acc[1] * m_s2_to_mg), (int16_t)(acc[2] * m_s2_to_mg)
            );
            mav_interface.send_message(msg);
            next_hil_state_time_us = sim_time_us + (hil_state_interval > 0 ? hil_state_interval : 0);
        }

        // HIL_GPS
        if (sim_time_us >= gps_start_time_us && gps.is_updated()) {
            double cog_deg = std::atan2(gps_vel_e, gps_vel_n) * 180.0 / M_PI;
            if (cog_deg < 0) cog_deg += 360.0;
            const double gps_speed_3d = std::sqrt(gps_vel_n * gps_vel_n + gps_vel_e * gps_vel_e + gps_vel_d * gps_vel_d);
            
            mavlink_msg_hil_gps_pack(sysid, 51, &msg,
                sim_time_us, 3, (int32_t)(gps_data.lat * 1e7), (int32_t)(gps_data.lon * 1e7), (int32_t)(gps_data.alt * 1000),
                100, 100, (uint16_t)(gps_speed_3d * 100),
                (int16_t)(gps_vel_n * 100), (int16_t)(gps_vel_e * 100), (int16_t)(gps_vel_d * 100),
                (uint16_t)(cog_deg * 100), 10, 0, 0
            );
            mav_interface.send_message(msg);
        }

        // Periodic HEARTBEAT (every 1s sim time)
        if (sim_time_us >= last_heartbeat_time_us + 1000000) {
            mavlink_msg_heartbeat_pack(sysid, 51, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
            mav_interface.send_message(msg);
            last_heartbeat_time_us = sim_time_us;
        }

        // Periodic SYSTEM_TIME (every 1s sim time)
        if (sim_time_us >= last_system_time_us + 1000000) {
            uint64_t unix_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            mavlink_msg_system_time_pack(sysid, 51, &msg, unix_time, sim_time_us / 1000);
            mav_interface.send_message(msg);
            last_system_time_us = sim_time_us;
        }

        // 5. Publish to WebSocket (every 32ms sim time)
        if (sim_time_us % 32000 < dt_us) {
            double alpha = 0.0, beta = 0.0;
            Eigen::Vector3d vel_rel = state.y.segment<3>(7); 
            double va = vel_rel.norm();
            if (va > 1e-6) {
                alpha = std::atan2(vel_rel[2], vel_rel[0]) * 180.0 / M_PI;
                beta = std::asin(vel_rel[1] / va) * 180.0 / M_PI;
            }

            nlohmann::json j;
            j["system_id"] = (int)sysid;
            j["time_usec"] = sim_time_us;
            j["u"] = {u[0], u[1], u[2], u[3], 0.0, 0.0, 0.0, 0.0}; 
            j["position_ned_m"] = {state.y[0], state.y[1], state.y[2]};
            j["quaternion_wxyz"] = {state.y[3], state.y[4], state.y[5], state.y[6]};
            j["velocity_body_mps"] = {state.y[7], state.y[8], state.y[9]};
            j["angular_rate_body_rps"] = {state.y[10], state.y[11], state.y[12]};
            j["lla"] = {{"lat_deg", gt_lat}, {"lon_deg", gt_lon}, {"alt_m", gt_alt}};
            j["aero"] = {{"alpha_deg", alpha}, {"beta_deg", beta}};
            ws_pub.publish(j);
        }

        // 6. Sleep to maintain 250Hz rate
        auto elapsed = std::chrono::steady_clock::now() - step_start_time;
        auto sleep_duration = 4ms - elapsed;
        if (sleep_duration > 0ms) {
            std::this_thread::sleep_for(sleep_duration);
        }
    }

    return 0;
}

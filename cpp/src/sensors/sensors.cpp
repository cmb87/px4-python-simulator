#include "sensors.hpp"
#include "../dynamics/vehicle_parameters.hpp"
#include "../dynamics/dynamics.hpp"
#include "sensor_params.hpp"
#include <cmath>

namespace sensors {

SensorSuite::SensorSuite() {
    imu.set_noise(true);
    mag.set_noise(true);
    mag.set_update_rate(MAG_UPDATE_RATE_HZ);
    gps.set_noise(true);
    gps.set_update_rate(GPS_UPDATE_RATE_HZ);
    baro.set_update_rate(BARO_UPDATE_RATE_HZ);
    baro.set_drift_rate(BARO_DRIFT_PA_S);
    baro.set_noise(true);
    airspeed.set_update_rate(AIRSPEED_UPDATE_RATE_HZ);
}

void SensorSuite::initialize_from_parameters(const ::VehicleParameters& P) {
    gps.set_home(P.gps_origin.at("lat"), P.gps_origin.at("lon"), P.gps_origin.at("alt"));
    baro.set_home_altitude(P.gps_origin.at("alt"));
    baro.set_gravity(P.gravity);
    imu.set_gravity(P.gravity);
    params_initialized_ = true;
}

void SensorSuite::update(double t_us, bool paused, const Eigen::VectorXd& y, const Eigen::VectorXd& ydot, const Eigen::Vector3d& wind, const ::VehicleParameters& P) {
    if (paused) return;

    if (!params_initialized_) {
        initialize_from_parameters(P);
    }

    imu.update(t_us, y, ydot);
    mag.update(t_us, y);
    baro.update(t_us, y);
    gps.update(t_us, y);
    airspeed.update(t_us, y, wind, P.rho, P.pitot_axis_body);

    outputs_.accel = imu.get_accel();
    outputs_.gyro = imu.get_gyro();
    outputs_.mag = mag.get_mag_field_body();
    
    outputs_.baro.abs_pressure_pa = baro.get_pressure_hpa() * 100.0;
    outputs_.baro.pressure_alt = baro.get_pressure_altitude();
    outputs_.baro.temp_c = baro.get_temperature_c();
    
    outputs_.airspeed.ias = airspeed.get_ias_mps();
    outputs_.airspeed.tas = airspeed.get_tas_mps();
    outputs_.airspeed.dynamic_pressure_pa = airspeed.get_dynamic_pressure_pa();
    
    outputs_.gps = gps.get_data();
    
    outputs_.imu_updated = imu.is_updated();
    outputs_.mag_updated = mag.is_updated();
    outputs_.baro_updated = baro.is_updated();
    outputs_.gps_updated = gps.is_updated();
    outputs_.airspeed_updated = airspeed.is_updated();
    
    Eigen::Vector4d quat = y.segment<4>(3);
    double q_norm = quat.norm();
    if (q_norm > 1e-9) quat /= q_norm;
    
    // Euler angles in degrees
    Eigen::Vector3d euler_rad = dynamics::euler_from_quat(quat);
    outputs_.euler_deg = euler_rad * 180.0 / M_PI;
}

} // namespace sensors

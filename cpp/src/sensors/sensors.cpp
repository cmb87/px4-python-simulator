#include "sensors.hpp"
#include "../dynamics/dynamics.hpp"
#include <algorithm>
#include <cmath>

namespace sensors {

// --- IMU ---
IMUSensor::IMUSensor() : gen(std::random_device{}()), dist(0.0, 1.0) {}

void IMUSensor::update(double dt, const Eigen::Vector3d& acc_body_rate, const Eigen::Vector3d& vel_body, const Eigen::Vector3d& gyro_body, const Eigen::Vector4d& quat) {
    Eigen::Matrix3d Mfg = dynamics::Mfg_from_quat(quat);
    Eigen::Vector3d gravity_body = Mfg * gravity_vec;
    
    Eigen::Vector3d accel_for_imu = acc_body_rate + gyro_body.cross(vel_body);
    Eigen::Vector3d specific_force = accel_for_imu + gravity_body;

    // LPF (120 Hz)
    double cutoff_hz = 120.0;
    double tau = 1.0 / (2.0 * M_PI * cutoff_hz);
    double alpha = dt / (tau + dt);

    if (!lpf_initialized) {
        acc_lpf_state = specific_force;
        gyro_lpf_state = gyro_body;
        lpf_initialized = true;
    } else {
        acc_lpf_state = acc_lpf_state + alpha * (specific_force - acc_lpf_state);
        gyro_lpf_state = gyro_lpf_state + alpha * (gyro_body - gyro_lpf_state);
    }
    
    if (enable_noise) {
        // Match Python ADIS16448IMU: white noise + first-order Gauss-Markov bias
        const double dt_safe = std::max(dt, 1e-6);

        const double gyro_noise_density = 2.0 * 35.0 / 3600.0 / 180.0 * M_PI;
        const double gyro_random_walk = 2.0 * 4.0 / 3600.0 / 180.0 * M_PI;
        const double gyro_bias_tau = 1.0e3;

        const double acc_noise_density = 2.0 * 1.2e-3;
        const double acc_random_walk = 2.0 * 1.5e-3;
        const double acc_bias_tau = 300.0;

        const double gyro_phi = std::exp(-dt_safe / gyro_bias_tau);
        const double acc_phi = std::exp(-dt_safe / acc_bias_tau);

        const double gyro_sigma_b = std::sqrt(
            -gyro_random_walk * gyro_random_walk * gyro_bias_tau / 2.0
            * (std::exp(-2.0 * dt_safe / gyro_bias_tau) - 1.0)
        );
        const double acc_sigma_b = std::sqrt(
            -acc_random_walk * acc_random_walk * acc_bias_tau / 2.0
            * (std::exp(-2.0 * dt_safe / acc_bias_tau) - 1.0)
        );

        const double gyro_sigma_d = gyro_noise_density / std::sqrt(dt_safe);
        const double acc_sigma_d = acc_noise_density / std::sqrt(dt_safe);

        gyro_bias = gyro_phi * gyro_bias + Eigen::Vector3d(dist(gen), dist(gen), dist(gen)) * gyro_sigma_b;
        acc_bias = acc_phi * acc_bias + Eigen::Vector3d(dist(gen), dist(gen), dist(gen)) * acc_sigma_b;

        gyro_meas = gyro_lpf_state + gyro_bias + Eigen::Vector3d(dist(gen), dist(gen), dist(gen)) * gyro_sigma_d;
        acc_meas = acc_lpf_state + acc_bias + Eigen::Vector3d(dist(gen), dist(gen), dist(gen)) * acc_sigma_d;
    } else {
        acc_meas = acc_lpf_state + acc_bias;
        gyro_meas = gyro_lpf_state + gyro_bias;
    }
}

// --- Barometer ---
BarometerSensor::BarometerSensor() : gen(std::random_device{}()), dist(0.0, 1.0) {}

void BarometerSensor::update(double dt, double alt_amsl) {
    time_since_last_update += dt;
    if (time_since_last_update < update_interval) {
        updated = false;
        return;
    }
    updated = true;
    time_since_last_update = 0;

    const double ISA_TEMPERATURE_MSL_K = 288.15;
    const double ISA_PRESSURE_MSL_PA = 101325.0;
    const double ISA_LAPSE_RATE_K_PER_M = 0.0065;
    const double ISA_AIR_DENSITY_MSL_KGPM3 = 1.225;
    const double GRAVITY = 9.80665;
    
    // ISA Temperature at altitude
    double temperature = ISA_TEMPERATURE_MSL_K - ISA_LAPSE_RATE_K_PER_M * alt_amsl;
    
    // ISA Pressure (Power Law)
    double pressure_ratio = std::pow(ISA_TEMPERATURE_MSL_K / temperature, 5.256);
    double absolute_pressure = ISA_PRESSURE_MSL_PA / pressure_ratio;

    double noise = enable_noise ? dist(gen) * 0.35 : 0.0;
    double pressure_noisy = absolute_pressure + noise;
    
    pressure_hpa = pressure_noisy * 0.01;

    // pressure altitude (approximate, using density ratio)
    double density_ratio = std::pow(ISA_TEMPERATURE_MSL_K / temperature, 4.256);
    double air_density = ISA_AIR_DENSITY_MSL_KGPM3 / density_ratio;
    pressure_altitude = alt_amsl - noise / (GRAVITY * air_density);
}

// --- GPS ---
GPSSensor::GPSSensor() : gen(std::random_device{}()), dist(0.0, 1.0) {
    lat_home = 0; lon_home = 0; alt_home = 0;
}

void GPSSensor::set_home(double lat_deg, double lon_deg, double alt_m) {
    lat_home = lat_deg * M_PI / 180.0;
    lon_home = lon_deg * M_PI / 180.0;
    alt_home = alt_m;
}

void GPSSensor::update(double dt, const Eigen::Vector3d& pos_ned, const Eigen::Vector3d& vel_ned) {
    time_since_last_update += dt;
    if (time_since_last_update < update_interval) {
        updated = false;
        return;
    }
    updated = true;
    time_since_last_update = 0;

    const double R_EARTH = 6371000.0;
    
    const double dt_safe = std::max(dt, 1e-6);

    const double xy_noise_density = 2.0e-4;
    const double z_noise_density = 4.0e-4;
    const double vxy_noise_density = 0.2;
    const double vz_noise_density = 0.18;
    const double xy_random_walk = 2.0;
    const double z_random_walk = 1.6;
    const double correlation_time = 60.0;

    Eigen::Vector3d noise_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d noise_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d random_walk = Eigen::Vector3d::Zero();
    if (enable_noise) {
        noise_pos <<
            xy_noise_density * std::sqrt(dt_safe) * dist(gen),
            xy_noise_density * std::sqrt(dt_safe) * dist(gen),
            z_noise_density * std::sqrt(dt_safe) * dist(gen);

        noise_vel <<
            vxy_noise_density * std::sqrt(dt_safe) * dist(gen),
            vxy_noise_density * std::sqrt(dt_safe) * dist(gen),
            vz_noise_density * std::sqrt(dt_safe) * dist(gen);

        random_walk <<
            xy_random_walk * std::sqrt(dt_safe) * dist(gen),
            xy_random_walk * std::sqrt(dt_safe) * dist(gen),
            z_random_walk * std::sqrt(dt_safe) * dist(gen);
    }

    for (int i = 0; i < 3; ++i) {
        bias[i] += random_walk[i] * dt_safe - bias[i] / correlation_time;
    }
    
    Eigen::Vector3d noisy_pos = pos_ned + noise_pos + bias;
    data.vel = vel_ned + noise_vel;
    
    // Reproject pos_ned to lat/lon
    double x_rad = noisy_pos[0] / R_EARTH;
    double y_rad = noisy_pos[1] / R_EARTH;
    double c = std::sqrt(x_rad*x_rad + y_rad*y_rad);
    
    double lat = lat_home, lon = lon_home;
    if (c > 0.0) {
        double sin_c = std::sin(c), cos_c = std::cos(c);
        double sin_lat0 = std::sin(lat_home), cos_lat0 = std::cos(lat_home);
        lat = std::asin(cos_c * sin_lat0 + (x_rad * sin_c * cos_lat0) / c);
        lon = lon_home + std::atan2(y_rad * sin_c, c * cos_lat0 * cos_c - x_rad * sin_lat0 * sin_c);
    }
    
    data.lat = lat * 180.0 / M_PI;
    data.lon = lon * 180.0 / M_PI;
    data.alt = alt_home - noisy_pos[2];
}

// --- Magnetometer ---
MagnetometerSensor::MagnetometerSensor() : gen(std::random_device{}()), dist(0.0, 1.0) {}

void MagnetometerSensor::update(double dt, const Eigen::Vector4d& quat, const Eigen::Vector3d& mag_ned) {
    Eigen::Matrix3d Mfg = dynamics::Mfg_from_quat(quat);
    Eigen::Vector3d mag_body = Mfg * mag_ned;
    
    if (enable_noise) {
        Eigen::Vector3d noise(dist(gen), dist(gen), dist(gen));
        mag_meas = mag_body + noise * 0.01;
    } else {
        mag_meas = mag_body;
    }
}

}

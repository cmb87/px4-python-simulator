#pragma once

#include <Eigen/Dense>
#include <random>
#include <map>
#include <string>
#include <cstdint>

namespace sensors {

class SensorBase {
public:
    virtual ~SensorBase() = default;
    virtual void set_noise(bool enabled) = 0;
};

class IMUSensor : public SensorBase {
public:
    IMUSensor();
    void set_noise(bool enabled) override { enable_noise = enabled; }
    void update(double dt, const Eigen::Vector3d& acc_body_rate, const Eigen::Vector3d& vel_body, const Eigen::Vector3d& gyro_body, const Eigen::Vector4d& quat);
    
    Eigen::Vector3d get_accel() const { return acc_meas; }
    Eigen::Vector3d get_gyro() const { return gyro_meas; }

private:
    bool enable_noise = true;
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_meas = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_meas = Eigen::Vector3d::Zero();
    Eigen::Vector3d gravity_vec = Eigen::Vector3d(0, 0, -9.81);
    
    Eigen::Vector3d acc_lpf_state = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_lpf_state = Eigen::Vector3d::Zero();
    bool lpf_initialized = false;

    std::mt19937 gen;
    std::normal_distribution<double> dist;
};

class BarometerSensor : public SensorBase {
public:
    BarometerSensor();
    void set_noise(bool enabled) override { enable_noise = enabled; }
    void set_home_altitude(double alt_m) { home_altitude_amsl = alt_m; }
    void update(double dt, double alt_amsl);
    bool is_updated() const { return updated; }
    
    double get_pressure_hpa() const { return pressure_hpa; }
    double get_altitude() const { return pressure_altitude; }

private:
    bool enable_noise = true;
    bool updated = false;
    uint64_t elapsed_us = 0;
    uint64_t update_interval_us = 50000; // 20 Hz
    double home_altitude_amsl = 0.0;
    double pressure_hpa = 1013.25;
    double pressure_altitude = 0.0;
    double pressure_drift_pa_per_sec = 0.0;
    double pressure_drift_pa = 0.0;
    std::mt19937 gen;
    std::normal_distribution<double> dist;
};

class GPSSensor : public SensorBase {
public:
    GPSSensor();
    void set_noise(bool enabled) override { enable_noise = enabled; }
    void set_home(double lat_deg, double lon_deg, double alt_m);
    void update(double dt, const Eigen::Vector3d& pos_ned, const Eigen::Vector3d& vel_ned);
    bool is_updated() const { return updated; }
    
    struct GpsData {
        double lat = 0.0;
        double lon = 0.0;
        double alt = 0.0;
        Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    };
    GpsData get_data() const { return data; }

private:
    bool enable_noise = true;
    bool updated = false;
    uint64_t elapsed_us = 0;
    uint64_t update_interval_us = 200000; // 5 Hz
    double update_interval = 1.0 / 5.0; // 5 Hz
    double lat_home, lon_home, alt_home;
    GpsData data;
    Eigen::Vector3d bias = Eigen::Vector3d::Zero();
    std::mt19937 gen;
    std::normal_distribution<double> dist;
};

class MagnetometerSensor : public SensorBase {
public:
    MagnetometerSensor();
    void set_noise(bool enabled) override { enable_noise = enabled; }
    void update(double dt, const Eigen::Vector4d& quat, const Eigen::Vector3d& mag_ned);
    
    Eigen::Vector3d get_mag() const { return mag_meas; }

private:
    bool enable_noise = true;
    Eigen::Vector3d mag_meas = Eigen::Vector3d::Zero();
    Eigen::Vector3d bias = Eigen::Vector3d::Zero();
    std::mt19937 gen;
    std::normal_distribution<double> dist;
};

}

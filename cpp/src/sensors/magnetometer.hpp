#pragma once

#include "sensor_base.hpp"
#include "sensor_params.hpp"
#include <random>

namespace sensors {

class MagnetometerSensor : public SensorBase {
public:
    MagnetometerSensor();
    void set_noise(bool enabled) override { enable_noise_ = enabled; }
    void set_update_rate(double hz);
    void set_mag_field_ned(const Eigen::Vector3d& mag_ned);
    void update(double t_us, const Eigen::VectorXd& y);
    
    Eigen::Vector3d get_mag_field_body() const { return mag_meas_; }

private:
    double dt_s_ = 1.0 / MAG_UPDATE_RATE_HZ;
    
    Eigen::Vector3d mag_ned_ = Eigen::Vector3d(0.21523, 0.01, 0.43);
    Eigen::Matrix3d soft_iron_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d hard_iron_ = Eigen::Vector3d::Zero();
    
    Eigen::Vector3d bias_ = Eigen::Vector3d::Zero();
    uint64_t next_update_us_ = 0;
    uint64_t last_t_us_ = 0;
    
    Eigen::Vector3d mag_meas_ = Eigen::Vector3d::Zero();

    std::mt19937 gen_;
    std::normal_distribution<double> dist_;

    Eigen::Vector3d add_noise(Eigen::Vector3d mag_vector, double dt);
};

} // namespace sensors

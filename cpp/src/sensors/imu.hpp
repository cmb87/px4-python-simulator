#pragma once

#include "sensor_base.hpp"
#include "sensor_params.hpp"
#include <random>
#include <cstdint>

namespace sensors {

class IMUSensor : public SensorBase {
public:
    IMUSensor();
    void set_noise(bool enabled) override { enable_noise_ = enabled; }
    void set_gravity(double gravity_mps2);
    
    void update(double t_us, const Eigen::VectorXd& y, const Eigen::VectorXd& ydot);
    
    Eigen::Vector3d get_accel() const { return acc_meas_; }
    Eigen::Vector3d get_gyro() const { return gyro_meas_; }

private:
    Eigen::Vector3d acc_meas_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_meas_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d gravity_vec_ = Eigen::Vector3d(0, 0, -9.81);
    
    double acc_std_ = IMU_ACCEL_STD;
    double gyro_std_ = IMU_GYRO_STD;
    
    uint64_t last_t_us_ = 0;

    std::mt19937 gen_;
    std::normal_distribution<double> dist_;
};

} // namespace sensors

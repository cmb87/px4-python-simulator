#include "imu.hpp"
#include "../dynamics/dynamics.hpp"
#include <cmath>

namespace sensors {

IMUSensor::IMUSensor() : gen_(std::random_device{}()), dist_(0.0, 1.0) {}

void IMUSensor::set_gravity(double gravity_mps2) {
    gravity_vec_ = Eigen::Vector3d(0, 0, -std::abs(gravity_mps2));
}

void IMUSensor::update(double t_us, const Eigen::VectorXd& y, const Eigen::VectorXd& ydot) {
    updated_ = false;
    if (y.size() < 13 || ydot.size() < 10) return;

    double dt = (last_t_us_ == 0) ? DT_DEFAULT : (t_us - last_t_us_) / 1e6;
    if (dt <= 0.0) dt = DT_DEFAULT;
    last_t_us_ = static_cast<uint64_t>(t_us);

    Eigen::Vector4d quat = y.segment<4>(3);
    double q_norm = quat.norm();
    if (q_norm > 1e-9) quat /= q_norm;

    Eigen::Vector3d omega = y.segment<3>(10);
    Eigen::Vector3d vel = y.segment<3>(7);
    Eigen::Vector3d acc_body = ydot.segment<3>(7);

    Eigen::Vector3d acc_body_imu = acc_body + omega.cross(vel);
    Eigen::Matrix3d Mfg = dynamics::Mfg_from_quat(quat);
    Eigen::Vector3d gravity_body = Mfg * gravity_vec_;

    acc_meas_ = acc_body_imu + gravity_body;
    gyro_meas_ = omega;

    if (enable_noise_) {
        acc_meas_ += Eigen::Vector3d(dist_(gen_), dist_(gen_), dist_(gen_)) * acc_std_;
        gyro_meas_ += Eigen::Vector3d(dist_(gen_), dist_(gen_), dist_(gen_)) * gyro_std_;
    }

    updated_ = true;
}

} // namespace sensors

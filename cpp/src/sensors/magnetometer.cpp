#include "magnetometer.hpp"
#include "../dynamics/dynamics.hpp"
#include <cmath>
#include <algorithm>

namespace sensors {

MagnetometerSensor::MagnetometerSensor() : gen_(std::random_device{}()), dist_(0.0, 1.0) {}

void MagnetometerSensor::set_update_rate(double hz) {
    dt_s_ = 1.0 / std::max(hz, 1e-6);
}

void MagnetometerSensor::set_mag_field_ned(const Eigen::Vector3d& mag_ned) {
    mag_ned_ = mag_ned;
}

Eigen::Vector3d MagnetometerSensor::add_noise(Eigen::Vector3d mag_vector, double dt) {
    if (!enable_noise_) {
        return mag_vector;
    }

    double tau = std::max(MAG_BIAS_CORRELATION_TIME, 1e-6);
    double sigma_d = MAG_NOISE_DENSITY / std::sqrt(dt);
    double sigma_b = MAG_RANDOM_WALK;
    double sigma_b_d = std::sqrt(-std::pow(sigma_b, 2) * tau / 2.0 * (std::exp(-2.0 * dt / tau) - 1.0));
    double phi_d = std::exp(-dt / tau);

    for (int i = 0; i < 3; ++i) {
        bias_[i] = phi_d * bias_[i] + sigma_b_d * dist_(gen_);
        mag_vector[i] += bias_[i] + sigma_d * dist_(gen_);
    }

    return mag_vector;
}

void MagnetometerSensor::update(double t_us, const Eigen::VectorXd& y) {
    updated_ = false;
    if (y.size() < 7) return;

    if (last_t_us_ > 0 && t_us < next_update_us_) {
        return;
    }

    double dt = (last_t_us_ == 0) ? dt_s_ : (t_us - last_t_us_) / 1e6;
    if (dt <= 0.0) dt = dt_s_;

    last_t_us_ = static_cast<uint64_t>(t_us);
    next_update_us_ = static_cast<uint64_t>(t_us + dt_s_ * 1e6);

    Eigen::Vector4d quat = y.segment<4>(3);
    double q_norm = quat.norm();
    if (q_norm > 1e-9) quat /= q_norm;

    Eigen::Matrix3d Mfg = dynamics::Mfg_from_quat(quat);
    Eigen::Vector3d mag_body = Mfg * mag_ned_;
    
    mag_body = soft_iron_ * mag_body + hard_iron_;

    mag_meas_ = add_noise(mag_body, dt);

    updated_ = true;
}

} // namespace sensors

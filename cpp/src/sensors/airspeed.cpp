#include "airspeed.hpp"
#include <cmath>
#include <algorithm>

namespace sensors {

AirspeedSensor::AirspeedSensor() : gen_(std::random_device{}()), dist_(0.0, 1.0) {}

void AirspeedSensor::set_update_rate(double hz) {
    dt_s_ = 1.0 / std::max(hz, 1e-6);
}

void AirspeedSensor::update(double t_us, const Eigen::VectorXd& y, const Eigen::Vector3d& wind, double rho, const Eigen::Vector3d& pitot_axis_body) {
    updated_ = false;
    if (y.size() < 10) return;

    if (last_t_us_ > 0 && t_us < next_update_us_) {
        return;
    }

    double dt = (last_t_us_ == 0) ? dt_s_ : (t_us - last_t_us_) / 1e6;
    if (dt <= 0.0) dt = dt_s_;

    last_t_us_ = static_cast<uint64_t>(t_us);
    next_update_us_ = static_cast<uint64_t>(t_us + dt_s_ * 1e6);

    Eigen::Vector3d vel_body = y.segment<3>(7);
    Eigen::Vector3d vel_air_body = vel_body - wind;

    Eigen::Vector3d pitot_axis = pitot_axis_body;
    double pitot_norm = pitot_axis.norm();
    if (pitot_norm <= 1e-9) {
        pitot_axis = Eigen::Vector3d(1.0, 0.0, 0.0);
    } else {
        pitot_axis /= pitot_norm;
    }

    double pitot_speed = vel_air_body.dot(pitot_axis);
    double dynamic_pressure_ideal = 0.5 * rho * std::pow(std::max(pitot_speed, 0.0), 2);

    double alpha_dp = dt / (AIRSPEED_LPF_TAU + dt);

    if (!diff_pressure_initialized_) {
        diff_pressure_pa_ = dynamic_pressure_ideal;
        diff_pressure_initialized_ = true;
    } else {
        diff_pressure_pa_ = diff_pressure_pa_ + alpha_dp * (dynamic_pressure_ideal - diff_pressure_pa_);
    }

    double dynamic_pressure_meas = diff_pressure_pa_ + (enable_noise_ ? dist_(gen_) * AIRSPEED_NOISE_STD : 0.0);
    dynamic_pressure_meas = std::max(dynamic_pressure_meas, 0.0);

    double rho_safe = std::max(rho, 1e-6);
    ias_mps_ = std::sqrt(2.0 * dynamic_pressure_meas / rho_safe);
    tas_mps_ = vel_air_body.norm();
    dynamic_pressure_pa_ = dynamic_pressure_meas;

    updated_ = true;
}

} // namespace sensors

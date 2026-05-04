#include "gps.hpp"
#include "../dynamics/dynamics.hpp"
#include <cmath>
#include <algorithm>

namespace sensors {

GPSSensor::GPSSensor() : gen_(std::random_device{}()), dist_(0.0, 1.0) {}

void GPSSensor::set_home(double lat_deg, double lon_deg, double alt_m) {
    lat_home_rad_ = lat_deg * M_PI / 180.0;
    lon_home_rad_ = lon_deg * M_PI / 180.0;
    alt_home_m_ = alt_m;
}

void GPSSensor::set_update_rate(double hz) {
    dt_s_ = 1.0 / std::max(hz, 1e-6);
}

std::pair<double, double> GPSSensor::reproject(const Eigen::Vector3d& pos_m) {
    double x_rad = pos_m[0] / earth_radius_m_;
    double y_rad = pos_m[1] / earth_radius_m_;
    double c = std::sqrt(x_rad * x_rad + y_rad * y_rad);

    double lat, lon;
    if (c > 0.0) {
        double sin_c = std::sin(c);
        double cos_c = std::cos(c);
        double sin_lat0 = std::sin(lat_home_rad_);
        double cos_lat0 = std::cos(lat_home_rad_);

        lat = std::asin(cos_c * sin_lat0 + (x_rad * sin_c * cos_lat0) / c);
        lon = lon_home_rad_ + std::atan2(y_rad * sin_c, c * cos_lat0 * cos_c - x_rad * sin_lat0 * sin_c);
    } else {
        lat = lat_home_rad_;
        lon = lon_home_rad_;
    }
    return {lat, lon};
}

void GPSSensor::update(double t_us, const Eigen::VectorXd& y) {
    updated_ = false;
    if (y.size() < 10) return;

    if (last_t_us_ > 0 && t_us < next_update_us_) {
        return;
    }

    double dt = (last_t_us_ == 0) ? dt_s_ : (t_us - last_t_us_) / 1e6;
    if (dt <= 0.0) dt = dt_s_;

    last_t_us_ = static_cast<uint64_t>(t_us);
    next_update_us_ = static_cast<uint64_t>(t_us + dt_s_ * 1e6);

    Eigen::Vector3d pos_m = y.segment<3>(0);
    Eigen::Vector3d vel_body = y.segment<3>(7);
    Eigen::Vector4d quat = y.segment<4>(3);
    double q_norm = quat.norm();
    if (q_norm > 1e-9) quat /= q_norm;

    Eigen::Matrix3d Mfg = dynamics::Mfg_from_quat(quat);
    Eigen::Matrix3d Mgf = Mfg.transpose();
    Eigen::Vector3d vel_ned = Mgf * vel_body;

    Eigen::Vector3d noise_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d noise_vel = Eigen::Vector3d::Zero();

    if (enable_noise_) {
        noise_pos <<
            xy_noise_density_ * std::sqrt(dt) * dist_(gen_),
            xy_noise_density_ * std::sqrt(dt) * dist_(gen_),
            z_noise_density_ * std::sqrt(dt) * dist_(gen_);
        
        noise_vel <<
            vxy_noise_density_ * std::sqrt(dt) * dist_(gen_),
            vxy_noise_density_ * std::sqrt(dt) * dist_(gen_),
            vz_noise_density_ * std::sqrt(dt) * dist_(gen_);
    }

    Eigen::Vector3d noisy_pos = pos_m + noise_pos + bias_;
    auto [lat_rad, lon_rad] = reproject(noisy_pos);
    Eigen::Vector3d noisy_vel = vel_ned + noise_vel;

    data_.lat_deg = lat_rad * 180.0 / M_PI;
    data_.lon_deg = lon_rad * 180.0 / M_PI;
    data_.alt_m = alt_home_m_ - noisy_pos[2];
    data_.vel_n = noisy_vel[0];
    data_.vel_e = noisy_vel[1];
    data_.vel_d = noisy_vel[2];
    data_.eph = 1.0;
    data_.epv = 1.0;

    updated_ = true;
}

} // namespace sensors

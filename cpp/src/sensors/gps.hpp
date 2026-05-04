#pragma once

#include "sensor_base.hpp"
#include "sensor_params.hpp"
#include <random>

namespace sensors {

class GPSSensor : public SensorBase {
public:
    GPSSensor();
    void set_noise(bool enabled) override { enable_noise_ = enabled; }
    void set_home(double lat_deg, double lon_deg, double alt_m);
    void set_update_rate(double hz);
    
    void update(double t_us, const Eigen::VectorXd& y);
    
    struct GpsData {
        double lat_deg = 0.0;
        double lon_deg = 0.0;
        double alt_m = 0.0;
        double vel_n = 0.0;
        double vel_e = 0.0;
        double vel_d = 0.0;
        double eph = 1.0;
        double epv = 1.0;
    };
    GpsData get_data() const { return data_; }

private:
    double dt_s_ = 1.0 / GPS_UPDATE_RATE_HZ;
    double lat_home_rad_ = 0.0, lon_home_rad_ = 0.0, alt_home_m_ = 0.0;
    const double earth_radius_m_ = R_EARTH;
    
    double xy_noise_density_ = GPS_XY_NOISE_DENSITY;
    double z_noise_density_ = GPS_Z_NOISE_DENSITY;
    double vxy_noise_density_ = GPS_VXY_NOISE_DENSITY;
    double vz_noise_density_ = GPS_VZ_NOISE_DENSITY;
    double xy_random_walk_ = GPS_XY_RANDOM_WALK;
    double z_random_walk_ = GPS_Z_RANDOM_WALK;

    Eigen::Vector3d bias_ = Eigen::Vector3d::Zero();
    uint64_t next_update_us_ = 0;
    uint64_t last_t_us_ = 0;
    
    GpsData data_;

    std::mt19937 gen_;
    std::normal_distribution<double> dist_;

    std::pair<double, double> reproject(const Eigen::Vector3d& pos_m);
};

} // namespace sensors

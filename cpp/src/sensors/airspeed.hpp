#pragma once

#include "sensor_base.hpp"
#include "sensor_params.hpp"
#include <random>
#include <cstdint>

namespace sensors {

class AirspeedSensor : public SensorBase {
public:
    AirspeedSensor();
    void set_noise(bool enabled) override { enable_noise_ = enabled; }
    void set_update_rate(double hz);
    
    void update(double t_us, const Eigen::VectorXd& y, const Eigen::Vector3d& wind, double rho, const Eigen::Vector3d& pitot_axis_body);
    
    double get_ias_mps() const { return ias_mps_; }
    double get_tas_mps() const { return tas_mps_; }
    double get_dynamic_pressure_pa() const { return dynamic_pressure_pa_; }

private:
    bool diff_pressure_initialized_ = false;
    double diff_pressure_pa_ = 0.0;
    
    double ias_mps_ = 0.0;
    double tas_mps_ = 0.0;
    double dynamic_pressure_pa_ = 0.0;
    
    double dt_s_ = 1.0 / AIRSPEED_UPDATE_RATE_HZ;
    uint64_t next_update_us_ = 0;
    uint64_t last_t_us_ = 0;

    std::mt19937 gen_;
    std::normal_distribution<double> dist_;
};

} // namespace sensors

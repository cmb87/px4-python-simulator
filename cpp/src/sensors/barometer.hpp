#pragma once

#include "sensor_base.hpp"
#include "sensor_params.hpp"
#include <random>

namespace sensors {

class BarometerSensor : public SensorBase {
public:
    BarometerSensor();
    void set_noise(bool enabled) override { enable_noise_ = enabled; }
    void set_update_rate(double hz);
    void set_drift_rate(double drift_pa_per_sec);
    void set_home_altitude(double alt_m);
    void set_gravity(double gravity_mps2);
    
    void update(double dt_us, const Eigen::VectorXd& y);
    
    double get_pressure_hpa() const { return pressure_hpa_; }
    double get_pressure_altitude() const { return pressure_altitude_; }
    double get_temperature_c() const { return temperature_c_; }

private:
    double dt_s_ = 1.0 / BARO_UPDATE_RATE_HZ;
    double home_altitude_amsl_m_ = 0.0;
    double gravity_mps2_ = 9.81;
    
    double pressure_drift_pa_per_sec_ = BARO_DRIFT_PA_S;
    double pressure_drift_pa_ = 0.0;
    double noise_stddev_ = BARO_NOISE_STD;
    
    uint64_t next_update_us_ = 0;
    uint64_t last_t_us_ = 0;

    double pressure_hpa_ = ISA_PRESSURE_MSL_PA * 0.01;
    double pressure_altitude_ = 0.0;
    double temperature_c_ = ISA_TEMPERATURE_MSL_K + ABSOLUTE_ZERO_C;

    std::mt19937 gen_;
    std::normal_distribution<double> dist_;
};

} // namespace sensors

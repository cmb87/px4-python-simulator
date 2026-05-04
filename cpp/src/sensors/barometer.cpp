#include "barometer.hpp"
#include <cmath>
#include <algorithm>

namespace sensors {

BarometerSensor::BarometerSensor() : gen_(std::random_device{}()), dist_(0.0, 1.0) {}

void BarometerSensor::set_update_rate(double hz) {
    dt_s_ = 1.0 / std::max(hz, 1e-6);
}

void BarometerSensor::set_drift_rate(double drift_pa_per_sec) {
    pressure_drift_pa_per_sec_ = drift_pa_per_sec;
}

void BarometerSensor::set_home_altitude(double alt_m) {
    home_altitude_amsl_m_ = alt_m;
}

void BarometerSensor::set_gravity(double gravity_mps2) {
    gravity_mps2_ = gravity_mps2;
}

void BarometerSensor::update(double t_us, const Eigen::VectorXd& y) {
    updated_ = false;
    if (y.size() < 3) return;

    if (last_t_us_ > 0 && t_us < next_update_us_) {
        return;
    }

    double dt = (last_t_us_ == 0) ? dt_s_ : (t_us - last_t_us_) / 1e6;
    if (dt <= 0.0) dt = dt_s_;

    last_t_us_ = static_cast<uint64_t>(t_us);
    next_update_us_ = static_cast<uint64_t>(t_us + dt_s_ * 1e6);

    // y[2] is ze (Down positive), so local altitude is -y[2]
    double z_position_local = -y[2];
    double alt_amsl = home_altitude_amsl_m_ + z_position_local;

    // Temperature at current altitude
    double temperature = ISA_TEMPERATURE_MSL_K - ISA_LAPSE_RATE_K_PER_M * alt_amsl;

    // Ideal pressure at altitude
    double pressure_ratio = std::pow(ISA_TEMPERATURE_MSL_K / temperature, 5.256);
    double absolute_pressure = ISA_PRESSURE_MSL_PA / pressure_ratio;

    // Add noise and drift
    double noise = enable_noise_ ? dist_(gen_) * noise_stddev_ : 0.0;
    pressure_drift_pa_ += pressure_drift_pa_per_sec_ * dt;
    double pressure_noisy = absolute_pressure + noise + pressure_drift_pa_;

    pressure_hpa_ = pressure_noisy * 0.01;

    // Calculate air density at current altitude
    double density_ratio = std::pow(ISA_TEMPERATURE_MSL_K / temperature, 4.256);
    double air_density = ISA_AIR_DENSITY_MSL_KGPM3 / density_ratio;

    // Compute pressure altitude (approximate)
    pressure_altitude_ = alt_amsl - (noise + pressure_drift_pa_) / (gravity_mps2_ * air_density);

    temperature_c_ = temperature + ABSOLUTE_ZERO_C;
    updated_ = true;
}

} // namespace sensors

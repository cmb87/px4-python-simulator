#pragma once

#include "sensor_base.hpp"
#include "imu.hpp"
#include "barometer.hpp"
#include "gps.hpp"
#include "magnetometer.hpp"
#include "airspeed.hpp"

#include <Eigen/Dense>
#include <map>
#include <string>

struct VehicleParameters;

namespace sensors {

class SensorSuite {
public:
    SensorSuite();
    
    void update(double t_us, bool paused, const Eigen::VectorXd& y, const Eigen::VectorXd& ydot, const Eigen::Vector3d& wind, const ::VehicleParameters& P);
    
    struct SensorOutputs {
        Eigen::Vector3d accel;
        Eigen::Vector3d gyro;
        Eigen::Vector3d mag;
        struct Baro {
            double abs_pressure_pa;
            double pressure_alt;
            double temp_c;
        } baro;
        struct Airspeed {
            double ias;
            double tas;
            double dynamic_pressure_pa;
        } airspeed;
        GPSSensor::GpsData gps;
        
        bool imu_updated = false;
        bool mag_updated = false;
        bool baro_updated = false;
        bool airspeed_updated = false;
        bool gps_updated = false;
        
        Eigen::Vector3d euler_deg;
    };

    const SensorOutputs& get_outputs() const { return outputs_; }

    IMUSensor imu;
    BarometerSensor baro;
    GPSSensor gps;
    MagnetometerSensor mag;
    AirspeedSensor airspeed;

private:
    SensorOutputs outputs_;
    bool params_initialized_ = false;
    void initialize_from_parameters(const ::VehicleParameters& P);
};

} // namespace sensors

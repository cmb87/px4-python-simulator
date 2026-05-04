#pragma once

#include "mavlink_interface.hpp"
#include "../sensors/sensors.hpp"
#include <Eigen/Dense>
#include <cstdint>

namespace networking {

class MavlinkSimulator {
public:
    MavlinkSimulator(MavlinkInterface& interface);

    void send_heartbeat();
    void send_system_time(uint64_t sim_time_us);
    void send_hil_sensor(uint64_t sim_time_us, const sensors::SensorSuite::SensorOutputs& sensors, bool has_airspeed_sensor);
    
    // Updated to use ground truth for better estimator stability
    void send_hil_state_quaternion(uint64_t sim_time_us, const Eigen::VectorXd& y, const Eigen::Vector3d& vel_ned, double ias, double tas, double gt_lat, double gt_lon, double gt_alt, const Eigen::Vector3d& acc_body);
    
    void send_hil_gps(uint64_t sim_time_us, const sensors::SensorSuite::SensorOutputs& sensors);

private:
    MavlinkInterface& m_interface;
    uint8_t m_sysid = 42;
    uint8_t m_compid = 51;
};

} // namespace networking

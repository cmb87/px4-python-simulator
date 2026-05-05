#include "mavlink_simulator.hpp"
#include <chrono>
#include <cmath>

namespace networking {

MavlinkSimulator::MavlinkSimulator(MavlinkInterface& interface) : m_interface(interface) {}

void MavlinkSimulator::send_heartbeat() {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(m_interface.get_source_system(), m_compid, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
    m_interface.send_message(msg);
}

void MavlinkSimulator::send_system_time(uint64_t sim_time_us) {
    uint64_t unix_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    mavlink_message_t msg;
    mavlink_msg_system_time_pack(m_interface.get_source_system(), m_compid, &msg, unix_time, sim_time_us / 1000);
    m_interface.send_message(msg);
}

void MavlinkSimulator::send_hil_sensor(uint64_t sim_time_us, const sensors::SensorSuite::SensorOutputs& sensors, bool has_airspeed_sensor) {
    uint32_t fields_updated = 0;
    if (sensors.imu_updated) fields_updated |= 0x3F;
    if (sensors.mag_updated) fields_updated |= 0x1C0;
    
    // Correct bitmask for Barometer (Bits 9, 11, 12)
    if (sensors.baro_updated) fields_updated |= (1u << 9) | (1u << 11) | (1u << 12);
    
    if (has_airspeed_sensor && sensors.airspeed_updated) fields_updated |= (1u << 10);

    if (fields_updated == 0) return;

    // MATCH PYTHON EXACTLY: Calculate pressure altitude from static pressure using the exact formula
    // Python uses staticAbsolute which is in Pa
    double static_pressure_pa = sensors.baro.abs_pressure_pa;
    double pressure_alt = 44330.0 * (1.0 - std::pow(static_pressure_pa / 101325.0, 1.0 / 5.25588));

    mavlink_message_t msg;
    mavlink_msg_hil_sensor_pack(m_interface.get_source_system(), m_compid, &msg,
        sim_time_us,
        sensors.accel[0], sensors.accel[1], sensors.accel[2],
        sensors.gyro[0], sensors.gyro[1], sensors.gyro[2],
        sensors.mag[0], sensors.mag[1], sensors.mag[2],
        static_pressure_pa * 0.01, // static_pressure (hPa)
        sensors.airspeed.dynamic_pressure_pa * 0.01, // diff_pressure (hPa)
        pressure_alt, // pressure_alt (m)
        15.0, // temperature
        fields_updated,
        0
    );
    m_interface.send_message(msg);

    // Debug print
    printf("HIL_SENSOR: t=%lu accX=%.2f accZ=%.2f baro=%.2f altGps=%.2f velDGps=%.0f, fields_updated=%u\n",
        sim_time_us, sensors.accel[0], sensors.accel[2], 
        pressure_alt - 447.0, sensors.gps.alt_m - 447.0, 
        sensors.gps.vel_d, fields_updated);
}

void MavlinkSimulator::send_hil_state_quaternion(uint64_t sim_time_us, const Eigen::VectorXd& y, const Eigen::Vector3d& vel_ned, double ias, double tas, double gt_lat, double gt_lon, double gt_alt, const Eigen::Vector3d& acc_meas) {
    if (y.size() < 13) return;

    float q[4] = {(float)y[3], (float)y[4], (float)y[5], (float)y[6]};
    const double m_s2_to_mg = 1000.0 / 9.80665;
    
    // Airspeed field in state message is Horizontal Ground Speed (Python match)
    float horiz_speed_m_s = std::sqrt(vel_ned[0] * vel_ned[0] + vel_ned[1] * vel_ned[1]);

    mavlink_message_t msg;
    mavlink_msg_hil_state_quaternion_pack(m_interface.get_source_system(), m_compid, &msg,
        sim_time_us, q, y[10], y[11], y[12],
        (int32_t)(gt_lat * 1e7), (int32_t)(gt_lon * 1e7), (int32_t)(gt_alt * 1000),
        (int16_t)(vel_ned[0] * 100), (int16_t)(vel_ned[1] * 100), (int16_t)(vel_ned[2] * 100),
        (uint16_t)(horiz_speed_m_s * 100), (uint16_t)(horiz_speed_m_s * 100),
        (int16_t)(acc_meas[0] * m_s2_to_mg), (int16_t)(acc_meas[1] * m_s2_to_mg), (int16_t)(acc_meas[2] * m_s2_to_mg)
    );
    m_interface.send_message(msg);
}

void MavlinkSimulator::send_hil_gps(uint64_t sim_time_us, const sensors::SensorSuite::SensorOutputs& sensors) {
    if (!sensors.gps_updated) return;

    double cog_deg = std::atan2(sensors.gps.vel_e, sensors.gps.vel_n) * 180.0 / M_PI;
    if (cog_deg < 0) cog_deg += 360.0;
    
    const double vel_horiz = std::sqrt(sensors.gps.vel_n * sensors.gps.vel_n + sensors.gps.vel_e * sensors.gps.vel_e);

    mavlink_message_t msg;
    mavlink_msg_hil_gps_pack(m_interface.get_source_system(), m_compid, &msg,
        sim_time_us, 3, 
        (int32_t)(sensors.gps.lat_deg * 1e7), (int32_t)(sensors.gps.lon_deg * 1e7), (int32_t)(sensors.gps.alt_m * 1000),
        100, 100, (uint16_t)(vel_horiz * 100),
        (int16_t)(sensors.gps.vel_n * 100), (int16_t)(sensors.gps.vel_e * 100), (int16_t)(sensors.gps.vel_d * 100),
        (uint16_t)(cog_deg * 100), 10, 0, 0
    );
    m_interface.send_message(msg);

    printf("HIL_GPS: t=%lu lat=%.7f lon=%.7f alt=%.2f vel=%.2f\n",
        sim_time_us, sensors.gps.lat_deg, sensors.gps.lon_deg, sensors.gps.alt_m, vel_horiz);
}

} // namespace networking

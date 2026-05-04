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
    // Bits matching Python exactly:
    // 1-3: Acc, 4-6: Gyro, 7-9: Mag, 10: Baro Static, 11: Diff Press, 12: Baro Alt, 13: Baro Temp
    uint32_t fields_updated = 0;
    
    // IMU (Bits 0-5) always update at 125Hz
    if (sensors.imu_updated) {
        fields_updated |= 0x3F; // 0b111111 (Acc + Gyro)
    }

    // Mag (Bits 6-8) update at 62.5Hz
    if (sensors.mag_updated) {
        fields_updated |= 0x1C0; // 0b111000000
    }

    // Baro (Bits 9, 11, 12) update at 20Hz
    if (sensors.baro_updated) {
        fields_updated |= (1u << 9) | (1u << 11) | (1u << 12);
    }

    // Airspeed (Bit 10) updates at 125Hz
    if (has_airspeed_sensor && sensors.airspeed_updated) {
        fields_updated |= (1u << 10);
    }

    if (fields_updated == 0) return;

    mavlink_message_t msg;
    mavlink_msg_hil_sensor_pack(m_interface.get_source_system(), m_compid, &msg,
        sim_time_us,
        sensors.accel[0], sensors.accel[1], sensors.accel[2],
        sensors.gyro[0], sensors.gyro[1], sensors.gyro[2],
        sensors.mag[0], sensors.mag[1], sensors.mag[2],
        sensors.baro.abs_pressure_pa * 0.01, // hPa
        sensors.airspeed.dynamic_pressure_pa * 0.01, // hPa
        sensors.baro.pressure_alt,
        sensors.baro.temp_c,
        fields_updated,
        0
    );
    m_interface.send_message(msg);

    // Debug print matching Python log format
    printf("HIL_SENSOR: t=%lu accX=%.2f accZ=%.2f baro=%.2f altGps=%.2f velDGps=%.0f, fields_updated=%u\n",
        sim_time_us, sensors.accel[0], sensors.accel[2], 
        sensors.baro.pressure_alt - 447.0, sensors.gps.alt_m - 447.0, 
        sensors.gps.vel_d, fields_updated);
}

void MavlinkSimulator::send_hil_state_quaternion(uint64_t sim_time_us, const Eigen::VectorXd& y, const Eigen::Vector3d& vel_ned, double ias, double tas, double gt_lat, double gt_lon, double gt_alt, const Eigen::Vector3d& acc_body) {
    if (y.size() < 13) return;

    float q[4] = {(float)y[3], (float)y[4], (float)y[5], (float)y[6]};
    const double m_s2_to_mg = 1000.0 / 9.80665;

    mavlink_message_t msg;
    mavlink_msg_hil_state_quaternion_pack(m_interface.get_source_system(), m_compid, &msg,
        sim_time_us, q, y[10], y[11], y[12],
        (int32_t)(gt_lat * 1e7), (int32_t)(gt_lon * 1e7), (int32_t)(gt_alt * 1000),
        (int16_t)(vel_ned[0] * 100), (int16_t)(vel_ned[1] * 100), (int16_t)(vel_ned[2] * 100),
        (uint16_t)(ias * 100), (uint16_t)(tas * 100),
        (int16_t)(acc_body[0] * m_s2_to_mg), (int16_t)(acc_body[1] * m_s2_to_mg), (int16_t)(acc_body[2] * m_s2_to_mg)
    );
    m_interface.send_message(msg);
}

void MavlinkSimulator::send_hil_gps(uint64_t sim_time_us, const sensors::SensorSuite::SensorOutputs& sensors) {
    if (!sensors.gps_updated) return;

    double cog_deg = std::atan2(sensors.gps.vel_e, sensors.gps.vel_n) * 180.0 / M_PI;
    if (cog_deg < 0) cog_deg += 360.0;
    const double gps_speed_3d = std::sqrt(sensors.gps.vel_n * sensors.gps.vel_n + sensors.gps.vel_e * sensors.gps.vel_e + sensors.gps.vel_d * sensors.gps.vel_d);

    mavlink_message_t msg;
    mavlink_msg_hil_gps_pack(m_interface.get_source_system(), m_compid, &msg,
        sim_time_us, 3, 
        (int32_t)(sensors.gps.lat_deg * 1e7), (int32_t)(sensors.gps.lon_deg * 1e7), (int32_t)(sensors.gps.alt_m * 1000),
        100, 100, (uint16_t)(gps_speed_3d * 100),
        (int16_t)(sensors.gps.vel_n * 100), (int16_t)(sensors.gps.vel_e * 100), (int16_t)(sensors.gps.vel_d * 100),
        (uint16_t)(cog_deg * 100), 10, 0, 0
    );
    m_interface.send_message(msg);

    printf("HIL_GPS: t=%lu lat=%.7f lon=%.7f alt=%.2f vel=%.2f\n",
        sim_time_us, sensors.gps.lat_deg, sensors.gps.lon_deg, sensors.gps.alt_m, gps_speed_3d);
}

} // namespace networking

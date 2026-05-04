import logging
import time
import numpy as np
from pymavlink import mavutil

logger = logging.getLogger(__name__)

# HIL_SENSOR updated fields bitmask
HIL_SENSOR_ACCEL_UPDATED = 0x07
HIL_SENSOR_GYRO_UPDATED = 0x38
HIL_SENSOR_MAG_UPDATED = 0x1C0
HIL_SENSOR_BARO_STATIC_UPDATED = 0x200
HIL_SENSOR_DIFF_PRESSURE_UPDATED = 0x400
HIL_SENSOR_BARO_ALT_UPDATED = 0x800
HIL_SENSOR_BARO_TEMP_UPDATED = 0x1000

class MavlinkSimulator:
    def __init__(self, conn):
        self.conn = conn

    def send_heartbeat(self):
        self.conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GENERIC,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )

    def send_system_time(self, sim_time_us):
        self.conn.mav.system_time_send(int(time.time() * 1_000_000), int(sim_time_us / 1000))

    def send_hil_sensor(self, sim_time_us, sensors, has_airspeed_sensor=True):
        acc = np.asarray(sensors["accelerometer"], dtype=float)
        gyro = np.asarray(sensors["gyroscope"], dtype=float)
        mag = np.asarray(sensors["magnetometer"], dtype=float)
        baro = sensors["barometer"]

        fields_updated = 0
        if sensors.get("imu_updated", True):
            fields_updated |= HIL_SENSOR_GYRO_UPDATED | HIL_SENSOR_ACCEL_UPDATED
        
        if sensors.get("mag_updated", True):
            fields_updated |= HIL_SENSOR_MAG_UPDATED
            
        if sensors.get("baro_updated", True):
            fields_updated |= HIL_SENSOR_BARO_STATIC_UPDATED | HIL_SENSOR_BARO_ALT_UPDATED | HIL_SENSOR_BARO_TEMP_UPDATED
            
        if has_airspeed_sensor and sensors.get("diff_press_updated", True):
            fields_updated |= HIL_SENSOR_DIFF_PRESSURE_UPDATED

        # Calculate pressure altitude from absolute static pressure (Pa)
        # Standard Atmosphere formula: h = 44330 * (1 - (P/P0)^(1/5.255))
        static_pressure_pa = float(baro["staticAbsolute"])
        pressure_alt = 44330.0 * (1.0 - (static_pressure_pa / 101325.0)**(1.0 / 5.25588))


        gps = np.asarray(sensors["gps"], dtype=float)

        vel_down = float(-gps[5])
        
        # print(f"HIL_SENSOR: t={sim_time_us} accX={acc[0]:.2f} accZ={acc[2]:.2f} baro={pressure_alt-447.0:.2f} altGps={float(gps[2])-447.0:.2f} velDGps={int(round(vel_down ))}, fields_updated={fields_updated}")

        self.conn.mav.hil_sensor_send(
            int(sim_time_us),
            float(acc[0]),
            float(acc[1]),
            float(acc[2]),
            float(gyro[0]),
            float(gyro[1]),
            float(gyro[2]),
            float(mag[0]),
            float(mag[1]),
            float(mag[2]),
            float(baro["staticAbsolute"]) * 0.01,
            float(baro["dynamic"]) * 0.01,
            float(pressure_alt),
            15.0,
            int(fields_updated),
        )

    def send_hil_state_quaternion(self, sim_time_us, y, sensors):
        gps = np.asarray(sensors["gps"], dtype=float)
        acc = np.asarray(sensors["accelerometer"], dtype=float)
        
        vel_north = float(gps[3])
        vel_east = float(gps[4])
        vel_down = float(-gps[5])
        horiz_speed_m_s = float(np.hypot(vel_north, vel_east))
        m_s2_to_mg = 1000.0 / 9.80665
        
        self.conn.mav.hil_state_quaternion_send(
            int(sim_time_us),
            [float(v) for v in y[3:7]],
            float(y[10]),
            float(y[11]),
            float(y[12]),
            int(round(float(gps[0]) * 1e7)),
            int(round(float(gps[1]) * 1e7)),
            int(round(float(gps[2]) * 1000.0)),
            int(round(vel_north * 100.0)),
            int(round(vel_east * 100.0)),
            int(round(vel_down * 100.0)),
            int(round(horiz_speed_m_s * 100.0)),
            int(round(horiz_speed_m_s * 100.0)),
            int(round(float(acc[0]) * m_s2_to_mg)),
            int(round(float(acc[1]) * m_s2_to_mg)),
            int(round(float(acc[2]) * m_s2_to_mg)),
        )

    def send_hil_gps(self, sim_time_us, sensors):
        gps = np.asarray(sensors["gps"], dtype=float)
        vel_north = float(gps[3])
        vel_east = float(gps[4])
        vel_down = float(-gps[5])
        vel_3d = float(np.linalg.norm(np.array([vel_north, vel_east])))
        cog_rad = float(np.arctan2(vel_east, vel_north))
        if cog_rad < 0.0:
            cog_rad += 2.0 * np.pi

        self.conn.mav.hil_gps_send(
            int(sim_time_us),
            3,
            int(round(float(gps[0]) * 1e7)),
            int(round(float(gps[1]) * 1e7)),
            int(round(float(gps[2]) * 1000.0)),
            100,
            100,
            int(round(vel_3d * 100.0)),
            int(round(vel_north * 100.0)),
            int(round(vel_east * 100.0)),
            int(round(vel_down * 100.0)),
            int(round(np.degrees(cog_rad) * 100.0)),
            10,
            0,
            0,
        )

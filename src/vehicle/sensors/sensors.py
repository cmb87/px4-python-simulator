import numpy as np
from quaternion import Quaternion

try:
    from sensors.magnetometer import MagnetometerSim
    from sensors.imu import ADIS16448IMU
    from sensors.gps import GpsSensor
    from sensors.barometer import BarometerSensor
except ImportError:
    from magnetometer import MagnetometerSim
    from imu import ADIS16448IMU
    from gps import GpsSensor
    from barometer import BarometerSensor

from base_component import SimComponentBase


class SensorSuite(SimComponentBase):
    def __init__(self):
        super().__init__()
        self.mag = MagnetometerSim()
        self.mag.set_noise(True)
        self.imu = ADIS16448IMU()
        self.imu.set_noise(True)

        self._sensor_params_initialized = False

        self.gps = GpsSensor()
        self.gps.set_home(47.397742, 8.545594, 488.0)
        self.gps.set_noise(True)
        self.gps.set_update_rate(5.0)

        self.baro = BarometerSensor()
        self.baro.set_update_rate(20.0)
        self.baro.set_drift_rate(0.05)
        self.baro.set_noise(True)

    def update(self, t_us, paused):
        if paused:
            return self.last_output

        y = self._inputs.get("y")
        ydot = self._inputs.get("ydot")
        P = self._inputs.get("P")

        if y is None or ydot is None or P is None:
            raise ValueError("SensorSuite requires inputs: y, ydot, P")

        if not self._sensor_params_initialized:
            self.imu.set_biases(accel_bias_mps2=P.accel_bias, gyro_bias_rps=P.gyro_bias)
            self.mag.set_hard_iron(P.mag_bias)
            gps_origin = getattr(P, "gps_origin", {})
            lat_deg = float(gps_origin.get("lat", 47.397742))
            lon_deg = float(gps_origin.get("lon", 8.545594))
            alt_m = float(gps_origin.get("alt", 470.0))
            self.gps.set_home(lat_deg, lon_deg, alt_m)
            self._sensor_params_initialized = True

        pos = y[0:3]
        quat = y[3:7] / np.linalg.norm(y[3:7])
        vel = y[7:10]
        omega = y[10:13]
        accel_body_rate = ydot[7:10]

        Mfg = Quaternion.Mfg(quat)
        Mgf = Mfg.T

        euler = np.rad2deg(Quaternion.quat2Euler(quat))
        vel_ned = Mgf @ vel

        # Match jMAVSim sensor path: accelerometer should see specific force from
        # inertial acceleration projected to body. dynamics() provides body-velocity
        # derivative (u_dot, v_dot, w_dot), which includes -omega x v. Add omega x v
        # back to recover force/mass term for IMU modeling.
        accel_for_imu = np.asarray(accel_body_rate, dtype=float) + np.cross(np.asarray(omega, dtype=float), np.asarray(vel, dtype=float))

        self.imu.set_inputs(acc_body=accel_for_imu, ang_vel_body=omega, orientation_quat=quat)
        acc_meas, gyro_meas = self.imu.update(t_us, paused=False)

        self.mag.set_inputs(orientation_quat=quat, mag_field_ned=P.magnetic_ned)
        mag_meas = self.mag.update(t_us, paused=False)["mag_field_body_gauss"]

        self.baro.set_inputs(z_position_local=-pos[2])
        baro_reading = self.baro.update(t_us, paused=False)
        static_pressure = baro_reading["absolute_pressure_hpa"] * 100.0
        dynamic_pressure = 0.5 * P.rho * np.linalg.norm(vel_ned) ** 2
        baro_meas = {
            "staticAbsolute": static_pressure,
            "static": static_pressure,
            "dynamic": dynamic_pressure + np.random.normal(0, P.baro_noise_std),
            "pressure_altitude_m": float(baro_reading["pressure_altitude_m"]),
        }

        self.gps.set_inputs(position_m=pos, velocity_mps=vel_ned)
        data = self.gps.update(t_us, paused=False)
        gps_meas = np.array([
            data["latitude_deg"],
            data["longitude_deg"],
            data["altitude_m"],
            data["velocity_north"],
            data["velocity_east"],
            data["velocity_up"],
            0.0,
        ])

        self.last_output = {
            "accelerometer": acc_meas,
            "gyroscope": gyro_meas,
            "magnetometer": mag_meas,
            "barometer": baro_meas,
            "gps": gps_meas,
            "gps_updated": self.gps.is_updated(),
            "euler": euler,
        }
        self._last_t_us = int(t_us)
        return self.last_output


_DEFAULT_SENSOR_SUITE = SensorSuite()


def sensors(t, y, ydot, u, wind, P, dt):
    _DEFAULT_SENSOR_SUITE.set_inputs(y=y, ydot=ydot, u=u, wind=wind, P=P)
    return _DEFAULT_SENSOR_SUITE.update(int(float(t) * 1e6), paused=False)

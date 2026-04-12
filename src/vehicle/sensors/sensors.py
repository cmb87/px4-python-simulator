import numpy as np

from ..base_component import SimComponentBase
from ..quaternion import Quaternion
from .barometer import BarometerSensor
from .gps import GpsSensor
from .imu import ADIS16448IMU
from .magnetometer import MagnetometerSim


class SensorSuite(SimComponentBase):
    def __init__(self):
        super().__init__()
        self.mag = MagnetometerSim()
        self.mag.set_noise(True)
        self.imu = ADIS16448IMU()
        self.imu.set_noise(True)

        self._sensor_params_initialized = False

        self.gps = GpsSensor()
        self.gps.set_home(48.35386539065191, 11.78159133408772, 447.0)
        self.gps.set_noise(True)
        self.gps.set_update_rate(5.0)

        self.baro = BarometerSensor()
        self.baro.set_update_rate(20.0)
        self.baro.set_drift_rate(0.05)
        self.baro.set_noise(True)

        self._diff_pressure_initialized = False
        self._diff_pressure_pa = 0.0

    def update(self, t_us, paused):
        if paused:
            return self.last_output

        y = self._inputs.get("y")
        ydot = self._inputs.get("ydot")
        wind = self._inputs.get("wind")
        P = self._inputs.get("P")

        if y is None or ydot is None or P is None:
            raise ValueError("SensorSuite requires inputs: y, ydot, P")

        if not self._sensor_params_initialized:
            self.imu.set_biases(accel_bias_mps2=P.accel_bias, gyro_bias_rps=P.gyro_bias)
            self.mag.set_hard_iron(P.mag_bias)
            gps_origin = getattr(P, "gps_origin", {})
            lat_deg = float(gps_origin.get("lat", 48.35386539065191))
            lon_deg = float(gps_origin.get("lon", 11.78159133408772))
            alt_m = float(gps_origin.get("alt", 447.0))
            self.gps.set_home(lat_deg, lon_deg, alt_m)
            self._sensor_params_initialized = True
            self._diff_pressure_initialized = False
            self._diff_pressure_pa = 0.0

        pos = y[0:3]
        quat = y[3:7] / np.linalg.norm(y[3:7])
        vel = y[7:10]
        wind_vec = np.zeros(6) if wind is None else np.asarray(wind, dtype=float)
        wind_body = wind_vec[:3]
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

        vel_air_body = np.asarray(vel, dtype=float) - np.asarray(wind_body, dtype=float)

        pitot_axis_body = np.asarray(getattr(P, "pitot_axis_body", np.array([1.0, 0.0, 0.0])), dtype=float).reshape(3)
        pitot_axis_norm = float(np.linalg.norm(pitot_axis_body))
        if pitot_axis_norm <= 1e-9:
            pitot_axis_body = np.array([1.0, 0.0, 0.0], dtype=float)
        else:
            pitot_axis_body = pitot_axis_body / pitot_axis_norm

        pitot_speed = float(np.dot(vel_air_body, pitot_axis_body))
        dynamic_pressure_ideal = 0.5 * float(P.rho) * max(pitot_speed, 0.0) ** 2

        dt = self._compute_dt_s(t_us)
        if dt <= 0.0:
            dt = 1.0 / 250.0
        diff_pressure_lpf_tau_s = max(float(getattr(P, "diff_pressure_lpf_tau_s", 0.08)), 1e-6)
        alpha_dp = dt / (diff_pressure_lpf_tau_s + dt)

        if not self._diff_pressure_initialized:
            self._diff_pressure_pa = dynamic_pressure_ideal
            self._diff_pressure_initialized = True
        else:
            self._diff_pressure_pa = self._diff_pressure_pa + alpha_dp * (dynamic_pressure_ideal - self._diff_pressure_pa)

        diff_pressure_noise_std = float(getattr(P, "diff_pressure_noise_std", P.baro_noise_std))
        dynamic_pressure_meas = self._diff_pressure_pa + np.random.normal(0.0, diff_pressure_noise_std)
        dynamic_pressure_meas = max(float(dynamic_pressure_meas), 0.0)
        rho_air = max(float(P.rho), 1e-6)
        airspeed_ias_mps = float(np.sqrt(2.0 * dynamic_pressure_meas / rho_air))
        airspeed_tas_mps = float(np.linalg.norm(vel_air_body))
        baro_meas = {
            "staticAbsolute": static_pressure,
            "static": static_pressure,
            "dynamic": dynamic_pressure_meas,
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
            "airspeed_ias_mps": airspeed_ias_mps,
            "airspeed_tas_mps": airspeed_tas_mps,
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

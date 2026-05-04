import logging
import numpy as np

from vehicles.base_component import SimComponentBase
from dynamics.quaternion import Quaternion


logger = logging.getLogger(__name__)


class ADIS16448IMU(SimComponentBase):
    def __init__(self):
        super().__init__()
        self.dt = 1.0 / 250.0

        # Gyroscope parameters
        self.gyro_noise_density = 2.0 * 35.0 / 3600.0 / 180.0 * np.pi
        self.gyro_random_walk = 2.0 * 4.0 / 3600.0 / 180.0 * np.pi
        self.gyro_bias_correlation_time = 1.0e3
        self.gyro_turn_on_bias_sigma = 0.5 / 180.0 * np.pi

        # Accelerometer parameters
        self.acc_noise_density = 2.0 * 2.0e-3
        self.acc_random_walk = 2.0 * 3.0e-3
        self.acc_bias_correlation_time = 300.0
        self.acc_turn_on_bias_sigma = 20.0e-3 * 9.8

        self.acc_lpf_hz = 120.0
        self.gyro_lpf_hz = 120.0
        self.acc_range_mps2 = 18.0 * 9.80665
        self.gyro_range_rps = np.deg2rad(2000.0)
        self.enable_noise = True

        # Gravity vector (world frame)
        self.gravity_vector = np.array([0.0, 0.0, -9.81], dtype=float)

        # Bias initialization
        self.gyro_bias = np.array([0.0, 0.0, 0.0], dtype=float)
        self.acc_bias = np.array([0.0, 0.0, 0.0], dtype=float)

        # Random generator
        self.rng = np.random.default_rng()

        self._acc_lpf_state = np.zeros(3)
        self._gyro_lpf_state = np.zeros(3)
        self._acc_lpf_initialized = False
        self._gyro_lpf_initialized = False
        self._acc_saturation_latched = False
        self._gyro_saturation_latched = False
        self.updated = False

    def set_noise(self, enabled: bool):
        self.enable_noise = bool(enabled)

    def set_gravity(self, gravity_mps2: float):
        self.gravity_vector = np.array([0.0, 0.0, -float(gravity_mps2)], dtype=float)

    def set_biases(self, accel_bias_mps2=None, gyro_bias_rps=None):
        if accel_bias_mps2 is not None:
            self.acc_bias = np.asarray(accel_bias_mps2, dtype=float).reshape(3)
        if gyro_bias_rps is not None:
            self.gyro_bias = np.asarray(gyro_bias_rps, dtype=float).reshape(3)
        

    def update(self, t_us, paused):
        if paused:
            return self.last_output

        y = self._inputs.get("y")
        ydot = self._inputs.get("ydot")

        if y is not None and ydot is not None:
            orientation_quat = y[3:7] / np.linalg.norm(y[3:7])
            ang_vel_body = y[10:13]
            vel_body = y[7:10]
            accel_body_rate = ydot[7:10]
            # specific force = accel_body_rate + omega x vel_body
            acc_body = np.asarray(accel_body_rate, dtype=float) + np.cross(np.asarray(ang_vel_body, dtype=float), np.asarray(vel_body, dtype=float))
        else:
            acc_body = self._inputs.get("acc_body")
            ang_vel_body = self._inputs.get("ang_vel_body")
            orientation_quat = self._inputs.get("orientation_quat")

        if acc_body is None or ang_vel_body is None or orientation_quat is None:
            raise ValueError("ADIS16448IMU requires inputs: y/ydot or acc_body/ang_vel_body/orientation_quat")

        dt = self._compute_dt_s(t_us)
        if dt <= 0.0:
            dt = self.dt

        R_wb = Quaternion.Mfg(orientation_quat)

        gravity_body = R_wb @ self.gravity_vector

        # Specific force in body frame (accelerometer model)
        specific_force_body = np.asarray(acc_body, dtype=float) + gravity_body
        gyro_true = np.asarray(ang_vel_body, dtype=float)

        acc_filtered = self._apply_lpf(specific_force_body, dt, self.acc_lpf_hz, "acc")
        gyro_filtered = self._apply_lpf(gyro_true, dt, self.gyro_lpf_hz, "gyro")

        if self.enable_noise:
            acc_meas = self._add_acc_noise(acc_filtered, dt)
            gyro_meas = self._add_gyro_noise(gyro_filtered, dt)
        else:
            acc_meas = acc_filtered + self.acc_bias
            gyro_meas = gyro_filtered + self.gyro_bias

        acc_raw = np.asarray(acc_meas, dtype=float)
        gyro_raw = np.asarray(gyro_meas, dtype=float)
        acc_meas = np.clip(acc_raw, -self.acc_range_mps2, self.acc_range_mps2)
        gyro_meas = np.clip(gyro_raw, -self.gyro_range_rps, self.gyro_range_rps)

        acc_is_clipped = bool(np.any(np.abs(acc_raw - acc_meas) > 1e-12))
        gyro_is_clipped = bool(np.any(np.abs(gyro_raw - gyro_meas) > 1e-12))

        if acc_is_clipped and (not self._acc_saturation_latched):
            self._acc_saturation_latched = True
            logger.info("IMU accelerometer saturation")
        elif (not acc_is_clipped) and self._acc_saturation_latched:
            self._acc_saturation_latched = False

        if gyro_is_clipped and (not self._gyro_saturation_latched):
            self._gyro_saturation_latched = True
            logger.info("IMU gyroscope saturation")
        elif (not gyro_is_clipped) and self._gyro_saturation_latched:
            self._gyro_saturation_latched = False

        self.last_output = (acc_meas, gyro_meas)
        self.updated = True
        return self.last_output

    def is_updated(self) -> bool:
        return self.updated

    def _apply_lpf(self, value, dt, cutoff_hz, channel):
        tau = 1.0 / (2.0 * np.pi * max(cutoff_hz, 1e-6))
        alpha = dt / (tau + dt)

        if channel == "acc":
            if not self._acc_lpf_initialized:
                self._acc_lpf_state = np.asarray(value, dtype=float).copy()
                self._acc_lpf_initialized = True
                return self._acc_lpf_state.copy()
            self._acc_lpf_state = self._acc_lpf_state + alpha * (np.asarray(value, dtype=float) - self._acc_lpf_state)
            return self._acc_lpf_state.copy()

        if not self._gyro_lpf_initialized:
            self._gyro_lpf_state = np.asarray(value, dtype=float).copy()
            self._gyro_lpf_initialized = True
            return self._gyro_lpf_state.copy()
        self._gyro_lpf_state = self._gyro_lpf_state + alpha * (np.asarray(value, dtype=float) - self._gyro_lpf_state)
        return self._gyro_lpf_state.copy()


    def _add_gyro_noise(self, omega, dt):
        phi = np.exp(-dt / self.gyro_bias_correlation_time)
        sigma_b = np.sqrt(
            -self.gyro_random_walk**2 * self.gyro_bias_correlation_time / 2.0 *
            (np.exp(-2.0 * dt / self.gyro_bias_correlation_time) - 1.0)
        )
        sigma_d = self.gyro_noise_density / np.sqrt(dt)

        self.gyro_bias = phi * self.gyro_bias + sigma_b * self.rng.normal(0.0, 1.0, 3)
        noise = sigma_d * self.rng.normal(0, 1, 3)
        return omega + self.gyro_bias + noise

    def _add_acc_noise(self, acc, dt):
        phi = np.exp(-dt / self.acc_bias_correlation_time)
        sigma_b = np.sqrt(
            -self.acc_random_walk**2 * self.acc_bias_correlation_time / 2.0 *
            (np.exp(-2.0 * dt / self.acc_bias_correlation_time) - 1.0)
        )
        sigma_d = self.acc_noise_density / np.sqrt(dt)

        self.acc_bias = phi * self.acc_bias + sigma_b * self.rng.normal(0.0, 1.0, 3)
        noise = sigma_d * self.rng.normal(0, 1, 3)
        return acc + self.acc_bias + noise


class SimpleIMU(SimComponentBase):
    def __init__(self, acc_std=0.0001, gyro_std=0.00001):
        super().__init__()
        self.acc_std = acc_std
        self.gyro_std = gyro_std
        self.enable_noise = True
        self.gravity_vector = np.array([0.0, 0.0, -9.81], dtype=float)
        self.rng = np.random.default_rng()
        self.updated = False

    def set_noise(self, enabled: bool):
        self.enable_noise = bool(enabled)

    def set_gravity(self, gravity_mps2: float):
        self.gravity_vector = np.array([0.0, 0.0, -float(gravity_mps2)], dtype=float)

    def update(self, t_us, paused):
        if paused:
            return self.last_output

        y = self._inputs.get("y")
        ydot = self._inputs.get("ydot")

        if y is not None and ydot is not None:
            orientation_quat = y[3:7] / np.linalg.norm(y[3:7])
            ang_vel_body = y[10:13]
            vel_body = y[7:10]
            accel_body_rate = ydot[7:10]
            acc_body = np.asarray(accel_body_rate, dtype=float) + np.cross(np.asarray(ang_vel_body, dtype=float), np.asarray(vel_body, dtype=float))
        else:
            acc_body = self._inputs.get("acc_body")
            ang_vel_body = self._inputs.get("ang_vel_body")
            orientation_quat = self._inputs.get("orientation_quat")

        if acc_body is None or ang_vel_body is None or orientation_quat is None:
            raise ValueError("SimpleIMU requires inputs: y/ydot or acc_body/ang_vel_body/orientation_quat")

        R_wb = Quaternion.Mfg(orientation_quat)
        gravity_body = R_wb @ self.gravity_vector

        acc_meas = np.asarray(acc_body, dtype=float) + gravity_body
        gyro_meas = np.asarray(ang_vel_body, dtype=float)

        if self.enable_noise:
            acc_meas += self.rng.normal(0, self.acc_std, 3)
            gyro_meas += self.rng.normal(0, self.gyro_std, 3)

        self.last_output = (acc_meas, gyro_meas)
        self.updated = True
        return self.last_output

    def is_updated(self) -> bool:
        return self.updated

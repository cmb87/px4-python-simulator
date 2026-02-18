import numpy as np
from quaternion import Quaternion
from base_component import SimComponentBase


class ADIS16448IMU(SimComponentBase):
    def __init__(self, gravity_vector=np.array([0, 0, -9.8068])):
        super().__init__()
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

        # Gravity vector (world frame)
        self.gravity_vector = np.array(gravity_vector)

        # Bias initialization
        self.gyro_bias = self.gyro_turn_on_bias_sigma * np.random.randn(3)
        self.acc_bias = self.acc_turn_on_bias_sigma * np.random.randn(3)

        # Random generator
        self.rng = np.random.default_rng()
        

    def update(self, t_us, paused):
        """
        Requires inputs via set_inputs(acc_body=..., ang_vel_body=..., orientation_quat=...)
        """
        if paused:
            return self.last_output

        acc_body = self._inputs.get("acc_body")
        ang_vel_body = self._inputs.get("ang_vel_body")
        orientation_quat = self._inputs.get("orientation_quat")

        if acc_body is None or ang_vel_body is None or orientation_quat is None:
            raise ValueError("ADIS16448IMU requires inputs: acc_body, ang_vel_body, orientation_quat")

        dt = self._compute_dt_s(t_us)
        dt = max(dt, 1e-6)

        # Convert quaternion to rotation matrix (world -> body)

        R_wb = Quaternion.Mfg(orientation_quat)

        # Rotate gravity from world to body frame
        gravity_body = R_wb @ self.gravity_vector

        # Subtract gravity from measured acceleration
        acc_corrected =  acc_body + gravity_body

        # Add accelerometer noise and bias
        acc_noisy = self._add_acc_noise(acc_corrected, dt)

        # Add gyroscope noise and bias
        gyro_noisy = self._add_gyro_noise(ang_vel_body, dt)

        self.last_output = (acc_noisy, gyro_noisy)
        return self.last_output


    def _add_gyro_noise(self, omega, dt):
        phi = np.exp(-dt / self.gyro_bias_correlation_time)
        sigma_b = np.sqrt(
            -self.gyro_random_walk**2 * self.gyro_bias_correlation_time / 2.0 *
            (np.exp(-2.0 * dt / self.gyro_bias_correlation_time) - 1.0)
        )
        sigma_d = self.gyro_noise_density / np.sqrt(dt)

        self.gyro_bias = phi * self.gyro_bias + sigma_b * self.rng.normal(0, 1, 3)
        noise = sigma_d * self.rng.normal(0, 1, 3)
        return omega + self.gyro_bias + noise

    def _add_acc_noise(self, acc, dt):
        phi = np.exp(-dt / self.acc_bias_correlation_time)
        sigma_b = np.sqrt(
            -self.acc_random_walk**2 * self.acc_bias_correlation_time / 2.0 *
            (np.exp(-2.0 * dt / self.acc_bias_correlation_time) - 1.0)
        )
        sigma_d = self.acc_noise_density / np.sqrt(dt)

        self.acc_bias = phi * self.acc_bias + sigma_b * self.rng.normal(0, 1, 3)
        noise = sigma_d * self.rng.normal(0, 1, 3)
        return acc + self.acc_bias + noise



if __name__ == "__main__":
    imu = ADIS16448IMU()

    acc_world = np.array([0.0, 0.0, 0.0])  # e.g., stationary
    ang_vel_body = np.array([0.01, 0.0, 0.0])  # slight rotation
    quat = [1.0, 0.0, 0.0, 0.0]  # identity orientation
    imu.set_inputs(acc_body=acc_world, ang_vel_body=ang_vel_body, orientation_quat=quat)
    acc_measured, gyro_measured = imu.update(10_000, paused=False)
    print("Measured acceleration:", acc_measured)
    print("Measured angular velocity:", gyro_measured)

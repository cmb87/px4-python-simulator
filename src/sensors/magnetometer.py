import logging
import numpy as np

from vehicles.base_component import SimComponentBase
from dynamics.quaternion import Quaternion


logger = logging.getLogger(__name__)

class MagnetometerSim(SimComponentBase):
    def __init__(self,
                 pub_rate=100.0,
                 noise_density=0.4e-3,           # [gauss / sqrt(hz)]
                 random_walk=6.4e-6,             # [gauss * sqrt(hz)]
                 bias_correlation_time=600.0,    # [s]
                 declination_rad=0.0391,         # ~2.24° for Zurich
                 inclination_rad=1.1345,         # ~65° for Zurich
                 field_strength_gauss=0.475):    # [gauss] ~47.5 µT
        super().__init__()
        self.pub_rate = pub_rate
        self.dt = 1.0 / float(pub_rate)
        self.noise_density = noise_density
        self.random_walk = random_walk
        self.bias_correlation_time = bias_correlation_time

        # Magnetic field in NED frame [gauss]
        H = field_strength_gauss * np.cos(inclination_rad)
        Z = H * np.tan(inclination_rad)
        X = H * np.cos(declination_rad)
        Y = H * np.sin(declination_rad)
        self.mag_ned = np.array([X, Y, Z])
        self.soft_iron = np.eye(3)
        self.hard_iron = np.zeros(3)
        self.enable_noise = True

        self.bias = np.zeros(3)
        self.rng = np.random.default_rng()
        self.next_update_us = 0
        self.updated = False

    def set_noise(self, enabled: bool):
        self.enable_noise = bool(enabled)

    def set_hard_iron(self, hard_iron_gauss):
        self.hard_iron = np.asarray(hard_iron_gauss, dtype=float).reshape(3)

    def set_soft_iron(self, soft_iron_matrix):
        self.soft_iron = np.asarray(soft_iron_matrix, dtype=float).reshape(3, 3)

    def set_mag_field_ned(self, mag_field_ned):
        self.mag_ned = np.asarray(mag_field_ned, dtype=float).reshape(3)

    def add_noise(self, mag_vector, dt):
        if not self.enable_noise:
            return mag_vector

        tau = max(self.bias_correlation_time, 1e-6)
        sigma_d = self.noise_density / np.sqrt(dt)
        sigma_b = self.random_walk
        sigma_b_d = np.sqrt(-sigma_b**2 * tau / 2.0 * (np.exp(-2.0 * dt / tau) - 1.0))
        phi_d = np.exp(-dt / tau)

        for i in range(3):
            self.bias[i] = phi_d * self.bias[i] + sigma_b_d * self.rng.standard_normal()
            mag_vector[i] += self.bias[i] + sigma_d * self.rng.standard_normal()

        return mag_vector

    def update(self, t_us, paused):
        if paused:
            return self.last_output

        y = self._inputs.get("y")
        if y is not None:
            quat_body_to_world = y[3:7] / np.linalg.norm(y[3:7])
        else:
            quat_body_to_world = self._inputs.get("orientation_quat")

        if quat_body_to_world is None:
            raise ValueError("MagnetometerSim requires input: y or orientation_quat")

        if self.last_output is not None and int(t_us) < self.next_update_us:
            self.updated = False
            return self.last_output

        mag_field_ned = self._inputs.get("mag_field_ned")
        if mag_field_ned is not None:
            self.set_mag_field_ned(mag_field_ned)

        dt = self._compute_dt_s(t_us)
        if dt <= 0.0:
            dt = self.dt

        self.next_update_us = int(int(t_us) + self.dt * 1e6)

        r = Quaternion.Mfg(quat_body_to_world)

        mag_body = r @ self.mag_ned
        mag_body = self.soft_iron @ mag_body + self.hard_iron
        mag_noisy = self.add_noise(mag_body.copy(), dt)

        self.last_output = {
            'timestamp_us': int(t_us),
            'mag_field_body_gauss': mag_noisy
        }
        self.updated = True
        return self.last_output

    def is_updated(self) -> bool:
        return self.updated

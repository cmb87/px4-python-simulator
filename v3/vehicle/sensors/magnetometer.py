import numpy as np
import time
from scipy.spatial.transform import Rotation as R
from quaternion import Quaternion
from base_component import SimComponentBase

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
        self.noise_density = noise_density
        self.random_walk = random_walk
        self.bias_correlation_time = bias_correlation_time

        # Magnetic field in NED frame [gauss]
        H = field_strength_gauss * np.cos(inclination_rad)
        Z = H * np.tan(inclination_rad)
        X = H * np.cos(declination_rad)
        Y = H * np.sin(declination_rad)
        self.mag_ned = np.array([X, Y, Z])

        self.bias = np.zeros(3)
        self.rng = np.random.default_rng()

    def add_noise(self, mag_vector, dt):
        tau = self.bias_correlation_time
        sigma_d = self.noise_density / np.sqrt(dt)
        sigma_b = self.random_walk
        sigma_b_d = np.sqrt(-sigma_b**2 * tau / 2.0 * (np.exp(-2.0 * dt / tau) - 1.0))
        phi_d = np.exp(-dt / tau)

        for i in range(3):
            self.bias[i] = phi_d * self.bias[i] + sigma_b_d * self.rng.standard_normal()
            mag_vector[i] += self.bias[i] + sigma_d * self.rng.standard_normal()

        return mag_vector

    def update(self, t_us, paused):
        """
        Requires inputs via set_inputs(orientation_quat=...)
        """
        if paused:
            return self.last_output

        quat_body_to_world = self._inputs.get("orientation_quat")
        if quat_body_to_world is None:
            raise ValueError("MagnetometerSim requires input: orientation_quat")

        dt = self._compute_dt_s(t_us)
        dt = max(dt, 1e-6)

        r = Quaternion.Mfg(quat_body_to_world)

        mag_body = r @ self.mag_ned
        mag_noisy = self.add_noise(mag_body.copy(), dt)

        self.last_output = {
            'timestamp_us': int(t_us),
            'mag_field_body_gauss': mag_noisy
        }
        return self.last_output

    def simulate_step(self, quat_body_to_world):
        """Compatibility wrapper."""
        base_t_us = 0 if self._last_t_us is None else self._last_t_us
        next_t_us = int(base_t_us + int(1e6 / self.pub_rate))
        self.set_inputs(orientation_quat=quat_body_to_world)
        return self.update(next_t_us, paused=False)
    

if __name__ == "__main__":
    # Example usage
    import time

    sim = MagnetometerSim()

    for _ in range(500):
        quat = [1,0, 0, 0]  # Identity quaternion
        result = sim.simulate_step(quat)
        if result:
            print(f"[{result['timestamp_us']}] mag (body, gauss): {result['mag_field_body_gauss']}")
        time.sleep(0.005)  # Simulate your own timing (e.g., 200 Hz loop)

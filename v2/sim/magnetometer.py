import numpy as np
import time
from scipy.spatial.transform import Rotation as R
from quaternion import Quaternion

class MagnetometerSim:
    def __init__(self,
                 pub_rate=100.0,
                 noise_density=0.4e-3,           # [gauss / sqrt(hz)]
                 random_walk=6.4e-6,             # [gauss * sqrt(hz)]
                 bias_correlation_time=600.0,    # [s]
                 declination_rad=0.0391,         # ~2.24° for Zurich
                 inclination_rad=1.1345,         # ~65° for Zurich
                 field_strength_gauss=0.475):    # [gauss] ~47.5 µT
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
        self.last_time = time.time()
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

    def simulate_step(self, quat_body_to_world):
        """
        Simulate one magnetometer measurement.

        Args:
            quat_body_to_world: Quaternion [x, y, z, w] from body to world frame

        Returns:
            dict or None: {'timestamp_us': ..., 'mag_field_body_gauss': ...}
        """
        now = time.time()
        dt = now - self.last_time
       # if dt < 1.0 / self.pub_rate:
       #     return None

        r = Quaternion.Mfg(quat_body_to_world)

        mag_body = r @ self.mag_ned
        mag_noisy = self.add_noise(mag_body.copy(), dt)

        self.last_time = now
        return {
            'timestamp_us': int(now * 1e6),
            'mag_field_body_gauss': mag_noisy
        }
    

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
import math
import random
from typing import Tuple

import math
import random
from typing import Tuple

class GpsSensor:
    def __init__(self):
        # Internal simulation time
        self.time = 0.0
        self.dt = 0.2  # default to 5 Hz

        self.enable_noise = True

        # Home position (radians, meters)
        self.lat_home = math.radians(47.397742)
        self.lon_home = math.radians(8.545594)
        self.alt_home = 488.0

        # Noise parameters
        self.xy_noise_density = 2.0e-4
        self.z_noise_density = 4.0e-4
        self.vxy_noise_density = 0.2
        self.vz_noise_density = 0.4
        self.xy_random_walk = 2.0
        self.z_random_walk = 4.0

        # Bias and walk
        self.bias = [0.0, 0.0, 0.0]
        self.correlation_time = 60.0

    def set_home(self, lat_deg: float, lon_deg: float, alt_m: float):
        self.lat_home = math.radians(lat_deg)
        self.lon_home = math.radians(lon_deg)
        self.alt_home = alt_m

    def set_noise(self, enabled: bool):
        self.enable_noise = enabled

    def set_update_rate(self, hz: float):
        self.dt = 1.0 / hz

    def tick(self, position_m: Tuple[float, float, float], velocity_mps: Tuple[float, float, float]) -> dict:
        """Advance internal time and return simulated GPS reading."""
        self.time += self.dt

        # Apply noise and bias
        if self.enable_noise:
            noise_pos = [
                self.xy_noise_density * math.sqrt(self.dt) * random.gauss(0, 1),
                self.xy_noise_density * math.sqrt(self.dt) * random.gauss(0, 1),
                self.z_noise_density * math.sqrt(self.dt) * random.gauss(0, 1),
            ]
            noise_vel = [
                self.vxy_noise_density * math.sqrt(self.dt) * random.gauss(0, 1),
                self.vxy_noise_density * math.sqrt(self.dt) * random.gauss(0, 1),
                self.vz_noise_density * math.sqrt(self.dt) * random.gauss(0, 1),
            ]
            random_walk = [
                self.xy_random_walk * math.sqrt(self.dt) * random.gauss(0, 1),
                self.xy_random_walk * math.sqrt(self.dt) * random.gauss(0, 1),
                self.z_random_walk * math.sqrt(self.dt) * random.gauss(0, 1),
            ]
        else:
            noise_pos = [0.0, 0.0, 0.0]
            noise_vel = [0.0, 0.0, 0.0]
            random_walk = [0.0, 0.0, 0.0]

        # Bias update
        for i in range(3):
            self.bias[i] += random_walk[i] * self.dt - self.bias[i] / self.correlation_time

        noisy_pos = [position_m[i] + noise_pos[i] + self.bias[i] for i in range(3)]
        lat, lon = self._reproject(noisy_pos)
        noisy_vel = [velocity_mps[i] + noise_vel[i] for i in range(3)]

        # Return simulated GPS data
        return {
            "time_usec": int(self.time * 1e6),
            "latitude_deg": math.degrees(lat),
            "longitude_deg": math.degrees(lon),
            "altitude_m": noisy_pos[2] + self.alt_home,
            "velocity_north": noisy_vel[0],
            "velocity_east": noisy_vel[1],
            "velocity_up":  noisy_vel[2],
            "eph": 1.0,
            "epv": 1.0
        }

    def _reproject(self, pos_m: Tuple[float, float, float]) -> Tuple[float, float]:
        """Convert local ENU x,y to lat/lon."""
        R_EARTH = 6378137.0
        dLat = pos_m[0] / R_EARTH
        dLon = pos_m[1] / (R_EARTH * math.cos(self.lat_home))
        return self.lat_home + dLat, self.lon_home + dLon

    def reset(self, time_start: float = 0.0):
        """Reset internal time and biases."""
        self.time = time_start
        self.bias = [0.0, 0.0, 0.0]


if __name__ == "__main__":
    gps = GpsSensor()
    gps.set_update_rate(10.0)  # 10 Hz
    gps.set_home(47.397742, 8.545594, 488.0)
    gps.set_noise(True)

    # Simulate 5 steps of GPS output
    for _ in range(5):
        position = (10.0, 5.0, -1.0)  # Local position (meters)
        velocity = (0.5, 0.0, 0.0)    # Velocity (m/s)
        reading = gps.tick(position, velocity)
        print(reading)

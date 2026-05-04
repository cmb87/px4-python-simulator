import logging
import math
import random
from typing import Tuple

from vehicles.base_component import SimComponentBase


logger = logging.getLogger(__name__)

class GpsSensor(SimComponentBase):
    def __init__(self):
        super().__init__()
        # Internal simulation time
        self.time = 0.0
        self.time_us = 0
        self.dt = 0.2  # default to 5 Hz

        self.enable_noise = True

        # Home position (radians, meters)
        self.lat_home = 0.0
        self.lon_home = 0.0
        self.alt_home = 0.0
        self._earth_radius_m = 6371000.0

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
        self.next_update_us = 0
        self.updated = False

    def set_home(self, lat_deg: float, lon_deg: float, alt_m: float):
        self.lat_home = math.radians(lat_deg)
        self.lon_home = math.radians(lon_deg)
        self.alt_home = alt_m

    def set_noise(self, enabled: bool):
        self.enable_noise = enabled

    def set_update_rate(self, hz: float):
        self.dt = 1.0 / hz

    def update(self, t_us: int, paused: bool) -> dict:
        if paused:
            return self.last_output

        position_m = self._inputs.get("position_m")
        velocity_mps = self._inputs.get("velocity_mps")
        if position_m is None or velocity_mps is None:
            raise ValueError("GpsSensor requires inputs: position_m, velocity_mps")

        if self.last_output is not None and int(t_us) < self.next_update_us:
            self.updated = False
            return self.last_output

        dt = self._compute_dt_s(t_us)
        if dt <= 0.0:
            dt = self.dt
        self.time += dt
        self.time_us = int(t_us)
        self.next_update_us = int(self.time_us + self.dt * 1e6)

        # Apply noise and bias
        if self.enable_noise:
            noise_pos = [
                self.xy_noise_density * math.sqrt(dt) * random.gauss(0, 1),
                self.xy_noise_density * math.sqrt(dt) * random.gauss(0, 1),
                self.z_noise_density * math.sqrt(dt) * random.gauss(0, 1),
            ]
            noise_vel = [
                self.vxy_noise_density * math.sqrt(dt) * random.gauss(0, 1),
                self.vxy_noise_density * math.sqrt(dt) * random.gauss(0, 1),
                self.vz_noise_density * math.sqrt(dt) * random.gauss(0, 1),
            ]
            random_walk = [
                self.xy_random_walk * math.sqrt(dt) * random.gauss(0, 1),
                self.xy_random_walk * math.sqrt(dt) * random.gauss(0, 1),
                self.z_random_walk * math.sqrt(dt) * random.gauss(0, 1),
            ]
        else:
            noise_pos = [0.0, 0.0, 0.0]
            noise_vel = [0.0, 0.0, 0.0]
            random_walk = [0.0, 0.0, 0.0]

        # Bias update
        for i in range(3):
            self.bias[i] += random_walk[i] * dt - self.bias[i] / self.correlation_time

        noisy_pos = [position_m[i] + noise_pos[i] + self.bias[i] for i in range(3)]
        lat, lon = self._reproject((noisy_pos[0], noisy_pos[1], noisy_pos[2]))
        noisy_vel = [velocity_mps[i] + noise_vel[i] for i in range(3)]

        # Return simulated GPS data
        self.last_output = {
            "time_usec": self.time_us,
            "latitude_deg": math.degrees(lat),
            "longitude_deg": math.degrees(lon),
            "altitude_m": self.alt_home - noisy_pos[2],
            "velocity_north": noisy_vel[0],
            "velocity_east": noisy_vel[1],
            "velocity_up":  -noisy_vel[2],
            "eph": 1.0,
            "epv": 1.0
        }
        self.updated = True
        return self.last_output

    def is_updated(self) -> bool:
        return self.updated

    def tick(self, position_m: Tuple[float, float, float], velocity_mps: Tuple[float, float, float]) -> dict:
        """Compatibility wrapper."""
        next_t_us = int(self.time * 1e6 + self.dt * 1e6)
        self.set_inputs(position_m=position_m, velocity_mps=velocity_mps)
        return self.update(next_t_us, paused=False)

    def _reproject(self, pos_m: Tuple[float, float, float]) -> Tuple[float, float]:
        """Convert local NED x,y to lat/lon using jMAVSim projector math."""
        x_rad = float(pos_m[0]) / self._earth_radius_m
        y_rad = float(pos_m[1]) / self._earth_radius_m
        c = math.sqrt(x_rad * x_rad + y_rad * y_rad)

        if c > 0.0:
            sin_c = math.sin(c)
            cos_c = math.cos(c)
            sin_lat0 = math.sin(self.lat_home)
            cos_lat0 = math.cos(self.lat_home)

            lat = math.asin(cos_c * sin_lat0 + (x_rad * sin_c * cos_lat0) / c)
            lon = self.lon_home + math.atan2(
                y_rad * sin_c,
                c * cos_lat0 * cos_c - x_rad * sin_lat0 * sin_c,
            )
        else:
            lat = self.lat_home
            lon = self.lon_home

        return lat, lon

    def reset(self, time_start: float = 0.0):
        """Reset internal time and biases."""
        self.time = time_start
        self.time_us = int(time_start * 1e6)
        self.bias = [0.0, 0.0, 0.0]
        self.next_update_us = self.time_us
        self.updated = False
        self._last_t_us = self.time_us


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    gps = GpsSensor()
    gps.set_update_rate(10.0)  # 10 Hz
    gps.set_home(0.0, 0.0, 0.0)
    gps.set_noise(True)

    # Simulate 5 steps of GPS output
    for _ in range(5):
        position = (10.0, 5.0, -1.0)  # Local position (meters)
        velocity = (0.5, 0.0, 0.0)    # Velocity (m/s)
        reading = gps.tick(position, velocity)
        logger.info("%s", reading)

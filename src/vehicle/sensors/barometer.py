import logging
import math
import random

from ..base_component import SimComponentBase


logger = logging.getLogger(__name__)

ISA_TEMPERATURE_MSL_K = 288.15
ISA_PRESSURE_MSL_PA = 101325.0
ISA_LAPSE_RATE_K_PER_M = 0.0065
ISA_AIR_DENSITY_MSL_KGPM3 = 1.225
ABSOLUTE_ZERO_C = -273.15

class BarometerSensor(SimComponentBase):
    def __init__(self):
        super().__init__()
        # Simulation parameters
        self.time = 0.0
        self.time_us = 0
        self.dt = 1.0 / 50.0  # default 50 Hz update rate
        self.enable_noise = True

        self.home_altitude_amsl_m = 0.0
        self.gravity_mps2 = 9.81

        # Drift and noise
        self.pressure_drift_pa_per_sec = 0.0
        self.pressure_drift_pa = 0.0
        self.noise_stddev = 1.0  # 1 Pa RMS
        self.use_last_noise = False
        self.last_noise = 0.0

    def set_update_rate(self, hz: float):
        self.dt = 1.0 / hz

    def set_noise(self, enabled: bool):
        self.enable_noise = enabled

    def set_drift_rate(self, drift_pa_per_sec: float):
        self.pressure_drift_pa_per_sec = drift_pa_per_sec

    def set_home_altitude(self, alt_home_amsl_m: float):
        self.home_altitude_amsl_m = float(alt_home_amsl_m)

    def set_gravity(self, gravity_mps2: float):
        self.gravity_mps2 = float(gravity_mps2)

    def reset(self, time_start: float = 0.0):
        self.time = time_start
        self.time_us = int(time_start * 1e6)
        self.pressure_drift_pa = 0.0
        self.use_last_noise = False
        self._last_t_us = self.time_us

    def _gaussian_noise(self) -> float:
        """Generates Gaussian noise using Box-Muller transform."""
        if self.use_last_noise:
            self.use_last_noise = False
            return self.last_noise

        while True:
            x1 = 2.0 * random.random() - 1.0
            x2 = 2.0 * random.random() - 1.0
            w = x1 * x1 + x2 * x2
            if w < 1.0:
                break

        w = math.sqrt(-2.0 * math.log(w) / w)
        self.last_noise = x2 * w
        self.use_last_noise = True
        return x1 * w

    def update(self, t_us: int, paused: bool) -> dict:
        if paused:
            return self.last_output

        z_position_local = self._inputs.get("z_position_local")
        if z_position_local is None:
            raise ValueError("BarometerSensor requires input: z_position_local")

        dt = self._compute_dt_s(t_us)
        if dt <= 0.0:
            dt = self.dt

        self.time += dt
        self.time_us = int(t_us)

        # Altitude above sea level
        alt_rel = z_position_local
        alt_amsl = self.home_altitude_amsl_m + alt_rel

        # Temperature at current altitude
        temperature = ISA_TEMPERATURE_MSL_K - ISA_LAPSE_RATE_K_PER_M * alt_amsl

        # Ideal pressure at altitude
        pressure_ratio = (ISA_TEMPERATURE_MSL_K / temperature) ** 5.256
        absolute_pressure = ISA_PRESSURE_MSL_PA / pressure_ratio

        # Add noise and drift
        noise = self._gaussian_noise() * self.noise_stddev if self.enable_noise else 0.0
        self.pressure_drift_pa += self.pressure_drift_pa_per_sec * dt
        pressure_noisy = absolute_pressure + noise + self.pressure_drift_pa

        # Convert pressure to hPa
        pressure_hpa = pressure_noisy * 0.01

        # Calculate air density at current altitude
        density_ratio = (ISA_TEMPERATURE_MSL_K / temperature) ** 4.256
        air_density = ISA_AIR_DENSITY_MSL_KGPM3 / density_ratio

        # Compute pressure altitude (approximate)
        pressure_altitude = alt_amsl - (noise + self.pressure_drift_pa) / (self.gravity_mps2 * air_density)

        # Temperature in Celsius
        temperature_c = temperature + ABSOLUTE_ZERO_C

        self.last_output = {
            "time_usec": self.time_us,
            "absolute_pressure_hpa": pressure_hpa,
            "pressure_altitude_m": pressure_altitude,
            "temperature_c": temperature_c,
        }
        return self.last_output

    def tick(self, z_position_local: float) -> dict:
        """Compatibility wrapper."""
        next_t_us = int(self.time * 1e6 + self.dt * 1e6)
        self.set_inputs(z_position_local=z_position_local)
        return self.update(next_t_us, paused=False)
    

if __name__ == "__main__":
    # Example usage
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    baro = BarometerSensor()
    baro.set_update_rate(20.0)  # 20 Hz
    baro.set_drift_rate(0.05)   # 0.05 Pa/s drift
    baro.set_noise(True)

    for _ in range(10):
        altitude = 20.0  # local z-position in meters
        reading = baro.tick(altitude)
        logger.info("%s", reading)

import numpy as np
from vehicles.base_component import SimComponentBase

DEFAULT_DIFF_PRESSURE_NOISE_STD = 0.002
DEFAULT_DIFF_PRESSURE_LPF_TAU_S = 0.08

class AirspeedSensor(SimComponentBase):
    def __init__(self):
        super().__init__()
        self._diff_pressure_initialized = False
        self._diff_pressure_pa = 0.0
        self.updated = False
        self.last_output = {
            "airspeed_ias_mps": 0.0,
            "airspeed_tas_mps": 0.0,
            "dynamic_pressure_pa": 0.0,
        }

    def update(self, t_us, paused):
        if paused:
            return self.last_output

        y = self._inputs.get("y")
        wind = self._inputs.get("wind")
        P = self._inputs.get("P")

        if y is None or P is None:
            self.updated = False
            return self.last_output

        vel_body = y[7:10]
        wind_vec = np.zeros(6) if wind is None else np.asarray(wind, dtype=float)
        wind_body = wind_vec[:3]

        vel_air_body = np.asarray(vel_body, dtype=float) - np.asarray(wind_body, dtype=float)

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
        
        diff_pressure_lpf_tau_s = max(float(DEFAULT_DIFF_PRESSURE_LPF_TAU_S), 1e-6)
        alpha_dp = dt / (diff_pressure_lpf_tau_s + dt)

        if not self._diff_pressure_initialized:
            self._diff_pressure_pa = dynamic_pressure_ideal
            self._diff_pressure_initialized = True
        else:
            self._diff_pressure_pa = self._diff_pressure_pa + alpha_dp * (dynamic_pressure_ideal - self._diff_pressure_pa)

        diff_pressure_noise_std = float(DEFAULT_DIFF_PRESSURE_NOISE_STD)
        dynamic_pressure_meas = self._diff_pressure_pa + np.random.normal(0.0, diff_pressure_noise_std)
        dynamic_pressure_meas = max(float(dynamic_pressure_meas), 0.0)
        
        rho_air = max(float(P.rho), 1e-6)
        airspeed_ias_mps = float(np.sqrt(2.0 * dynamic_pressure_meas / rho_air))
        airspeed_tas_mps = float(np.linalg.norm(vel_air_body))

        self.last_output = {
            "airspeed_ias_mps": airspeed_ias_mps,
            "airspeed_tas_mps": airspeed_tas_mps,
            "dynamic_pressure_pa": dynamic_pressure_meas,
        }
        self.updated = True
        return self.last_output

    def is_updated(self) -> bool:
        return self.updated

import numpy as np

from base_component import SimComponentBase
from dynamics import Dynamics6DOF
from model.iris import IrisParameters, build_force_models as build_iris_force_models
from model.ts04 import TS04Parameters, build_force_models as build_ts04_force_models
from model.x8 import X8Parameters, build_force_models as build_x8_force_models
from sensors.sensors import SensorSuite


class World(SimComponentBase):
    def __init__(
        self,
        parameters=None,
        y0=None,
        u0=None,
        wind0=None,
        z_ground=100.0,
        force_models=None,
        vehicle_model="x8",
        ts04_pitch90_start=False,
    ):
        super().__init__()
        self.vehicle_model = str(vehicle_model).strip().lower()
        self.ts04_pitch90_start = bool(ts04_pitch90_start)
        self.P = self._build_parameters(self.vehicle_model) if parameters is None else parameters

        self.y = np.zeros(13) if y0 is None else np.asarray(y0, dtype=float).copy()
        if y0 is None:
            if self.vehicle_model == "ts04" and self.ts04_pitch90_start:
                self.y[3:7] = np.array([np.sqrt(0.5), 0.0, np.sqrt(0.5), 0.0])
            else:
                self.y[3] = 1.0
        self.u = np.zeros(4) if u0 is None else np.asarray(u0, dtype=float).copy()
        self.wind = np.zeros(6) if wind0 is None else np.asarray(wind0, dtype=float).copy()

        self.dynamics = Dynamics6DOF(z_ground=z_ground)
        if force_models is None:
            self.force_models = self._build_force_models(self.vehicle_model)
        else:
            self.force_models = list(force_models)
        self.sensor_suite = SensorSuite()

    @staticmethod
    def _build_parameters(vehicle_model):
        if vehicle_model == "x8":
            return X8Parameters()
        if vehicle_model == "iris":
            return IrisParameters()
        if vehicle_model == "ts04":
            return TS04Parameters()
        raise ValueError(f"Unknown vehicle model '{vehicle_model}'")

    @staticmethod
    def _build_force_models(vehicle_model):
        if vehicle_model == "x8":
            return build_x8_force_models()
        if vehicle_model == "iris":
            return build_iris_force_models()
        if vehicle_model == "ts04":
            return build_ts04_force_models()
        raise ValueError(f"Unknown vehicle model '{vehicle_model}'")

    def set_state(self, y):
        self.y = np.asarray(y, dtype=float).copy()

    def set_controls(self, u):
        self.u = np.asarray(u, dtype=float).copy()

    def set_wind(self, wind):
        self.wind = np.asarray(wind, dtype=float).copy()

    def set_force_models(self, force_models):
        self.force_models = list(force_models)

    def add_force_model(self, force_model):
        self.force_models.append(force_model)

    def clear_force_models(self):
        self.force_models = []

    def update(self, t_us, paused, freeze_dynamics=False):
        if paused:
            return self.last_output

        dt = self._compute_dt_s(t_us)

        if freeze_dynamics:
            tau = np.zeros(6)
            ydot = np.zeros(13)
        else:
            tau = np.zeros(6)
            for force_model in self.force_models:
                force_model.set_inputs(y=self.y, u=self.u, wind=self.wind, P=self.P)
                tau = tau + force_model.update(t_us, paused=False)

            self.dynamics.set_inputs(y=self.y, P=self.P, tau=tau)
            ydot = self.dynamics.update(t_us, paused=False)

            if dt > 0.0:
                self.y = self.y + ydot * dt
                quat_norm = np.linalg.norm(self.y[3:7])
                if quat_norm > 0.0:
                    self.y[3:7] = self.y[3:7] / quat_norm

        self.sensor_suite.set_inputs(y=self.y, ydot=ydot, u=self.u, wind=self.wind, P=self.P, tau=tau)
        z = self.sensor_suite.update(t_us, paused=False)

        self.last_output = {
            "t_us": int(t_us),
            "y": self.y.copy(),
            "ydot": ydot.copy(),
            "tau": tau.copy(),
            "sensors": z,
        }
        return self.last_output

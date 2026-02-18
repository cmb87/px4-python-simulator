import numpy as np

from base_component import SimComponentBase
from dynamics import Dynamics6DOF
from forces.wing_x8 import WingX8ForceModel
from sensors.sensors import SensorSuite


class World(SimComponentBase):
    def __init__(self, parameters, y0=None, u0=None, wind0=None, z_ground=100.0, force_models=None):
        super().__init__()
        self.P = parameters

        self.y = np.zeros(13) if y0 is None else np.asarray(y0, dtype=float).copy()
        if y0 is None:
            self.y[3] = 1.0
        self.u = np.zeros(4) if u0 is None else np.asarray(u0, dtype=float).copy()
        self.wind = np.zeros(6) if wind0 is None else np.asarray(wind0, dtype=float).copy()

        self.dynamics = Dynamics6DOF(z_ground=z_ground)
        if force_models is None:
            self.force_models = [WingX8ForceModel()]
        else:
            self.force_models = list(force_models)
        self.sensor_suite = SensorSuite()

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

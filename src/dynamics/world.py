import logging

import numpy as np

from vehicles.base_component import SimComponentBase
from .dynamics import Dynamics6DOF, rail_dynamics
from vehicles.vehicle_catalog import get_vehicle_definition
from vehicles.common_forces.catapult_rail import railForces, rail_alignment_quaternion_wxyz
from sensors.sensors import SensorSuite


logger = logging.getLogger(__name__)


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
        model_options=None,
    ):
        super().__init__()
        self.vehicle_model = str(vehicle_model).strip().lower()
        self.model_options = {} if model_options is None else dict(model_options)

        self._vehicle_def = get_vehicle_definition(self.vehicle_model)
        self.P = self._vehicle_def.make_parameters() if parameters is None else parameters

        if y0 is None:
            self.y = np.asarray(self._vehicle_def.make_initial_state(self.model_options), dtype=float).copy()
        else:
            self.y = np.asarray(y0, dtype=float).copy()
        self.u = np.zeros(4) if u0 is None else np.asarray(u0, dtype=float).copy()
        self.wind = np.zeros(6) if wind0 is None else np.asarray(wind0, dtype=float).copy()

        self.dynamics = Dynamics6DOF(z_ground=z_ground)
        if force_models is None:
            self.force_models = list(self._vehicle_def.make_force_models(self.P))
        else:
            self.force_models = list(force_models)
        self.sensor_suite = SensorSuite()

        self.rail_launch_enabled = bool(
            self.model_options.get("rail_launch_enabled", getattr(self.P, "rail_launch_enabled", False))
        )
        self._rail_exit_logged = False
        self._rail_force_logged = False
        self._configure_rail_launch()

    def reset(self, y0=None, u0=None, wind0=None, t_us=0, launch_mode="default"):
        launch_mode = str(launch_mode).strip().lower()
        if launch_mode not in {"default", "catapult", "airborne", "free"}:
            raise ValueError("launch_mode must be one of default|catapult|airborne|free")

        if y0 is None:
            self.y = np.asarray(self._vehicle_def.make_initial_state(self.model_options), dtype=float).copy()
        else:
            self.y = np.asarray(y0, dtype=float).copy()
        self.u = np.zeros(4) if u0 is None else np.asarray(u0, dtype=float).copy()
        self.wind = np.zeros(6) if wind0 is None else np.asarray(wind0, dtype=float).copy()

        self.dynamics = Dynamics6DOF(z_ground=self.dynamics.z_ground)
        self.force_models = list(self._vehicle_def.make_force_models(self.P))
        self.sensor_suite = SensorSuite()
        self.last_output = None
        self._last_t_us = int(t_us)
        self.dynamics._last_t_us = int(t_us)
        self._rail_exit_logged = False
        self._rail_force_logged = False

        if launch_mode == "catapult":
            self.rail_launch_enabled = True
            self._configure_rail_launch(place_on_rail=True)
        elif launch_mode == "airborne":
            self.rail_launch_enabled = bool(
                self.model_options.get("rail_launch_enabled", getattr(self.P, "rail_launch_enabled", False))
            )
            self._configure_rail_launch(place_on_rail=False)
        elif launch_mode == "free":
            self.rail_launch_enabled = False
            self._configure_rail_launch(place_on_rail=False)
        else:
            self.rail_launch_enabled = bool(
                self.model_options.get("rail_launch_enabled", getattr(self.P, "rail_launch_enabled", False))
            )
            self._configure_rail_launch(place_on_rail=True)

    def _configure_rail_launch(self, place_on_rail=True):
        if not self.rail_launch_enabled:
            self.P.left_rail = True
            return

        default_dir = np.array([np.cos(np.deg2rad(45.0)), 0.0, -np.sin(np.deg2rad(45.0))], dtype=float)
        rail_dir = np.asarray(self.model_options.get("rail_dir_ned", getattr(self.P, "rail_dir_ned", default_dir)), dtype=float)
        rail_dir_norm = float(np.linalg.norm(rail_dir))
        if rail_dir_norm <= 1e-9:
            raise ValueError("Rail launch requires non-zero rail_dir_ned")
        self.P.rail_dir_ned = rail_dir / rail_dir_norm

        default_start = np.array([0.0, 0.0, 0.0], dtype=float)
        self.P.rail_start_ned = np.asarray(
            self.model_options.get("rail_start_ned", getattr(self.P, "rail_start_ned", default_start)),
            dtype=float,
        )
        self.P.rail_length = float(self.model_options.get("rail_length_m", getattr(self.P, "rail_length", 2.0)))
        self.P.rail_pull_max = float(self.model_options.get("rail_pull_max", getattr(self.P, "rail_pull_max", 1.0)))
        self.P.left_rail = not bool(place_on_rail)

        initial_pull_n = self.P.rail_pull_max * float(self.P.gravity) * float(self.P.rail_length)
        logger.info(
            "Catapult launch enabled: pull_max=%.3f, rail_length=%.3f m, initial_pull=%.3f N",
            float(self.P.rail_pull_max),
            float(self.P.rail_length),
            float(initial_pull_n),
        )

        if place_on_rail:
            self.y[0:3] = self.P.rail_start_ned
            rail_quat = rail_alignment_quaternion_wxyz(self.P)
            self.y[3:7] = rail_quat
            self.y[10:13] = 0.0

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

    def _evaluate_forces(self, t_us):
        tau = np.zeros(6)
        for force_model in self.force_models:
            force_model.set_inputs(y=self.y, u=self.u, wind=self.wind, P=self.P)
            tau = tau + force_model.update(t_us, paused=False)
        return tau

    def _run_dynamics(self, t_us, tau, dt):
        tau_used = np.asarray(tau, dtype=float).copy()
        if self.rail_launch_enabled and (not bool(getattr(self.P, "left_rail", True))):
            self.y[3:7] = rail_alignment_quaternion_wxyz(self.P)
            self.y[10:13] = 0.0
            rail_tau = railForces(float(t_us) / 1e6, self.y, self.u, self.wind, self.P)
            tau_used = tau_used + rail_tau
            if not self._rail_force_logged:
                self._rail_force_logged = True
                logger.info(
                    "Applying catapult rail force: Fx=%.3f Fy=%.3f Fz=%.3f N",
                    float(rail_tau[0]),
                    float(rail_tau[1]),
                    float(rail_tau[2]),
                )
            ydot = rail_dynamics(float(t_us) / 1e6, self.y, self.P, tau_used)
        else:
            self.dynamics.set_inputs(y=self.y, P=self.P, tau=tau_used)
            ydot = self.dynamics.update(t_us, paused=False)

        if dt > 0.0:
            self.y = self.y + ydot * dt
            quat_norm = np.linalg.norm(self.y[3:7])
            if quat_norm > 0.0:
                self.y[3:7] = self.y[3:7] / quat_norm
        if self.rail_launch_enabled and (not bool(getattr(self.P, "left_rail", True))):
            self.y[3:7] = rail_alignment_quaternion_wxyz(self.P)
            self.y[10:13] = 0.0
        elif self.rail_launch_enabled and bool(getattr(self.P, "left_rail", False)) and (not self._rail_exit_logged):
            self._rail_exit_logged = True
            logger.info("Rail left; switching to free 6DOF dynamics")
        return ydot, tau_used

    def _run_sensors(self, t_us, ydot, tau):
        self.sensor_suite.set_inputs(y=self.y, ydot=ydot, u=self.u, wind=self.wind, P=self.P, tau=tau)
        return self.sensor_suite.update(t_us, paused=False)

    def update(self, t_us, paused, freeze_dynamics=False):
        if paused:
            return self.last_output

        dt = self._compute_dt_s(t_us)

        if freeze_dynamics:
            tau = np.zeros(6)
            ydot = np.zeros(13)
        else:
            tau = self._evaluate_forces(t_us)
            ydot, tau = self._run_dynamics(t_us, tau, dt)

        z = self._run_sensors(t_us, ydot, tau)

        self.last_output = {
            "t_us": int(t_us),
            "y": self.y.copy(),
            "ydot": ydot.copy(),
            "tau": tau.copy(),
            "sensors": z,
        }
        return self.last_output

    def observe_external_state(self, t_us, y, ydot, tau=None):
        self.y = np.asarray(y, dtype=float).copy()
        ydot = np.asarray(ydot, dtype=float).copy()
        tau_vec = np.zeros(6) if tau is None else np.asarray(tau, dtype=float).copy()

        z = self._run_sensors(t_us, ydot, tau_vec)

        self.last_output = {
            "t_us": int(t_us),
            "y": self.y.copy(),
            "ydot": ydot.copy(),
            "tau": tau_vec.copy(),
            "sensors": z,
        }
        return self.last_output

    def sync_time(self, t_us):
        t_val = int(t_us)
        self._last_t_us = t_val
        self.dynamics._last_t_us = t_val

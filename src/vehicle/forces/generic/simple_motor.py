import numpy as np


class SimpleMotor:
    def __init__(self, full_thrust=1.0, full_torque=0.02, time_constant=0.08):
        self.tau = float(time_constant)
        self.full_thrust = float(full_thrust)
        self.full_torque = float(full_torque)
        self.w = 0.0
        self.control = 0.0
        self._last_t_us = None

    def set_control(self, control):
        self.control = float(np.clip(control, 0.0, 1.0))

    def set_full_thrust(self, full_thrust):
        self.full_thrust = float(full_thrust)

    def set_full_torque(self, full_torque):
        self.full_torque = float(full_torque)

    def set_time_constant(self, time_constant):
        self.tau = max(float(time_constant), 1e-6)

    def update(self, t_us, paused):
        if paused:
            return

        if self._last_t_us is not None:
            dt = max((int(t_us) - int(self._last_t_us)) / 1e6, 0.0)
            alpha = 1.0 - np.exp(-dt / max(self.tau, 1e-6))
            self.w += (self.control - self.w) * alpha
        self._last_t_us = int(t_us)

    def get_thrust(self):
        return self.w * self.full_thrust

    def get_torque(self):
        return self.control * self.full_torque

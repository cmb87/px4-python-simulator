import numpy as np

from base_component import SimComponentBase
from forces.generic.simple_motor import SimpleMotor


class IrisQuadForceModel(SimComponentBase):
    def __init__(self):
        super().__init__()
        self.motors = [SimpleMotor() for _ in range(4)]

    def _configure_from_parameters(self, P):
        for motor in self.motors:
            motor.set_full_thrust(P.motor_full_thrust)
            motor.set_full_torque(P.motor_full_torque)
            motor.set_time_constant(P.motor_time_constant)

    def update(self, t_us, paused):
        if paused:
            return self.last_output

        y = self._inputs.get("y")
        u = self._inputs.get("u")
        P = self._inputs.get("P")

        if y is None or u is None or P is None:
            raise ValueError("IrisQuadForceModel requires inputs: y, u, P")

        self._configure_from_parameters(P)
        body_rates = np.asarray(y[10:13], dtype=float)

        controls = np.asarray(u, dtype=float)
        if controls.shape[0] < 4:
            controls = np.pad(controls, (0, 4 - controls.shape[0]))
        controls = np.clip(controls[:4], 0.0, 1.0)

        total_force = np.zeros(3)
        total_torque = np.zeros(3)

        arm = float(P.arm_length)
        arm_xy = arm / np.sqrt(2.0)

        # Option A mapping/order:
        # u0 -> FrontRight, u1 -> RearLeft, u2 -> FrontLeft, u3 -> RearRight
        rotor_positions = np.array(
            [
                [arm_xy, arm_xy, 0.0],
                [-arm_xy, -arm_xy, 0.0],
                [arm_xy, -arm_xy, 0.0],
                [-arm_xy, arm_xy, 0.0],
            ],
            dtype=float,
        )

        # Yaw torque sign per rotor (CW positive, CCW negative).
        yaw_sign = np.array([1.0, 1.0, -1.0, -1.0], dtype=float)
        rotor_max_omega = float(getattr(P, "motor_max_omega", 0.0))
        rotor_polar_inertia = float(getattr(P, "rotor_polar_inertia", 0.0))
        rotor_angular_momentum_z = 0.0

        for i, motor in enumerate(self.motors):
            motor.set_control(controls[i])
            motor.update(t_us, paused=False)

            thrust = motor.get_thrust()
            reaction_torque = motor.get_torque()

            force_i = np.array([0.0, 0.0, -thrust], dtype=float)
            torque_from_arm = np.cross(rotor_positions[i], force_i)
            torque_i = torque_from_arm + np.array([0.0, 0.0, yaw_sign[i] * reaction_torque], dtype=float)

            total_force += force_i
            total_torque += torque_i
            rotor_angular_momentum_z += yaw_sign[i] * rotor_polar_inertia * (motor.w * rotor_max_omega)

        rotor_angular_momentum = np.array([0.0, 0.0, rotor_angular_momentum_z], dtype=float)
        gyroscopic_moment = -np.cross(body_rates, rotor_angular_momentum)
        total_torque += gyroscopic_moment

        self.last_output = np.concatenate([total_force, total_torque])
        self._last_t_us = int(t_us)
        return self.last_output

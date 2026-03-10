import numpy as np

from ...Rzyx import Rzyx
from ...base_component import SimComponentBase
from ...quaternion import Quaternion
from ..common_forces.simple_motor import SimpleMotor


class TS04ForceModel(SimComponentBase):
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
            raise ValueError("TS04ForceModel requires inputs: y, u, P")

        self._configure_from_parameters(P)
        body_rates = np.asarray(y[10:13], dtype=float)

        controls = np.asarray(u, dtype=float)
        if controls.shape[0] < 4:
            controls = np.pad(controls, (0, 4 - controls.shape[0]))
        controls = np.clip(controls[:4], 0.0, 1.0)

        total_force = np.zeros(3)
        total_torque = np.zeros(3)

        motor_count = len(self.motors)
        rotor_positions = np.asarray(getattr(P, "motor_positions_body_m", np.zeros((motor_count, 3))), dtype=float)

        if rotor_positions.shape != (motor_count, 3):
            raise ValueError(
                f"TS04ForceModel expected motor_positions_body_m shape {(motor_count, 3)}, got {rotor_positions.shape}"
            )

        motor_directions = np.asarray(
            getattr(P, "motor_thrust_directions_body", np.tile(np.array([[1.0, 0.0, 0.0]]), (motor_count, 1))),
            dtype=float,
        )

        if motor_directions.shape != (motor_count, 3):
            raise ValueError(
                f"TS04ForceModel expected motor_thrust_directions_body shape {(motor_count, 3)}, got {motor_directions.shape}"
            )

        direction_norms = np.linalg.norm(motor_directions, axis=1)
        if np.any(direction_norms <= 1e-9):
            raise ValueError("TS04ForceModel motor_thrust_directions_body must contain non-zero vectors")
        motor_directions = motor_directions / direction_norms[:, None]

        spin_sign = np.asarray(getattr(P, "motor_spin_sign", np.array([-1.0, -1.0, 1.0, 1.0])), dtype=float)
        if spin_sign.shape != (motor_count,):
            raise ValueError(f"TS04ForceModel expected motor_spin_sign shape {(motor_count,)}, got {spin_sign.shape}")

        rotor_max_omega = float(getattr(P, "motor_max_omega", 0.0))
        rotor_polar_inertia = float(getattr(P, "rotor_polar_inertia", 0.0))
        rotor_angular_momentum = np.zeros(3)

        for i, motor in enumerate(self.motors):
            motor.set_control(controls[i])
            motor.update(t_us, paused=False)

            thrust = motor.get_thrust()
            reaction_torque = motor.w * float(P.motor_full_torque)

            direction_i = motor_directions[i]
            force_i = thrust * direction_i
            torque_from_arm = np.cross(rotor_positions[i], force_i)
            reaction_torque_i = spin_sign[i] * reaction_torque * direction_i
            torque_i = torque_from_arm + reaction_torque_i

            total_force += force_i
            total_torque += torque_i
            rotor_angular_momentum += spin_sign[i] * rotor_polar_inertia * (motor.w * rotor_max_omega) * direction_i

        gyroscopic_moment = -np.cross(body_rates, rotor_angular_momentum)
        total_torque += gyroscopic_moment

        self.last_output = np.concatenate([total_force, total_torque])
        self._last_t_us = int(t_us)
        return self.last_output


def _smoothstep01(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


class TS04BlendedPassiveAeroForceModel(SimComponentBase):
    def __init__(self):
        super().__init__()
        self._debug_counter = 0

    def update(self, t_us, paused):
        if paused:
            return self.last_output

        y = self._inputs.get("y")
        wind = self._inputs.get("wind")
        P = self._inputs.get("P")

        if y is None or wind is None or P is None:
            raise ValueError("TS04BlendedPassiveAeroForceModel requires inputs: y, wind, P")

        vel_body = np.asarray(y[7:10], dtype=float)
        body_rates = np.asarray(y[10:13], dtype=float)
        wind_body = np.asarray(wind[:3], dtype=float)
        wind_rates = np.asarray(wind[3:6], dtype=float)
        vel_rel = vel_body - wind_body

        speed = np.linalg.norm(vel_rel)
        q_sphere = 0.5 * float(P.rho) * speed
        sphere_force = -q_sphere * float(P.sphere_cd) * float(P.sphere_area) * vel_rel

        u_r, v_r, w_r = vel_rel
        va = max(speed, 1e-5)
        alpha = np.arctan2(w_r, u_r)
        beta = np.arcsin(np.clip(v_r / va, -1.0, 1.0))

        p, q, r = body_rates + wind_rates

        c_l_alpha = float(P.C_L_0) + float(P.C_L_alpha) * alpha
        f_lift_s = 0.5 * float(P.rho) * va**2 * float(P.S_wing) * (
            c_l_alpha + float(P.C_L_q) * float(P.c) / (2.0 * va) * q
        )

        c_d_alpha = float(P.C_D_0) + float(P.C_D_alpha1) * alpha + float(P.C_D_alpha2) * alpha**2
        c_d_beta = float(P.C_D_beta1) * beta + float(P.C_D_beta2) * beta**2

        f_drag_s = 2.0 * 0.5 * float(P.rho) * va**2 * float(P.S_wing) * (
            c_d_alpha + c_d_beta + float(P.C_D_q) * float(P.c) / (2.0 * va) * q
        )

        m_a = float(P.C_m_0) + float(P.C_m_alpha) * alpha
        m = 0.1 * 0.5 * float(P.rho) * va**2 * float(P.S_wing) * float(P.c) * (
            m_a + float(P.C_m_q) * float(P.c) / (2.0 * va) * q
        )

        f_y = 0.5 * float(P.rho) * va**2 * float(P.S_wing) * (
            float(P.C_Y_0)
            + float(P.C_Y_beta) * beta
            + float(P.C_Y_p) * float(P.b) / (2.0 * va) * p
            + float(P.C_Y_r) * float(P.b) / (2.0 * va) * r
        )

        l = 0.1 * 0.5 * float(P.rho) * va**2 * float(P.b) * float(P.S_wing) * (
            float(P.C_l_0)
            + float(P.C_l_beta) * beta
            + float(P.C_l_p) * float(P.b) / (2.0 * va) * p
            + float(P.C_l_r) * float(P.b) / (2.0 * va) * r
        )

        n = 0.1 * 0.5 * float(P.rho) * va**2 * float(P.b) * float(P.S_wing) * (
            float(P.C_n_0)
            + float(P.C_n_beta) * beta
            + float(P.C_n_p) * float(P.b) / (2.0 * va) * p
            + float(P.C_n_r) * float(P.b) / (2.0 * va) * r
        )

        wing_force = Rzyx(0.0, alpha, beta).T @ np.array([-f_drag_s, f_y, -f_lift_s], dtype=float)
        wing_torque = np.array([l, m, n], dtype=float)

        abs_alpha_deg = float(np.abs(np.rad2deg(alpha)))
        stall_end_deg = float(P.passive_wing_stall_alpha_deg)
        stall_start_deg = float(getattr(P, "passive_wing_stall_start_alpha_deg", stall_end_deg - 5.0))
        if stall_end_deg < stall_start_deg:
            stall_start_deg, stall_end_deg = stall_end_deg, stall_start_deg
        stall_span = max(stall_end_deg - stall_start_deg, 1e-6)

        if abs_alpha_deg <= stall_start_deg:
            stall_weight = 1.0
        elif abs_alpha_deg >= stall_end_deg:
            stall_weight = 0.0
        else:
            stall_blend = (abs_alpha_deg - stall_start_deg) / stall_span
            stall_weight = 1.0 - _smoothstep01(stall_blend)

        wing_force = wing_force * stall_weight
        wing_torque = wing_torque * stall_weight

        wing_force = wing_force * float(P.passive_wing_force_scale)
        wing_torque = wing_torque * float(P.passive_wing_moment_scale)

        quat = np.asarray(y[3:7], dtype=float)
        quat_norm = np.linalg.norm(quat)
        if quat_norm <= 0.0:
            quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        else:
            quat = quat / quat_norm

        body_x_ned = Quaternion.Mfg(quat).T @ np.array([1.0, 0.0, 0.0], dtype=float)
        tilt_deg = np.rad2deg(np.abs(np.arcsin(np.clip(body_x_ned[2], -1.0, 1.0))))

        tilt_sphere_deg = float(P.blend_tilt_sphere_deg)
        tilt_wing_deg = float(P.blend_tilt_wing_deg)
        tilt_high = max(tilt_sphere_deg, tilt_wing_deg)
        tilt_low = min(tilt_sphere_deg, tilt_wing_deg)
        tilt_span = max(tilt_high - tilt_low, 1e-6)
        blend_linear = (tilt_high - tilt_deg) / tilt_span
        wing_weight = _smoothstep01(blend_linear)

        total_force = (1.0 - wing_weight) * sphere_force + wing_weight * wing_force
        total_torque = wing_weight * wing_torque

        self.last_output = np.concatenate([total_force, total_torque])
        self._last_t_us = int(t_us)
        return self.last_output

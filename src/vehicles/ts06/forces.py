import logging
import numpy as np

from vehicles.base_component import SimComponentBase
from .aero_lookup import lookup_aerodynamics


logger = logging.getLogger(__name__)


def forces(t, y, u, wind, P, alpha_override=None, beta_override=None):
    vel = y[7:10]
    rate = y[10:13]

    p = rate[0] + wind[3]
    q = rate[1] + wind[4]
    r = rate[2] + wind[5]
    rates_winded = np.array([p, q, r])

    wind_b = wind[:3]
    vel_r = vel - wind_b

    v_air = np.linalg.norm(vel_r)
    if v_air < 1e-5:
        v_air = 1e-5

    alpha = np.arctan2(vel_r[2], vel_r[0])
    beta = np.arcsin(np.clip(vel_r[1] / v_air, -1.0, 1.0))

    # Apply overrides (for filtered aerodynamic angles)
    alpha_val = alpha_override if alpha_override is not None else alpha
    beta_val = beta_override if beta_override is not None else beta

    alpha_deg_unclamped = np.rad2deg(alpha_val)
    beta_deg_unclamped = np.rad2deg(beta_val)

    # Envelope protection: clamp to +/- 15 degrees for CFD lookup
    alpha_deg = np.clip(alpha_deg_unclamped, -15.0, 15.0)
    beta_deg = np.clip(beta_deg_unclamped, -15.0, 15.0)

    is_out_of_envelope = (abs(alpha_deg_unclamped) > 15.0) or (abs(beta_deg_unclamped) > 15.0)
    if is_out_of_envelope:
        # Rate-limited warning (max once per second) to prevent console spam
        if t - getattr(forces, "_last_envelope_warn_t", 0.0) > 1.0:
            logger.warning(
                "WARNING: TS06 OUT OF NOMINAL ENVELOPE (alpha = %.2f deg, beta = %.2f deg)",
                alpha_deg_unclamped, beta_deg_unclamped
            )
            forces._last_envelope_warn_t = t

    # 1. CFD-based Aerodynamics Lookup (using provided reference area and reference length)
    Aref = 1.520000e-01
    lRef = 9.580000e-02
    vair_const = 45.0

    Cd, Cs, Cl, CmRoll, CmPitch, CmYaw = lookup_aerodynamics(vair_const, alpha_deg, beta_deg)

    q_bar = 0.5 * P.rho * v_air**2
    
    f_aero_body = q_bar * Aref * np.array([Cd, Cs, Cl])
    
    # Dynamic Rate Damping Moments
    p_hat = p * lRef / (2.0 * v_air)
    q_hat = q * lRef / (2.0 * v_air)
    r_hat = r * lRef / (2.0 * v_air)
    
    C_l_p = getattr(P, "C_l_p", -0.5)
    C_m_q = getattr(P, "C_m_q", -8.0)
    C_n_r = getattr(P, "C_n_r", -0.2)
    
    t_damping = q_bar * Aref * lRef * np.array([
        C_l_p * p_hat,
        C_m_q * q_hat,
        C_n_r * r_hat
    ])

    # Needs to be -CMYaw because Openfoam uses a different COS
    
    t_aero_body = q_bar * Aref * lRef * np.array([CmRoll, CmPitch, -CmYaw]) + t_damping

    # 2. Propulsion (4 Motors Model)
    f_prop_total = np.zeros(3)
    t_prop_total = np.zeros(3)

    # Pad control signals to 4 elements if needed
    u_motors = np.zeros(4)
    for i in range(min(4, len(u))):
        u_motors[i] = u[i]

    # Motor rotation directions from parameters (1 = CCW, -1 = CW)
    dir_motors = getattr(P, "dir_motors", [1, -1, -1, 1])

    for i in range(4):
        r_motor = np.array([P.x_motors[i], P.y_motors[i], P.z_motors[i]], dtype=float)
        v_motor_body = vel_r + np.cross(rates_winded, r_motor)
        v_a_motor = v_motor_body[0]  # Forward airspeed seen by propeller

        throttle_i = np.clip(u_motors[i], 0.0, 1.0)
        v_d_motor = v_a_motor + throttle_i * (P.k_motor - v_a_motor)
        thrust_i = 0.5 * P.rho * P.S_prop * P.C_prop * v_d_motor * (v_d_motor - v_a_motor)

        f_motor_i = np.array([thrust_i, 0.0, 0.0])
        t_reaction_i = np.array([-dir_motors[i] * P.k_T_P * thrust_i, 0.0, 0.0])
        t_moment_i = np.cross(r_motor, f_motor_i)

        f_prop_total += f_motor_i
        t_prop_total += t_reaction_i + t_moment_i

    # Sum all forces and torques
    force = f_prop_total + f_aero_body
    torque = t_aero_body + t_prop_total

    return np.concatenate([force, torque])


class Ts06ForceModel(SimComponentBase):
    def __init__(self):
        super().__init__()
        self.alpha_filtered = None
        self.beta_filtered = None

    def update(self, t_us, paused):
        if paused:
            return self.last_output

        y = self._inputs.get("y")
        u = self._inputs.get("u")
        wind = self._inputs.get("wind")
        P = self._inputs.get("P")

        if y is None or u is None or wind is None or P is None:
            raise ValueError("Ts06ForceModel requires inputs: y, u, wind, P")

        # Compute raw aerodynamic angles at the CG for filtering
        vel = y[7:10]
        wind_b = wind[:3]
        vel_r = vel - wind_b

        v_a = np.linalg.norm(vel_r)
        if v_a == 0:
            v_a = 1e-5

        alpha_raw = np.arctan2(vel_r[2], vel_r[0])
        beta_raw = np.arcsin(np.clip(vel_r[1] / v_a, -1.0, 1.0))

        # Low-pass filter logic with variable dt support
        if self._last_t_us is None or self.alpha_filtered is None:
            self.alpha_filtered = alpha_raw
            self.beta_filtered = beta_raw
        else:
            dt = float(t_us - self._last_t_us) / 1e6
            if dt > 0:
                tau_alpha = getattr(P, "tau_alpha", 0.05)
                tau_beta = getattr(P, "tau_beta", 0.05)
                
                alpha_f = dt / (tau_alpha + dt) if tau_alpha > 0 else 1.0
                beta_f = dt / (tau_beta + dt) if tau_beta > 0 else 1.0
                
                self.alpha_filtered = alpha_f * alpha_raw + (1.0 - alpha_f) * self.alpha_filtered
                self.beta_filtered = beta_f * beta_raw + (1.0 - beta_f) * self.beta_filtered

        t_s = float(t_us) / 1e6
        self.last_output = forces(t_s, y, u, wind, P, 
                                  alpha_override=self.alpha_filtered, 
                                  beta_override=self.beta_filtered)
        self._last_t_us = int(t_us)
        return self.last_output

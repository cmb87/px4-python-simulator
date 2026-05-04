from pathlib import Path
from typing import cast
import logging

import numpy as np

from vehicles.base_component import SimComponentBase
from .aero_lut import AeroLookupTable, OutOfBoundsMode


logger = logging.getLogger(__name__)


def _relative_air_data(y, wind):
    vel_body = np.asarray(y[7:10], dtype=float)
    wind_body = np.asarray(wind[:3], dtype=float)
    vel_rel = vel_body - wind_body

    u_r, v_r, w_r = vel_rel
    v_air = max(float(np.linalg.norm(vel_rel)), 1.0e-5)
    alpha = np.arctan2(w_r, u_r)
    beta = np.arcsin(np.clip(v_r / v_air, -1.0, 1.0))

    return u_r, v_air, alpha, beta


def _controls_to_fwlm_inputs(u, P):
    controls = np.asarray(u, dtype=float)
    if controls.shape[0] < 3:
        controls = np.pad(controls, (0, 3 - controls.shape[0]))

    throttle = float(np.clip(controls[0], 0.0, 1.0))
    delta21_deg =  float(np.clip(controls[1], -1.0, 1.0) * float(P.delta21_max_deg))
    delta22_deg =  -float(np.clip(controls[2], -1.0, 1.0) * float(P.delta22_max_deg))
    return throttle, delta21_deg, delta22_deg


class FWLMAeroLUTForceModel(SimComponentBase):
    def __init__(self):
        super().__init__()
        self._aero_lut: AeroLookupTable | None = None
        self._debug_counter = 0

    def _load_aero_lut(self, P):
        if self._aero_lut is not None:
            return

        mode_raw = str(getattr(P, "aero_oob_mode", "clamp"))
        if mode_raw not in {"clamp", "raise", "extrapolate"}:
            raise ValueError(f"Invalid aero_oob_mode '{mode_raw}'")
        mode = cast(OutOfBoundsMode, mode_raw)

        table_path = Path(__file__).with_name(str(P.aero_table_filename))
        self._aero_lut = AeroLookupTable.from_csv(
            csv_path=table_path,
            s_ref=float(P.aero_s_ref),
            b_ref=float(P.aero_b_ref),
            c_ref=float(P.aero_c_ref),
            out_of_bounds=mode,
            force_sign=tuple(P.aero_force_sign),
            moment_sign=tuple(P.aero_moment_sign),
        )

    def update(self, t_us, paused):
        if paused:
            return self.last_output

        y = self._inputs.get("y")
        u = self._inputs.get("u")
        wind = self._inputs.get("wind")
        P = self._inputs.get("P")
        if y is None or u is None or wind is None or P is None:
            raise ValueError("FWLMAeroLUTForceModel requires inputs: y, u, wind, P")

        self._load_aero_lut(P)
        if self._aero_lut is None:
            raise ValueError("FWLMAeroLUTForceModel failed to initialize aerodynamic lookup table")
        lut = self._aero_lut

        _, v_air, alpha, beta = _relative_air_data(y=y, wind=wind)
        _, delta21_deg, delta22_deg = _controls_to_fwlm_inputs(u=u, P=P)

        self._debug_counter += 1
        debug_stride = int(getattr(P, "debug_alpha_beta_stride", 1))
        if bool(getattr(P, "debug_alpha_beta", False)) and (self._debug_counter % max(debug_stride, 1) == 0):
            logger.info(
                "FWLM aero angles: alpha=%.3f deg, beta=%.3f deg, Va=%.3f m/s",
                float(np.rad2deg(alpha)),
                float(np.rad2deg(beta)),
                float(v_air),
            )

        coeffs = lut.eval_coeffs(
            v_air=v_air,
            alpha_deg=float(np.rad2deg(alpha)),
            beta_deg=float(np.rad2deg(beta)),
            delta21_deg=delta21_deg,
            delta22_deg=delta22_deg,
            rho=float(P.rho),
            h_m=max(float(y[2]) * -1.0, 0.0),
        )

        body_rates = np.asarray(y[10:13], dtype=float)
        wind_rates = np.asarray(wind[3:6], dtype=float)
        p, q, r = body_rates + wind_rates

        b_ref = max(float(lut.b_ref), 1.0e-6)
        c_ref = max(float(lut.c_ref), 1.0e-6)
        p_hat = p * b_ref / (2.0 * v_air)
        q_hat = q * c_ref / (2.0 * v_air)
        r_hat = r * b_ref / (2.0 * v_air)

        cf_x = float(coeffs["cf_x"]) + float(coeffs.get("cf_xp", 0.0)) * p_hat
        cf_y = float(coeffs["cf_y"]) + float(coeffs.get("cf_yp", 0.0)) * p_hat + float(coeffs.get("cf_yr", 0.0)) * r_hat
        cf_z = float(coeffs["cf_z"]) + float(coeffs.get("cf_zq", 0.0)) * q_hat

        cm_x = float(coeffs["cm_x"])
        cm_y = float(coeffs["cm_y"])
        cm_z = float(coeffs["cm_z"]) + float(coeffs.get("cm_zr", 0.0)) * r_hat

        qbar = 0.5 * float(P.rho) * v_air * v_air

        f_aero = qbar * float(lut.s_ref) * np.array([cf_x, cf_y, cf_z], dtype=float)
        f_aero = f_aero * np.asarray(lut.force_sign, dtype=float)

        m_aero = qbar * float(lut.s_ref) * np.array(
            [float(lut.b_ref) * cm_x, float(lut.c_ref) * cm_y, float(lut.b_ref) * cm_z],
            dtype=float,
        )
        m_aero = m_aero * np.asarray(lut.moment_sign, dtype=float)
        m_aero = m_aero * float(getattr(P, "aero_moment_scale", 1.0))

        self.last_output = np.concatenate([f_aero, m_aero])
        self._last_t_us = int(t_us)
        return self.last_output


class FWLMMotorForceModel(SimComponentBase):
    def __init__(self):
        super().__init__()
        self._omega_rad_s = 0.0

    @staticmethod
    def _motor_step(omega, omega_cmd, tau, dt):
        if dt <= 0.0:
            return omega
        alpha = 1.0 - np.exp(-dt / max(float(tau), 1.0e-6))
        return float(omega + (omega_cmd - omega) * alpha)

    def _propulsion(self, u_r, P):
        omega = max(self._omega_rad_s, 0.0)
        n_hz = omega / (2.0 * np.pi)
        d_prop = float(P.prop_diameter_m)
        rho = float(P.rho)

        v_axial = max(float(u_r), 0.0)
        if n_hz <= 1.0e-6 or d_prop <= 1.0e-6:
            return np.zeros(3), np.zeros(3)

        j = v_axial / (n_hz * d_prop)
        c_t = float(P.ct0) - float(P.ct1) * j - float(P.ct2) * (j**2)
        c_t = max(c_t, float(getattr(P, "ct_min", -0.03)))
        c_q = max(float(P.cq0) - float(P.cq1) * j, 0.0)

        thrust = rho * (n_hz**2) * (d_prop**4) * c_t
        q_prop = rho * (n_hz**2) * (d_prop**5) * c_q

        force_prop = np.array([thrust, 0.0, 0.0], dtype=float)
        torque_prop_x = 0.0
        if bool(getattr(P, "enable_prop_reaction_torque", False)):
            torque_prop_x = -float(P.prop_spin_sign) * q_prop * float(getattr(P, "prop_reaction_torque_scale", 1.0))
        torque_prop = np.array([torque_prop_x, 0.0, 0.0], dtype=float)
        return force_prop, torque_prop

    def update(self, t_us, paused):
        if paused:
            return self.last_output

        y = self._inputs.get("y")
        u = self._inputs.get("u")
        wind = self._inputs.get("wind")
        P = self._inputs.get("P")
        if y is None or u is None or wind is None or P is None:
            raise ValueError("FWLMMotorForceModel requires inputs: y, u, wind, P")

        u_r, _, _, _ = _relative_air_data(y=y, wind=wind)
        throttle, _, _ = _controls_to_fwlm_inputs(u=u, P=P)

        dt = self._compute_dt_s(t_us)
        omega_cmd = throttle * float(P.motor_omega_max_rad_s)
        self._omega_rad_s = self._motor_step(self._omega_rad_s, omega_cmd, P.motor_time_constant, dt)

        f_prop, m_prop = self._propulsion(u_r=u_r, P=P)
        self.last_output = np.concatenate([f_prop, m_prop])
        self._last_t_us = int(t_us)
        return self.last_output

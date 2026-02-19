import logging
import numpy as np
from Rzyx import Rzyx
from base_component import SimComponentBase


logger = logging.getLogger(__name__)



def forces(t, y, u, wind, P):

    vel = y[7:10]
    rate = y[10:13]

    p = rate[0] + wind[3]
    q = rate[1] + wind[4]
    r = rate[2] + wind[5]
    

    leftElevon =  u[1]
    rightElevon = u[2]

    throttle = u[0]

    elevator = -0.5 * (leftElevon + rightElevon) * 40.0 * 3.14/180.0
    aileron = 0.5 * ( -leftElevon + rightElevon) * 40.0 * 3.14/180.0
    rudder = 0.0 #; //-(control.size() > 3 ? control.get(3) : 0.0) * 10.0 * deg2rad;


    # Relative velocity
    wind_b = wind[:3]
    vel_r = vel - wind_b
    u_r, v_r, w_r = vel_r

    # Airspeed, alpha, beta
    Va = np.linalg.norm(vel_r)
    if Va == 0:
        Va = 1e-5

    alpha = np.arctan2(w_r, u_r)
    beta = np.arcsin(v_r / Va)

    # Longitudinal forces
    C_L_alpha = P.C_L_0 + P.C_L_alpha * alpha
    f_lift_s = 0.5 * P.rho * Va**2 * P.S_wing * (
        C_L_alpha + P.C_L_q * P.c / (2 * Va) * q + P.C_L_delta_e * elevator
    )

    C_D_alpha = P.C_D_0 + P.C_D_alpha1 * alpha + P.C_D_alpha2 * alpha**2
    C_D_beta = P.C_D_beta1 * beta + P.C_D_beta2 * beta**2
    
    f_drag_s = 0.5 * P.rho * Va**2 * P.S_wing * (
        C_D_alpha + C_D_beta + P.C_D_q * P.c / (2 * Va) * q + P.C_D_delta_e * elevator**2
    )

    m_a = P.C_m_0 + P.C_m_alpha * alpha
    m = 0.5 * P.rho * Va**2 * P.S_wing * P.c * (
        m_a + P.C_m_q * P.c / (2 * Va) * q + P.C_m_delta_e * elevator
    )

    # Lateral forces and moments
    f_y = 0.5 * P.rho * Va**2 * P.S_wing * (
        P.C_Y_0 + P.C_Y_beta * beta + P.C_Y_p * P.b / (2 * Va) * p + 
        P.C_Y_r * P.b / (2 * Va) * r + P.C_Y_delta_a * aileron + P.C_Y_delta_r * rudder
    )

    l = 0.5 * P.rho * Va**2 * P.b * P.S_wing * (
        P.C_l_0 + P.C_l_beta * beta + P.C_l_p * P.b / (2 * Va) * p +
        P.C_l_r * P.b / (2 * Va) * r + P.C_l_delta_a * aileron + P.C_l_delta_r * rudder
    )

    n = 0.5 * P.rho * Va**2 * P.b * P.S_wing * (
        P.C_n_0 + P.C_n_beta * beta + P.C_n_p * P.b / (2 * Va) * p +
        P.C_n_r * P.b / (2 * Va) * r + P.C_n_delta_a * aileron + P.C_n_delta_r * rudder
    )

    # Sum aerodynamic forces in body frame
    F_aero = Rzyx(0, alpha, beta).T @ np.array([-f_drag_s, f_y, -f_lift_s])
    T_aero = np.array([l, m, n])

    # Propulsion force and torque
    Vd = Va + throttle * (P.k_motor - Va)
    F_prop = np.array([0.5 * P.rho * P.S_prop * P.C_prop * Vd * (Vd - Va), 0, 0])
    T_prop = np.array([-P.k_T_P * (P.k_Omega * throttle)**2, 0, 0])

    # Total forces and torques
    Force = F_prop + F_aero
    Torque = T_aero + T_prop

    # ToDo: Effect of r_cg missing in propeller force  and aero force!

    return np.concatenate([Force, Torque])


class WingX8ForceModel(SimComponentBase):
    def update(self, t_us, paused):
        if paused:
            return self.last_output

        y = self._inputs.get("y")
        u = self._inputs.get("u")
        wind = self._inputs.get("wind")
        P = self._inputs.get("P")

        if y is None or u is None or wind is None or P is None:
            raise ValueError("WingX8ForceModel requires inputs: y, u, wind, P")

        t_s = float(t_us) / 1e6
        self.last_output = forces(t_s, y, u, wind, P)
        self._last_t_us = int(t_us)
        return self.last_output


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    from model.x8 import X8Parameters

    P = X8Parameters()
    t = 0.0
    y = np.zeros(12)  # Example state vector
    u = np.zeros(4)  # Example control inputs
    wind = np.zeros(6)  # Example wind vector

    
    out = forces(t, y, u, wind, P)

    logger.info("%s", out)

import numpy as np
from Rzyx import Rzyx
from parameters import Parameters
from quaternion import Quaternion


def railForces(t, y, u, wind, P):

    pos = y[0:3]            # ned
    quaternions = y[3:7]
    vel = y[7:10]           # in Body frame
    Omega = y[10:13]        # Rates

    rel_pos = pos - P.rail_start_ned
    rail_dist = np.dot(rel_pos, P.rail_dir_ned)


    rail_pull_max = 10.0 # kg

    Force = rail_pull_max *P.gravity * (P.rail_length - rail_dist) * P.rail_dir_ned

    Torque = np.zeros(3)

    return np.concatenate([Force, Torque])





def forces(t, y, u, wind, P):

    quats = y[3:7]


    vel = y[7:10]
    rate = y[10:13]

    phi, theta, psi = Quaternion.quat2Euler(quats)

    p = rate[0] + wind[3]
    q = rate[1] + wind[4]
    r = rate[2] + wind[5]
    
    elevator, aileron, rudder, throttle = u # In degrees, throttle is normalized [0, 1]
    
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

    # Gravitational force in body frame
    fg_N = np.array([0, 0, P.mass * P.gravity])
    fg_b = Rzyx(phi, theta, psi).T @ fg_N

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
    Force = F_prop + fg_b + F_aero
    Torque = T_aero + T_prop

    # ToDo: Effect of r_cg missing in propeller force  and aero force!

    return np.concatenate([Force, Torque])


if __name__ == "__main__":

    P = Parameters()
    t = 0.0
    y = np.zeros(12)  # Example state vector
    u = np.zeros(4)  # Example control inputs
    wind = np.zeros(6)  # Example wind vector

    
    out = forces(t, y, u, wind, P)

    print(out)
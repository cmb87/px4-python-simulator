import os

import numpy as np
from ..sim_utils import parse_env_float


class Ts06Parameters:
    def __init__(self):
        self.r_cg = [0, 0, 0]
        self.mass = 3.5

        # From Andrew
        self.Jx = 20167947.03 * 1E-9 # 20167947.03 * 1E-9
        self.Jy = 97052404.49 * 1E-9
        self.Jz = 107868350.68 * 1E-9
        self.Jxz = 0.0
        self.I_cg = np.array(
            [
                [self.Jx, 0, -self.Jxz],
                [0, self.Jy, 0],
                [-self.Jxz, 0, self.Jz],
            ]
        )
        self.I_cg_inv = np.linalg.inv(self.I_cg)

        ## Wing Properties
        # FRD distances rel to CG (Can be overridden via environment variables)
        self.x_wing = -0.033
        self.y_wing =  0.0
        self.z_wing = -0.04

        self.rho = 1.225
        self.span_wing = 0.54/2# half wing spand
        self.chord_wing = 0.08
        self.area_wing = self.span_wing*self.chord_wing
        self.aeroTable_wing = "naca2408.dat"


        self.x_motors = [-0.235, -0.235, -0.235, -0.235]
        self.y_motors = [0.205/2, -0.205/2, -0.205/2, 0.205/2]  # [TopRight, BottomLeft, TopLeft, BottomRight]
        self.z_motors = [-0.205/2, 0.205/2, -0.205/2, 0.205/2]  # [TopRight, BottomLeft, TopLeft, BottomRight]
        self.dir_motors = [1, 1, -1, -1]                        # 1 = CCW, -1 = CW

        self.C_l_p = -1.5  # roll damping
        self.C_m_q = -8.0  # pitch damping
        self.C_n_r = -4.2  # yaw damping

       # self.C_l_p = -0.2  # roll damping
       # self.C_m_q = -8.2  # pitch damping
       # self.C_n_r = -0.2  # yaw damping

        self.S_prop = 0.10178760197630929

        self.k_motor = 50
        self.k_T_P = 0.02 # Torque-to-thrust ratio (Newton-meters of torque per Newton of thrust)
        self.k_Omega = 0.001/2
        self.C_prop = 1



   
        ## Low Pass Filter Time Constants (for alpha/beta smoothing)
        self.tau_alpha = parse_env_float("SIM_TS06_TAU_ALPHA", 0.02)
        self.tau_beta = parse_env_float("SIM_TS06_TAU_BETA", 0.02)

        self.gravity = 9.81
        self.magnetic_ned = np.array([0.21523, 0.01, 0.43])

        self.has_airspeed_sensor = True
        self.pitot_axis_body = np.array([1.0, 0.0, 0.0], dtype=float)

        self.gps_origin = {
            "lat": float(os.getenv("SIM_GPS_LAT", "48.35386539065191")),
            "lon": float(os.getenv("SIM_GPS_LON", "11.78159133408772")),
            "alt": float(os.getenv("SIM_GPS_ALT", "447.0")),
        }

        self.rail_launch_enabled = True
        self.rail_dir_ned = np.array([np.cos(np.deg2rad(45)), 0.0, -np.sin(np.deg2rad(45))], dtype=float)
        self.rail_start_ned = np.asarray([0.0, 0.0, 0.0], dtype=float)
        self.left_rail = False
        self.rail_length = 2.01
        self.rail_pull_max = 20.1

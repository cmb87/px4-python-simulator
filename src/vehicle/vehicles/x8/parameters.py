import os

import numpy as np


class X8Parameters:
    def __init__(self):
        self.r_cg = [0, 0, 0]
        self.mass = 4.364

        self.rho = 1.225
        self.Jx = 1.229
        self.Jy = 0.1702
        self.Jz = 0.8808
        self.Jxz = 0.9343 * 0.001

        self.I_cg = np.array(
            [
                [self.Jx, 0, -self.Jxz],
                [0, self.Jy, 0],
                [-self.Jxz, 0, self.Jz],
            ]
        )
        self.I_cg_inv = np.linalg.inv(self.I_cg)

        self.S_wing = 0.75
        self.b = 2.1
        self.c = 0.35714285714285715
        self.S_prop = 0.10178760197630929

        self.k_motor = 40
        self.k_T_P = 0
        self.k_Omega = 0
        self.C_prop = 1

        self.C_L_alpha = 4.0203282440006793
        self.C_L_0 = 0.08673556671610734
        self.C_L_q = 3.87
        self.C_L_delta_e = 0.27807362017347131

        self.C_D_delta_e = 0.063347396781802318
        self.C_D_alpha2 = 1.0554699867680841
        self.C_D_alpha1 = 0.079091463157662967
        self.C_D_0 = 0.01970001181915082
        self.C_D_beta2 = 0.14781193079241584
        self.C_D_beta1 = -0.0058429803454153884
        self.C_D_q = 0

        self.C_m_alpha = -0.4629
        self.C_m_0 = 0.02275
        self.C_m_q = -1.3012370370370372
        self.C_m_delta_e = -0.2292

        self.C_Y_beta = -0.22387215700254048
        self.C_Y_0 = 0
        self.C_Y_p = -0.13735505263157893
        self.C_Y_r = 0.083868768421052634
        self.C_Y_delta_a = 0.043276402502774876
        self.C_Y_delta_r = 0

        self.C_l_beta = -0.084896286396624165
        self.C_l_0 = 0
        self.C_l_p = -0.40419799999999995
        self.C_l_r = 0.055520599999999996
        self.C_l_delta_a = 0.12018814125782745
        self.C_l_delta_r = 0

        self.C_n_beta = 0.0283
        self.C_n_0 = 0
        self.C_n_p = 0.0043655115789473682
        self.C_n_r = -0.072000000000000008
        self.C_n_delta_a = -0.00339
        self.C_n_delta_r = 0

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
        self.rail_length = 2.0
        self.rail_pull_max = 10.0

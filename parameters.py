import numpy as np


def Smtrx(v):
    """Return the skew-symmetric matrix of a 3-element vector."""
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

class Parameters:
    def __init__(self):

        self.rho = 1.225  # Air density at sea level in kg/m^3

        self.r_cg = [0, 0, 0]
        self.mass = 3.364
        self.Jx = 1.229
        self.Jy = 0.1702
        self.Jz = 0.8808
        self.Jxz = 0.9343

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

        self.gravity = 9.81  # Make sure to define gravity if needed in the rest of your code

        # Inertia matrix
        self.I_cg = np.array([
            [self.Jx, 0, -self.Jxz],
            [0, self.Jy, 0],
            [-self.Jxz, 0, self.Jz]
        ])

        # Inverse inertia matrix
        self.I_cg_inv = np.linalg.inv(self.I_cg)
        
        # Mass matrix
        Sm_r_cg = Smtrx(self.r_cg)
        self.M_rb = np.block([
            [np.eye(3) * self.mass, -self.mass * Sm_r_cg],
            [self.mass * Sm_r_cg, self.I_cg]
        ])

if __name__ == "__main__":
    P = Parameters()
    print(P.mass)
    print(P.C_L_alpha)

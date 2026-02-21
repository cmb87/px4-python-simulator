import numpy as np

from forces.vtol_ts04 import TS04ForceModel, TS04BlendedPassiveAeroForceModel


class TS04Parameters:
    def __init__(self):
        self.r_cg = [0, 0, 0]
        self.mass = 1.6

        self.rho = 1.225
        self.Jx = 0.029125
        self.Jy = 1.3*0.029125
        self.Jz = 0.055225
        self.Jxz = 0.0

        self.I_cg = np.array(
            [
                [self.Jx, 0.0, -self.Jxz],
                [0.0, self.Jy, 0.0],
                [-self.Jxz, 0.0, self.Jz],
            ]
        )
        self.I_cg_inv = np.linalg.inv(self.I_cg)

        self.gravity = 9.81

        self.wing_angle = 20.0  # deg
        self.arm_length = 0.52
        self.motor_full_thrust = 10.0
        self.motor_full_torque = 0.22
        self.motor_time_constant = 0.06
        self.motor_max_omega = 900.0
        self.rotor_polar_inertia = 5.0e-5

        c = np.cos(np.deg2rad(self.wing_angle))
        s = np.sin(np.deg2rad(self.wing_angle))
        arm = self.arm_length

        # Motor order: FrontRight, RearLeft, FrontLeft, RearRight
        # Body frame is FRD. Original XY arm layout is remapped to YZ.
        self.motor_positions_body_m = np.array(
            [
                [0.0, arm * c, arm * s],
                [0.0, -arm * c, -arm * s],
                [0.0, -arm * c, arm * s],
                [0.0, arm * c, -arm * s],
            ],
            dtype=float,
        )

        # Motor thrust axes in body frame (FRD), one vector per motor.
        self.motor_thrust_directions_body = np.array(
            [
                [1.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
            ],
            dtype=float,
        )

        # Rotor spin direction sign (+1/-1), one value per motor.
        self.motor_spin_sign = np.array([-1.0, -1.0, 1.0, 1.0], dtype=float)

        self.sphere_cd = 0.47
        self.sphere_area = 0.04

        self.S_wing = 0.35
        self.b = 1.20
        self.c = self.S_wing / self.b

        self.C_L_alpha = 4.0203282440006793
        self.C_L_0 = 0.08673556671610734
        self.C_L_q = 3.87

        self.C_D_alpha2 = 1.0554699867680841
        self.C_D_alpha1 = 0.07909146315766297
        self.C_D_0 = 0.01970001181915082
        self.C_D_beta2 = 0.14781193079241584
        self.C_D_beta1 = -0.0058429803454153884
        self.C_D_q = 0.0

        self.C_m_alpha = -0.4629
        self.C_m_0 = 0.02275
        self.C_m_q = -1.3012370370370372

        self.C_Y_beta = -0.22387215700254048
        self.C_Y_0 = 0.0
        self.C_Y_p = -0.13735505263157893
        self.C_Y_r = 0.08386876842105263

        self.C_l_beta = -0.08489628639662417
        self.C_l_0 = 0.0
        self.C_l_p = -0.40419799999999995
        self.C_l_r = 0.0555206

        self.C_n_beta = 0.0283
        self.C_n_0 = 0.0
        self.C_n_p = 0.0043655115789473682
        self.C_n_r = -0.07200000000000001

        self.passive_wing_force_scale = 0.45
        self.passive_wing_moment_scale = 0.35
        self.blend_tilt_sphere_deg = 65.0
        self.blend_tilt_wing_deg = 25.0
        self.passive_wing_stall_alpha_deg = 30.0

        self.magnetic_ned = np.array([0.21523, 0.01, 0.43])

        self.accel_bias = 0 * np.array([0.01, -0.01, 0.02])
        self.gyro_bias = 0 * np.array([0.005, -0.003, 0.002])
        self.mag_bias = 0 * np.array([0.001, 0.001, 0.001])
        self.baro_bias = 0.5

        self.accel_noise_std = 0.00000000001
        self.gyro_noise_std = 0.00000000001
        self.mag_noise_std = 0.000001
        self.baro_noise_std = 0.01
        self.gps_pos_noise_std = 0.0001 * np.array([0.01, 0.01, 0.01])
        self.gps_vel_noise_std = 0.0001 * np.array([0.01, 0.01, 0.01])

        self.gps_origin = {
            "lat": 47.397742,
            "lon": 8.545594,
            "alt": 470.0,
        }


def build_force_models():
    return [TS04ForceModel(), TS04BlendedPassiveAeroForceModel()]

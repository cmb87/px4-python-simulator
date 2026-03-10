import os

import numpy as np


class FWLMParameters:
    def __init__(self):
        self.r_cg = [0, 0, 0]

        # Scale inertia from X8 baseline to a 2 kg airframe.
        self.mass = 2.0
        self.rho = 1.225
        x8_mass = 4.364
        inertia_scale = self.mass / x8_mass
        self.Jx = 1.01 * inertia_scale
        self.Jy = 1.01 * inertia_scale
        self.Jz = 1.01 * inertia_scale
        self.Jxz = 0.0*(0.9343 * 0.001) * inertia_scale

        self.I_cg = np.array(
            [
                [self.Jx, 0.0, -self.Jxz],
                [0.0, self.Jy, 0.0],
                [-self.Jxz, 0.0, self.Jz],
            ]
        )
        self.I_cg_inv = np.linalg.inv(self.I_cg)

        self.gravity = 9.81

        # Aerodynamic LUT configuration.
        self.aero_table_filename = "aero_lookup.csv"
        self.aero_s_ref = 0.0476
        self.aero_b_ref = 0.07
        self.aero_c_ref = 0.07
        self.aero_oob_mode = "clamp"
        self.aero_force_sign = (-1.0, 1.0, -1.0)
        self.aero_moment_sign = (-1.0, 1.0, -1.0) # please check
        self.debug_alpha_beta = False
        self.debug_alpha_beta_stride = 1

        # Inputs are [throttle, delta21, delta22], with surface controls in [-1, 1].
        self.delta21_max_deg = 30.0
        self.delta22_max_deg = 30.0

        # 3 kg class single-prop motor/prop model (airspeed dependent via J).
        self.motor_time_constant = 0.08
        self.motor_omega_max_rad_s = 900.0
        self.prop_diameter_m = 0.33
        self.prop_spin_sign = 1.0
        self.ct0 = 0.42
        self.ct1 = 0.10
        self.ct2 = 0.04
        self.ct_min = -0.03
        self.cq0 = 0.012
        self.cq1 = 0.010
        self.enable_prop_reaction_torque = False
        self.prop_reaction_torque_scale = 0.1

        # LUT moments are aggressive at high speed for this simplified model.
        self.aero_moment_scale = 0.25

        self.magnetic_ned = np.array([0.21523, 0.01, 0.43])

        self.accel_bias = 0 * np.array([0.01, -0.01, 0.02])
        self.gyro_bias = 0 * np.array([0.005, -0.003, 0.002])
        self.mag_bias = 0 * np.array([0.001, 0.001, 0.001])
        self.baro_bias = 0.5

        self.accel_noise_std = 1.0e-11
        self.gyro_noise_std = 1.0e-11
        self.mag_noise_std = 1.0e-6
        self.baro_noise_std = 0.01
        self.diff_pressure_noise_std = 0.002
        self.has_airspeed_sensor = True
        self.pitot_axis_body = np.array([1.0, 0.0, 0.0], dtype=float)
        self.diff_pressure_lpf_tau_s = 0.08
        self.gps_pos_noise_std = 0.0001 * np.array([0.01, 0.01, 0.01])
        self.gps_vel_noise_std = 0.0001 * np.array([0.01, 0.01, 0.01])

        self.gps_origin = {
            "lat": float(os.getenv("SIM_GPS_LAT", "47.397742")),
            "lon": float(os.getenv("SIM_GPS_LON", "8.545594")),
            "alt": float(os.getenv("SIM_GPS_ALT", "470.0")),
        }

        self.rail_launch_enabled = True
        self.rail_dir_ned = np.array([np.cos(np.deg2rad(45.0)), 0.0, -np.sin(np.deg2rad(45.0))], dtype=float)
        self.rail_start_ned = np.asarray([0.0, 0.0, 0.0], dtype=float)
        self.left_rail = False
        self.rail_length = 2.0
        self.rail_pull_max = 100.0

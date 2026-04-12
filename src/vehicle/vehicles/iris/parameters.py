import os

import numpy as np


class IrisParameters:
    def __init__(self):
        self.r_cg = [0, 0, 0]
        self.mass = 1.5

        self.rho = 1.225
        self.Jx = 0.029125
        self.Jy = 0.029125
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

        self.arm_length = 0.225
        self.motor_full_thrust = 12.0
        self.motor_full_torque = 0.08
        self.motor_time_constant = 0.06
        self.motor_max_omega = 900.0
        self.rotor_polar_inertia = 6.0e-5

        self.sphere_cd = 0.47
        self.sphere_area = 0.04

        self.magnetic_ned = np.array([0.21523, 0.01, 0.43])

        self.accel_bias = 0 * np.array([0.01, -0.01, 0.02])
        self.gyro_bias = 0 * np.array([0.005, -0.003, 0.002])
        self.mag_bias = 0 * np.array([0.001, 0.001, 0.001])
        self.baro_bias = 0.5

        self.accel_noise_std = 0.00000000001
        self.gyro_noise_std = 0.00000000001
        self.mag_noise_std = 0.000001
        self.baro_noise_std = 0.01
        self.diff_pressure_noise_std = 0.002
        self.has_airspeed_sensor = False
        self.pitot_axis_body = np.array([1.0, 0.0, 0.0], dtype=float)
        self.diff_pressure_lpf_tau_s = 0.08
        self.gps_pos_noise_std = 0.0001 * np.array([0.01, 0.01, 0.01])
        self.gps_vel_noise_std = 0.0001 * np.array([0.01, 0.01, 0.01])

        self.gps_origin = {
            "lat": float(os.getenv("SIM_GPS_LAT", "48.35386539065191")),
            "lon": float(os.getenv("SIM_GPS_LON", "11.78159133408772")),
            "alt": float(os.getenv("SIM_GPS_ALT", "447.0")),
        }

        self.rail_launch_enabled = False
        self.rail_dir_ned = np.array([np.cos(np.deg2rad(45)), 0.0, -np.sin(np.deg2rad(45))], dtype=float)
        self.rail_start_ned = np.asarray([0.0, 0.0, 0.0], dtype=float)
        self.left_rail = False
        self.rail_length = 2.0
        self.rail_pull_max = 1.0

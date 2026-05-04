#pragma once

#include <Eigen/Dense>
#include <map>
#include <string>
#include <cmath>
#include "../dynamics/vehicle_parameters.hpp"

namespace iris {

struct IrisParameters : public VehicleParameters {
    IrisParameters() {
        r_cg = Eigen::Vector3d::Zero();
        mass = 1.5;

        rho = 1.225;
        Jx = 0.029125;
        Jy = 0.029125;
        Jz = 0.055225;
        Jxz = 0.0;

        I_cg << Jx, 0.0, -Jxz,
                0.0, Jy, 0.0,
                -Jxz, 0.0, Jz;
        
        I_cg_inv = I_cg.inverse();

        gravity = 9.81;

        arm_length = 0.225;
        motor_full_thrust = 12.0;
        motor_full_torque = 0.08;
        motor_time_constant = 0.06;
        motor_max_omega = 900.0;
        rotor_polar_inertia = 6.0e-5;

        sphere_cd = 0.47;
        sphere_area = 0.04;

        magnetic_ned << 0.21523, 0.01, 0.43;

        has_airspeed_sensor = false;
        pitot_axis_body << 1.0, 0.0, 0.0;

        gps_origin["lat"] = 48.35386539065191;
        gps_origin["lon"] = 11.78159133408772;
        gps_origin["alt"] = 447.0;

        rail_launch_enabled = false;
        double rail_angle_rad = 45.0 * M_PI / 180.0;
        rail_dir_ned << std::cos(rail_angle_rad), 0.0, -std::sin(rail_angle_rad);
        rail_start_ned << 0.0, 0.0, 0.0;
        left_rail = false;
        rail_length = 2.0;
        rail_pull_max = 1.0;
    }

    Eigen::Vector3d r_cg;
    double Jx, Jy, Jz, Jxz;

    double arm_length;
    double motor_full_thrust;
    double motor_full_torque;
    double motor_time_constant;
    double motor_max_omega;
    double rotor_polar_inertia;
    double sphere_cd;
    double sphere_area;
};

} // namespace iris

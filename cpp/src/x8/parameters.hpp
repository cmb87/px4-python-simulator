#pragma once

#include <Eigen/Dense>
#include <map>
#include <string>

struct X8Parameters {
    X8Parameters() {
        r_cg = Eigen::Vector3d::Zero();
        mass = 4.364;

        rho = 1.225;
        Jx = 1.229;
        Jy = 0.1702;
        Jz = 0.8808;
        Jxz = 0.9343 * 0.001;

        I_cg << Jx, 0, -Jxz,
                0, Jy, 0,
                -Jxz, 0, Jz;
        
        I_cg_inv = I_cg.inverse();

        S_wing = 0.75;
        b = 2.1;
        c = 0.35714285714285715;
        S_prop = 0.10178760197630929;

        k_motor = 40;
        k_T_P = 0;
        k_Omega = 0;
        C_prop = 1;

        C_L_alpha = 4.0203282440006793;
        C_L_0 = 0.08673556671610734;
        C_L_q = 3.87;
        C_L_delta_e = 0.27807362017347131;

        C_D_delta_e = 0.063347396781802318;
        C_D_alpha2 = 1.0554699867680841;
        C_D_alpha1 = 0.079091463157662967;
        C_D_0 = 0.01970001181915082;
        C_D_beta2 = 0.14781193079241584;
        C_D_beta1 = -0.0058429803454153884;
        C_D_q = 0;

        C_m_alpha = -0.4629;
        C_m_0 = 0.02275;
        C_m_q = -1.3012370370370372;
        C_m_delta_e = -0.2292;

        C_Y_beta = -0.22387215700254048;
        C_Y_0 = 0;
        C_Y_p = -0.13735505263157893;
        C_Y_r = 0.083868768421052634;
        C_Y_delta_a = 0.043276402502774876;
        C_Y_delta_r = 0;

        C_l_beta = -0.084896286396624165;
        C_l_0 = 0;
        C_l_p = -0.40419799999999995;
        C_l_r = 0.055520599999999996;
        C_l_delta_a = 0.12018814125782745;
        C_l_delta_r = 0;

        C_n_beta = 0.0283;
        C_n_0 = 0;
        C_n_p = 0.0043655115789473682;
        C_n_r = -0.072000000000000008;
        C_n_delta_a = -0.00339;
        C_n_delta_r = 0;

        gravity = 9.81;
        magnetic_ned << 0.21523, 0.01, 0.43;

        has_airspeed_sensor = true;
        pitot_axis_body << 1.0, 0.0, 0.0;

        gps_origin["lat"] = 48.35386539065191;
        gps_origin["lon"] = 11.78159133408772;
        gps_origin["alt"] = 447.0;

        rail_launch_enabled = true;
        double rail_angle_rad = 45.0 * M_PI / 180.0;
        rail_dir_ned << std::cos(rail_angle_rad), 0.0, -std::sin(rail_angle_rad);
        rail_start_ned << 0.0, 0.0, 0.0;
        left_rail = false;
        rail_length = 2.0;
        rail_pull_max = 100.0;
    }

    Eigen::Vector3d r_cg;
    double mass;
    double rho;
    double Jx, Jy, Jz, Jxz;
    Eigen::Matrix3d I_cg;
    Eigen::Matrix3d I_cg_inv;

    double S_wing, b, c, S_prop;
    double k_motor, k_T_P, k_Omega, C_prop;

    double C_L_alpha, C_L_0, C_L_q, C_L_delta_e;
    double C_D_delta_e, C_D_alpha2, C_D_alpha1, C_D_0, C_D_beta2, C_D_beta1, C_D_q;
    double C_m_alpha, C_m_0, C_m_q, C_m_delta_e;
    double C_Y_beta, C_Y_0, C_Y_p, C_Y_r, C_Y_delta_a, C_Y_delta_r;
    double C_l_beta, C_l_0, C_l_p, C_l_r, C_l_delta_a, C_l_delta_r;
    double C_n_beta, C_n_0, C_n_p, C_n_r, C_n_delta_a, C_n_delta_r;

    double gravity;
    Eigen::Vector3d magnetic_ned;

    bool has_airspeed_sensor;
    Eigen::Vector3d pitot_axis_body;

    std::map<std::string, double> gps_origin;

    bool rail_launch_enabled;
    Eigen::Vector3d rail_dir_ned;
    Eigen::Vector3d rail_start_ned;
    bool left_rail;
    double rail_length;
    double rail_pull_max;
};

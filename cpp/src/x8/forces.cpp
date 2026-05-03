#include "forces.hpp"
#include <cmath>
#include <iostream>

namespace x8 {

Eigen::Matrix3d Rzyx(double phi, double theta, double psi) {
    double cphi = std::cos(phi);
    double sphi = std::sin(phi);
    double ctheta = std::cos(theta);
    double stheta = std::sin(theta);
    double cpsi = std::cos(psi);
    double spsi = std::sin(psi);

    Eigen::Matrix3d R;
    R << ctheta * cpsi, ctheta * spsi, -stheta,
         sphi * stheta * cpsi - cphi * spsi, sphi * stheta * spsi + cphi * cpsi, sphi * ctheta,
         cphi * stheta * cpsi + sphi * spsi, cphi * stheta * spsi - sphi * cpsi, cphi * ctheta;
    return R;
}

Eigen::VectorXd forces(double t, const Eigen::VectorXd& y, const Eigen::VectorXd& u, const Eigen::Vector3d& wind, const X8Parameters& P) {
    Eigen::Vector3d vel = y.segment<3>(7);
    Eigen::Vector3d rate = y.segment<3>(10);

    double p = rate[0] + wind[0]; 
    double q = rate[1] + wind[1];
    double r = rate[2] + wind[2];

    double left_elevon = u[1];
    double right_elevon = u[2];
    double throttle = u[0];

    double elevator = -0.5 * (left_elevon + right_elevon) * 40.0 * M_PI / 180.0;
    double aileron = 0.5 * (-left_elevon + right_elevon) * 40.0 * M_PI / 180.0;
    double rudder = 0.0;

    Eigen::Vector3d wind_vel_b = wind.segment<3>(0);
    Eigen::Vector3d vel_r = vel - wind_vel_b;
    double u_r = vel_r[0];
    double v_r = vel_r[1];
    double w_r = vel_r[2];

    double va = vel_r.norm();
    if (va < 1e-5) va = 1e-5;

    double alpha = std::atan2(w_r, u_r);
    double beta = std::asin(v_r / va);

    double c_l_alpha = P.C_L_0 + P.C_L_alpha * alpha;
    double f_lift_s = 0.5 * P.rho * std::pow(va, 2) * P.S_wing * (
        c_l_alpha + P.C_L_q * P.c / (2 * va) * rate[1] + P.C_L_delta_e * elevator
    );

    double c_d_alpha = P.C_D_0 + P.C_D_alpha1 * alpha + P.C_D_alpha2 * std::pow(alpha, 2);
    double c_d_beta = P.C_D_beta1 * beta + P.C_D_beta2 * std::pow(beta, 2);

    double f_drag_s = 0.5 * P.rho * std::pow(va, 2) * P.S_wing * (
        c_d_alpha + c_d_beta + P.C_D_q * P.c / (2 * va) * rate[1] + P.C_D_delta_e * std::pow(elevator, 2)
    );

    double m_a = P.C_m_0 + P.C_m_alpha * alpha;
    double m = 0.5 * P.rho * std::pow(va, 2) * P.S_wing * P.c * (
        m_a + P.C_m_q * P.c / (2 * va) * rate[1] + P.C_m_delta_e * elevator
    );

    double f_y = 0.5 * P.rho * std::pow(va, 2) * P.S_wing * (
        P.C_Y_0 + P.C_Y_beta * beta + P.C_Y_p * P.b / (2 * va) * rate[0] + P.C_Y_r * P.b / (2 * va) * rate[2]
        + P.C_Y_delta_a * aileron + P.C_Y_delta_r * rudder
    );

    double l = 0.5 * P.rho * std::pow(va, 2) * P.b * P.S_wing * (
        P.C_l_0 + P.C_l_beta * beta + P.C_l_p * P.b / (2 * va) * rate[0]
        + P.C_l_r * P.b / (2 * va) * rate[2] + P.C_l_delta_a * aileron + P.C_l_delta_r * rudder
    );

    double n = 0.5 * P.rho * std::pow(va, 2) * P.b * P.S_wing * (
        P.C_n_0 + P.C_n_beta * beta + P.C_n_p * P.b / (2 * va) * rate[0]
        + P.C_n_r * P.b / (2 * va) * rate[2] + P.C_n_delta_a * aileron + P.C_n_delta_r * rudder
    );

    if (std::abs(alpha) > (40.0 * M_PI / 180.0)) {
        f_drag_s = 0.0;
        f_y = 0.0;
        f_lift_s = 0.0;
    }

    Eigen::Matrix3d R_aero_to_body = Rzyx(0, alpha, beta).transpose();
    Eigen::Vector3d f_aero = R_aero_to_body * Eigen::Vector3d(-f_drag_s, f_y, -f_lift_s);
    Eigen::Vector3d t_aero(l, m, n);

    double vd = va + throttle * (P.k_motor - va);
    Eigen::Vector3d f_prop(0.5 * P.rho * P.S_prop * P.C_prop * vd * (vd - va), 0, 0);
    Eigen::Vector3d t_prop(-P.k_T_P * std::pow(P.k_Omega * throttle, 2), 0, 0);

    Eigen::VectorXd out(6);
    out.segment<3>(0) = f_prop + f_aero;
    out.segment<3>(3) = t_aero + t_prop;

    return out;
}

}

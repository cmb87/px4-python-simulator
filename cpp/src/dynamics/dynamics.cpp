#include "dynamics.hpp"
#include <cmath>

namespace dynamics {

Eigen::Matrix4d MOmega(const Eigen::Vector3d& Omega) {
    double p = Omega[0];
    double q = Omega[1];
    double r = Omega[2];
    Eigen::Matrix4d M;
    M << 0, -p, -q, -r,
         p,  0,  r, -q,
         q, -r,  0,  p,
         r,  q, -p,  0;
    return M;
}

Eigen::Matrix3d Mfg_from_quat(const Eigen::Vector4d& q) {
    double qw = q[0];
    double qx = q[1];
    double qy = q[2];
    double qz = q[3];

    Eigen::Matrix3d M;
    M << qw*qw + qx*qx - qy*qy - qz*qz, 2*(qx*qy + qw*qz), 2*(qx*qz - qw*qy),
         2*(qx*qy - qw*qz), qw*qw - qx*qx + qy*qy - qz*qz, 2*(qy*qz + qw*qx),
         2*(qx*qz + qw*qy), 2*(qy*qz - qw*qx), qw*qw - qx*qx - qy*qy + qz*qz;
    return M;
}

Eigen::Vector3d euler_from_quat(const Eigen::Vector4d& q) {
    double a = q[0], b = q[1], c = q[2], d = q[3];
    Eigen::Vector3d euler;
    euler[0] = std::atan2(2 * (c * d + a * b), (a * a - b * b - c * c + d * d));
    euler[1] = std::asin(std::max(-1.0, std::min(1.0, 2 * (a * c - b * d))));
    euler[2] = std::atan2(2 * (b * c + a * d), (a * a + b * b - c * c - d * d));
    return euler;
}

Eigen::VectorXd dynamics_6dof(double t, const Eigen::VectorXd& y, const VehicleParameters& P, const Eigen::VectorXd& tau) {
    Eigen::Vector3d pos = y.segment<3>(0);
    Eigen::Vector4d quat = y.segment<4>(3);
    Eigen::Vector3d vel = y.segment<3>(7); // Body frame
    Eigen::Vector3d Omega = y.segment<3>(10); // Rates

    quat.normalize();

    Eigen::Matrix3d Mfg = Mfg_from_quat(quat);
    Eigen::Matrix3d Mgf = Mfg.transpose();
    Eigen::Matrix4d Momega = MOmega(Omega);

    Eigen::Vector3d vel_ned = Mgf * vel;
    Eigen::Vector3d gravity_body = Mfg * Eigen::Vector3d(0, 0, P.mass * P.gravity);

    Eigen::Vector3d body_force = tau.segment<3>(0) + gravity_body;
    Eigen::Vector3d accel_body = body_force / P.mass - Omega.cross(vel);

    Eigen::Vector3d accel_ned = Mgf * accel_body;

    // Simplified ground contact
    Eigen::Vector3d out_pos_dot = vel_ned;
    Eigen::Vector3d out_accel_body = accel_body;

    if (pos[2] >= 0.0) {
        if (accel_ned[2] > 0.0) {
            accel_ned[2] = 0.0;
            out_accel_body = Mfg * accel_ned;
        }
        if (vel_ned[2] > 0.0) {
            vel_ned[2] = 0.0;
        }
        out_pos_dot[2] = 0.0;
    }

    Eigen::Vector3d aux1 = P.I_cg * Omega;
    Eigen::Vector3d aux2 = tau.segment<3>(3) - Omega.cross(aux1);
    Eigen::Vector3d rot_accel_body = P.I_cg_inv * aux2;

    Eigen::Vector4d quat_dot = 0.5 * Momega * quat;

    Eigen::VectorXd xdot(13);
    xdot.segment<3>(0) = out_pos_dot;
    xdot.segment<4>(3) = quat_dot;
    xdot.segment<3>(7) = out_accel_body;
    xdot.segment<3>(10) = rot_accel_body;

    return xdot;
}

Eigen::VectorXd rail_dynamics(double t, const Eigen::VectorXd& y, VehicleParameters& P, const Eigen::VectorXd& tau) {
    Eigen::Vector4d quat = y.segment<4>(3);
    Eigen::Vector3d vel = y.segment<3>(7);

    quat.normalize();

    Eigen::Matrix3d Mfg = Mfg_from_quat(quat);
    Eigen::Matrix3d Mgf = Mfg.transpose();

    Eigen::Vector3d rail_dir_ned = P.rail_dir_ned.normalized();
    Eigen::Vector3d vel_ned = Mgf * vel;
    double vel_rail_scalar = vel_ned.dot(rail_dir_ned);
    Eigen::Vector3d vel_rail_ned = vel_rail_scalar * rail_dir_ned;

    Eigen::Vector3d gravity_body = Mfg * Eigen::Vector3d(0, 0, P.mass * P.gravity);
    Eigen::Vector3d body_force = tau.segment<3>(0) + gravity_body;
    Eigen::Vector3d accel_body_raw = body_force / P.mass;

    Eigen::Vector3d rail_dir_body = Mfg * rail_dir_ned;
    Eigen::Vector3d accel_body = accel_body_raw.dot(rail_dir_body) * rail_dir_body;

    Eigen::Vector3d pos = y.segment<3>(0);
    Eigen::Vector3d rel_pos = pos - P.rail_start_ned;
    double rail_dist = rel_pos.dot(rail_dir_ned);
    if (rail_dist >= P.rail_length) {
        P.left_rail = true;
    }

    Eigen::VectorXd xdot = Eigen::VectorXd::Zero(13);
    xdot.segment<3>(0) = vel_rail_ned;
    xdot.segment<4>(3).setZero();
    xdot.segment<3>(7) = accel_body;
    xdot.segment<3>(10).setZero();

    return xdot;
}


}

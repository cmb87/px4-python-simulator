#include "forces.hpp"
#include <algorithm>
#include <cmath>

namespace iris {

SimpleMotor::SimpleMotor() : w(0.0), tau(0.08), full_thrust(1.0), full_torque(0.02), control_val(0.0) {}

void SimpleMotor::set_full_thrust(double thrust) { full_thrust = thrust; }
void SimpleMotor::set_full_torque(double torque) { full_torque = torque; }
void SimpleMotor::set_time_constant(double tc) { tau = std::max(tc, 1e-6); }
void SimpleMotor::set_control(double control) {
    control_val = std::max(0.0, std::min(control, 1.0));
}

void SimpleMotor::update(double dt) {
    if (dt > 0.0) {
        double alpha = 1.0 - std::exp(-dt / std::max(tau, 1e-6));
        w += (control_val - w) * alpha;
    }
}

double SimpleMotor::get_thrust() const { return w * full_thrust; }
double SimpleMotor::get_torque() const { return control_val * full_torque; }


IrisQuadForceModel::IrisQuadForceModel() : configured(false) {}

void IrisQuadForceModel::configure_from_parameters(const IrisParameters& P) {
    for (int i = 0; i < 4; ++i) {
        motors[i].set_full_thrust(P.motor_full_thrust);
        motors[i].set_full_torque(P.motor_full_torque);
        motors[i].set_time_constant(P.motor_time_constant);
    }
    configured = true;
}

Eigen::VectorXd IrisQuadForceModel::update(double dt, const Eigen::VectorXd& y, const Eigen::VectorXd& u, const Eigen::Matrix<double, 6, 1>& wind, const IrisParameters& P) {
    if (!configured) {
        configure_from_parameters(P);
    }

    Eigen::Vector3d body_rates = y.segment<3>(10);

    Eigen::VectorXd controls = Eigen::VectorXd::Zero(4);
    for (int i = 0; i < std::min(4, (int)u.size()); ++i) {
        controls[i] = std::max(0.0, std::min(u[i], 1.0));
    }

    Eigen::Vector3d total_force = Eigen::Vector3d::Zero();
    Eigen::Vector3d total_torque = Eigen::Vector3d::Zero();

    double arm = P.arm_length;
    double arm_xy = arm / std::sqrt(2.0);

    Eigen::Matrix<double, 4, 3> rotor_positions;
    rotor_positions << arm_xy, arm_xy, 0.0,
                      -arm_xy, -arm_xy, 0.0,
                       arm_xy, -arm_xy, 0.0,
                      -arm_xy, arm_xy, 0.0;

    double yaw_sign[4] = {1.0, 1.0, -1.0, -1.0};
    double rotor_max_omega = P.motor_max_omega;
    double rotor_polar_inertia = P.rotor_polar_inertia;
    double rotor_angular_momentum_z = 0.0;

    for (int i = 0; i < 4; ++i) {
        motors[i].set_control(controls[i]);
        motors[i].update(dt);

        double thrust = motors[i].get_thrust();
        double reaction_torque = motors[i].get_torque();

        Eigen::Vector3d force_i(0.0, 0.0, -thrust);
        Eigen::Vector3d pos_i = rotor_positions.row(i).transpose();
        Eigen::Vector3d torque_from_arm = pos_i.cross(force_i);
        Eigen::Vector3d torque_i = torque_from_arm + Eigen::Vector3d(0.0, 0.0, yaw_sign[i] * reaction_torque);

        total_force += force_i;
        total_torque += torque_i;
        rotor_angular_momentum_z += yaw_sign[i] * rotor_polar_inertia * (motors[i].w * rotor_max_omega);
    }

    Eigen::Vector3d rotor_angular_momentum(0.0, 0.0, rotor_angular_momentum_z);
    Eigen::Vector3d gyroscopic_moment = -body_rates.cross(rotor_angular_momentum);
    total_torque += gyroscopic_moment;

    Eigen::VectorXd out(6);
    out.segment<3>(0) = total_force;
    out.segment<3>(3) = total_torque;
    return out;
}

} // namespace iris

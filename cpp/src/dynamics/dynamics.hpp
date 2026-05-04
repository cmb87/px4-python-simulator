#pragma once

#include <Eigen/Dense>
#include "vehicle_parameters.hpp"

namespace dynamics {

Eigen::Matrix4d MOmega(const Eigen::Vector3d& Omega);
Eigen::Matrix3d Mfg_from_quat(const Eigen::Vector4d& q);
Eigen::Vector3d euler_from_quat(const Eigen::Vector4d& q);
Eigen::VectorXd dynamics_6dof(double t, const Eigen::VectorXd& y, const VehicleParameters& P, const Eigen::VectorXd& tau);
Eigen::VectorXd rail_dynamics(double t, const Eigen::VectorXd& y, VehicleParameters& P, const Eigen::VectorXd& tau);


struct State {
    Eigen::VectorXd y;
    State() {
        y = Eigen::VectorXd::Zero(13);
        y[3] = 1.0; // Quaternion w
    }
};

class IntegratorRK4 {
public:
    typedef std::function<Eigen::VectorXd(double, const Eigen::VectorXd&)> DerivativeFunc;

    static Eigen::VectorXd step(double t, double dt, const Eigen::VectorXd& y, DerivativeFunc f) {
        Eigen::VectorXd k1 = f(t, y);
        Eigen::VectorXd k2 = f(t + 0.5 * dt, y + 0.5 * dt * k1);
        Eigen::VectorXd k3 = f(t + 0.5 * dt, y + 0.5 * dt * k2);
        Eigen::VectorXd k4 = f(t + dt, y + dt * k3);
        return y + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    }
};

class IntegratorEuler {
public:
    typedef std::function<Eigen::VectorXd(double, const Eigen::VectorXd&)> DerivativeFunc;

    static Eigen::VectorXd step(double t, double dt, const Eigen::VectorXd& y, DerivativeFunc f) {
        return y + dt * f(t, y);
    }
};

}

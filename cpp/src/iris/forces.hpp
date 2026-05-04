#pragma once

#include <Eigen/Dense>
#include "parameters.hpp"

namespace iris {

class SimpleMotor {
public:
    SimpleMotor();

    void set_full_thrust(double thrust);
    void set_full_torque(double torque);
    void set_time_constant(double tc);
    void set_control(double control);
    void update(double dt);

    double get_thrust() const;
    double get_torque() const;

    double w;
private:
    double tau;
    double full_thrust;
    double full_torque;
    double control_val;
};

class IrisQuadForceModel {
public:
    IrisQuadForceModel();
    void configure_from_parameters(const IrisParameters& P);
    Eigen::VectorXd update(double dt, const Eigen::VectorXd& y, const Eigen::VectorXd& u, const Eigen::Matrix<double, 6, 1>& wind, const IrisParameters& P);

private:
    SimpleMotor motors[4];
    bool configured;
};

} // namespace iris

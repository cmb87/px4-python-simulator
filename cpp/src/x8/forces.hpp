#pragma once

#include <Eigen/Dense>
#include "parameters.hpp"

namespace x8 {

Eigen::VectorXd forces(double t, const Eigen::VectorXd& y, const Eigen::VectorXd& u, const Eigen::Vector3d& wind, const X8Parameters& P);

}

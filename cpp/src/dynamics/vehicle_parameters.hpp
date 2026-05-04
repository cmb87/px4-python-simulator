#pragma once

#include <Eigen/Dense>
#include <map>
#include <string>

struct VehicleParameters {
    double mass;
    double gravity;
    double rho;
    Eigen::Matrix3d I_cg;
    Eigen::Matrix3d I_cg_inv;
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

    virtual ~VehicleParameters() = default;
};

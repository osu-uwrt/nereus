#pragma once

#include <Eigen/Core>

namespace robotics::sensors {
// Data-only contracts: no simulation, model, middleware, or viewer dependencies.
struct ImuReading {
    Eigen::Vector3d specific_force;   // Sensor frame, m/s^2; no orientation estimate.
    Eigen::Vector3d angular_velocity; // Sensor frame, rad/s.
    Eigen::Matrix3d force_covariance, angular_covariance;
};

struct FogReading {
    Eigen::VectorXd angular_rates; // One to three configured axes, rad/s.
    Eigen::MatrixXd covariance;
};

struct DvlReading {
    Eigen::Vector3d bottom_relative_velocity; // Sensor frame, m/s.
    Eigen::Matrix3d covariance;
    double bottom_distance; // Ideal slant range along bottom_axis; not noisy altitude.
};
} // namespace robotics::sensors

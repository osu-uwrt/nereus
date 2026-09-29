#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace robotics::sensors {
// Data-only contracts: no simulation, model, middleware, or viewer dependencies.
struct ImuReading {
    Eigen::Vector3d specific_force;   // Sensor frame, m/s^2; no orientation estimate.
    Eigen::Vector3d angular_velocity; // Sensor frame, rad/s.
    Eigen::Matrix3d force_covariance, angular_covariance;
};

struct AttitudeReading {
    Eigen::Quaterniond sensor_to_world;
    Eigen::Matrix3d covariance; // Sensor-axis small-angle uncertainty, rad^2.
};
struct AhrsReading {
    ImuReading inertial;
    AttitudeReading attitude; // Acquired with inertial at the same state/time.
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
struct VelocityReading {
    Eigen::Vector3d reference_relative_velocity; // Sensor frame, m/s; no range observation.
    Eigen::Matrix3d covariance;
};
struct AltitudeReading {
    double mounted_world_z; // m, measured altitude of the configured sensor point.
    double target_world_z;  // m, corrected to the configured body-fixed target at acquisition.
    double variance;        // m^2, shared by both perfectly correlated scalar observations.
};
struct PressureReading {
    double absolute_pressure; // Pa, including surface atmospheric pressure.
    double pressure_variance; // Pa^2.
    double depth;             // m, positive down; derived from measured pressure and calibration.
    double depth_variance;    // m^2; correlated with pressure, not an independent observation.
};
} // namespace robotics::sensors

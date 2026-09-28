#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace robotics::simulation {
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;

// SI units. World Z up; body X forward, Y left, Z up. All offsets are from COM.
struct BodyState {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity(); // Body to world.
    Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();       // Body frame.
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();      // Body frame.
};

struct BodyParameters {
    double mass = 10.0;
    Eigen::Matrix3d inertia = Eigen::Matrix3d::Identity();
    Matrix6 added_mass = Matrix6::Zero();
    Matrix6 linear_damping = Matrix6::Zero();
    Vector6 quadratic_damping = Vector6::Zero();
    Eigen::Vector3d damping_center = Eigen::Vector3d::Zero();
    double displaced_volume = 0.01;
    Eigen::Vector3d buoyancy_center = Eigen::Vector3d::Zero();
    Eigen::Vector3d buoyancy_radii{0.2, 0.2, 0.2};
    double collision_radius = 0.2; // COM-centered sphere; independent of buoyancy geometry.
};

struct Thruster {
    std::string id;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d direction = Eigen::Vector3d::UnitX(); // Unit thrust axis in body frame.
    double delay = 0.1;
    double rise_time = 0.08;
    double fall_time = 0.06;
    double slew_rate = 300.0; // N/s; zero disables rate limiting.
    double forward_limit = 28.0;
    double reverse_limit = 28.0;
    // If present, scale thrust by the immersed fraction of this disk (metres).
    // Absence leaves actuator force unmodulated for other propulsion models.
    std::optional<double> propeller_radius = std::nullopt;
};

struct Pool {
    double length = 20.0;
    double width = 10.0;
    double depth = 5.0;
    double water_level = 0.0; // Free surface, not a collision ceiling.
    double water_density = 1000.0;
    Eigen::Vector3d current_velocity = Eigen::Vector3d::Zero();              // Mean, world frame.
    Eigen::Vector3d current_oscillation_amplitude = Eigen::Vector3d::Zero(); // m/s.
    double current_oscillation_frequency = 0; // Hz; zero disables oscillation.
};

struct PlantParameters {
    BodyParameters body;
    Pool pool;
    std::vector<Thruster> thrusters;
    std::chrono::nanoseconds timestep{2'000'000};
    double command_timeout = 0.5; // Simulated seconds; zero holds commands indefinitely.
};

struct Snapshot {
    std::uint64_t generation = 0;
    std::uint64_t tick = 0;
    std::chrono::nanoseconds elapsed{0};
    BodyState body;
    Eigen::VectorXd thruster_forces;
};

// Instantaneous rigid-body kinematics, not a noisy sensor measurement.
struct MotionSample {
    Snapshot state;
    Eigen::Vector3d acceleration_body = Eigen::Vector3d::Zero(); // Inertial COM acceleration.
    Eigen::Vector3d angular_acceleration_body = Eigen::Vector3d::Zero();
    Eigen::Vector3d gravity_world{0, 0, -9.80665};
    // The current impulsive pool-contact model has no instantaneous force history.
    bool acceleration_valid = true;
};

// Single-owner, synchronous plant. Methods are not concurrently callable.
// No ROS, wall time, IO, callbacks, or global mutable state.
class Plant {
  public:
    Plant(const PlantParameters &parameters, const BodyState &initial);
    ~Plant();
    Plant(const Plant &) = delete;
    Plant &operator=(const Plant &) = delete;
    Plant(Plant &&) = delete;
    Plant &operator=(Plant &&) = delete;

    // Forces in profile order, N. Applied at the current tick boundary.
    // Last command at a boundary wins. Rejects wrong-size/nonfinite input before mutation.
    void command(const Eigen::VectorXd &forces);
    Snapshot advance(std::uint64_t ticks = 1);
    Snapshot observe() const;                 // Value copy: callers cannot mutate the plant.
    MotionSample motion() const;              // Read-only derivatives for sensor acquisition.
    Snapshot reset(const BodyState &initial); // Restarts time and clears actuator history.

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace robotics::simulation

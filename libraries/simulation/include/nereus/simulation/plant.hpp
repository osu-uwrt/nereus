#pragma once
// Public simulation plant API: body, pool, thruster and contact parameters, and the Plant itself.
#include <nereus/simulation/contacts.hpp>
#include <nereus/simulation/floor_profile.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace nereus::simulation {
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;

// SI units. World Z up; body X forward, Y left, Z up. All offsets are from COM.
struct BodyState {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity(); // Body to world.
    Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();       // Body frame.
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();      // Body frame.
};

// Rigid-body, added-mass and damping model (6x6 matrices in body axes about the COM) and the buoyancy ellipsoid.
struct BodyParameters {
    double mass = 10.0;
    Eigen::Matrix3d inertia = Eigen::Matrix3d::Identity();
    Matrix6 added_mass = Matrix6::Zero();
    Matrix6 linear_damping = Matrix6::Zero();
    Vector6 quadratic_damping = Vector6::Zero();
    // Point (body frame, from COM) where damping acts.
    Eigen::Vector3d damping_center = Eigen::Vector3d::Zero();
    // Displaced volume (m^3) and the ellipsoid (center, semi-axes in m) whose immersed fraction scales buoyancy.
    double displaced_volume = 0.01;
    Eigen::Vector3d buoyancy_center = Eigen::Vector3d::Zero();
    Eigen::Vector3d buoyancy_radii{0.2, 0.2, 0.2};

    double collision_radius = 0.2; // SpherePool only; independent of buoyancy/box geometry.
};

// One thruster: mount on the body (from COM), actuator timing (s) and force shaping (N).
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
    double deadband = 0; // Command magnitude below this threshold becomes zero, N.
    double forward_scale = 1, reverse_scale = 1;
    double efficiency = 1; // [0,1], applied after command scaling and saturation.

    // If present, scale thrust by the immersed fraction of this disk (meters).
    // Absence leaves actuator force unmodulated for other propulsion models.
    std::optional<double> propeller_radius = std::nullopt;
};

// Rectangular pool: `length` along pool-local X and `width` along Y from the corner at origin_xy_world (m);
// `depth` below water_level.
struct Pool {
    Eigen::Vector2d origin_xy_world = Eigen::Vector2d::Zero(); // Pool corner in world XY.
    double yaw_world = 0;                                      // Pool-local XY axes rotated into world, radians.
    double length = 20.0;
    double width = 10.0;
    double depth = 5.0;
    double water_level = 0.0; // Free surface, not a collision ceiling.

    double water_density = 1000.0;
    Eigen::Vector3d current_velocity = Eigen::Vector3d::Zero();              // Mean, world frame.
    Eigen::Vector3d current_oscillation_amplitude = Eigen::Vector3d::Zero(); // m/s.
    double current_oscillation_frequency = 0;                                // Hz; zero disables oscillation.
    // Sloped floor; empty means flat at `depth`. When set, its deepest point is `depth`.
    PoolFloor floor;
};

// The pool's floor, or a flat one at `depth`.
PoolFloor floorOf(const Pool &pool);

// Everything a Plant needs; timestep is the fixed tick, in (0, 100 ms].
struct PlantParameters {
    BodyParameters body;
    Pool pool;
    ContactParameters contacts;
    std::vector<Thruster> thrusters;
    std::chrono::nanoseconds timestep{2'000'000};
    double command_timeout = 0.5; // Simulated seconds; zero holds commands indefinitely.
};

// Committed plant state at a tick boundary. generation counts reset() calls; thruster_forces are the simulated
// (not commanded) forces in profile order, N.
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

    // Clear delayed commands/targets immediately; existing force coasts down on
    // subsequent steps. Does not reset time or latch out future explicit commands.
    void stopThrusters();
    // Advances `ticks` fixed timesteps; a throw faults the plant until reset().
    Snapshot advance(std::uint64_t ticks = 1);

    Snapshot observe() const;    // Value copy: callers cannot mutate the plant.
    MotionSample motion() const; // Read-only derivatives for sensor acquisition.

    // Teleports without restarting time/generation; optionally clears propulsion history.
    Snapshot place(const BodyState &state, bool clear_actuators = true);
    Snapshot reset(const BodyState &initial); // Restarts time and clears actuator history.

    // Optional extra contact response (nullptr removes it); see ContactResolver.
    void setContactResolver(std::shared_ptr<ContactResolver> resolver);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace nereus::simulation

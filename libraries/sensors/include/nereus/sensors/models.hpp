#pragma once

#include "nereus/sensors/readings.hpp"
#include "nereus/sensors/types.hpp"
#include "nereus/simulation/plant.hpp"
#include <functional>
#include <random>

namespace nereus::sensors {
struct Mount {
    Eigen::Vector3d position_body = Eigen::Vector3d::Zero(); // From COM, metres.
    Eigen::Quaterniond sensor_to_body = Eigen::Quaterniond::Identity();
};

struct NoiseParameters {
    Eigen::Vector3d bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d white_stddev = Eigen::Vector3d::Zero(); // Per acquisition, not density.
    Eigen::Vector3d walk_stddev = Eigen::Vector3d::Zero();  // Per sqrt(second).
};

// Composable, instance-owned noise. Reset restarts both RNG and accumulated drift.
class Noise3 {
  public:
    explicit Noise3(NoiseParameters parameters = {});
    void reset(std::uint64_t seed, const std::string &id, const std::string &component);
    Eigen::Vector3d sample(double elapsed_seconds);
    Eigen::Matrix3d covariance() const;

  private:
    NoiseParameters parameters_;
    Eigen::Vector3d drift_ = Eigen::Vector3d::Zero();
    double elapsed_ = 0;
    std::mt19937_64 random_;
    std::normal_distribution<double> normal_;
};

struct ImuReporting {
    // Measurement-only gravity calibration; direction still comes from the environment.
    std::optional<double> gravity_magnitude; // m/s^2; absent uses physical gravity unchanged.
    // Optional sensor-frame diagonal variances, independent of generated noise.
    std::optional<Eigen::Vector3d> force_variance, angular_variance;
};

class Imu {
  public:
    using Reading = ImuReading;
    explicit Imu(Mount mount = {}, NoiseParameters acceleration = {}, NoiseParameters gyro = {},
                 ImuReporting reporting = {});
    void reset(std::uint64_t seed, const std::string &id);
    Measurement<Reading> sample(const simulation::MotionSample &, double elapsed_seconds);

  private:
    Mount mount_;
    Noise3 acceleration_, gyro_;
    ImuReporting reporting_;
};

struct AttitudeParameters {
    double angle_stddev = 0;       // rad, normally distributed angle about an isotropic axis.
    double heading_drift_rate = 0; // rad/s, deterministic rotation about heading_axis_world.
    Eigen::Vector3d heading_axis_world = Eigen::Vector3d::UnitZ();
    std::optional<Eigen::Vector3d> reported_variance; // Sensor-axis diagonal rad^2.
};

// Explicit simulated attitude observation, not an estimator driven by raw IMU data.
class Attitude {
  public:
    using Reading = AttitudeReading;
    explicit Attitude(Mount mount = {}, AttitudeParameters parameters = {});
    void reset(std::uint64_t seed, const std::string &id);
    Measurement<Reading> sample(const simulation::MotionSample &, double elapsed_seconds);

  private:
    Mount mount_;
    AttitudeParameters parameters_;
    std::mt19937_64 random_;
    std::normal_distribution<double> normal_;
};

struct AhrsParameters {
    NoiseParameters acceleration_noise, gyro_noise;
    ImuReporting inertial_reporting;
    AttitudeParameters attitude;
};
// Composes two independent models using one mount and one scheduled acquisition.
class Ahrs {
  public:
    using Reading = AhrsReading;
    explicit Ahrs(Mount mount = {}, AhrsParameters parameters = {});
    void reset(std::uint64_t seed, const std::string &id);
    Measurement<Reading> sample(const simulation::MotionSample &, double elapsed_seconds);

  private:
    Imu inertial_;
    Attitude attitude_;
};

class Fog {
  public:
    using Reading = FogReading;
    explicit Fog(Mount mount = {}, std::vector<Eigen::Vector3d> axes = {Eigen::Vector3d::UnitZ()},
                 NoiseParameters gyro = {}, std::optional<Eigen::Vector3d> reported_variance = {});
    void reset(std::uint64_t seed, const std::string &id);
    Measurement<Reading> sample(const simulation::MotionSample &, double elapsed_seconds);

  private:
    Mount mount_;
    Eigen::Matrix<double, Eigen::Dynamic, 3> axes_;
    Noise3 gyro_;
    std::optional<Eigen::Vector3d> reported_variance_; // Sensor axes, before projection.
};

struct BottomHit {
    double distance; // Along the unit ray, metres.
    Eigen::Vector3d velocity_world = Eigen::Vector3d::Zero();
};
using BottomQuery = std::function<std::optional<BottomHit>(const Eigen::Vector3d &origin_world,
                                                           const Eigen::Vector3d &direction_world)>;
// Finite, stationary pool floor. Geometry is copied; no plant or renderer ownership.
class PoolBottom {
  public:
    explicit PoolBottom(const simulation::Pool &pool);
    std::optional<BottomHit> operator()(const Eigen::Vector3d &origin_world,
                                        const Eigen::Vector3d &direction_world) const;

  private:
    double length_, width_, surface_;
    simulation::PoolFloor floor_;
    Eigen::Vector2d origin_xy_;
    Eigen::Matrix2d world_to_pool_;
    double boundary_tolerance_{0};
};
struct DvlParameters {
    Mount mount;
    Eigen::Vector3d bottom_axis = -Eigen::Vector3d::UnitZ(); // Unit ray in sensor frame.
    double minimum_range = 0.1, maximum_range = 50;
    NoiseParameters velocity_noise;
};

class Dvl {
  public:
    using Reading = DvlReading;
    Dvl(DvlParameters parameters, BottomQuery bottom);
    void reset(std::uint64_t seed, const std::string &id);
    Measurement<Reading> sample(const simulation::MotionSample &, double elapsed_seconds);

  private:
    DvlParameters parameters_;
    BottomQuery bottom_;
    Noise3 velocity_;
};

struct InclinationLimit {
    Eigen::Vector3d sensor_axis = -Eigen::Vector3d::UnitZ();
    Eigen::Vector3d reference_axis_world = -Eigen::Vector3d::UnitZ();
    double maximum_angle = 0; // rad; zero permits only aligned axes. Absence disables the gate.
};
struct ReferenceVelocityParameters {
    Mount mount;
    Eigen::Vector3d reference_velocity_world = Eigen::Vector3d::Zero();
    NoiseParameters noise;
    std::optional<Eigen::Vector3d> reported_variance; // Sensor-axis diagonal (m/s)^2.
    std::optional<InclinationLimit> inclination_limit;
};
// Ideal mounted velocity relative to an explicit constant world velocity.
// No bottom/range/water query is implied by this separately selected model.
class ReferenceVelocity {
  public:
    using Reading = VelocityReading;
    explicit ReferenceVelocity(ReferenceVelocityParameters parameters = {});
    void reset(std::uint64_t seed, const std::string &id);
    Measurement<Reading> sample(const simulation::MotionSample &, double elapsed_seconds);

  private:
    ReferenceVelocityParameters parameters_;
    Noise3 noise_;
};

struct ScalarNoiseParameters {
    double bias = 0, white_stddev = 0, walk_stddev = 0;
};
struct ReferenceAltitudeParameters {
    Mount mount;
    std::optional<Eigen::Vector3d> target_position_body; // COM-local metres; absent uses mount.
    ScalarNoiseParameters noise;                         // m, m per acquisition, m/sqrt(s).
    std::optional<double> reported_variance;             // m^2, independent of generated noise.
};
// Ideal world-Z observation with same-acquisition target-point correction, not pressure.
class ReferenceAltitude {
  public:
    using Reading = AltitudeReading;
    explicit ReferenceAltitude(ReferenceAltitudeParameters parameters = {});
    void reset(std::uint64_t seed, const std::string &id);
    Measurement<Reading> sample(const simulation::MotionSample &, double elapsed_seconds);

  private:
    ReferenceAltitudeParameters parameters_;
    Eigen::Vector3d target_delta_body_;
    Noise3 noise_;
};

struct PressureParameters {
    Mount mount;
    ScalarNoiseParameters noise;                         // Pa, Pa per acquisition, Pa/sqrt(s).
    double reference_pressure = 101325;                  // Depth conversion calibration, Pa.
    double reference_density = 1000;                     // kg/m^3.
    double reference_gravity = 9.80665;                  // m/s^2.
    double minimum_pressure = 0, maximum_pressure = 1e7; // Inclusive operating range, Pa.
};
using PressureQuery = std::function<std::optional<double>(const Eigen::Vector3d &position_world)>;
// Constant-density fluid below a horizontal surface; constant atmospheric pressure above it.
class HydrostaticPressure {
  public:
    HydrostaticPressure(double water_level, double density = 1000, double surface_pressure = 101325,
                        double gravity = 9.80665);
    std::optional<double> operator()(const Eigen::Vector3d &position_world) const;

  private:
    double level_, surface_pressure_, gradient_;
};
class Pressure {
  public:
    using Reading = PressureReading;
    Pressure(PressureParameters parameters, PressureQuery environment);
    void reset(std::uint64_t seed, const std::string &id);
    Measurement<Reading> sample(const simulation::MotionSample &, double elapsed_seconds);

  private:
    PressureParameters parameters_;
    PressureQuery environment_;
    Noise3 noise_; // Reuse the scalar X component of the shared noise process.
    double depth_scale_;
};
} // namespace nereus::sensors

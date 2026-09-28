#pragma once

#include "robotics/sensors/readings.hpp"
#include "robotics/sensors/types.hpp"
#include "robotics/simulation/plant.hpp"
#include <functional>
#include <random>

namespace robotics::sensors {
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

class Imu {
  public:
    using Reading = ImuReading;
    explicit Imu(Mount mount = {}, NoiseParameters acceleration = {}, NoiseParameters gyro = {});
    void reset(std::uint64_t seed, const std::string &id);
    Measurement<Reading> sample(const simulation::MotionSample &, double elapsed_seconds);

  private:
    Mount mount_;
    Noise3 acceleration_, gyro_;
};

class Fog {
  public:
    using Reading = FogReading;
    explicit Fog(Mount mount = {}, std::vector<Eigen::Vector3d> axes = {Eigen::Vector3d::UnitZ()},
                 NoiseParameters gyro = {});
    void reset(std::uint64_t seed, const std::string &id);
    Measurement<Reading> sample(const simulation::MotionSample &, double elapsed_seconds);

  private:
    Mount mount_;
    Eigen::Matrix<double, Eigen::Dynamic, 3> axes_;
    Noise3 gyro_;
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
    double length_, width_, floor_, surface_;
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
} // namespace robotics::sensors

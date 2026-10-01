#pragma once
// Pure sampling geometry: per-sample random streams, robot pose samplers, pool-local checks and camera
// intrinsics scaling. No GL.
#include <nereus/cameras/camera.hpp>
#include <nereus/datasets/job.hpp>
#include <nereus/session/pool.hpp>
#include <nereus/spatial/frames.hpp>

#include <cstdint>
#include <map>
#include <optional>
#include <random>
#include <string>

namespace nereus::datasets {
using spatial::Pose;

std::uint64_t splitmix64(std::uint64_t x);

// A sample's random stream: mt19937_64 seeded with splitmix64(seed ^ splitmix64(k)). Every draw of the sample
// (randomization, all attempts, image noise seed) comes from it, so output does not depend on sharding.
// Draws are implemented here (not std:: distributions) so they are identical across standard libraries.
class Stream {
  public:
    Stream(std::uint64_t seed, std::int64_t sample);
    std::uint64_t bits() {
        return engine_();
    }
    double uniform(); // [0, 1)
    double uniform(double lo, double hi) {
        return lo + (hi - lo) * uniform();
    }
    double uniform(const Range &r) {
        return uniform(r.lo, r.hi);
    }
    double symmetric(double half) {
        return uniform(-half, half);
    }
    bool chance(double p) {
        return uniform() < p;
    }
    std::size_t index(std::size_t n) {
        return std::min(n - 1, static_cast<std::size_t>(uniform() * static_cast<double>(n)));
    }

  private:
    std::mt19937_64 engine_;
};

// Pool-local frame (session::PoolModel convention: corner origin, +x along the length, water surface at
// surface_z) placed in the world by the scenario's pool_placement (yaw about +Z, then position).
struct PoolFrame {
    Pose world_from_pool;
    double length = 0, width = 0, surface_z = 0;
    simulation::PoolFloor floor;

    static PoolFrame fromScenario(const session::ResolvedScenario &);
    Eigen::Vector3d toPool(const Eigen::Vector3d &world) const;
    Eigen::Vector3d toWorld(const Eigen::Vector3d &pool) const;
    double floorZ(const Eigen::Vector2d &pool_xy) const {
        return surface_z - floor.depthAt(pool_xy);
    }
    // §3.2 check 1 for a camera optical centre: empty when acceptable, else the rejection reason
    // ("above_surface" | "outside_pool" | "near_floor").
    std::optional<std::string> checkCamera(const Eigen::Vector3d &world, double surface_margin = .1,
                                           double wall_margin = .3, double floor_margin = .2) const;
};

// Robot root attitude from yaw / nose-up pitch / roll (radians), FLU body axes: Rz(yaw) Ry(-pitch_up) Rx(roll).
Eigen::Quaterniond attitude(double yaw, double pitch_up, double roll);

// A drawn robot pose. `reason` is set when the attempt failed before rendering.
struct PoseDraw {
    std::optional<Pose> world_from_root;
    std::string frame; // the target frame picked
    std::string reason;
};
// world_from_frames: the sample's target task frames ("task" and the task's own frames) in the world, after
// placement jitter. Background (free) samples ignore it.
PoseDraw samplePose(const Sampler &, Stream &, const std::map<std::string, Pose> &world_from_frames, const PoolFrame &,
                    const Pose &root_from_optical);

// Intrinsics for an output of (width, height) keeping the field of view: s = max(w'/w, h'/h), centre crop.
cameras::Intrinsics scaleIntrinsics(const cameras::Intrinsics &native, int width, int height);

Json poseJson(const Pose &);
} // namespace nereus::datasets

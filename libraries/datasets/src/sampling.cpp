// Sampling geometry: per-sample random streams, the robot pose samplers (fixed, free, overhead, approach),
// pool-frame checks and intrinsics scaling.
#include <nereus/datasets/sampling.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace nereus::datasets {
namespace {

constexpr double kDeg = M_PI / 180;
constexpr double kSurfaceMarginM = .1, kFloorMarginM = .2; // §3.2 check 1 (A8: drawn within, not rejected)

// World z of the water surface and of the pool floor below a world point.
double surfaceWorldZ(const PoolFrame &pool) {
    return pool.toWorld({0, 0, pool.surface_z}).z();
}

double floorWorldZ(const PoolFrame &pool, const Eigen::Vector3d &world) {
    const auto local = pool.toPool(world);
    return pool.toWorld({local.x(), local.y(), pool.floorZ(local.head<2>())}).z();
}

// Root position that puts the optical center at `camera` for a robot attitude.
Pose rootFor(const Eigen::Vector3d &camera, const Eigen::Quaterniond &attitude, const Pose &root_from_optical) {
    Pose root;
    root.rotation = attitude.normalized();
    root.translation = camera - root.rotation * root_from_optical.translation;
    return root;
}

// A named frame of the sample's task (world pose); throws when the sampler names an unknown frame.
const Pose &targetFrame(const std::map<std::string, Pose> &frames, const std::string &id) {
    const auto found = frames.find(id);
    if (found == frames.end())
        throw std::runtime_error("sampler frame '" + id + "' is not a frame of the sample's task");
    return found->second;
}

} // namespace

// SplitMix64 finalizer (Steele et al.): decorrelates nearby seeds / sample indices.
std::uint64_t splitmix64(std::uint64_t x) {
    x += 0x9e3779b97f4a7c15ull;
    x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ull;
    x = (x ^ (x >> 27)) * 0x94d049bb133111ebull;
    return x ^ (x >> 31);
}

Stream::Stream(std::uint64_t seed, std::int64_t sample)
    : engine_(splitmix64(seed ^ splitmix64(static_cast<std::uint64_t>(sample)))) {}

// Top 53 bits of one engine output scaled to [0, 1): exact doubles, same on every standard library.
double Stream::uniform() {
    return static_cast<double>(engine_() >> 11) * 0x1.0p-53;
}

PoolFrame PoolFrame::fromScenario(const session::ResolvedScenario &resolved) {
    // Placement: yaw about +Z (half-angle quaternion), then translation.
    PoolFrame result;
    const auto &placement = resolved.scenario.at("pool_placement");
    const auto &at = placement.at("position_m");
    const double half = placement.at("yaw_deg").get<double>() * kDeg / 2;
    result.world_from_pool.translation = {at.at(0).get<double>(), at.at(1).get<double>(), at.at(2).get<double>()};
    result.world_from_pool.rotation = Eigen::Quaterniond(std::cos(half), 0, 0, std::sin(half));

    // Pool dimensions, surface height and floor profile from the pool pack.
    const auto &p = resolved.pool.at("parameters");
    result.length = p.at("length_m").get<double>();
    result.width = p.at("width_m").get<double>();
    const auto model = session::poolModel(resolved.pool);
    result.surface_z = model.surface_z;
    result.floor = model.floor;
    return result;
}

Eigen::Vector3d PoolFrame::toPool(const Eigen::Vector3d &world) const {
    return spatial::apply(spatial::inverse(world_from_pool), world);
}

Eigen::Vector3d PoolFrame::toWorld(const Eigen::Vector3d &pool) const {
    return spatial::apply(world_from_pool, pool);
}

std::optional<std::string> PoolFrame::checkCamera(const Eigen::Vector3d &world, double surface_margin,
                                                  double wall_margin, double floor_margin) const {
    const auto p = toPool(world);
    if (p.x() < wall_margin || p.x() > length - wall_margin || p.y() < wall_margin || p.y() > width - wall_margin)
        return "outside_pool";
    if (p.z() > surface_z - surface_margin)
        return "above_surface";
    if (p.z() < floorZ(p.head<2>()) + floor_margin)
        return "near_floor";
    return std::nullopt;
}

Eigen::Quaterniond attitude(double yaw, double pitch_up, double roll) {
    return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-pitch_up, Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
}

PoseDraw samplePose(const Sampler &s, Stream &rng, const std::map<std::string, Pose> &frames, const PoolFrame &pool,
                    const Pose &root_from_optical) {
    PoseDraw draw;

    // Fixed: the given pose, no draws.
    if (s.type == "fixed") {
        draw.world_from_root = s.world_from_root;
        draw.frame = s.target_frame.value_or("");
        return draw;
    }
    const double zTop = surfaceWorldZ(pool) - kSurfaceMarginM;

    // Free (background): anywhere in the pool 0.5 m from the walls, any heading, depth below the surface.
    if (s.type == "free") {
        const double margin = .5;
        const double x = rng.uniform(margin, pool.length - margin), y = rng.uniform(margin, pool.width - margin);
        const double depth = rng.uniform(s.depth_m);
        const double yaw = rng.uniform(0, 2 * M_PI), pitch = rng.symmetric(s.pitch_deg) * kDeg,
                     roll = rng.symmetric(s.roll_deg) * kDeg;
        Pose root;
        root.translation = pool.toWorld({x, y, pool.surface_z - depth});
        root.rotation = (pool.world_from_pool.rotation * attitude(yaw, pitch, roll)).normalized();
        draw.world_from_root = root;
        return draw;
    }

    // Target-relative samplers: pick a target frame, aim at its `offset` point.
    draw.frame = s.frames[rng.index(s.frames.size())];
    const Pose &world_frame = targetFrame(frames, draw.frame);
    const Eigen::Vector3d target = spatial::apply(world_frame, s.offset);

    // Overhead: a point on a disc of radius_m around the target (uniform by area), at an altitude above it.
    if (s.type == "overhead") {
        const double radius = s.radius_m * std::sqrt(rng.uniform()), angle = rng.uniform(0, 2 * M_PI);
        Eigen::Vector3d camera = target + Eigen::Vector3d(radius * std::cos(angle), radius * std::sin(angle), 0);
        // Altitude within the water column at that point (A8 for approach; the same idea here).
        const double lo = std::max(s.altitude_m.lo, floorWorldZ(pool, camera) + kFloorMarginM - target.z());
        const double hi = std::min(s.altitude_m.hi, zTop - target.z());
        // Every value is drawn before the feasibility check, so each attempt consumes the same number of draws.
        const double u = rng.uniform();
        const double yaw = s.yaw_deg ? *s.yaw_deg * kDeg : rng.uniform(0, 2 * M_PI);
        const double pitch = rng.symmetric(s.pitch_deg) * kDeg, roll = rng.symmetric(s.roll_deg) * kDeg;
        if (lo > hi) {
            draw.reason = "altitude_infeasible";
            return draw;
        }
        camera.z() = target.z() + lo + (hi - lo) * u;
        draw.world_from_root = rootFor(camera, attitude(yaw, pitch, roll), root_from_optical);
        return draw;
    }

    // approach: from the side the frame's `facing` points to, at range_m, within ±bearing_deg of that side,
    // then aim the camera's optical axis at the target.
    Eigen::Vector3d facing = world_frame.rotation * s.facing;
    if (s.both_sides && rng.chance(.5))
        facing = -facing;
    const Eigen::Vector2d horizontal = facing.head<2>().normalized();
    const double range = rng.uniform(s.range_m), bearing = rng.symmetric(s.bearing_deg) * kDeg;
    const Eigen::Vector2d dir = Eigen::Rotation2Dd(bearing) * horizontal;
    // Elevation within the feasible interval: the camera stays below the surface and above the floor.
    const double zBottom = floorWorldZ(pool, target + range * Eigen::Vector3d(dir.x(), dir.y(), 0)) + kFloorMarginM;
    const double sinLo = (zBottom - target.z()) / range, sinHi = (zTop - target.z()) / range;
    const double eLo = std::max(s.elevation_deg.lo * kDeg, std::asin(std::clamp(sinLo, -1.0, 1.0)));
    const double eHi = std::min(s.elevation_deg.hi * kDeg, std::asin(std::clamp(sinHi, -1.0, 1.0)));
    // Draw everything before the feasibility check (fixed number of draws per attempt).
    const double u = rng.uniform();
    const double aim = rng.symmetric(s.aim_jitter_deg) * kDeg, pitchJitter = rng.symmetric(s.pitch_deg) * kDeg,
                 roll = rng.symmetric(s.roll_deg) * kDeg;
    if (sinLo > 1 || sinHi < -1 || eLo > eHi) {
        draw.reason = "elevation_infeasible";
        return draw;
    }
    const double elevation = eLo + (eHi - eLo) * u;
    const Eigen::Vector3d camera = target + range * Eigen::Vector3d(std::cos(elevation) * dir.x(),
                                                                    std::cos(elevation) * dir.y(), std::sin(elevation));
    // Mount geometry: the optical axis in the root frame, its heading and elevation on a level robot.
    const Eigen::Vector3d axis = root_from_optical.rotation * Eigen::Vector3d::UnitZ();
    const double axisXY = axis.head<2>().norm();
    const double mountHeading = axisXY > 1e-3 ? std::atan2(axis.y(), axis.x()) : 0.0;
    const double mountElevation = std::atan2(axis.z(), axisXY);

    // Robot yaw / pitch that point the mounted optical axis at the target; aim pitch limited to max_aim_pitch_deg.
    const Eigen::Vector3d look = target - camera;
    const double lookPitch = std::atan2(look.z(), look.head<2>().norm());
    const double maxPitch = s.max_aim_pitch_deg * kDeg;
    const double yaw = std::atan2(look.y(), look.x()) - mountHeading + aim;
    const double pitch = std::clamp(lookPitch, -maxPitch, maxPitch) - mountElevation + pitchJitter;
    draw.world_from_root = rootFor(camera, attitude(yaw, pitch, roll), root_from_optical);
    return draw;
}

// Scale so the output is covered, then crop the overflow equally from both sides. The +-0.5 terms scale about
// pixel corners rather than pixel centers.
cameras::Intrinsics scaleIntrinsics(const cameras::Intrinsics &native, int width, int height) {
    const double s = std::max(double(width) / native.width, double(height) / native.height);
    cameras::Intrinsics k = native;
    k.width = width;
    k.height = height;
    k.fx = native.fx * s;
    k.fy = native.fy * s;
    k.cx = (native.cx + .5) * s - .5 - (native.width * s - width) / 2;
    k.cy = (native.cy + .5) * s - .5 - (native.height * s - height) / 2;
    k.validate();
    return k;
}

Json poseJson(const Pose &pose) {
    const auto &q = pose.rotation;
    return {{"position_m", {pose.translation.x(), pose.translation.y(), pose.translation.z()}},
            {"orientation_wxyz", {q.w(), q.x(), q.y(), q.z()}}};
}

} // namespace nereus::datasets

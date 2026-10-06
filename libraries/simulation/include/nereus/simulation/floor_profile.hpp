#pragma once
// Sloped pool floors: depth profiles, ray queries and the contact boxes built from them.
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <optional>
#include <vector>

namespace nereus::simulation {

// Pool floor depth along one pool axis, constant across the other. Built from control points
// (position along the axis, depth below the water surface) joined by a monotone cubic (PCHIP,
// Fritsch-Carlson), so flat stretches stay flat and a curve never overshoots its end depths. The
// curve is kept as a polyline sampled finely enough for sensing, contacts and rendering to share.
class FloorProfile {
  public:
    enum class Axis { X, Y };

    FloorProfile() = default; // Empty: no profile, the owner falls back to a flat floor.
    // Level floor at `depth` over [0, extent].
    static FloorProfile flat(double depth, double extent, Axis axis = Axis::X);
    // Positions strictly increasing from 0 to the pool extent along `axis`; depths positive.
    // `step` is the sampling pitch; collinear samples within `tolerance` are merged.
    static FloorProfile smooth(Axis axis, const std::vector<Eigen::Vector2d> &points, double step = 0.05,
                               double tolerance = 1e-3);

    bool empty() const {
        return polyline_.empty();
    }
    Axis axis() const {
        return axis_;
    }
    double extent() const {
        return polyline_.back().x();
    }
    double minDepth() const;
    double maxDepth() const;
    bool isFlat() const {
        return polyline_.size() == 2 && polyline_[0].y() == polyline_[1].y();
    }
    // (position along the axis, depth) vertices, from 0 to extent().
    const std::vector<Eigen::Vector2d> &polyline() const {
        return polyline_;
    }

    // Depth at a position along the axis, clamped to [0, extent()].
    double depthAt(double position) const;
    // Depth under a pool-local (x, y) point.
    double depthAt(const Eigen::Vector2d &pool_xy) const {
        return depthAt(along(pool_xy));
    }
    // Position along this profile's axis of a pool-local point.
    double along(const Eigen::Vector2d &pool_xy) const {
        return axis_ == Axis::X ? pool_xy.x() : pool_xy.y();
    }

    // Distance along a unit ray (pool-local, z up, water surface at `surface_z`) to its first floor
    // crossing, or nullopt for a ray that does not point down. The floor continues flat past both ends at
    // the end depths, like the plane of a flat pool; callers check that the hit lies inside the pool.
    std::optional<double> rayDistance(const Eigen::Vector3d &origin, const Eigen::Vector3d &direction,
                                      double surface_z) const;

  private:
    Axis axis_ = Axis::X;
    std::vector<Eigen::Vector2d> polyline_;
};

// A pool floor built from one or more profiles, each sloping along its own axis. Where they overlap the
// floor is the shallowest of them, so a deep flat area can rise toward more than one wall (a dive well
// deep beside its platforms). Below each profile's surface is solid; the floor solid is their union.
class PoolFloor {
  public:
    PoolFloor() = default; // Empty: no profile, the owner falls back to a flat floor.
    explicit PoolFloor(std::vector<FloorProfile> profiles);
    // A single flat profile along X.
    static PoolFloor flat(double depth, double length);

    bool empty() const {
        return profiles_.empty();
    }
    const std::vector<FloorProfile> &profiles() const {
        return profiles_;
    }
    // One profile with a single level segment.
    bool isFlat() const {
        return profiles_.size() == 1 && profiles_.front().isFlat();
    }
    // Depth under a pool-local (x, y) point: the shallowest profile there.
    double depthAt(const Eigen::Vector2d &pool_xy) const;
    // Deepest point of the floor (each profile varies along its own axis only, so this is the shallowest
    // of the profiles' deepest points).
    double maxDepth() const;
    // First floor crossing of a unit ray, as FloorProfile::rayDistance: the nearest crossing of any profile.
    std::optional<double> rayDistance(const Eigen::Vector3d &origin, const Eigen::Vector3d &direction,
                                      double surface_z) const;

  private:
    std::vector<FloorProfile> profiles_;
};

// Oriented box in pool-local coordinates.
struct FloorBox {
    Eigen::Vector3d size = Eigen::Vector3d::Zero();
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

// Contact boxes for the floor: one per polyline segment, spanning the pool across the axis, top face
// on the segment, `thickness` deep, overlapping each neighbor by `overlap` so seams stay closed.
std::vector<FloorBox> floorBoxes(const FloorProfile &profile, double span, double surface_z, double thickness = 1.0,
                                 double overlap = 0.01);
// Every profile's boxes; each spans the pool across its profile's axis (`length` x `width` pool).
std::vector<FloorBox> floorBoxes(const PoolFloor &floor, double length, double width, double surface_z,
                                 double thickness = 1.0, double overlap = 0.01);

} // namespace nereus::simulation

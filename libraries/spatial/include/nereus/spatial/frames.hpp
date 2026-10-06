// Rigid poses and an immutable tree of named fixed frames (sensor mounts).
#pragma once
#include <Eigen/Geometry>
#include <map>
#include <string>
#include <vector>

namespace nereus::spatial {

// Maps child coordinates into parent coordinates. SI meters, unit quaternion.
struct Pose {
    Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
};

// Throws std::invalid_argument unless translation is finite (|x| <= 1e12 m) and rotation is unit.
void validate(const Pose &pose);

// Operations require validated poses; composition/inversion preserve rigidity.
// compose(a_from_b, b_from_c) -> a_from_c.
Pose compose(const Pose &parent, const Pose &child);
Pose inverse(const Pose &pose);

// Maps a point from the pose's child frame into its parent frame.
Eigen::Vector3d apply(const Pose &pose, const Eigen::Vector3d &point);

// One rigid mount: `pose` maps `child` coordinates into `parent` coordinates.
struct FixedFrame {
    std::string parent;
    std::string child;
    Pose pose;
};

// Immutable named rigid mounts. Resolves all frames once at construction.
// No clocks, geometry, robot names, transport, or dynamic transform history.
class FixedFrames {
  public:
    FixedFrames(std::string root, std::vector<FixedFrame> edges);
    const std::string &root() const;
    const std::vector<FixedFrame> &edges() const; // Validated, normalized edges in declaration order.

    // root_from_frame; throws for an unknown frame.
    const Pose &fromRoot(const std::string &frame) const;
    // target_from_from: maps points expressed in `from` into `target` coordinates.
    Pose lookup(const std::string &target, const std::string &from) const;

  private:
    std::string root_;
    std::vector<FixedFrame> edges_;
    std::map<std::string, Pose> resolved_; // frame -> root_from_frame, including the root.
};

} // namespace nereus::spatial

#pragma once
#include <Eigen/Geometry>
#include <map>
#include <string>
#include <vector>

namespace nereus::spatial {
// Maps child coordinates into parent coordinates. SI metres, unit quaternion.
struct Pose {
    Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
};
void validate(const Pose &pose);
// Operations require validated poses; composition/inversion preserve rigidity.
Pose compose(const Pose &parent, const Pose &child);
Pose inverse(const Pose &pose);
Eigen::Vector3d apply(const Pose &pose, const Eigen::Vector3d &point);

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
    const std::vector<FixedFrame> &edges() const;
    const Pose &fromRoot(const std::string &frame) const;
    Pose lookup(const std::string &target, const std::string &from) const;

  private:
    std::string root_;
    std::vector<FixedFrame> edges_;
    std::map<std::string, Pose> resolved_;
};
} // namespace nereus::spatial

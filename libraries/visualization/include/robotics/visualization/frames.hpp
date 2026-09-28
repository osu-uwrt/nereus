#pragma once

#include <Eigen/Geometry>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace robotics::visualization {
using Time = std::int64_t; // Nonnegative nanoseconds in an explicitly named source clock.
struct Pose {
    Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
};
void validate(const Pose &pose);
Pose compose(const Pose &parent, const Pose &child);
Pose inverse(const Pose &pose);
Eigen::Vector3d apply(const Pose &pose, const Eigen::Vector3d &point);

struct TransformSample {
    Time time_ns{0};
    Pose pose;
};
struct FrameEdge {
    std::string parent;
    std::string child;
    bool is_static{false};
    std::vector<TransformSample> samples;
};
struct Lookup {
    std::optional<Pose> pose;
    std::string issue;
};

// Immutable, single-source frame tree. Poses map child coordinates into parent coordinates.
class FrameGraph {
  public:
    FrameGraph(std::string root, std::vector<FrameEdge> edges);
    Lookup lookup(const std::string &target, const std::string &from, Time time_ns) const;
    std::vector<std::string> frames() const;

  private:
    std::string root_;
    std::map<std::string, FrameEdge> edges_;
};
} // namespace robotics::visualization

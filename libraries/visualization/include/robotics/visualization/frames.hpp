#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <robotics/spatial/frames.hpp>
#include <string>
#include <vector>

namespace robotics::visualization {
using Time = std::int64_t; // Nonnegative nanoseconds in an explicitly named source clock.
using spatial::apply;
using spatial::compose;
using spatial::inverse;
using spatial::Pose;
using spatial::validate;

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

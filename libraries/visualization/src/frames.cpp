#include <robotics/visualization/frames.hpp>

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <utility>

namespace robotics::visualization {
void validate(const Pose &pose) {
    if (!pose.translation.allFinite() || pose.translation.cwiseAbs().maxCoeff() > 1e12 ||
        !pose.rotation.coeffs().allFinite() || std::abs(pose.rotation.norm() - 1.0) > 1e-8)
        throw std::invalid_argument("pose requires finite translation and a unit quaternion");
}
Pose compose(const Pose &parent, const Pose &child) {
    return {apply(parent, child.translation), parent.rotation * child.rotation};
}
Pose inverse(const Pose &pose) {
    const auto rotation = pose.rotation.conjugate();
    return {rotation * -pose.translation, rotation};
}
Eigen::Vector3d apply(const Pose &pose, const Eigen::Vector3d &point) {
    return pose.translation + pose.rotation * point;
}
FrameGraph::FrameGraph(std::string root, std::vector<FrameEdge> edges) : root_(std::move(root)) {
    if (root_.empty() || edges.size() > 128)
        throw std::invalid_argument("frame tree requires a root and at most 128 edges");
    std::size_t total = 0;
    for (auto &edge : edges) {
        if (edge.parent.empty() || edge.child.empty() || edge.child == root_ ||
            edge.samples.empty() || edge.samples.size() > 10000 ||
            (edge.is_static && edge.samples.size() != 1))
            throw std::invalid_argument("invalid frame edge or sample count");
        Time previous = -1;
        for (const auto &sample : edge.samples) {
            validate(sample.pose);
            if (sample.time_ns < 0 || sample.time_ns <= previous)
                throw std::invalid_argument("frame sample times must increase strictly");
            previous = sample.time_ns;
        }
        total += edge.samples.size();
        const auto child = edge.child;
        if (!edges_.emplace(child, std::move(edge)).second)
            throw std::invalid_argument("frame child has more than one parent: " + child);
    }
    if (total > 100000)
        throw std::invalid_argument("frame history exceeds 100000 samples");
    for (const auto &[child, edge] : edges_) {
        (void)edge;
        std::set<std::string> visited;
        auto frame = child;
        while (frame != root_) {
            if (!visited.insert(frame).second)
                throw std::invalid_argument("frame cycle at " + frame);
            const auto found = edges_.find(frame);
            if (found == edges_.end())
                throw std::invalid_argument("unknown frame parent: " + frame);
            frame = found->second.parent;
        }
    }
}
std::vector<std::string> FrameGraph::frames() const {
    std::vector<std::string> result{root_};
    for (const auto &[name, edge] : edges_) {
        (void)edge;
        result.push_back(name);
    }
    return result;
}
Lookup FrameGraph::lookup(const std::string &target, const std::string &from, Time time_ns) const {
    if (time_ns < 0)
        return {std::nullopt, "negative lookup time"};
    for (const auto &frame : {target, from})
        if (frame != root_ && edges_.count(frame) == 0)
            return {std::nullopt, "unknown frame: " + frame};
    std::set<std::string> ancestors{root_};
    for (auto frame = from; frame != root_; frame = edges_.at(frame).parent)
        ancestors.insert(frame);
    auto common = target;
    while (ancestors.count(common) == 0)
        common = edges_.at(common).parent;
    auto toCommon = [&](std::string frame) -> Lookup {
        Pose result;
        while (frame != common) {
            const auto &edge = edges_.at(frame);
            Pose transform;
            if (edge.is_static) {
                transform = edge.samples.front().pose;
            } else {
                const auto upper = std::lower_bound(
                    edge.samples.begin(), edge.samples.end(), time_ns,
                    [](const TransformSample &sample, Time time) { return sample.time_ns < time; });
                if (upper == edge.samples.end() ||
                    (upper == edge.samples.begin() && upper->time_ns != time_ns))
                    return {std::nullopt, "transform outside history: " + frame};
                if (upper->time_ns == time_ns) {
                    transform = upper->pose;
                } else {
                    const auto &lower = *std::prev(upper);
                    const double alpha = static_cast<double>(time_ns - lower.time_ns) /
                                         static_cast<double>(upper->time_ns - lower.time_ns);
                    transform.translation =
                        (1 - alpha) * lower.pose.translation + alpha * upper->pose.translation;
                    transform.rotation = lower.pose.rotation.slerp(alpha, upper->pose.rotation);
                }
            }
            result = compose(transform, result);
            frame = edge.parent;
        }
        return {result, {}};
    };
    const auto source = toCommon(from);
    if (!source.pose)
        return source;
    const auto destination = toCommon(target);
    if (!destination.pose)
        return destination;
    return {compose(inverse(*destination.pose), *source.pose), {}};
}
} // namespace robotics::visualization

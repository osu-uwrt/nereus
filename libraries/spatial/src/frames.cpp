#include <cmath>
#include <robotics/spatial/frames.hpp>
#include <set>
#include <stdexcept>
#include <utility>

namespace robotics::spatial {
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
FixedFrames::FixedFrames(std::string root, std::vector<FixedFrame> edges)
    : root_(std::move(root)), edges_(std::move(edges)) {
    if (root_.empty() || edges_.size() > 4096)
        throw std::invalid_argument("fixed frames require a root and at most 4096 edges");
    std::map<std::string, std::size_t> indices;
    for (std::size_t i = 0; i < edges_.size(); ++i) {
        auto &edge = edges_[i];
        if (edge.parent.empty() || edge.child.empty() || edge.child == root_ || !indices.emplace(edge.child, i).second)
            throw std::invalid_argument("invalid or duplicate fixed frame: " + edge.child);
        validate(edge.pose);
        edge.pose.rotation.normalize();
    }
    resolved_.emplace(root_, Pose{});
    for (const auto &edge : edges_) {
        std::vector<std::size_t> path;
        std::set<std::string> visited;
        auto frame = edge.child;
        while (resolved_.count(frame) == 0) {
            if (!visited.insert(frame).second)
                throw std::invalid_argument("fixed frame cycle at " + frame);
            const auto found = indices.find(frame);
            if (found == indices.end())
                throw std::invalid_argument("unknown fixed frame parent: " + frame);
            path.push_back(found->second);
            frame = edges_[found->second].parent;
        }
        auto pose = resolved_.at(frame);
        for (auto item = path.rbegin(); item != path.rend(); ++item) {
            const auto &next = edges_[*item];
            pose = compose(pose, next.pose);
            pose.rotation.normalize();
            validate(pose);
            resolved_.emplace(next.child, pose);
        }
    }
}
const std::string &FixedFrames::root() const {
    return root_;
}
const std::vector<FixedFrame> &FixedFrames::edges() const {
    return edges_;
}
const Pose &FixedFrames::fromRoot(const std::string &frame) const {
    const auto found = resolved_.find(frame);
    if (found == resolved_.end())
        throw std::invalid_argument("unknown fixed frame: " + frame);
    return found->second;
}
Pose FixedFrames::lookup(const std::string &target, const std::string &from) const {
    return compose(inverse(fromRoot(target)), fromRoot(from));
}
} // namespace robotics::spatial

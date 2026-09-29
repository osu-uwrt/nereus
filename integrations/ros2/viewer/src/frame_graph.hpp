// Fixed frame tree from a robot pack (frames.transforms): child pose expressed in its parent.
#pragma once
#include "math.hpp"
#include <map>
#include <set>
#include <string>

namespace robotics::ros_viewer::host {
class FrameGraph {
  public:
    void add(const std::string &parent, const std::string &child, const glm::mat4 &childInParent) {
        if (parent.empty() || child.empty() || parent == child)
            throw std::invalid_argument("invalid frame edge " + parent + " -> " + child);
        if (parents_.count(child))
            throw std::invalid_argument("frame '" + child + "' has two parents");
        parents_[child] = {parent, childInParent};
        known_.insert(parent);
        known_.insert(child);
    }
    bool has(const std::string &name) const {
        return known_.count(name) > 0;
    }
    const std::set<std::string> &names() const {
        return known_;
    }
    // Pose of `to` expressed in `from`; both must share a root.
    glm::mat4 relative(const std::string &from, const std::string &to) const {
        return glm::inverse(inRoot(from)) * inRoot(to);
    }

  private:
    glm::mat4 inRoot(const std::string &name) const {
        if (!has(name))
            throw std::out_of_range("unknown frame '" + name + "'");
        glm::mat4 result(1);
        std::set<std::string> visited;
        std::string current = name;
        for (auto it = parents_.find(current); it != parents_.end(); it = parents_.find(current)) {
            if (!visited.insert(current).second)
                throw std::invalid_argument("frame cycle at '" + current + "'");
            result = it->second.pose * result;
            current = it->second.parent;
        }
        return result;
    }
    struct Edge {
        std::string parent;
        glm::mat4 pose{1};
    };
    std::map<std::string, Edge> parents_;
    std::set<std::string> known_;
};
} // namespace robotics::ros_viewer::host

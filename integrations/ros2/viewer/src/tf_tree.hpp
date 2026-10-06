// TF frame visibility tree for the TF window: parent links from TF, per-frame and per-branch show / hide.
#pragma once

#include <map>
#include <set>
#include <string>
#include <vector>

namespace nereus::ros_viewer::host {
// Selection belongs to frame IDs, so it survives reparenting and temporary
// disappearance. Parent relationships come from TF, not slash-separated names.
class TfTree {
  public:
    struct Frame {
        std::string parent;
        bool enabled = true;    // shown in the TF overlay
        bool available = false; // resolvable this frame (set by the caller after update())
    };

    std::map<std::string, Frame> frames; // every frame ever seen, by name
    std::map<std::string, std::vector<std::string>> children;
    std::vector<std::string> roots;

    // Rebuild the hierarchy from child -> parent links. New frames take the last select-all state; known frames
    // keep their selection.
    void update(const std::map<std::string, std::string> &parents) {
        children.clear();
        roots.clear();
        for (const auto &[name, parent] : parents) {
            auto [it, inserted] = frames.try_emplace(name);
            if (inserted)
                it->second.enabled = defaultEnabled;
            it->second.parent = parent;
            it->second.available = false;
        }
        // Cut malformed cycles so every frame remains reachable in the UI.
        auto treeParents = parents;
        for (const auto &[name, parent] : parents) {
            std::set<std::string> ancestors{name};
            auto current = parent;
            while (!current.empty() && treeParents.count(current)) {
                if (!ancestors.insert(current).second) {
                    treeParents[name].clear();
                    break;
                }
                current = treeParents.at(current);
            }
        }

        // Frames whose parent is unknown (or cut) are roots.
        for (const auto &[name, parent] : treeParents) {
            if (parent.empty() || !parents.count(parent))
                roots.push_back(name);
            else
                children[parent].push_back(name);
        }
    }

    // Show or hide every frame, including frames that appear later.
    void selectAll(bool enabled) {
        defaultEnabled = enabled;
        for (auto &[name, frame] : frames)
            frame.enabled = enabled;
    }

    enum class Selection { Hidden, Shown, Mixed };

    // Combined state of `name` and all its descendants.
    Selection branchSelection(const std::string &name) const {
        bool any = false, all = true;
        std::set<std::string> visited;
        std::vector<std::string> pending{name};
        while (!pending.empty()) {
            auto current = pending.back();
            pending.pop_back();
            if (!visited.insert(current).second)
                continue;
            const bool enabled = frames.at(current).enabled;
            any = any || enabled;
            all = all && enabled;
            const auto found = children.find(current);
            if (found != children.end())
                pending.insert(pending.end(), found->second.begin(), found->second.end());
        }
        return all ? Selection::Shown : any ? Selection::Mixed : Selection::Hidden;
    }

    // Show or hide `name` and all its descendants.
    void selectBranch(const std::string &name, bool enabled) {
        std::set<std::string> visited;
        std::vector<std::string> pending{name};
        while (!pending.empty()) {
            auto current = pending.back();
            pending.pop_back();
            if (!visited.insert(current).second)
                continue;
            frames.at(current).enabled = enabled;
            const auto found = children.find(current);
            if (found != children.end())
                pending.insert(pending.end(), found->second.begin(), found->second.end());
        }
    }

  private:
    bool defaultEnabled = true;
};
} // namespace nereus::ros_viewer::host

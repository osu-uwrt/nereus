#pragma once
#include <robotics/rendering/scene.hpp>
#include <robotics/visualization/source.hpp>

namespace robotics::scene_view {
// A group is resolved atomically. Empty source means a fixed-frame asset, not
// permission to align unrelated source frames that happen to share a name.
struct Group {
    std::string id, source, frame;
    rendering::Scene content;
};
struct Document {
    std::vector<Group> groups;
};
struct Resolved {
    rendering::Scene scene;
    std::map<std::string, std::string> issues;
};
// No retained pose/time cache: disconnect, source changes and rewinds cannot reuse
// a previous transform. Instances share immutable CPU meshes with the document.
Resolved resolve(const Document &, const std::string &fixed_frame,
                 const visualization::SourceSnapshot *source);
// Version 1 YAML; resource paths are relative to the document. Loads CPU data only.
Document load(const std::filesystem::path &path);
} // namespace robotics::scene_view

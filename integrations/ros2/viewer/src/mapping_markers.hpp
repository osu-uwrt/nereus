// The stack's RViz course markers (riptide_rviz config/markers.yaml, published there by MarkerPublisher.py):
// one riptide_meshes model per mapping TF frame. The viewer reads the same file and draws each mesh at the
// latest transform of its frame, so the course follows the mapping estimate exactly as RViz shows it.
#pragma once
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <functional>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace nereus::ros_viewer::host {
struct MappingMarker {
    std::string name, mesh, frame;
    std::string label;          // display name: the frame without its "_frame" suffix (unique, unlike meshes)
    std::filesystem::path path; // resolved model.dae
    glm::mat4 local{1};         // pose (xyz, roll-pitch-yaw) * scale, in the marker frame
    bool visible = true;        // operator toggle (Pool Viewer > Course > Meshes)
};
// `share(package)` resolves a ROS package share directory (used for the file's mesh_pkg unless `meshes`
// names a local folder of <mesh>/model.dae). Marker entries whose mesh is a builtin shape (arrow,
// sphere, cube) are skipped: only mesh markers make up the course.
inline std::vector<MappingMarker>
loadMappingMarkers(const std::filesystem::path &file,
                   const std::function<std::filesystem::path(const std::string &)> &share,
                   const std::filesystem::path &meshesOverride = {}) {
    const YAML::Node root = YAML::LoadFile(file.string());
    if (!root.IsMap() || root.size() != 1)
        throw std::runtime_error(file.string() + ": expected one node's ros__parameters");
    const YAML::Node params = root.begin()->second["ros__parameters"];
    const auto meshes = !meshesOverride.empty() ? meshesOverride
                                                : share(params["mesh_pkg"].as<std::string>()) /
                                                      params["mesh_directory"].as<std::string>("meshes");
    std::vector<MappingMarker> out;
    // MarkerPublisher reads marker0, marker1, ... until the first incomplete entry.
    for (int i = 0;; ++i) {
        const YAML::Node entry = params["markers"]["marker" + std::to_string(i)];
        if (!entry || entry["mesh"].as<std::string>("").empty() || entry["frame"].as<std::string>("").empty())
            break;
        MappingMarker marker;
        marker.name = "marker" + std::to_string(i);
        marker.mesh = entry["mesh"].as<std::string>();
        marker.frame = entry["frame"].as<std::string>();
        constexpr std::string_view suffix = "_frame";
        marker.label = marker.frame.size() > suffix.size() &&
                               marker.frame.compare(marker.frame.size() - suffix.size(), suffix.size(), suffix) == 0
                           ? marker.frame.substr(0, marker.frame.size() - suffix.size())
                           : marker.frame;
        if (marker.mesh == "arrow" || marker.mesh == "sphere" || marker.mesh == "cube")
            continue;
        marker.path = meshes / marker.mesh / "model.dae";
        const auto pose = entry["pose"].as<std::vector<double>>(std::vector<double>(6, 0.));
        const auto scale = entry["scale"].as<std::vector<double>>(std::vector<double>(3, 1.));
        if (pose.size() != 6 || scale.size() != 3)
            throw std::runtime_error(file.string() + ": " + marker.name + " needs pose[6] and scale[3]");
        // transforms3d euler2quat(roll, pitch, yaw) with the default static xyz axes: Rz * Ry * Rx.
        const glm::quat rotation = glm::angleAxis(float(pose[5]), glm::vec3(0, 0, 1)) *
                                   glm::angleAxis(float(pose[4]), glm::vec3(0, 1, 0)) *
                                   glm::angleAxis(float(pose[3]), glm::vec3(1, 0, 0));
        marker.local = glm::translate(glm::mat4(1), glm::vec3(pose[0], pose[1], pose[2])) * glm::mat4_cast(rotation) *
                       glm::scale(glm::mat4(1), glm::vec3(scale[0], scale[1], scale[2]));
        out.push_back(std::move(marker));
    }
    return out;
}
inline std::string lowercase(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return char(std::tolower(c)); });
    return text;
}
// Alphabetical by label, ignoring case (the operator's list; draw order does not matter).
inline void sortMappingMarkers(std::vector<MappingMarker> &markers) {
    std::stable_sort(markers.begin(), markers.end(), [](const MappingMarker &a, const MappingMarker &b) {
        return lowercase(a.label) < lowercase(b.label);
    });
}
// The list filter: `query` found in the label, mesh or frame, ignoring case; an empty query matches all.
inline bool mappingMarkerMatches(const MappingMarker &marker, const std::string &query) {
    const auto needle = lowercase(query);
    return needle.empty() || lowercase(marker.label).find(needle) != std::string::npos ||
           lowercase(marker.mesh).find(needle) != std::string::npos ||
           lowercase(marker.frame).find(needle) != std::string::npos;
}
// Hides the markers each name matches: a frame, a label or a mesh (a mesh name hides every copy, e.g. all
// slalom posts). Returns the names that matched nothing.
inline std::vector<std::string> hideMappingMarkers(std::vector<MappingMarker> &markers,
                                                   const std::vector<std::string> &names) {
    std::vector<std::string> unknown;
    for (const auto &name : names) {
        bool found = false;
        for (auto &marker : markers)
            if (marker.frame == name || marker.label == name || marker.mesh == name) {
                marker.visible = false;
                found = true;
            }
        if (!found)
            unknown.push_back(name);
    }
    return unknown;
}
} // namespace nereus::ros_viewer::host

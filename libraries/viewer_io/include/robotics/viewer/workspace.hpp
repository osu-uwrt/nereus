#pragma once
#include <robotics/visualization/camera.hpp>
#include <robotics/visualization/display.hpp>

#include <filesystem>

namespace robotics::viewer {
using Camera = visualization::Camera;
struct SourceBinding {
    std::string id;
    std::string type{"local"};
    std::filesystem::path file;
};
struct Workspace {
    std::vector<SourceBinding> sources;
    std::string selected_source;
    std::string fixed_frame{"world"};
    std::vector<visualization::DisplaySettings> displays;
    Camera camera;
    std::filesystem::path scene; // Optional neutral visual document; no simulation configuration.
};
Workspace emptyWorkspace();
Workspace loadWorkspace(const std::filesystem::path &path);
std::string serializeWorkspace(const Workspace &workspace,
                               const std::filesystem::path &destination);
visualization::Recording loadRecording(const std::filesystem::path &path);
} // namespace robotics::viewer

// Screen-space overlays drawn over the viewport image with the ImGui window draw list.
#pragma once
#include "ros_side.hpp"
#include <imgui.h>

namespace robotics::ros_viewer::host {
struct ScreenRect {
    ImVec2 position{0, 0};
    float width = 1, height = 1;
};
bool projectToScreen(const glm::mat4 &viewProjection, const ScreenRect &rect, const glm::vec4 &world, ImVec2 &pixel);
ImU32 rgba(float r, float g, float b, float a);
void drawDetections(const std::vector<PlacedDetection> &, const glm::mat4 &viewProjection, const ScreenRect &);
void drawMpcPath(const std::vector<glm::mat4> &, const glm::mat4 &viewProjection, const ScreenRect &);
struct TfOverlay {
    const TfSnapshot *snapshot = nullptr;
    float axisLength = .12f;
    bool names = true;
    std::string caption;
    ImFont *font = nullptr;
};
void drawTfAxes(const TfOverlay &, const glm::mat4 &viewProjection, const ScreenRect &);
// Frame tree table with tri-state branch selection; `root` is the fixed frame (opened by default).
void drawTfTree(TfTree &, const std::string &root);
} // namespace robotics::ros_viewer::host

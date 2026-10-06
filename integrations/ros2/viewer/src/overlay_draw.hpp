// Screen-space overlays drawn over the viewport image with the ImGui window draw list.
#pragma once
#include "ros_side.hpp"
#include <imgui.h>

namespace nereus::ros_viewer::host {
// The viewport image's rectangle in ImGui screen pixels.
struct ScreenRect {
    ImVec2 position{0, 0};
    float width = 1, height = 1;
};

// World point -> pixel in `rect`; false when it is behind the camera or outside the depth range.
bool projectToScreen(const glm::mat4 &viewProjection, const ScreenRect &rect, const glm::vec4 &world, ImVec2 &pixel);
ImU32 rgba(float r, float g, float b, float a);

// `both`: truth and estimate placements are shown together, so estimate ones get a distinct cyan outline.
// Approximate estimate placements (TF had not reached the stamp) are dim and dashed. A legend explains the styles.
void drawDetections(const std::vector<PlacedDetection> &, const glm::mat4 &viewProjection, const ScreenRect &,
                    bool both = false);

// The MPC's predicted horizon: orange line through the stage poses with heading ticks.
void drawMpcPath(const std::vector<glm::mat4> &, const glm::mat4 &viewProjection, const ScreenRect &);

// A whole follow_path plan: thin blue line with heading ticks, ring at the end.
void drawPlannedPath(const std::vector<glm::mat4> &, const glm::mat4 &viewProjection, const ScreenRect &);

// Thruster forces as arrows from each thruster along its axis, `metersPerNewton` long per newton (negative forces
// point backwards), like RViz's wrench displays. `forces` is in bridge order; `body` is the drawn robot.
void drawThrust(const std::vector<ThrusterMount> &, const std::vector<float> &forces, const glm::mat4 &body,
                float metersPerNewton, const glm::mat4 &viewProjection, const ScreenRect &);

// What drawTfAxes draws: the snapshot's frames as RGB axes (meters long), optional names, and a caption.
// `font` must be set (it is dereferenced).
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
} // namespace nereus::ros_viewer::host

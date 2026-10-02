// Compact status chips for header and toolbar forms (telemetry readings, recording indicators).
#pragma once
#include "nereus/ros_viewer/panels/capabilities.hpp"
#include <imgui.h>

namespace nereus::ros_viewer::panels {
inline ImVec4 levelColor(Level level) {
    switch (level) {
    case Level::Ok:
        return {.32f, .86f, .82f, 1};
    case Level::Warn:
        return {.94f, .73f, .35f, 1};
    case Level::Error:
        return {.96f, .36f, .33f, 1};
    default:
        return {.47f, .57f, .64f, 1};
    }
}
inline const char *levelName(Level level) {
    switch (level) {
    case Level::Ok:
        return "OK";
    case Level::Warn:
        return "Warning";
    case Level::Error:
        return "Error";
    default:
        return "No data";
    }
}
inline float chipWidth(const char *text) {
    return ImGui::CalcTextSize(text).x + 2 * ImGui::GetStyle().FramePadding.x;
}
// A dim, tinted button that reads as a label (same look as the viewer's status pill); returns hover.
inline bool statusChip(const char *text, ImVec4 tint) {
    const ImVec4 fill(tint.x * .15f, tint.y * .15f, tint.z * .15f, 1);
    ImGui::PushStyleColor(ImGuiCol_Button, fill);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, fill);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, fill);
    ImGui::PushStyleColor(ImGuiCol_Text, tint);
    ImGui::Button(text);
    ImGui::PopStyleColor(4);
    return ImGui::IsItemHovered();
}
} // namespace nereus::ros_viewer::panels

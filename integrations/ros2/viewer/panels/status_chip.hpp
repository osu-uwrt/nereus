// Compact status chips for header and toolbar forms (telemetry readings, recording indicators).
#pragma once
#include "nereus/ros_viewer/panels/capabilities.hpp"
#include "nereus/ros_viewer/theme.hpp"
#include <imgui.h>

namespace nereus::ros_viewer::panels {
inline ImVec4 levelColor(Level level) {
    const auto &p = palette();
    switch (level) {
    case Level::Ok:
        return p.accent;
    case Level::Warn:
        return p.warn;
    case Level::Error:
        return p.error;
    default:
        return p.muted;
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
    const ImVec4 fill = tintedFill(tint);
    ImGui::PushStyleColor(ImGuiCol_Button, fill);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, fill);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, fill);
    ImGui::PushStyleColor(ImGuiCol_Text, tint);
    ImGui::Button(text);
    ImGui::PopStyleColor(4);
    return ImGui::IsItemHovered();
}
} // namespace nereus::ros_viewer::panels

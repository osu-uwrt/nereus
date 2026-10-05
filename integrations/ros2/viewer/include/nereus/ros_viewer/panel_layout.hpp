#pragma once
#include "nereus/ros_viewer/theme.hpp"
#include <algorithm>
#include <imgui.h>
namespace nereus::ros_viewer {
inline void sameLineIfFits(float width) {
    const float right = ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x;
    if (ImGui::GetItemRectMax().x + ImGui::GetStyle().ItemSpacing.x + width <= right)
        ImGui::SameLine();
}

inline float buttonWidth(const char *label) {
    return ImGui::CalcTextSize(label, nullptr, true).x + 2 * ImGui::GetStyle().FramePadding.x;
}

// The theme's fill for an active state (selected mode, toggle on): one place so every "on" looks the same.
// Pushes four colours; pop them with popActiveColors().
inline void pushActiveColors(bool active) {
    const auto &p = palette();
    const auto &colors = ImGui::GetStyle().Colors;
    ImGui::PushStyleColor(ImGuiCol_Button, active ? p.active : colors[ImGuiCol_Button]);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, active ? p.activeHovered : colors[ImGuiCol_ButtonHovered]);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, active ? p.activePressed : colors[ImGuiCol_ButtonActive]);
    ImGui::PushStyleColor(ImGuiCol_Text, active ? p.activeText : colors[ImGuiCol_Text]);
}
inline void popActiveColors() {
    ImGui::PopStyleColor(4);
}

// A toolbar toggle drawn as a button that is filled while on (reads at a glance, unlike a checkbox row).
// Wraps to the next line when it does not fit. Returns true when clicked.
inline bool toggleChip(const char *label, bool *value) {
    sameLineIfFits(buttonWidth(label));
    pushActiveColors(*value);
    const bool clicked = ImGui::Button(label);
    popActiveColors();
    if (clicked)
        *value = !*value;
    return clicked;
}

// A toggle chip with a small arrow beside it for its settings. Returns true when the arrow was clicked.
inline bool toggleChipWithMenu(const char *label, bool *value, const char *menuId) {
    const float arrow = ImGui::GetFrameHeight();
    sameLineIfFits(buttonWidth(label) + arrow);
    pushActiveColors(*value);
    if (ImGui::Button(label))
        *value = !*value;
    ImGui::SameLine(0, 1);
    ImGui::PushID(menuId);
    const bool menu = ImGui::ArrowButton("##settings", ImGuiDir_Down);
    ImGui::PopID();
    popActiveColors();
    return menu;
}

// A toolbar button that shows/hides a tool window; filled while the window is open.
inline bool windowToggle(const char *label, bool *open) {
    sameLineIfFits(buttonWidth(label));
    pushActiveColors(*open);
    const bool clicked = ImGui::Button(label);
    popActiveColors();
    if (clicked)
        *open = !*open;
    return clicked;
}

// A dockable tool window opened from a toolbar button or the Windows menu. The first time it opens it floats
// under the button that opened it (anchor, in screen coordinates) at `size` (height 0: fit the contents once);
// afterwards the operator places it (float, dock, tab) and the layout remembers. A floating window is kept
// inside the application window. Call End() whatever this returns, as with ImGui::Begin.
inline bool beginToolWindow(const char *name, bool *open, ImVec2 size, ImVec2 anchor, bool focus = false) {
    const float margin = ui(12);
    const auto *viewport = ImGui::GetMainViewport();
    const ImVec2 low(viewport->WorkPos.x + margin, viewport->WorkPos.y + margin);
    const ImVec2 available(std::max(1.f, viewport->WorkSize.x - 2 * margin),
                           std::max(1.f, viewport->WorkSize.y - 2 * margin));
    const float width = std::min(size.x, available.x);
    ImGui::SetNextWindowSize({width, std::min(size.y, available.y)}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos({std::clamp(anchor.x, low.x, low.x + std::max(0.f, available.x - width)),
                             std::clamp(anchor.y, low.y, low.y + std::max(0.f, available.y - ui(120)))},
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSizeConstraints({std::min(ui(240), available.x), std::min(ui(100), available.y)}, available);
    if (focus) {
        ImGui::SetNextWindowFocus();
        ImGui::SetNextWindowCollapsed(false);
    }
    const bool visible = ImGui::Begin(name, open);
    if (!ImGui::IsWindowDocked() && !ImGui::IsWindowAppearing()) {
        const auto pos = ImGui::GetWindowPos(), extent = ImGui::GetWindowSize();
        const ImVec2 clamped(std::clamp(pos.x, low.x, low.x + std::max(0.f, available.x - extent.x)),
                             std::clamp(pos.y, low.y, low.y + std::max(0.f, available.y - extent.y)));
        if (clamped.x != pos.x || clamped.y != pos.y)
            ImGui::SetWindowPos(clamped);
    }
    return visible;
}
} // namespace nereus::ros_viewer

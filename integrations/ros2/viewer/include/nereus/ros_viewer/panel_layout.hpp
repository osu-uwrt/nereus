// Small ImGui layout helpers shared by the viewer's panels and toolbars: wrapping toolbar rows, themed toggle
// buttons and tool windows that open under their button.
#pragma once
#include "nereus/ros_viewer/theme.hpp"
#include <algorithm>
#include <imgui.h>
namespace nereus::ros_viewer {

// Continue the current row when an item `width` pixels wide still fits before the window's right edge; otherwise
// let ImGui start a new line.
inline void sameLineIfFits(float width) {
    const float right = ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x;
    if (ImGui::GetItemRectMax().x + ImGui::GetStyle().ItemSpacing.x + width <= right)
        ImGui::SameLine();
}

// Width of a default-sized button with this label (text before "##" only, plus frame padding).
inline float buttonWidth(const char *label) {
    return ImGui::CalcTextSize(label, nullptr, true).x + 2 * ImGui::GetStyle().FramePadding.x;
}

// The theme's fill for an active state (selected mode, toggle on): one place so every "on" looks the same.
// Pushes four colors; pop them with popActiveColors().
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

// Records the item just drawn as a button that toggles the tool window `open` belongs to. A click on it is that
// button's to handle, so it is not a click outside the window (which would close it, and the button's release
// would open it again).
void markToolWindowButton(const bool *open);

// A toolbar button that shows/hides a tool window; filled while the window is open.
inline bool windowToggle(const char *label, bool *open) {
    sameLineIfFits(buttonWidth(label));
    pushActiveColors(*open);
    const bool clicked = ImGui::Button(label);
    popActiveColors();
    markToolWindowButton(open);
    if (clicked)
        *open = !*open;
    return clicked;
}

// A dockable tool window opened from a toolbar button or the Windows menu. The first time it opens it floats
// under the button that opened it (anchor, in screen coordinates), `size.x` wide (size.y is unused); afterwards
// the operator places it (float, dock, tab) and the layout remembers. A floating window is kept inside the
// application window and sized to show all its contents as far as the window allows: each time it opens its height
// fits them (its width grows to them), and while open it grows with them but is never shrunk under the operator. It
// closes at a click anywhere outside it, unless the pin in its title bar keeps it open; a docked one stays until
// closed. Call End() whatever this returns, as with ImGui::Begin.
bool beginToolWindow(const char *name, bool *open, ImVec2 size, ImVec2 anchor, bool focus = false);

// Registers the ini handler that keeps the tool windows' pins ([NereusToolWindows]), so a saved layout carries
// them. Once per ImGui context.
void installToolWindows();
} // namespace nereus::ros_viewer

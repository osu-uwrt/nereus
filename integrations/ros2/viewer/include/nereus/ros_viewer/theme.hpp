// Viewer themes: the whole ImGui style plus the few colours the viewer and its panels draw themselves (accent,
// "on" fills, KILL, robot state, bars). The 3D view and its overlays keep their own colours in every theme.
#pragma once
#include <imgui.h>
#include <string>
#include <vector>

namespace nereus::ros_viewer {
struct Palette {
    ImVec4 text, muted;                                      // body text, secondary text
    ImVec4 accent;                                           // highlights: headings, OK status, selected tab line
    ImVec4 active, activeHovered, activePressed, activeText; // filled "on" buttons and toggles
    ImVec4 danger, dangerHovered, dangerPressed, dangerText; // KILL, power cuts
    ImVec4 warn, error;                                      // status levels
    ImVec4 robotEnabled, robotKilled;                        // robot state beside Enable / KILL
    ImVec4 bar, toolbar;                                     // menu / command bar, pool view toolbar strip
};
struct Theme {
    std::string id, label, description;
    std::string fontFamily; // empty: the viewer's own font (DejaVu Sans)
    float fontPoints = 0;   // the family's size in points (at 96 dpi), 0: the viewer's sizes
};
// Built-in themes, the default (abyss) first.
const std::vector<Theme> &themes();
const Theme &currentThemeInfo();
// Applies a theme to the current ImGui context's style and to palette(); false (and no change) if unknown.
bool applyTheme(const std::string &id);
const std::string &currentTheme();
const Palette &palette();
// Interface scale (1 = the viewer's own 100 % sizes): the style's sizes and every fixed size written with ui().
// Re-applies the current theme; the host rebuilds its fonts at the same scale.
void setInterfaceScale(float scale);
float interfaceScale();
inline float ui(float pixels) {
    return pixels * interfaceScale();
}
inline ImVec2 ui(ImVec2 pixels) {
    return {ui(pixels.x), ui(pixels.y)};
}
// The window background tinted a little toward `tint` (status chips and pills): a dark wash on dark themes, a
// pale one on light themes.
inline ImVec4 tintedFill(ImVec4 tint) {
    const auto bg = ImGui::GetStyle().Colors[ImGuiCol_WindowBg];
    return {bg.x * .82f + tint.x * .18f, bg.y * .82f + tint.y * .18f, bg.z * .82f + tint.z * .18f, 1};
}
} // namespace nereus::ros_viewer

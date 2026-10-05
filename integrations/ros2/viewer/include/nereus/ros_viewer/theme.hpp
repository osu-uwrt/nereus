// Viewer themes: the whole ImGui style plus the few colours the viewer and its panels draw themselves (accent,
// "on" fills, KILL, robot state, bars). The 3D view and its overlays keep their own colours in every theme.
#pragma once
#include <cstring>
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
// WCAG contrast ratio of two colours (1..21).
float contrastRatio(ImVec4 a, ImVec4 b);
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
// The host's fonts beyond the body font, at the interface scale (null until the host loads them: the current font).
struct TypeRamp {
    ImFont *strong = nullptr; // the body size in bold: section titles, table headers
    ImFont *number = nullptr; // large figures: run time, score
    ImFont *small = nullptr;  // captions
    ImFont *smallStrong = nullptr; // chart labels
};
void setTypeRamp(const TypeRamp &);
const TypeRamp &typeRamp();
// A section title: bold body text over a rule (ImGui::SeparatorText in the strong font).
void sectionTitle(const char *text);
// A table's header row in the strong font.
void tableHeaders();
// A panel's explanation of what it shows once connected (muted, wrapped).
inline void emptyState(const char *text) {
    ImGui::PushStyleColor(ImGuiCol_Text, palette().muted);
    ImGui::TextWrapped("%s", text);
    ImGui::PopStyleColor();
}
// A status chip: an outlined capsule with a status dot and the text in the status colour. It reads as a label, not
// a button (buttons are filled rectangles); returns hover for a tooltip.
inline float statusChipWidth(const char *text) {
    return ImGui::CalcTextSize(text, nullptr, true).x + 2 * ImGui::GetStyle().FramePadding.x + ui(13);
}
inline bool statusChip(const char *text, ImVec4 tint) {
    const ImVec2 at = ImGui::GetCursorScreenPos(), size{statusChipWidth(text), ImGui::GetFrameHeight()};
    ImGui::InvisibleButton("##status_chip", size);
    auto *draw = ImGui::GetWindowDrawList();
    const ImVec2 end{at.x + size.x, at.y + size.y};
    const float radius = size.y * .5f, pad = ImGui::GetStyle().FramePadding.x, dot = ui(3.5f);
    draw->AddRectFilled(at, end, ImGui::GetColorU32({tint.x, tint.y, tint.z, .08f}), radius);
    draw->AddRect(at, end, ImGui::GetColorU32({tint.x, tint.y, tint.z, .6f}), radius, 0, ui(1));
    draw->AddCircleFilled({at.x + pad + dot, at.y + radius}, dot, ImGui::GetColorU32(tint));
    const char *shown = std::strstr(text, "##");
    draw->AddText({at.x + pad + ui(13), at.y + (size.y - ImGui::GetFontSize()) * .5f}, ImGui::GetColorU32(tint), text,
                  shown);
    return ImGui::IsItemHovered();
}
} // namespace nereus::ros_viewer

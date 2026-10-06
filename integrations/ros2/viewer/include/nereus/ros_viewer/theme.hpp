// Viewer themes: the whole ImGui style plus the few colors the viewer and its panels draw themselves (accent,
// "on" fills, KILL, robot state, bars). The 3D view and its overlays keep their own colors in every theme.
#pragma once
#include <array>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <imgui.h>
#include <string>
#include <vector>

namespace nereus::ros_viewer {
// The colors the viewer draws itself, per theme (ImGui's own colors live in its style).
struct Palette {
    ImVec4 text, muted;                                      // body text, secondary text
    ImVec4 accent;                                           // highlights: headings, OK status, selected tab line
    ImVec4 active, activeHovered, activePressed, activeText; // filled "on" buttons and toggles
    ImVec4 danger, dangerHovered, dangerPressed, dangerText; // KILL, power cuts
    ImVec4 warn, error;                                      // status levels
    ImVec4 robotEnabled, robotKilled;                        // robot state beside Enable / KILL
    ImVec4 bar, toolbar;                                     // menu / command bar, pool view toolbar strip
    ImVec4 change, changeText;                               // the lamp (a running clock) and its ink
    ImVec4 enable, enableHovered, enablePressed, enableText; // Enable (the robot is off): its own action ink
    // Plot lines in a fixed order (a theme's `plot: series:`); unset (alpha 0) where the theme gives none, and the
    // plots then use a validated set for light or dark windows.
    std::array<ImVec4, 8> series{};
};

// A theme's identity and fonts, as listed in the theme menu.
struct Theme {
    std::string id, label, description;
    std::string fontFamily; // empty: the viewer's own font (DejaVu Sans)
    float fontPoints = 0;   // the family's size in points (at 96 dpi), 0: the viewer's sizes
    // Font files shipped with the viewer (content/viewer/fonts), in place of a family: body, bold, large figures.
    std::string fontRegular, fontStrong, fontFigures;
};

// Themes are data: one YAML file each (content/viewer/themes). loadThemes reads a directory (replacing the
// built-in fallback) and returns a warning per file it could not use; themes() lists them by their `order`, the
// default first.
std::vector<std::string> loadThemes(const std::filesystem::path &directory);
const std::vector<Theme> &themes();
const Theme &currentThemeInfo();
// Applies a theme to the current ImGui context's style and to palette(); false (and no change) if unknown.
bool applyTheme(const std::string &id);
const std::string &currentTheme();
const Palette &palette();

// WCAG contrast ratio of two colors (1..21).
float contrastRatio(ImVec4 a, ImVec4 b);
// `ink` where it reads on `fill` (at least `ratio`:1), else white or black, whichever reads better: text drawn over
// a highlight (a selected or hovered row) in any theme, e.g. Classic's black ink on its navy selection.
inline ImVec4 inkOn(ImVec4 fill, ImVec4 ink, float ratio = 4.5f) {
    if (contrastRatio(ink, fill) >= ratio)
        return ink;
    const ImVec4 white(1, 1, 1, ink.w), black(0, 0, 0, ink.w);
    return contrastRatio(white, fill) >= contrastRatio(black, fill) ? white : black;
}

// Interface scale (1 = the viewer's own 100 % sizes): the style's sizes and every fixed size written with ui().
// Re-applies the current theme; the host rebuilds its fonts at the same scale.
void setInterfaceScale(float scale);
float interfaceScale();
// A size in 100 %-scale pixels, scaled to the current interface scale.
inline float ui(float pixels) {
    return pixels * interfaceScale();
}
inline ImVec2 ui(ImVec2 pixels) {
    return {ui(pixels.x), ui(pixels.y)};
}

// The host's fonts beyond the body font, at the interface scale (null until the host loads them: the current font).
struct TypeRamp {
    ImFont *strong = nullptr;      // the body size in bold: section titles, table headers
    ImFont *number = nullptr;      // large figures: run time, score
    ImFont *small = nullptr;       // captions
    ImFont *smallStrong = nullptr; // chart labels
};
void setTypeRamp(const TypeRamp &);
const TypeRamp &typeRamp();

// A section title in the strong font: over a rule (ImGui::SeparatorText), or in a ruled theme the text with a
// heavy rule beneath it, the way a results sheet heads each event.
void sectionTitle(const char *text);
// A table's header row in the strong font (a ruled theme draws a heavy rule under it).
void tableHeaders();
// Under a header row drawn by hand (TableHeader per column): the ruled theme's heavy rule; nothing otherwise.
void ruleUnderHeaders();
// The theme draws heads ruled (heavy ink rules, no header fills).
bool ruledTheme();
// Corner radius of status chips: a capsule unless the theme squares them off.
float chipRounding(float height);

// Surfaces inside the theme. The board (menu and command bars) can carry its own palette: inside
// beginSurface(Surface::Board) .. endSurface(), palette() and ImGui's text, button, frame and popup colors are
// the board's; Surface::Sheet restores the panels' colors inside a board (a menu's dropdown). Nest freely.
enum class Surface { Sheet, Board };
void beginSurface(Surface);
void endSurface();
// Only a surface's popup colors (background, border): for a menu on the board whose dropdown is a sheet, pushed
// before BeginMenu (which creates the dropdown) while the menu's label keeps the board's colors.
void pushPopupColors(Surface);
void popPopupColors();

// A readout on the board (FOG 41.2°C, PORT 86 %): a level dot, the label in muted text, the value in bold (in
// `valueInk`); no box, so it reads as an instrument, not a button. Returns hover for a tooltip.
inline float readoutWidth(const char *label, const char *value) {
    float width = ui(13) + ImGui::CalcTextSize(label).x + ui(5);
    if (auto *font = typeRamp().strong)
        width += font->CalcTextSizeA(font->FontSize, 1e9f, 0, value).x;
    else
        width += ImGui::CalcTextSize(value).x;
    return width;
}
// Draws the readout described above, readoutWidth() wide.
inline bool statusReadout(const char *label, const char *value, ImVec4 dot, ImVec4 valueInk) {
    const ImVec2 at = ImGui::GetCursorScreenPos(), size{readoutWidth(label, value), ImGui::GetFrameHeight()};
    ImGui::InvisibleButton("##readout", size);
    auto *draw = ImGui::GetWindowDrawList();
    const float middle = at.y + size.y * .5f, radius = ui(3.5f);
    draw->AddCircleFilled({at.x + radius, middle}, radius, ImGui::GetColorU32(dot));
    const float labelX = at.x + ui(13);
    draw->AddText({labelX, middle - ImGui::GetFontSize() * .5f}, ImGui::GetColorU32(palette().muted), label);
    ImFont *font = typeRamp().strong ? typeRamp().strong : ImGui::GetFont();
    draw->AddText(font, font->FontSize, {labelX + ImGui::CalcTextSize(label).x + ui(5), middle - font->FontSize * .5f},
                  ImGui::GetColorU32(valueInk), value);
    return ImGui::IsItemHovered();
}

// A panel's explanation of what it shows once connected (muted, wrapped).
inline void emptyState(const char *text) {
    ImGui::PushStyleColor(ImGuiCol_Text, palette().muted);
    ImGui::TextWrapped("%s", text);
    ImGui::PopStyleColor();
}

// A theme with bevels (Classic: a border shadow): edges drawn raised (light above / left, shadow below / right) or
// sunken (the reverse), as Qt's Windows style draws buttons and fields.
inline bool bevelledTheme() {
    return ImGui::GetStyle().Colors[ImGuiCol_BorderShadow].w > 0;
}
// Draws the 1-pixel bevel inside the rectangle [min, max), raised or sunken.
inline void bevel(ImDrawList *draw, ImVec2 min, ImVec2 max, bool raised) {
    const ImU32 light = ImGui::GetColorU32(ImGuiCol_Border), shadow = ImGui::GetColorU32(ImGuiCol_BorderShadow);
    const ImU32 topLeft = raised ? light : shadow, bottomRight = raised ? shadow : light;
    draw->AddLine({min.x, max.y - 1}, {min.x, min.y}, topLeft);
    draw->AddLine({min.x, min.y}, {max.x - 1, min.y}, topLeft);
    draw->AddLine({max.x - 1, min.y}, {max.x - 1, max.y - 1}, bottomRight);
    draw->AddLine({max.x - 1, max.y - 1}, {min.x, max.y - 1}, bottomRight);
}

// A progress bar whose label stays at its left; over the filled part the label is drawn again in whichever of the
// theme's inks (text, window paper) stands out more from the fill, so it reads wherever the fill has reached.
// No label: the percentage.
inline void progressBar(float fraction, const char *label = nullptr) {
    fraction = fraction > 0 ? (fraction < 1 ? fraction : 1) : 0;
    char percent[16];
    if (!label) {
        std::snprintf(percent, sizeof(percent), "%.0f %%", double(fraction * 100));
        label = percent;
    }

    // Reserve a frame-high row across the available width, then draw the track, the fill and the border.
    const auto &style = ImGui::GetStyle();
    const ImVec2 at = ImGui::GetCursorScreenPos(), size{ImGui::GetContentRegionAvail().x, ImGui::GetFrameHeight()};
    ImGui::Dummy(size);
    auto *draw = ImGui::GetWindowDrawList();
    const ImVec2 end{at.x + size.x, at.y + size.y};
    const float filled = at.x + size.x * fraction, rounding = style.FrameRounding;
    draw->AddRectFilled(at, end, ImGui::GetColorU32(ImGuiCol_FrameBg), rounding);
    if (filled > at.x) { // the bar's own shape, cut at the fill
        draw->PushClipRect(at, {filled, end.y}, true);
        draw->AddRectFilled(at, end, ImGui::GetColorU32(ImGuiCol_PlotHistogram), rounding);
        draw->PopClipRect();
    }
    if (bevelledTheme())
        bevel(draw, at, end, false);
    else if (style.FrameBorderSize > 0)
        draw->AddRect(at, end, ImGui::GetColorU32(ImGuiCol_Border), rounding, 0, style.FrameBorderSize);

    // The label twice, clipped: normal ink over the empty part, the higher-contrast ink over the fill.
    const ImVec4 fill = style.Colors[ImGuiCol_PlotHistogram], ink = style.Colors[ImGuiCol_Text],
                 paper = style.Colors[ImGuiCol_WindowBg];
    const ImVec4 onFill =
        contrastRatio(ink, fill) >= contrastRatio(paper, fill) ? ink : ImVec4(paper.x, paper.y, paper.z, 1);
    const ImVec2 text{at.x + style.FramePadding.x, at.y + (size.y - ImGui::GetFontSize()) * .5f};
    draw->PushClipRect({filled, at.y}, end, true);
    draw->AddText(text, ImGui::GetColorU32(ink), label);
    draw->PopClipRect();
    if (filled > at.x) {
        draw->PushClipRect(at, {filled, end.y}, true);
        draw->AddText(text, ImGui::GetColorU32(onFill), label);
        draw->PopClipRect();
    }
}

// A status chip: an outlined capsule with a status dot and the text in the status color (in a bevelled theme, a
// sunken status-bar panel, as Qt draws one). It reads as a label, not a button; returns hover for a tooltip.
inline float statusChipWidth(const char *text) {
    return ImGui::CalcTextSize(text, nullptr, true).x + 2 * ImGui::GetStyle().FramePadding.x + ui(13);
}
// Draws the chip described above, statusChipWidth() wide; text after "##" is the ID only.
inline bool statusChip(const char *text, ImVec4 tint) {
    const ImVec2 at = ImGui::GetCursorScreenPos(), size{statusChipWidth(text), ImGui::GetFrameHeight()};
    ImGui::InvisibleButton("##status_chip", size);
    auto *draw = ImGui::GetWindowDrawList();
    const ImVec2 end{at.x + size.x, at.y + size.y};
    const float radius = chipRounding(size.y), pad = ImGui::GetStyle().FramePadding.x, dot = ui(3.5f);
    if (bevelledTheme()) {
        bevel(draw, at, end, false);
    } else {
        draw->AddRectFilled(at, end, ImGui::GetColorU32({tint.x, tint.y, tint.z, .08f}), radius);
        draw->AddRect(at, end, ImGui::GetColorU32({tint.x, tint.y, tint.z, .6f}), radius, 0, ui(1));
    }
    draw->AddCircleFilled({at.x + pad + dot, at.y + size.y * .5f}, dot, ImGui::GetColorU32(tint));
    const char *shown = std::strstr(text, "##");
    draw->AddText({at.x + pad + ui(13), at.y + (size.y - ImGui::GetFontSize()) * .5f}, ImGui::GetColorU32(tint), text,
                  shown);
    return ImGui::IsItemHovered();
}
} // namespace nereus::ros_viewer

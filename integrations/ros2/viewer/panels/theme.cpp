#include "nereus/ros_viewer/theme.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <sstream>

namespace nereus::ros_viewer {
namespace {
// The colours a theme chooses; everything else in ImGuiStyle derives from them.
struct Spec {
    ImVec4 window, child, popup, frame, frameHovered, frameActive, button, buttonHovered, buttonPressed, border, header,
        headerHovered, headerPressed, title, titleActive, tab, tabHovered, tabSelected, tabDimmed, tabDimmedSelected,
        scrollGrab, tableRowAlt;
    float windowRounding = 8, frameRounding = 5, tabRounding = 5, windowBorder = 1, frameBorder = 0;
    ImVec4 borderShadow{0, 0, 0, 0}; // drawn 1 px below / right of framed widgets: a raised bevel
    Palette palette;
};

Palette current;
std::string currentId;

ImVec4 alpha(ImVec4 c, float a) {
    return {c.x, c.y, c.z, a};
}

float scale = 1;
std::function<Spec()> currentSpec;

void apply(const Spec &t) {
    ImGuiStyle s; // from ImGui's defaults each time, so scaling never compounds
    // Sizes are shared by every theme so a layout looks the same in each; only corners and borders change.
    s.WindowPadding = {14, 12};
    s.FramePadding = {10, 7};
    s.ItemSpacing = {10, 9};
    s.WindowRounding = t.windowRounding;
    s.ChildRounding = t.windowRounding;
    s.PopupRounding = t.frameRounding + 1;
    s.FrameRounding = t.frameRounding;
    s.GrabRounding = t.frameRounding;
    s.ScrollbarRounding = t.frameRounding + 4;
    s.TabRounding = t.tabRounding;
    s.WindowBorderSize = t.windowBorder;
    s.ChildBorderSize = 1;
    s.PopupBorderSize = 1;
    s.FrameBorderSize = t.frameBorder;
    s.TabBarBorderSize = 1;
    s.TabBarOverlineSize = 2;
    s.DockingSeparatorSize = 3;
    s.WindowMenuButtonPosition = ImGuiDir_None; // windows move by their tab; no separate handle
    s.TabCloseButtonMinWidthSelected = 0;       // a tab's close box shows while the tab is hovered
    s.TabCloseButtonMinWidthUnselected = 0;
    const auto &p = t.palette;
    auto *c = s.Colors;
    c[ImGuiCol_Text] = p.text;
    c[ImGuiCol_TextDisabled] = p.muted;
    c[ImGuiCol_WindowBg] = t.window;
    c[ImGuiCol_ChildBg] = t.child;
    c[ImGuiCol_PopupBg] = t.popup;
    c[ImGuiCol_Border] = t.border;
    c[ImGuiCol_BorderShadow] = t.borderShadow;
    c[ImGuiCol_FrameBg] = t.frame;
    c[ImGuiCol_FrameBgHovered] = t.frameHovered;
    c[ImGuiCol_FrameBgActive] = t.frameActive;
    c[ImGuiCol_TitleBg] = t.title;
    c[ImGuiCol_TitleBgActive] = t.titleActive;
    c[ImGuiCol_TitleBgCollapsed] = t.title;
    c[ImGuiCol_MenuBarBg] = p.bar;
    c[ImGuiCol_ScrollbarBg] = {0, 0, 0, 0};
    c[ImGuiCol_ScrollbarGrab] = t.scrollGrab;
    c[ImGuiCol_ScrollbarGrabHovered] = t.buttonHovered;
    c[ImGuiCol_ScrollbarGrabActive] = p.accent;
    c[ImGuiCol_CheckMark] = p.accent;
    c[ImGuiCol_SliderGrab] = p.accent;
    c[ImGuiCol_SliderGrabActive] = p.activeHovered;
    c[ImGuiCol_Button] = t.button;
    c[ImGuiCol_ButtonHovered] = t.buttonHovered;
    c[ImGuiCol_ButtonActive] = t.buttonPressed;
    c[ImGuiCol_Header] = t.header;
    c[ImGuiCol_HeaderHovered] = t.headerHovered;
    c[ImGuiCol_HeaderActive] = t.headerPressed;
    c[ImGuiCol_Separator] = t.border;
    c[ImGuiCol_SeparatorHovered] = alpha(p.accent, .6f);
    c[ImGuiCol_SeparatorActive] = p.accent;
    c[ImGuiCol_ResizeGrip] = alpha(p.accent, .12f);
    c[ImGuiCol_ResizeGripHovered] = alpha(p.accent, .5f);
    c[ImGuiCol_ResizeGripActive] = p.accent;
    c[ImGuiCol_Tab] = t.tab;
    c[ImGuiCol_TabHovered] = t.tabHovered;
    c[ImGuiCol_TabSelected] = t.tabSelected;
    c[ImGuiCol_TabSelectedOverline] = p.accent;
    c[ImGuiCol_TabDimmed] = t.tabDimmed;
    c[ImGuiCol_TabDimmedSelected] = t.tabDimmedSelected;
    c[ImGuiCol_TabDimmedSelectedOverline] = alpha(p.accent, .5f);
    c[ImGuiCol_DockingPreview] = alpha(p.accent, .35f);
    c[ImGuiCol_DockingEmptyBg] = p.bar;
    c[ImGuiCol_PlotLines] = p.accent;
    c[ImGuiCol_PlotLinesHovered] = p.activeHovered;
    c[ImGuiCol_PlotHistogram] = p.accent;
    c[ImGuiCol_PlotHistogramHovered] = p.activeHovered;
    // a quiet step off the panel, not the selection colour (a header is not a selected row)
    c[ImGuiCol_TableHeaderBg] = {t.window.x * .9f + p.text.x * .1f, t.window.y * .9f + p.text.y * .1f,
                                 t.window.z * .9f + p.text.z * .1f, 1};
    c[ImGuiCol_TableBorderStrong] = t.border;
    c[ImGuiCol_TableBorderLight] = t.border;
    c[ImGuiCol_TableRowBg] = {0, 0, 0, 0};
    c[ImGuiCol_TableRowBgAlt] = t.tableRowAlt;
    c[ImGuiCol_TextLink] = p.accent;
    c[ImGuiCol_TextSelectedBg] = alpha(p.accent, .35f);
    c[ImGuiCol_DragDropTarget] = p.warn;
    c[ImGuiCol_NavCursor] = p.accent;
    c[ImGuiCol_NavWindowingHighlight] = alpha(p.text, .7f);
    c[ImGuiCol_NavWindowingDimBg] = {.2f, .2f, .2f, .2f};
    c[ImGuiCol_ModalWindowDimBg] = {.1f, .1f, .1f, .45f};
    s.ScaleAllSizes(scale);
    ImGui::GetStyle() = s;
    current = p;
}

// Deep water: the viewer's original dark teal.
Spec abyss() {
    Spec t;
    t.window = {.035f, .052f, .066f, 1};
    t.child = {.052f, .074f, .091f, 1};
    t.popup = {.045f, .066f, .082f, .98f};
    t.frame = {.083f, .12f, .145f, 1};
    t.frameHovered = {.11f, .17f, .2f, 1};
    t.frameActive = {.13f, .22f, .25f, 1};
    t.button = {.085f, .14f, .17f, 1};
    t.buttonHovered = {.13f, .27f, .29f, 1};
    t.buttonPressed = {.11f, .36f, .35f, 1};
    t.border = {.12f, .18f, .21f, 1};
    t.header = {.09f, .25f, .27f, 1};
    t.headerHovered = {.13f, .32f, .33f, 1};
    t.headerPressed = {.11f, .38f, .37f, 1};
    t.title = {.04f, .06f, .075f, 1};
    t.titleActive = {.06f, .1f, .12f, 1};
    t.tab = {.06f, .09f, .11f, 1};
    t.tabHovered = {.13f, .27f, .29f, 1};
    t.tabSelected = {.085f, .16f, .19f, 1};
    t.tabDimmed = {.045f, .066f, .082f, 1};
    t.tabDimmedSelected = {.07f, .12f, .14f, 1};
    t.scrollGrab = {.12f, .2f, .23f, 1};
    t.tableRowAlt = {1, 1, 1, .025f};
    auto &p = t.palette;
    p.text = {.87f, .92f, .95f, 1};
    p.muted = {.47f, .57f, .64f, 1};
    p.accent = {.32f, .86f, .82f, 1};
    p.active = {.12f, .48f, .46f, 1};
    p.activeHovered = {.16f, .6f, .56f, 1};
    p.activePressed = {.1f, .4f, .38f, 1};
    p.activeText = p.text;
    p.danger = {.65f, .16f, .19f, 1};
    p.dangerHovered = {.8f, .22f, .25f, 1};
    p.dangerPressed = {.55f, .12f, .15f, 1};
    p.dangerText = {1, 1, 1, 1};
    p.warn = {.94f, .73f, .35f, 1};
    p.error = {.96f, .36f, .33f, 1};
    p.robotEnabled = {.3f, 1, .65f, 1};
    p.robotKilled = {1, .65f, .4f, 1};
    p.bar = {.028f, .043f, .057f, 1};
    p.toolbar = {.045f, .066f, .082f, 1};
    return t;
}

// Neutral graphite with a blue accent, square-ish corners.
Spec midnight() {
    Spec t;
    t.windowRounding = 4;
    t.frameRounding = 3;
    t.tabRounding = 3;
    t.window = {.098f, .102f, .114f, 1};
    t.child = {.122f, .126f, .141f, 1};
    t.popup = {.11f, .114f, .129f, .98f};
    t.frame = {.165f, .17f, .192f, 1};
    t.frameHovered = {.2f, .21f, .24f, 1};
    t.frameActive = {.22f, .25f, .31f, 1};
    t.button = {.18f, .185f, .21f, 1};
    t.buttonHovered = {.24f, .27f, .34f, 1};
    t.buttonPressed = {.2f, .3f, .46f, 1};
    t.border = {.21f, .22f, .25f, 1};
    t.header = {.2f, .26f, .38f, 1};
    t.headerHovered = {.24f, .32f, .47f, 1};
    t.headerPressed = {.27f, .38f, .56f, 1};
    t.title = {.075f, .078f, .09f, 1};
    t.titleActive = {.11f, .118f, .14f, 1};
    t.tab = {.11f, .114f, .129f, 1};
    t.tabHovered = {.24f, .29f, .4f, 1};
    t.tabSelected = {.16f, .17f, .2f, 1};
    t.tabDimmed = {.098f, .102f, .114f, 1};
    t.tabDimmedSelected = {.135f, .14f, .16f, 1};
    t.scrollGrab = {.24f, .25f, .28f, 1};
    t.tableRowAlt = {1, 1, 1, .03f};
    auto &p = t.palette;
    p.text = {.9f, .91f, .93f, 1};
    p.muted = {.54f, .56f, .62f, 1};
    p.accent = {.42f, .66f, 1, 1};
    p.active = {.2f, .4f, .74f, 1};
    p.activeHovered = {.26f, .48f, .86f, 1};
    p.activePressed = {.17f, .34f, .64f, 1};
    p.activeText = {1, 1, 1, 1};
    p.danger = {.72f, .18f, .2f, 1};
    p.dangerHovered = {.86f, .25f, .27f, 1};
    p.dangerPressed = {.6f, .14f, .16f, 1};
    p.dangerText = {1, 1, 1, 1};
    p.warn = {.98f, .74f, .3f, 1};
    p.error = {1, .4f, .38f, 1};
    p.robotEnabled = {.42f, .9f, .55f, 1};
    p.robotKilled = {1, .66f, .38f, 1};
    p.bar = {.075f, .078f, .09f, 1};
    p.toolbar = {.114f, .118f, .133f, 1};
    return t;
}

// Light, for a sunny pool deck where a dark screen washes out.
Spec daylight() {
    Spec t;
    t.windowRounding = 6;
    t.frameRounding = 5;
    t.window = {.94f, .95f, .96f, 1};
    t.child = {.98f, .985f, .99f, 1};
    t.popup = {.99f, .99f, 1, .99f};
    t.frame = {.86f, .89f, .91f, 1};
    t.frameHovered = {.8f, .86f, .89f, 1};
    t.frameActive = {.74f, .84f, .87f, 1};
    t.button = {.85f, .88f, .9f, 1};
    t.buttonHovered = {.76f, .86f, .88f, 1};
    t.buttonPressed = {.64f, .81f, .83f, 1};
    t.border = {.74f, .78f, .81f, 1};
    t.header = {.72f, .86f, .87f, 1};
    t.headerHovered = {.66f, .83f, .85f, 1};
    t.headerPressed = {.58f, .79f, .81f, 1};
    t.title = {.88f, .9f, .92f, 1};
    t.titleActive = {.82f, .87f, .9f, 1};
    t.tab = {.87f, .89f, .91f, 1};
    t.tabHovered = {.74f, .86f, .88f, 1};
    t.tabSelected = {.98f, .985f, .99f, 1};
    t.tabDimmed = {.9f, .915f, .93f, 1};
    t.tabDimmedSelected = {.95f, .96f, .97f, 1};
    t.scrollGrab = {.72f, .76f, .79f, 1};
    t.tableRowAlt = {0, 0, 0, .035f};
    auto &p = t.palette;
    p.text = {.07f, .11f, .15f, 1};
    p.muted = {.36f, .43f, .49f, 1};
    p.accent = {0, .5f, .55f, 1};
    p.active = {.45f, .8f, .8f, 1};
    p.activeHovered = {.52f, .86f, .85f, 1};
    p.activePressed = {.38f, .72f, .72f, 1};
    p.activeText = {.03f, .12f, .14f, 1};
    p.danger = {.8f, .14f, .17f, 1};
    p.dangerHovered = {.9f, .2f, .22f, 1};
    p.dangerPressed = {.68f, .1f, .13f, 1};
    p.dangerText = {1, 1, 1, 1};
    p.warn = {.75f, .45f, 0, 1};
    p.error = {.8f, .14f, .14f, 1};
    p.robotEnabled = {0, .55f, .25f, 1};
    p.robotKilled = {.8f, .35f, 0, 1};
    p.bar = {.86f, .89f, .91f, 1};
    p.toolbar = {.9f, .92f, .94f, 1};
    return t;
}

// Black, white and yellow with outlined controls: the most legible at a glance.
Spec contrast() {
    Spec t;
    t.windowRounding = 0;
    t.frameRounding = 0;
    t.tabRounding = 0;
    t.frameBorder = 1;
    t.window = {0, 0, 0, 1};
    t.child = {.03f, .03f, .03f, 1};
    t.popup = {.02f, .02f, .02f, 1};
    t.frame = {.06f, .06f, .06f, 1};
    t.frameHovered = {.14f, .14f, .14f, 1};
    t.frameActive = {.2f, .2f, .2f, 1};
    t.button = {.08f, .08f, .08f, 1};
    t.buttonHovered = {.22f, .22f, .22f, 1};
    t.buttonPressed = {.32f, .32f, .32f, 1};
    t.border = {.7f, .7f, .7f, 1};
    t.header = {.25f, .22f, 0, 1};
    t.headerHovered = {.35f, .31f, 0, 1};
    t.headerPressed = {.45f, .4f, 0, 1};
    t.title = {0, 0, 0, 1};
    t.titleActive = {.1f, .1f, .1f, 1};
    t.tab = {0, 0, 0, 1};
    t.tabHovered = {.25f, .25f, .25f, 1};
    t.tabSelected = {.12f, .12f, .12f, 1};
    t.tabDimmed = {0, 0, 0, 1};
    t.tabDimmedSelected = {.08f, .08f, .08f, 1};
    t.scrollGrab = {.5f, .5f, .5f, 1};
    t.tableRowAlt = {1, 1, 1, .06f};
    auto &p = t.palette;
    p.text = {1, 1, 1, 1};
    p.muted = {.78f, .78f, .78f, 1};
    p.accent = {1, .86f, 0, 1};
    p.active = {1, .86f, 0, 1};
    p.activeHovered = {1, .92f, .35f, 1};
    p.activePressed = {.85f, .72f, 0, 1};
    p.activeText = {0, 0, 0, 1};
    p.danger = {.9f, 0, 0, 1};
    p.dangerHovered = {1, .2f, .2f, 1};
    p.dangerPressed = {.7f, 0, 0, 1};
    p.dangerText = {1, 1, 1, 1};
    p.warn = {1, .6f, 0, 1};
    p.error = {1, .25f, .25f, 1};
    p.robotEnabled = {.2f, 1, .3f, 1};
    p.robotKilled = {1, .6f, 0, 1};
    p.bar = {0, 0, 0, 1};
    p.toolbar = {.05f, .05f, .05f, 1};
    return t;
}

ImVec4 mix(ImVec4 a, ImVec4 b, float t) {
    return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, 1};
}
float luminance(ImVec4 c) {
    return .2126f * c.x + .7152f * c.y + .0722f * c.z;
}
// A whole theme from four colours: the background, body and secondary text, and the accent. Surfaces step from
// the background toward the text; hovered / pressed states and "on" fills lean toward the accent.
Spec derived(ImVec4 bg, ImVec4 text, ImVec4 muted, ImVec4 accent, float rounding) {
    const bool light = luminance(bg) > .5f;
    Spec t;
    t.windowRounding = rounding;
    t.frameRounding = std::max(0.f, rounding - 2);
    t.tabRounding = t.frameRounding;
    t.window = bg;
    t.child = mix(bg, text, .035f);
    t.popup = mix(bg, text, .03f);
    t.frame = mix(bg, text, light ? .08f : .09f);
    t.frameHovered = mix(t.frame, accent, .18f);
    t.frameActive = mix(t.frame, accent, .3f);
    t.button = mix(bg, text, light ? .09f : .1f);
    t.buttonHovered = mix(t.button, accent, .3f);
    t.buttonPressed = mix(t.button, accent, .45f);
    t.border = mix(bg, text, light ? .2f : .16f);
    t.header = mix(bg, accent, .3f);
    t.headerHovered = mix(bg, accent, .4f);
    t.headerPressed = mix(bg, accent, .5f);
    t.title = mix(bg, light ? text : ImVec4(0, 0, 0, 1), light ? .05f : .25f);
    t.titleActive = mix(bg, text, .06f);
    t.tab = mix(bg, text, .04f);
    t.tabHovered = t.buttonHovered;
    t.tabSelected = mix(bg, text, .09f);
    t.tabDimmed = bg;
    t.tabDimmedSelected = mix(bg, text, .06f);
    t.scrollGrab = mix(bg, text, .22f);
    t.tableRowAlt = {text.x, text.y, text.z, .035f};
    auto &p = t.palette;
    p.text = text;
    p.muted = muted;
    p.accent = accent;
    p.active = mix(bg, accent, light ? .5f : .55f);
    p.activeHovered = mix(bg, accent, light ? .62f : .68f);
    p.activePressed = mix(bg, accent, light ? .4f : .45f);
    p.activeText = luminance(p.active) > .5f ? ImVec4(.05f, .05f, .06f, 1) : ImVec4(1, 1, 1, 1);
    p.danger = light ? ImVec4(.8f, .14f, .17f, 1) : ImVec4(.7f, .17f, .19f, 1);
    p.dangerHovered = light ? ImVec4(.9f, .2f, .22f, 1) : ImVec4(.84f, .24f, .26f, 1);
    p.dangerPressed = light ? ImVec4(.68f, .1f, .13f, 1) : ImVec4(.58f, .13f, .15f, 1);
    p.dangerText = {1, 1, 1, 1};
    p.warn = light ? ImVec4(.72f, .44f, 0, 1) : ImVec4(.96f, .74f, .33f, 1);
    p.error = light ? ImVec4(.8f, .14f, .14f, 1) : ImVec4(.98f, .38f, .35f, 1);
    p.robotEnabled = light ? ImVec4(0, .55f, .25f, 1) : ImVec4(.35f, .95f, .6f, 1);
    p.robotKilled = light ? ImVec4(.8f, .35f, 0, 1) : ImVec4(1, .65f, .4f, 1);
    p.bar = mix(bg, light ? text : ImVec4(0, 0, 0, 1), light ? .06f : .3f);
    p.toolbar = mix(bg, text, .03f);
    return t;
}

// Navy and cyan-blue: deep open water.
Spec ocean() {
    return derived({.035f, .06f, .11f, 1}, {.86f, .91f, .97f, 1}, {.5f, .6f, .72f, 1}, {.25f, .7f, 1, 1}, 8);
}
// Green phosphor on black, like a sonar or oscilloscope screen.
Spec sonar() {
    return derived({.01f, .035f, .02f, 1}, {.66f, 1, .72f, 1}, {.35f, .62f, .42f, 1}, {.2f, 1, .45f, 1}, 2);
}
// Warm charcoal with an orange accent.
Spec ember() {
    return derived({.09f, .075f, .07f, 1}, {.95f, .9f, .86f, 1}, {.62f, .55f, .5f, 1}, {1, .55f, .2f, 1}, 6);
}
// Cool slate and frost blue (Nord-like).
Spec arctic() {
    return derived({.18f, .2f, .25f, 1}, {.9f, .92f, .95f, 1}, {.6f, .65f, .73f, 1}, {.53f, .75f, .82f, 1}, 6);
}
// Warm light paper with a blue accent: easy on the eyes in daylight.
Spec paper() {
    return derived({.97f, .95f, .9f, 1}, {.2f, .18f, .15f, 1}, {.5f, .46f, .4f, 1}, {.15f, .45f, .75f, 1}, 6);
}

// Classic: Qt's Windows style on this desktop (RViz and the other Qt tools): the "win 2000 grey" palette of
// QWindowsStyle, with raised bevels and the Ubuntu font.
ImVec4 hex(unsigned rgb) {
    return {float((rgb >> 16) & 255) / 255.f, float((rgb >> 8) & 255) / 255.f, float(rgb & 255) / 255.f, 1};
}
Spec classic() {
    // QPalette roles of QWindowsStyle::standardPalette (#d4d0c8 face, lighter / darker of it, mid Qt::gray, white
    // base) with QPalette's defaults for the rest: navy highlight, black shadow.
    enum Role {
        WindowText,
        Button,
        Light,
        Midlight,
        Dark,
        Mid,
        Text,
        Base,
        Window,
        Shadow,
        Highlight,
        HighlightedText
    };
    static const unsigned colors[] = {0x000000, 0xd4d0c8, 0xffffff, 0xe9e7e3, 0x6a6864, 0xa0a0a4,
                                      0x000000, 0xffffff, 0xd4d0c8, 0x000000, 0x000080, 0xffffff};
    const auto role = [](Role r) { return hex(colors[r]); };
    Spec t;
    t.windowRounding = t.frameRounding = t.tabRounding = 0;
    t.frameBorder = 1;
    t.window = role(Window);
    t.child = role(Window);
    t.popup = role(Window);
    t.frame = role(Base);
    t.frameHovered = mix(role(Base), role(Highlight), .15f);
    t.frameActive = mix(role(Base), role(Highlight), .3f);
    t.button = role(Button);
    t.buttonHovered = mix(role(Button), role(Light), .2f);
    t.buttonPressed = mix(role(Button), role(Shadow), .5f);
    // Qt's bevels: a light edge with the shadow just below / right, as raised buttons and frames.
    t.border = role(Light);
    t.borderShadow = role(Shadow);
    t.header = role(Highlight);
    t.headerHovered = mix(role(Highlight), role(Light), .15f);
    t.headerPressed = mix(role(Highlight), role(Shadow), .25f);
    // Dock title bars in Mid (RViz's grey title strips), tabs in the button face, the shown tab light and tinted.
    t.title = role(Mid);
    t.titleActive = role(Mid);
    t.tab = role(Button);
    t.tabHovered = t.buttonHovered;
    t.tabSelected = mix(role(Base), role(Highlight), .12f);
    t.tabDimmed = role(Button);
    t.tabDimmedSelected = mix(role(Base), role(Highlight), .06f);
    t.scrollGrab = role(Midlight);
    t.tableRowAlt = {role(Text).x, role(Text).y, role(Text).z, .05f};
    auto &p = t.palette;
    p.text = role(WindowText);
    // Secondary labels: the text toward the window (Qt draws them as normal text; its disabled grey is too faint).
    p.muted = mix(role(WindowText), role(Window), .4f);
    p.accent = role(Highlight);
    p.active = role(Highlight);
    p.activeHovered = mix(role(Highlight), role(Light), .15f);
    p.activePressed = mix(role(Highlight), role(Shadow), .25f);
    p.activeText = role(HighlightedText);
    // Status colours readable on this background: deep ones under dark text, bright ones under light text.
    const bool darkText = luminance(p.text) < .5f;
    p.danger = {.72f, .1f, .12f, 1};
    p.dangerHovered = {.84f, .16f, .18f, 1};
    p.dangerPressed = {.6f, .07f, .09f, 1};
    p.dangerText = {1, 1, 1, 1};
    p.warn = darkText ? ImVec4(.48f, .27f, 0, 1) : ImVec4(1, .78f, .3f, 1);
    p.error = darkText ? ImVec4(.58f, 0, 0, 1) : ImVec4(1, .4f, .38f, 1);
    p.robotEnabled = darkText ? ImVec4(0, .36f, .12f, 1) : ImVec4(.4f, 1, .6f, 1);
    p.robotKilled = darkText ? ImVec4(.55f, .2f, 0, 1) : ImVec4(1, .66f, .4f, 1);
    p.bar = role(Window); // menu bar and command bar as Qt's: the window colour
    p.toolbar = role(Window);
    return t;
}

struct Entry {
    Theme theme;
    std::function<Spec()> spec;
};
const std::vector<Entry> &entries() {
    static const std::vector<Entry> list{
        {{"abyss", "Abyss", "Dark teal, the default"}, abyss},
        {{"midnight", "Midnight", "Neutral dark grey with a blue accent"}, midnight},
        {{"daylight", "Daylight", "Light, for bright rooms and a sunny pool deck"}, daylight},
        {{"contrast", "High contrast", "Black, white and yellow, outlined controls"}, contrast},
        {{"ocean", "Ocean", "Navy with a bright blue accent"}, ocean},
        {{"arctic", "Arctic", "Cool slate with a frost-blue accent"}, arctic},
        {{"ember", "Ember", "Warm charcoal with an orange accent"}, ember},
        {{"sonar", "Sonar", "Green phosphor on black"}, sonar},
        {{"paper", "Paper", "Warm light paper with a blue accent"}, paper},
        {{"classic", "Classic",
          "Windows-style grey with bevelled controls and the Ubuntu font, like the desktop's Qt "
          "tools (RViz)",
          "Ubuntu", 11},
         classic}};
    return list;
}
} // namespace

const std::vector<Theme> &themes() {
    static const std::vector<Theme> list = [] {
        std::vector<Theme> out;
        for (const auto &entry : entries())
            out.push_back(entry.theme);
        return out;
    }();
    return list;
}

// Legibility floor for every theme (the pool deck is in daylight): WCAG contrast of 4.5:1 for secondary, status and
// accent text on every surface it is drawn on, and for the labels of filled "on" controls. A colour that falls short
// moves toward black or white (keeping its hue) until it passes; KILL's colours are the theme's own.
namespace {
float channel(float c) {
    return c <= .04045f ? c / 12.92f : std::pow((c + .055f) / 1.055f, 2.4f);
}
float relativeLuminance(ImVec4 c) {
    return .2126f * channel(c.x) + .7152f * channel(c.y) + .0722f * channel(c.z);
}
} // namespace
float contrastRatio(ImVec4 a, ImVec4 b) {
    float la = relativeLuminance(a), lb = relativeLuminance(b);
    if (la < lb)
        std::swap(la, lb);
    return (la + .05f) / (lb + .05f);
}
namespace {
float contrast(ImVec4 a, ImVec4 b) {
    return contrastRatio(a, b);
}
template <typename Worst> ImVec4 pushUntil(ImVec4 c, ImVec4 toward, float target, Worst worst) {
    if (worst(c) >= target)
        return c;
    for (int i = 1; i <= 50; ++i) {
        const ImVec4 next = mix(c, toward, float(i) / 50);
        if (worst(next) >= target)
            return next;
    }
    return toward;
}
// Text `fg` readable on every surface in `surfaces` (pills included: their fill is the window tinted 18 % by fg).
ImVec4 readable(ImVec4 fg, const std::vector<ImVec4> &surfaces, bool onPill, float target = 4.5f) {
    float mean = 0;
    for (const auto &s : surfaces)
        mean += relativeLuminance(s);
    mean /= float(surfaces.size());
    const ImVec4 toward = mean < .18f ? ImVec4(1, 1, 1, fg.w) : ImVec4(0, 0, 0, fg.w);
    return pushUntil(fg, toward, target, [&](ImVec4 c) {
        float worst = 1e9f;
        for (const auto &s : surfaces)
            worst = std::min(worst, contrast(c, s));
        if (onPill)
            worst = std::min(worst, contrast(c, mix(surfaces.front(), c, .18f)));
        return worst;
    });
}
// A filled control's colour, darkened or lightened (away from its label) until the label reads.
ImVec4 fillFor(ImVec4 fill, ImVec4 label, float target = 4.5f) {
    const ImVec4 toward = relativeLuminance(label) > .5f ? ImVec4(0, 0, 0, fill.w) : ImVec4(1, 1, 1, fill.w);
    return pushUntil(fill, toward, target, [&](ImVec4 c) { return contrast(c, label); });
}
Spec legible(Spec t) {
    auto &p = t.palette;
    // Input boxes, checkboxes and buttons stand out from the panel they sit on (an unchecked checkbox used to vanish
    // at 1.1:1); hovered / pressed shades keep their step beyond the resting one.
    const auto stepOut = [&](ImVec4 &fill, std::initializer_list<ImVec4 *> shades, float target) {
        const auto against = [&](ImVec4 c) { return std::min(contrast(c, t.window), contrast(c, t.child)); };
        const ImVec4 toward(p.text.x, p.text.y, p.text.z, fill.w);
        const float before = against(fill);
        fill = pushUntil(fill, toward, target, against);
        const float gained = against(fill) / before;
        for (auto *shade : shades)
            *shade = pushUntil(*shade, ImVec4(p.text.x, p.text.y, p.text.z, shade->w),
                               std::max(target, against(*shade) * gained), against);
    };
    stepOut(t.frame, {&t.frameHovered, &t.frameActive}, 1.35f);
    stepOut(t.button, {&t.buttonHovered, &t.buttonPressed}, 1.25f);
    const std::vector<ImVec4> surfaces{t.window, t.child, t.popup, t.frame, t.button, t.tabSelected, p.bar, p.toolbar};
    // 5:1 rather than 4.5: small anti-aliased text measures below its colour pair
    p.muted = readable(p.muted, surfaces, false, 5.5f);
    p.accent = readable(p.accent, {t.window, t.child, p.bar}, false, 5);
    for (auto *status : {&p.warn, &p.error, &p.robotEnabled, &p.robotKilled})
        *status = readable(*status, {t.window, t.child, p.bar}, true, 5);
    // tabs stand out from the tab bar behind them (an inactive tab used to vanish into it)
    for (auto *tab : {&t.tab, &t.tabDimmed})
        *tab = pushUntil(*tab, ImVec4(p.text.x, p.text.y, p.text.z, tab->w), 1.2f, [&](ImVec4 c) {
            return std::min(contrast(c, t.title), contrast(c, t.titleActive));
        });
    p.active = fillFor(p.active, p.activeText, 5);
    p.activeHovered = fillFor(p.activeHovered, p.activeText, 5);
    p.activePressed = fillFor(p.activePressed, p.activeText, 5);
    return t;
}

} // namespace

bool applyTheme(const std::string &id) {
    for (const auto &entry : entries())
        if (entry.theme.id == id) {
            const auto spec = [make = entry.spec] { return legible(make()); };
            if (ImGui::GetCurrentContext())
                apply(spec()); // style and palette
            else
                current = spec().palette;
            currentId = id;
            currentSpec = spec;
            return true;
        }
    return false;
}

// Before any applyTheme(): the default theme's colours, without touching the ImGui style.
static void ensureDefault() {
    if (currentId.empty()) {
        current = legible(entries().front().spec()).palette;
        currentId = entries().front().theme.id;
    }
}

const Theme &currentThemeInfo() {
    ensureDefault();
    for (const auto &entry : entries())
        if (entry.theme.id == currentId)
            return entry.theme;
    return entries().front().theme;
}

const std::string &currentTheme() {
    ensureDefault();
    return currentId;
}

void setInterfaceScale(float value) {
    scale = std::clamp(value, .5f, 4.f);
    if (currentSpec && ImGui::GetCurrentContext())
        apply(currentSpec());
}

float interfaceScale() {
    return scale;
}

const Palette &palette() {
    ensureDefault();
    return current;
}

namespace {
TypeRamp ramp;
}
void setTypeRamp(const TypeRamp &fonts) {
    ramp = fonts;
}
const TypeRamp &typeRamp() {
    return ramp;
}
void sectionTitle(const char *text) {
    if (ramp.strong)
        ImGui::PushFont(ramp.strong);
    ImGui::SeparatorText(text);
    if (ramp.strong)
        ImGui::PopFont();
}
void tableHeaders() {
    if (ramp.strong)
        ImGui::PushFont(ramp.strong);
    ImGui::TableHeadersRow();
    if (ramp.strong)
        ImGui::PopFont();
}
} // namespace nereus::ros_viewer

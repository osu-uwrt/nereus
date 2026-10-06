// Viewer themes: YAML theme loading, derivation of a full ImGui style from a few colors, the WCAG legibility
// pass, and the shared section / table / surface helpers that draw in the active theme.
#include "nereus/ros_viewer/theme.hpp"
#include <imgui_internal.h>
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
#include <yaml-cpp/yaml.h>

namespace nereus::ros_viewer {
namespace {
// The colors a theme chooses; everything else in ImGuiStyle derives from them.
struct Spec {
    ImVec4 window, child, popup, frame, frameHovered, frameActive, button, buttonHovered, buttonPressed, border, header,
        headerHovered, headerPressed, title, titleActive, tab, tabHovered, tabSelected, tabDimmed, tabDimmedSelected,
        scrollGrab, tableRowAlt;
    float windowRounding = 8, frameRounding = 5, tabRounding = 5, windowBorder = 1, frameBorder = 0;
    ImVec4 borderShadow{0, 0, 0, 0}; // drawn 1 px below / right of framed widgets: a raised bevel
    // Spacing (at 100 %); every theme shares these unless it sets a denser or looser rhythm.
    ImVec2 windowPadding{14, 12}, framePadding{10, 7}, itemSpacing{10, 9}, cellPadding{4, 2};
    bool ruled = false;       // heads carry a heavy ink rule and no fill (a results sheet's column heads)
    bool squareChips = false; // status chips take the frame's corners instead of a capsule
    float scrollbar = 14;     // scrollbar width (at 100 %)
    // a frame round each tab, the tab bar's rule, the line on the shown tab
    float tabBorder = 0, tabBarBorder = 1, tabOverlineSize = 2;
    ImVec4 tabOverline{0, 0, 0, 0}; // the shown tab's top line; transparent: the accent
    // The board (menu and command bars) in its own colors; without one it is drawn in the panels' colors.
    bool hasBoard = false;
    Palette board;
    ImVec4 boardButton, boardButtonHovered, boardButtonPressed, boardFrame, boardPopup, boardBorder;
    Palette palette;
};

// The palette palette() returns (swapped by beginSurface / endSurface) and the applied theme's id.
Palette current;
std::string currentId;

ImVec4 alpha(ImVec4 c, float a) {
    return {c.x, c.y, c.z, a};
}

// Interface scale and the applied theme's spec factory, kept so a scale change can re-apply it.
float scale = 1;
std::function<Spec()> currentSpec;
Spec active; // the applied theme (after the legibility pass): the surfaces' colors

// Builds the ImGui style from a theme spec at the current scale and makes it the live style and palette.
void apply(const Spec &t) {
    ImGuiStyle s; // from ImGui's defaults each time, so scaling never compounds
    // Sizes are shared by every theme so a layout looks the same in each; only corners and borders change.
    s.WindowPadding = t.windowPadding;
    s.FramePadding = t.framePadding;
    s.ItemSpacing = t.itemSpacing;
    s.CellPadding = t.cellPadding;
    s.ScrollbarSize = t.scrollbar;
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
    s.TabBorderSize = t.tabBorder;
    s.TabBarBorderSize = t.tabBarBorder;
    s.TabBarOverlineSize = t.tabOverlineSize;
    s.DockingSeparatorSize = 3;
    s.WindowMenuButtonPosition = ImGuiDir_None; // windows move by their tab; no separate handle
    s.TabCloseButtonMinWidthSelected = 0;       // a tab's close box shows while the tab is hovered
    s.TabCloseButtonMinWidthUnselected = 0;

    // Colors: text and surfaces from the spec, interactive accents from the palette.
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
    c[ImGuiCol_TabSelectedOverline] = t.tabOverline.w > 0 ? t.tabOverline : p.accent;
    c[ImGuiCol_TabDimmed] = t.tabDimmed;
    c[ImGuiCol_TabDimmedSelected] = t.tabDimmedSelected;
    c[ImGuiCol_TabDimmedSelectedOverline] = alpha(p.accent, .5f);
    c[ImGuiCol_DockingPreview] = alpha(p.accent, .35f);
    c[ImGuiCol_DockingEmptyBg] = p.bar;
    c[ImGuiCol_PlotLines] = p.accent;
    c[ImGuiCol_PlotLinesHovered] = p.activeHovered;
    c[ImGuiCol_PlotHistogram] = p.accent;
    c[ImGuiCol_PlotHistogramHovered] = p.activeHovered;
    // a quiet step off the panel, not the selection color (a header is not a selected row)
    c[ImGuiCol_TableHeaderBg] = t.ruled ? ImVec4(0, 0, 0, 0)
                                        : ImVec4(t.window.x * .9f + p.text.x * .1f, t.window.y * .9f + p.text.y * .1f,
                                                 t.window.z * .9f + p.text.z * .1f, 1);
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
    active = t;
}

// Linear blend from a to b (t = 0..1); the result is always opaque.
ImVec4 mix(ImVec4 a, ImVec4 b, float t) {
    return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, 1};
}

// Quick luma (Rec. 709 weights, no gamma) used only to tell light themes from dark ones.
float luminance(ImVec4 c) {
    return .2126f * c.x + .7152f * c.y + .0722f * c.z;
}

// A whole theme from four colors: the background, body and secondary text, and the accent. Surfaces step from
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

    // Palette: active fills from the accent, fixed status hues tuned separately for light and dark backgrounds.
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

// 0xRRGGBB to an opaque color.
ImVec4 hex(unsigned rgb) {
    return {float((rgb >> 16) & 255) / 255.f, float((rgb >> 8) & 255) / 255.f, float(rgb & 255) / 255.f, 1};
}

// Themes are data (content/viewer/themes/*.yaml, read by loadThemes); until they load, one built-in fallback.
struct Entry {
    Theme theme;
    std::function<Spec()> spec;
    int order = 0;
};

// The loaded themes in menu order (replaced by loadThemes).
std::vector<Entry> &entries() {
    static std::vector<Entry> list{{{"fallback", "Fallback", "Dark teal (built in: no theme files were found)"}, [] {
                                        return derived(hex(0x091116), hex(0xdeebf2), hex(0x7891a3), hex(0x52dbd1), 8);
                                    }}};
    return list;
}

// Cached Theme descriptions for themes(); rebuilt when its size no longer matches entries().
std::vector<Theme> &themeList() {
    static std::vector<Theme> list;
    return list;
}
} // namespace

namespace {
// A color as "#rrggbb" or "#rrggbbaa".
ImVec4 color(const YAML::Node &node, const std::string &where) {
    const auto text = node.as<std::string>("");
    const auto bad = [&] {
        return std::runtime_error(where + ": \"" + text + "\" is not a #rrggbb or #rrggbbaa color");
    };
    const auto digit = [&](char c) -> int {
        if (c >= '0' && c <= '9')
            return c - '0';
        if (c >= 'a' && c <= 'f')
            return c - 'a' + 10;
        if (c >= 'A' && c <= 'F')
            return c - 'A' + 10;
        throw bad();
    };
    if ((text.size() != 7 && text.size() != 9) || text[0] != '#')
        throw bad();
    const auto byte = [&](std::size_t i) { return float(digit(text[i]) * 16 + digit(text[i + 1])) / 255.f; };
    return {byte(1), byte(3), byte(5), text.size() == 9 ? byte(7) : 1.f};
}

// Throws unless `node` is a map whose keys are all in `allowed`.
void only(const YAML::Node &node, std::initializer_list<const char *> allowed, const std::string &where) {
    if (!node.IsMap())
        throw std::runtime_error(where + " must be a map");
    for (const auto &item : node) {
        const auto key = item.first.as<std::string>();
        if (std::none_of(allowed.begin(), allowed.end(), [&](const char *k) { return key == k; }))
            throw std::runtime_error(where + ": unknown key '" + key + "'");
    }
}

// YAML key -> color member tables for the theme file's surfaces, palette and board sections.
using SpecColor = ImVec4 Spec::*;
using PaletteColor = ImVec4 Palette::*;
const std::vector<std::pair<const char *, SpecColor>> &surfaceKeys() {
    static const std::vector<std::pair<const char *, SpecColor>> keys{
        {"window", &Spec::window},
        {"child", &Spec::child},
        {"popup", &Spec::popup},
        {"frame", &Spec::frame},
        {"frame_hovered", &Spec::frameHovered},
        {"frame_active", &Spec::frameActive},
        {"button", &Spec::button},
        {"button_hovered", &Spec::buttonHovered},
        {"button_pressed", &Spec::buttonPressed},
        {"border", &Spec::border},
        {"border_shadow", &Spec::borderShadow},
        {"header", &Spec::header},
        {"header_hovered", &Spec::headerHovered},
        {"header_pressed", &Spec::headerPressed},
        {"title", &Spec::title},
        {"title_active", &Spec::titleActive},
        {"tab", &Spec::tab},
        {"tab_hovered", &Spec::tabHovered},
        {"tab_selected", &Spec::tabSelected},
        {"tab_dimmed", &Spec::tabDimmed},
        {"tab_dimmed_selected", &Spec::tabDimmedSelected},
        {"tab_overline", &Spec::tabOverline},
        {"scroll_grab", &Spec::scrollGrab},
        {"table_row_alt", &Spec::tableRowAlt}};
    return keys;
}

const std::vector<std::pair<const char *, PaletteColor>> &paletteKeys() {
    static const std::vector<std::pair<const char *, PaletteColor>> keys{{"text", &Palette::text},
                                                                          {"muted", &Palette::muted},
                                                                          {"accent", &Palette::accent},
                                                                          {"active", &Palette::active},
                                                                          {"active_hovered", &Palette::activeHovered},
                                                                          {"active_pressed", &Palette::activePressed},
                                                                          {"active_text", &Palette::activeText},
                                                                          {"danger", &Palette::danger},
                                                                          {"danger_hovered", &Palette::dangerHovered},
                                                                          {"danger_pressed", &Palette::dangerPressed},
                                                                          {"danger_text", &Palette::dangerText},
                                                                          {"warn", &Palette::warn},
                                                                          {"error", &Palette::error},
                                                                          {"robot_enabled", &Palette::robotEnabled},
                                                                          {"robot_killed", &Palette::robotKilled},
                                                                          {"bar", &Palette::bar},
                                                                          {"toolbar", &Palette::toolbar},
                                                                          {"change", &Palette::change},
                                                                          {"change_text", &Palette::changeText},
                                                                          {"enable", &Palette::enable},
                                                                          {"enable_hovered", &Palette::enableHovered},
                                                                          {"enable_pressed", &Palette::enablePressed},
                                                                          {"enable_text", &Palette::enableText}};
    return keys;
}

// Board-only colors; a board section may also override any palette key.
const std::vector<std::pair<const char *, SpecColor>> &boardKeys() {
    static const std::vector<std::pair<const char *, SpecColor>> keys{{"button", &Spec::boardButton},
                                                                       {"button_hovered", &Spec::boardButtonHovered},
                                                                       {"button_pressed", &Spec::boardButtonPressed},
                                                                       {"frame", &Spec::boardFrame},
                                                                       {"popup", &Spec::boardPopup},
                                                                       {"border", &Spec::boardBorder}};
    return keys;
}

// Reads a map of key: color into `target`, rejecting keys missing from `keys`.
template <typename Target, typename Keys>
void readColors(const YAML::Node &node, const Keys &keys, Target &target, const std::string &where) {
    if (!node.IsMap())
        throw std::runtime_error(where + " must be a map");
    for (const auto &item : node) {
        const auto key = item.first.as<std::string>();
        const auto found = std::find_if(keys.begin(), keys.end(), [&](const auto &k) { return key == k.first; });
        if (found == keys.end())
            throw std::runtime_error(where + ": unknown color '" + key + "'");
        target.*(found->second) = color(item.second, where + "." + key);
    }
}

// One theme file: its name and fonts, then its colors and sizes (derived from four colors, or given as surfaces
// and palette, with any explicit values on top), and optionally the board's own colors.
Entry parseTheme(const YAML::Node &doc, const std::string &file) {
    only(doc,
         {"id", "label", "description", "order", "fonts", "derive", "shape", "spacing", "style", "surfaces", "palette",
          "board"},
         file);
    Entry entry;
    auto &theme = entry.theme;
    theme.id = doc["id"].as<std::string>("");
    if (theme.id.empty())
        throw std::runtime_error(file + ": no id");
    theme.label = doc["label"].as<std::string>(theme.id);
    theme.description = doc["description"].as<std::string>("");
    entry.order = doc["order"].as<int>(100);

    if (const auto fonts = doc["fonts"]) {
        only(fonts, {"family", "points", "regular", "strong", "figures"}, file + " fonts");
        theme.fontFamily = fonts["family"].as<std::string>("");
        theme.fontPoints = fonts["points"].as<float>(0);
        theme.fontRegular = fonts["regular"].as<std::string>("");
        theme.fontStrong = fonts["strong"].as<std::string>("");
        theme.fontFigures = fonts["figures"].as<std::string>("");
    }

    // Base colors: derived from four, or (with no derive block) the defaults that surfaces / palette fill in.
    Spec t;
    if (const auto d = doc["derive"]) {
        only(d, {"background", "text", "muted", "accent", "rounding"}, file + " derive");
        t = derived(color(d["background"], file + " derive.background"), color(d["text"], file + " derive.text"),
                    color(d["muted"], file + " derive.muted"), color(d["accent"], file + " derive.accent"),
                    d["rounding"].as<float>(6));
    } else if (!doc["surfaces"] || !doc["palette"])
        throw std::runtime_error(file + ": needs derive, or surfaces and palette");

    // Optional overrides on top: shape, spacing, style flags, then explicit colors.
    if (const auto shape = doc["shape"]) {
        only(shape,
             {"window_rounding", "frame_rounding", "tab_rounding", "window_border", "frame_border", "scrollbar",
              "tab_border", "tab_bar_border", "tab_overline"},
             file + " shape");
        t.tabBorder = shape["tab_border"].as<float>(t.tabBorder);
        t.tabBarBorder = shape["tab_bar_border"].as<float>(t.tabBarBorder);
        t.tabOverlineSize = shape["tab_overline"].as<float>(t.tabOverlineSize);
        t.windowRounding = shape["window_rounding"].as<float>(t.windowRounding);
        t.frameRounding = shape["frame_rounding"].as<float>(t.frameRounding);
        t.tabRounding = shape["tab_rounding"].as<float>(t.tabRounding);
        t.windowBorder = shape["window_border"].as<float>(t.windowBorder);
        t.frameBorder = shape["frame_border"].as<float>(t.frameBorder);
        t.scrollbar = shape["scrollbar"].as<float>(t.scrollbar);
    }
    if (const auto spacing = doc["spacing"]) {
        only(spacing, {"window_padding", "frame_padding", "item_spacing", "cell_padding"}, file + " spacing");
        const auto pair = [&](const char *key, ImVec2 &out) {
            if (const auto v = spacing[key])
                out = {v[0].as<float>(), v[1].as<float>()};
        };
        pair("window_padding", t.windowPadding);
        pair("frame_padding", t.framePadding);
        pair("item_spacing", t.itemSpacing);
        pair("cell_padding", t.cellPadding);
    }
    if (const auto style = doc["style"]) {
        only(style, {"ruled", "square_chips"}, file + " style");
        t.ruled = style["ruled"].as<bool>(t.ruled);
        t.squareChips = style["square_chips"].as<bool>(t.squareChips);
    }
    if (const auto surfaces = doc["surfaces"])
        readColors(surfaces, surfaceKeys(), t, file + " surfaces");
    if (const auto palette = doc["palette"])
        readColors(palette, paletteKeys(), t.palette, file + " palette");
    if (const auto board = doc["board"]) { // the board's own colors: the panels' palette with these changes
        if (!board.IsMap())
            throw std::runtime_error(file + " board must be a map");
        t.hasBoard = true;
        t.board = t.palette;
        t.boardButton = t.button;
        t.boardButtonHovered = t.buttonHovered;
        t.boardButtonPressed = t.buttonPressed;
        t.boardFrame = t.frame;
        t.boardPopup = t.popup;
        t.boardBorder = t.border;
        for (const auto &item : board) {
            const auto key = item.first.as<std::string>();
            const auto extra =
                std::find_if(boardKeys().begin(), boardKeys().end(), [&](const auto &k) { return key == k.first; });
            const auto shade =
                std::find_if(paletteKeys().begin(), paletteKeys().end(), [&](const auto &k) { return key == k.first; });
            if (extra != boardKeys().end())
                t.*(extra->second) = color(item.second, file + " board." + key);
            else if (shade != paletteKeys().end())
                t.board.*(shade->second) = color(item.second, file + " board." + key);
            else
                throw std::runtime_error(file + " board: unknown color '" + key + "'");
        }
    }
    entry.spec = [t] { return t; };
    return entry;
}
} // namespace

// Parses every *.yaml in the directory (sorted by file name); bad files and duplicate ids become warnings.
// The current set is only replaced when at least one theme loads.
std::vector<std::string> loadThemes(const std::filesystem::path &directory) {
    std::vector<std::string> warnings;
    std::vector<Entry> loaded;
    std::error_code error;
    std::vector<std::filesystem::path> files;
    for (const auto &file : std::filesystem::directory_iterator(directory, error))
        if (file.path().extension() == ".yaml")
            files.push_back(file.path());
    std::sort(files.begin(), files.end());

    for (const auto &file : files)
        try {
            auto entry = parseTheme(YAML::LoadFile(file.string()), file.filename().string());
            if (std::any_of(loaded.begin(), loaded.end(), [&](const Entry &e) { return e.theme.id == entry.theme.id; }))
                warnings.push_back(file.filename().string() + ": theme id '" + entry.theme.id + "' is already taken");
            else
                loaded.push_back(std::move(entry));
        } catch (const std::exception &e) {
            warnings.push_back(e.what());
        }
    if (error)
        warnings.push_back(directory.string() + ": " + error.message());

    if (loaded.empty()) {
        warnings.push_back(directory.string() + ": no themes; using the built-in fallback");
        return warnings;
    }

    // Menu order: the theme's `order`, then id.
    std::stable_sort(loaded.begin(), loaded.end(), [](const Entry &a, const Entry &b) {
        return a.order != b.order ? a.order < b.order : a.theme.id < b.theme.id;
    });
    entries() = std::move(loaded);
    themeList().clear();
    if (std::none_of(entries().begin(), entries().end(), [](const Entry &e) { return e.theme.id == currentId; }))
        currentId.clear(); // the next palette() or applyTheme() picks again
    return warnings;
}

const std::vector<Theme> &themes() {
    auto &list = themeList();
    if (list.size() != entries().size()) {
        list.clear();
        for (const auto &entry : entries())
            list.push_back(entry.theme);
    }
    return list;
}

// Legibility floor for every theme (the pool deck is in daylight): WCAG contrast of 4.5:1 for secondary, status and
// accent text on every surface it is drawn on, and for the labels of filled "on" controls. A color that falls short
// moves toward black or white (keeping its hue) until it passes; KILL's colors are the theme's own.
namespace {
// sRGB channel to linear light (the WCAG relative-luminance transfer).
float channel(float c) {
    return c <= .04045f ? c / 12.92f : std::pow((c + .055f) / 1.055f, 2.4f);
}

float relativeLuminance(ImVec4 c) {
    return .2126f * channel(c.x) + .7152f * channel(c.y) + .0722f * channel(c.z);
}
} // namespace

// WCAG contrast ratio, 1..21, independent of argument order.
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

// Moves `c` toward `toward` in 2 % steps until worst(c) reaches `target`; gives `toward` itself if none does.
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
    // Push toward white on dark surfaces, black on light ones.
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

// A filled control's color, darkened or lightened (away from its label) until the label reads.
ImVec4 fillFor(ImVec4 fill, ImVec4 label, float target = 4.5f) {
    const ImVec4 toward = relativeLuminance(label) > .5f ? ImVec4(0, 0, 0, fill.w) : ImVec4(1, 1, 1, fill.w);
    return pushUntil(fill, toward, target, [&](ImVec4 c) { return contrast(c, label); });
}

// The legibility pass (see above) applied to a theme spec before it is used.
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
    if (t.frameBorder <= 0) { // an outlined control is found by its outline
        stepOut(t.frame, {&t.frameHovered, &t.frameActive}, 1.35f);
        stepOut(t.button, {&t.buttonHovered, &t.buttonPressed}, 1.25f);
    }

    // the panels' text is held against the panels; the bar too unless the board carries its own palette
    std::vector<ImVec4> surfaces{t.window, t.child, t.popup, t.frame, t.button, t.tabSelected, p.toolbar},
        grounds{t.window, t.child};
    if (!t.hasBoard) {
        surfaces.push_back(p.bar);
        grounds.push_back(p.bar);
    }
    // 5:1 rather than 4.5: small anti-aliased text measures below its color pair
    p.muted = readable(p.muted, surfaces, false, 5.5f);
    p.accent = readable(p.accent, grounds, false, 5);
    for (auto *status : {&p.warn, &p.error, &p.robotEnabled, &p.robotKilled})
        *status = readable(*status, grounds, true, 5);
    // tabs stand out from the tab bar behind them (an inactive tab used to vanish into it)
    for (auto *tab : {&t.tab, &t.tabDimmed})
        *tab = pushUntil(*tab, ImVec4(p.text.x, p.text.y, p.text.z, tab->w), 1.2f,
                         [&](ImVec4 c) { return std::min(contrast(c, t.title), contrast(c, t.titleActive)); });

    // Filled "on" controls: darken / lighten the fill until its label reads.
    p.active = fillFor(p.active, p.activeText, 5);
    p.activeHovered = fillFor(p.activeHovered, p.activeText, 5);
    p.activePressed = fillFor(p.activePressed, p.activeText, 5);

    // Enable is green in every theme (go beside KILL's stop red; the labels tell them apart without color): the
    // theme's own green, or its "robot enabled" green deepened under white text
    if (p.enable.w <= 0) {
        p.enableText = {1, 1, 1, 1};
        p.enable = fillFor(mix(p.robotEnabled, ImVec4(0, 0, 0, 1), .25f), p.enableText, 5);
        p.enableHovered = mix(p.enable, ImVec4(1, 1, 1, 1), .1f);
        p.enablePressed = mix(p.enable, ImVec4(0, 0, 0, 1), .15f);
    }
    p.enable = fillFor(p.enable, p.enableText, 5);

    // The figure lamp: the theme's own, or its warning color as a lamp (a pale amber on a light theme)
    if (p.change.w <= 0)
        p.change = relativeLuminance(t.window) > .18f ? mix(p.warn, ImVec4(1, 1, 1, 1), .55f) : p.warn;
    if (p.changeText.w <= 0)
        p.changeText = relativeLuminance(p.change) > .18f ? ImVec4(.05f, .05f, .06f, 1) : ImVec4(1, 1, 1, 1);

    // The board: its own palette held to the same floor against the board, or the panels' colors.
    if (t.hasBoard) {
        auto &b = t.board;
        const std::vector<ImVec4> boardSurfaces{b.bar, t.boardButton, t.boardFrame, t.boardPopup};
        b.muted = readable(b.muted, boardSurfaces, false, 5.5f);
        b.accent = readable(b.accent, {b.bar}, false, 5);
        for (auto *status : {&b.warn, &b.error, &b.robotEnabled, &b.robotKilled})
            *status = readable(*status, {b.bar}, true, 5);
        b.active = fillFor(b.active, b.activeText, 5);
        b.activeHovered = fillFor(b.activeHovered, b.activeText, 5);
        b.activePressed = fillFor(b.activePressed, b.activeText, 5);
        b.change = p.change;
        b.changeText = p.changeText;
        if (b.enable.w <= 0) {
            b.enableText = {1, 1, 1, 1};
            b.enable = fillFor(mix(b.robotEnabled, ImVec4(0, 0, 0, 1), .25f), b.enableText, 5);
            b.enableHovered = mix(b.enable, ImVec4(1, 1, 1, 1), .1f);
            b.enablePressed = mix(b.enable, ImVec4(0, 0, 0, 1), .15f);
        }
        b.enable = fillFor(b.enable, b.enableText, 5);
    } else {
        t.board = p;
        t.boardButton = t.button;
        t.boardButtonHovered = t.buttonHovered;
        t.boardButtonPressed = t.buttonPressed;
        t.boardFrame = t.frame;
        t.boardPopup = t.popup;
        t.boardBorder = t.border;
    }
    return t;
}

} // namespace

// Makes `id` the current theme; without an ImGui context yet only the palette is set. False if unknown.
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

// Before any applyTheme(): the default theme's colors, without touching the ImGui style.
static void ensureDefault() {
    if (currentId.empty()) {
        active = legible(entries().front().spec());
        current = active.palette;
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

// The fonts from setTypeRamp (any may be null: draw with the default font then).
namespace {
TypeRamp ramp;
}

void setTypeRamp(const TypeRamp &fonts) {
    ramp = fonts;
}

const TypeRamp &typeRamp() {
    return ramp;
}

// A section heading in the strong font: a ruled heading on ruled themes, ImGui's separator text otherwise.
void sectionTitle(const char *text) {
    if (ramp.strong)
        ImGui::PushFont(ramp.strong);
    if (active.ruled) { // the text, then a heavy rule the width of the panel; more room above than below
        ImGui::Dummy({0, ui(4)});
        ImGui::TextUnformatted(text);
        const float y = ImGui::GetItemRectMax().y + ui(3);
        const float left = ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMin().x;
        const float right = ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x;
        ImGui::GetWindowDrawList()->AddLine({left, y}, {right, y}, ImGui::GetColorU32(current.text), ui(2));
        ImGui::Dummy({0, ui(4)});
    } else
        ImGui::SeparatorText(text);
    if (ramp.strong)
        ImGui::PopFont();
}

// Table header row in the strong font, ruled underneath on ruled themes.
void tableHeaders() {
    if (ramp.strong)
        ImGui::PushFont(ramp.strong);
    ImGui::TableHeadersRow();
    if (ramp.strong)
        ImGui::PopFont();
    ruleUnderHeaders();
}

// Draws the heavy rule under the current table's header row (ruled themes only).
void ruleUnderHeaders() {
    auto *table = ImGui::GetCurrentTable();
    if (!active.ruled || !table)
        return;
    auto *draw = table->InnerWindow->DrawList;
    const float y = table->RowPosY2 - ui(1);
    draw->PushClipRect(table->InnerClipRect.Min, table->InnerClipRect.Max, false);
    draw->AddLine({table->WorkRect.Min.x, y}, {table->WorkRect.Max.x, y}, ImGui::GetColorU32(current.text), ui(2));
    draw->PopClipRect();
}

bool ruledTheme() {
    return active.ruled;
}

// Status chip corner radius: the frame's corners on square-chip themes, a full capsule otherwise.
float chipRounding(float height) {
    return active.squareChips ? ImGui::GetStyle().FrameRounding : height * .5f;
}

// Palettes saved by beginSurface; kSurfaceColors must match the number of colors it pushes.
namespace {
std::vector<Palette> surfaceStack;
constexpr int kSurfaceColors = 14;
} // namespace

// Switches palette() and the ImGui colors to the board's or the panels' (pushes kSurfaceColors colors).
void beginSurface(Surface surface) {
    surfaceStack.push_back(current);
    const bool board = surface == Surface::Board;
    const Palette &p = board ? active.board : active.palette;
    current = p;
    const std::pair<ImGuiCol, ImVec4> colors[kSurfaceColors] = {
        {ImGuiCol_Text, p.text},
        {ImGuiCol_TextDisabled, p.muted},
        {ImGuiCol_Button, board ? active.boardButton : active.button},
        {ImGuiCol_ButtonHovered, board ? active.boardButtonHovered : active.buttonHovered},
        {ImGuiCol_ButtonActive, board ? active.boardButtonPressed : active.buttonPressed},
        {ImGuiCol_FrameBg, board ? active.boardFrame : active.frame},
        {ImGuiCol_FrameBgHovered, board ? active.boardButtonHovered : active.frameHovered},
        {ImGuiCol_FrameBgActive, board ? active.boardButtonPressed : active.frameActive},
        {ImGuiCol_PopupBg, board ? active.boardPopup : active.popup},
        {ImGuiCol_Border, board ? active.boardBorder : active.border},
        {ImGuiCol_CheckMark, p.accent},
        {ImGuiCol_Header, board ? active.boardButtonHovered : active.header},
        {ImGuiCol_HeaderHovered, board ? active.boardButtonHovered : active.headerHovered},
        {ImGuiCol_HeaderActive, board ? active.boardButtonPressed : active.headerPressed}};
    for (const auto &[index, color] : colors)
        ImGui::PushStyleColor(index, color);
}

// Popup background and border for the given surface (pair with popPopupColors).
void pushPopupColors(Surface surface) {
    const bool board = surface == Surface::Board;
    ImGui::PushStyleColor(ImGuiCol_PopupBg, board ? active.boardPopup : active.popup);
    ImGui::PushStyleColor(ImGuiCol_Border, board ? active.boardBorder : active.border);
}

void popPopupColors() {
    ImGui::PopStyleColor(2);
}

// Restores the colors and palette from before the matching beginSurface.
void endSurface() {
    if (surfaceStack.empty())
        return;
    ImGui::PopStyleColor(kSurfaceColors);
    current = surfaceStack.back();
    surfaceStack.pop_back();
}

} // namespace nereus::ros_viewer

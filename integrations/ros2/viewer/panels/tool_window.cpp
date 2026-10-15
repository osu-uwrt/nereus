// Tool windows (Scene, Display, TF frames, Simulation, Help, the scorecard): placement under their toolbar button,
// closing at a click outside while they float, and the title-bar pin that keeps one open.
#include "nereus/ros_viewer/panel_layout.hpp"
#include <cmath>
#include <cstring>
#include <imgui_internal.h>
#include <set>
#include <string>
#include <vector>

namespace nereus::ros_viewer {
namespace {

// A button that toggles a tool window, as last drawn: the window's open flag, the button's ID, rectangle and frame.
struct ToggleButton {
    const bool *open;
    ImGuiID id;
    ImRect rect;
    int frame;
};

std::vector<ToggleButton> &toggleButtons() {
    static std::vector<ToggleButton> buttons;
    return buttons;
}

// The pinned (kept open) tool windows by key: the name's ID part after "###", else the whole name.
std::set<std::string> &keptOpen() {
    static std::set<std::string> kept;
    return kept;
}

std::string windowKey(const char *name) {
    const char *id = std::strstr(name, "###");
    return id ? id + 3 : name;
}

// Whether `at` is on a button that toggles this window (drawn this frame or the last).
bool onToggleButton(const bool *open, ImVec2 at) {
    const int frame = ImGui::GetFrameCount();
    for (const auto &button : toggleButtons())
        if (button.open == open && button.frame >= frame - 1 && button.rect.Contains(at))
            return true;
    return false;
}

// A press of any mouse button this frame away from the current window (its child windows and popups included).
// Not while a popup is open (that press closes the popup first) nor on one of the window's toggle buttons.
bool clickedOutside(const bool *open) {
    int button = -1;
    for (int b = 0; b < 3 && button < 0; ++b)
        if (ImGui::IsMouseClicked(b))
            button = b;
    if (button < 0 || ImGui::IsWindowAppearing() ||
        ImGui::IsPopupOpen(nullptr, ImGuiPopupFlags_AnyPopupId | ImGuiPopupFlags_AnyPopupLevel))
        return false;
    if (ImGui::IsWindowHovered(ImGuiHoveredFlags_RootAndChildWindows | ImGuiHoveredFlags_AllowWhenBlockedByPopup |
                               ImGuiHoveredFlags_AllowWhenBlockedByActiveItem))
        return false;
    return !onToggleButton(open, ImGui::GetIO().MouseClickedPos[button]);
}

// A pushpin in a box `size` wide around `center`: upright while it keeps the window open, tilted 45 degrees
// (head up and right) while not.
void drawPin(ImDrawList *draw, ImVec2 center, float size, bool upright, ImU32 color) {
    const float half = size * .5f, angle = upright ? 0.f : .7853982f;
    const float c = std::cos(angle), s = std::sin(angle);
    // Points in units of half the box, y down, turned by `angle`.
    const auto at = [&](float x, float y) {
        return ImVec2(center.x + half * (x * c - y * s), center.y + half * (x * s + y * c));
    };
    const auto quad = [&](float x0, float y0, float x1, float y1) {
        const ImVec2 corners[4] = {at(x0, y0), at(x1, y0), at(x1, y1), at(x0, y1)};
        draw->AddConvexPolyFilled(corners, 4, color);
    };
    quad(-.3f, -.8f, .3f, -.25f);                                              // head
    quad(-.6f, -.3f, .6f, -.12f);                                              // plate
    draw->AddLine(at(0, -.12f), at(0, .85f), color, std::max(1.f, size / 14)); // needle
}

// The pin left of the close box in the current (floating) window's title bar. Returns true when clicked. The
// title bar stays the last item, so a context menu the caller attaches next still opens on it (IsItemHovered
// also needs the window not yet written to after Begin).
bool titleBarPin(bool kept) {
    ImGuiContext &g = *GImGui;
    ImGuiWindow *window = g.CurrentWindow;
    const ImGuiStyle &style = g.Style;
    const ImRect title = window->TitleBarRect();
    const float size = g.FontSize;
    // ImGui puts the close box FramePadding.x in from the right edge; the pin sits ItemInnerSpacing.x left of it.
    const ImVec2 pos(title.Max.x - style.FramePadding.x - 2 * size - style.ItemInnerSpacing.x,
                     title.Min.y + style.FramePadding.y);
    const ImRect box(pos, {pos.x + size, pos.y + size});
    const ImGuiID id = window->GetID("#KEEP_OPEN");

    const ImGuiLastItemData last = g.LastItemData;
    const bool written = window->WriteAccessed;
    const ImGuiNavLayer layer = window->DC.NavLayerCurrent;
    window->DC.NavLayerCurrent = ImGuiNavLayer_Menu; // as ImGui's own title bar buttons
    ImGui::PushClipRect(title.Min, title.Max, false);
    bool hovered = false, held = false, pressed = false;
    if (ImGui::ItemAdd(box, id, nullptr, ImGuiItemFlags_NoNavDefaultFocus)) {
        pressed = ImGui::ButtonBehavior(box, id, &hovered, &held);
        if (hovered)
            window->DrawList->AddRectFilled(box.Min, box.Max,
                                            ImGui::GetColorU32(held ? ImGuiCol_ButtonActive : ImGuiCol_ButtonHovered));
        drawPin(window->DrawList, box.GetCenter(), size, kept,
                kept ? ImGui::GetColorU32(ImGuiCol_Text) : ImGui::GetColorU32(palette().muted));
    }
    ImGui::PopClipRect();
    window->DC.NavLayerCurrent = layer;
    g.LastItemData = last;
    window->WriteAccessed = written;
    if (hovered)
        ImGui::SetTooltip(kept ? "Kept open: a click outside leaves it. Click to let a click outside close it."
                               : "Keep open when clicking outside the window");
    return pressed;
}

} // namespace

void markToolWindowButton(const bool *open) {
    const ImGuiID id = ImGui::GetItemID();
    const ImRect rect(ImGui::GetItemRectMin(), ImGui::GetItemRectMax());
    const int frame = ImGui::GetFrameCount();
    for (auto &button : toggleButtons())
        if (button.open == open && button.id == id) {
            button.rect = rect;
            button.frame = frame;
            return;
        }
    toggleButtons().push_back({open, id, rect, frame});
}

bool beginToolWindow(const char *name, bool *open, ImVec2 size, ImVec2 anchor, bool focus) {
    // The usable area: the main viewport's work area inset by a margin (at least 1 px each way).
    const float margin = ui(12);
    const auto *viewport = ImGui::GetMainViewport();
    const ImVec2 low(viewport->WorkPos.x + margin, viewport->WorkPos.y + margin);
    const ImVec2 available(std::max(1.f, viewport->WorkSize.x - 2 * margin),
                           std::max(1.f, viewport->WorkSize.y - 2 * margin));

    // First-use size and position (clamped so the window starts fully on screen), then size limits.
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
    if (ImGui::IsWindowDocked())
        return visible; // part of the layout: it stays until closed

    // Pull a floating window back inside the work area if it was dragged (or resized) past an edge.
    if (!ImGui::IsWindowAppearing()) {
        const auto pos = ImGui::GetWindowPos(), extent = ImGui::GetWindowSize();
        const ImVec2 clamped(std::clamp(pos.x, low.x, low.x + std::max(0.f, available.x - extent.x)),
                             std::clamp(pos.y, low.y, low.y + std::max(0.f, available.y - extent.y)));
        if (clamped.x != pos.x || clamped.y != pos.y)
            ImGui::SetWindowPos(clamped);
    }

    // Floating: the pin keeps it open, else a click anywhere outside closes it.
    const auto key = windowKey(name);
    bool kept = keptOpen().count(key) > 0;
    if (titleBarPin(kept)) {
        kept = !kept;
        if (kept)
            keptOpen().insert(key);
        else
            keptOpen().erase(key);
        ImGui::MarkIniSettingsDirty();
    }
    if (!kept && *open && clickedOutside(open))
        *open = false;
    return visible;
}

// One line per pinned tool window: its key.
void installToolWindows() {
    ImGuiSettingsHandler handler;
    handler.TypeName = "NereusToolWindows";
    handler.TypeHash = ImHashStr("NereusToolWindows");
    handler.ClearAllFn = [](ImGuiContext *, ImGuiSettingsHandler *) { keptOpen().clear(); };
    handler.ReadOpenFn = [](ImGuiContext *, ImGuiSettingsHandler *, const char *name) -> void * {
        return std::strcmp(name, "KeepOpen") == 0 ? &keptOpen() : nullptr;
    };
    handler.ReadLineFn = [](ImGuiContext *, ImGuiSettingsHandler *, void *, const char *line) {
        if (*line)
            keptOpen().insert(line);
    };
    handler.WriteAllFn = [](ImGuiContext *, ImGuiSettingsHandler *, ImGuiTextBuffer *out) {
        out->append("[NereusToolWindows][KeepOpen]\n");
        for (const auto &key : keptOpen())
            out->appendf("%s\n", key.c_str());
        out->append("\n");
    };
    ImGui::AddSettingsHandler(&handler);
}

} // namespace nereus::ros_viewer

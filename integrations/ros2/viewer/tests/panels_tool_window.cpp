// Tool windows (headless ImGui): a floating one closes at a press outside it but not on its toggle button, inside
// it or while a popup is open; its title-bar pin keeps it open and survives the ini; a docked one stays. A floating
// one is sized to its contents: fitted on opening, grown with them, never shrunk while open, capped at the display.
#include "nereus/ros_viewer/panel_layout.hpp"
#include <cassert>
#include <cfloat>
#include <imgui.h>
#include <imgui_internal.h>
#include <iostream>
#include <string>

using namespace nereus::ros_viewer;

int main() {
    ImGui::CreateContext();
    auto &io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.DisplaySize = {1000, 700};
    io.DeltaTime = 1.f / 30;
    unsigned char *pixels;
    int w, h;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &w, &h);
    installToolWindows();

    const char *name = "Display###display", *dockedName = "Docked###docked";
    const ImGuiID dockspace = ImHashStr("test_dockspace");
    bool open = false, docked = true, dockPending = true, openMenu = false;
    int lines = 1;     // the floating window's contents: lines of text,
    bool fill = false; // or a box filling the window (at least 420 x 240) above two lines, as the TF frames table

    // One viewer frame: the dock space with one docked tool window, the toolbar with the toggle (and a popup
    // when asked), then the floating tool window with a context menu on its title bar.
    const auto frame = [&] {
        ImGui::NewFrame();
        ImGui::DockSpaceOverViewport(dockspace, ImGui::GetMainViewport());
        if (dockPending) {
            ImGui::DockBuilderDockWindow(dockedName, dockspace);
            dockPending = false;
        }
        if (docked) {
            beginToolWindow(dockedName, &docked, {300, 200}, {0, 0});
            ImGui::End();
        }
        ImGui::SetNextWindowPos({0, 0});
        ImGui::SetNextWindowSize({600, 60});
        ImGui::Begin("toolbar", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoDocking);
        windowToggle("Display", &open);
        if (openMenu) {
            ImGui::OpenPopup("menu");
            openMenu = false;
        }
        if (ImGui::BeginPopup("menu")) {
            ImGui::TextUnformatted("a menu item");
            ImGui::EndPopup();
        }
        ImGui::End();
        if (open) {
            beginToolWindow(name, &open, {300, 200}, {100, 100});
            if (ImGui::BeginPopupContextItem("##window_menu")) { // as the viewer's windowContextMenu
                ImGui::TextUnformatted("Pin to toolbar");
                ImGui::EndPopup();
            }
            if (fill) {
                const ImVec2 available = ImGui::GetContentRegionAvail();
                const float below = 2 * ImGui::GetTextLineHeightWithSpacing();
                ImGui::BeginChild("box", {std::max(420.f, available.x), std::max(240.f, available.y - below)},
                                  ImGuiChildFlags_Borders);
                ImGui::EndChild();
                ImGui::TextUnformatted("first line below");
                ImGui::TextUnformatted("second line below");
            } else
                for (int i = 0; i < lines; ++i)
                    ImGui::Text("setting %d", i);
            ImGui::End();
        }
        ImGui::Render();
    };
    const auto click = [&](ImVec2 at, ImGuiMouseButton button = ImGuiMouseButton_Left) {
        io.AddMousePosEvent(at.x, at.y);
        frame();
        io.AddMouseButtonEvent(button, true);
        frame();
        io.AddMouseButtonEvent(button, false);
        frame();
        frame();
    };
    const ImVec2 toggle{14, 14}, outside{900, 650};
    const auto window = [&] { return ImGui::FindWindowByName(name); };
    const auto inside = [&] { return window()->Rect().GetCenter(); };
    const auto popups = [] { return ImGui::GetCurrentContext()->OpenPopupStack.Size; };

    frame();
    frame();
    assert(docked && ImGui::FindWindowByName(dockedName)->DockIsActive);

    // The toggle opens it, a press inside keeps it, the toggle closes it (not closed by the press, then reopened).
    click(toggle);
    assert(open && !window()->DockIsActive);
    click(inside());
    assert(open);
    click(toggle);
    assert(!open);

    // A press outside with any button closes it; the docked window stays.
    click(toggle);
    assert(open);
    click(outside);
    assert(!open && docked);
    click(toggle);
    click(outside, ImGuiMouseButton_Right);
    assert(!open && docked);

    // While a popup is open, the press outside closes only the popup.
    click(toggle);
    openMenu = true;
    frame();
    assert(popups() == 1);
    click(outside);
    assert(open && popups() == 0);

    // A right-click on the title bar opens its context menu (the title bar is still the last item after the pin).
    const ImRect title = window()->TitleBarRect();
    click({title.Min.x + 20, title.GetCenter().y}, ImGuiMouseButton_Right);
    assert(open && popups() == 1);
    click(outside);
    assert(open && popups() == 0);

    // The pin, left of the close box, keeps it open at a press outside.
    const auto &style = ImGui::GetStyle();
    const float size = ImGui::GetFontSize();
    const ImVec2 pin{title.Max.x - style.FramePadding.x - 1.5f * size - style.ItemInnerSpacing.x,
                     title.Min.y + style.FramePadding.y + .5f * size};
    click(pin);
    click(outside);
    assert(open);

    // The pin is kept in the ini; cleared, outside closes it again; loaded back, it is kept. (Only this section:
    // clearing all settings would also undock the docked window.)
    const std::string section = "[NereusToolWindows][KeepOpen]\ndisplay\n";
    assert(std::string(ImGui::SaveIniSettingsToMemory()).find(section) != std::string::npos);
    auto *handler = ImGui::FindSettingsHandler("NereusToolWindows");
    handler->ClearAllFn(ImGui::GetCurrentContext(), handler);
    click(outside);
    assert(!open);
    ImGui::LoadIniSettingsFromMemory(section.c_str(), section.size());
    dockPending = true; // loading ini text resets the dock tree as well
    click(toggle);
    click(outside);
    assert(open);

    // Unpinned again: a press outside closes it.
    click(pin);
    click(outside);
    assert(!open && docked);

    // Opening fits the contents; growing contents grow it; shrinking ones leave it while open; the next opening fits.
    const auto fitsAll = [&] { return window()->ScrollMax.y == 0; };
    const auto settle = [&] {
        for (int i = 0; i < 3; ++i)
            frame();
    };
    lines = 10;
    click(toggle);
    settle();
    assert(open && fitsAll());
    const float ten = window()->Size.y;
    lines = 25;
    settle();
    assert(fitsAll() && window()->Size.y > ten);
    const float grown = window()->Size.y;
    lines = 3;
    settle();
    assert(window()->Size.y == grown);
    click(toggle);
    assert(!open);
    click(toggle);
    settle();
    assert(open && fitsAll() && window()->Size.y < ten);

    // The operator's resize holds until the contents grow.
    lines = 12;
    settle();
    ImGui::SetWindowSize(name, {300, 120});
    settle();
    assert(std::abs(window()->Size.y - 120) < 1 && !fitsAll());
    lines = 20;
    settle();
    assert(fitsAll() && window()->Size.y > 120);

    // Contents taller than the display: the window stops at the work area (12 px margins) and scrolls.
    lines = 400;
    settle();
    assert(window()->Size.y <= io.DisplaySize.y - 24 + 1 && window()->Pos.y >= 12 - 1 && !fitsAll());
    assert(docked && ImGui::FindWindowByName(dockedName)->DockIsActive);

    // A box filling the window: the window keeps what the operator sets, larger (the box fills it exactly, no
    // scrolling) or smaller (it scrolls), without growing on its own; opened again it fits the box's minimum.
    const auto extent = [&] { return window()->Size; };
    const auto near = [](ImVec2 a, ImVec2 b) { return std::abs(a.x - b.x) < 1 && std::abs(a.y - b.y) < 1; };
    click(toggle);
    fill = true;
    click(toggle);
    settle();
    const ImVec2 first = extent();
    settle();
    assert(open && fitsAll() && near(extent(), first));
    ImGui::SetWindowSize(name, {640, 520});
    settle();
    settle();
    assert(near(extent(), {640, 520}) && fitsAll());
    ImGui::SetWindowSize(name, {300, 150});
    settle();
    settle();
    assert(near(extent(), {300, 150}) && !fitsAll());
    click(toggle);
    click(toggle);
    settle();
    const ImVec2 minimum = extent();
    assert(fitsAll() && minimum.x >= 420 && minimum.x < 500 && minimum.y >= 240 && minimum.y < 360);
    settle();
    assert(near(extent(), minimum));

    ImGui::DestroyContext();
    std::cout << "panels_tool_window: ok\n";
}

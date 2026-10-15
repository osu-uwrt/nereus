// Tool windows (headless ImGui): a floating one closes at a press outside it but not on its toggle button, inside
// it or while a popup is open; its title-bar pin keeps it open and survives the ini; a docked one stays.
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
            ImGui::TextUnformatted("settings");
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

    ImGui::DestroyContext();
    std::cout << "panels_tool_window: ok\n";
}

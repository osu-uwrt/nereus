// Built-in dock layouts, window open state in the ini, and restoring a layout mid-session (headless ImGui).
#include "nereus/ros_viewer/dock_layout.hpp"
#include <algorithm>
#include <cassert>
#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <imgui_internal.h>
#include <iostream>
#include <unistd.h>

using namespace nereus::ros_viewer;
namespace {
struct Win {
    std::string name;
    Dock area;
    bool stacked = false, selected = false, open = true;
};
ImGuiWindow *find(const std::string &name) {
    return ImGui::FindWindowByName(name.c_str());
}
} // namespace

int main() {
    ImGui::CreateContext();
    auto &io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.DisplaySize = {1600, 900};
    io.DeltaTime = 1.f / 30;
    unsigned char *pixels;
    int w, h;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &w, &h);
    const ImGuiID dockspace = ImHashStr("test_dockspace");
    const std::string pool = "Pool view###pool";
    std::vector<Win> windows{{"Motion###panel.motion", Dock::LeftTop},
                             {"Mapping###panel.mapping", Dock::Left},
                             {"Autonomy###panel.autonomy", Dock::Left, false, true},
                             {"FFC###camera.ffc", Dock::Right, true},
                             {"DFC###camera.dfc", Dock::Right, true},
                             {"Course map###map", Dock::RightBottom},
                             {"Electrical###panel.electrical", Dock::Floating}};
    WindowStates states([&] {
        WindowStates::Flags flags;
        for (auto &window : windows)
            flags.emplace_back(window.name.substr(window.name.find("###") + 3), &window.open);
        return flags;
    });
    states.install();
    std::string pendingPreset = "standard", pendingIni;
    ImGuiDockNodeFlags dockFlags = 0;
    const auto frame = [&] {
        if (!pendingIni.empty()) { // as the viewer: between frames, before NewFrame
            ImGui::LoadIniSettingsFromMemory(pendingIni.c_str(), pendingIni.size());
            pendingIni.clear();
        }
        ImGui::NewFrame();
        if (!pendingPreset.empty()) {
            std::vector<LayoutWindow> layout;
            for (const auto &window : windows)
                layout.push_back({window.name, window.area, window.stacked, window.selected});
            buildLayout(dockspace, ImGui::GetMainViewport()->WorkSize, *findPreset(pendingPreset), pool, layout);
            pendingPreset.clear();
        }
        ImGui::DockSpaceOverViewport(dockspace, ImGui::GetMainViewport(), dockFlags);
        ImGui::Begin(pool.c_str());
        ImGui::End();
        for (auto &window : windows) {
            if (!window.open)
                continue;
            ImGui::Begin(window.name.c_str(), &window.open);
            ImGui::TextUnformatted(window.name.c_str());
            ImGui::End();
        }
        states.update();
        ImGui::Render();
    };
    const auto settle = [&] {
        for (int i = 0; i < 4; ++i)
            frame();
    };
    const auto node = [&](const std::string &name) { return find(name)->DockNode; };
    const auto shown = [&](const std::string &name) {
        auto *window = find(name);
        return window && window->DockNode && window->DockNode->TabBar &&
               window->DockNode->TabBar->VisibleTabId == window->TabId;
    };
    const auto checkStandard = [&] {
        assert(node(pool) && node(pool)->IsCentralNode());
        for (int i = 0; i < 6; ++i)
            assert(find(windows[i].name)->DockIsActive);
        assert(!find(windows[6].name)->DockIsActive); // floating
        assert(node(windows[1].name) == node(windows[2].name));
        assert(node(windows[0].name) != node(windows[1].name));
        assert(shown(windows[2].name) && !shown(windows[1].name)); // `selected` tab shown, the other behind it
        const auto *motion = find(windows[0].name), *mapping = find(windows[1].name), *poolView = find(pool),
                   *ffc = find(windows[3].name), *dfc = find(windows[4].name), *map = find(windows[5].name);
        assert(std::abs(motion->Pos.x) < 1 && motion->Pos.y < mapping->Pos.y);
        assert(std::abs(poolView->Pos.x - .24f * 1600) < 8);
        assert(ffc->Pos.x > poolView->Pos.x + poolView->Size.x - 1 && std::abs(ffc->Pos.x - dfc->Pos.x) < 1);
        assert(ffc->Pos.y < dfc->Pos.y && dfc->Pos.y < map->Pos.y);
        assert(std::abs(ffc->Size.y - dfc->Size.y) < 4); // stacked: even shares
    };
    settle();
    checkStandard();
    // Lock layout (the viewer's dock space flags) keeps the arrangement as it is.
    dockFlags = ImGuiDockNodeFlags_NoUndocking | ImGuiDockNodeFlags_NoResize | ImGuiDockNodeFlags_NoDocking |
                ImGuiDockNodeFlags_NoCloseButton;
    settle();
    checkStandard();
    dockFlags = ImGuiDockNodeFlags_NoCloseButton;

    // In front: shown and not behind another tab of its node (checked before the windows' Begin, as the toolbar).
    ImGui::NewFrame();
    assert(windowInFront(windows[0].name.c_str()) && windowInFront(windows[2].name.c_str()));
    assert(!windowInFront(windows[1].name.c_str())); // a tab behind Autonomy
    assert(windowInFront(windows[6].name.c_str()) && !windowInFront("nothing###none"));
    ImGui::EndFrame();
    // Right-click on a docked window's tab opens a context menu called right after its Begin.
    {
        const auto *motion = find(windows[0].name); // alone in its node, its tab still shown
        const ImVec2 tab((motion->DC.DockTabItemRect.Min.x + motion->DC.DockTabItemRect.Max.x) / 2,
                         (motion->DC.DockTabItemRect.Min.y + motion->DC.DockTabItemRect.Max.y) / 2);
        bool opened = false;
        const auto menuFrame = [&] {
            ImGui::NewFrame();
            ImGui::DockSpaceOverViewport(dockspace, ImGui::GetMainViewport(), dockFlags);
            for (auto &window : windows)
                if (window.open && ImGui::Begin(window.name.c_str(), &window.open)) {
                    if (ImGui::BeginPopupContextItem("##window_menu")) {
                        opened = opened || &window == &windows[0];
                        ImGui::EndPopup();
                    }
                    ImGui::TextUnformatted(window.name.c_str());
                    ImGui::End();
                } else if (window.open)
                    ImGui::End();
            ImGui::Begin(pool.c_str());
            ImGui::End();
            ImGui::Render();
        };
        io.AddMousePosEvent(tab.x, tab.y);
        menuFrame();
        io.AddMouseButtonEvent(ImGuiMouseButton_Right, true);
        menuFrame();
        io.AddMouseButtonEvent(ImGuiMouseButton_Right, false);
        menuFrame();
        menuFrame();
        assert(opened);
        io.AddKeyEvent(ImGuiKey_Escape, true);
        menuFrame();
        io.AddKeyEvent(ImGuiKey_Escape, false);
        io.AddMousePosEvent(-FLT_MAX, -FLT_MAX);
        settle();
    }

    // Sides: what is docked left / right of the pool view, their widths, and closing one (snap shut).
    {
        const float left = sideWidth(dockspace, Side::Left), right = sideWidth(dockspace, Side::Right);
        assert(std::abs(left - .24f * 1600) < 8 && right > 300);
        auto names = sideWindows(dockspace, Side::Left);
        std::sort(names.begin(), names.end());
        assert((names == std::vector<std::string>{windows[2].name, windows[1].name, windows[0].name}));
        assert(sideWindows(dockspace, Side::Right).size() == 3); // the feeds and the map
        for (int i = 0; i < 3; ++i)
            windows[i].open = false;
        settle();
        assert(sideWidth(dockspace, Side::Left) == 0 && find(pool)->Pos.x == 0);
        assert(sideWindows(dockspace, Side::Left).empty() && !setSideWidth(dockspace, Side::Left, 300));
        for (int i = 0; i < 3; ++i)
            windows[i].open = true;
        settle();
        assert(std::abs(sideWidth(dockspace, Side::Left) - left) < 8); // back where it was
        assert(setSideWidth(dockspace, Side::Left, 500));
        settle();
        assert(std::abs(sideWidth(dockspace, Side::Left) - 500) < 4);
        // Snap shut as the viewer does: dragged narrow, back to the remembered width and closed in one frame;
        // reopening then gives the remembered width, not the narrow one.
        assert(setSideWidth(dockspace, Side::Left, 60));
        settle();
        assert(sideWidth(dockspace, Side::Left) < 100);
        ImGui::NewFrame();
        ImGui::DockSpaceOverViewport(dockspace, ImGui::GetMainViewport(), dockFlags);
        assert(setSideWidth(dockspace, Side::Left, 420));
        for (int i = 0; i < 3; ++i)
            windows[i].open = false;
        ImGui::EndFrame();
        settle();
        assert(sideWidth(dockspace, Side::Left) == 0);
        for (int i = 0; i < 3; ++i)
            windows[i].open = true;
        settle();
        assert(std::abs(sideWidth(dockspace, Side::Left) - 420) < 4);
        assert(setSideWidth(dockspace, Side::Right, 300));
        settle();
        assert(std::abs(sideWidth(dockspace, Side::Right) - 300) < 4);
        pendingPreset = "standard";
        settle();
        checkStandard();
    }

    // The ini carries the dock tree and which windows are open.
    std::string saved = ImGui::SaveIniSettingsToMemory();
    assert(saved.find("[Nereus][Windows]") != std::string::npos);
    assert(saved.find("panel.mapping=1") != std::string::npos);

    // Maximize the pool view (close the rest), then restore the snapshot mid-session.
    for (auto &window : windows)
        window.open = false;
    settle();
    assert(ImGui::GetCurrentContext()->SettingsDirtyTimer > 0); // closing windows is a layout change to save
    assert(std::abs(find(pool)->Size.x - 1600) < 1);
    pendingIni = saved;
    settle();
    for (const auto &window : windows)
        assert(window.open);
    checkStandard();

    // Another preset moves the feeds and the map under the pool view, side by side.
    pendingPreset = "wide";
    settle();
    {
        const auto *poolView = find(pool), *ffc = find(windows[3].name), *dfc = find(windows[4].name),
                   *map = find(windows[5].name);
        assert(ffc->Pos.y > poolView->Pos.y + poolView->Size.y - 1);
        assert(ffc->Pos.x < dfc->Pos.x && dfc->Pos.x < map->Pos.x && std::abs(ffc->Pos.y - map->Pos.y) < 1);
        assert(std::abs(poolView->Pos.x - .22f * 1600) < 8);
    }
    pendingPreset = "cameras";
    settle();
    {
        const auto *poolView = find(pool), *ffc = find(windows[3].name), *map = find(windows[5].name);
        assert(std::abs(ffc->Pos.x + ffc->Size.x - 1600) < 1 && ffc->Size.x > .4f * 1600 - 8); // wide feeds
        assert(map->Pos.y > ffc->Pos.y && poolView->Size.x < .4f * 1600);
    }

    // States of windows that do not exist now survive a save; a reopened window keeps its saved state.
    pendingIni = "[Nereus][Windows]\nmap=0\ncamera.gone=1\n";
    settle();
    assert(!windows[5].open && windows[3].open);
    saved = ImGui::SaveIniSettingsToMemory();
    assert(saved.find("camera.gone=1") != std::string::npos && saved.find("map=0") != std::string::npos);
    assert(states.saved("camera.gone") && !states.saved("nothing"));

    assert(layoutFileStem("  Pool day / 2 ") == "Pool day  2");
    assert(layoutFileStem("../..") == "" && layoutFileStem("a_b-c") == "a_b-c");
    const auto dir = std::filesystem::temp_directory_path() / ("nereus_layouts_" + std::to_string(::getpid()));
    std::filesystem::create_directories(dir);
    for (const char *name : {"zeta.ini", "alpha.ini", "notes.txt"})
        std::ofstream(dir / name) << "x";
    assert((savedLayouts(dir) == std::vector<std::string>{"alpha", "zeta"}));
    assert(savedLayouts(dir / "missing").empty());
    std::filesystem::remove_all(dir);
    setenv("XDG_CONFIG_HOME", "/tmp/xdg", 1);
    assert(configDirectory() == "/tmp/xdg/nereus");
    assert(findPreset("standard") && findPreset("wide") && findPreset("cameras") && !findPreset("nope"));
    ImGui::DestroyContext();
    std::cout << "PASS: dock presets, tab selection, window states in the ini, mid-session restore\n";
}

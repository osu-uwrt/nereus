// Controls pinned to the toolbar (headless ImGui): the pinned copy drives the original button / checkbox / combo,
// follows its disabled state, keeps working with its window not drawn, and survives the ini.
#include "nereus/ros_viewer/pins.hpp"
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
    io.DisplaySize = {1000, 700};
    io.DeltaTime = 1.f / 30;
    unsigned char *pixels;
    int w, h;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &w, &h);
    pins::install();
    int fired = 0, started = 0, mode = 0;
    bool armed = false, fireDisabled = false, panelShown = true;
    const auto panel = [&] {
        ImGui::BeginDisabled(fireDisabled);
        if (pins::Button("Fire###fire"))
            ++fired;
        ImGui::EndDisabled();
        pins::Checkbox("Arm", &armed);
        pins::Combo("Mode", &mode, "One\0Two\0Three\0");
        pins::Scope camera("ffc");
        if (pins::Button("Start FFC###start"))
            ++started;
    };
    const auto frame = [&] {
        ImGui::NewFrame();
        pins::newFrame();
        ImGui::SetNextWindowPos({0, 0});
        ImGui::SetNextWindowSize({600, 60});
        ImGui::Begin("toolbar", nullptr, ImGuiWindowFlags_NoDecoration);
        pins::drawPinned();
        ImGui::End();
        if (panelShown) {
            ImGui::SetNextWindowPos({0, 200});
            ImGui::SetNextWindowSize({400, 300});
            ImGui::Begin("panel", nullptr, ImGuiWindowFlags_NoDecoration);
            pins::beginScope("panel.a", "Panel A");
            panel();
            pins::endScope();
            ImGui::End();
        }
        if (pins::needsDrawing("panel.a"))
            pins::drawOffscreen("panel.a", "Panel A", panel);
        pins::drawMenu();
        ImGui::Render();
    };
    // Click the first control in the toolbar (window padding 8, frame padding 4 by default).
    const auto clickFirstPinned = [&] {
        io.AddMousePosEvent(12, 10); // moved there, as a real mouse is (keyboard use turns hover off until it moves)
        frame();
        io.AddMousePosEvent(16, 14);
        frame();
        io.AddMouseButtonEvent(ImGuiMouseButton_Left, true);
        frame();
        io.AddMouseButtonEvent(ImGuiMouseButton_Left, false);
        frame();
        frame();
        io.AddMousePosEvent(-FLT_MAX, -FLT_MAX);
    };
    frame();
    assert(!pins::any() && !pins::needsDrawing("panel.a"));

    // A pinned button: its copy fires the original once per click; disabled with it.
    pins::setPinned("panel.a/fire", true);
    assert(pins::pinnedKeys() == std::vector<std::string>{"panel.a/fire"});
    clickFirstPinned();
    assert(fired == 1);
    fireDisabled = true;
    clickFirstPinned();
    assert(fired == 1);
    fireDisabled = false;
    pins::setPinned("panel.a/fire", false);

    // A pinned checkbox toggles the original's value; a per-camera button keeps its scope part in the key.
    pins::setPinned("panel.a/Arm", true);
    clickFirstPinned();
    assert(armed);
    pins::setPinned("panel.a/Arm", false);
    pins::setPinned("panel.a/ffc/start", true);
    clickFirstPinned();
    assert(started == 1);

    // With its window not drawn, the panel runs off screen and the copy still works.
    panelShown = false;
    frame();
    assert(started == 1);
    clickFirstPinned();
    assert(started == 2);
    panelShown = true;

    // Right-click a widget in the panel: the pin menu opens.
    io.AddMousePosEvent(20, 210); // the Fire button
    frame();
    io.AddMouseButtonEvent(ImGuiMouseButton_Right, true);
    frame();
    io.AddMouseButtonEvent(ImGuiMouseButton_Right, false);
    frame();
    assert(ImGui::GetCurrentContext()->OpenPopupStack.Size == 1);
    io.AddMousePosEvent(900, 650); // click elsewhere to close it
    frame();
    io.AddMouseButtonEvent(ImGuiMouseButton_Left, true);
    frame();
    io.AddMouseButtonEvent(ImGuiMouseButton_Left, false);
    frame();
    assert(ImGui::GetCurrentContext()->OpenPopupStack.Size == 0);

    // Pins survive the ini (saved layouts) with their labels.
    pins::setPinned("panel.a/Mode", true);
    const std::string ini = ImGui::SaveIniSettingsToMemory();
    assert(ini.find("[NereusPins][Toolbar]") != std::string::npos);
    assert(ini.find("panel.a/ffc/start\tStart FFC\tPanel A") != std::string::npos);
    assert(ini.find("combo\tpanel.a/Mode") != std::string::npos);
    pins::clear();
    assert(!pins::any());
    ImGui::LoadIniSettingsFromMemory(ini.c_str(), ini.size());
    assert((pins::pinnedKeys() == std::vector<std::string>{"panel.a/ffc/start", "panel.a/Mode"}));
    frame();
    (void)mode;
    ImGui::DestroyContext();
    std::cout << "PASS: pinned buttons, checkboxes, scoped keys, disabled state, off-screen panels, ini\n";
}

#include "nereus/ros_viewer/panel_layout.hpp"
#include "nereus/ros_viewer/panels/composition.hpp"
#include "nereus/ros_viewer/pins.hpp"
#include <algorithm>
#include <cmath>
#include <imgui.h>
namespace nereus::ros_viewer::panels {
namespace {
class SimulationPanel final : public Panel {
    std::shared_ptr<Simulation> simulation;
    float draft = 1;
    bool dirty = false, open = false;
    ImVec2 anchor{80, 120};

  public:
    explicit SimulationPanel(const Binding &b) : simulation(std::dynamic_pointer_cast<Simulation>(b.provider)) {}
    // Toolbar: a button for the Simulation window (speed, pause, sync, reset), which stays open while in use and
    // can be docked like any panel.
    void toolbar() override {
        nereus::ros_viewer::windowToggle("Simulation", &open);
        anchor = {ImGui::GetItemRectMin().x, ImGui::GetItemRectMax().y + ImGui::GetStyle().ItemSpacing.y};
    }
    void windowMenu() override {
        if (ImGui::MenuItem("Simulation", nullptr, open))
            open = !open;
    }
    void drawWindows() override {
        if (!open)
            return;
        // The instance's ID keeps several simulation tools' windows apart.
        const auto name = "Simulation###simulation_" + std::to_string(ImGui::GetID("simulation"));
        if (nereus::ros_viewer::beginToolWindow(name.c_str(), &open, ui(ImVec2(340, 0)), anchor))
            draw();
        ImGui::End();
    }
    void draw() override {
        const auto s = simulation ? simulation->state() : SimulationState{};
        const bool paused = s.rate == 0;
        if (!dirty)
            draft = paused ? s.resumeRate : s.rate;
        if (!simulation)
            emptyState("Connected to the simulator, this sets its speed, pauses it, and syncs or resets it.");
        else if (!s.message.empty())
            ImGui::TextWrapped("%s", s.message.c_str());
        if (s.connected) {
            ImGui::BeginDisabled(paused);
            ImGui::Text("Selected speed: %.2fx", paused ? s.resumeRate : s.rate);
            ImGui::EndDisabled();
        }
        ImGui::BeginDisabled(!simulation || !s.connected || s.pending);
        ImGui::BeginDisabled(paused);
        ImGui::SetNextItemWidth(ui(180));
        if (ImGui::InputFloat("Speed", &draft, .25f, 1.f, "%.2fx"))
            dirty = true;
        ImGui::BeginDisabled(!std::isfinite(draft) || draft <= 0 || draft > s.maxRate);
        if (pins::Button(s.pending ? "Applying...###apply" : "Apply###apply", ui(ImVec2(90, 36)))) {
            simulation->setRate(draft);
            dirty = false;
        }
        ImGui::EndDisabled();
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (pins::Button(paused ? "Resume###pause" : "Pause###pause", ui(ImVec2(80, 36)))) {
            simulation->setPaused(!paused);
            dirty = false;
        }
        ImGui::SameLine();
        ImGui::BeginDisabled(paused || s.maxRate < 1);
        if (pins::Button("1x", ui(ImVec2(60, 36)))) {
            simulation->setRate(1);
            dirty = false;
        }
        ImGui::EndDisabled();
        ImGui::EndDisabled();
        ImGui::Text("Speed range: >0 to %.0fx", s.maxRate);
        ImGui::TextDisabled("Pause preserves the selected speed.");
        ImGui::Separator();
        ImGui::BeginDisabled(!simulation || !s.syncReady || s.operationPending);
        if (pins::Button("Sync sim", ui(ImVec2(130, 36))))
            simulation->sync();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(
                "Move simulation to the estimated robot pose. Keep velocities and the estimate unchanged.");
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(!simulation || !s.resetReady || s.operationPending);
        if (pins::Button("Reset sim", ui(ImVec2(130, 36))))
            simulation->reset();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(
                "Reset simulation to its start pose, at rest, with thrusters cleared; re-seed the estimator.");
        ImGui::EndDisabled();
        if (!s.operationMessage.empty())
            ImGui::TextWrapped("%s", s.operationMessage.c_str());
    }
};
} // namespace
void registerSimulationPanel(Registry &registry) {
    registry.panels.emplace(
        "simulation", ViewFactory<Panel>{Kind::Simulation, [](const YAML::Node &n) { keys(n, {}, "simulation tool"); },
                                         [](const Binding &b) { return std::make_unique<SimulationPanel>(b); }});
}
} // namespace nereus::ros_viewer::panels

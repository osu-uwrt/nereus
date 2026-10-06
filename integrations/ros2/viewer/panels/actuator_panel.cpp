// Actuators panel: one full-width button per mechanism action (claw, droppers, torpedoes, ...) plus status
// readings, all supplied by the bound Actuators provider.
#include "nereus/ros_viewer/panel_layout.hpp"
#include "nereus/ros_viewer/panels/composition.hpp"
#include "nereus/ros_viewer/pins.hpp"
#include <imgui.h>

namespace nereus::ros_viewer::panels {
namespace {

class ActuatorPanel final : public Panel {
    std::shared_ptr<Actuators> actuators;

  public:
    explicit ActuatorPanel(const Binding &b) : actuators(std::dynamic_pointer_cast<Actuators>(b.provider)) {}

    void draw() override {
        auto s = actuators ? actuators->state() : ActuatorState{};
        if (!actuators)
            emptyState("Connected, this holds a button per mechanism action (claw, droppers, torpedoes, magnet).");
        else if (!s.message.empty())
            ImGui::TextWrapped("%s", s.message.c_str());

        // One button per action; "###id" keeps the ImGui ID stable when the label changes.
        for (const auto &action : s.actions) {
            ImGui::PushID(action.id.c_str());
            ImGui::BeginDisabled(!action.available);
            if (pins::Button((action.label + "###" + action.id).c_str(), {-1, ui(36)}))
                actuators->command(action.id);
            ImGui::EndDisabled();
            ImGui::PopID();
        }

        // Provider-reported key/value readings.
        if (!s.readings.empty())
            sectionTitle("Status");
        for (const auto &reading : s.readings)
            ImGui::TextWrapped("%s: %s", reading.first.c_str(), reading.second.c_str());
    }
};

} // namespace

// Registers the "actuators" panel type (takes no YAML keys).
void registerActuatorPanel(Registry &r) {
    r.panels.emplace("actuators",
                     ViewFactory<Panel>{Kind::Actuators, [](const YAML::Node &n) { keys(n, {}, "actuators panel"); },
                                        [](const Binding &b) { return std::make_unique<ActuatorPanel>(b); }});
}

} // namespace nereus::ros_viewer::panels

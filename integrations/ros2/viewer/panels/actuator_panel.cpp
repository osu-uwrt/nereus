// Actuators panel: whether each mechanism's hardware is connected, one full-width button per mechanism action
// (claw, droppers, torpedoes, ...) plus status readings, all supplied by the bound Actuators provider.
#include "nereus/ros_viewer/panel_layout.hpp"
#include "nereus/ros_viewer/panels/composition.hpp"
#include "nereus/ros_viewer/pins.hpp"
#include "status_chip.hpp"
#include <imgui.h>
#include <cmath>
#include <cstdlib>
#include <cerrno>

namespace nereus::ros_viewer::panels {
namespace {

class ActuatorPanel final : public Panel {
    std::shared_ptr<Actuators> actuators;
    char duration[64] = {};

  public:
    explicit ActuatorPanel(const Binding &b) : actuators(std::dynamic_pointer_cast<Actuators>(b.provider)) {}

    void draw() override {
        auto s = actuators ? actuators->state() : ActuatorState{};
        if (!actuators)
            emptyState("Connected, this holds a button per mechanism action (claw, droppers, torpedoes, magnet).");
        else if (!s.message.empty())
            ImGui::TextWrapped("%s", s.message.c_str());

        // One readout per mechanism link (Claw ● Connected), the firmware state in the hover tooltip.
        for (std::size_t i = 0; i < s.links.size(); ++i) {
            const auto &link = s.links[i];
            if (i)
                ImGui::SameLine(0, ui(14));
            ImGui::PushID(int(i));
            const ImVec4 ink = link.level == Level::Ok ? palette().text : levelColor(link.level);
            if (statusReadout(link.label.c_str(), link.value.c_str(), levelColor(link.level), ink))
                ImGui::SetTooltip("%s: %s\n%s", link.label.c_str(), link.value.c_str(), link.detail.c_str());
            ImGui::PopID();
        }

        float seconds = 0;
        bool validDuration = false, durationShown = false;
        // Put the shared duration immediately before the first timed action.
        for (const auto &action : s.actions) {
            if (action.usesDuration && !durationShown) {
                durationShown = true;
                ImGui::SetNextItemWidth(-1);
                ImGui::InputTextWithHint("##claw_duration", "Claw duration (seconds)", duration, sizeof(duration),
                                         ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CallbackCharFilter,
                                         [](ImGuiInputTextCallbackData *data) -> int {
                                             return data->EventChar == '-' || data->EventChar == '+' ||
                                                    data->EventChar == '*' || data->EventChar == '/';
                                         });
                char *end = nullptr;
                errno = 0;
                seconds = std::strtof(duration, &end);
                validDuration = end != duration && *end == '\0' && errno != ERANGE && std::isfinite(seconds) && seconds > 0;
            }
            ImGui::PushID(action.id.c_str());
            ImGui::BeginDisabled(!action.available || (action.usesDuration && !validDuration));
            if (pins::Button((action.label + "###" + action.id).c_str(), {-1, ui(36)}))
                actuators->command(action.id, action.usesDuration ? seconds : 0);
            ImGui::EndDisabled();
            ImGui::PopID();
        }

        if (s.hasCommandFeedback) {
            ImGui::Spacing();
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {ui(10), ui(8)});
            ImGui::PushStyleColor(ImGuiCol_ChildBg, ImGui::GetStyleColorVec4(ImGuiCol_FrameBg));
            ImGui::BeginChild("##command_feedback", {0, 0},
                              ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_AlwaysUseWindowPadding);
            const char *result = !s.commandResultReceived ? "Waiting" : s.commandSucceeded ? "Success" : "Failed";
            const ImVec4 color = !s.commandResultReceived ? palette().muted
                                 : s.commandSucceeded ? palette().robotEnabled : palette().error;
            statusReadout("Latest command", result, color, color);
            ImGui::PushStyleColor(ImGuiCol_ChildBg, ImGui::GetStyleColorVec4(ImGuiCol_WindowBg));
            ImGui::BeginChild("##feedback_text", {0, 0},
                              ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_AlwaysUseWindowPadding);
            if (!s.commandFeedbackReceived) {
                // Animated dots distinguish the pending state from every possible firmware response.
                const ImVec2 pos = ImGui::GetCursorScreenPos();
                const float height = ImGui::GetTextLineHeight(), radius = ui(2.5f);
                ImGui::InvisibleButton("##feedback_pending", {ui(34), height});
                for (int i = 0; i < 3; ++i) {
                    ImVec4 color = palette().muted;
                    color.w *= 0.35f + 0.65f * (0.5f + 0.5f * std::sin(float(ImGui::GetTime() * 4) - i));
                    ImGui::GetWindowDrawList()->AddCircleFilled(
                        {pos.x + ui(5 + i * 10), pos.y + height * .5f}, radius, ImGui::GetColorU32(color));
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Waiting for feedback");
            } else {
                ImGui::TextWrapped("%s", s.commandFeedback.c_str());
            }
            ImGui::EndChild();
            ImGui::PopStyleColor();
            ImGui::EndChild();
            ImGui::PopStyleColor();
            ImGui::PopStyleVar();
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

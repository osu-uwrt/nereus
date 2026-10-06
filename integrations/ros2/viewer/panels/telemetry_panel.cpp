// "telemetry" panel: the robot's telemetry readings as header / toolbar readouts or a sidebar list.
#include "nereus/ros_viewer/panel_layout.hpp"
#include "nereus/ros_viewer/panels/composition.hpp"
#include "nereus/ros_viewer/plots/plots.hpp"
#include "status_chip.hpp"
#include <imgui.h>

namespace nereus::ros_viewer::panels {
namespace {
// Header / toolbar form: one readout per reading ("FOG 41.2°C": dot, label, value), colored by level, details on
// hover.
// Sidebar form: the same readings with their details.
class TelemetryPanel final : public Panel {
    std::shared_ptr<Telemetry> telemetry;

    // the value in the level's color when it needs attention, plain when fine, muted when stale
    static ImVec4 valueInk(Level level) {
        return level == Level::Warn || level == Level::Error ? levelColor(level)
               : level == Level::Ok                          ? palette().text
                                                             : palette().muted;
    }

    // One readout per reading on a single line, with label, level and detail in the hover tooltip.
    void chips(const TelemetryState &s) {
        for (std::size_t i = 0; i < s.readings.size(); ++i) {
            const auto &reading = s.readings[i];
            if (i)
                ImGui::SameLine(0, ui(14));
            ImGui::PushID(int(i));
            // Hover: the reading's last 30 s and its details; right-click: plot it.
            const std::string figure = "telemetry." + reading.id;
            if (statusReadout(reading.label.c_str(), reading.value.c_str(), levelColor(reading.level),
                              valueInk(reading.level)))
                plots::figureTooltip(figure, reading.label + ": " + levelName(reading.level), reading.detail);
            if (ImGui::IsItemClicked(ImGuiMouseButton_Right))
                ImGui::OpenPopup("plot");
            pushPopupColors(Surface::Sheet);
            if (ImGui::BeginPopup("plot")) {
                plots::figureMenuItems(figure);
                ImGui::EndPopup();
            }
            popPopupColors();
            ImGui::PopID();
        }
    }

  public:
    explicit TelemetryPanel(const Binding &b) : telemetry(std::dynamic_pointer_cast<Telemetry>(b.provider)) {}
    void header() override {
        if (telemetry)
            chips(telemetry->state());
    }

    // Same readouts, starting a new line when they don't fit next to the previous toolbar item.
    void toolbar() override {
        if (!telemetry)
            return;
        const auto s = telemetry->state();
        float width = 0;
        for (const auto &reading : s.readings)
            width += readoutWidth(reading.label.c_str(), reading.value.c_str()) + ui(14);
        sameLineIfFits(width);
        chips(s);
    }

    void draw() override {
        if (!telemetry) {
            emptyState("Connected, this lists the robot's telemetry readings.");
            return;
        }
        for (const auto &reading : telemetry->state().readings) {
            ImGui::TextColored(levelColor(reading.level), "%s  %s", reading.label.c_str(), reading.value.c_str());
            ImGui::SameLine();
            ImGui::TextDisabled("(%s)", levelName(reading.level));
            ImGui::Indent();
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]);
            ImGui::TextWrapped("%s", reading.detail.c_str());
            ImGui::PopStyleColor();
            ImGui::Unindent();
        }
    }
};
} // namespace

void registerTelemetryPanel(Registry &r) {
    r.panels.emplace("telemetry",
                     ViewFactory<Panel>{Kind::Telemetry, [](const YAML::Node &n) { keys(n, {}, "telemetry panel"); },
                                        [](const Binding &b) { return std::make_unique<TelemetryPanel>(b); }});
}
} // namespace nereus::ros_viewer::panels

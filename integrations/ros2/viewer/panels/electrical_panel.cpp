// Electrical panel: power rail commands, IMU mag cal and register access, FOG gyro tare, pinger settings and the
// IVC link, each section shown only when the bound Electrical provider configures it.
#include "nereus/ros_viewer/panel_layout.hpp"
#include "nereus/ros_viewer/panels/composition.hpp"
#include "nereus/ros_viewer/pins.hpp"
#include <algorithm>
#include <cstdio>
#include <imgui.h>

namespace nereus::ros_viewer::panels {
namespace {

// Wrapped text in the disabled-text colour; nothing for an empty string.
void muted(const std::string &text) {
    if (text.empty())
        return;
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]);
    ImGui::TextWrapped("%s", text.c_str());
    ImGui::PopStyleColor();
}

// The RViz electrical panel: power commands (power cuts ask first), IMU mag cal and register edit, FOG tare,
// pinger and IVC.
class ElectricalPanel final : public Panel {
    std::shared_ptr<Electrical> electrical;
    char reg[8]{}, data[128]{};                                   // IMU register number and value text being edited
    std::string shownValue;                                       // last register value copied into `data`
    int samples = 100000, header = 0, status = 0, rawCommand = 0; // tare samples; IVC header/status/command
    double tareTimeout = 20;                                      // seconds
    bool pingerEnabled = true, pingerInitialized = false;         // checkbox seeded once from the provider
    std::size_t logLines = 0;                                     // IVC log length last frame, to auto-scroll

    // Power commands two per row; those marked `confirm` (power cuts) open a confirmation popup first.
    void power(const ElectricalState &s) {
        sectionTitle("Power");
        const float half = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) / 2;
        for (std::size_t i = 0; i < s.commands.size(); ++i) {
            const auto &command = s.commands[i];
            if (i % 2)
                ImGui::SameLine();
            ImGui::PushID(command.id.c_str());
            if (command.confirm)
                ImGui::PushStyleColor(ImGuiCol_Button, palette().dangerPressed);
            if (pins::Button(command.label.c_str(), {half, ui(30)})) {
                if (command.confirm)
                    ImGui::OpenPopup("confirm_power");
                else
                    electrical->command(command.id);
            }
            if (command.confirm)
                ImGui::PopStyleColor();
            if (ImGui::BeginPopup("confirm_power")) {
                ImGui::Text("%s?", command.label.c_str());
                ImGui::TextDisabled("This cuts power on the robot.");
                if (ImGui::Button("Send", ui(ImVec2(90, 0)))) {
                    electrical->command(command.id);
                    ImGui::CloseCurrentPopup();
                }
                ImGui::SameLine();
                if (ImGui::Button("Cancel", ui(ImVec2(90, 0))))
                    ImGui::CloseCurrentPopup();
                ImGui::EndPopup();
            }
            ImGui::PopID();
        }
        muted(s.commandMessage);
    }

    // VectorNav mag cal (start/cancel with progress) and register read/write/save.
    void imu(const ElectricalState &s) {
        sectionTitle("IMU (VectorNav)");
        ImGui::BeginDisabled(!s.magCalRunning && !s.magCalReady);
        if (pins::Button(s.magCalRunning ? "Cancel mag cal###mag_cal" : "Mag cal###mag_cal", {-1, ui(30)})) {
            if (s.magCalRunning)
                electrical->cancelMagCal();
            else
                electrical->startMagCal();
        }
        ImGui::EndDisabled();
        if (s.magCalRunning || s.magCalProgress > 0)
            progressBar(s.magCalProgress);
        muted(s.magCalReady || s.magCalRunning ? s.magCalMessage : "Mag cal action unavailable");

        // Register access: Read needs a register, Write also a value; all wait for the service.
        if (s.registerValue != shownValue) { // a read fills the value field, as RViz does
            shownValue = s.registerValue;
            std::snprintf(data, sizeof(data), "%s", shownValue.c_str());
        }
        ImGui::SetNextItemWidth(ui(70));
        ImGui::InputTextWithHint("##reg", "Register", reg, sizeof(reg), ImGuiInputTextFlags_CharsDecimal);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(-1);
        ImGui::InputTextWithHint("##value", "Value(s)", data, sizeof(data));
        const float third = (ImGui::GetContentRegionAvail().x - 2 * ImGui::GetStyle().ItemSpacing.x) / 3;
        ImGui::BeginDisabled(!s.registerReady || s.registerPending);
        ImGui::BeginDisabled(!reg[0]);
        if (pins::Button("Read###register_read", {third, 0}))
            electrical->readRegister(reg);
        ImGui::SameLine();
        ImGui::BeginDisabled(!data[0]);
        if (pins::Button("Write###register_write", {third, 0}))
            electrical->writeRegister(reg, data);
        ImGui::EndDisabled();
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (pins::Button("Save to flash", {third, 0}))
            electrical->saveImuSettings();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Writes the current IMU settings to its flash ($VNWNV) so they survive a power cycle.");
        ImGui::EndDisabled();
        muted(s.registerReady || s.registerPending ? s.registerMessage : "IMU config service unavailable");
    }

    // FOG gyro tare: sample count and timeout (locked while running), then start/cancel.
    void fog(const ElectricalState &s) {
        sectionTitle("FOG");
        ImGui::BeginDisabled(s.tareRunning);
        ImGui::SetNextItemWidth(ui(130));
        if (ImGui::InputInt("Samples", &samples, 1000, 10000))
            samples = std::clamp(samples, 1000, 1000000);
        ImGui::SetNextItemWidth(ui(130));
        if (ImGui::InputDouble("Timeout (s)", &tareTimeout, 1, 5, "%.1f"))
            tareTimeout = std::clamp(tareTimeout, .1, 60.);
        ImGui::EndDisabled();
        ImGui::BeginDisabled(!s.tareRunning && !s.tareReady);
        if (pins::Button(s.tareRunning ? "Cancel tare###tare" : "Tare gyro###tare", {-1, ui(30)})) {
            if (s.tareRunning)
                electrical->cancelTare();
            else
                electrical->startTare(samples, tareTimeout);
        }
        ImGui::EndDisabled();
        muted(s.tareReady || s.tareRunning ? s.tareMessage : "Gyro tare action unavailable");
    }

    // Pinger enable plus one button per frequency (the board's current one highlighted) and its amplitude.
    void pinger(const ElectricalState &s) {
        sectionTitle("Pinger");
        if (!pingerInitialized) {
            pingerEnabled = s.pingerEnabled;
            pingerInitialized = true;
        }
        if (pins::Checkbox("Enable pinger", &pingerEnabled))
            electrical->setPingerEnabled(pingerEnabled);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Re-sent every second, so a rebooted board picks it up (as RViz).");

        bool first = true;
        for (const int khz : s.pingerFrequencies) {
            const std::string label = std::to_string(khz) + " kHz";
            const float width = ImGui::CalcTextSize(label.c_str()).x + 2 * ImGui::GetStyle().FramePadding.x;
            if (!first)
                sameLineIfFits(width);
            first = false;
            const bool selected = s.pingerSelected && *s.pingerSelected == khz;
            if (selected)
                ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyle().Colors[ImGuiCol_ButtonActive]);
            ImGui::PushID(khz);
            if (pins::Button(label.c_str()))
                electrical->setPingerFrequency(khz);
            ImGui::PopID();
            if (selected)
                ImGui::PopStyleColor();
        }

        char amplitude[32] = "--";
        if (s.pingerAmplitude)
            std::snprintf(amplitude, sizeof(amplitude), "%.3g", *s.pingerAmplitude);
        muted("Selected: " + (s.pingerSelected ? std::to_string(*s.pingerSelected) + " kHz" : std::string("--")) +
              "   Amplitude: " + amplitude);
    }

    // IVC: pick a header, then a status (for status headers) or a raw 0..31 command, send, and the traffic log.
    void ivc(const ElectricalState &s) {
        sectionTitle("IVC");
        header = std::clamp(header, 0, std::max(0, int(s.ivcHeaders.size()) - 1));
        ImGui::SetNextItemWidth(-1);
        if (ImGui::BeginCombo("##header", s.ivcHeaders.empty() ? "" : s.ivcHeaders[std::size_t(header)].c_str())) {
            for (int i = 0; i < int(s.ivcHeaders.size()); ++i)
                if (ImGui::Selectable(s.ivcHeaders[std::size_t(i)].c_str(), i == header))
                    header = i;
            ImGui::EndCombo();
        }

        const bool statusMessage = header < s.ivcStatusHeaders;
        const float send = 70;
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - send - ImGui::GetStyle().ItemSpacing.x);
        if (statusMessage) {
            status = std::clamp(status, 0, std::max(0, int(s.ivcStatuses.size()) - 1));
            if (ImGui::BeginCombo("##status",
                                  s.ivcStatuses.empty() ? "" : s.ivcStatuses[std::size_t(status)].c_str())) {
                for (int i = 0; i < int(s.ivcStatuses.size()); ++i)
                    if (ImGui::Selectable(s.ivcStatuses[std::size_t(i)].c_str(), i == status))
                        status = i;
                ImGui::EndCombo();
            }
        } else if (ImGui::InputInt("##command", &rawCommand))
            rawCommand = std::clamp(rawCommand, 0, 31);
        ImGui::SameLine();
        ImGui::BeginDisabled(s.ivcHeaders.empty() || (statusMessage && s.ivcStatuses.empty()));
        if (pins::Button("Send###ivc_send", {send, 0}))
            electrical->sendIvc(header, statusMessage ? status : rawCommand);
        ImGui::EndDisabled();

        ImGui::BeginChild("ivc_log", {-1, 130}, ImGuiChildFlags_Borders);
        if (s.ivcLog.empty())
            ImGui::TextDisabled("No IVC traffic yet");
        for (const auto &line : s.ivcLog)
            ImGui::TextWrapped("%s", line.c_str());
        if (s.ivcLog.size() != logLines) { // follow new traffic
            logLines = s.ivcLog.size();
            ImGui::SetScrollHereY(1);
        }
        ImGui::EndChild();
    }

  public:
    explicit ElectricalPanel(const Binding &b) : electrical(std::dynamic_pointer_cast<Electrical>(b.provider)) {}

    void draw() override {
        if (!electrical) {
            emptyState("Connected, this shows the power rails and their switches, the IMU, FOG and pinger, and the "
                       "IVC link.");
            return;
        }

        // Only the sections the provider has configured.
        const auto s = electrical->state();
        if (!s.commands.empty())
            power(s);
        if (s.hasImu)
            imu(s);
        if (s.hasTare)
            fog(s);
        if (s.hasPinger)
            pinger(s);
        if (s.hasIvc)
            ivc(s);
    }
};

} // namespace

// Registers the "electrical" panel type (takes no YAML keys).
void registerElectricalPanel(Registry &r) {
    r.panels.emplace("electrical",
                     ViewFactory<Panel>{Kind::Electrical, [](const YAML::Node &n) { keys(n, {}, "electrical panel"); },
                                        [](const Binding &b) { return std::make_unique<ElectricalPanel>(b); }});
}

} // namespace nereus::ros_viewer::panels

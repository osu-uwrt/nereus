#include "nereus/ros_viewer/panel_layout.hpp"
#include "nereus/ros_viewer/panels/composition.hpp"
#include "nereus/ros_viewer/panels/pose_math.hpp"
#include "nereus/ros_viewer/pins.hpp"
#include <cmath>
#include <cstdio>
#include <imgui.h>
#include <imgui_internal.h>

namespace nereus::ros_viewer::panels {
namespace {
// Numeric displays align to the right edge of their column. Editable values use
// normal text editing while active and the same right alignment at rest.
void numericText(const char *text, bool available = true) {
    ImGui::AlignTextToFramePadding();
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() +
                         std::max(0.f, ImGui::GetContentRegionAvail().x - ImGui::CalcTextSize(text).x));
    if (available)
        ImGui::TextUnformatted(text);
    else
        ImGui::TextDisabled("%s", text);
}
void numericValue(float value, bool available) {
    char text[64];
    std::snprintf(text, sizeof(text), "%.2f", value);
    numericText(available ? text : "--", available);
}
bool targetInput(float *value) {
    const bool editing = ImGui::GetActiveID() == ImGui::GetID("##target");
    const auto color = ImGui::GetColorU32(ImGuiCol_Text);
    if (!editing)
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0, 0, 0, 0));
    const bool changed = ImGui::InputFloat("##target", value, 0, 0, "%.2f");
    if (!editing) {
        ImGui::PopStyleColor();
        char text[64];
        std::snprintf(text, sizeof(text), "%.2f", *value);
        const auto lo = ImGui::GetItemRectMin(), hi = ImGui::GetItemRectMax();
        const auto padding = ImGui::GetStyle().FramePadding;
        auto *draw = ImGui::GetWindowDrawList();
        draw->PushClipRect(lo, hi, true);
        draw->AddText({std::max(lo.x + padding.x, hi.x - padding.x - ImGui::CalcTextSize(text).x), lo.y + padding.y},
                      color, text);
        draw->PopClipRect();
    }
    return changed;
}
class MotionPanel final : public Panel {
    std::shared_ptr<Motion> motion;
    std::function<void()> kill, drawOverlayControls;
    glm::vec3 position{0}, degrees{0};
    bool initialized = false, dirty = false, hasDive;
    Mode selected = Mode::Position;
    float diveZ;
    uint64_t revision = 0;
    void copy(const Pose &p) {
        position = glm::vec3(p[3]);
        degrees = glm::degrees(glm::eulerAngles(glm::quat_cast(p)));
        initialized = true;
    }

  public:
    explicit MotionPanel(const Binding &b)
        : motion(std::dynamic_pointer_cast<Motion>(b.provider)), kill(b.kill),
          drawOverlayControls(b.drawOverlayControls), hasDive(bool(b.options["dive_z"])),
          diveZ(b.options["dive_z"].as<float>(0)) {}
    void enableKillButton(ImVec2 size) {
        const auto s = motion ? motion->state() : MotionState{};
        // Always switchable once connected; a robot seen enabled, or another operator on the switch, offers KILL.
        const bool canKill = s.enabled || s.pending || s.competing || (s.observedKilled && !*s.observedKilled);
        ImGui::BeginDisabled(!motion);
        const auto &p = palette();
        ImGui::PushStyleColor(ImGuiCol_Button, canKill ? p.danger : p.active);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, canKill ? p.dangerHovered : p.activeHovered);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, canKill ? p.dangerPressed : p.activePressed);
        ImGui::PushStyleColor(ImGuiCol_Text, canKill ? p.dangerText : p.activeText);
        if (pins::Button(canKill ? "KILL###enable_kill" : "Enable###enable_kill", size)) {
            if (canKill)
                kill();
            else
                motion->enable();
        }
        ImGui::PopStyleColor(4);
        ImGui::EndDisabled();
    }
    // "Robot: enabled" etc., coloured by the robot's reported kill state.
    const char *stateText(const MotionState &s, ImVec4 &tint) const {
        const auto &p = palette();
        tint = !motion ? p.muted : s.observedKilled.value_or(true) ? p.robotKilled : p.robotEnabled;
        return !motion            ? "Preview: not connected to a robot"
               : s.observedKilled ? (*s.observedKilled ? "Robot: killed" : "Robot: enabled")
                                  : "Robot: state unknown";
    }
    void toolbar() override {
        nereus::ros_viewer::sameLineIfFits(ui(90));
        enableKillButton({90, ImGui::GetFrameHeight()});
    }
    // Command bar: always visible whatever the window layout, so KILL is never behind a closed or tabbed window.
    void pinned() override {
        const auto s = motion ? motion->state() : MotionState{};
        const float height = std::max(ui(34), ImGui::GetFrameHeight());
        enableKillButton({ui(132), height});
        // The robot's state beside it, centred on the button (drawn directly: a Text after a tall button takes
        // the line's text baseline instead).
        const float top = ImGui::GetItemRectMin().y; // the button's, not its line's (it may continue a line)
        ImGui::SameLine();
        const ImVec2 at(ImGui::GetCursorScreenPos().x, top);
        ImVec4 tint;
        const char *text = stateText(s, tint);
        ImGui::Dummy({ImGui::CalcTextSize(text).x, height});
        ImGui::GetWindowDrawList()->AddText({at.x, at.y + (height - ImGui::GetFontSize()) * .5f},
                                            ImGui::GetColorU32(tint), text);
    }
    void draw() override {
        const auto s = motion ? motion->state() : MotionState{};
        // Enable / KILL here too (also pinned in the command bar), full width with the robot's state under it.
        enableKillButton({ImGui::GetContentRegionAvail().x, std::max(ui(36), ImGui::GetFrameHeight())});
        ImVec4 tint;
        const char *state = stateText(s, tint);
        ImGui::TextColored(tint, "%s", state);
        ImGui::Separator();
        if (s.fresh && (!initialized || (!dirty && !s.hasCommand)))
            copy(s.actual);
        if (s.revision != revision) {
            if (!dirty && s.hasCommand)
                copy(s.commanded);
            revision = s.revision;
        }
        if (!s.pending)
            selected = s.mode == Mode::Feedforward ? Mode::Feedforward : Mode::Position;
        if (s.frame.empty())
            ImGui::TextDisabled("Metres and degrees");
        else
            ImGui::TextDisabled("Frame: %s  /  metres, degrees", s.frame.c_str());
        ImGui::TextWrapped("%s", s.message.c_str());
        const bool unavailable = !motion || !s.enabled || !s.fresh || s.pending || s.blocked || s.competing;
        ImGui::BeginDisabled(unavailable);
        const float modeWidth = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * .5f;
        for (const auto choice : {Mode::Position, Mode::Feedforward}) {
            if (choice == Mode::Feedforward)
                ImGui::SameLine();
            ImGui::BeginDisabled(choice == Mode::Feedforward && !s.supportsFeedforward);
            nereus::ros_viewer::pushActiveColors(s.mode == choice);
            if (pins::Button(choice == Mode::Position ? "Position" : "Feedforward", {modeWidth, 0})) {
                selected = choice;
                motion->activate(choice, s.actual);
                copy(s.actual);
                dirty = false;
            }
            nereus::ros_viewer::popActiveColors();
            ImGui::EndDisabled();
        }
        if (s.mode == Mode::Feedforward)
            ImGui::TextWrapped("Feedforward controller mode; pose feedback is disabled.");
        ImGui::EndDisabled();
        ImGui::PushStyleVar(ImGuiStyleVar_CellPadding, ui(ImVec2(2, 2)));
        // The table scrolls on its own when the panel is short, so the actions below it stay in view.
        const float rowHeight = ImGui::GetFrameHeight() + ui(4);
        const float buttonRow = std::max(ui(40), ImGui::GetFrameHeight()) + ImGui::GetStyle().ItemSpacing.y;
        const bool oneRow = !hasDive || ImGui::CalcTextSize("CurrentCommandDive in place").x + 6 * 4 +
                                                2 * ImGui::GetStyle().ItemSpacing.x <=
                                            ImGui::GetContentRegionAvail().x;
        const float actions = buttonRow * (oneRow ? 1 : 2) + ImGui::GetTextLineHeightWithSpacing() +
                              2 * ImGui::GetStyle().ItemSpacing.y;
        const float tableHeight = std::min(7 * rowHeight + ui(4),
                                           std::max(3 * rowHeight, ImGui::GetContentRegionAvail().y - actions));
        if (ImGui::BeginTable("pose", 5, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY,
                              {0, tableHeight})) {
            ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, ui(36));
            ImGui::TableSetupColumn("Actual");
            ImGui::TableSetupColumn("Commanded");
            ImGui::TableSetupColumn("Error");
            ImGui::TableSetupColumn("Target", ImGuiTableColumnFlags_WidthFixed, ui(66));
            ImGui::TableNextRow();
            ImGui::TableSetupScrollFreeze(0, 1);
            // a heading too wide for its column (a narrow panel, a large interface scale) shortens; the tooltip names it
            const std::pair<const char *, const char *> headings[] = {
                {"", ""}, {"Actual", "Act"}, {"Commanded", "Cmd"}, {"Error", "Err"}, {"Target", "Target"}};
            for (const auto &[full, brief] : headings) {
                ImGui::TableNextColumn();
                const bool fits = ImGui::CalcTextSize(full).x + ui(8) <= ImGui::GetContentRegionAvail().x;
                const char *shown = fits ? full : brief; // right-aligned over the right-aligned figures
                ImGui::SetCursorPosX(ImGui::GetCursorPosX() +
                                     std::max(0.f, ImGui::GetContentRegionAvail().x - ImGui::CalcTextSize(shown).x));
                ImGui::TextDisabled("%s", shown);
                if (ImGui::IsItemHovered() && std::string(full) == "Error")
                    ImGui::SetTooltip("Commanded minus actual / angles wrapped to [-180, 180] degrees");
                else if (ImGui::IsItemHovered() && !fits)
                    ImGui::SetTooltip("%s", full);
            }
            const auto actualAngles = glm::degrees(glm::eulerAngles(glm::quat_cast(s.actual)));
            const auto sentAngles = glm::degrees(glm::eulerAngles(glm::quat_cast(s.commanded)));
            const char *names[] = {"X", "Y", "Z", "Roll", "Pitch", "Yaw"};
            for (int i = 0; i < 6; ++i) {
                ImGui::PushID(i);
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                ImGui::TextUnformatted(names[i]);
                ImGui::TableNextColumn();
                numericValue(i < 3 ? s.actual[3][i] : actualAngles[i - 3], s.fresh);
                ImGui::TableNextColumn();
                numericValue(i < 3 ? s.commanded[3][i] : sentAngles[i - 3], s.hasCommand);
                ImGui::TableNextColumn();
                const float error = i < 3 ? s.commanded[3][i] - s.actual[3][i]
                                          : std::remainder(sentAngles[i - 3] - actualAngles[i - 3], 360.f);
                numericValue(error, s.fresh && s.hasCommand);
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(-1);
                ImGui::BeginDisabled(!motion || !s.fresh || s.pending || s.blocked);
                if (targetInput(i < 3 ? &position[i] : &degrees[i - 3]))
                    dirty = true;
                ImGui::EndDisabled();
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
        ImGui::PopStyleVar();
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4, ImGui::GetStyle().FramePadding.y));
        const float actionsWidth = ImGui::CalcTextSize("Current").x + ImGui::CalcTextSize("Command").x +
                                   4 * ImGui::GetStyle().FramePadding.x + ImGui::GetStyle().ItemSpacing.x +
                                   (hasDive ? ImGui::CalcTextSize("Dive in place").x +
                                                  2 * ImGui::GetStyle().FramePadding.x + ImGui::GetStyle().ItemSpacing.x
                                            : 0);
        const float extraWidth = std::max(0.f, ImGui::GetContentRegionAvail().x - actionsWidth) / (hasDive ? 3 : 2);
        const auto actionSize = [&](const char *label) {
            return ImVec2(ImGui::CalcTextSize(label).x + 2 * ImGui::GetStyle().FramePadding.x + extraWidth,
                          std::max(ui(40), ImGui::GetFrameHeight()));
        };
        ImGui::BeginDisabled(!motion || !s.fresh || s.pending || s.blocked);
        if (pins::Button("Current", actionSize("Current"))) {
            copy(s.actual);
            dirty = true;
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(unavailable);
        const bool finite = std::isfinite(position.x + position.y + position.z + degrees.x + degrees.y + degrees.z);
        ImGui::BeginDisabled(!finite);
        if (pins::Button("Command", actionSize("Command"))) {
            motion->activate(selected, nereus::ros_viewer::rpyPose(position, glm::radians(degrees)));
            dirty = false;
        }
        ImGui::EndDisabled();
        if (hasDive) // its own line when the three do not fit
            nereus::ros_viewer::sameLineIfFits(actionSize("Dive in place").x);
        if (hasDive && pins::Button("Dive in place", actionSize("Dive in place"))) {
            copy(s.actual);
            position.z = diveZ;
            degrees.x = degrees.y = 0;
            selected = Mode::Position;
            motion->activate(selected, nereus::ros_viewer::rpyPose(position, glm::radians(degrees)));
            dirty = false;
        }
        ImGui::EndDisabled();
        ImGui::PopStyleVar();
        ImGui::TextDisabled("%s", dirty ? "Target edited / not sent" : "Drag an axis or ring in the pool view");
        if (drawOverlayControls)
            drawOverlayControls();
    }
};
} // namespace
void registerMotionPanel(Registry &r) {
    r.panels.emplace("motion", ViewFactory<Panel>{Kind::Motion,
                                                  [](const YAML::Node &n) {
                                                      keys(n, {"dive_z", "dive_max_depth_z"}, "motion panel");
                                                      if (n["dive_max_depth_z"])
                                                          required(n, {"dive_z"});
                                                      for (const char *key : {"dive_z", "dive_max_depth_z"})
                                                          if (n[key] && !std::isfinite(n[key].as<float>()))
                                                              throw std::invalid_argument("nonfinite dive setting");
                                                  },
                                                  [](const Binding &b) { return std::make_unique<MotionPanel>(b); }});
}
} // namespace nereus::ros_viewer::panels

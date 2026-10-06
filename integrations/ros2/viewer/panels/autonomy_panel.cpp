// Autonomy panel: pick a behaviour tree from the robot's list, start/stop it, and show the running tree's
// execution stack as reported by the bound Autonomy provider.
#include "nereus/ros_viewer/panel_layout.hpp"
#include "nereus/ros_viewer/panels/composition.hpp"
#include "nereus/ros_viewer/pins.hpp"
#include "nereus/ros_viewer/theme.hpp"
#include <cfloat>
#include <filesystem>
#include <imgui.h>

namespace nereus::ros_viewer::panels {
namespace {

class AutonomyPanel final : public Panel {
    std::shared_ptr<Autonomy> mission;
    std::function<bool()> mayStart; // Composition::mayStart: the owned motion is enabled, fresh, uncontested
    std::string selected;           // full path of the chosen tree; the UI shows only its filename
    ImGuiTextFilter filter;         // search box inside the tree combo

  public:
    explicit AutonomyPanel(const Binding &b)
        : mission(std::dynamic_pointer_cast<Autonomy>(b.provider)), mayStart(b.mayStart) {}

    void draw() override {
        const auto s = mission ? mission->state() : MissionState{};
        if (!mission)
            emptyState("Connected, this lists the robot's behaviour trees to start and shows the running tree's "
                       "execution stack.");
        else if (!s.message.empty())
            ImGui::TextWrapped("%s", s.message.c_str());

        // Tree picker, refresh and start are locked while disconnected or while a tree runs/starts.
        ImGui::BeginDisabled(!mission || !s.connected || s.busy || s.pending);

        // Size the combo popup to the longest tree filename, clamped to the display width.
        float labelWidth = ImGui::CalcTextSize("Select a tree").x;
        for (const auto &tree : s.trees)
            labelWidth = std::max(labelWidth, ImGui::CalcTextSize(std::filesystem::path(tree).filename().c_str()).x);
        const float popupWidth =
            std::min(ImGui::GetIO().DisplaySize.x - 36,
                     labelWidth + std::max(2 * ImGui::GetStyle().WindowPadding.x,
                                           2 * ImGui::GetStyle().FramePadding.x + ImGui::GetFrameHeight()));
        ImGui::SetNextItemWidth(std::min(ImGui::GetContentRegionAvail().x, popupWidth));
        ImGui::SetNextWindowSizeConstraints({popupWidth, 0}, {popupWidth, FLT_MAX});

        // Searchable tree list; hovering an entry shows its full path.
        if (ImGui::BeginCombo("##tree", selected.empty() ? "Select a tree"
                                                         : std::filesystem::path(selected).filename().c_str())) {
            ImGui::SetNextItemWidth(-1);
            if (ImGui::IsWindowAppearing())
                ImGui::SetKeyboardFocusHere();
            if (ImGui::InputTextWithHint("##tree_search", "Search trees", filter.InputBuf,
                                         IM_ARRAYSIZE(filter.InputBuf), ImGuiInputTextFlags_AutoSelectAll))
                filter.Build();
            for (const auto &tree : s.trees)
                if (filter.PassFilter(tree.c_str())) {
                    ImGui::PushID(tree.c_str());
                    if (ImGui::Selectable(std::filesystem::path(tree).filename().c_str(), selected == tree))
                        selected = tree;
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("%s", tree.c_str());
                    ImGui::PopID();
                }
            ImGui::EndCombo();
        }

        if (pins::Button(s.refreshing ? "Refreshing...###refresh" : "Refresh###refresh"))
            mission->refresh();
        ImGui::SameLine();
        ImGui::BeginDisabled(selected.empty() || !mayStart());
        if (pins::Button("Start"))
            mission->start(selected);
        ImGui::EndDisabled();
        ImGui::EndDisabled();

        // Stop stays enabled whenever a tree is running.
        ImGui::SameLine();
        ImGui::BeginDisabled(!mission || !s.busy);
        if (pins::Button("Stop"))
            mission->stop();
        ImGui::EndDisabled();

        // Execution stack: numbered entries, the innermost (last) one highlighted in the accent colour.
        if (!s.activeTree.empty())
            ImGui::TextWrapped("Tree: %s", std::filesystem::path(s.activeTree).filename().c_str());
        sectionTitle(s.stackStale ? "Execution stack (stale)" : s.busy ? "Execution stack" : "Last execution stack");
        if (s.stack.empty()) {
            ImGui::TextDisabled(s.busy ? "Waiting for the stack..." : "None yet: start a tree to see its stack.");
            return;
        }
        ImGui::BeginChild("stack",
                          {0, std::max(ui(80), ImGui::GetContentRegionAvail().y - ImGui::GetStyle().ItemSpacing.y)},
                          ImGuiChildFlags_None);
        for (size_t i = 0; i < s.stack.size(); ++i) {
            ImGui::TextColored(i + 1 == s.stack.size() ? palette().accent : palette().muted, "%02zu", i + 1);
            ImGui::SameLine();
            ImGui::TextWrapped("%s", s.stack[i].c_str());
        }
        ImGui::EndChild();
    }
};

} // namespace

// Registers the "autonomy" panel type (takes no YAML keys).
void registerAutonomyPanel(Registry &r) {
    r.panels.emplace("autonomy",
                     ViewFactory<Panel>{Kind::Autonomy, [](const YAML::Node &n) { keys(n, {}, "autonomy panel"); },
                                        [](const Binding &b) { return std::make_unique<AutonomyPanel>(b); }});
}

} // namespace nereus::ros_viewer::panels

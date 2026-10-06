#include "nereus/ros_viewer/panels/composition.hpp"
#include "nereus/ros_viewer/theme.hpp"
#include <algorithm>
namespace nereus::ros_viewer::panels {
void registerMotionPanel(Registry &);
void registerAutonomyPanel(Registry &);
void registerPoseGizmo(Registry &);
void registerMappingPanel(Registry &);
void registerActuatorPanel(Registry &);
void registerRunPanel(Registry &);
void registerSimulationPanel(Registry &);
void registerTelemetryPanel(Registry &);
void registerRecordingPanel(Registry &);
void registerElectricalPanel(Registry &);
void registerBaggingPanel(Registry &);
namespace {
// Toolbar item "panels_menu": a Windows popup that shows or hides windows (drawn by the composition).
struct PanelsMenu final : Panel {
    std::function<void()> menu;
    explicit PanelsMenu(const Binding &b) : menu(b.drawPanelMenu) {}
    void toolbar() override {
        if (menu)
            menu();
    }
    void draw() override {}
};
// Toolbar item "separator": a thin divider between toolbar groups (none at the end of a row that wraps).
struct Separator final : Panel {
    void toolbar() override {
        const float gap = ui(8);
        const float right = ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x;
        if (ImGui::GetItemRectMax().x + 2 * gap + ui(60) > right)
            return; // the next item wraps; a divider at a row's end means nothing
        ImGui::SameLine(0, gap);
        const ImVec2 at = ImGui::GetCursorScreenPos();
        const float h = ImGui::GetFrameHeight();
        ImGui::GetWindowDrawList()->AddLine({at.x, at.y + h * .18f}, {at.x, at.y + h * .82f},
                                            ImGui::GetColorU32(ImGuiCol_Separator), std::max(1.f, ui(1)));
        ImGui::Dummy({std::max(1.f, ui(1)), h});
    }
    void draw() override {}
};
} // namespace
void registerPanels(Registry &registry) {
    registry.panels.emplace("panels_menu",
                            ViewFactory<Panel>{Kind::Motion, [](const YAML::Node &n) { keys(n, {}, "panels_menu"); },
                                               [](const Binding &b) { return std::make_unique<PanelsMenu>(b); }, true,
                                               true});
    registry.panels.emplace("separator",
                            ViewFactory<Panel>{Kind::Motion, [](const YAML::Node &n) { keys(n, {}, "separator"); },
                                               [](const Binding &) { return std::make_unique<Separator>(); }, true,
                                               true});
    registerSimulationPanel(registry);
    registerRunPanel(registry);
    registerActuatorPanel(registry);
    registerMappingPanel(registry);
    registerMotionPanel(registry);
    registerAutonomyPanel(registry);
    registerPoseGizmo(registry);
    registerTelemetryPanel(registry);
    registerRecordingPanel(registry);
    registerElectricalPanel(registry);
    registerBaggingPanel(registry);
}
} // namespace nereus::ros_viewer::panels

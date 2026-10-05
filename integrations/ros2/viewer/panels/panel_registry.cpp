#include "nereus/ros_viewer/panels/composition.hpp"
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
} // namespace
void registerPanels(Registry &registry) {
    registry.panels.emplace("panels_menu",
                            ViewFactory<Panel>{Kind::Motion, [](const YAML::Node &n) { keys(n, {}, "panels_menu"); },
                                               [](const Binding &b) { return std::make_unique<PanelsMenu>(b); }, true,
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
}
} // namespace nereus::ros_viewer::panels

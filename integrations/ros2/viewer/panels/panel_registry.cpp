// Ported from riptide_simulator camera_faker pool_viewer panel_registry.cpp; see docs/PROVENANCE.md.
#include "robotics/ros_viewer/panels/composition.hpp"
namespace robotics::ros_viewer::panels {
void registerMotionPanel(Registry &);
void registerAutonomyPanel(Registry &);
void registerPoseGizmo(Registry &);
void registerMappingPanel(Registry &);
void registerActuatorPanel(Registry &);
void registerRunPanel(Registry &);
void registerSimulationPanel(Registry &);
namespace {
// Toolbar item "panels_menu": the popup that shows or hides sidebar panels (drawn by the composition).
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
}
} // namespace robotics::ros_viewer::panels

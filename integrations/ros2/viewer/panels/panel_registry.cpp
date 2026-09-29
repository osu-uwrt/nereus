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
void registerPanels(Registry &registry) {
    registerSimulationPanel(registry);
    registerRunPanel(registry);
    registerActuatorPanel(registry);
    registerMappingPanel(registry);
    registerMotionPanel(registry);
    registerAutonomyPanel(registry);
    registerPoseGizmo(registry);
}
} // namespace robotics::ros_viewer::panels

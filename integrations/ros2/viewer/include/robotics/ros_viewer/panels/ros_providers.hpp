// Ported from riptide_simulator camera_faker pool_viewer (panels/ros_providers.hpp); see docs/PROVENANCE.md.
#pragma once
#include "robotics/ros_viewer/panels/composition.hpp"

namespace robotics::ros_viewer::panels {
// Application boundary: the composition, panels and capability contracts have
// no dependency on the ROS implementation hidden behind this owner.
class RosProviders {
  public:
    RosProviders();
    ~RosProviders();
    void registerFactories(Registry &);
    void start();
    void stop();

  private:
    struct Impl;
    std::shared_ptr<Impl> impl;
};
} // namespace robotics::ros_viewer::panels

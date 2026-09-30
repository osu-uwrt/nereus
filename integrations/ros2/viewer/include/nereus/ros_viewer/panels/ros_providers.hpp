#pragma once
#include "nereus/ros_viewer/panels/composition.hpp"

namespace nereus::ros_viewer::panels {
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
} // namespace nereus::ros_viewer::panels

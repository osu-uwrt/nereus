// Owner of the viewer panels' shared ROS node: registers the ROS-backed capability providers (standard, and
// UWRT when built with it) and runs the node's executor on a background thread.
#pragma once
#include "nereus/ros_viewer/panels/composition.hpp"

namespace nereus::ros_viewer::panels {
// Application boundary: the composition, panels and capability contracts have
// no dependency on the ROS implementation hidden behind this owner.
class RosProviders {
  public:
    RosProviders();
    ~RosProviders();

    // Adds the ROS provider factories; the node itself is created by the first provider that needs it.
    void registerFactories(Registry &);
    // Spin / stop the executor thread (no-ops before the node exists).
    void start();
    void stop();

  private:
    struct Impl;
    std::shared_ptr<Impl> impl;
};
} // namespace nereus::ros_viewer::panels

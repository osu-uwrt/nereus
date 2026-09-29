#pragma once
// rclcpp transport around BridgeCore: QoS, generic publishers/subscriptions, services, TF and
// pacing (port of node.py). One thread (the caller of run()) owns the core and the session;
// an executor thread runs every ROS callback, which only enqueues work that the stepping thread
// applies between ticks, so the stepping loop never blocks on the middleware. Wall time only
// paces stepping; every stamp comes from simulation time.
#include "core.hpp"

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <deque>
#include <mutex>
#include <thread>

namespace robotics::ros_bridge {

class BridgeNode {
  public:
    // Creates every ROS entity of the bridge pack (the ROS context must be initialised).
    BridgeNode(BridgeCore &core, CameraSink *cameras = nullptr);
    ~BridgeNode();

    void startExecutor();
    void stopExecutor();

    // Steps until the duration (simulated ns) or shutdown; returns ticks run.
    std::int64_t run(std::optional<std::int64_t> duration_ns, int max_catchup_ticks = 20);
    void requestStop() { stop_ = true; }

    rclcpp::Node &node() { return *node_; }

  private:
    void send(const std::vector<Publication> &publications);
    void publishClock(std::int64_t ns);
    void broadcast(const std::vector<Transform> &transforms);
    void tick();
    struct Impl;
    std::unique_ptr<Impl> impl_;
    BridgeCore &core_;
    std::shared_ptr<rclcpp::Node> node_;
    std::atomic<bool> stop_{false};
};

} // namespace robotics::ros_bridge

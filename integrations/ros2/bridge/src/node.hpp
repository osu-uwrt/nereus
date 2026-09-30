#pragma once
// rclcpp transport around BridgeCore: QoS, generic publishers/subscriptions, services, TF and
// pacing. One thread (the caller of run()) owns the core and the session;
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
    void requestStop() {
        stop_ = true;
    }

    rclcpp::Node &node() {
        return *node_;
    }

    // Stepping-loop cost: tick() wall time (step, publish, camera request) and achieved speed.
    struct Phase { // wall cost of one phase of the stepping loop
        std::int64_t total_ns{0}, max_ns{0}, over_1ms{0}, over_5ms{0};
        void add(std::int64_t ns) {
            total_ns += ns;
            max_ns = std::max(max_ns, ns);
            over_1ms += ns > 1'000'000;
            over_5ms += ns > 5'000'000;
        }
    };
    struct Performance {
        std::int64_t ticks{0}, tick_ns_total{0}, tick_ns_max{0}, wall_ns{0}, sim_ns{0};
        Phase drain, step, publish, cameras, oversleep;   // oversleep: wake-up later than requested
        std::int64_t max_behind_ns{0}, catchup_bursts{0}; // simulated time owed; iterations with >=5 ticks
        double meanTickUs() const {
            return ticks ? tick_ns_total / 1e3 / static_cast<double>(ticks) : 0.0;
        }
        double realTimeFactor() const {
            return wall_ns ? static_cast<double>(sim_ns) / static_cast<double>(wall_ns) : 0.0;
        }
    };
    const Performance &performance() const {
        return performance_;
    }
    // Publications skipped because the topic had no subscribers, per stream (plus "/clock", "/tf").
    std::map<std::string, std::int64_t> skippedPublications() const;

  private:
    void send(const std::vector<Publication> &publications);
    void publishClock(std::int64_t ns);
    void broadcast(const std::vector<Transform> &transforms);
    void tick();
    void refreshSubscribers();
    struct Impl;
    std::unique_ptr<Impl> impl_;
    BridgeCore &core_;
    std::shared_ptr<rclcpp::Node> node_;
    std::atomic<bool> stop_{false};
    Performance performance_;
};

} // namespace robotics::ros_bridge

// Shared ROS plumbing for the viewer's ROS-backed panel providers: one node + TF buffer spun on a worker
// thread, the per-adapter register functions, and RosMotion, the common base for manual-motion adapters.
#pragma once
#include "nereus/ros_viewer/panels/composition.hpp"
#include <glm/gtc/quaternion.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <thread>

namespace nereus::ros_viewer::panels {
using Steady = std::chrono::steady_clock;

// One "viewer_panels" node in the robot namespace, shared by every ROS provider. start() spins the
// executor on `worker`; stop() cancels and joins it, so callbacks run off the UI thread.
struct RosRuntime {
    explicit RosRuntime(const Context &ctx);
    ~RosRuntime();
    void start();
    void stop();

    std::shared_ptr<rclcpp::Node> node;
    tf2_ros::Buffer tf;
    tf2_ros::TransformListener listener;
    rclcpp::executors::SingleThreadedExecutor executor;
    std::thread worker;
};

// Returns the shared runtime (created lazily on first use) for a provider.
using RuntimeFactory = std::function<std::shared_ptr<RosRuntime>(const Context &)>;

// Provider registration, one per adapter; the UWRT ones are only compiled with NEREUS_VIEWER_UWRT.
void registerUwrtMotion(Registry &, const RuntimeFactory &);
void registerUwrtAutonomy(Registry &, const RuntimeFactory &);
void registerUwrtMapping(Registry &, const RuntimeFactory &);
void registerUwrtActuators(Registry &, const RuntimeFactory &);
void registerUwrtTelemetry(Registry &, const RuntimeFactory &);
void registerUwrtRecording(Registry &, const RuntimeFactory &);
void registerUwrtElectrical(Registry &, const RuntimeFactory &);
void registerSimulationRate(Registry &, const RuntimeFactory &);
void registerSimRun(Registry &, const RuntimeFactory &);
void registerStandardMotion(Registry &, const RuntimeFactory &);
void registerRosBagging(Registry &, const RuntimeFactory &);

// Converts a ROS transform to a 4x4 pose matrix (rotation normalized).
Pose poseMatrix(const geometry_msgs::msg::Transform &);
// True when every matrix element is finite.
bool finitePose(const Pose &);

// Shared ROS mechanics; protocol-specific types and endpoints stay in adapters.
class RosMotion : public Motion {
  public:
    RosMotion(std::shared_ptr<RosRuntime>, const YAML::Node &, const Context &);
    MotionState state() override;
    void touch() override;
    void enable() override;
    void kill() override;
    void activate(Mode, const Pose &) override;
    void drag(const Pose &) override;
    void block(bool) override;

  protected:
    // Heartbeat timer (config heartbeat_period, default 0.05 s) that runs tick(): pose freshness,
    // watchdogs, setpoint TF and report().
    void startTimer();
    void tick();
    // Pose is fresh, autonomy is not blocking, no competing operator, and the UI touched us recently.
    bool ready() const;
    // *Locked helpers expect `mutex` held. Kill disables the switch; release only drops manual control.
    void killLocked(const std::string &);
    void releaseLocked(const std::string &);
    // Called by the adapter when a requestMode() finishes; ignored unless `epoch` is still current.
    void complete(uint64_t epoch, Mode, const Pose &, bool success, const std::string &message);

    // Adapter hooks: publish a command, report the enable switch state, check the mode service, and
    // start/cancel an asynchronous mode change (which ends in complete()).
    virtual void send(const Pose &, Mode) = 0;
    virtual void report() = 0;
    virtual bool modeReady() = 0;
    virtual void requestMode(uint64_t, Mode, const Pose &) = 0;
    virtual void cancelRequest() {}

    std::shared_ptr<RosRuntime> runtime;
    YAML::Node config;
    Context context;

    // Guards everything below; taken by both the UI thread and ROS callbacks.
    std::mutex mutex;
    MotionState value;
    // Maps fixed-frame poses into the command frame; refreshed from TF every tick.
    Pose commandFromFixed{1};
    std::string baseFrame, commandFrame, setpointFrame;
    // Only created when the config names a setpoint_frame.
    std::unique_ptr<tf2_ros::TransformBroadcaster> setpointTf;
    // Becomes true on the first kill/enable; until then tick() shows the idle "Ready" / "Waiting" message.
    bool session = false;
    // Bumped whenever control is taken away, so stale mode-request completions are dropped.
    uint64_t generation = 0;
    int64_t poseStamp = -1;
    // Seconds (config pose_timeout, ui_timeout, request_timeout).
    double poseTimeout, uiTimeout, requestTimeout;
    // competitor: last time another operator was seen; observedSince: when observedKilled was last
    // updated (tick() clears it after 2 s without news).
    Steady::time_point lastUi = Steady::now(), lastPose{}, pendingSince{}, competitor{}, observedSince{};
    rclcpp::TimerBase::SharedPtr timer;
};
} // namespace nereus::ros_viewer::panels

// The shared ROS runtime (node, TF, executor thread), the RosProviders facade that registers every ROS provider
// factory, and RosMotion's adapter-independent enable/kill/command logic and heartbeat watchdogs.
#include "ros_runtime.hpp"
#include "nereus/ros_viewer/panels/ros_providers.hpp"
#include <glm/gtc/matrix_transform.hpp>

namespace nereus::ros_viewer::panels {

// The node ignores global ROS arguments (the host's command line) and follows the context's use_sim_time.
RosRuntime::RosRuntime(const Context &ctx)
    : node(std::make_shared<rclcpp::Node>("viewer_panels", "/" + ctx.robotNamespace,
                                          rclcpp::NodeOptions().use_global_arguments(false).parameter_overrides(
                                              {rclcpp::Parameter("use_sim_time", ctx.useSimTime)}))),
      tf(node->get_clock()), listener(tf, node, false) {
    executor.add_node(node);
}

RosRuntime::~RosRuntime() {
    stop();
}

void RosRuntime::start() {
    if (!worker.joinable())
        worker = std::thread([this] { executor.spin(); });
}

void RosRuntime::stop() {
    executor.cancel();
    if (worker.joinable())
        worker.join();
}

// Owns the one runtime, created by the first provider that asks for it.
struct RosProviders::Impl {
    std::shared_ptr<RosRuntime> runtime;
    std::shared_ptr<RosRuntime> get(const Context &ctx) {
        if (!runtime)
            runtime = std::make_shared<RosRuntime>(ctx);
        return runtime;
    }
};

RosProviders::RosProviders() : impl(std::make_shared<Impl>()) {}

RosProviders::~RosProviders() {
    stop();
}

// start/stop do nothing until some provider has created the runtime.
void RosProviders::start() {
    if (impl->runtime)
        impl->runtime->start();
}

void RosProviders::stop() {
    if (impl->runtime)
        impl->runtime->stop();
}

// Every provider factory shares the lazily created runtime; the UWRT adapters only exist in the UWRT build.
void RosProviders::registerFactories(Registry &registry) {
    RuntimeFactory factory = [owner = impl](const Context &ctx) { return owner->get(ctx); };
#ifdef NEREUS_VIEWER_UWRT
    registerUwrtMotion(registry, factory);
    registerUwrtAutonomy(registry, factory);
#endif
    registerStandardMotion(registry, factory);
    registerSimRun(registry, factory);
    registerSimulationRate(registry, factory);
    registerRosBagging(registry, factory);
#ifdef NEREUS_VIEWER_UWRT
    registerUwrtActuators(registry, factory);
    registerUwrtMapping(registry, factory);
    registerUwrtTelemetry(registry, factory);
    registerUwrtRecording(registry, factory);
    registerUwrtElectrical(registry, factory);
#endif
}

Pose poseMatrix(const geometry_msgs::msg::Transform &t) {
    auto &q = t.rotation;
    return glm::translate(Pose(1), glm::vec3(t.translation.x, t.translation.y, t.translation.z)) *
           glm::mat4_cast(glm::normalize(glm::quat(q.w, q.x, q.y, q.z)));
}

bool finitePose(const Pose &pose) {
    for (int c = 0; c < 4; ++c)
        for (int r = 0; r < 4; ++r)
            if (!std::isfinite(pose[c][r]))
                return false;
    return true;
}

RosMotion::RosMotion(std::shared_ptr<RosRuntime> runtime, const YAML::Node &cfg, const Context &ctx)
    : runtime(std::move(runtime)), config(cfg), context(ctx),
      baseFrame(expand(cfg["base_frame"].as<std::string>(), ctx)),
      commandFrame(expand(cfg["command_frame"].as<std::string>(), ctx)), poseTimeout(cfg["pose_timeout"].as<double>(1)),
      uiTimeout(cfg["ui_timeout"].as<double>(.75)), requestTimeout(cfg["request_timeout"].as<double>(3)) {
    value.frame = ctx.fixedFrame;

    // Optional: broadcast the commanded pose as a TF frame (must not alias a frame the motion itself reads).
    if (cfg["setpoint_frame"]) {
        setpointFrame = expand(cfg["setpoint_frame"].as<std::string>(), ctx);
        if (setpointFrame == ctx.fixedFrame || setpointFrame == baseFrame || setpointFrame == commandFrame)
            throw std::invalid_argument("setpoint_frame must be distinct from motion reference frames");
        setpointTf = std::make_unique<tf2_ros::TransformBroadcaster>(this->runtime->node);
    }
}

MotionState RosMotion::state() {
    std::lock_guard<std::mutex> lock(mutex);
    return value;
}

// The UI liveness heartbeat: a viewer that stops calling this loses manual control after ui_timeout.
void RosMotion::touch() {
    std::lock_guard<std::mutex> lock(mutex);
    lastUi = Steady::now();
}

bool RosMotion::ready() const {
    return value.fresh && !value.blocked && !value.competing &&
           std::chrono::duration<double>(Steady::now() - lastUi).count() < uiTimeout;
}

// Enable is the operator's switch, like the RViz panel: it never waits on pose, the UI watchdog or autonomy.
// Only a competing operator on the same switch refuses it, since the two would fight over the robot.
void RosMotion::enable() {
    std::lock_guard<std::mutex> lock(mutex);
    if (value.pending || value.competing)
        return;

    // Start from a clean, disabled controller state, then turn the switch on.
    killLocked("Enable requested");
    value.enabled = true;
    report();
}

void RosMotion::kill() {
    std::lock_guard<std::mutex> lock(mutex);
    killLocked("Kill requested");
}

// Turns the switch off, drops any request in flight and sends a Disabled command at the last commanded pose.
void RosMotion::killLocked(const std::string &message) {
    session = true;
    value.enabled = false;
    value.pending = false;
    value.mode = Mode::Disabled;
    ++generation;
    cancelRequest();
    value.message = message;
    report();
    send(value.commanded, Mode::Disabled);
}

// Drops the viewer's manual control without touching the enable switch or the controller: the robot keeps
// its last command. Watchdogs use this (never a kill) so a stalled UI, a lost tether or a slow service
// cannot kill a robot running on its own; only the KILL button, the physical kill, a competing operator or the
// viewer coming up (the switch starts killed) do.
void RosMotion::releaseLocked(const std::string &message) {
    ++generation;
    cancelRequest();
    value.pending = false;
    value.mode = Mode::Disabled;
    value.message = message;
}

// Autonomy ownership (Composition::syncOwnership): on block, drop manual control; on release, say how to resume.
void RosMotion::block(bool active) {
    std::lock_guard<std::mutex> lock(mutex);
    if (active && !value.blocked) {
        ++generation;
        cancelRequest();
        value.pending = false;
        value.mode = Mode::Disabled;
        value.message = "Autonomy owns motion";
    }
    if (!active && value.blocked)
        value.message = value.enabled ? "Autonomy ended / Command to resume" : "Autonomy ended / enable to control";
    value.blocked = active;
}

// Asks the adapter to switch the controller mode at `pose`; the result arrives in complete().
void RosMotion::activate(Mode mode, const Pose &pose) {
    std::lock_guard<std::mutex> lock(mutex);
    if (!value.enabled || !ready() || value.pending || !finitePose(pose) || mode == Mode::Disabled ||
        (mode == Mode::Feedforward && !value.supportsFeedforward))
        return;
    if (!modeReady()) {
        value.message = "Control mode service unavailable";
        return;
    }
    value.pending = true;
    pendingSince = Steady::now();
    value.message = "Changing control mode...";
    requestMode(++generation, mode, pose);
}

void RosMotion::complete(uint64_t epoch, Mode mode, const Pose &pose, bool success, const std::string &message) {
    std::lock_guard<std::mutex> lock(mutex);
    if (epoch != generation)
        return;
    value.pending = false;
    if (!success) {
        releaseLocked("Mode request failed: " + message);
        return;
    }
    if (!value.enabled || !ready())
        return;

    // The controller accepted the mode: adopt the pose as the new command and send it.
    value.mode = mode;
    value.commanded = pose;
    value.hasCommand = true;
    ++value.revision;
    value.message = mode == Mode::Position ? "Position control" : "Feedforward control";
    send(pose, mode);
}

// Streams a new position target (gizmo drags); only while already in Position mode.
void RosMotion::drag(const Pose &pose) {
    std::lock_guard<std::mutex> lock(mutex);
    if (!value.enabled || !ready() || value.pending || value.mode != Mode::Position || !finitePose(pose))
        return;
    value.commanded = pose;
    value.hasCommand = true;
    ++value.revision;
    send(pose, Mode::Position);
}

void RosMotion::startTimer() {
    timer =
        runtime->node->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                             std::chrono::duration<double>(config["heartbeat_period"].as<double>(.05))),
                                         [this] { tick(); });
}

// Heartbeat, on the executor thread.
void RosMotion::tick() {
    std::lock_guard<std::mutex> lock(mutex);
    const auto now = Steady::now();

    // Another operator seen within 1 s; forget the robot's kill state after 2 s without a report.
    value.competing = now - competitor < std::chrono::seconds(1);
    if (now - observedSince > std::chrono::seconds(2))
        value.observedKilled.reset();

    // Pose: fresh while the base frame's TF stamp keeps changing within pose_timeout.
    try {
        const auto pose = runtime->tf.lookupTransform(context.fixedFrame, baseFrame, tf2::TimePointZero);
        value.actual = poseMatrix(pose.transform);
        commandFromFixed =
            poseMatrix(runtime->tf.lookupTransform(commandFrame, context.fixedFrame, tf2::TimePointZero).transform);
        const auto stamp = rclcpp::Time(pose.header.stamp).nanoseconds();
        if (stamp != poseStamp) {
            poseStamp = stamp;
            lastPose = now;
        }
        value.fresh = std::chrono::duration<double>(now - lastPose).count() < poseTimeout && finitePose(value.actual) &&
                      finitePose(commandFromFixed);
    } catch (const tf2::TransformException &) {
        value.fresh = false;
    }

    if (!session)
        value.message = value.fresh ? "Ready / enable to control" : "Waiting for pose";

    // Watchdogs: release manual control (never kill) on a stuck request, a silent UI or a stale pose.
    const bool controlling = value.mode != Mode::Disabled || value.pending;
    if (value.pending && std::chrono::duration<double>(now - pendingSince).count() > requestTimeout)
        releaseLocked("Control request timed out");
    else if (controlling && std::chrono::duration<double>(now - lastUi).count() >= uiTimeout)
        releaseLocked("Viewer unresponsive; manual control released, robot holds its last command");
    else if (controlling && !value.fresh)
        releaseLocked("Pose stale; manual control released, robot holds its last command");

    // Setpoint telemetry is independent of manual ownership and gizmo visibility.
    if (setpointTf && value.hasCommand && value.fresh && finitePose(value.commanded)) {
        geometry_msgs::msg::TransformStamped target;
        target.header.stamp = runtime->node->now();
        target.header.frame_id = context.fixedFrame;
        target.child_frame_id = setpointFrame;
        target.transform.translation.x = value.commanded[3].x;
        target.transform.translation.y = value.commanded[3].y;
        target.transform.translation.z = value.commanded[3].z;
        const auto q = glm::normalize(glm::quat_cast(value.commanded));
        target.transform.rotation.w = q.w;
        target.transform.rotation.x = q.x;
        target.transform.rotation.y = q.y;
        target.transform.rotation.z = q.z;
        setpointTf->sendTransform(target);
    }

    // The switch reports from the first heartbeat, so the robot starts killed whenever the viewer comes up.
    report();
}

} // namespace nereus::ros_viewer::panels

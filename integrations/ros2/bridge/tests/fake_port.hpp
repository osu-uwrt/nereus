#pragma once
// Deterministic recording SessionPort for core tests (the generic non-Talos layout of the Python
// bridge tests): time advances one fixed step per advance(), every mutating call is recorded.
#include "core.hpp"

#include <robotics/session/scenario.hpp>

namespace bridge_test {
using namespace robotics;
using namespace robotics::ros_bridge;

constexpr std::int64_t kEpochNs = 1'700'000'000'123'456'789;
constexpr std::int64_t kStepNs = 2'000'000;
inline const Eigen::Vector3d kOffset{0.1, 0.0, -0.05};
inline const char *kWorld = "odom_like_world";

inline Json qos() {
    return {{"reliability", "reliable"}, {"durability", "volatile"}, {"history", "keep_last"}, {"depth", 10}};
}
inline Json publishStream(const std::string &name, const std::string &type, const std::string &native,
                          Json fields, double rate_hz, const std::string &frame_id = "") {
    return {{"id", name}, {"direction", "publish"}, {"topic", "/demo/" + name}, {"message_type", type},
            {"native", native}, {"fields", fields}, {"rate_hz", rate_hz}, {"frame_id", frame_id}, {"qos", qos()}};
}
inline Json subscribeStream(const std::string &name, const std::string &type, const std::string &native,
                            Json fields, Json extra = Json::object()) {
    Json stream = {{"id", name}, {"direction", "subscribe"}, {"topic", "/demo/" + name}, {"message_type", type},
                   {"native", native}, {"fields", fields}, {"rate_hz", 0}, {"frame_id", ""}, {"qos", qos()}};
    for (const auto &[key, value] : extra.items())
        stream[key] = value;
    return stream;
}

inline Json defaultRobot() {
    return Json::parse(R"({
      "reference_frame": "base",
      "frames": {"root": "com", "transforms": [{"parent": "com", "child": "base",
                 "position_m": [0.1, 0.0, -0.05], "orientation_wxyz": [1.0, 0.0, 0.0, 0.0]}]},
      "thrusters": [{"id": "a"}, {"id": "b"}, {"id": "c"}],
      "safety": {"initially_killed": true, "kill_stops_thrusters": true,
                 "commands_while_killed": "zero_force", "kill_disarms_mechanisms": false,
                 "arming": {"required": false}},
      "sensors": [{"id": "alt", "type": "reference_altitude", "frame": "world", "mount_frame": "base",
                   "period_ns": 20000000, "parameters": {}}]})");
}

inline Json defaultBridge() {
    Json bridge = {
        {"namespace", "/demo"},
        {"clock", {{"topic", "/clock"}, {"epoch", "system_time_at_start"}, {"rate_hz", 500},
                   {"reset_policy", "preserve_ros_epoch_and_time"}, {"publish_before_data", true},
                   {"real_time_factor", 1.0}, {"qos", qos()}}},
        {"thrusters", {{"order", {"c", "a", "b"}}, {"input_unit", "N"}, {"input_scales", {2.0, 1.0, -1.0}},
                       {"reject", {"wrong_length", "nonfinite"}}}},
        {"frame_names", {{"world", kWorld}}},
    };
    Json streams = Json::array();
    streams.push_back(publishStream("altitude", "std_msgs/msg/Float64", "sensor:alt",
                                    {{"data", {{"from", "reading.target_world_z"}}}}, 50));
    streams.push_back(publishStream("altitude_stamped", "geometry_msgs/msg/PointStamped", "sensor:alt",
                                    {{"header.stamp", {{"from", "sample.time"}}},
                                     {"point.z", {{"from", "reading.target_world_z"}}}},
                                    50, kWorld));
    streams.push_back(publishStream("ticker", "std_msgs/msg/UInt8", "timer", {{"data", {{"constant", 7}}}}, 10));
    streams.push_back(publishStream(
        "pose", "geometry_msgs/msg/PoseStamped", "state:robot",
        {{"header.stamp", {{"from", "sim.time"}}},
         {"pose.position", {{"from", "reference_pose.position"}}},
         {"pose.orientation.w", {{"from", "reference_pose.orientation_wxyz[0]"}}},
         {"pose.orientation.x", {{"from", "reference_pose.orientation_wxyz[1]"}}},
         {"pose.orientation.y", {{"from", "reference_pose.orientation_wxyz[2]"}}},
         {"pose.orientation.z", {{"from", "reference_pose.orientation_wxyz[3]"}}}},
        100, kWorld));
    streams.push_back(publishStream("kill_event", "std_msgs/msg/Bool", "event:robot.kill_changed",
                                    {{"data", {{"from", "killed"}}}}, 0));
    streams.push_back(subscribeStream("thruster_cmd", "std_msgs/msg/Float64MultiArray", "command:thrusters.set_forces",
                                      {{"forces_n", {{"from", "data"}}}}));
    streams.push_back(subscribeStream("kill_cmd", "std_msgs/msg/Bool", "command:robot.set_killed",
                                      {{"killed", {{"from", "data"}}}},
                                      {{"accept_if", Json::array({{{"field", "data"}, {"equals", true}}})}}));
    streams.push_back(subscribeStream("unkill_cmd", "std_msgs/msg/Bool", "command:robot.set_killed",
                                      {{"killed", {{"from", "data"}}}},
                                      {{"accept_if", Json::array({{{"field", "data"}, {"equals", false}}})}}));
    bridge["streams"] = streams;
    return bridge;
}

inline session::ResolvedScenario makeScenario(const Json &bridge, const Json &robot) {
    Json document = {{"format", "robotics_platform.resolved_scenario"},
                     {"version", 1},
                     {"scenario", {{"world_frame", "scenario_world"}, {"seed", 0}}},
                     {"robot", robot},
                     {"pool", Json::object()},
                     {"tasks", Json::object()},
                     {"task_definitions", Json::array()},
                     {"bridge", bridge},
                     {"run_options", Json::object()},
                     {"asset_paths", Json::object()}};
    return session::parseResolvedScenario(document);
}

class FakePort : public SessionPort {
  public:
    FakePort() {
        spatial::Pose pose;
        pose.translation = kOffset;
        frames_ = std::make_shared<spatial::FixedFrames>(
            "com", std::vector<spatial::FixedFrame>{{"com", "base", pose}});
        body_.position = {1.0, 2.0, 3.0};
        start_.position = {1.0, 2.0, 3.0};
    }
    // scripted state
    std::uint64_t tick{0}, generation{0};
    simulation::BodyState body_, start_;
    bool killed_{true};
    std::vector<std::string> sensors{"alt"};
    std::vector<SensorSample> queue;
    std::vector<Eigen::VectorXd> commands;
    int stops{0};
    struct Placement {
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
        bool clear;
    };
    std::vector<Placement> placed;
    std::vector<std::string> calls;
    Json feed{Json::array()}, indicator_list{Json::array()};
    std::vector<PropVisual> props;
    std::vector<PayloadVisual> payloads;
    std::optional<Json> run_snapshot;
    session::CommandResult next_result{true, "ok"};
    session::Events fire_events;

    simulation::Snapshot snapshot() const {
        simulation::Snapshot s;
        s.generation = generation;
        s.tick = tick;
        s.elapsed = std::chrono::nanoseconds(static_cast<std::int64_t>(tick) * kStepNs);
        s.body = body_;
        s.thruster_forces = Eigen::VectorXd::Zero(3);
        return s;
    }
    StepResult advance() override {
        ++tick;
        last_ = snapshot();
        return {last_, {}};
    }
    simulation::Snapshot observe() const override { return snapshot(); }
    const simulation::Snapshot &lastSnapshot() const override { return last_; }
    std::int64_t timeNs() const override { return static_cast<std::int64_t>(tick) * kStepNs; }
    std::int64_t timestepNs() const override { return kStepNs; }
    std::vector<SensorSample> drainSensors() override {
        std::vector<SensorSample> out;
        out.swap(queue);
        return out;
    }
    std::map<std::string, sensors::StreamStats> sensorStats() const override { return {}; }
    std::vector<std::string> sensorIds() const override { return sensors; }
    std::vector<std::string> deferredSensorIds() const override { return {}; }
    std::shared_ptr<const spatial::FixedFrames> frames() const override { return frames_; }
    spatial::Pose referencePose(const simulation::BodyState &b) const override {
        return spatial::compose(spatial::Pose{b.position, b.orientation}, frames_->fromRoot("base"));
    }
    void commandThrusters(const Eigen::VectorXd &f) override { commands.push_back(f); }
    void setKilled(bool k) override {
        if (k)
            ++stops;
        killed_ = k;
    }
    bool killed() const override { return killed_; }
    session::CommandResult setArmed(bool armed) override {
        calls.push_back(std::string("set_armed:") + (armed ? "1" : "0"));
        return next_result;
    }
    session::CommandResult reloadAll() override { calls.push_back("reload_all"); return next_result; }
    session::CommandResult commandClaw(const std::string &id, bool open) override {
        calls.push_back("claw:" + id + ":" + (open ? "open" : "close"));
        return next_result;
    }
    session::CommandResult moveClaw(const std::string &id, double s) override {
        calls.push_back("move_claw:" + id + ":" + std::to_string(s));
        return next_result;
    }
    session::CommandResult fire(const std::string &id, session::Events &events) override {
        calls.push_back("fire:" + id);
        for (const auto &e : fire_events)
            events.push_back(e);
        return next_result;
    }
    std::optional<session::MechanismState> mechanismState() const override { return state; }
    std::optional<session::MechanismState> state;
    simulation::Snapshot place(const simulation::BodyState &s, bool clear) override {
        placed.push_back({s.position, s.orientation, clear});
        body_ = s;
        return snapshot();
    }
    const simulation::BodyState &startState() const override { return start_; }
    session::CommandResult resetTasks() override { calls.push_back("reset_tasks"); return next_result; }
    simulation::Snapshot fullReset() override {
        calls.push_back("full_reset");
        ++generation;
        tick = 0;
        return snapshot();
    }
    std::uint64_t seed() const override { return 7; }
    session::CommandResult runStart(const Json &o) override { calls.push_back("run_start:" + o.dump()); return next_result; }
    session::CommandResult runStop() override { calls.push_back("run_stop"); return next_result; }
    session::CommandResult runAdjust(double p) override { calls.push_back("run_adjust:" + std::to_string(p)); return next_result; }
    std::optional<Json> runSnapshot() const override { return run_snapshot; }
    Json takeFeed() override {
        Json out = feed;
        feed = Json::array();
        return out;
    }
    Json taskCounters() const override { return {{"gate", 1}}; }
    Eigen::VectorXd thrusterForces() const override { return Eigen::Vector3d(4.0, -6.0, 2.0); }
    std::map<std::string, std::array<double, 2>> clawJaws() const override { return {{"claw", {0.01, 0.02}}}; }
    Json indicators() const override { return indicator_list; }
    std::vector<PropVisual> propVisuals() const override { return props; }
    std::vector<PayloadVisual> payloadVisuals() const override { return payloads; }

  private:
    std::shared_ptr<spatial::FixedFrames> frames_;
    simulation::Snapshot last_;
};

inline SensorSample altSample(std::int64_t acquired_ns, std::optional<double> z) {
    SensorSample sample;
    sample.sensor = "alt";
    sample.acquired_ns = acquired_ns;
    if (z)
        sample.reading = Value::map({{"mounted_world_z", Value::real(*z + 0.5)},
                                     {"target_world_z", Value::real(*z)}, {"variance", Value::real(0.01)}});
    return sample;
}

inline std::int64_t stampNs(const Json &stamp) {
    return stamp.at("sec").get<std::int64_t>() * 1'000'000'000 + stamp.at("nanosec").get<std::int64_t>();
}
inline Json publicationJson(const Publication &p) {
    return messageToJson(p.message->type().members(), p.message->data());
}
inline std::vector<Publication> byStream(const std::vector<Publication> &all, const std::string &stream) {
    std::vector<Publication> out;
    for (const auto &item : all)
        if (item.stream == stream)
            out.push_back(item);
    return out;
}
} // namespace bridge_test

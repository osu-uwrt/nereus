// BridgeCore semantics on a generic (non-Talos) layout with a fake, recording session (port of
// integrations/ros2/tests/test_bridge_core.py, test_bridge_alignment.py, test_bridge_static_tf.py
// and the placement/run-command/reset parts of test_bridge_mechanisms.py). No middleware.
#include "fake_port.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <robot_localization/srv/set_pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <gtest/gtest.h>

using namespace bridge_test;

namespace {
const Eigen::Quaterniond kYaw90(std::sqrt(0.5), 0, 0, std::sqrt(0.5));

Json withStreams(const std::map<std::string, Json> &changes) {
    Json bridge = defaultBridge(), streams = Json::array();
    for (const auto &stream : bridge["streams"]) {
        const auto found = changes.find(stream["id"]);
        if (found == changes.end()) {
            streams.push_back(stream);
        } else if (!found->second.is_null()) {
            Json changed = stream;
            for (const auto &[key, value] : found->second.items())
                changed[key] = value;
            streams.push_back(changed);
        }
    }
    bridge["streams"] = streams;
    return bridge;
}

Json setPoseService(bool becomes_start = true, bool align = false) {
    return {{"id", "set_pose"}, {"service", "set_pose"}, {"service_type", "robot_localization/srv/SetPose"},
            {"action", "command:robot.place"},
            {"request", {{"frame", {{"from", "pose.header.frame_id"}}},
                         {"position_m", {{"from", "pose.pose.pose.position"}}},
                         {"orientation_w", {{"from", "pose.pose.pose.orientation.w"}}},
                         {"orientation_x", {{"from", "pose.pose.pose.orientation.x"}}},
                         {"orientation_y", {{"from", "pose.pose.pose.orientation.y"}}},
                         {"orientation_z", {{"from", "pose.pose.pose.orientation.z"}}}}},
            {"response", Json::object()},
            {"placement", {{"pose_source", "request"}, {"keep_velocity", false},
                           {"becomes_start_pose", becomes_start}, {"align_estimator", align}}}};
}
Json resetService() {
    return {{"id", "reset"}, {"service", "reset"}, {"service_type", "std_srvs/srv/Trigger"},
            {"action", "command:robot.reset_to_start"}, {"request", Json::object()},
            {"response", {{"success", {{"from", "accepted"}}}, {"message", {{"from", "message"}}}}}};
}
Json withServices(std::vector<Json> services, Json bridge = defaultBridge()) {
    bridge["services"] = Json::array();
    for (auto &service : services)
        bridge["services"].push_back(service);
    return bridge;
}

struct Rig {
    session::ResolvedScenario resolved;
    FakePort port;
    std::unique_ptr<BridgeCore> core;
    explicit Rig(const Json &bridge = defaultBridge(), const Json &robot = defaultRobot(),
                 std::vector<std::string> selected = {"alt"}, Lookup lookup = {})
        : resolved(makeScenario(bridge, robot)) {
        port.sensors = std::move(selected);
        core = std::make_unique<BridgeCore>(resolved, port, kEpochNs, std::move(lookup));
    }
};

template <class Fn> std::string bridgeError(Fn &&fn) {
    try {
        fn();
    } catch (const BridgeError &error) {
        return error.what();
    }
    return "<no error>";
}

template <class T> std::shared_ptr<Message> makeMessage(const char *type) {
    return std::make_shared<Message>(MessageType::get(type));
}
std::shared_ptr<Message> boolMessage(bool value) {
    auto message = makeMessage<std_msgs::msg::Bool>("std_msgs/msg/Bool");
    static_cast<std_msgs::msg::Bool *>(message->data())->data = value;
    return message;
}
std::shared_ptr<Message> floats(std::vector<double> values) {
    auto message = makeMessage<std_msgs::msg::Float64MultiArray>("std_msgs/msg/Float64MultiArray");
    static_cast<std_msgs::msg::Float64MultiArray *>(message->data())->data = std::move(values);
    return message;
}
std::vector<Publication> receive(Rig &rig, const std::string &stream, const std::shared_ptr<Message> &message) {
    return rig.core->receive(stream, message->data());
}
robot_localization::srv::SetPose::Request poseRequest(const std::string &frame, Eigen::Vector3d p,
                                                       Eigen::Quaterniond q = Eigen::Quaterniond::Identity()) {
    robot_localization::srv::SetPose::Request request;
    request.pose.header.frame_id = frame;
    request.pose.pose.pose.position.x = p.x();
    request.pose.pose.pose.position.y = p.y();
    request.pose.pose.pose.position.z = p.z();
    request.pose.pose.pose.orientation.w = q.w();
    request.pose.pose.pose.orientation.x = q.x();
    request.pose.pose.pose.orientation.y = q.y();
    request.pose.pose.pose.orientation.z = q.z();
    return request;
}
} // namespace

// ------------------------------------------------------------------ construction

TEST(Construction, SensorRateMismatchRaises) {
    EXPECT_NE(bridgeError([] { Rig rig(withStreams({{"altitude", {{"rate_hz", 49}}}})); }).find("rate_hz"), std::string::npos);
}
TEST(Construction, UnknownNativeEndpointRaises) {
    EXPECT_NE(bridgeError([] { Rig rig(withStreams({{"ticker", {{"native", "state:unknown"}}}})); }).find("state:unknown"),
              std::string::npos);
}
TEST(Construction, UnselectedAndUnknownSensorsRaise) {
    EXPECT_NE(bridgeError([] { Rig rig(defaultBridge(), defaultRobot(), {}); }).find("not selected"), std::string::npos);
    EXPECT_NE(bridgeError([] { Rig rig(withStreams({{"altitude", {{"native", "sensor:missing"}}}})); }).find("unknown robot sensor"),
              std::string::npos);
}
TEST(Construction, ImageStreamsAreNotExecuted) {
    EXPECT_NE(bridgeError([] { Rig rig(withStreams({{"ticker", {{"image", {{"encoding", "rgb8"}}}}}})); }).find("image"),
              std::string::npos);
}
TEST(Construction, FastTimersAndClocksRaise) {
    EXPECT_NE(bridgeError([] { Rig rig(withStreams({{"ticker", {{"rate_hz", 1000}}}})); }).find("physics step rate"), std::string::npos);
    Json bridge = defaultBridge();
    bridge["clock"]["rate_hz"] = 1000;
    EXPECT_NE(bridgeError([&] { Rig rig(bridge); }).find("clock"), std::string::npos);
}
TEST(Construction, ThrusterBlockValidation) {
    Json bridge = defaultBridge();
    bridge["thrusters"]["order"] = {"c", "a", "a"};
    EXPECT_NE(bridgeError([&] { Rig rig(bridge); }).find("permutation"), std::string::npos);
    bridge = defaultBridge();
    bridge["thrusters"]["reject"] = {"wrong_length"};
    EXPECT_NE(bridgeError([&] { Rig rig(bridge); }).find("reject"), std::string::npos);
    bridge = defaultBridge();
    bridge.erase("thrusters");
    EXPECT_NE(bridgeError([&] { Rig rig(bridge); }).find("thrusters block"), std::string::npos);
}
TEST(Construction, InvalidFieldMapIsABridgeError) {
    EXPECT_NE(bridgeError([] { Rig rig(withStreams({{"altitude", {{"fields", {{"data", {{"from", "reading.nope"}}}}}}}})); }),
              "<no error>");
}
TEST(Construction, EpochAndWorldFrameComeFromConfiguration) {
    Rig rig;
    EXPECT_EQ(rig.core->worldFrame(), kWorld);
    EXPECT_EQ(rig.core->clockNs(), kEpochNs);
}
TEST(Construction, UnsupportedServiceTypeAndActionRaise) {
    Json service = resetService();
    service["id"] = "teleport";
    service["action"] = "command:robot.teleport";
    EXPECT_NE(bridgeError([&] { Rig rig(withServices({service})); }).find("command:robot.teleport"), std::string::npos);
    service = resetService();
    service["service_type"] = "std_srvs/srv/Empty";
    EXPECT_NE(bridgeError([&] { Rig rig(withServices({service})); }).find("not supported by this bridge"), std::string::npos);
}
TEST(Construction, KillBindingsMustMatchEndpoints) {
    Json bridge = defaultBridge();
    bridge["streams"].push_back(publishStream("kill_dup", "std_msgs/msg/Bool", "event:robot.kill_changed",
                                               {{"data", {{"from", "killed"}}}}, 0));
    bridge["kill"] = {{"command_stream", "kill_cmd"}, {"state_stream", "altitude"}};
    EXPECT_NE(bridgeError([&] { Rig rig(bridge); }).find("kill.state_stream"), std::string::npos);
}

// ------------------------------------------------------------------ stepping

TEST(Stepping, ClockPrecedesDataAndDataUsesAcquisitionTime) {
    Rig rig;
    rig.port.queue = {altSample(500'000, -1.5), altSample(1'000'000, std::nullopt), altSample(1'500'000, -2.5)};
    const auto out = rig.core->step();
    EXPECT_EQ(out.clocks, std::vector<std::int64_t>{kEpochNs + kStepNs});
    EXPECT_TRUE(out.transforms.empty());
    const auto plain = byStream(out.publications, "altitude");
    ASSERT_EQ(plain.size(), 2u);
    EXPECT_EQ(publicationJson(plain[0]).at("data"), -1.5);
    EXPECT_EQ(publicationJson(plain[1]).at("data"), -2.5);
    const auto stamped = byStream(out.publications, "altitude_stamped");
    ASSERT_EQ(stamped.size(), 2u);
    EXPECT_EQ(stampNs(publicationJson(stamped[0])["header"]["stamp"]), kEpochNs + 500'000);
    EXPECT_EQ(stampNs(publicationJson(stamped[1])["header"]["stamp"]), kEpochNs + 1'500'000);
    EXPECT_EQ(publicationJson(stamped[1])["point"]["z"], -2.5);
    EXPECT_EQ(publicationJson(stamped[0])["header"]["frame_id"], kWorld);
    EXPECT_EQ(out.publications.size(), 4u);
}

TEST(Stepping, UnavailableSamplesAreCountedNotPublished) {
    Rig rig;
    rig.port.queue = {altSample(500'000, std::nullopt), altSample(1'000'000, -1.0)};
    const auto out = rig.core->step();
    EXPECT_EQ(rig.core->counters().unavailable_samples, (CounterTable{{"alt", 1}}));
    EXPECT_EQ(byStream(out.publications, "altitude").size(), 1u);
    EXPECT_EQ(rig.core->counters().published.at("altitude"), 1u);
    EXPECT_EQ(rig.core->counters().published.at("altitude_stamped"), 1u);
}

TEST(Stepping, ClockStampFollowsElapsedTimeEveryStep) {
    Rig rig;
    std::vector<std::int64_t> stamps;
    for (int k = 0; k < 5; ++k)
        for (const auto stamp : rig.core->step().clocks)
            stamps.push_back(stamp);
    std::vector<std::int64_t> expected;
    for (int n = 1; n <= 5; ++n)
        expected.push_back(kEpochNs + kStepNs * n);
    EXPECT_EQ(stamps, expected);
    EXPECT_EQ(rig.core->clockNs(), kEpochNs + 5 * kStepNs);
}

TEST(Stepping, SlowClockPublishesAtItsPeriod) {
    Json bridge = defaultBridge();
    bridge["clock"]["rate_hz"] = 100;
    Rig rig(bridge);
    std::vector<std::int64_t> stamps;
    for (int k = 0; k < 20; ++k)
        for (const auto stamp : rig.core->step().clocks)
            stamps.push_back(stamp);
    EXPECT_EQ(stamps, (std::vector<std::int64_t>{kEpochNs + kStepNs, kEpochNs + 10'000'000, kEpochNs + 20'000'000,
                                                 kEpochNs + 30'000'000, kEpochNs + 40'000'000}));
}

TEST(Stepping, TimerAndStatePublishAtTheirPeriods) {
    Rig rig;
    std::vector<Publication> all;
    for (int k = 0; k < 100; ++k)
        for (auto &item : rig.core->step().publications)
            all.push_back(item);
    const auto ticker = byStream(all, "ticker"), pose = byStream(all, "pose");
    EXPECT_EQ(ticker.size(), 2u);
    EXPECT_EQ(pose.size(), 20u);
    EXPECT_EQ(publicationJson(ticker[0]).at("data"), 7);
    for (std::size_t n = 0; n < pose.size(); ++n)
        EXPECT_EQ(stampNs(publicationJson(pose[n])["header"]["stamp"]), kEpochNs + 10'000'000 * static_cast<std::int64_t>(n + 1));
    EXPECT_EQ(rig.core->counters().published.at("ticker"), 2u);
    EXPECT_EQ(rig.core->counters().published.count("altitude"), 0u);
}

TEST(Stepping, PoseIsTheCenterOfMassPoseComposedWithTheReferenceOffset) {
    Rig rig;
    rig.port.body_.position = {1.0, 2.0, 3.0};
    rig.port.body_.orientation = kYaw90;
    std::vector<Publication> published;
    for (int k = 0; k < 5; ++k)
        for (auto &item : byStream(rig.core->step().publications, "pose"))
            published.push_back(item);
    ASSERT_EQ(published.size(), 1u);
    const Json pose = publicationJson(published[0]);
    EXPECT_EQ(pose["header"]["frame_id"], kWorld);
    EXPECT_NEAR(pose["pose"]["position"]["x"].get<double>(), 1.0, 1e-12);
    EXPECT_NEAR(pose["pose"]["position"]["y"].get<double>(), 2.1, 1e-12); // yaw 90 maps x to y
    EXPECT_NEAR(pose["pose"]["position"]["z"].get<double>(), 2.95, 1e-12);
    EXPECT_NEAR(pose["pose"]["orientation"]["w"].get<double>(), std::sqrt(0.5), 1e-12);
    EXPECT_NEAR(pose["pose"]["orientation"]["z"].get<double>(), std::sqrt(0.5), 1e-12);
}

TEST(Stepping, TransformsAreProducedOnlyWhenConfigured) {
    Json bridge = defaultBridge();
    bridge["tf"] = {{"publish", Json::array({{{"native", "state:robot.reference_pose"}, {"parent", kWorld},
                                              {"child", "base"}, {"rate_hz", 100}}})}};
    Rig rig(bridge);
    std::vector<Transform> transforms;
    for (int k = 0; k < 10; ++k)
        for (auto &t : rig.core->step().transforms)
            transforms.push_back(t);
    ASSERT_EQ(transforms.size(), 2u);
    EXPECT_EQ(transforms[0].parent, kWorld);
    EXPECT_EQ(transforms[0].child, "base");
    EXPECT_EQ(transforms[0].stamp_ns, kEpochNs + 5 * kStepNs);
    EXPECT_TRUE(transforms[0].translation.isApprox(Eigen::Vector3d(1.1, 2.0, 2.95)));
}

TEST(Stepping, ResetsPreserveRosTimeAndReplayTheClock) {
    Json bridge = withServices({{{"id", "full"}, {"service", "full"}, {"service_type", "std_srvs/srv/Trigger"},
                                 {"action", "command:scenario.reset"}, {"request", Json::object()},
                                 {"response", {{"success", {{"from", "accepted"}}}, {"message", {{"from", "message"}}}}}}});
    Rig rig(bridge);
    for (int k = 0; k < 10; ++k)
        rig.core->step();
    const auto before = rig.core->clockNs();
    std_srvs::srv::Trigger::Request request;
    std_srvs::srv::Trigger::Response response;
    rig.core->call("full", &request, &response);
    EXPECT_TRUE(response.success);
    EXPECT_EQ(rig.port.calls.back(), "full_reset");
    const auto out = rig.core->step();
    ASSERT_EQ(out.clocks.size(), 1u);
    EXPECT_GT(out.clocks[0], before); // ROS time never rewinds; the offset carries it forward
    EXPECT_EQ(out.clocks[0], before + 2 * kStepNs);
}

TEST(Stepping, FeedAndStatusStreamsUseTheSessionRecords) {
    Json bridge = defaultBridge();
    bridge["streams"].push_back({{"id", "feed"}, {"direction", "publish"}, {"topic", "/demo/feed"},
                                 {"message_type", "std_msgs/msg/String"}, {"native", "event:tasks.feed"},
                                 {"format", "json"}, {"fields", Json::object()}, {"rate_hz", 0},
                                 {"frame_id", ""}, {"qos", qos()}});
    bridge["streams"].push_back({{"id", "score"}, {"direction", "publish"}, {"topic", "/demo/score"},
                                 {"message_type", "std_msgs/msg/String"}, {"native", "state:task_score"},
                                 {"format", "json"}, {"fields", Json::object()}, {"rate_hz", 100},
                                 {"frame_id", ""}, {"qos", qos()}});
    Rig rig(bridge);
    rig.port.feed = Json::array({{{"type", "gate_pass"}}, {{"type", "torpedo_hit"}}});
    std::vector<Publication> all;
    for (int k = 0; k < 5; ++k)
        for (auto &item : rig.core->step().publications)
            all.push_back(item);
    const auto feed = byStream(all, "feed");
    ASSERT_EQ(feed.size(), 2u);
    EXPECT_EQ(Json::parse(publicationJson(feed[1]).at("data").get<std::string>()).at("type"), "torpedo_hit");
    const auto score = byStream(all, "score");
    ASSERT_EQ(score.size(), 1u);
    EXPECT_EQ(Json::parse(publicationJson(score[0]).at("data").get<std::string>()), (Json{{"gate", 1}}));
}

// ------------------------------------------------------------------ commands

TEST(Commands, ThrusterOrderAndScalesMapRosDataToNativeOrder) {
    Rig rig;
    receive(rig, "unkill_cmd", boolMessage(false));
    EXPECT_TRUE(receive(rig, "thruster_cmd", floats({1.0, 2.0, 3.0})).empty());
    ASSERT_EQ(rig.port.commands.size(), 1u);
    // order [c, a, b] with scales [2, 1, -1]: c=1*2, a=2*1, b=3*-1; native order [a, b, c]
    EXPECT_EQ(rig.port.commands[0], Eigen::Vector3d(2.0, -3.0, 2.0));
}

TEST(Commands, WrongLengthAndNonfiniteAreRejectedAndCounted) {
    Rig rig;
    receive(rig, "unkill_cmd", boolMessage(false));
    receive(rig, "thruster_cmd", floats({1.0, 2.0}));
    receive(rig, "thruster_cmd", floats({1.0, 2.0, 3.0, 4.0}));
    receive(rig, "thruster_cmd", floats({1.0, std::nan(""), 3.0}));
    receive(rig, "thruster_cmd", floats({INFINITY, 0.0, 0.0}));
    receive(rig, "thruster_cmd", floats({1e308, 0.0, 0.0})); // finite but overflows when scaled
    EXPECT_TRUE(rig.port.commands.empty());
    EXPECT_EQ(rig.core->counters().rejected_commands,
              (CounterTable{{"thruster_cmd:wrong_length", 2}, {"thruster_cmd:nonfinite", 2}, {"thruster_cmd:nonfinite_scaled", 1}}));
}

TEST(Commands, InitiallyKilledPoliciesZeroOrReject) {
    Rig zero;
    receive(zero, "thruster_cmd", floats({1.0, 2.0, 3.0}));
    ASSERT_EQ(zero.port.commands.size(), 1u);
    EXPECT_EQ(zero.port.commands[0], Eigen::Vector3d::Zero());
    EXPECT_TRUE(zero.core->counters().rejected_commands.empty());

    Json robot = defaultRobot();
    robot["safety"]["commands_while_killed"] = "rejected";
    Rig rejected(defaultBridge(), robot);
    receive(rejected, "thruster_cmd", floats({1.0, 2.0, 3.0}));
    EXPECT_TRUE(rejected.port.commands.empty());
    EXPECT_EQ(rejected.core->counters().rejected_commands, (CounterTable{{"thruster_cmd:killed", 1}}));
}

TEST(Commands, KillAndUnkillPublishOneEventEach) {
    Rig rig;
    auto events = receive(rig, "unkill_cmd", boolMessage(false));
    ASSERT_EQ(events.size(), 1u);
    EXPECT_EQ(publicationJson(events[0]).at("data"), false);
    EXPECT_FALSE(rig.core->killed());
    EXPECT_EQ(rig.port.stops, 0);
    events = receive(rig, "kill_cmd", boolMessage(true));
    EXPECT_TRUE(rig.core->killed());
    EXPECT_EQ(rig.port.stops, 1);
    ASSERT_EQ(events.size(), 1u);
    EXPECT_EQ(publicationJson(events[0]).at("data"), true);
    EXPECT_EQ(rig.core->counters().published.at("kill_event"), 2u);
    receive(rig, "thruster_cmd", floats({1.0, 2.0, 3.0}));
    EXPECT_EQ(rig.port.commands.back(), Eigen::Vector3d::Zero());
}

TEST(Commands, AcceptIfFiltersAreCountedAndIgnored) {
    Rig rig;
    receive(rig, "unkill_cmd", boolMessage(false));
    EXPECT_TRUE(receive(rig, "kill_cmd", boolMessage(false)).empty());
    EXPECT_TRUE(receive(rig, "unkill_cmd", boolMessage(true)).empty());
    EXPECT_FALSE(rig.core->killed());
    EXPECT_EQ(rig.core->counters().filtered_messages, (CounterTable{{"kill_cmd", 1}, {"unkill_cmd", 1}}));
    EXPECT_EQ(rig.core->counters().published, (CounterTable{{"kill_event", 1}}));
}

TEST(Commands, RunCommandsStartStopAdjustAndRejectMalformedText) {
    Rig rig;
    EXPECT_TRUE(rig.core->runCommand(R"({"action": "start", "role": "b"})").accepted);
    EXPECT_EQ(rig.port.calls.back(), R"(run_start:{"role":"b"})");
    EXPECT_TRUE(rig.core->runCommand(R"({"action": "stop"})").accepted);
    EXPECT_TRUE(rig.core->runCommand(R"({"action": "adjustment", "points": 5})").accepted);
    EXPECT_EQ(rig.port.calls.back(), "run_adjust:5.000000");
    for (const char *bad : {"not json", R"({"action": "dance"})", R"({})", R"({"action": "adjustment"})",
                            R"({"action": "adjustment", "points": "x"})", "[1]"})
        EXPECT_FALSE(rig.core->runCommand(bad).accepted) << bad;
    const std::uint64_t rejected = rig.core->counters().rejected_commands.at("run_command");
    EXPECT_EQ(rejected, 6u);
    const auto unknown = rig.core->runCommand("{\"action\": \"dance\"}");
    EXPECT_EQ(unknown.message, "Unknown run command");
}

TEST(RealTimeFactor, ValidatesAndAppliesSpeed) {
    Rig rig;
    EXPECT_EQ(rig.core->realTimeFactor(), 1.0);
    EXPECT_FALSE(rig.core->setRealTimeFactor(0.0).has_value()); // pause
    EXPECT_FALSE(rig.core->setRealTimeFactor(2.5).has_value());
    EXPECT_EQ(rig.core->realTimeFactor(), 2.5);
    EXPECT_TRUE(rig.core->setRealTimeFactor(-1.0).has_value());
    EXPECT_TRUE(rig.core->setRealTimeFactor(INFINITY).has_value());
    EXPECT_TRUE(rig.core->setRealTimeFactor(std::nan("")).has_value());
    EXPECT_EQ(rig.core->realTimeFactor(), 2.5);
}

// ------------------------------------------------------------------ placement

TEST(Placement, WorldFrameRequestPlacesCenterOfMassFromReferencePose) {
    Rig rig(withServices({setPoseService(), resetService()}));
    auto request = poseRequest("", {2.0, 3.0, -1.0});
    robot_localization::srv::SetPose::Response response;
    rig.core->call("set_pose", &request, &response);
    ASSERT_EQ(rig.port.placed.size(), 1u);
    EXPECT_TRUE(rig.port.placed[0].position.isApprox(Eigen::Vector3d(2.0, 3.0, -1.0) - kOffset, 1e-12));
    EXPECT_TRUE(rig.port.placed[0].orientation.isApprox(Eigen::Quaterniond::Identity()));
    EXPECT_TRUE(rig.port.placed[0].clear);
    EXPECT_EQ(rig.core->counters().service_calls, (CounterTable{{"set_pose", 1}}));
}

TEST(Placement, RotatedRequestRotatesTheOffset) {
    Rig rig(withServices({setPoseService()}));
    auto request = poseRequest(kWorld, {2.0, 3.0, -1.0}, kYaw90);
    robot_localization::srv::SetPose::Response response;
    rig.core->call("set_pose", &request, &response);
    ASSERT_EQ(rig.port.placed.size(), 1u);
    EXPECT_TRUE(rig.port.placed[0].position.isApprox(Eigen::Vector3d(2.0, 3.0 - 0.1, -1.0 + 0.05), 1e-12));
    EXPECT_TRUE(rig.port.placed[0].orientation.isApprox(kYaw90));
}

TEST(Placement, NonWorldFramesUseTheInjectedLookup) {
    std::vector<std::pair<std::string, std::string>> calls;
    Rig rig(withServices({setPoseService()}), defaultRobot(), {"alt"},
            [&](const std::string &target, const std::string &source) -> std::optional<spatial::Pose> {
                calls.push_back({target, source});
                spatial::Pose pose;
                pose.translation = {10.0, 0.0, 0.0};
                return pose;
            });
    auto request = poseRequest("map", {1.0, 1.0, 1.0});
    robot_localization::srv::SetPose::Response response;
    rig.core->call("set_pose", &request, &response);
    ASSERT_EQ(calls.size(), 1u);
    EXPECT_EQ(calls[0], std::make_pair(std::string(kWorld), std::string("map")));
    EXPECT_TRUE(rig.port.placed.at(0).position.isApprox(Eigen::Vector3d(11.0, 1.0, 1.0) - kOffset, 1e-12));
}

TEST(Placement, MissingTransformZeroQuaternionAndNonfinitePositionAreRefused) {
    Rig none(withServices({setPoseService()}), defaultRobot(), {"alt"},
             [](const std::string &, const std::string &) { return std::nullopt; });
    robot_localization::srv::SetPose::Response response;
    auto request = poseRequest("map", {1.0, 1.0, 1.0});
    none.core->call("set_pose", &request, &response);
    EXPECT_TRUE(none.port.placed.empty());

    Rig rig(withServices({setPoseService()}));
    auto zero = poseRequest("", {0, 0, 0});
    zero.pose.pose.pose.orientation.w = 0;
    rig.core->call("set_pose", &zero, &response);
    auto nan = poseRequest("", {std::nan(""), 0, 0});
    rig.core->call("set_pose", &nan, &response);
    auto unknown = poseRequest("map", {0, 0, 0}); // no lookup at all
    rig.core->call("set_pose", &unknown, &response);
    EXPECT_TRUE(rig.port.placed.empty());
}

TEST(Placement, ResetReturnsToThePlacedStartPose) {
    Rig rig(withServices({setPoseService(), resetService()}));
    robot_localization::srv::SetPose::Response set_response;
    auto request = poseRequest("", {2.0, 3.0, -1.0});
    rig.core->call("set_pose", &request, &set_response);
    rig.port.body_.position = {9.0, 9.0, 9.0}; // the vehicle drifts away
    std_srvs::srv::Trigger::Request trigger;
    std_srvs::srv::Trigger::Response response;
    rig.core->call("reset", &trigger, &response);
    EXPECT_TRUE(response.success);
    EXPECT_NE(response.message.find("(2.000, 3.000, -1.000)"), std::string::npos);
    ASSERT_EQ(rig.port.placed.size(), 2u);
    EXPECT_TRUE(rig.port.placed[1].position.isApprox(Eigen::Vector3d(2.0, 3.0, -1.0) - kOffset, 1e-12));
    EXPECT_TRUE(rig.port.placed[1].clear);
}

TEST(Placement, ResetWithoutPlacementReturnsToTheInitialStateAndNonStartPlacementKeepsIt) {
    Rig initial(withServices({resetService()}));
    std_srvs::srv::Trigger::Request trigger;
    std_srvs::srv::Trigger::Response response;
    initial.core->call("reset", &trigger, &response);
    ASSERT_EQ(initial.port.placed.size(), 1u);
    EXPECT_TRUE(initial.port.placed[0].position.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));

    Rig kept(withServices({setPoseService(false), resetService()}));
    auto request = poseRequest("", {2.0, 3.0, -1.0});
    robot_localization::srv::SetPose::Response set_response;
    kept.core->call("set_pose", &request, &set_response);
    kept.core->call("reset", &trigger, &response);
    EXPECT_TRUE(kept.port.placed[1].position.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
}

TEST(Placement, InvalidPlacementFromTheSessionIsRefusedNotThrown) {
    struct Throwing : FakePort {
        simulation::Snapshot place(const simulation::BodyState &, bool) override {
            throw std::invalid_argument("outside the pool");
        }
    };
    session::ResolvedScenario resolved = makeScenario(withServices({setPoseService()}), defaultRobot());
    Throwing port;
    BridgeCore core(resolved, port, kEpochNs);
    auto request = poseRequest("", {1, 1, 1});
    robot_localization::srv::SetPose::Response response;
    core.call("set_pose", &request, &response); // the SetPose response is empty; nothing thrown
    SUCCEED();
}

TEST(Placement, MalformedRequestIsRejectedWithoutPlacing) {
    Json service = resetService();
    service["id"] = "arm";
    service["service_type"] = "std_srvs/srv/SetBool";
    service["action"] = "command:mechanisms.set_armed";
    service["request"] = {{"armed", {{"from", "data"}}}};
    Rig rig(withServices({service}));
    std_srvs::srv::SetBool::Request request;
    request.data = true;
    std_srvs::srv::SetBool::Response response;
    rig.core->call("arm", &request, &response);
    EXPECT_TRUE(response.success);
    EXPECT_EQ(rig.port.calls.back(), "set_armed:1");
    rig.port.next_result = {false, "rejected while killed"};
    rig.core->call("arm", &request, &response);
    EXPECT_FALSE(response.success);
    EXPECT_EQ(response.message, "rejected while killed");
}

// ------------------------------------------------------------------ mechanism topics and replies

TEST(Mechanisms, TopicCommandsReplyOnTheDeclaredStream) {
    Json robot = defaultRobot();
    robot["mechanisms"] = Json::array({{{"id", "claw"}, {"type", "claw"}, {"parameters", Json::object()}},
                                       {{"id", "torpedo"}, {"type", "launcher"}, {"parameters", Json::object()}}});
    Json bridge = defaultBridge();
    bridge["streams"].push_back(publishStream("cmd_status", "std_msgs/msg/Bool", "event:mechanisms.command_result",
                                               {{"data", {{"from", "accepted"}}}}, 0));
    bridge["streams"].push_back(subscribeStream("torpedo_topic", "std_msgs/msg/Empty", "command:mechanisms.torpedo.fire",
                                                Json::object(), {{"reply_stream", "cmd_status"}}));
    bridge["streams"].push_back(subscribeStream("claw_move", "std_msgs/msg/Float32", "command:mechanisms.claw.timed_move",
                                                {{"signed_duration_s", {{"from", "data"}}}}, {{"reply_stream", "cmd_status"}}));
    bridge["streams"].push_back(subscribeStream("claw_topic", "std_msgs/msg/Bool", "command:mechanisms.claw.command",
                                                {{"open", {{"from", "data"}}}}, {{"reply_stream", "cmd_status"}}));
    Rig rig(bridge, robot);
    rig.port.fire_events = {{{"type", "release"}}};
    auto replies = receive(rig, "torpedo_topic", makeMessage<std_msgs::msg::Empty>("std_msgs/msg/Empty"));
    ASSERT_EQ(replies.size(), 1u);
    EXPECT_EQ(replies[0].stream, "cmd_status");
    EXPECT_EQ(publicationJson(replies[0]).at("data"), true);
    EXPECT_EQ(rig.core->taskEvents().size(), 1u); // release events land in the run record
    auto move = makeMessage<std_msgs::msg::Float32>("std_msgs/msg/Float32");
    static_cast<std_msgs::msg::Float32 *>(move->data())->data = 1.5f;
    rig.port.next_result = {false, "disarmed"};
    replies = receive(rig, "claw_move", move);
    EXPECT_EQ(publicationJson(replies.at(0)).at("data"), false);
    EXPECT_EQ(rig.port.calls.back(), "move_claw:claw:1.500000");
    receive(rig, "claw_topic", boolMessage(true));
    EXPECT_EQ(rig.port.calls.back(), "claw:claw:open");
    // Wrong mechanism kinds fail at construction.
    bridge["streams"].push_back(subscribeStream("bad", "std_msgs/msg/Empty", "command:mechanisms.claw.fire", Json::object()));
    EXPECT_NE(bridgeError([&] { Rig bad(bridge, robot); }).find("is not a robot launcher/dropper mechanism"), std::string::npos);
}

// ------------------------------------------------------------------ static TF and alignment

TEST(StaticTf, StaticEdgesComeFromTheFrameTreeAndAreValidated) {
    Json bridge = defaultBridge();
    bridge["tf"] = {{"static", Json::array({{{"parent", "rig"}, {"child", "cam"}, {"from_frame", "com"}, {"to_frame", "base"}}})},
                    {"never_publish", Json::array({"forbidden*"})}};
    Rig rig(bridge);
    ASSERT_EQ(rig.core->staticTransforms().size(), 1u);
    EXPECT_EQ(rig.core->staticTransforms()[0].parent, "rig");
    EXPECT_TRUE(rig.core->staticTransforms()[0].translation.isApprox(kOffset));

    Json duplicate = bridge;
    duplicate["tf"]["static"].push_back(duplicate["tf"]["static"][0]);
    EXPECT_NE(bridgeError([&] { Rig r(duplicate); }).find("duplicate child/owner 'cam'"), std::string::npos);
    Json cycle = bridge;
    cycle["tf"]["static"] = Json::array({{{"parent", "a"}, {"child", "b"}, {"from_frame", "com"}, {"to_frame", "base"}},
                                         {{"parent", "b"}, {"child", "a"}, {"from_frame", "com"}, {"to_frame", "base"}}});
    EXPECT_NE(bridgeError([&] { Rig r(cycle); }).find("tf: cycle"), std::string::npos);
    Json forbidden = bridge;
    forbidden["tf"]["static"][0]["child"] = "forbidden_child";
    EXPECT_NE(bridgeError([&] { Rig r(forbidden); }).find("never_publish"), std::string::npos);
    Json unknown = bridge;
    unknown["tf"]["static"][0]["to_frame"] = "missing";
    EXPECT_NE(bridgeError([&] { Rig r(unknown); }).find("invalid static robot frames"), std::string::npos);
    Json names = bridge;
    names["frame_names"]["base"] = "other";
    names["tf"]["static"][0]["to_frame"] = "base";
    names["tf"]["static"][0]["child"] = "cam";
    EXPECT_NE(bridgeError([&] { Rig r(names); }).find("differs from frame_names['base']"), std::string::npos);
    Json truth = bridge;
    truth["tf"]["publish"] = Json::array({{{"native", "state:robot.reference_pose"}, {"parent", "not_world"}, {"child", "x"}, {"rate_hz", 100}}});
    EXPECT_NE(bridgeError([&] { Rig r(truth); }).find("truth transforms must have parent"), std::string::npos);
}

TEST(StaticTf, TruthEdgesMirrorFramesUnderAPublishedTruthFrame) {
    Json bridge = defaultBridge();
    bridge["frame_names"]["base"] = "vehicle/mount";
    bridge["tf"] = {{"publish", Json::array({{{"native", "state:robot.reference_pose"}, {"parent", bridge["frame_names"]["world"]},
                                              {"child", "sim/root"}, {"rate_hz", 10}}})},
                    {"static", Json::array({{{"parent", "sim/root"}, {"child", "sim/mount"}, {"from_frame", "com"},
                                             {"to_frame", "base"}, {"truth", true}}})}};
    Rig rig(bridge); // same robot frame as frame_names['base'] under another ROS name: allowed
    ASSERT_EQ(rig.core->staticTransforms().size(), 1u);
    EXPECT_EQ(rig.core->staticTransforms()[0].child, "sim/mount");
    EXPECT_TRUE(rig.core->staticTransforms()[0].translation.isApprox(kOffset));
    Json detached = bridge;
    detached["tf"]["static"][0]["parent"] = "elsewhere";
    EXPECT_NE(bridgeError([&] { Rig r(detached); }).find("must descend from a tf.publish frame"), std::string::npos);
}

namespace {
Json alignmentBridge() {
    Json bridge = defaultBridge();
    bridge["streams"].push_back(subscribeStream(
        "estimate", "nav_msgs/msg/Odometry", "estimate:latest",
        {{"frame", {{"from", "header.frame_id"}}}, {"position_m", {{"from", "pose.pose.position"}}},
         {"orientation_w", {{"from", "pose.pose.orientation.w"}}}, {"orientation_x", {{"from", "pose.pose.orientation.x"}}},
         {"orientation_y", {{"from", "pose.pose.orientation.y"}}}, {"orientation_z", {{"from", "pose.pose.orientation.z"}}}}));
    bridge["placement"] = {{"estimator_alignment", {{"client", "/demo/set_pose"},
                                                   {"service_type", "robot_localization/srv/SetPose"},
                                                   {"triggers", Json::array({"startup", "placement", "reset_to_start", "full_reset"})},
                                                   {"estimate_stream", "estimate"}, {"pose", "reference_frame"},
                                                   {"covariance_diagonal", 1e-6}}}};
    return withServices({setPoseService(true, true), resetService()}, bridge);
}
} // namespace

TEST(Alignment, PlacementSupersedesUnsentStartupAndUsesTheLatestPose) {
    Rig rig(alignmentBridge(), defaultRobot(), {"alt"},
            [](const std::string &, const std::string &) { return std::optional<spatial::Pose>(spatial::Pose{}); });
    EXPECT_FALSE(rig.core->pendingAlignment().has_value()); // no estimate frame yet
    auto request = poseRequest("map", {8.0, 8.0, -1.0});
    robot_localization::srv::SetPose::Response response;
    rig.core->call("set_pose", &request, &response);
    EXPECT_EQ(rig.core->counters().alignments_superseded, (CounterTable{{"startup", 1}}));
    auto estimate = makeMessage<nav_msgs::msg::Odometry>("nav_msgs/msg/Odometry");
    auto &odometry = *static_cast<nav_msgs::msg::Odometry *>(estimate->data());
    odometry.header.frame_id = "odom";
    odometry.pose.pose.orientation.w = 1.0;
    receive(rig, "estimate", estimate);
    const auto alignment = rig.core->pendingAlignment();
    ASSERT_TRUE(alignment.has_value());
    EXPECT_EQ(alignment->trigger, "placement");
    EXPECT_EQ(alignment->frame, "odom");
    EXPECT_NEAR(alignment->position.x(), 8.0, 1e-12);
    EXPECT_NEAR(alignment->position.z(), -1.0, 1e-12);
    EXPECT_EQ(alignment->covariance_diagonal, 1e-6);
    EXPECT_EQ(rig.core->counters().alignments, (CounterTable{{"placement", 1}}));
    EXPECT_FALSE(rig.core->pendingAlignment().has_value()); // never resends an old pose
}

TEST(Alignment, ResetAndFullResetRequestAlignmentAndBadDeclarationsRaise) {
    Rig rig(alignmentBridge());
    std_srvs::srv::Trigger::Request trigger;
    std_srvs::srv::Trigger::Response response;
    rig.core->call("reset", &trigger, &response);
    EXPECT_EQ(rig.core->counters().alignments_superseded, (CounterTable{{"startup", 1}}));
    Json bad = alignmentBridge();
    bad["placement"]["estimator_alignment"]["triggers"] = Json::array({"whenever"});
    EXPECT_NE(bridgeError([&] { Rig r(bad); }).find("unknown trigger"), std::string::npos);
    bad = alignmentBridge();
    bad["placement"]["estimator_alignment"]["estimate_stream"] = "thruster_cmd";
    EXPECT_NE(bridgeError([&] { Rig r(bad); }).find("estimate:latest subscription"), std::string::npos);
    bad = alignmentBridge();
    bad["placement"]["estimator_alignment"]["service_type"] = "std_srvs/srv/Trigger";
    EXPECT_NE(bridgeError([&] { Rig r(bad); }).find("not supported by this bridge"), std::string::npos);
}

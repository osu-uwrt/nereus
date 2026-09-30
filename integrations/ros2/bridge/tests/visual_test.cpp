// Viewer-facing encoders: JSON streams and MarkerArray streams
// driven by pack data. Uses the fake session; no middleware.
#include "fake_port.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

#include <gtest/gtest.h>

using namespace bridge_test;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace {
Json robotWithMechanisms() {
    Json robot = defaultRobot();
    robot["mechanisms"] =
        Json::array({{{"id", "torpedo_launcher"}, {"type", "launcher"}, {"parameters", Json::object()}},
                     {{"id", "dropper"}, {"type", "dropper"}, {"parameters", Json::object()}}});
    return robot;
}
Json marker(const std::string &id, const std::string &topic, const std::string &native, Json options,
            const std::string &frame = kWorld) {
    return {{"id", id},
            {"direction", "publish"},
            {"topic", "/demo/" + topic},
            {"message_type", "visualization_msgs/msg/MarkerArray"},
            {"native", native},
            {"format", "marker_array"},
            {"fields", Json::object()},
            {"rate_hz", 50},
            {"frame_id", frame},
            {"qos", qos()},
            {"options", options}};
}
session::ResolvedScenario scenarioWith(const std::vector<Json> &streams, const Json &robot = robotWithMechanisms()) {
    Json bridge = defaultBridge();
    bridge["streams"] = Json::array();
    for (const auto &stream : streams)
        bridge["streams"].push_back(stream);
    Json document = {
        {"format", "nereus.resolved_scenario"},
        {"version", 1},
        {"scenario", {{"world_frame", "scenario_world"}, {"seed", 0}}},
        {"robot", robot},
        {"pool", Json::object()},
        {"tasks", Json::object()},
        {"task_definitions",
         Json::array(
             {{{"id", "table"},
               {"props",
                Json::array(
                    {{{"id", "crate"}, {"type", "rigid_body"}, {"parameters", {{"visual_asset", "crate_mesh"}}}},
                     {{"id", "ghost"}, {"type", "rigid_body"}, {"parameters", Json::object()}}})}}})},
        {"bridge", bridge},
        {"run_options", Json::object()},
        {"asset_paths",
         {{"tasks", {{"crate_mesh", "/assets/crate.obj"}}}, {"robot", {{"projectile_mesh", "/assets/torpedo.obj"}}}}}};
    return session::parseResolvedScenario(document);
}
template <class Fn> std::string bridgeError(Fn &&fn) {
    try {
        fn();
    } catch (const BridgeError &error) {
        return error.what();
    }
    return "<no error>";
}
const MarkerArray &markers(const Publication &publication) {
    return *static_cast<const MarkerArray *>(publication.message->data());
}
std::vector<Publication> stepUntil(BridgeCore &core, const std::string &stream, int ticks = 5) {
    std::vector<Publication> out;
    for (int k = 0; k < ticks; ++k)
        for (auto &item : byStream(core.step().publications, stream))
            out.push_back(item);
    return out;
}
} // namespace

TEST(Viewer, MagnetLightsUseLatchedColorsFromTheIndicatorState) {
    const auto resolved = scenarioWith({marker("lights", "lights", "state:indicators",
                                               {{"shape", "sphere"},
                                                {"scale_m", {0.002, 0.044, 0.044}},
                                                {"colors", {{"red", {1, 0, 0, 1}}, {"green", {0, 1, 0, 1}}}}})});
    FakePort port;
    port.indicator_list = Json::array({{{"task", "table"},
                                        {"region", "magnet_a"},
                                        {"position_m", {1.0, 2.0, 3.0}},
                                        {"orientation_wxyz", {1.0, 0.0, 0.0, 0.0}},
                                        {"latched", false},
                                        {"colors", {{"initial", "red"}, {"latched", "green"}}}},
                                       {{"task", "table"},
                                        {"region", "magnet_b"},
                                        {"position_m", {4.0, 5.0, 6.0}},
                                        {"orientation_wxyz", {0.0, 0.0, 0.0, 1.0}},
                                        {"latched", true},
                                        {"colors", {{"initial", "red"}, {"latched", "green"}}}}});
    BridgeCore core(resolved, port, kEpochNs);
    const auto out = stepUntil(core, "lights", 10);
    ASSERT_EQ(out.size(), 1u);
    const auto &list = markers(out[0]).markers;
    ASSERT_EQ(list.size(), 2u);
    EXPECT_EQ(list[0].ns, "magnet_a");
    EXPECT_EQ(list[0].id, 0);
    EXPECT_EQ(list[0].type, Marker::SPHERE);
    EXPECT_FLOAT_EQ(list[0].color.r, 1.0f);
    EXPECT_FLOAT_EQ(list[1].color.g, 1.0f);
    EXPECT_EQ(list[1].header.frame_id, kWorld);
    EXPECT_DOUBLE_EQ(list[1].pose.position.y, 5.0);
    EXPECT_DOUBLE_EQ(list[1].pose.orientation.z, 1.0);
    EXPECT_DOUBLE_EQ(list[0].scale.y, 0.044);
    EXPECT_EQ(list[0].header.stamp.sec * 1'000'000'000LL + list[0].header.stamp.nanosec,
              static_cast<std::int64_t>(kEpochNs + 10 * kStepNs));
}

TEST(Viewer, IndicatorOptionsAreValidatedAgainstThePackState) {
    FakePort port;
    port.indicator_list = Json::array({{{"task", "t"},
                                        {"region", "m"},
                                        {"position_m", {0, 0, 0}},
                                        {"orientation_wxyz", {1, 0, 0, 0}},
                                        {"latched", false},
                                        {"colors", {{"initial", "red"}, {"latched", "blue"}}}}});
    const auto make = [&](Json options) {
        const auto resolved = scenarioWith({marker("lights", "lights", "state:indicators", options)});
        BridgeCore core(resolved, port, kEpochNs);
    };
    EXPECT_NE(bridgeError([&] {
                  make({{"shape", "sphere"}, {"scale_m", {1, 1, 1}}, {"colors", {{"red", {1, 0, 0, 1}}}}});
              }).find("no color for ['blue'] of indicator 'm'"),
              std::string::npos);
    EXPECT_NE(bridgeError([&] {
                  make({{"shape", "cube"}, {"scale_m", {1, 1, 1}}, {"colors", Json::object()}});
              }).find("only 'sphere'"),
              std::string::npos);
    EXPECT_NE(bridgeError([&] {
                  make({{"shape", "sphere"}, {"scale_m", {1, 1}}, {"colors", Json::object()}});
              }).find("must be 3 finite numbers"),
              std::string::npos);
    EXPECT_NE(bridgeError([&] { make({{"shape", "sphere"}}); }).find("missing ['colors', 'scale_m']"),
              std::string::npos);
}

TEST(Viewer, HeldPropsAreExpressedInTheHeldFrame) {
    const auto resolved =
        scenarioWith({marker("objects", "objects", "state:props", {{"held_frame_id", "robot/base_link"}})});
    FakePort port;
    PropVisual free_prop{"table", "crate", {5.0, 0.0, -1.0}, Eigen::Quaterniond::Identity(), false};
    PropVisual held{
        "table", "crate", {1.1, 2.0, 2.95}, Eigen::Quaterniond::Identity(), true}; // at the reference origin
    PropVisual no_mesh{"table", "ghost", {0, 0, 0}, Eigen::Quaterniond::Identity(), false};
    port.props = {free_prop, no_mesh, held};
    BridgeCore core(resolved, port, kEpochNs);
    const auto out = stepUntil(core, "objects", 10);
    ASSERT_EQ(out.size(), 1u);
    const auto &list = markers(out[0]).markers;
    ASSERT_EQ(list.size(), 2u); // the prop without a display asset is skipped
    EXPECT_EQ(list[0].type, Marker::MESH_RESOURCE);
    EXPECT_EQ(list[0].mesh_resource, "file:///assets/crate.obj");
    EXPECT_TRUE(list[0].mesh_use_embedded_materials);
    EXPECT_EQ(list[0].header.frame_id, kWorld);
    EXPECT_EQ(list[0].id, 0);
    EXPECT_EQ(list[1].header.frame_id, "robot/base_link");
    EXPECT_EQ(list[1].id, 2); // the index counts the skipped prop
    EXPECT_NEAR(list[1].pose.position.x, 0.0, 1e-9);
    EXPECT_NEAR(list[1].pose.position.z, 0.0, 1e-9);
}

TEST(Viewer, PayloadsUseNamespacesLoadedSuffixAndDeleteAll) {
    const Json options = {{"namespaces", {{"launcher", "torpedo"}, {"dropper", "dropper"}}},
                          {"loaded_suffix", "_loaded"},
                          {"mesh_asset", "projectile_mesh"},
                          {"color", {0.65, 0.025, 0.035, 1.0}},
                          {"delete_all", true}};
    const auto resolved = scenarioWith({marker("projectiles", "projectiles", "state:payloads", options)});
    FakePort port;
    port.payloads = {{"torpedo_launcher", "launcher", 3, false, {1, 2, 3}, Eigen::Quaterniond::Identity(), 0.4, 0.05},
                     {"dropper", "dropper", 0, true, {0, 0, 0}, Eigen::Quaterniond::Identity(), 0.2, 0.03}};
    BridgeCore core(resolved, port, kEpochNs);
    const auto out = stepUntil(core, "projectiles", 10);
    ASSERT_EQ(out.size(), 1u);
    const auto &list = markers(out[0]).markers;
    ASSERT_EQ(list.size(), 3u);
    EXPECT_EQ(list[0].action, Marker::DELETEALL);
    EXPECT_EQ(list[1].ns, "torpedo");
    EXPECT_EQ(list[1].id, 3);
    EXPECT_DOUBLE_EQ(list[1].scale.x, 0.4);
    EXPECT_DOUBLE_EQ(list[1].scale.y, 0.1);
    EXPECT_EQ(list[2].ns, "dropper_loaded");
    EXPECT_FALSE(list[2].mesh_use_embedded_materials);
    Json missing = options;
    missing["namespaces"] = {{"launcher", "torpedo"}};
    EXPECT_NE(bridgeError([&] {
                  BridgeCore c(scenarioWith({marker("p", "p", "state:payloads", missing)}), port, kEpochNs);
              }).find("no namespace for mechanism types ['dropper']"),
              std::string::npos);
    Json asset = options;
    asset["mesh_asset"] = "absent";
    EXPECT_NE(bridgeError([&] {
                  BridgeCore c(scenarioWith({marker("p", "p", "state:payloads", asset)}), port, kEpochNs);
              }).find("asset 'absent' is not present in the robot pack"),
              std::string::npos);
}

TEST(Viewer, FormatStreamsAreValidated) {
    FakePort port;
    const auto build = [&](const Json &stream) { BridgeCore core(scenarioWith({stream}), port, kEpochNs); };
    Json wrong_frame = marker("m", "m", "state:props", {{"held_frame_id", "x"}}, "elsewhere");
    EXPECT_NE(bridgeError([&] { build(wrong_frame); }).find("marker frame_id must be the world frame"),
              std::string::npos);
    Json wrong_format = marker("m", "m", "state:run", Json::object());
    EXPECT_NE(bridgeError([&] { build(wrong_format); }).find("cannot use format 'marker_array'"), std::string::npos);
    Json json_stream = {{"id", "j"},
                        {"direction", "publish"},
                        {"topic", "/j"},
                        {"message_type", "std_msgs/msg/String"},
                        {"native", "state:run"},
                        {"format", "json"},
                        {"fields", {{"data", {{"constant", "x"}}}}},
                        {"rate_hz", 50},
                        {"frame_id", ""},
                        {"qos", qos()}};
    EXPECT_NE(bridgeError([&] { build(json_stream); }).find("take no field map"), std::string::npos);
    json_stream["fields"] = Json::object();
    json_stream["frame_id"] = "map";
    EXPECT_NE(bridgeError([&] { build(json_stream); }).find("json streams are unstamped"), std::string::npos);
    json_stream["frame_id"] = "";
    json_stream["message_type"] = "std_msgs/msg/Bool";
    EXPECT_NE(bridgeError([&] { build(json_stream); }).find("string 'data' field"), std::string::npos);
    json_stream["message_type"] = "std_msgs/msg/String";
    json_stream["native"] = "event:tasks.feed";
    json_stream["rate_hz"] = 5;
    EXPECT_NE(bridgeError([&] { build(json_stream); }).find("event streams have rate_hz 0"), std::string::npos);
    json_stream["direction"] = "subscribe";
    EXPECT_NE(bridgeError([&] { build(json_stream); }).find("format/options apply to publish streams"),
              std::string::npos);
}

TEST(Viewer, ScenarioDescriptionIsPublishedOnceAsTheResolvedDocument) {
    Json stream = {
        {"id", "scenario"},
        {"direction", "publish"},
        {"topic", "/scenario"},
        {"message_type", "std_msgs/msg/String"},
        {"native", "event:scenario.description"},
        {"format", "json"},
        {"fields", Json::object()},
        {"rate_hz", 0},
        {"frame_id", ""},
        {"qos",
         {{"reliability", "reliable"}, {"durability", "transient_local"}, {"history", "keep_last"}, {"depth", 1}}}};
    const auto resolved = scenarioWith({stream});
    FakePort port;
    BridgeCore core(resolved, port, kEpochNs);
    const auto startup = core.startupPublications();
    ASSERT_EQ(startup.size(), 1u);
    const Json parsed = Json::parse(publicationJson(startup[0]).at("data").get<std::string>());
    EXPECT_EQ(parsed, resolved.document);
    EXPECT_EQ(parsed.at("asset_paths").at("tasks").at("crate_mesh"), "/assets/crate.obj");
    EXPECT_TRUE(core.startupPublications().size() == 1u); // callable again; the node calls it once
}

TEST(Viewer, PausedRefreshSkipsRobotTruthAndSensors) {
    Json thrusters = {{"id", "forces"},
                      {"direction", "publish"},
                      {"topic", "/f"},
                      {"message_type", "std_msgs/msg/Float32MultiArray"},
                      {"native", "state:thrusters"},
                      {"fields", {{"data", {{"from", "forces_n"}}}}},
                      {"rate_hz", 100},
                      {"frame_id", ""},
                      {"qos", qos()}};
    Json truth = publishStream("truth", "std_msgs/msg/UInt8", "state:robot", {{"data", {{"constant", 1}}}}, 100);
    const auto resolved = scenarioWith({thrusters, truth});
    FakePort port;
    BridgeCore core(resolved, port, kEpochNs);
    const auto refreshed = core.refresh();
    ASSERT_EQ(refreshed.size(), 1u);
    EXPECT_EQ(refreshed[0].stream, "forces");
    // Realized native forces (4, -6, 2) are reported in the thrusters.order layout [c, a, b] / scales [2, 1, -1].
    const auto data = publicationJson(refreshed[0]).at("data");
    ASSERT_EQ(data.size(), 3u);
    EXPECT_DOUBLE_EQ(data[0].get<double>(), 1.0); // c: 2 / 2
    EXPECT_DOUBLE_EQ(data[1].get<double>(), 4.0); // a: 4 / 1
    EXPECT_DOUBLE_EQ(data[2].get<double>(), 6.0); // b: -6 / -1
}

// The bridge core over the real Session with the Talos/UWRT packs (port of test_bridge_mechanisms.py
// and the placement/reset parts of test_bridge_review.py). Camera streams are removed like
// --no-cameras; no middleware.
#include "session_adapter.hpp"

#include "core.hpp"

#include <rules/registry.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <gtest/gtest.h>

using namespace robotics;
using namespace robotics::ros_bridge;

namespace {
session::ResolvedScenario withoutCameras(session::ResolvedScenario resolved) {
    std::set<std::string> cameras;
    for (const auto &sensor : resolved.robot.at("sensors"))
        if (sensor.at("type") == "stereo_camera")
            cameras.insert(sensor.at("id").get<std::string>());
    Json streams = Json::array();
    for (const auto &stream : resolved.bridge.at("streams")) {
        const std::string native = stream.at("native");
        const bool camera = native.rfind("sensor:", 0) == 0 &&
                            cameras.count(native.substr(7, native.find('.') == std::string::npos ? std::string::npos
                                                                                                   : native.find('.') - 7));
        if (!stream.contains("image") && !camera)
            streams.push_back(stream);
    }
    resolved.bridge["streams"] = streams;
    return resolved;
}

struct Talos {
    session::ResolvedScenario resolved = withoutCameras(session::loadResolvedScenario(RP_RESOLVED_TALOS));
    std::vector<std::string> native_ids = [this] {
        std::vector<std::string> ids;
        for (const auto &sensor : resolved.robot.at("sensors"))
            if (sensor.at("type") != "stereo_camera" && sensor.value("enabled", true))
                ids.push_back(sensor.at("id").get<std::string>());
        return ids;
    }();
    SessionAdapter adapter{resolved, rules::standardRules(), &native_ids};
    BridgeCore core{resolved, adapter, 1'000'000'000'000'000'000LL};

    std::vector<Publication> run(int ticks) {
        std::vector<Publication> out;
        for (int k = 0; k < ticks; ++k)
            for (auto &item : core.step().publications)
                out.push_back(std::move(item));
        return out;
    }
    std::pair<bool, std::string> trigger(const std::string &service) {
        std_srvs::srv::Trigger::Request request;
        std_srvs::srv::Trigger::Response response;
        core.call(service, &request, &response);
        return {response.success, response.message};
    }
    std::pair<bool, std::string> setBool(const std::string &service, bool value) {
        std_srvs::srv::SetBool::Request request;
        request.data = value;
        std_srvs::srv::SetBool::Response response;
        core.call(service, &request, &response);
        return {response.success, response.message};
    }
    template <class T> std::vector<Publication> send(const std::string &stream, const char *type, void (*fill)(T &)) {
        Message message(MessageType::get(type));
        fill(*static_cast<T *>(message.data()));
        return core.receive(stream, message.data());
    }
    void unkill() {
        Message message(MessageType::get("riptide_msgs2/msg/KillSwitchReport"));
        const auto *members = message.type().members();
        *static_cast<std::uint8_t *>(locate(message.data(), resolveField(members, "kill_switch_id"))) = 1;
        *static_cast<bool *>(locate(message.data(), resolveField(members, "switch_asserting_kill"))) = false;
        core.receive("software_kill", message.data());
    }
    void kill() {
        Message message(MessageType::get("riptide_msgs2/msg/KillSwitchReport"));
        const auto *members = message.type().members();
        *static_cast<std::uint8_t *>(locate(message.data(), resolveField(members, "kill_switch_id"))) = 1;
        *static_cast<bool *>(locate(message.data(), resolveField(members, "switch_asserting_kill"))) = true;
        core.receive("software_kill", message.data());
    }
};

Json statusOf(const Publication &p) { return messageToJson(p.message->type().members(), p.message->data()); }
std::vector<Json> collect(const std::vector<Publication> &all, const std::string &stream) {
    std::vector<Json> out;
    for (const auto &item : all)
        if (item.stream == stream)
            out.push_back(statusOf(item));
    return out;
}
} // namespace

TEST(TalosBridge, ArmingFollowsKillAndStatusIsPublishedAt50Hz) {
    Talos talos;
    EXPECT_TRUE(talos.core.killed());
    EXPECT_FALSE(talos.setBool("arm", true).first); // rejected while killed
    talos.unkill();
    EXPECT_TRUE(talos.setBool("arm", true).first);
    const auto status = collect(talos.run(50), "actuator_status");
    ASSERT_EQ(status.size(), 5u);
    const Json &last = status.back();
    EXPECT_TRUE(last["actuators_armed"].get<bool>());
    EXPECT_EQ(last["torpedo_state"], 3);
    EXPECT_EQ(last["torpedo_available_count"], 2);
    EXPECT_EQ(last["dropper_state"], 2);
    EXPECT_EQ(last["dropper_available_count"], 2);
    EXPECT_EQ(last["claw_state"], 4);
    talos.kill(); // kill disarms (robot safety.kill_disarms_mechanisms)
    EXPECT_FALSE(collect(talos.run(10), "actuator_status").back()["actuators_armed"].get<bool>());
}

TEST(TalosBridge, FireSharesCooldownAndReloadDisarms) {
    Talos talos;
    talos.unkill();
    talos.setBool("arm", true);
    talos.run(1);
    EXPECT_TRUE(talos.trigger("fire_torpedo").first);
    EXPECT_FALSE(talos.trigger("fire_dropper").first); // shared release group busy
    const auto busy = collect(talos.run(10), "actuator_busy");
    ASSERT_EQ(busy.size(), 1u);
    EXPECT_TRUE(busy[0]["data"].get<bool>());
    talos.run(250);
    EXPECT_TRUE(talos.trigger("fire_dropper").first);
    EXPECT_EQ(talos.core.taskEvents().empty(), false); // release events reach the run record
    EXPECT_TRUE(talos.trigger("reload").first);
    const auto state = talos.adapter.mechanismState();
    ASSERT_TRUE(state.has_value());
    EXPECT_FALSE(state->armed);
    EXPECT_EQ(state->releases.at("torpedo_launcher").available, 2);
}

TEST(TalosBridge, ClawTopicCommandsReplyAndMoveTheJaws) {
    Talos talos;
    const auto send = [&](float seconds) {
        Message message(MessageType::get("std_msgs/msg/Float32"));
        static_cast<std_msgs::msg::Float32 *>(message.data())->data = seconds;
        return talos.core.receive("claw_timed_move", message.data());
    };
    auto replies = send(1.0f);
    ASSERT_EQ(replies.size(), 1u);
    EXPECT_EQ(replies[0].stream, "actuator_cmd_status");
    EXPECT_FALSE(statusOf(replies[0])["data"].get<bool>()); // disarmed and killed
    talos.unkill();
    talos.setBool("arm", true);
    replies = send(1.0f);
    EXPECT_TRUE(statusOf(replies.at(0))["data"].get<bool>());
    talos.run(600);
    const auto claw = talos.adapter.mechanismState()->claws.at("claw");
    EXPECT_NEAR(claw.gap_m, 0.0012 + 0.066, 1e-3); // 1 s at 0.033 m/s per jaw
    EXPECT_TRUE(talos.setBool("claw", false).first);
    talos.run(2000);
    EXPECT_EQ(talos.adapter.mechanismState()->claws.at("claw").state, "closed");
}

TEST(TalosBridge, TopicFormCommandsReplyOnCmdStatus) {
    Talos talos;
    talos.unkill();
    const auto boolean = [&](const char *stream, bool value) {
        Message message(MessageType::get("std_msgs/msg/Bool"));
        static_cast<std_msgs::msg::Bool *>(message.data())->data = value;
        return talos.core.receive(stream, message.data());
    };
    const auto empty = [&](const char *stream) {
        Message message(MessageType::get("std_msgs/msg/Empty"));
        return talos.core.receive(stream, message.data());
    };
    auto replies = boolean("arm_topic", true);
    ASSERT_EQ(replies.size(), 1u);
    EXPECT_TRUE(statusOf(replies[0])["data"].get<bool>());
    talos.run(1);
    EXPECT_TRUE(statusOf(empty("torpedo_topic").at(0))["data"].get<bool>());
    EXPECT_FALSE(statusOf(empty("dropper_topic").at(0))["data"].get<bool>()); // cooldown
    EXPECT_TRUE(statusOf(boolean("claw_topic", true).at(0))["data"].get<bool>());
    EXPECT_TRUE(statusOf(empty("reload_topic").at(0))["data"].get<bool>());
    EXPECT_FALSE(talos.adapter.mechanismState()->armed);
}

TEST(TalosBridge, TaskResetClearsPayloadsReloadsAndDisarms) {
    Talos talos;
    talos.unkill();
    talos.setBool("arm", true);
    talos.run(1);
    talos.trigger("fire_torpedo");
    talos.run(10);
    EXPECT_TRUE(talos.trigger("reset_tasks").first);
    EXPECT_FALSE(talos.adapter.mechanismState()->armed);
    EXPECT_GT(talos.adapter.timeNs(), 0); // physics time is untouched
}

TEST(TalosBridge, FullResetRestoresStartWithoutRewindingRosTime) {
    Talos talos;
    talos.unkill();
    Message forces(MessageType::get("std_msgs/msg/Float32MultiArray"));
    static_cast<std_msgs::msg::Float32MultiArray *>(forces.data())->data = std::vector<float>(8, 20.0f);
    talos.core.receive("thruster_forces", forces.data());
    talos.run(500);
    const Eigen::Vector3d moved = talos.adapter.observe().body.position;
    const auto before = talos.core.clockNs();
    const auto reply = talos.trigger("reset_scenario");
    EXPECT_TRUE(reply.first) << reply.second;
    const auto snapshot = talos.adapter.observe();
    EXPECT_EQ(snapshot.elapsed.count(), 0);
    EXPECT_FALSE(moved.isApprox(snapshot.body.position));
    EXPECT_TRUE(snapshot.body.position.isApprox(talos.adapter.startState().position));
    EXPECT_TRUE(talos.core.killed()); // robot safety initially_killed
    const auto clocks = talos.core.step().clocks;
    ASSERT_FALSE(clocks.empty());
    EXPECT_GT(clocks[0], before);
}

TEST(TalosBridge, FullResetReplaysIdenticalSensorNoise) {
    const auto record = [](Talos &talos) {
        std::vector<double> samples;
        for (const auto &item : collect(talos.run(200), "imu"))
            samples.push_back(item["linear_acceleration"]["x"].get<double>());
        return samples;
    };
    Talos talos;
    const auto first = record(talos);
    ASSERT_FALSE(first.empty());
    talos.trigger("reset_scenario");
    EXPECT_EQ(record(talos), first);
}

TEST(TalosBridge, ExecutionCoversEveryDeclaredStreamAndPublishesOnSchedule) {
    Talos talos;
    const auto all = talos.run(500); // 1 s
    std::map<std::string, int> counts;
    for (const auto &item : all)
        ++counts[item.stream];
    EXPECT_EQ(counts["imu"], 50);
    EXPECT_EQ(counts["fog"], 500);
    EXPECT_EQ(counts["dvl"], 8);
    EXPECT_EQ(counts["depth"], 20);
    EXPECT_EQ(counts["ground_truth"], 100);
    EXPECT_EQ(counts["run_score"], 50);
    EXPECT_EQ(counts["task_score"], 50);
    EXPECT_EQ(counts["actuator_status"], 50);
    EXPECT_EQ(counts["actual_thruster_forces"], 100);
    EXPECT_EQ(counts["claw_joints"], 50);
    EXPECT_EQ(counts["thruster_telemetry_0_3"], 2);
    const auto scenario = talos.core.startupPublications();
    ASSERT_EQ(scenario.size(), 1u);
    EXPECT_EQ(Json::parse(statusOf(scenario[0])["data"].get<std::string>()), talos.resolved.document);
    const auto run = collect(all, "run_score");
    EXPECT_TRUE(Json::parse(run.back()["data"].get<std::string>()).contains("rows"));
}

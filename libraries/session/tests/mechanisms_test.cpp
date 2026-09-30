// Mechanisms equivalence with python mechanisms.Mechanisms on the Talos robot pack: the scripted
// op list (arm/kill/fire/claw/reload/reset/advance) captured from Python is replayed here and every
// command result, released-body state and mechanism snapshot is compared (kTrajectoryTolerance).
#include "session_test_util.hpp"

#include <robotics/session/mechanisms.hpp>

#include <gtest/gtest.h>

using namespace robotics::session;
using namespace robotics::session::testing;

TEST(Mechanisms, ReplaysThePythonScript) {
    const auto fixture = loadFixture("mechanisms_reference.json");
    const auto scenario = loadResolvedScenario(NEREUS_RESOLVED_TALOS);
    Mechanisms m(scenario.robot);
    const auto &p = fixture.at("pose");
    Pose pose{Eigen::Vector3d(p.at("translation")[0], p.at("translation")[1], p.at("translation")[2]),
              Eigen::Quaterniond(p.at("orientation")[0], p.at("orientation")[1], p.at("orientation")[2],
                                 p.at("orientation")[3])};
    const auto vec = [](const Json &j) { return Eigen::Vector3d(j[0], j[1], j[2]); };
    const Eigen::Vector3d lin = vec(fixture.at("linear")), ang = vec(fixture.at("angular"));
    const std::int64_t dt = fixture.at("dt_ns");
    bool killed = true;
    const auto &ops = fixture.at("ops");
    const auto &log = fixture.at("log");
    for (std::size_t i = 0; i < ops.size(); ++i) {
        const auto &op = ops[i];
        const std::string name = op[0];
        Json actual = Json::object();
        if (name == "arm") {
            actual["result"] = resultJson(m.setArmed(op[1], killed));
        } else if (name == "fire") {
            PayloadRelease release;
            const auto r = m.fire(op[1], pose, lin, ang, "base_link", 998.2, killed, release);
            actual["result"] = resultJson(r);
            if (r.accepted)
                actual["result"]["release"] = {
                    {"slot_id", release.slot_id}, {"slot_index", release.slot_index}, {"time_ns", release.time_ns},
                    {"position", flat(release.position_world)}, {"orientation", quat(release.orientation_world)},
                    {"velocity", flat(release.velocity_com_world)},
                    {"angular_velocity", flat(release.angular_velocity_world)}};
        } else if (name == "claw") {
            actual["result"] = resultJson(m.commandClaw(op[1], op[2], killed));
        } else if (name == "move_claw") {
            actual["result"] = resultJson(m.moveClaw(op[1], op[2], killed));
        } else if (name == "advance") {
            for (int n = 0; n < op[1].get<int>(); ++n) m.advance(dt, killed);
        } else if (name == "kill" || name == "unkill") {
            killed = name == "kill";
            m.advance(0, killed);
        } else if (name == "reload") {
            actual["result"] = resultJson(m.reloadAll(killed));
        } else if (name == "reset") {
            m.reset(killed);
        }
        actual["state"] = mechJson(m.snapshot(killed));
        ASSERT_EQ(diff(log[i], actual, "$.log[" + std::to_string(i) + "] " + op.dump()), "");
    }
    for (const auto &[id, expected] : fixture.at("mounts").items()) {
        ASSERT_EQ(m.slotCount(id), static_cast<int>(expected.size()));
        for (int k = 0; k < m.slotCount(id); ++k) {
            const auto mount = m.slotMount(id, k);
            Json actual = {{"translation", flat(mount.translation)}, {"orientation", quat(mount.rotation)}};
            EXPECT_EQ(diff(expected[static_cast<std::size_t>(k)], actual, "$.mounts." + id), "");
        }
    }
    EXPECT_EQ(m.type("claw"), "claw");
    EXPECT_EQ(m.type("torpedo_launcher"), "launcher");
    EXPECT_EQ(m.ids().size(), 4u);
}

TEST(Mechanisms, RejectsInvalidData) {
    auto robot = loadResolvedScenario(NEREUS_RESOLVED_TALOS).robot;
    auto broken = robot;
    broken["mechanisms"][0]["parameters"]["capacity"] = 3; // does not match the two slots
    EXPECT_THROW(Mechanisms{broken}, std::invalid_argument);
    broken = robot;
    broken["mechanisms"][2]["parameters"]["min_gap_m"] = 0.2; // max <= min
    EXPECT_THROW(Mechanisms{broken}, std::invalid_argument);
    broken = robot;
    broken["safety"]["arming"]["applies_to"].push_back("ghost");
    EXPECT_THROW(Mechanisms{broken}, std::invalid_argument);
    broken = robot;
    broken["mechanisms"][3]["parameters"]["actuated"] = true;
    EXPECT_THROW(Mechanisms{broken}, std::invalid_argument);
    Mechanisms m(robot);
    EXPECT_THROW(m.advance(-1, false), std::invalid_argument);
}

// Session equivalence with python session.Session (tasks disabled): scripted commands are replayed
// and body state / payloads / mechanisms / sensors are compared every 50 ticks, plus results, feed,
// counters and the run_score document builder. Reports ticks/s of the C++ session.
#include "session_test_util.hpp"

#include <rules/registry.hpp>

#include <gtest/gtest.h>

#include <chrono>

using namespace robotics::session;
using namespace robotics::session::testing;

namespace robotics::session { // private to rp_session (src/run_score.cpp)
Json buildRunSnapshot(const Json &, const Json &, Json, std::int64_t, double, const std::string &);
}

namespace {
Json payloadsJson(const Session &s) {
    Json out = Json::array();
    for (const auto &p : s.payloads())
        out.push_back({{"id", p.id}, {"mechanism_id", p.mechanism_id}, {"mechanism_type", p.mechanism_type},
                       {"active", p.active}, {"outcome", p.outcome}, {"released_ns", p.released_ns},
                       {"position", flat(p.state.position)}, {"orientation", quat(p.state.orientation)},
                       {"velocity", flat(p.state.velocity)}, {"angular_velocity", flat(p.state.angular_velocity)}});
    return out;
}
Json checkpoint(Session &s, bool tasks) {
    Json jaws = Json::object();
    for (const auto &[k, v] : s.clawJaws())
        jaws[k] = Json::array({v[0], v[1]});
    Json extra = Json::object();
    if (tasks) {
        extra["run"] = *s.runSnapshot();
        extra["indicators"] = s.indicators();
    }
    Json result = {{"time_ns", s.timeNs()}, {"killed", s.killed()},
            {"body", bodyJson(s.lastStep().snapshot.body)}, {"forces", flat(s.thrusterForces())},
            {"payloads", payloadsJson(s)}, {"mechanisms", mechJson(*s.mechanismState())}, {"jaws", jaws},
            {"sensors", sensorsJson(s.runtime(), s.pack())}};
    result.update(extra);
    return result;
}

void replay(const std::string &fixture_name, bool tasks) {
    const auto fixture = loadFixture(fixture_name);
    // The Python reference scripts were captured with the run started at boot; the pack default
    // (operator "start") is covered by the Python run-control tests.
    auto scenario = loadResolvedScenario(RP_RESOLVED_TALOS);
    scenario.scenario["run"]["auto_start"] = true;
    const RulesRegistry rules = robotics::rules::standardRules();
    const auto sensors = sensorNames(scenario);
    std::vector<std::string> task_ids;
    if (fixture.contains("task_ids"))
        task_ids = fixture.at("task_ids").get<std::vector<std::string>>();
    Session s(scenario, createRuntime(scenario, &sensors), rules,
              SessionOptions{tasks ? &task_ids : nullptr, tasks});
    EXPECT_EQ(s.timestepNs(), fixture.at("timestep_ns").get<std::int64_t>());
    const auto &ops = fixture.at("ops");
    const auto &log = fixture.at("log");
    int tick = 0;
    for (std::size_t i = 0; i < ops.size(); ++i) {
        const auto &op = ops[i];
        const std::string name = op[0];
        const std::string where = "$.log[" + std::to_string(i) + "] " + op.dump();
        Json actual = Json::object();
        Json events = Json::array();
        if (name == "advance") {
            Json checkpoints = Json::array();
            for (int n = 0; n < op[1].get<int>(); ++n) {
                for (const auto &e : s.advance().task_events)
                    events.push_back(e);
                if (++tick % 50 == 0)
                    checkpoints.push_back(checkpoint(s, tasks));
                drain(s.runtime(), s.pack());
            }
            actual["checkpoints"] = checkpoints;
        } else if (name == "arm") {
            actual["result"] = resultJson(s.setArmed(op[1]));
        } else if (name == "kill" || name == "unkill") {
            s.setKilled(name == "kill");
        } else if (name == "thrusters") {
            Eigen::VectorXd f(static_cast<Eigen::Index>(op[1].size()));
            for (std::size_t k = 0; k < op[1].size(); ++k) f[static_cast<Eigen::Index>(k)] = op[1][k].get<double>();
            s.commandThrusters(f);
        } else if (name == "fire") {
            const auto r = s.fire(op[1]);
            actual["result"] = resultJson(r);
            if (r.accepted) { // release details come from the payload the session created
                const auto &p = s.payloads().back();
                actual["result"]["release"] = {{"slot_id", log[i]["result"]["release"]["slot_id"]},
                    {"slot_index", log[i]["result"]["release"]["slot_index"]}, {"time_ns", p.released_ns},
                    {"position", flat(p.state.position)}, {"orientation", quat(p.state.orientation)},
                    {"velocity", flat(p.state.velocity)}, {"angular_velocity", flat(p.state.angular_velocity)}};
            }
        } else if (name == "claw") {
            actual["result"] = resultJson(s.commandClaw(op[1], op[2]));
        } else if (name == "move_claw") {
            actual["result"] = resultJson(s.moveClaw(op[1], op[2]));
        } else if (name == "reload") {
            actual["result"] = resultJson(s.reloadAll());
        } else if (name == "place") {
            robotics::simulation::BodyState state;
            state.position = Eigen::Vector3d(op[1][0], op[1][1], op[1][2]);
            state.orientation = Eigen::Quaterniond(op[2][0], op[2][1], op[2][2], op[2][3]);
            actual["body"] = bodyJson(s.place(state, op[3]).body);
        } else if (name == "place_moving") {
            robotics::simulation::BodyState state;
            state.position = Eigen::Vector3d(op[1][0], op[1][1], op[1][2]);
            state.orientation = Eigen::Quaterniond(op[2][0], op[2][1], op[2][2], op[2][3]);
            state.linear_velocity = Eigen::Vector3d(op[3][0], op[3][1], op[3][2]);
            actual["body"] = bodyJson(s.place(state, true).body);
        } else if (name == "reset_tasks") {
            actual["result"] = resultJson(s.resetTasks());
        } else if (name == "full_reset") {
            actual["body"] = bodyJson(s.fullReset(op[1].is_null() ? std::nullopt : std::optional<std::uint64_t>(op[1])).body);
            actual["seed"] = s.seed();
            tick = 0;
        } else if (name == "run_start") {
            actual["result"] = resultJson(s.runStart(op.size() > 1 && op[1].is_object() ? op[1] : Json::object()));
            actual["running"] = s.running();
            actual["snapshot"] = s.runSnapshot() ? *s.runSnapshot() : Json();
        } else if (name == "run_stop") {
            actual["result"] = resultJson(s.runStop());
            actual["running"] = s.running();
            actual["snapshot"] = s.runSnapshot() ? *s.runSnapshot() : Json();
        } else if (name == "run_adjust") {
            actual["result"] = resultJson(s.runAdjust(op[1]));
            actual["running"] = s.running();
            actual["snapshot"] = s.runSnapshot() ? *s.runSnapshot() : Json();
        }
        if (name != "advance")
            for (const auto &e : s.lastStep().task_events)
                events.push_back(e);
        actual["events"] = events;
        actual["feed"] = s.takeFeed();
        actual["counters"] = s.taskCounters();
        actual["final"] = checkpoint(s, tasks);
        ASSERT_EQ(diff(log[i], actual, where), "");
    }
}

} // namespace

TEST(Session, ReplaysThePythonScriptWithTasksDisabled) {
    replay("session_reference.json", false);
}

TEST(Session, ReplaysThePythonScriptWithTasksAndRunControl) {
    replay("session_tasks_reference.json", true);
}

TEST(Session, RunSnapshotDocumentMatchesPython) {
    const auto fixture = loadFixture("session_reference.json");
    const auto scenario = loadResolvedScenario(RP_RESOLVED_TALOS);
    for (const auto &c : fixture.at("run_snapshot_cases")) {
        const auto actual = buildRunSnapshot(scenario.tasks, c.at("snapshot"), c.at("extra"), c.at("now_ns"),
                                             c.at("adjustment"), c.at("message"));
        EXPECT_EQ(diff(c.at("expected"), actual, "$.run_snapshot", 1e-12), "");
    }
}

namespace {
void throughput(bool tasks, const char *label) {
    const auto scenario = loadResolvedScenario(RP_RESOLVED_TALOS);
    const RulesRegistry rules = robotics::rules::standardRules();
    const auto sensors = sensorNames(scenario);
    const std::vector<std::string> task_ids{"gate", "torpedo", "slalom"};
    Session s(scenario, createRuntime(scenario, &sensors), rules, SessionOptions{tasks ? &task_ids : nullptr, tasks});
    s.setKilled(false);
    s.setArmed(true);
    s.commandThrusters(Eigen::VectorXd::Constant(8, 3.0));
#ifdef NDEBUG
    constexpr int ticks = 50000;
#else
    constexpr int ticks = 3000; // Debug builds are ~150x slower; the number is only meaningful in Release
#endif
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < ticks; ++i) {
        s.advance();
        drain(s.runtime(), s.pack());
    }
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    std::printf("[ THROUGHPUT ] %s: %d ticks in %.3f s = %.0f ticks/s (%.1fx real time at 500 Hz)\n", label, ticks,
                seconds, ticks / seconds, ticks / seconds / 500.0);
    EXPECT_EQ(s.timeNs(), static_cast<std::int64_t>(ticks) * s.timestepNs());
}
} // namespace

TEST(Session, ThroughputWithTasksDisabled) {
    throughput(false, "tasks disabled");
}

TEST(Session, ThroughputWithTasksEnabled) {
    throughput(true, "tasks gate+torpedo+slalom");
}

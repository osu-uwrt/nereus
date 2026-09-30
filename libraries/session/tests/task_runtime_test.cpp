// TaskRuntime against a recorded reference. task_runtime_capture.json holds scripted inputs and
// the expected results for every task type the robosub_2026 pack uses (gate, slalom, torpedo, bins, surface + turn
// zone, table record()).
//   cases        TaskRuntime with scoring rules removed: events, projectile steps,
//                snapshots and indicators compared with a no-op Rules.
//   rules_cases  the compiled robosub_2026 Rules: full event streams (including
//                derived events), scores and describe().
// Comparison: discrete fields (strings, booleans, integers, keys, counts) exactly; floating
// point values within 1e-9 absolute + 1e-9 relative.
#include <robotics/session/tasks.hpp>

#include <rules/registry.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>

using namespace robotics::session;

namespace {
Json readJson(const std::string &path) {
    std::ifstream in(path);
    EXPECT_TRUE(in.good()) << path;
    return Json::parse(in);
}

::testing::AssertionResult near(const Json &expected, const Json &actual, const std::string &where) {
    if (expected.is_number() && actual.is_number()) {
        if (expected.is_number_integer() && actual.is_number_integer())
            return expected == actual ? ::testing::AssertionSuccess()
                                      : ::testing::AssertionFailure() << where << ": " << expected << " != " << actual;
        const double a = expected.get<double>(), b = actual.get<double>();
        return std::abs(a - b) <= 1e-9 + 1e-9 * std::abs(a)
                   ? ::testing::AssertionSuccess()
                   : ::testing::AssertionFailure() << where << ": " << a << " vs " << b;
    }
    if (expected.type() != actual.type())
        return ::testing::AssertionFailure() << where << ": type " << expected << " vs " << actual;
    if (expected.is_object()) {
        if (expected.size() != actual.size())
            return ::testing::AssertionFailure() << where << ": keys " << expected.dump() << " vs " << actual.dump();
        for (auto it = expected.begin(); it != expected.end(); ++it) {
            if (!actual.contains(it.key()))
                return ::testing::AssertionFailure() << where << ": missing " << it.key();
            auto r = near(it.value(), actual.at(it.key()), where + "/" + it.key());
            if (!r)
                return r;
        }
    } else if (expected.is_array()) {
        if (expected.size() != actual.size())
            return ::testing::AssertionFailure() << where << ": size " << expected.size() << " vs " << actual.size();
        for (std::size_t i = 0; i < expected.size(); ++i) {
            auto r = near(expected[i], actual[i], where + "[" + std::to_string(i) + "]");
            if (!r)
                return r;
        }
    } else if (expected != actual) {
        return ::testing::AssertionFailure() << where << ": " << expected << " != " << actual;
    }
    return ::testing::AssertionSuccess();
}

robotics::spatial::Pose poseOf(const Json &op) {
    robotics::spatial::Pose pose;
    pose.translation = Eigen::Vector3d(op["position"][0], op["position"][1], op["position"][2]);
    pose.rotation = Eigen::Quaterniond(op["wxyz"][0], op["wxyz"][1], op["wxyz"][2], op["wxyz"][3]);
    return pose;
}
Eigen::Vector3d vec(const Json &v) {
    return Eigen::Vector3d(v[0].get<double>(), v[1].get<double>(), v[2].get<double>());
}
Json toJson(const Events &events) {
    return Json(events);
}
Json vecJson(const Eigen::Vector3d &v) {
    return Json::array({v[0], v[1], v[2]});
}

// No-op stand-in for the competition rules (geometry-only comparison).
class NoRules : public Rules {
  public:
    Json evaluate(const Json &, const Events &, const Json &) override {
        return {{"scores", Json::array()}, {"events", Json::array()}};
    }
};
RulesRegistry noRules() {
    return {{"robosub_2026", [] { return std::make_unique<NoRules>(); }}};
}

const ResolvedScenario &scenario() {
    // The recorded cases start scoring at boot, not at the pack default (operator "start").
    static const ResolvedScenario s = [] {
        auto loaded = loadResolvedScenario(NEREUS_RESOLVED_TALOS);
        loaded.scenario["run"]["auto_start"] = true;
        return loaded;
    }();
    return s;
}

// Replays one recorded case and compares every op result.
void replay(const Json &captured, const RulesRegistry &rules, bool with_derived) {
    const std::string name = captured["name"];
    std::vector<std::string> task_ids = captured["task_ids"].get<std::vector<std::string>>();
    TaskRuntime runtime(scenario(), rules, &task_ids);
    std::size_t events_seen = 0;
    for (std::size_t i = 0; i < captured["ops"].size(); ++i) {
        const Json &op = captured["ops"][i], &expected = captured["results"][i];
        const std::string where = name + " op " + std::to_string(i) + " (" + op["op"].get<std::string>() + ")";
        const std::int64_t t = op["t"];
        const std::string kind = op["op"];
        if (kind == "observe") {
            const Events events = runtime.observe(t, poseOf(op));
            ASSERT_TRUE(near(expected["events"], toJson(events), where));
            events_seen += events.size();
        } else if (kind == "release") {
            const Events events =
                runtime.releaseProjectile(t, op["id"], op["mech"], vec(op["tip"]), op["radius"], op["length"]);
            ASSERT_TRUE(near(expected["events"], toJson(events), where));
            events_seen += events.size();
        } else if (kind == "step") {
            const ProjectileStep step = runtime.stepProjectile(t, op["id"], vec(op["start"]), vec(op["end"]),
                                                               vec(op["axis"]), vec(op["velocity"]));
            ASSERT_TRUE(near(expected["events"], toJson(step.events), where));
            EXPECT_EQ(expected["stop"].get<bool>(), step.stop) << where;
            ASSERT_TRUE(near(expected["position"], step.position_world ? vecJson(*step.position_world) : Json(),
                             where + " position"));
            ASSERT_TRUE(near(expected["velocity"], step.velocity_world ? vecJson(*step.velocity_world) : Json(),
                             where + " velocity"));
            events_seen += step.events.size();
        } else if (kind == "record") {
            const Events events = runtime.record(t, op["events"].get<Events>());
            ASSERT_TRUE(near(expected["events"], toJson(events), where));
            events_seen += events.size();
        } else if (kind == "stop") {
            const Events events = runtime.stop(t);
            ASSERT_TRUE(near(expected["events"], toJson(events), where));
            events_seen += events.size();
        } else if (kind == "start") {
            runtime.start(t, op["options"]);
        } else if (kind == "reset") {
            runtime.reset(t);
        } else {
            FAIL() << "unknown op " << kind;
        }
    }
    EXPECT_EQ(captured["event_count"].get<std::size_t>(), events_seen) << name;
    const Json snapshot = runtime.snapshot();
    if (!with_derived) {
        Json expected = captured["snapshot"];
        ASSERT_TRUE(near(expected, snapshot, name + " snapshot"));
        ASSERT_TRUE(near(captured["indicators"], runtime.indicators(), name + " indicators"));
    } else {
        ASSERT_TRUE(near(captured["snapshot"]["scores"], snapshot["scores"], name + " scores"));
        ASSERT_TRUE(near(captured["snapshot"]["history"], snapshot["history"], name + " history"));
        ASSERT_TRUE(near(captured["snapshot"]["latched"], snapshot["latched"], name + " latched"));
        ASSERT_TRUE(near(captured["describe"], runtime.describe(), name + " describe"));
        ASSERT_TRUE(near(captured["indicators"], runtime.indicators(), name + " indicators"));
    }
}
} // namespace

TEST(TaskRuntimeEquivalence, GeometryCasesMatchReference) {
    const Json fixture = readJson(std::string(NEREUS_SESSION_FIXTURES) + "/task_runtime_capture.json");
    ASSERT_EQ(fixture["cases"].size(), 7u);
    for (const auto &captured : fixture["cases"]) {
        SCOPED_TRACE(captured["name"].get<std::string>());
        replay(captured, noRules(), false);
    }
}

TEST(TaskRuntimeEquivalence, Robosub2026RulesMatchReference) {
    const Json fixture = readJson(std::string(NEREUS_SESSION_FIXTURES) + "/task_runtime_capture.json");
    ASSERT_EQ(fixture["rules_cases"].size(), 7u);
    for (const auto &captured : fixture["rules_cases"]) {
        SCOPED_TRACE(captured["name"].get<std::string>());
        replay(captured, robotics::rules::standardRules(), true);
    }
}

namespace {
struct ScriptedRules : Rules {
    Json evaluate(const Json &state, const Events &events, const Json &) override {
        Json out = {{"scores", Json::array()}, {"events", Json::array()}};
        for (const auto &event : events)
            if (event["id"] == "forward_pass") {
                out["scores"].push_back({{"row", "gate"}, {"points", 5}});
                out["events"].push_back({{"id", "derived"},
                                         {"type", "note"},
                                         {"task", "gate"},
                                         {"region", ""},
                                         {"time_ns", event["time_ns"]},
                                         {"data", {{"seen_scores", state["scores"].size()}}}});
                seen_running = state["run"]["running"];
            }
        return out;
    }
    Json describe(const Json &, const Json &) override {
        return {{"extra", 1}};
    }
    Json feed(const Json &state, const Events &events, const Json &context, const Json &) override {
        return Json::array({{{"events", events.size()},
                             {"payloads", context["payloads"].size()},
                             {"has_run", state.contains("run")}}});
    }
    bool seen_running{false};
};

RulesRegistry scripted(std::function<std::unique_ptr<Rules>()> factory) {
    return {{"robosub_2026", factory}};
}

robotics::spatial::Pose gatePose(double x_local) {
    // Gate task frame at the placement in the Talos scenario.
    const Json &placement = scenario().scenario["task_placements"][0];
    const double yaw = placement["yaw_deg"].get<double>() * 3.14159265358979323846 / 180 / 2;
    robotics::spatial::Pose base{
        Eigen::Vector3d(placement["position_m"][0], placement["position_m"][1], placement["position_m"][2]),
        Eigen::Quaterniond(std::cos(yaw), 0, 0, std::sin(yaw))};
    return robotics::spatial::compose(base, {Eigen::Vector3d(x_local, -0.75, -0.2), Eigen::Quaterniond::Identity()});
}
} // namespace

TEST(TaskRuntime, RulesOutputsAreValidatedCommittedAndVisible) {
    const std::vector<std::string> gate = {"gate"};
    TaskRuntime runtime(scenario(), scripted([] { return std::make_unique<ScriptedRules>(); }), &gate);
    Events all;
    std::int64_t t = 0;
    for (double x : {3.0, 2.0, 1.0, 0.5, 0.0, -0.5, -1.0, -2.0}) {
        t += 100'000'000;
        const auto events = runtime.observe(t, gatePose(x));
        all.insert(all.end(), events.begin(), events.end());
    }
    ASSERT_FALSE(all.empty());
    EXPECT_EQ(all.back()["id"], "derived"); // rule events follow the input events
    const Json snapshot = runtime.snapshot();
    EXPECT_EQ(snapshot["scores"]["gate"], 5);
    EXPECT_EQ(snapshot["history"].size(), all.size());
    EXPECT_EQ(snapshot["environment"]["floor_z_m"], scenario().pool["parameters"]["depth_m"].get<double>() * -1);
    EXPECT_EQ(runtime.describe()["extra"], 1);
    const Json feed = runtime.feed(all, {{"payloads", Json::array({1, 2})}});
    EXPECT_EQ(feed[0]["payloads"], 2);
    EXPECT_TRUE(feed[0]["has_run"]);
}

TEST(TaskRuntime, MalformedRulesOutputFailsTheObserverUntilReset) {
    struct Bad : Rules {
        Json evaluate(const Json &, const Events &events, const Json &) override {
            Json out = {{"scores", Json::array()}, {"events", Json::array()}};
            out["events"].push_back({{"id", "x"},
                                     {"type", "y"},
                                     {"task", "nope"},
                                     {"region", ""},
                                     {"time_ns", events[0]["time_ns"]},
                                     {"data", Json::object()}});
            return out;
        }
    };
    const std::vector<std::string> gate = {"gate"};
    TaskRuntime runtime(scenario(), scripted([] { return std::make_unique<Bad>(); }), &gate);
    std::int64_t t = 0;
    bool thrown = false;
    for (double x : {3.0, 2.0, 1.0, 0.5, 0.0, -0.5, -1.0, -2.0}) {
        t += 100'000'000;
        try {
            runtime.observe(t, gatePose(x));
        } catch (const std::invalid_argument &) {
            thrown = true;
            break;
        }
    }
    ASSERT_TRUE(thrown);
    EXPECT_TRUE(runtime.snapshot()["scores"].empty());
    EXPECT_TRUE(runtime.snapshot()["history"].empty());
    EXPECT_THROW(runtime.observe(t + 1, gatePose(0)), std::runtime_error);
    runtime.reset(t + 1);
    EXPECT_NO_THROW(runtime.observe(t + 2, gatePose(3)));
}

TEST(TaskRuntime, ConstructionAndRunControlErrors) {
    std::vector<std::string> unknown = {"nonexistent"}, duplicate = {"gate", "gate"};
    EXPECT_THROW(TaskRuntime(scenario(), noRules(), &unknown), std::invalid_argument);
    EXPECT_THROW(TaskRuntime(scenario(), noRules(), &duplicate), std::invalid_argument);
    EXPECT_THROW(TaskRuntime(scenario(), RulesRegistry{}), std::invalid_argument); // rules name unknown
    TaskRuntime runtime(scenario(), noRules());
    EXPECT_THROW(runtime.start(0, {{"no_such_option", 1}}), std::invalid_argument);
    runtime.start(1'000'000'000, {{"role", "rescue"}});
    Json snapshot = runtime.snapshot();
    EXPECT_TRUE(snapshot["run"]["running"]);
    EXPECT_EQ(snapshot["run"]["options"]["role"], "rescue");
    EXPECT_EQ(snapshot["run"]["started_ns"], 1'000'000'000);
    EXPECT_THROW(runtime.observe(999, gatePose(1)), std::invalid_argument); // time goes backwards
    EXPECT_NO_THROW(runtime.observe(1'000'000'000, gatePose(1)));           // rejected calls do not poison
    runtime.reset(0);
    EXPECT_TRUE(runtime.snapshot()["run"]["running"]); // scenario auto-starts its run
    runtime.stop(5);
    EXPECT_FALSE(runtime.snapshot()["run"]["running"]);
    EXPECT_EQ(runtime.snapshot()["run"]["stopped_ns"], 5);
}

TEST(TaskRuntime, ProjectileContractErrors) {
    TaskRuntime runtime(scenario(), noRules());
    const Eigen::Vector3d p(0, 0, 0), axis(1, 0, 0);
    EXPECT_THROW(runtime.stepProjectile(1, 5, p, p, axis, p), std::invalid_argument); // not released
    EXPECT_THROW(runtime.releaseProjectile(1, 5, "claw", p, 0.02, 0.08), std::invalid_argument);
    EXPECT_THROW(runtime.releaseProjectile(1, -1, "dropper", p, 0.02, 0.08), std::invalid_argument);
    EXPECT_THROW(runtime.releaseProjectile(1, 5, "dropper", p, 0.0, 0.08), std::invalid_argument);
    runtime.reset(0);
    runtime.releaseProjectile(1, 5, "dropper", p, 0.02, 0.08);
    EXPECT_THROW(runtime.releaseProjectile(2, 5, "dropper", p, 0.02, 0.08), std::invalid_argument);
    runtime.reset(0);
    runtime.releaseProjectile(1, 5, "dropper", p, 0.02, 0.08);
    EXPECT_THROW(runtime.stepProjectile(2, 5, p, p, Eigen::Vector3d(2, 0, 0), p), std::invalid_argument);
}

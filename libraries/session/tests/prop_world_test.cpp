// Equivalence of the C++ PropWorld with the Python/PyBullet reference. fixtures/prop_world.json
// is written by fixtures/capture_prop_world.py from the scripted cases of
// tests/python/test_prop_world.py (grasp, carry, basket, release, drop, reset); this test drives
// the C++ port with the identical per-tick inputs and compares the ordered events exactly and the
// prop states every 50 ticks within a tolerance.
#include <robotics/session/prop_world.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <fstream>
#include <iostream>

using namespace robotics::session;
using Matrix4 = Eigen::Matrix4d;

namespace {
// Pybullet's bundled Bullet and the system libbullet (3.05, double build) are different
// revisions and the contact scenes are chaotic at contact onset; the observed agreement is
// printed by the test. Poses: metres / radians of rotation angle. Event times: ticks.
constexpr double kPositionToleranceM = 2e-2;
constexpr double kRotationToleranceRad = 0.15;
constexpr int kEventToleranceTicks = 10;

Json readFixture() {
    std::ifstream stream(std::string(NEREUS_SESSION_FIXTURES) + "/prop_world.json");
    EXPECT_TRUE(stream.good());
    return Json::parse(stream);
}

Matrix4 matrixOf(const Json &rows) {
    Matrix4 m;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            m(r, c) = rows.at(static_cast<std::size_t>(r)).at(static_cast<std::size_t>(c)).get<double>();
    return m;
}
Eigen::Vector3d vec(const Json &v) { return {v.at(0).get<double>(), v.at(1).get<double>(), v.at(2).get<double>()}; }
Eigen::Quaterniond quat(const Json &v) {
    return {v.at(0).get<double>(), v.at(1).get<double>(), v.at(2).get<double>(), v.at(3).get<double>()};
}

struct Stats {
    double position{0}, rotation{0};
    std::map<std::string, double> position_by_prop;
};

void compareState(const std::map<std::string, PropState> &actual, const Json &expected, const std::string &where,
                  Stats &stats, bool check) {
    ASSERT_EQ(actual.size(), expected.size()) << where;
    for (const auto &[name, e] : expected.items()) {
        const PropState &a = actual.at(name);
        const double dp = (a.position - vec(e.at("position"))).norm();
        const double dr = a.orientation.angularDistance(quat(e.at("orientation_wxyz")));
        stats.position = std::max(stats.position, dp);
        stats.rotation = std::max(stats.rotation, dr);
        double &worst = stats.position_by_prop[name];
        worst = std::max(worst, dp);
        if (!check)
            continue;
        EXPECT_LT(dp, kPositionToleranceM) << where << " " << name << " actual " << a.position.transpose()
                                           << " expected " << vec(e.at("position")).transpose();
        EXPECT_LT(dr, kRotationToleranceRad) << where << " " << name;
        EXPECT_EQ(a.attached, e.at("attached").get<bool>()) << where << " " << name;
        EXPECT_EQ(a.basket.empty() ? Json() : Json(a.basket), e.at("basket")) << where << " " << name;
    }
}

struct Replay {
    Events events;
    std::vector<int> event_ticks;
    Stats stats;
    double step_us{0};
    int ticks{0};
};

Replay replay(const ResolvedScenario &scenario, const Json &data, double dt, int sample_ticks, PropWorld *shared = nullptr,
           bool check = true, std::size_t op_limit = SIZE_MAX, Matrix4 *final_mount = nullptr) {
    std::unique_ptr<PropWorld> owned;
    if (!shared)
        owned = std::make_unique<PropWorld>(scenario, "table");
    PropWorld &world = shared ? *shared : *owned;
    Stats initial_stats;
    compareState(world.props(), data.at("initial"), "initial", initial_stats, false);

    const Matrix4 inverse_mount = matrixOf(data.at("mount_local")).inverse();
    Matrix4 mount = Matrix4::Identity();
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            mount(r, c) = data.at("mount_rotation").at(static_cast<std::size_t>(r)).at(static_cast<std::size_t>(c)).get<double>();
    // Expand the run-length encoded claw joints.
    std::vector<double> joints;
    for (const auto &item : data.at("joints_rle"))
        joints.insert(joints.end(), item.at(1).get<std::size_t>(), item.at(0).get<double>());

    Replay run;
    std::int64_t time_ns = 0;
    std::size_t next_sample = 0;
    const auto &samples = data.at("samples");
    const std::int64_t tick_ns = std::llround(dt * 1e9);
    double spent = 0;
    for (const auto &op : data.at("ops")) {
        if (op_limit-- == 0)
            break;
        if (op.contains("place")) {
            mount.block<3, 1>(0, 3) = vec(op.at("place"));
        } else if (op.contains("reset")) {
            world.reset();
            time_ns = 0;
            const auto actual = world.props();
            compareState(actual, data.at("after_reset"), "after reset", run.stats, false);
            for (const auto &[name, state] : actual)
                EXPECT_FALSE(state.attached) << name;
        } else {
            const Eigen::Vector3d velocity = vec(op.at("velocity"));
            for (int i = 0; i < op.at("move").get<int>(); ++i) {
                time_ns += tick_ns;
                mount.block<3, 1>(0, 3) += velocity * dt;
                const Matrix4 body = mount * inverse_mount;
                robotics::spatial::Pose pose;
                pose.translation = body.block<3, 1>(0, 3);
                pose.rotation = Eigen::Quaterniond(Eigen::Matrix3d(body.block<3, 3>(0, 0)));
                const double j = joints.at(static_cast<std::size_t>(run.ticks));
                ++run.ticks;
                const auto begin = std::chrono::steady_clock::now();
                auto events = world.step(dt, time_ns, pose, velocity, Eigen::Vector3d::Zero(), {j, j}, Water{}, true);
                spent += std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
                for (auto &e : events) {
                    run.events.push_back(e);
                    run.event_ticks.push_back(run.ticks);
                }
                if (run.ticks % sample_ticks == 0 && next_sample < samples.size()) {
                    const Json &s = samples.at(next_sample++);
                    EXPECT_EQ(s.at("tick").get<int>(), run.ticks);
                    compareState(world.props(), s.at("props"), "tick " + std::to_string(run.ticks), run.stats, check);
                }
            }
        }
    }
    run.step_us = 1e6 * spent / run.ticks;
    if (final_mount)
        *final_mount = mount;
    return run;
}

void compareEvents(const Replay &run, const Json &expected, const std::string &name) {
    ASSERT_EQ(run.events.size(), expected.size()) << name;
    for (std::size_t i = 0; i < expected.size(); ++i) {
        const Json &e = expected.at(i);
        const Event &a = run.events.at(i);
        for (const char *key : {"task", "type", "id", "region", "data"})
            EXPECT_EQ(a.at(key), e.at(key)) << name << " event " << i << " " << key;
        const int dt = std::abs(run.event_ticks.at(i) - e.at("tick").get<int>());
        EXPECT_LE(dt, kEventToleranceTicks) << name << " event " << i;
    }
}
} // namespace

class PropWorldEquivalence : public testing::TestWithParam<const char *> {};

TEST_P(PropWorldEquivalence, MatchesPythonReference) {
    const std::string name = GetParam();
    const Json fixture = readFixture();
    const Json &data = fixture.at("cases").at(name);
    const auto scenario = loadResolvedScenario(NEREUS_RESOLVED_TALOS);
    const Replay run = replay(scenario, data, fixture.at("dt").get<double>(), fixture.at("sample_ticks").get<int>());
    compareEvents(run, data.at("events"), name);
    int worst = 0;
    for (std::size_t i = 0; i < run.events.size(); ++i)
        worst = std::max(worst, std::abs(run.event_ticks[i] - data.at("events").at(i).at("tick").get<int>()));
    std::cout << "[observed] " << name << ": max prop position diff " << run.stats.position << " m (";
    for (const auto &[prop, d] : run.stats.position_by_prop)
        std::cout << prop << " " << d << " ";
    std::cout << "), rotation diff " << run.stats.rotation << " rad, max event tick offset " << worst << ", C++ step " << run.step_us
              << " us\n";
}

INSTANTIATE_TEST_SUITE_P(Cases, PropWorldEquivalence,
                         testing::Values("grasp_carry_release", "drop_helmet_basket", "drop_warning_basket",
                                         "release_elsewhere", "empty_jaws", "reset_then_pick"));

TEST(PropWorld, FinalStatesAndQueriesMatch) {
    const Json fixture = readFixture();
    const auto scenario = loadResolvedScenario(NEREUS_RESOLVED_TALOS);
    for (const char *name : {"grasp_carry_release", "drop_helmet_basket", "drop_warning_basket", "release_elsewhere"}) {
        const Json &data = fixture.at("cases").at(name);
        PropWorld world(scenario, "table");
        replay(scenario, data, fixture.at("dt").get<double>(), fixture.at("sample_ticks").get<int>(), &world, false);
        Stats stats;
        compareState(world.props(), data.at("final"), name, stats, true);
        EXPECT_NEAR(world.jawPosition(), data.at("final_jaw").get<double>(), 1e-3) << name;
        std::map<std::string, std::string> contents;
        for (const auto &[k, v] : data.at("final_basket_contents").items())
            contents[k] = v.get<std::string>();
        EXPECT_EQ(world.basketContents(), contents) << name;
    }
}

namespace {
using State = robotics::simulation::ContactResolver::State;
using Matrix6 = robotics::simulation::ContactResolver::Matrix6;
// Robot COM state whose claw mount is at `mount` (world), moving with world velocity `velocity`.
State stateAt(const Matrix4 &mount, const Matrix4 &mount_local, const Eigen::Vector3d &velocity) {
    const Matrix4 body = mount * mount_local.inverse();
    const Eigen::Quaterniond q(Eigen::Matrix3d(body.block<3, 3>(0, 0)));
    State x;
    x << body.block<3, 1>(0, 3), q.w(), q.x(), q.y(), q.z(), q.conjugate() * velocity, Eigen::Vector3d::Zero();
    return x;
}
Eigen::Vector3d worldVelocity(const State &x) {
    return Eigen::Quaterniond(x[3], x[4], x[5], x[6]).normalized() * Eigen::Vector3d(x.segment<3>(7));
}
// Mount height at which lowering the robot in 1 mm steps first meets robot-side contact.
double contactHeight(robotics::simulation::ContactResolver &contacts, Matrix4 mount, const Matrix4 &mount_local,
                     const Matrix6 &inverse_mass) {
    for (int i = 0; i < 600; ++i, mount(2, 3) -= 1e-3) {
        const State x = stateAt(mount, mount_local, Eigen::Vector3d::Zero());
        if (!contacts.resolve(x, inverse_mass).isApprox(x, 1e-12))
            return mount(2, 3);
    }
    return std::nan("");
}
// Translation-only robot (these harnesses integrate position only): the contact impulse cannot
// turn into rotation about the COM, so the contact point velocity is the COM velocity.
const Matrix6 kInverseMass = (Eigen::Matrix<double, 6, 1>() << 1 / 40., 1 / 45., 1 / 45., 1e-9, 1e-9, 1e-9)
                                 .finished()
                                 .asDiagonal();
} // namespace

// Old task_contacts.cpp: the claw pads cannot pass through the table; the robot is pushed out and
// loses its approach velocity. Free space is untouched.
TEST(PropWorld, RobotContactsStopThePadsAtTheTable) {
    const Json fixture = readFixture();
    const Json &data = fixture.at("cases").at("empty_jaws");
    const auto scenario = loadResolvedScenario(NEREUS_RESOLVED_TALOS);
    PropWorld world(scenario, "table");
    auto contacts = world.vehicleContacts(0.4);
    const Matrix4 mount_local = matrixOf(data.at("mount_local"));
    Matrix4 mount = Matrix4::Identity();
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            mount(r, c) = data.at("mount_rotation").at(static_cast<std::size_t>(r)).at(static_cast<std::size_t>(c)).get<double>();
    mount.block<3, 1>(0, 3) = vec(data.at("ops").at(0).at("place")) + Eigen::Vector3d(0, 0, .3);
    const double touch = contactHeight(*contacts, mount, mount_local, kInverseMass);
    ASSERT_TRUE(std::isfinite(touch)) << "no robot contact while lowering the claw onto the table";

    mount(2, 3) = touch + .05;
    const State free = stateAt(mount, mount_local, {0, 0, -.2});
    EXPECT_TRUE(contacts->resolve(free, kInverseMass).isApprox(free, 1e-12));

    mount(2, 3) = touch - .01; // 1 cm into the table, still sinking
    const State pressed = stateAt(mount, mount_local, {0, 0, -.2});
    const State out = contacts->resolve(pressed, kInverseMass);
    EXPECT_GT(out[2] - pressed[2], .008) << "pushed back out of the table";
    EXPECT_GT(worldVelocity(out).z(), -1e-6) << "approach velocity removed";
}

// The held prop belongs to the robot: pressing it into the table pushes the robot back instead of
// driving the prop through scenery, so the finite grasp holds. Without robot contacts the same
// press tears the prop out of the grasp (the "wobble, then dropped" of the missing port).
TEST(PropWorld, HeldPropIsPartOfTheRobotAgainstScenery) {
    const Json fixture = readFixture();
    const Json &data = fixture.at("cases").at("grasp_carry_release");
    const auto scenario = loadResolvedScenario(NEREUS_RESOLVED_TALOS);
    const double dt = fixture.at("dt").get<double>();
    const Matrix4 mount_local = matrixOf(data.at("mount_local"));
    const auto press = [&](bool robot_contacts) {
        PropWorld world(scenario, "table");
        auto contacts = world.vehicleContacts(0.4);
        Matrix4 mount;
        const auto run = replay(scenario, data, dt, fixture.at("sample_ticks").get<int>(), &world, false, 5, &mount);
        EXPECT_TRUE(world.props().at("pill").attached) << "script prefix grasps the pill";
        std::vector<std::string> detached;
        // From where the script holds it (the pill just above the table), drive the claw straight
        // down at 0.15 m/s for 1.5 s.
        State x = stateAt(mount, mount_local, {0, 0, -.15});
        std::int64_t time_ns = static_cast<std::int64_t>(std::llround(run.ticks * dt * 1e9));
        double lowest = x[2];
        for (int i = 0; i < static_cast<int>(1.5 / dt); ++i) {
            x.segment<3>(7) = Eigen::Quaterniond(x[3], x[4], x[5], x[6]).conjugate() * Eigen::Vector3d(0, 0, -.15);
            if (robot_contacts)
                x = contacts->resolve(x, kInverseMass);
            x.head<3>() += worldVelocity(x) * dt;
            if (robot_contacts)
                x = contacts->resolve(x, kInverseMass);
            lowest = std::min(lowest, x[2]);
            robotics::spatial::Pose pose;
            pose.translation = x.head<3>();
            pose.rotation = Eigen::Quaterniond(x[3], x[4], x[5], x[6]).normalized();
            time_ns += std::llround(dt * 1e9);
            for (const auto &e : world.step(dt, time_ns, pose, worldVelocity(x), Eigen::Vector3d::Zero(),
                                            {0.0, 0.0}, Water{}, true))
                if (e.at("type") == "detach")
                    detached.push_back(e.at("data").at("reason").get<std::string>());
        }
        return std::make_pair(detached, stateAt(mount, mount_local, {}).head<3>().z() - lowest);
    };
    const auto [with, depth_with] = press(true);
    EXPECT_TRUE(with.empty()) << "grasp lost with robot contacts: " << (with.empty() ? "" : with.front());
    EXPECT_LT(depth_with, .01) << "robot stopped at the table";
    const auto [without, depth_without] = press(false);
    std::cout << "[observed] without robot contacts: sank " << depth_without << " m, detach "
              << (without.empty() ? "none" : without.front()) << "\n";
    EXPECT_FALSE(without.empty()) << "the same press without robot contacts should tear the prop out";
}

TEST(PropWorld, RejectsInvalidInputsAndMissingAssets) {
    auto scenario = loadResolvedScenario(NEREUS_RESOLVED_TALOS);
    PropWorld world(scenario, "table");
    EXPECT_EQ(world.task(), "table");
    EXPECT_EQ(world.mechanismId(), "claw");
    const robotics::spatial::Pose pose;
    const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
    EXPECT_THROW(world.step(0.0, 0, pose, zero, zero, {0, 0}, Water{}, true), std::invalid_argument);
    EXPECT_THROW(world.step(0.004, 0, pose, {0, std::nan(""), 0}, zero, {0, 0}, Water{}, true), std::invalid_argument);
    world.step(0.004, 8, pose, zero, zero, {0, 0}, Water{}, true);
    EXPECT_THROW(world.step(0.004, 4, pose, zero, zero, {0, 0}, Water{}, true), std::invalid_argument);
    EXPECT_THROW(PropWorld(scenario, "no_such_task"), std::invalid_argument);
    scenario.asset_paths["robot"].erase("claw_pad_left");
    EXPECT_THROW(PropWorld(scenario, "table"), std::invalid_argument);
}

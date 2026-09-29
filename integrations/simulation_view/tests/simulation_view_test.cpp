#include <gtest/gtest.h>
#include <limits>
#include <robotics/integrations/simulation_view.hpp>
#include <robotics/sensors/models.hpp>
#include <robotics/sensors/runtime.hpp>

using namespace robotics;
using namespace std::chrono_literals;

namespace {
visualization::RotorRig rig() {
    visualization::RotorRig result;
    result.inputs = {"thrust"};
    result.animation.curve.forward = result.animation.curve.reverse = {
        0, 30. / 3.14159265358979323846, 0, 0}; // One radian/second per newton.
    result.mounts = {{"body", "rotor", {1, 0, 0}, Eigen::Vector3d::UnitZ()}};
    return result;
}
visualization::LivePoseOptions options() {
    visualization::LivePoseOptions result{"sim", "simulation", "world", "body", "pose", 2, 8};
    result.moving_frames = {{"body", "rotor"}};
    return result;
}
spatial::Pose rotor(const visualization::SourceSnapshot &view) {
    return view.data->frames->lookup("body", "rotor", view.time_ns).pose.value();
}
simulation::Snapshot observation(std::uint64_t tick, double force) {
    simulation::Snapshot result;
    result.tick = tick;
    result.elapsed = 100ms * tick;
    result.thruster_forces = Eigen::VectorXd::Constant(1, force);
    return result;
}
} // namespace

TEST(SimulationView, PublishesComPoseWithSimulationTime) {
    visualization::LivePoseSource source({"sim", "simulation"});
    simulation::Snapshot state;
    state.elapsed = 123ns;
    state.body.position = {1, 2, 3};
    state.body.orientation = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY());
    integrations::publishSimulationPose(source, state);
    const auto view = source.snapshot();
    ASSERT_TRUE(view.data);
    EXPECT_EQ(view.time_ns, 123);
    const auto pose = view.data->frames->lookup("world", "body", 123).pose;
    ASSERT_TRUE(pose);
    EXPECT_EQ(pose->translation, state.body.position);
    EXPECT_TRUE(pose->rotation.isApprox(state.body.orientation));
}

TEST(SimulationView, PollingDisconnectAndResetDoNotChangeDynamicsOrNoisySensors) {
    simulation::PlantParameters parameters;
    parameters.thrusters.push_back({"thrust"});
    simulation::BodyState initial;
    initial.position = {5, 5, -2};
    sensors::Runtime plain(parameters, initial, 42), viewed(parameters, initial, 42);
    sensors::NoiseParameters noise;
    noise.white_stddev.setConstant(0.02);
    noise.walk_stddev.setConstant(0.01);
    const sensors::Device device{"imu", "body", 5ms, 3ms, 64};
    auto a = plain.add(device, sensors::Imu({}, noise, noise));
    auto b = viewed.add(device, sensors::Imu({}, noise, noise));
    visualization::LivePoseSource source(options());
    integrations::SimulationPosePublisher publisher(source, {"thrust"}, rig());
    publisher.publish(viewed.observe());
    for (int tick = 0; tick < 1000; ++tick) {
        if (tick == 500) {
            plain.reset(initial, 42);
            publisher.publish(viewed.reset(initial, 42));
        }
        plain.command(Eigen::VectorXd::Constant(1, 4));
        viewed.command(Eigen::VectorXd::Constant(1, 4));
        const auto left = plain.advance(), right = viewed.advance();
        publisher.publish(right);
        if (tick < 100 || tick % 50 == 0)
            source.snapshot();
        if (tick == 200)
            source.disconnect();
        if (tick == 600)
            source.reconnect();
        EXPECT_EQ(left.tick, right.tick);
        EXPECT_EQ(left.elapsed, right.elapsed);
        EXPECT_EQ(left.generation, right.generation);
        EXPECT_EQ(left.body.position, right.body.position);
        EXPECT_EQ(left.body.orientation.coeffs(), right.body.orientation.coeffs());
        EXPECT_EQ(left.body.linear_velocity, right.body.linear_velocity);
        EXPECT_EQ(left.body.angular_velocity, right.body.angular_velocity);
        EXPECT_EQ(left.thruster_forces, right.thruster_forces);
        const auto readings_a = a->drain(), readings_b = b->drain();
        ASSERT_EQ(readings_a.size(), readings_b.size());
        for (std::size_t i = 0; i < readings_a.size(); ++i) {
            const auto &x = readings_a[i], &y = readings_b[i];
            EXPECT_EQ(x.header.acquired, y.header.acquired);
            EXPECT_EQ(x.header.delivered, y.header.delivered);
            EXPECT_EQ(x.header.sequence, y.header.sequence);
            EXPECT_EQ(x.header.generation, y.header.generation);
            ASSERT_TRUE(x.measurement.value);
            ASSERT_TRUE(y.measurement.value);
            EXPECT_EQ(x.measurement.value->specific_force, y.measurement.value->specific_force);
            EXPECT_EQ(x.measurement.value->angular_velocity, y.measurement.value->angular_velocity);
        }
    }
    EXPECT_GT(source.snapshot().delivery.dropped_queue, 0U);
}

TEST(SimulationView, NamedRotorChannelsAndQueueLossPreservePhase) {
    visualization::LivePoseSource fast(options()), slow(options());
    integrations::SimulationPosePublisher a(fast, {"unused", "thrust"}, rig());
    integrations::SimulationPosePublisher b(slow, {"unused", "thrust"}, rig());
    for (std::uint64_t tick = 0; tick <= 10; ++tick) {
        auto state = observation(tick, 0);
        state.thruster_forces = Eigen::Vector2d(1000, tick < 5 ? 2 : -1);
        state.body.position = {static_cast<double>(tick), 0, 0};
        a.publish(state);
        b.publish(state);
        fast.snapshot();
        if (tick == 6)
            slow.disconnect();
    }
    slow.reconnect();
    const auto left = fast.snapshot(), right = slow.snapshot();
    EXPECT_EQ(left.time_ns, right.time_ns);
    EXPECT_GT(right.delivery.dropped_queue, 0U);
    const Eigen::Quaterniond expected(Eigen::AngleAxisd(.5, Eigen::Vector3d::UnitZ()));
    EXPECT_TRUE(rotor(left).rotation.isApprox(expected, 1e-14));
    EXPECT_EQ(rotor(left).rotation.coeffs(), rotor(right).rotation.coeffs());
    EXPECT_EQ(rotor(left).translation, rotor(right).translation);
    EXPECT_EQ(right.data->frames->lookup("world", "body", right.time_ns).pose->translation,
              Eigen::Vector3d(10, 0, 0));
    auto reset = observation(0, 4);
    reset.thruster_forces = Eigen::Vector2d(1000, 4);
    reset.generation = 1;
    b.publish(reset);
    EXPECT_TRUE(rotor(slow.snapshot()).rotation.isApprox(Eigen::Quaterniond::Identity()));
    EXPECT_THROW(b.publish(observation(11, 2)), std::invalid_argument);
    EXPECT_THROW(b.publish(reset), std::invalid_argument);
}

TEST(SimulationView, RejectedForcesResetAndSkippedTicksDoNotAdvanceAnimation) {
    auto config = rig();
    config.animation.curve.forward[1] = 1e300;
    visualization::LivePoseSource source(options());
    integrations::SimulationPosePublisher publisher(source, {"thrust"}, config);
    publisher.publish(observation(0, -1));
    auto invalid = observation(0, 1e30);
    invalid.generation = 1;
    EXPECT_THROW(publisher.publish(invalid), std::overflow_error);
    invalid = observation(1, std::numeric_limits<double>::max());
    EXPECT_THROW(publisher.publish(invalid), std::invalid_argument);
    EXPECT_THROW(publisher.publish(observation(2, -1)), std::invalid_argument);
    invalid = observation(1, -1);
    invalid.thruster_forces.resize(0);
    EXPECT_THROW(publisher.publish(invalid), std::invalid_argument);
    publisher.publish(observation(1, -1));
    const auto view = source.snapshot();
    EXPECT_EQ(view.time_ns, 100000000);
    EXPECT_TRUE(rotor(view).rotation.isApprox(
        Eigen::Quaterniond(Eigen::AngleAxisd(-.1, Eigen::Vector3d::UnitZ())), 1e-14));
}

TEST(SimulationView, InvalidPivotOutputDoesNotPartiallyCommitPhase) {
    auto config = rig();
    config.mounts.front().pivot = {1e12, 0, 0};
    visualization::LivePoseSource source(options());
    integrations::SimulationPosePublisher publisher(source, {"thrust"}, config);
    publisher.publish(observation(0, 20));
    EXPECT_THROW(publisher.publish(observation(1, 0)), std::invalid_argument);
    auto recovery = observation(1, 0);
    recovery.elapsed = 10ms;
    publisher.publish(recovery);
    EXPECT_TRUE(
        rotor(source.snapshot())
            .rotation.isApprox(Eigen::Quaterniond(Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitZ())),
                               1e-14));
}

TEST(SimulationView, RigAndDestinationMustAgreeBeforePublishing) {
    visualization::LivePoseSource source(options());
    EXPECT_THROW((integrations::SimulationPosePublisher(source, {"missing"}, rig())),
                 std::invalid_argument);
    EXPECT_THROW((integrations::SimulationPosePublisher(source, {"thrust", "thrust"}, rig())),
                 std::invalid_argument);
    EXPECT_THROW((integrations::SimulationPosePublisher(source, {"thrust"})),
                 std::invalid_argument);
    auto config = rig();
    config.mounts.front().child_frame = "other";
    EXPECT_THROW((integrations::SimulationPosePublisher(source, {"thrust"}, config)),
                 std::invalid_argument);
}

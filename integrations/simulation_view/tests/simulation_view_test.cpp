#include <gtest/gtest.h>
#include <robotics/integrations/simulation_view.hpp>
#include <robotics/sensors/models.hpp>
#include <robotics/sensors/runtime.hpp>

using namespace robotics;
using namespace std::chrono_literals;

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
    visualization::LivePoseSource source({"sim", "simulation", "world", "body", "pose", 4, 16});
    for (int tick = 0; tick < 1000; ++tick) {
        if (tick == 500) {
            plain.reset(initial, 42);
            integrations::publishSimulationPose(source, viewed.reset(initial, 42));
        }
        plain.command(Eigen::VectorXd::Constant(1, 4));
        viewed.command(Eigen::VectorXd::Constant(1, 4));
        const auto left = plain.advance(), right = viewed.advance();
        integrations::publishSimulationPose(source, right);
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

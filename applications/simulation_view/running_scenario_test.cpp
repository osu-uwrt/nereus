#include "../simulator/telemetry.hpp"
#include "running_scenario.hpp"
#include <gtest/gtest.h>

using namespace robotics;

TEST(ScenarioExecution, UnpolledWorkerMatchesSynchronousScenarioAndDrainsSensors) {
    const auto scenario =
        config::loadScenario(std::filesystem::path(RP_CONTENT) / "examples/profile_pool.yaml");
    auto expected = config::makeRuntime(scenario);
    auto observers = runner::telemetry(*expected, scenario.sensors, nullptr);
    std::size_t command = 0;
    for (std::uint64_t tick = 0; tick < scenario.ticks; ++tick) {
        if (command < scenario.commands.size() && scenario.commands[command].tick == tick)
            expected->command(scenario.commands[command++].forces);
        expected->advance();
        for (const auto &observe : observers)
            observe();
    }
    auto source = std::make_shared<visualization::LivePoseSource>(
        visualization::LivePoseOptions{"simulation", "clock", "world", "body", "pose", 2, 4});
    runner::RunningScenario execution(scenario, source, false);
    execution.join();
    EXPECT_TRUE(execution.finished());
    const auto view = source->snapshot();
    ASSERT_TRUE(view.data);
    EXPECT_EQ(view.time_ns, expected->observe().elapsed.count());
    const auto &pose = view.data->streams.at("pose").back().pose;
    EXPECT_EQ(pose.translation, expected->observe().body.position);
    EXPECT_EQ(pose.rotation.coeffs(), expected->observe().body.orientation.coeffs());
    EXPECT_GT(view.delivery.dropped_queue, 0U);
}

TEST(ScenarioExecution, StopJoinsWithoutCompletingLongScenario) {
    auto scenario =
        config::loadScenario(std::filesystem::path(RP_CONTENT) / "examples/profile_pool.yaml");
    scenario.ticks = 10000000;
    auto source = std::make_shared<visualization::LivePoseSource>(
        visualization::LivePoseOptions{"simulation", "clock"});
    runner::RunningScenario execution(scenario, source);
    execution.stop();
    execution.join();
    EXPECT_TRUE(execution.finished());
    EXPECT_LT(source->snapshot().time_ns,
              static_cast<std::int64_t>(scenario.ticks) * scenario.plant.timestep.count());
}

TEST(ScenarioExecution, WorkerFailurePropagatesToOwner) {
    config::Scenario scenario;
    scenario.plant.timestep = std::chrono::nanoseconds(0);
    auto source = std::make_shared<visualization::LivePoseSource>(
        visualization::LivePoseOptions{"simulation", "clock"});
    runner::RunningScenario execution(scenario, source, false);
    EXPECT_THROW(execution.join(), std::invalid_argument);
    EXPECT_TRUE(execution.finished());
}

#include <robotics/session/scenario.hpp>

#include <gtest/gtest.h>

using robotics::session::loadResolvedScenario;

TEST(ResolvedScenario, LoadsTheTalosDocumentWithAbsoluteAssets) {
    const auto scenario = loadResolvedScenario(RP_RESOLVED_TALOS);
    EXPECT_EQ(scenario.robot.at("id"), "talos");
    EXPECT_EQ(scenario.task("gate").at("kind"), "task");
    EXPECT_TRUE(scenario.asset("robot", "body_mesh").is_absolute());
    EXPECT_THROW(scenario.asset("robot", "no_such_asset"), std::out_of_range);
    EXPECT_THROW(scenario.task("no_such_task"), std::out_of_range);
}

TEST(ResolvedScenario, RejectsOtherDocuments) {
    EXPECT_THROW(robotics::session::parseResolvedScenario({{"format", "other"}}), std::runtime_error);
    EXPECT_THROW(loadResolvedScenario("/nonexistent.json"), std::runtime_error);
}

#include <nereus/session/scenario.hpp>

#include <gtest/gtest.h>

using nereus::session::loadResolvedScenario;

TEST(ResolvedScenario, LoadsTheTalosDocumentWithAbsoluteAssets) {
    const auto scenario = loadResolvedScenario(NEREUS_RESOLVED_TALOS);
    EXPECT_EQ(scenario.robot.at("id"), "talos");
    EXPECT_EQ(scenario.task("gate").at("kind"), "task");
    EXPECT_TRUE(scenario.asset("robot", "body_mesh").is_absolute());
    EXPECT_THROW(scenario.asset("robot", "no_such_asset"), std::out_of_range);
    EXPECT_THROW(scenario.task("no_such_task"), std::out_of_range);
}

TEST(ResolvedScenario, RejectsOtherDocuments) {
    EXPECT_THROW(nereus::session::parseResolvedScenario({{"format", "other"}}), std::runtime_error);
    EXPECT_THROW(loadResolvedScenario("/nonexistent.json"), std::runtime_error);
}

TEST(PoolFloor, ProfiledPoolsGenerateFlaggedFloorBoxes) {
    using nereus::session::Json;
    auto pool = Json::parse(R"({
        "parameters": {"length_m": 20, "width_m": 8, "depth_m": 5, "water_level_m": 0},
        "collision_boxes": [{"id": "end_wall", "size_m": [1, 8, 6], "center_m": [20.5, 4, -2],
                             "orientation_wxyz": [1, 0, 0, 0]}]
    })");
    // Flat: the pool's own boxes, untouched.
    EXPECT_EQ(nereus::session::poolCollisionBoxes(pool), pool.at("collision_boxes"));
    EXPECT_TRUE(nereus::session::poolFloor(pool).isFlat());
    EXPECT_DOUBLE_EQ(nereus::session::poolFloor(pool).depthAt(10.0), 5);
    pool["parameters"]["floor_profile"] = {{"along", "x"}, {"points_m", {{0, 5}, {8, 5}, {14, 3}, {20, 3}}}};
    const auto floor = nereus::session::poolFloor(pool);
    EXPECT_DOUBLE_EQ(floor.depthAt(4.0), 5);
    EXPECT_DOUBLE_EQ(floor.depthAt(17.0), 3);
    const auto boxes = nereus::session::poolCollisionBoxes(pool);
    ASSERT_EQ(boxes.size(), 1 + floor.polyline().size() - 1);
    EXPECT_EQ(boxes[0].at("id"), "end_wall");
    for (std::size_t k = 1; k < boxes.size(); ++k) {
        EXPECT_TRUE(boxes[k].at("floor").get<bool>());
        EXPECT_EQ(boxes[k].at("id"), "floor_" + std::to_string(k - 1));
        EXPECT_DOUBLE_EQ(boxes[k].at("size_m")[1].get<double>(), 8);
    }
}

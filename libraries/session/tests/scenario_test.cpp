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
    EXPECT_DOUBLE_EQ(nereus::session::poolFloor(pool).depthAt({10, 4}), 5);
    pool["parameters"]["floor_profile"] = {{"along", "x"}, {"points_m", {{0, 5}, {8, 5}, {14, 3}, {20, 3}}}};
    const auto floor = nereus::session::poolFloor(pool);
    EXPECT_DOUBLE_EQ(floor.depthAt({4, 4}), 5);
    EXPECT_DOUBLE_EQ(floor.depthAt({17, 4}), 3);
    const auto boxes = nereus::session::poolCollisionBoxes(pool);
    ASSERT_EQ(boxes.size(), 1 + floor.profiles()[0].polyline().size() - 1);
    EXPECT_EQ(boxes[0].at("id"), "end_wall");
    for (std::size_t k = 1; k < boxes.size(); ++k) {
        EXPECT_TRUE(boxes[k].at("floor").get<bool>());
        EXPECT_EQ(boxes[k].at("id"), "floor_" + std::to_string(k - 1));
        EXPECT_DOUBLE_EQ(boxes[k].at("size_m")[1].get<double>(), 8);
    }
    // A list of profiles: the floor is the shallowest, with every profile's boxes.
    pool["parameters"]["floor_profile"] = Json::array(
        {pool["parameters"]["floor_profile"], {{"along", "y"}, {"points_m", {{0, 5}, {3, 5}, {6, 4}, {8, 4}}}}});
    const auto both = nereus::session::poolFloor(pool);
    EXPECT_DOUBLE_EQ(both.depthAt({4, 7}), 4);
    EXPECT_EQ(nereus::session::poolCollisionBoxes(pool).size(),
              1 + both.profiles()[0].polyline().size() - 1 + both.profiles()[1].polyline().size() - 1);
}

TEST(PoolFloor, FixtureBoxesRestOnTheFloorAndCanCollide) {
    using nereus::session::Json;
    const auto pool = Json::parse(R"({
        "parameters": {"length_m": 20, "width_m": 8, "depth_m": 5, "water_level_m": 0.5,
                       "floor_profile": {"along": "x", "points_m": [[0, 5], [10, 5], [20, 3]]}},
        "collision_boxes": [],
        "fixtures": [
            {"id": "grate", "type": "box", "center_m": [15, 2], "size_m": [1.2, 0.6, 0.04], "yaw_deg": 90,
             "contact": true},
            {"id": "tread", "type": "box", "center_m": [5, 0.2, -0.6], "size_m": [1, 0.4, 0.05]},
            {"id": "stairs", "type": "recess", "wall": "y_min", "from": [4, -1], "to": [6, 0.3], "depth_m": 0.4}
        ]
    })");
    const auto boxes = nereus::session::poolFixtureBoxes(pool);
    ASSERT_EQ(boxes.size(), 2u);
    // On the sloped floor: its bottom at the floor under its centre (z relative to the pool frame).
    const double floor = 0.5 - nereus::session::poolFloor(pool).depthAt({15, 2});
    EXPECT_NEAR(boxes[0].center.z() - boxes[0].size.z() / 2, floor, 1e-12);
    EXPECT_TRUE(boxes[0].on_floor);
    EXPECT_NEAR(boxes[0].yaw, M_PI / 2, 1e-12);
    EXPECT_DOUBLE_EQ(boxes[1].center.z(), 0.5 - 0.6); // given z is relative to the water surface
    EXPECT_FALSE(boxes[1].on_floor);
    // Only the contact box collides; the profiled floor's boxes follow it.
    const auto contacts = nereus::session::poolCollisionBoxes(pool);
    ASSERT_GE(contacts.size(), 2u);
    EXPECT_EQ(contacts[0].at("id"), "grate");
    EXPECT_NEAR(contacts[0].at("orientation_wxyz")[3].get<double>(), std::sin(M_PI / 4), 1e-12);
    for (std::size_t k = 1; k < contacts.size(); ++k)
        EXPECT_TRUE(contacts[k].at("floor").get<bool>());
}

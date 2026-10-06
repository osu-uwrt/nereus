// Tests for building the pool model (floor profile, contact boxes, fixtures) from pool pack JSON.
#include <nereus/session/pool.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <string>

using nereus::session::Json;
using nereus::session::poolFloor;
using nereus::session::poolModel;

// A flat pool keeps its own boxes; a floor_profile replaces the floor box with one box per profile segment.
TEST(PoolFloor, ProfiledPoolsGenerateFlaggedFloorBoxes) {
    auto pool = Json::parse(R"({
        "parameters": {"length_m": 20, "width_m": 8, "depth_m": 5, "water_level_m": 0},
        "collision_boxes": [{"id": "end_wall", "size_m": [1, 8, 6], "center_m": [20.5, 4, -2],
                             "orientation_wxyz": [1, 0, 0, 0]},
                            {"id": "floor", "size_m": [20, 8, 1], "center_m": [10, 4, -5.5],
                             "orientation_wxyz": [1, 0, 0, 0]}]
    })");
    // Flat: the pool's own boxes, untouched; the one topped at the floor depth is the floor.
    const auto flat = poolModel(pool);
    ASSERT_EQ(flat.contacts.size(), 2u);
    EXPECT_EQ(flat.contacts[0].id, "end_wall");
    EXPECT_EQ(flat.contacts[0].size, Eigen::Vector3d(1, 8, 6));
    EXPECT_EQ(flat.contacts[0].center, Eigen::Vector3d(20.5, 4, -2));
    EXPECT_FALSE(flat.contacts[0].floor);
    EXPECT_TRUE(flat.contacts[1].floor);
    EXPECT_FALSE(flat.profiled);
    EXPECT_TRUE(flat.floor.isFlat());
    EXPECT_DOUBLE_EQ(flat.floor.depthAt({10, 4}), 5);
    EXPECT_DOUBLE_EQ(poolFloor(pool).depthAt({10, 4}), 5);

    // Replace the floor box with a profile along x: one generated floor box per profile segment.
    pool["collision_boxes"].erase(1);
    pool["parameters"]["floor_profile"] = {{"along", "x"}, {"points_m", {{0, 5}, {8, 5}, {14, 3}, {20, 3}}}};
    const auto model = poolModel(pool);
    const auto &floor = model.floor;
    EXPECT_TRUE(model.profiled);
    EXPECT_DOUBLE_EQ(floor.depthAt({4, 4}), 5);
    EXPECT_DOUBLE_EQ(floor.depthAt({17, 4}), 3);
    const auto &boxes = model.contacts;
    ASSERT_EQ(boxes.size(), 1 + floor.profiles()[0].polyline().size() - 1);
    EXPECT_EQ(boxes[0].id, "end_wall");
    EXPECT_FALSE(boxes[0].floor);
    for (std::size_t k = 1; k < boxes.size(); ++k) {
        EXPECT_TRUE(boxes[k].floor);
        EXPECT_EQ(boxes[k].id, "floor_" + std::to_string(k - 1));
        EXPECT_DOUBLE_EQ(boxes[k].size.y(), 8);
    }
    // A list of profiles: the floor is the shallowest, with every profile's boxes.
    pool["parameters"]["floor_profile"] = Json::array(
        {pool["parameters"]["floor_profile"], {{"along", "y"}, {"points_m", {{0, 5}, {3, 5}, {6, 4}, {8, 4}}}}});
    const auto both = poolModel(pool);
    EXPECT_DOUBLE_EQ(both.floor.depthAt({4, 7}), 4);
    EXPECT_EQ(both.contacts.size(),
              1 + both.floor.profiles()[0].polyline().size() - 1 + both.floor.profiles()[1].polyline().size() - 1);
}

// Fixture boxes and meshes without a z sit on the (sloped) floor; only fixtures with contact: true collide.
TEST(PoolFloor, FixtureBoxesRestOnTheFloorAndCanCollide) {
    const auto pool = Json::parse(R"({
        "parameters": {"length_m": 20, "width_m": 8, "depth_m": 5, "water_level_m": 0.5,
                       "floor_profile": {"along": "x", "points_m": [[0, 5], [10, 5], [20, 3]]}},
        "collision_boxes": [],
        "fixtures": [
            {"id": "grate", "type": "box", "center_m": [15, 2], "size_m": [1.2, 0.6, 0.04], "rpy_deg": [0, 0, 90],
             "contact": true},
            {"id": "tread", "type": "box", "center_m": [5, 0.2, -0.6], "size_m": [1, 0.4, 0.05]},
            {"id": "stairs", "type": "recess", "wall": "y_min", "from": [4, -1], "to": [6, 0.3], "depth_m": 0.4},
            {"id": "rail", "type": "mesh", "asset": "rail", "center_m": [5, 0.2]}
        ]
    })");
    const auto model = poolModel(pool);
    EXPECT_DOUBLE_EQ(model.surface_z, 0.5);
    const auto &boxes = model.boxes;
    ASSERT_EQ(boxes.size(), 2u);
    // On the sloped floor: its bottom at the floor under its center (z relative to the pool frame).
    const double floor = 0.5 - model.floor.depthAt({15, 2});
    EXPECT_NEAR(boxes[0].center.z() - boxes[0].size.z() / 2, floor, 1e-12);
    EXPECT_TRUE(boxes[0].on_floor);
    EXPECT_TRUE(boxes[0].orientation.isApprox(Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())),
                                              1e-12));
    EXPECT_DOUBLE_EQ(boxes[1].center.z(), 0.5 - 0.6); // given z is relative to the water surface
    EXPECT_FALSE(boxes[1].on_floor);
    // A mesh given only x, y has its origin on the floor.
    ASSERT_EQ(model.meshes.size(), 1u);
    EXPECT_EQ(model.meshes[0].asset, "rail");
    EXPECT_TRUE(model.meshes[0].on_floor);
    EXPECT_DOUBLE_EQ(model.meshes[0].center.z(), 0.5 - model.floor.depthAt({5, 0.2}));
    // Only the contact box collides; the profiled floor's boxes follow it.
    const auto &contacts = model.contacts;
    ASSERT_GE(contacts.size(), 2u);
    EXPECT_EQ(contacts[0].id, "grate");
    EXPECT_FALSE(contacts[0].floor);
    EXPECT_NEAR(contacts[0].orientation.z(), std::sin(M_PI / 4), 1e-12);
    for (std::size_t k = 1; k < contacts.size(); ++k)
        EXPECT_TRUE(contacts[k].floor);
}

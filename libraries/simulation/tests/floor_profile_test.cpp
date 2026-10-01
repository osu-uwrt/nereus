#include <nereus/simulation/floor_profile.hpp>
#include <nereus/simulation/plant.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

using nereus::simulation::FloorProfile;
using Axis = FloorProfile::Axis;

namespace {
// Deep flat, a curved rise, shallow flat: the shape of a dive well floor.
FloorProfile well() {
    return FloorProfile::smooth(Axis::X, {{0, 5}, {5, 5}, {10, 3}, {20, 3}});
}
} // namespace

TEST(FloorProfile, SmoothPassesThroughPointsKeepsFlatsFlatAndNeverOvershoots) {
    const auto floor = well();
    EXPECT_DOUBLE_EQ(floor.depthAt(0.0), 5);
    EXPECT_DOUBLE_EQ(floor.depthAt(2.5), 5);
    EXPECT_DOUBLE_EQ(floor.depthAt(5.0), 5);
    EXPECT_DOUBLE_EQ(floor.depthAt(10.0), 3);
    EXPECT_DOUBLE_EQ(floor.depthAt(15.0), 3);
    EXPECT_DOUBLE_EQ(floor.extent(), 20);
    EXPECT_DOUBLE_EQ(floor.maxDepth(), 5);
    EXPECT_DOUBLE_EQ(floor.minDepth(), 3);
    // Zero slope where the curve meets each flat, symmetric S in between.
    EXPECT_NEAR(floor.depthAt(5.05), 5, 2e-3);
    EXPECT_NEAR(floor.depthAt(9.95), 3, 2e-3);
    EXPECT_NEAR(floor.depthAt(7.5), 4, 2e-3);
    double previous = floor.depthAt(0.0);
    for (double s = 0; s <= 20; s += .01) {
        const double depth = floor.depthAt(s);
        EXPECT_LE(depth, previous + 1e-12) << s;
        EXPECT_GE(depth, 3 - 1e-12) << s;
        previous = depth;
    }
    // The flats are single segments; the curve is sampled at 5 cm or better.
    const auto &line = floor.polyline();
    EXPECT_EQ(line[1].x(), 5);
    EXPECT_EQ(line[line.size() - 2].x(), 10);
    EXPECT_LE(line.size(), 104u);
}

TEST(FloorProfile, FlatProfilesCollapseAndTwoPointsAreLinear) {
    const auto flat = FloorProfile::smooth(Axis::X, {{0, 3}, {10, 3}, {20, 3}});
    EXPECT_EQ(flat.polyline().size(), 2u);
    EXPECT_TRUE(flat.isFlat());
    EXPECT_TRUE(FloorProfile::flat(3, 20).isFlat());
    const auto ramp = FloorProfile::smooth(Axis::Y, {{0, 5}, {10, 4}});
    EXPECT_EQ(ramp.polyline().size(), 2u);
    EXPECT_FALSE(ramp.isFlat());
    EXPECT_DOUBLE_EQ(ramp.depthAt(5.0), 4.5);
    EXPECT_DOUBLE_EQ(ramp.depthAt(Eigen::Vector2d(100, 2.5)), 4.75); // along y
    EXPECT_DOUBLE_EQ(ramp.depthAt(-1.0), 5);                         // clamped
    EXPECT_DOUBLE_EQ(ramp.depthAt(11.0), 4);
}

TEST(FloorProfile, RejectsMalformedPoints) {
    EXPECT_THROW(FloorProfile::smooth(Axis::X, {{0, 5}}), std::invalid_argument);
    EXPECT_THROW(FloorProfile::smooth(Axis::X, {{1, 5}, {10, 4}}), std::invalid_argument);
    EXPECT_THROW(FloorProfile::smooth(Axis::X, {{0, 5}, {10, 4}, {10, 3}}), std::invalid_argument);
    EXPECT_THROW(FloorProfile::smooth(Axis::X, {{0, 5}, {10, 0}}), std::invalid_argument);
    EXPECT_THROW(FloorProfile::smooth(Axis::X, {{0, 5}, {10, NAN}}), std::invalid_argument);
    EXPECT_THROW(FloorProfile::flat(-1, 10), std::invalid_argument);
}

TEST(FloorProfile, RaysHitTheFloorTheyCross) {
    // Flat: the plane arithmetic of a flat pool.
    const auto flat = FloorProfile::flat(2, 10);
    EXPECT_DOUBLE_EQ(*flat.rayDistance({5, 1, -.5}, {0, 0, -1}, 0), 1.5);
    EXPECT_FALSE(flat.rayDistance({5, 1, -.5}, {0, 0, 1}, 0));
    EXPECT_FALSE(flat.rayDistance({5, 1, -.5}, {1, 0, 0}, 0));

    const auto floor = well();
    // Straight down onto the slope; surface raised to 1.
    EXPECT_DOUBLE_EQ(*floor.rayDistance({7.5, 3, -1}, {0, 0, -1}, 1), floor.depthAt(7.5) - 2);
    // A shallow ray heading up the slope meets it before it could reach the deep floor.
    const Eigen::Vector3d origin(.5, 1, -1), direction = Eigen::Vector3d(1, 0, -.3).normalized();
    const double t = *floor.rayDistance(origin, direction, 0);
    const Eigen::Vector3d hit = origin + t * direction;
    EXPECT_GT(hit.x(), 5);
    EXPECT_LT(hit.x(), 10);
    EXPECT_NEAR(hit.z(), -floor.depthAt(hit.x()), 1e-9);
    // Heading down the slope from the shallow end it lands on the deep flat.
    const Eigen::Vector3d back(19, 1, -.5), down = Eigen::Vector3d(-1, 0, -.4).normalized();
    const Eigen::Vector3d landed = back + *floor.rayDistance(back, down, 0) * down;
    EXPECT_NEAR(landed.z(), -floor.depthAt(landed.x()), 1e-9);
    // Past either end the floor continues flat.
    const Eigen::Vector3d out(19, 1, -.5), away = Eigen::Vector3d(1, 0, -.1).normalized();
    EXPECT_NEAR((out + *floor.rayDistance(out, away, 0) * away).z(), -3, 1e-9);
}

TEST(FloorProfile, FloorBoxesTopFacesFollowTheProfile) {
    const auto floor = well();
    const auto boxes = nereus::simulation::floorBoxes(floor, 8, 0, 1, .01);
    ASSERT_EQ(boxes.size(), floor.polyline().size() - 1);
    for (std::size_t k = 0; k < boxes.size(); ++k) {
        const auto &a = floor.polyline()[k], &b = floor.polyline()[k + 1];
        const auto &box = boxes[k];
        const Eigen::Matrix3d r = box.orientation.toRotationMatrix();
        const Eigen::Vector3d top = box.center + r.col(2) * box.size.z() / 2;
        EXPECT_NEAR(top.x(), (a.x() + b.x()) / 2, 1e-9);
        EXPECT_NEAR(top.y(), 4, 1e-9);
        EXPECT_NEAR(top.z(), -(a.y() + b.y()) / 2, 1e-9);
        EXPECT_GT(r.col(2).z(), .8); // faces up (this test floor is steep: ~31 degrees at most)
        EXPECT_NEAR(box.size.x(), std::hypot(b.x() - a.x(), b.y() - a.y()) + .02, 1e-9);
        EXPECT_DOUBLE_EQ(box.size.y(), 8);
    }
    // A flat floor is one upright box; along y the boxes run along y.
    const auto flat = nereus::simulation::floorBoxes(FloorProfile::flat(2, 10), 6, 0);
    ASSERT_EQ(flat.size(), 1u);
    EXPECT_TRUE(flat[0].orientation.isApprox(Eigen::Quaterniond::Identity()));
    EXPECT_TRUE(flat[0].center.isApprox(Eigen::Vector3d(5, 3, -2.5)));
    const auto alongY = nereus::simulation::floorBoxes(FloorProfile::smooth(Axis::Y, {{0, 5}, {10, 4}}), 6, 0);
    ASSERT_EQ(alongY.size(), 1u);
    EXPECT_NEAR((alongY[0].orientation * Eigen::Vector3d::UnitX()).y(), std::cos(std::atan(.1)), 1e-9);
    EXPECT_NEAR(alongY[0].center.x(), 3, 1e-9);
}

TEST(FloorProfile, PlantAcceptsAProfiledPoolOnlyWithBoxContacts) {
    using namespace nereus::simulation;
    PlantParameters p;
    p.pool.length = 20;
    p.pool.width = 8;
    p.pool.depth = 5;
    p.pool.floor = nereus::simulation::PoolFloor({well()});
    p.contacts.model = ContactModel::Disabled;
    BodyState start;
    start.position = {2, 2, -1};
    EXPECT_NO_THROW(Plant(p, start));
    p.contacts.model = ContactModel::SpherePool;
    EXPECT_THROW(Plant(p, start), std::invalid_argument);
    p.contacts.model = ContactModel::Disabled;
    p.pool.depth = 6; // not the profile's deepest point
    EXPECT_THROW(Plant(p, start), std::invalid_argument);
    p.pool.depth = 5;
    p.pool.length = 25; // profile does not span the pool
    EXPECT_THROW(Plant(p, start), std::invalid_argument);
}

TEST(PoolFloor, TheShallowestProfileIsTheFloor) {
    using nereus::simulation::PoolFloor;
    // Deep beside y = 0 at the x = 0 end, rising along x (to 3 m) and across y (to 4 m).
    const PoolFloor floor({well(), FloorProfile::smooth(Axis::Y, {{0, 5}, {4, 5}, {7, 4}, {8, 4}})});
    EXPECT_DOUBLE_EQ(floor.depthAt({2, 2}), 5);   // the deep flat
    EXPECT_DOUBLE_EQ(floor.depthAt({2, 7.5}), 4); // up the cross slope
    EXPECT_DOUBLE_EQ(floor.depthAt({17, 2}), 3);  // up the long slope
    EXPECT_DOUBLE_EQ(floor.depthAt({17, 7.5}), 3);
    EXPECT_DOUBLE_EQ(floor.maxDepth(), 5);
    EXPECT_FALSE(floor.isFlat());
    EXPECT_TRUE(PoolFloor::flat(2, 10).isFlat());
    // A ray meets whichever surface it reaches first.
    EXPECT_DOUBLE_EQ(*floor.rayDistance({2, 7.5, -1}, {0, 0, -1}, 0), 3);
    const Eigen::Vector3d origin(2, 2, -1), across = Eigen::Vector3d(0, 1, -.4).normalized();
    const Eigen::Vector3d hit = origin + *floor.rayDistance(origin, across, 0) * across;
    EXPECT_NEAR(hit.z(), -floor.depthAt(hit.head<2>()), 1e-9);
    EXPECT_GT(hit.y(), 4);
    // Contact boxes: every profile's, each spanning the pool across its axis.
    const auto boxes = nereus::simulation::floorBoxes(floor, 20, 8, 0);
    ASSERT_EQ(boxes.size(), floor.profiles()[0].polyline().size() - 1 + floor.profiles()[1].polyline().size() - 1);
    EXPECT_DOUBLE_EQ(boxes.front().size.y(), 8);
    EXPECT_DOUBLE_EQ(boxes.back().size.y(), 20);
}

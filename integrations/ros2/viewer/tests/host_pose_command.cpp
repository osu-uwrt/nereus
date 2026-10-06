// Keyboard pose commands: the palette's moves and the keyboard-drive steps, from the pose last commanded.
#include "../src/pose_command.hpp"
#include <gtest/gtest.h>

using namespace nereus::ros_viewer::host;

namespace {
PoseTarget facingNorth() { // at (1, 2, -1), heading +y
    PoseTarget pose;
    pose.position = {1, 2, -1};
    pose.degrees = {0, 0, 90};
    return pose;
}

void expectNear(const glm::vec3 &a, const glm::vec3 &b) {
    EXPECT_NEAR(a.x, b.x, 1e-4f);
    EXPECT_NEAR(a.y, b.y, 1e-4f);
    EXPECT_NEAR(a.z, b.z, 1e-4f);
}
} // namespace

// Only text starting with a move word is treated as a move; incomplete moves report how to finish them.
TEST(PoseCommand, OnlyMoveWordsAreMoves) {
    EXPECT_FALSE(parseMove("", facingNorth()).isMove);
    EXPECT_FALSE(parseMove("theme", facingNorth()).isMove);
    EXPECT_FALSE(parseMove("xyz", facingNorth()).isMove);

    // a move word without its number: a move, with how to finish it, and nothing to run
    const auto unfinished = parseMove("forward", facingNorth());
    EXPECT_TRUE(unfinished.isMove);
    EXPECT_FALSE(unfinished.command);
    EXPECT_NE(unfinished.error.find("forward 0.5"), std::string::npos);

    // a move word followed by words: not a runnable move either
    const auto words = parseMove("left panels", facingNorth());
    EXPECT_TRUE(words.isMove);
    EXPECT_FALSE(words.command);
}

// forward/right/down are relative to the current heading; comma-separated moves apply in order.
TEST(PoseCommand, RelativeMovesFollowTheHeading) {
    const auto forward = parseMove("forward 0.5", facingNorth());
    ASSERT_TRUE(forward.command);
    expectNear(forward.command->target.position, {1, 2.5f, -1});
    EXPECT_EQ(forward.command->summary, "forward 0.5 m");

    const auto right = parseMove("right 1m", facingNorth());
    ASSERT_TRUE(right.command);
    expectNear(right.command->target.position, {2, 2, -1});

    const auto down = parseMove("down 0.25", facingNorth());
    ASSERT_TRUE(down.command);
    expectNear(down.command->target.position, {1, 2, -1.25f});

    // in order: turn left 90 (now heading -x), then forward 1
    const auto chained = parseMove("turn 90, forward 1", facingNorth());
    ASSERT_TRUE(chained.command);
    expectNear(chained.command->target.position, {0, 2, -1});
    EXPECT_NEAR(chained.command->target.degrees.z, 180, 1e-3f);
    EXPECT_EQ(chained.command->summary, "turn 90°, forward 1 m");
}

// Yaw wraps into (-180, 180]: 90 - 100 = -10, 90 + 270 = 360 -> 0.
TEST(PoseCommand, TurnsWrapAndTakeADirection) {
    const auto right = parseMove("turn right 100", facingNorth());
    ASSERT_TRUE(right.command);
    EXPECT_NEAR(right.command->target.degrees.z, -10, 1e-3f);

    const auto around = parseMove("turn 270deg", facingNorth());
    ASSERT_TRUE(around.command);
    EXPECT_NEAR(around.command->target.degrees.z, 0, 1e-3f);
}

// Absolute axis words set single coordinates; "go"/"goto" take x y z, then optionally yaw or roll pitch yaw.
TEST(PoseCommand, AbsoluteAxesAndGo) {
    const auto depth = parseMove("z -1.5 yaw 180", facingNorth());
    ASSERT_TRUE(depth.command);
    expectNear(depth.command->target.position, {1, 2, -1.5f});
    EXPECT_NEAR(depth.command->target.degrees.z, 180, 1e-3f);

    const auto go = parseMove("go 3 4 -2 -90", facingNorth());
    ASSERT_TRUE(go.command);
    expectNear(go.command->target.position, {3, 4, -2});
    EXPECT_NEAR(go.command->target.degrees.z, -90, 1e-3f);

    const auto full = parseMove("goto 3 4 -2 5 -5 45", facingNorth());
    ASSERT_TRUE(full.command);
    expectNear(full.command->target.degrees, {5, -5, 45});

    // Too few or a partial attitude is rejected (3, 4 or 6 numbers only).
    EXPECT_FALSE(parseMove("go 3 4", facingNorth()).command);
    EXPECT_FALSE(parseMove("go 1 2 3 4 5", facingNorth()).command);
}

TEST(PoseCommand, LevelZeroesRollAndPitch) {
    PoseTarget tilted = facingNorth();
    tilted.degrees = {12, -7, 30};
    const auto level = parseMove("level", tilted);
    ASSERT_TRUE(level.command);
    expectNear(level.command->target.degrees, {0, 0, 30});
    expectNear(level.command->target.position, tilted.position);
}

// Roll and pitch only appear in the description when the target is not level.
TEST(PoseCommand, DescribesTheTarget) {
    EXPECT_EQ(describe(facingNorth()), "x 1.00  y 2.00  z -1.00  yaw 90°");
    PoseTarget tilted = facingNorth();
    tilted.degrees.x = 10;
    EXPECT_EQ(describe(tilted), "x 1.00  y 2.00  z -1.00  roll 10°  pitch 0°  yaw 90°");
}

// stepped(forward, left, up, turn): heading +y puts "left" along -x; a negative turn is to the right.
TEST(PoseCommand, DriveStepsAlongTheHeading) {
    const auto step = stepped(facingNorth(), 0.25f, 0, 0, 0);
    expectNear(step.position, {1, 2.25f, -1});

    const auto strafe = stepped(facingNorth(), 0, 0.25f, 0.1f, -15);
    expectNear(strafe.position, {0.75f, 2, -0.9f});
    EXPECT_NEAR(strafe.degrees.z, 75, 1e-3f);
}

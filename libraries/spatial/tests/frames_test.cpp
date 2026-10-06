// Tests for fixed-frame resolution, lookups and pose validation.
#include <gtest/gtest.h>
#include <limits>
#include <nereus/spatial/frames.hpp>

using namespace nereus::spatial;

TEST(FixedFrames, ResolvesUnorderedMountsAndRelativeRotatedFrames) {
    const Pose tool{{.4, -.2, .1}, Eigen::Quaterniond(Eigen::AngleAxisd(.7, Eigen::Vector3d::UnitZ()))};
    const Pose sensor{{.2, 0, -.1}, Eigen::Quaterniond(Eigen::AngleAxisd(-.4, Eigen::Vector3d::UnitY()))};
    FixedFrames frames("center", {{"tool", "sensor", sensor}, {"center", "tool", tool}});
    const Eigen::Vector3d point(.3, .2, .7);
    const Eigen::Vector3d expected = tool.translation + tool.rotation * (sensor.translation + sensor.rotation * point);
    EXPECT_TRUE(apply(frames.fromRoot("sensor"), point).isApprox(expected, 1e-14));
    EXPECT_TRUE(apply(frames.lookup("tool", "sensor"), point).isApprox(apply(sensor, point), 1e-14));
    EXPECT_TRUE(apply(frames.lookup("sensor", "center"), expected).isApprox(point, 1e-14));
    EXPECT_TRUE(frames.fromRoot("center").translation.isZero());
    EXPECT_EQ(frames.edges().front().child, "sensor"); // Original declarative order retained.
    EXPECT_THROW(frames.fromRoot("missing"), std::invalid_argument);
}

TEST(FixedFrames, RejectsCyclesDisconnectedOrAmbiguousFramesAndBadPoses) {
    EXPECT_THROW((FixedFrames("", {})), std::invalid_argument);
    EXPECT_THROW((FixedFrames("root", {{"other", "a", {}}})), std::invalid_argument);
    EXPECT_THROW((FixedFrames("root", {{"a", "b", {}}, {"b", "a", {}}})), std::invalid_argument);
    EXPECT_THROW((FixedFrames("root", {{"root", "a", {}}, {"root", "a", {}}})), std::invalid_argument);
    EXPECT_THROW((FixedFrames("root", {{"root", "root", {}}})), std::invalid_argument);
    Pose bad;
    bad.rotation.coeffs().setZero();
    EXPECT_THROW((FixedFrames("root", {{"root", "a", bad}})), std::invalid_argument);
    bad = {};
    bad.translation.x() = std::numeric_limits<double>::infinity();
    EXPECT_THROW((FixedFrames("root", {{"root", "a", bad}})), std::invalid_argument);
    bad.translation.x() = 6e11;
    EXPECT_THROW((FixedFrames("root", {{"root", "a", bad}, {"a", "b", bad}})), std::invalid_argument);
}

TEST(FixedFrames, NormalizesAcceptedRoundingWithoutMutatingInputs) {
    Pose almost_unit;
    almost_unit.rotation.w() = 1 + 5e-9;
    FixedFrames frames("root", {{"root", "a", almost_unit}, {"a", "b", almost_unit}});
    EXPECT_DOUBLE_EQ(frames.fromRoot("b").rotation.norm(), 1);
    EXPECT_DOUBLE_EQ(frames.edges().front().pose.rotation.norm(), 1);
    EXPECT_GT(almost_unit.rotation.norm(), 1);
}

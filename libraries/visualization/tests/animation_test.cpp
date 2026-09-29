#include <gtest/gtest.h>
#include <robotics/visualization/animation.hpp>

#include <fstream>
#include <limits>
#include <sstream>

namespace v = robotics::visualization;
namespace {
std::vector<std::vector<double>> rows(const std::string &name, std::size_t columns) {
    std::ifstream file(std::string(RP_ANIMATION_FIXTURES) + "/animation_reference_" + name +
                       ".csv");
    if (!file)
        throw std::runtime_error("missing original animation reference");
    std::vector<std::vector<double>> result;
    for (std::string line; std::getline(file, line);) {
        std::vector<double> row;
        std::istringstream input(line);
        for (std::string value; std::getline(input, value, ',');)
            row.push_back(std::stod(value));
        if (columns && row.size() != columns)
            throw std::runtime_error("invalid animation reference row");
        result.push_back(std::move(row));
    }
    return result;
}
v::RotorAnimation linear() {
    v::RotorAnimation config;
    config.curve.forward = config.curve.reverse = {0, 30. / 3.14159265358979323846, 0, 0};
    config.input_count = 2;
    config.rotors = {{0, -1}, {1, 1}};
    return config;
}
} // namespace
TEST(Animation, OriginalEightRotorPhasesAndLocalMatrices) {
    const auto source = rows("config", 0);
    ASSERT_EQ(source.size(), 9);
    ASSERT_EQ(source[0].size(), 12);
    v::RotorAnimation config;
    config.timeout = source[0][0];
    config.speed_scale = source[0][2];
    config.curve.deadband = source[0][1];
    config.input_count = 8;
    config.rotors.clear();
    std::vector<Eigen::Vector3d> pivots, axes;
    for (std::size_t i = 0; i < 4; ++i) {
        config.curve.forward[i] = source[0][4 + i];
        config.curve.reverse[i] = source[0][8 + i];
    }
    for (std::size_t i = 1; i < source.size(); ++i) {
        const auto &rotor = source[i];
        ASSERT_EQ(rotor.size(), 8);
        config.rotors.push_back({static_cast<std::size_t>(rotor[0]), rotor[1]});
        pivots.emplace_back(rotor[2], rotor[3], rotor[4]);
        axes.emplace_back(rotor[5], rotor[6], rotor[7]);
    }
    ASSERT_EQ(config.rotors.size(), 8);
    v::RotorAnimator animator(config);
    const auto reference = rows("rotors", 10 + 8 * 17);
    ASSERT_GE(reference.size(), 160);
    for (std::size_t row = 0; row < reference.size(); ++row) {
        SCOPED_TRACE(row);
        const auto &values = reference[row];
        if (values[0] == 1) {
            std::vector<float> forces;
            for (std::size_t i = 0; i < 8; ++i)
                forces.push_back(static_cast<float>(values[2 + i]));
            animator.receive(forces, values[1]);
        } else {
            animator.advance(values[1]);
        }
        for (std::size_t i = 0; i < 8; ++i) {
            EXPECT_NEAR(animator.angles()[i], values[10 + i * 17], 1e-10);
            const auto pose = v::pivotRotation(pivots[i], axes[i], animator.angles()[i]);
            Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
            matrix.topLeftCorner<3, 3>() = pose.rotation.toRotationMatrix();
            matrix.topRightCorner<3, 1>() = pose.translation;
            for (int j = 0; j < 16; ++j)
                EXPECT_NEAR(matrix.data()[j], values[11 + i * 17 + static_cast<std::size_t>(j)],
                            1e-6);
        }
    }
}
TEST(Animation, OriginalIndicatorModesPulseAndClamping) {
    const auto source = rows("config", 0);
    ASSERT_EQ(source.size(), 9);
    ASSERT_EQ(source[0].size(), 12);
    v::Indicator indicator;
    const auto reference = rows("lights", 9);
    ASSERT_GE(reference.size(), 390);
    for (std::size_t row = 0; row < reference.size(); ++row) {
        SCOPED_TRACE(row);
        const auto &values = reference[row];
        if (values[0] == 1)
            indicator.command({static_cast<float>(values[3]), static_cast<float>(values[4]),
                               static_cast<float>(values[5])},
                              static_cast<v::IndicatorMode>(static_cast<int>(values[1])), values[2],
                              source[0][3]);
        const auto color = indicator.color(values[2]);
        for (int axis = 0; axis < 3; ++axis)
            EXPECT_EQ(color[axis], static_cast<float>(values[6 + axis]));
    }
}
TEST(Animation, ReplacementsTimeoutPauseRewindAndRejectedPackets) {
    v::RotorAnimator animator(linear());
    animator.receive({2, -3}, 0);
    animator.advance(.1);
    EXPECT_NEAR(animator.angles()[0], -.2, 1e-14);
    EXPECT_NEAR(animator.angles()[1], -.3, 1e-14);
    animator.advance(.1);
    EXPECT_NEAR(animator.angles()[0], -.2, 1e-14);
    // Replace between draws; the old speed still covers [.1,.2].
    animator.receive({-2, 3}, .2);
    animator.advance(.3);
    EXPECT_NEAR(animator.angles()[0], -.2, 1e-14);
    EXPECT_THROW(animator.receive({2}, .4), std::invalid_argument);
    EXPECT_THROW(animator.receive({std::numeric_limits<float>::quiet_NaN(), 1}, .5),
                 std::invalid_argument);
    animator.advance(1); // Previous valid command expires at .7.
    EXPECT_NEAR(animator.angles()[0], .6, 1e-14);
    animator.advance(2);
    EXPECT_NEAR(animator.angles()[0], .6, 1e-14);
    animator.advance(0);
    animator.advance(.1);
    EXPECT_EQ(animator.angles(), (std::vector<double>{0, 0}));
    animator.receive({100, 100}, .1);
    animator.advance(.101);
    EXPECT_NEAR(animator.angles()[0], -.1, 1e-14); // No artificial speed cap.
    animator.reset();
    EXPECT_EQ(animator.angles(), (std::vector<double>{0, 0}));
    animator.advance(20);
    EXPECT_EQ(animator.angles(), (std::vector<double>{0, 0}));
}
TEST(Animation, PivotAndShaftRemainFixedAndModelsAreIndependent) {
    const Eigen::Vector3d pivot(.1, -.2, .3), axis(1, 2, 3);
    const auto pose = v::pivotRotation(pivot, axis, .7);
    EXPECT_TRUE(robotics::spatial::apply(pose, pivot).isApprox(pivot, 1e-14));
    EXPECT_TRUE(robotics::spatial::apply(pose, pivot + axis).isApprox(pivot + axis, 1e-14));
    v::RotorAnimator first(linear()), second(linear());
    first.receive({1, 1}, 0);
    first.advance(.1);
    second.advance(.1);
    EXPECT_NE(first.angles(), second.angles());
    EXPECT_THROW(v::pivotRotation(pivot, Eigen::Vector3d::Zero(), 0), std::invalid_argument);
    auto invalid = linear();
    invalid.rotors[0].index = 2;
    EXPECT_THROW(v::RotorAnimator{invalid}, std::invalid_argument);
}
TEST(Animation, IndicatorResetAndFailuresCannotResurrectPulse) {
    v::Indicator light;
    light.command({0, 1, 0}, v::IndicatorMode::Solid, 0);
    light.command({1, 0, 0}, v::IndicatorMode::Pulse, 1, .2);
    EXPECT_THROW(light.command({1, 1, 1}, v::IndicatorMode::Pulse, 1.1, -1), std::invalid_argument);
    EXPECT_TRUE(light.color(1.15).isApprox(Eigen::Vector3f(1, 0, 0)));
    EXPECT_TRUE(light.color(1.2).isApprox(Eigen::Vector3f(0, 1, 0)));
    EXPECT_THROW(
        light.command({0, 0, std::numeric_limits<float>::infinity()}, v::IndicatorMode::Solid, 1.3),
        std::invalid_argument);
    light.reset();
    EXPECT_TRUE(light.color(0).isZero());
    EXPECT_TRUE(light.color(1.1).isZero());
    EXPECT_THROW(light.color(-1), std::invalid_argument);
}

#include <array>
#include <fstream>
#include <gtest/gtest.h>
#include <limits>
#include <robotics/simulation/plant.hpp>
#include <sstream>

using namespace robotics::simulation;
namespace {
PlantParameters parameters() {
    PlantParameters p;
    p.body.inertia = Eigen::Vector3d(.3, .4, .5).asDiagonal();
    p.body.added_mass.diagonal() << 2, 3, 4, .1, .2, .3;
    p.body.linear_damping = Matrix6::Identity() * .3;
    p.body.quadratic_damping = Vector6::Constant(.1);
    p.pool.current_velocity = {.15, -.1, .05};
    p.pool.current_oscillation_amplitude = {.2, .1, .04};
    p.pool.current_oscillation_frequency = .7;
    Thruster t;
    t.id = "propeller";
    t.position = {.1, -.2, .025};
    t.delay = .004;
    t.propeller_radius = .05;
    p.thrusters.push_back(t);
    return p;
}
BodyState initial(int scenario) {
    BodyState state;
    state.position = {5, 5, scenario == 0 ? -.025 : scenario == 1 ? -2 : .2};
    state.orientation = Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitY());
    state.linear_velocity = {.1, -.05, .02};
    state.angular_velocity = {.3, .2, .1};
    return state;
}
} // namespace
TEST(StageReference, MatchesPinnedLegacySurfaceSubmergedAndEntryMotion) {
    std::ifstream fixture(std::string(NEREUS_FIXTURES) + "/legacy_stage_dynamics.csv");
    ASSERT_TRUE(fixture);
    std::string row;
    ASSERT_TRUE(std::getline(fixture, row));
    std::size_t rows = 0;
    for (int scenario = 0; scenario < 3; ++scenario) {
        Plant plant(parameters(), initial(scenario));
        for (int tick = 0; tick <= 250; ++tick) {
            ASSERT_TRUE(std::getline(fixture, row));
            std::istringstream fields(row);
            std::array<double, 22> expected{};
            for (auto &value : expected) {
                std::string field;
                ASSERT_TRUE(std::getline(fields, field, ','));
                value = std::stod(field);
            }
            EXPECT_EQ(expected[0], scenario);
            EXPECT_EQ(expected[1], tick);
            SCOPED_TRACE("case=" + std::to_string(scenario) + " tick=" + std::to_string(tick));
            const auto observed = plant.motion();
            const auto &body = observed.state.body;
            Eigen::Matrix<double, 20, 1> actual;
            actual << body.position, body.orientation.w(), body.orientation.x(), body.orientation.y(),
                body.orientation.z(), body.linear_velocity, body.angular_velocity, observed.state.thruster_forces,
                observed.acceleration_body, observed.angular_acceleration_body;
            for (int i = 0; i < actual.size(); ++i)
                EXPECT_NEAR(actual[i], expected[static_cast<std::size_t>(i) + 2], 2e-12);
            EXPECT_EQ(observed.state.elapsed.count(), tick * 2000000LL);
            ++rows;
            if (tick == 250)
                break;
            if (tick == 0 || tick == 100 || tick == 200)
                plant.command(Eigen::VectorXd::Constant(1, tick == 0 ? 12 : tick == 100 ? -6 : 0));
            plant.advance();
        }
    }
    EXPECT_EQ(rows, 753U);
    EXPECT_FALSE(std::getline(fixture, row));
}

TEST(StageReference, InvalidImmersionAndFlowParametersRejectBeforeRuntime) {
    auto p = parameters();
    p.thrusters[0].propeller_radius = 0;
    EXPECT_THROW(Plant(p, initial(0)), std::invalid_argument);
    p.thrusters[0].propeller_radius = std::numeric_limits<double>::infinity();
    EXPECT_THROW(Plant(p, initial(0)), std::invalid_argument);
    p = parameters();
    p.pool.current_oscillation_frequency = -1;
    EXPECT_THROW(Plant(p, initial(0)), std::invalid_argument);
    p.pool.current_oscillation_frequency = std::numeric_limits<double>::max();
    EXPECT_THROW(Plant(p, initial(0)), std::invalid_argument);
    p = parameters();
    p.pool.current_oscillation_amplitude.x() = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(Plant(p, initial(0)), std::invalid_argument);
}

TEST(StageReference, CurrentPhaseAndImmersionReplayAcrossBatchingAndReset) {
    Plant batched(parameters(), initial(0)), stepped(parameters(), initial(0));
    const auto force = Eigen::VectorXd::Constant(1, 12).eval();
    batched.command(force);
    stepped.command(force);
    const auto expected = batched.advance(100);
    for (int i = 0; i < 100; ++i)
        stepped.advance();
    EXPECT_EQ(expected.body.position, stepped.observe().body.position);
    EXPECT_EQ(expected.body.orientation.coeffs(), stepped.observe().body.orientation.coeffs());
    EXPECT_EQ(batched.motion().acceleration_body, stepped.motion().acceleration_body);
    batched.reset(initial(0));
    batched.command(force);
    const auto replay = batched.advance(100);
    EXPECT_EQ(expected.body.position, replay.body.position);
    EXPECT_EQ(expected.body.orientation.coeffs(), replay.body.orientation.coeffs());
    EXPECT_EQ(expected.body.linear_velocity, replay.body.linear_velocity);
    EXPECT_EQ(expected.body.angular_velocity, replay.body.angular_velocity);
    EXPECT_EQ(expected.thruster_forces, replay.thruster_forces);
    EXPECT_EQ(batched.motion().acceleration_body, stepped.motion().acceleration_body);
}

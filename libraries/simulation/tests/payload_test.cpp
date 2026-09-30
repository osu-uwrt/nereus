#include "robotics/simulation/payload.hpp"

#include <cmath>
#include <fstream>
#include <gtest/gtest.h>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {
using namespace robotics::simulation;
std::vector<std::string> split(const std::string &line) {
    std::istringstream stream(line);
    std::vector<std::string> fields;
    for (std::string field; std::getline(stream, field, ',');)
        fields.push_back(field);
    return fields;
}

TEST(Payload, MatchesEveryStateOfIndependentOriginalTrajectories) {
    // This checked capture executes the pinned original Python module, independently
    // of this implementation. Includes entry/exit, initial surface, neutral mass,
    // current, dry gyroscopic motion and angular-damping substeps for both models.
    std::ifstream input(NEREUS_PAYLOAD_FIXTURE);
    ASSERT_TRUE(input);
    PayloadParameters parameters;
    PayloadEnvironment environment;
    PayloadState actual;
    double dt = 0.0;
    int current_case = -1, next_tick = 0, states = 0, cases = 0;
    for (std::string line; std::getline(input, line);) {
        const auto fields = split(line);
        ASSERT_FALSE(fields.empty());
        if (fields[0] == "# case") {
            ASSERT_EQ(fields.size(), 22U);
            if (current_case >= 0) {
                ASSERT_EQ(next_tick, 101);
            }
            current_case = std::stoi(fields[1]);
            ASSERT_EQ(current_case, cases++);
            parameters.model =
                std::stoi(fields[2]) ? PayloadModel::Finned : PayloadModel::FixedAxis;
            parameters.neutral_buoyancy = std::stoi(fields[3]) != 0;
            const std::vector<double *> targets{&parameters.mass,
                                                &parameters.displaced_volume,
                                                &parameters.added_mass,
                                                &parameters.length,
                                                &parameters.radius,
                                                &parameters.drag_axial,
                                                &parameters.drag_lateral,
                                                &parameters.center_of_mass,
                                                &parameters.center_of_buoyancy,
                                                &parameters.center_of_drag,
                                                &parameters.angular_damping,
                                                &parameters.spring_energy,
                                                &environment.water_density,
                                                &environment.water_level,
                                                &environment.water_velocity.x(),
                                                &environment.water_velocity.y(),
                                                &environment.water_velocity.z(),
                                                &dt};
            for (std::size_t i = 0; i < targets.size(); ++i)
                *targets[i] = std::stod(fields[4 + i]);
            next_tick = 0;
            continue;
        }
        if (fields[0][0] == '#')
            continue;
        ASSERT_GE(current_case, 0);
        ASSERT_EQ(fields.size(), 20U);
        ASSERT_EQ(std::stoi(fields[0]), current_case);
        ASSERT_EQ(std::stoi(fields[1]), next_tick);
        SCOPED_TRACE("case=" + fields[0] + " tick=" + fields[1]);
        Eigen::Matrix<double, 18, 1> expected;
        for (int i = 0; i < 18; ++i) {
            expected[i] = std::stod(fields[2 + i]);
            ASSERT_TRUE(std::isfinite(expected[i]));
        }
        const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> expected_rotation(
            expected.data() + 6);
        if (next_tick == 0) {
            actual.position = expected.head<3>();
            actual.velocity = expected.segment<3>(3);
            actual.orientation = Eigen::Quaterniond(expected_rotation).normalized();
            actual.angular_velocity = expected.tail<3>();
        } else {
            actual = PayloadDynamics(parameters).advance(actual, environment, dt);
        }
        EXPECT_TRUE(actual.position.isApprox(expected.head<3>(), 1e-10));
        EXPECT_LE((actual.position - expected.head<3>()).cwiseAbs().maxCoeff(), 1e-10);
        EXPECT_LE((actual.velocity - expected.segment<3>(3)).cwiseAbs().maxCoeff(), 1e-10);
        EXPECT_LE((actual.orientation.toRotationMatrix() - expected_rotation).cwiseAbs().maxCoeff(),
                  1e-10);
        EXPECT_LE((actual.angular_velocity - expected.tail<3>()).cwiseAbs().maxCoeff(), 1e-10);
        EXPECT_NEAR(actual.orientation.norm(), 1.0, 2e-15);
        ++next_tick;
        ++states;
    }
    EXPECT_EQ(cases, 16);
    EXPECT_EQ(next_tick, 101);
    EXPECT_EQ(states, 1616);
}

TEST(Payload, HelpersUseEffectiveMassAndCapsuleSupport) {
    PayloadParameters parameters;
    parameters.mass = .01222;
    parameters.displaced_volume = 1.2e-5;
    parameters.added_mass = .003;
    parameters.spring_energy = .047;
    parameters.length = .08299993;
    parameters.radius = .013;
    PayloadEnvironment environment;
    environment.water_density = 998.2;
    const PayloadDynamics dynamics(parameters);
    EXPECT_DOUBLE_EQ(dynamics.launch_speed(environment), std::sqrt(2.0 * .047 / (.01222 + .003)));
    parameters.neutral_buoyancy = true;
    EXPECT_DOUBLE_EQ(PayloadDynamics(parameters).launch_speed(environment),
                     std::sqrt(2.0 * .047 / (998.2 * 1.2e-5 + .003)));
    const Eigen::Vector3d axis = Eigen::Vector3d(1., -2., 3.).normalized();
    EXPECT_TRUE(dynamics.support_extent(axis).isApprox(
        Eigen::Vector3d::Constant(.013) + (.08299993 / 2. - .013) * axis.cwiseAbs(), 1e-15));
    parameters.mass = 12.0;
    EXPECT_DOUBLE_EQ(dynamics.parameters().mass, .01222); // Owns parameter copy.
}

TEST(Payload, DryFreeFallAndFixedAxisDoNotIntegrateAngularVelocity) {
    PayloadParameters parameters;
    PayloadState input;
    input.position = Eigen::Vector3d(2., -1., 5.);
    input.velocity = Eigen::Vector3d(1., 2., 3.);
    input.angular_velocity = Eigen::Vector3d(3., -2., 1.);
    input.orientation = Eigen::AngleAxisd(.6, Eigen::Vector3d::UnitY());
    const auto output = PayloadDynamics(parameters).advance(input, {}, .1);
    EXPECT_TRUE(output.position.isApprox(
        input.position + .1 * input.velocity + Eigen::Vector3d(0., 0., -.04903325), 1e-15));
    EXPECT_TRUE(
        output.velocity.isApprox(input.velocity + Eigen::Vector3d(0., 0., -.980665), 1e-15));
    EXPECT_TRUE(output.orientation.isApprox(input.orientation));
    EXPECT_EQ(output.angular_velocity, input.angular_velocity);
    EXPECT_EQ(input.position, Eigen::Vector3d(2., -1., 5.));
}

TEST(Payload, InvalidRequestsAreRejectedWithoutMutation) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    PayloadParameters parameters;
    parameters.mass = 0.;
    EXPECT_THROW(PayloadDynamics{parameters}, std::invalid_argument);
    parameters = {};
    parameters.radius = 1.;
    EXPECT_THROW(PayloadDynamics{parameters}, std::invalid_argument);
    parameters = {};
    parameters.angular_damping = -1.;
    EXPECT_THROW(PayloadDynamics{parameters}, std::invalid_argument);
    parameters = {};
    parameters.center_of_drag = nan;
    EXPECT_THROW(PayloadDynamics{parameters}, std::invalid_argument);
    parameters = {};
    parameters.model = static_cast<PayloadModel>(42);
    EXPECT_THROW(PayloadDynamics{parameters}, std::invalid_argument);
    parameters = {};
    const PayloadDynamics dynamics(parameters);
    PayloadState state;
    PayloadEnvironment environment;
    EXPECT_THROW(dynamics.advance(state, environment, 0.), std::invalid_argument);
    EXPECT_THROW(dynamics.advance(state, environment, -1.), std::invalid_argument);
    EXPECT_THROW(dynamics.advance(state, environment, inf), std::invalid_argument);
    environment.water_density = nan;
    EXPECT_THROW(dynamics.advance(state, environment, .1), std::invalid_argument);
    environment = {};
    state.orientation.coeffs() *= 2.;
    EXPECT_THROW(dynamics.advance(state, environment, .1), std::invalid_argument);
    EXPECT_DOUBLE_EQ(state.orientation.norm(), 2.);
    state = {};
    state.velocity.x() = inf;
    EXPECT_THROW(dynamics.advance(state, environment, .1), std::invalid_argument);
    EXPECT_THROW(dynamics.support_extent(Eigen::Vector3d::Zero()), std::invalid_argument);
    parameters.model = PayloadModel::Finned;
    parameters.angular_damping = 1e10;
    state = {};
    EXPECT_THROW(PayloadDynamics(parameters).advance(state, environment, 1.),
                 std::invalid_argument);
    EXPECT_EQ(state.position, Eigen::Vector3d::Zero());
}
} // namespace

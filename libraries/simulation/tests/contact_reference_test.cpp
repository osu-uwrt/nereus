#include "detail/box_contacts.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <gtest/gtest.h>
#include <nereus/simulation/plant.hpp>
#include <sstream>

using namespace nereus::simulation::detail;
namespace {
struct Case {
    std::vector<BoxProxy> body, world;
    State13d state;
};
Case contactCase(int scenario) {
    std::vector<BoxProxy> body{{"hull", {.6, .4, .3}, {.05, -.02, .03}}, {"probe", {.2, .05, .05}, {.4, -.2, -.1}}};
    if (scenario == 10)
        body[0].orientation = Eigen::AngleAxisd(.3, Eigen::Vector3d::UnitX());
    std::vector<BoxProxy> world{{"floor", {10, 10, 1}, {0, 0, -.5}}, {"wall", {1, 10, 3}, {1, 0, 1}}};
    State13d state = State13d::Zero();
    state.head<3>() << (scenario >= 4 ? .45 : 0), 0, (scenario < 4 ? .1 : .3);
    const Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(.1 * scenario, Eigen::Vector3d::UnitZ())) *
                                 Eigen::Quaterniond(Eigen::AngleAxisd(.03 * scenario, Eigen::Vector3d::UnitY()));
    state.segment<4>(3) << q.w(), q.x(), q.y(), q.z();
    if (scenario == 6)
        state.segment<4>(3) *= 1.01;
    state.segment<3>(7) << (scenario >= 4 ? .5 : .2), .3, (scenario % 2 == 0 ? -.4 : .4);
    state.tail<3>() << .1, -.2, .3;
    if (scenario == 8) {
        world.pop_back();
        state.head<3>() << 0, 0, .1;
        state.segment<4>(3) << 1, 0, 0, 0;
        state.segment<3>(7) << .2, .3, .4;
        state.tail<3>().setZero();
    } else if (scenario == 9) {
        state.head<3>() << -2, 0, 2;
    }
    if (scenario == 3)
        world.clear();
    else if (scenario == 7)
        world = {{"tilted", {1, 3, 3}, {1, 0, 1}, Eigen::Quaterniond(Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitZ()))}};
    return {std::move(body), std::move(world), state};
}
} // namespace
TEST(BoxContacts, MatchesOriginalStaticBoxImpulseAndDepenetration) {
    std::ifstream fixture(std::string(NEREUS_FIXTURES) + "/legacy_box_contacts.csv");
    ASSERT_TRUE(fixture);
    std::string row;
    ASSERT_TRUE(std::getline(fixture, row));
    MarineDynamics dynamics;
    Matrix6d added = Matrix6d::Identity();
    added(0, 4) = added(4, 0) = .2;
    dynamics.configure(10, Eigen::Vector3d(.3, .4, .5).asDiagonal(), added);
    for (int scenario = 0; scenario < 11; ++scenario) {
        SCOPED_TRACE(scenario);
        auto input = contactCase(scenario);
        const auto &body = input.body, &world = input.world;
        const auto &state = input.state;
        BoxContacts contacts(body, world);
        const auto result = contacts.resolve(state, dynamics.inverseMass());
        ASSERT_TRUE(std::getline(fixture, row));
        std::istringstream fields(row);
        std::string value;
        ASSERT_TRUE(std::getline(fields, value, ','));
        EXPECT_EQ(std::stoi(value), scenario);
        for (int i = 0; i < 13; ++i) {
            ASSERT_TRUE(std::getline(fields, value, ','));
            EXPECT_NEAR(result[i], std::stod(value), 2e-12);
        }
        if (scenario == 8) {
            EXPECT_GT(result[2], state[2]);
            EXPECT_EQ(result.tail<6>(), state.tail<6>()); // Separating: correction, no impulse.
        }
        if (scenario == 3 || scenario == 9) {
            EXPECT_EQ(result, state);
        }
    }
    EXPECT_FALSE(std::getline(fixture, row));
}
TEST(BoxContacts, RejectsInvalidGeometryAndCoefficients) {
    BoxProxy box;
    box.id = "box";
    box.size.x() = 0;
    EXPECT_THROW(BoxContacts({box}, {}), std::invalid_argument);
    box.size.x() = 1;
    EXPECT_THROW(BoxContacts({box, box}, {}), std::invalid_argument);
    EXPECT_THROW(BoxContacts({box}, {}, 1.1), std::invalid_argument);
    EXPECT_THROW(BoxContacts({box}, {}, .1, -1), std::invalid_argument);
    box.orientation.coeffs().setZero();
    EXPECT_THROW(BoxContacts({box}, {}), std::invalid_argument);
}

// The reference trajectories were recorded on aarch64. Other ISAs round differently (Eigen uses SSE instead
// of NEON, no fused multiply-add) and contact onset amplifies last-bit differences, so only aarch64 is a
// bit-level check; elsewhere agreement to about 1e-4 is expected.
#if defined(__aarch64__)
constexpr double kReferenceTolerance = 1e-9;
#else
constexpr double kReferenceTolerance = 1e-3;
#endif

TEST(BoxContacts, PlantMatchesOriginalPreAndPostIntegrationContactSequence) {
    // The original solver itself takes different contact branches after rounding
    // at -O0 vs -O2. Match one complete captured trajectory, never a per-field mix.
    std::array<std::ifstream, 2> fixtures{
        std::ifstream(std::string(NEREUS_FIXTURES) + "/legacy_box_steps.csv"),
        std::ifstream(std::string(NEREUS_FIXTURES) + "/legacy_box_steps_unoptimized.csv")};
    std::array<double, 2> maximum_error{};
    std::string row;
    for (auto &fixture : fixtures) {
        ASSERT_TRUE(fixture);
        ASSERT_TRUE(std::getline(fixture, row));
    }
    for (int scenario = 0; scenario < 11; ++scenario) {
        const auto input = contactCase(scenario);
        nereus::simulation::PlantParameters p;
        p.body.inertia = Eigen::Vector3d(.3, .4, .5).asDiagonal();
        p.body.added_mass = Matrix6d::Identity();
        p.body.added_mass(0, 4) = p.body.added_mass(4, 0) = .2;
        p.body.displaced_volume = 0;
        p.pool.water_density = 998.2;
        p.contacts.model = nereus::simulation::ContactModel::BoxScene;
        p.contacts.body_boxes = input.body;
        p.contacts.world_boxes = input.world;
        nereus::simulation::BodyState initial;
        initial.position = input.state.head<3>();
        initial.orientation = Eigen::Quaterniond(input.state[3], input.state[4], input.state[5], input.state[6]);
        initial.linear_velocity = input.state.segment<3>(7);
        initial.angular_velocity = input.state.tail<3>();
        nereus::simulation::Plant plant(p, initial);
        for (int tick = 1; tick <= 50; ++tick) {
            SCOPED_TRACE("case=" + std::to_string(scenario) + " tick=" + std::to_string(tick));
            const auto result = plant.advance();
            const auto &body = result.body;
            State13d actual;
            actual << body.position, body.orientation.w(), body.orientation.x(), body.orientation.y(),
                body.orientation.z(), body.linear_velocity, body.angular_velocity;
            ASSERT_TRUE(actual.allFinite());
            for (std::size_t candidate = 0; candidate < fixtures.size(); ++candidate) {
                ASSERT_TRUE(std::getline(fixtures[candidate], row));
                std::istringstream fields(row);
                std::string value;
                ASSERT_TRUE(std::getline(fields, value, ','));
                ASSERT_EQ(std::stoi(value), scenario);
                ASSERT_TRUE(std::getline(fields, value, ','));
                ASSERT_EQ(std::stoi(value), tick);
                for (int i = 0; i < 13; ++i) {
                    ASSERT_TRUE(std::getline(fields, value, ','));
                    maximum_error[candidate] =
                        std::max(maximum_error[candidate], std::abs(actual[i] - std::stod(value)));
                }
            }
            EXPECT_EQ(result.elapsed.count(), tick * 2000000LL);
        }
    }
    for (auto &fixture : fixtures)
        EXPECT_FALSE(std::getline(fixture, row));
    EXPECT_LE(std::min(maximum_error[0], maximum_error[1]), kReferenceTolerance)
        << "optimized reference maximum error: " << maximum_error[0]
        << "; unoptimized reference maximum error: " << maximum_error[1];
}

#include "detail/box_contacts.hpp"
#include <fstream>
#include <gtest/gtest.h>
#include <sstream>

using namespace robotics::simulation::detail;
TEST(BoxContacts, MatchesOriginalStaticBoxImpulseAndDepenetration) {
    std::ifstream fixture(std::string(RP_FIXTURES) + "/legacy_box_contacts.csv");
    ASSERT_TRUE(fixture);
    std::string row;
    ASSERT_TRUE(std::getline(fixture, row));
    MarineDynamics dynamics;
    Matrix6d added = Matrix6d::Identity();
    added(0, 4) = added(4, 0) = .2;
    dynamics.configure(10, Eigen::Vector3d(.3, .4, .5).asDiagonal(), added);
    for (int scenario = 0; scenario < 11; ++scenario) {
        SCOPED_TRACE(scenario);
        std::vector<BoxProxy> body{{"hull", {.6, .4, .3}, {.05, -.02, .03}},
                                   {"probe", {.2, .05, .05}, {.4, -.2, -.1}}};
        if (scenario == 10)
            body[0].orientation = Eigen::AngleAxisd(.3, Eigen::Vector3d::UnitX());
        std::vector<BoxProxy> world{{"floor", {10, 10, 1}, {0, 0, -.5}},
                                    {"wall", {1, 10, 3}, {1, 0, 1}}};
        State13d state = State13d::Zero();
        state.head<3>() << (scenario >= 4 ? .45 : 0), 0, (scenario < 4 ? .1 : .3);
        const Eigen::Quaterniond q =
            Eigen::Quaterniond(Eigen::AngleAxisd(.1 * scenario, Eigen::Vector3d::UnitZ())) *
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
            world = {{"tilted",
                      {1, 3, 3},
                      {1, 0, 1},
                      Eigen::Quaterniond(Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitZ()))}};
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

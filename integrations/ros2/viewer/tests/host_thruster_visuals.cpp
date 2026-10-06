#include "thruster_visuals.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

using namespace nereus::ros_viewer::host;
namespace {
bool near(double a, double b) {
    return std::abs(a - b) < 1e-6;
}
const std::vector<std::string> kOrder{"VUS", "VUP", "HUS", "HUP", "HLS", "HLP", "VLS", "VLP"};
} // namespace

TEST(HostThrusterVisuals, RpmIntegrationTimeoutAndGeometry) {
    ThrusterVisuals absent;
    ASSERT_TRUE(absent.rotors.empty() && absent.topic.empty());
    ThrusterVisuals visuals(YAML::LoadFile(std::string(NEREUS_VIEWER_CONTENT) + "/talos_uwrt_thruster_visuals.yaml"),
                            kOrder);
    ASSERT_TRUE(visuals.rotors.size() == 8);
    ASSERT_TRUE(visuals.rotors[3].inputIndex == 3 && visuals.rotors[3].asset == "rotor_HUP");
    // Recorded expectations from the T200 propeller law (K_T = a + b rpm, separate forward/reverse).
    ASSERT_TRUE(near(visuals.rpm(4), 1024.9989755045349));
    ASSERT_TRUE(near(visuals.rpm(-4), -1152.0447935214627));
    ASSERT_TRUE(near(visuals.rpm(24), 2448.8169288985055));
    ASSERT_TRUE(near(visuals.rpm(-24), -2739.711837643229));
    ASSERT_TRUE(visuals.rpm(0) == 0 && visuals.rpm(.011) >= 0 && visuals.rpm(-.011) <= 0);
    ASSERT_TRUE(visuals.receive(std::vector<float>(8, 4), 0));
    visuals.advance(.01);
    ASSERT_TRUE(near(visuals.rotors[1].angle, 1024.9989755045349 * glm::pi<double>() / 30. * .01));
    visuals.advance(.18); // Preserve full turns between render frames, without slowing or clamping.
    ASSERT_TRUE(near(visuals.rotors[1].angle,
                     std::remainder(1024.9989755045349 * glm::pi<double>() / 30. * .18, 2 * glm::pi<double>())));
    visuals.advance(-1); // Reset before testing timing with an independent symmetric fixture: rpm = sqrt(F / 3e-4),
    visuals.forwardKt = visuals.reverseKt = {1, 0}; // slow enough that no angle below wraps.
    visuals.ktScale = 3e-4;
    const auto turn = [](double force, double dt) { // rad, for the fixture
        return std::copysign(std::sqrt(std::abs(force) / 3e-4), force) * glm::pi<double>() / 30. * dt;
    };
    const double timedOut = std::remainder(-turn(2, .5), 2 * glm::pi<double>()); // VUS after a full .5 s window
    std::vector<float> forces{2, 2, -3, 0, 0, 0, 0, 0};
    ASSERT_TRUE(visuals.receive(forces, 0));
    visuals.advance(.1);
    ASSERT_TRUE(near(visuals.rotors[0].angle, -turn(2, .1))); // VUS spins in the -1 direction
    ASSERT_TRUE(near(visuals.rotors[1].angle, turn(2, .1)));
    ASSERT_TRUE(near(visuals.rotors[2].angle, turn(-3, .1)));
    for (size_t i = 3; i < 8; ++i)
        ASSERT_TRUE(near(visuals.rotors[i].angle, 0));
    visuals.advance(.1); // Pausing the ROS clock freezes the animation.
    ASSERT_TRUE(near(visuals.rotors[0].angle, -turn(2, .1)));
    for (auto &f : forces)
        f = -f;
    ASSERT_TRUE(visuals.receive(forces, .1));
    visuals.advance(.2);
    for (const auto &rotor : visuals.rotors)
        ASSERT_TRUE(near(rotor.angle, 0));
    ASSERT_TRUE(visuals.receive(std::vector<float>(8, 0), .2));
    visuals.advance(.4);
    ASSERT_TRUE(near(visuals.rotors[0].angle, 0));

    forces.assign(8, 2);
    ASSERT_TRUE(visuals.receive(forces, .4));
    ASSERT_TRUE(!visuals.receive({1, 2}, .6));
    forces[0] = std::numeric_limits<float>::quiet_NaN();
    ASSERT_TRUE(!visuals.receive(forces, .7));
    visuals.advance(1.0); // Invalid packets must not extend the .5 s timeout.
    ASSERT_TRUE(near(visuals.rotors[0].angle, timedOut));
    visuals.advance(2.0);
    ASSERT_TRUE(near(visuals.rotors[0].angle, timedOut));
    ASSERT_TRUE(visuals.receive(std::vector<float>(8, .005f), 2));
    visuals.advance(2.1);
    ASSERT_TRUE(near(visuals.rotors[0].angle, timedOut)); // Negligible residual force is idle.
    visuals.advance(0);                                   // A simulator clock reset clears old forces and phase.
    visuals.advance(.2);
    for (const auto &rotor : visuals.rotors)
        ASSERT_TRUE(near(rotor.angle, 0));
    ASSERT_TRUE(visuals.receive(std::vector<float>(8, 100), .2));
    visuals.advance(.201);
    ASSERT_TRUE(near(visuals.rotors[0].angle, -turn(100, .001))); // No artificial display speed cap.

    for (auto &rotor : visuals.rotors) {
        rotor.angle = .7;
        auto matrix = rotor.transform();
        glm::vec3 pivot = matrix * glm::vec4(rotor.pivot, 1);
        glm::vec3 shaft = rotor.pivot + .02f * rotor.axis;
        ASSERT_TRUE(glm::length(pivot - rotor.pivot) < 1e-6f);
        ASSERT_TRUE(glm::length(glm::vec3(matrix * glm::vec4(shaft, 1)) - shaft) < 1e-6f);
        // An off-axis blade moves without changing its distance to the shaft.
        glm::vec3 radial = glm::cross(rotor.axis, glm::vec3(0, 0, 1));
        glm::vec3 point = rotor.pivot + .02f * radial;
        glm::vec3 rotated = matrix * glm::vec4(point, 1);
        ASSERT_TRUE(glm::length(rotated - point) > .001f);
        ASSERT_TRUE(near(glm::length(rotated - rotor.pivot), glm::length(point - rotor.pivot)));
    }
}

TEST(HostThrusterVisuals, RotorNamingAnUnknownThrusterIsRejected) {
    const auto config = YAML::LoadFile(std::string(NEREUS_VIEWER_CONTENT) + "/talos_uwrt_thruster_visuals.yaml");
    EXPECT_THROW(ThrusterVisuals(config, {"VUS", "VUP", "HUS"}), std::invalid_argument);
    EXPECT_THROW(ThrusterVisuals(config, {}), std::invalid_argument);
}

// Force-array layout comes from the bridge (thrusters.order), not from the visuals document.
TEST(HostThrusterVisuals, ForceIndexFollowsBridgeOrder) {
    auto reordered = kOrder;
    std::swap(reordered[0], reordered[1]);
    ThrusterVisuals visuals(YAML::LoadFile(std::string(NEREUS_VIEWER_CONTENT) + "/talos_uwrt_thruster_visuals.yaml"),
                            reordered);
    ASSERT_TRUE(visuals.rotors[0].id == "VUS" && visuals.rotors[0].inputIndex == 1);
    ASSERT_TRUE(visuals.rotors[1].id == "VUP" && visuals.rotors[1].inputIndex == 0);
}

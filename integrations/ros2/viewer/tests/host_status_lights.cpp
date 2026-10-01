#include "status_lights.hpp"
#include <gtest/gtest.h>
#include <limits>

using namespace nereus::ros_viewer::host;
namespace {
bool equal(glm::vec3 a, glm::vec3 b) {
    return glm::length(a - b) < 1e-5f;
}
} // namespace

TEST(HostStatusLights, ModesTargetsAndPulses) {
    StatusLights absent;
    ASSERT_TRUE(absent.lights.empty() && absent.input.empty() && absent.topic.empty());
    StatusLights lights(YAML::LoadFile(std::string(NEREUS_VIEWER_CONTENT) + "/talos_uwrt_status_lights.yaml"));
    ASSERT_TRUE(lights.lights.size() == 3);
    ASSERT_TRUE(lights.topic == "command/led");
    for (const auto &light : lights.lights) {
        ASSERT_TRUE(light.frame == "origin");
        ASSERT_TRUE(light.mount[3].y > 0 && light.mount[3].z > 0); // port hull, above the CAD origin
        ASSERT_TRUE(equal(light.state.color(0), glm::vec3(0)));
    }
    auto color = [&](double t) { return lights.lights.front().state.color(t); };
    lights.command({1, 0, 1}, LightMode::Solid, 0, 0);
    ASSERT_TRUE(equal(color(0), glm::vec3(0))); // TARGET_NONE
    lights.command({1, 0, 1}, LightMode::Solid, 1, 0);
    ASSERT_TRUE(equal(color(0), glm::vec3(0))); // CCB must not change ALU lights
    lights.command({1, 0, 1}, LightMode::Solid, 2, 0);
    ASSERT_TRUE(equal(color(0), {1, 0, 1}));
    lights.command({0, 1, 0}, LightMode::SlowFlash, 3, 0);
    ASSERT_TRUE(equal(color(.5), glm::vec3(0)));
    ASSERT_TRUE(equal(color(1.5), {0, 1, 0}));
    lights.command({0, 1, 0}, LightMode::FastFlash, 3, 0);
    ASSERT_TRUE(equal(color(.1), glm::vec3(0)));
    ASSERT_TRUE(equal(color(.3), {0, 1, 0}));
    lights.command({1, 0, 0}, LightMode::Breath, 3, 0);
    ASSERT_TRUE(equal(color(0), {.5f, 0, 0}));
    ASSERT_TRUE(equal(color(.75), {1, 0, 0}));
    ASSERT_TRUE(equal(color(2.25), glm::vec3(0)));
    // A detection pulse temporarily overlays, then restores the status command.
    lights.command({0, 0, 1}, LightMode::Solid, 3, 0);
    lights.command({1, 1, 1}, LightMode::Flash, 2, 10);
    ASSERT_TRUE(equal(color(10.05), glm::vec3(1)));
    ASSERT_TRUE(equal(color(10.2), {0, 0, 1}));
    ASSERT_TRUE(equal(color(0), {0, 0, 1})); // reset clock cannot extend old pulses
    lights.command({std::numeric_limits<float>::quiet_NaN(), 0, 0}, LightMode::Solid, 3, 11);
    ASSERT_TRUE(equal(color(11), {0, 0, 1}));
    lights.command({-1, 2, .5f}, LightMode::Solid, UINT32_MAX, 11);
    ASSERT_TRUE(equal(color(11), {0, 1, .5f}));
    lights.command({0, 0, 0}, LightMode::Solid, 3, 12);
    for (const auto &light : lights.lights)
        ASSERT_TRUE(equal(light.state.color(12), glm::vec3(0)));
    // A robot can independently route commands to multiple groups of emitters.
    lights.lights.front().targets = 1;
    lights.command({1, 0, 0}, LightMode::Solid, 1, 13);
    lights.command({0, 1, 0}, LightMode::Solid, 2, 13);
    ASSERT_TRUE(equal(color(13), {1, 0, 0}));
    ASSERT_TRUE(equal(lights.lights.back().state.color(13), {0, 1, 0}));
}

TEST(HostStatusLights, RejectsInvalidDocuments) {
    EXPECT_THROW(StatusLights(YAML::Load("input: {type: other/msg/Type, topic: x}\nlights: []")),
                 std::invalid_argument);
    EXPECT_THROW(StatusLights(YAML::Load("input: {type: std_msgs/msg/ColorRGBA, topic: x}\nlights: []")),
                 std::invalid_argument);
    EXPECT_THROW(StatusLights(YAML::Load("input: {type: std_msgs/msg/ColorRGBA, topic: x}\nlights:\n"
                                         "  - {id: a, pose: [0,0,0,0,0,0], size: [1,1,-1]}")),
                 std::invalid_argument);
}

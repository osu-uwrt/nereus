#include "camera_geometry.hpp"
#include "frame_graph.hpp"
#include <gtest/gtest.h>

using namespace nereus::ros_viewer::host;

TEST(HostCameraGeometry, ProjectionRegistrationAndAxialDepth) {
    Intrinsics k;
    k.width = 960;
    k.height = 600;
    k.fx = 480;
    k.fy = 500;
    k.cx = 301.25;
    k.cy = 275.75;
    k.validate();
    // Independently project an optical-frame point through the graphics matrix.
    const glm::vec3 optical(.31, -.14, 2.7);
    const auto clip = k.projection() * glm::vec4(optical.x, -optical.y, -optical.z, 1);
    const auto ndc = glm::vec3(clip) / clip.w;
    EXPECT_NEAR((ndc.x + 1) * k.width / 2 - .5, k.fx * optical.x / optical.z + k.cx, .001);
    EXPECT_NEAR((1 - ndc.y) * k.height / 2 - .5, k.fy * optical.y / optical.z + k.cy, .001);
    EXPECT_NEAR(linearDepth(ndc.z * .5f + .5f), optical.z, .0001); // axial metres, not ray length
    for (float z : {.1f, 1.f, 4.f, 20.f}) {
        const auto c = k.projection() * glm::vec4(0, 0, -z, 1);
        EXPECT_NEAR(linearDepth((c.z / c.w + 1) / 2), z, .003);
    }
    Intrinsics bad = k;
    bad.fx = 0;
    EXPECT_THROW(bad.validate(), std::runtime_error);
}

TEST(HostCameraGeometry, OpticalViewLooksAlongOpticalZ) {
    Intrinsics k;
    k.fx = k.fy = 1000;
    k.cx = 960;
    k.cy = 600;
    // Downward camera: optical +Z toward world -Z, optical +X toward world +X, optical +Y toward world -Y.
    glm::mat4 world(1);
    world[0] = {1, 0, 0, 0};
    world[1] = {0, -1, 0, 0};
    world[2] = {0, 0, -1, 0};
    world[3] = {1, 2, 3, 1};
    const auto view = sensorView(world, k);
    EXPECT_NEAR(glm::distance(view.eye, glm::vec3(1, 2, 3)), 0, 1e-6);
    const auto ahead = view.view * glm::vec4(1, 2, 3 - 2, 1); // 2 m below the camera
    EXPECT_NEAR(ahead.x, 0, 1e-5);
    EXPECT_NEAR(ahead.y, 0, 1e-5);
    EXPECT_NEAR(ahead.z, -2, 1e-5);
    // A point to the camera's optical right lands at positive NDC x.
    const auto right = view.projection * view.view * glm::vec4(2, 2, 1, 1);
    EXPECT_GT(right.x / right.w, 0);
}

// Robot-pack frames replace the vehicle yaml: a CAD point at the base_link anchor must land exactly on the
// world base link, and slots/cameras compose through the tree exactly once.
TEST(HostFrameGraph, ComposesPackFramesThroughTheRoot) {
    FrameGraph g;
    g.add("com", "cad", poseQuat({.157f, -.04f, .048f}, glm::quat(1, 0, 0, 0)));
    g.add("cad", "base_link", poseQuat({-.14f, .03f, -.09f}, glm::quat(1, 0, 0, 0)));
    g.add("cad", "torpedoes_mount", pose({.048f, .182f, -.1324f}, {0, 0, glm::half_pi<float>()}));
    const auto cadInBase = g.relative("base_link", "cad");
    EXPECT_NEAR(cadInBase[3].x, .14, 1e-6);
    EXPECT_NEAR(cadInBase[3].y, -.03, 1e-6);
    EXPECT_NEAR(cadInBase[3].z, .09, 1e-6);
    for (const auto &angles : {glm::vec3(0), glm::vec3(0, 0, glm::pi<float>()), glm::vec3(.4f, -.3f, 1.2f)}) {
        const auto worldBase = pose({2, -3, -.7f}, angles);
        const auto worldCad = worldBase * cadInBase;
        // The base link point in CAD coordinates maps back onto base_link.
        const auto anchor = worldCad * glm::vec4(-.14f, .03f, -.09f, 1);
        EXPECT_NEAR(glm::distance(glm::vec3(anchor), glm::vec3(worldBase[3])), 0, 1e-6);
    }
    const auto mount = g.relative("base_link", "torpedoes_mount");
    EXPECT_NEAR(mount[3].x, .14 + .048, 1e-6);
    EXPECT_NEAR(glm::distance(glm::vec3(mount[0]), glm::vec3(0, 1, 0)), 0, 1e-6); // actuator yaw
    EXPECT_THROW(g.relative("base_link", "missing"), std::out_of_range);
    EXPECT_THROW(g.add("cad", "base_link", glm::mat4(1)), std::invalid_argument); // two parents
}

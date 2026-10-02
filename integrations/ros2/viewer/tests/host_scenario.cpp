// The host reads the bridge's scenario document only: poses, frames, cameras, topics and landmarks.
#include "scenario.hpp"
#include <gtest/gtest.h>

using namespace nereus::ros_viewer::host;
namespace {
const char *kDocument = R"json({
 "scenario": {"id": "s1", "robot": "../r", "pool": "../p", "tasks": "../t", "bridge": "../b", "world_frame": "map",
   "pool_placement": {"position_m": [1, 2, 0.5], "yaw_deg": 90},
   "task_placements": [{"task": "gate", "position_m": [10, 0, -1], "yaw_deg": 180}]},
 "robot": {"id": "bot", "reference_frame": "base_link",
   "frames": {"root": "com", "transforms": [
     {"parent": "com", "child": "cad", "position_m": [0.1, 0, 0], "orientation_wxyz": [1, 0, 0, 0]},
     {"parent": "cad", "child": "base_link", "position_m": [-0.1, 0, 0], "orientation_wxyz": [1, 0, 0, 0]},
     {"parent": "cad", "child": "cam_mount", "position_m": [0.3, 0, 0.1], "orientation_wxyz": [1, 0, 0, 0]},
     {"parent": "cam_mount", "child": "cam_optical", "position_m": [0, 0, 0], "orientation_wxyz": [0.5, -0.5, 0.5, -0.5]},
     {"parent": "cad", "child": "launch_mount", "position_m": [0, 0.2, 0], "orientation_wxyz": [1, 0, 0, 0]},
     {"parent": "cad", "child": "thruster_b", "position_m": [0, 0, 0.3], "orientation_wxyz": [0.70710678, 0, 0, 0.70710678]}]},
   "thrusters": [{"id": "B", "type": "lagged_force", "frame": "thruster_b"},
                 {"id": "A", "type": "lagged_force", "position_m": [0, 0.5, 0], "direction": [0, 0, 1]},
                 {"id": "C", "type": "lagged_force", "frame": "not_a_frame"}],
   "visuals": [{"asset": "body_mesh", "frame": "cad", "position_m": [0, 0, 0], "orientation_wxyz": [1, 0, 0, 0]}],
   "assets": [{"id": "body_mesh", "path": "assets/body.glb"}],
   "sensors": [
     {"id": "cam", "type": "stereo_camera", "frame": "cam_optical", "mount_frame": "cam_mount", "period_ns": 66666667,
      "parameters": {"resolution_px": [1920, 1200], "intrinsics_left": {"fx": 1800.0, "fy": 1800.0, "cx": 960.0, "cy": 600.0},
                     "depth": {"min_range_m": 0.15, "max_range_m": 4.0}}},
     {"id": "imu", "type": "ahrs", "frame": "cad", "parameters": {}}],
   "mechanisms": [{"id": "launcher", "type": "launcher", "frame": "launch_mount",
     "parameters": {"slots": [{"position_m": [0.01, 0, 0], "orientation_wxyz": [1, 0, 0, 0]}],
                    "projectile": {"length_m": 0.08, "radius_m": 0.013}}}]},
 "pool": {"id": "pool1", "assets": [],
   "parameters": {"length_m": 50.0, "width_m": 22.86, "depth_m": 2.1336, "deck_height_m": 0.3, "water_level_m": 0.0},
   "water_optics": {"tint_rgb": [0.1, 0.2, 0.3], "distance_scale": 1.5, "distance_power": 0.5, "clear_distance_m": 0.0,
                    "scattering": 0.4, "absorption_per_m_rgb": [0.1, 0.03, 0.02]},
   "lighting": {"profile": "outdoor", "direct_light": 1.0, "ambient_light": 0.8, "sun_azimuth_deg": 200.0,
                "sun_elevation_deg": 50.0, "glare": 0.5}},
 "tasks": {"id": "t", "assets": [{"id": "gate_mesh", "path": "assets/gate.dae"}]},
 "task_definitions": [{"id": "gate", "frames": [{"id": "gate_left", "position_m": [0, -1, 0.5], "orientation_wxyz": [1, 0, 0, 0]}],
   "regions": [], "props": [{"id": "structure", "type": "static_body",
     "parameters": {"visuals": [{"asset": "gate_mesh", "frame": "task", "position_m": [0, 0, 0], "orientation_wxyz": [1, 0, 0, 0]}]}}]}],
 "asset_paths": {"robot": {"body_mesh": "/abs/body.glb"}, "tasks": {"gate_mesh": "/abs/gate.dae"}},
 "bridge": {"namespace": "/bot", "frame_names": {"world": "map", "cam_mount": "bot/cam_link", "cam_optical": "bot/cam_optical"},
   "thrusters": {"order": ["A", "B"], "input_scales": [1, 2]},
   "tf": {"publish": [{"parent": "map", "child": "simulator/bot/base_link", "native": "state:robot.reference_pose"}]},
   "streams": [
     {"id": "rgb", "direction": "publish", "topic": "cam/rgb/compressed", "native": "sensor:cam.rgb_left"},
     {"id": "depth", "direction": "publish", "topic": "cam/depth", "native": "sensor:cam.depth_left"},
     {"id": "info", "direction": "publish", "topic": "/abs/info", "native": "sensor:cam.camera_info"},
     {"id": "kill", "direction": "subscribe", "topic": "command/software_kill", "native": "command:robot.set_killed"}]}
})json";
} // namespace

TEST(HostScenario, ReadsFramesCamerasMechanismsAndLandmarks) {
    const auto config = YAML::Load("cameras: {cam: {title: FRONT, model: TESTCAM}}\nui: {focus: [Course], extra: 1}");
    const auto s = parseScenario(kDocument, config, {});
    EXPECT_EQ(s.robotId, "bot");
    EXPECT_EQ(s.mapFrame, "map");
    EXPECT_EQ(s.truthBaseFrame, "simulator/bot/base_link");
    EXPECT_EQ(s.estimateBaseFrame, "bot/base_link"); // not listed in frame_names: namespace/base id
    EXPECT_EQ(s.thrusterOrder, (std::vector<std::string>{"A", "B"}));
    // Thruster mounts in base_link, keeping their ROS array index and bridge input scale. A: resolved form
    // (body frame = the tree root `com`); B: a named frame's +X; C (not in the bridge order) is left out.
    ASSERT_EQ(s.thrusterMounts.size(), 2u);
    const auto &a = s.thrusterMounts[0], &b = s.thrusterMounts[1];
    EXPECT_EQ(a.id, "A");
    EXPECT_EQ(a.index, 0u);
    EXPECT_NEAR(a.position.x, 0, 1e-6); // com and base_link coincide (+0.1 then -0.1)
    EXPECT_NEAR(a.position.y, .5, 1e-6);
    EXPECT_NEAR(a.axis.z, 1, 1e-6);
    EXPECT_EQ(b.id, "B");
    EXPECT_EQ(b.index, 1u);
    EXPECT_NEAR(b.inputScale, 2, 1e-6);
    EXPECT_NEAR(b.position.x, .1, 1e-6); // cad sits +0.1 from base_link
    EXPECT_NEAR(b.position.z, .3, 1e-6);
    EXPECT_NEAR(b.axis.y, 1, 1e-5); // yawed 90 deg: the force axis is base +Y
    // Pool placement: yaw 90 about Z at (1,2); water level raised by the placement height.
    EXPECT_NEAR(s.waterLevel, .5, 1e-6);
    const auto corner = s.poolToWorld * glm::vec4(10, 0, 0, 1);
    EXPECT_NEAR(corner.x, 1, 1e-5);
    EXPECT_NEAR(corner.y, 12, 1e-5);
    EXPECT_NEAR(glm::distance(glm::vec3(s.worldToPool * corner), glm::vec3(10, 0, 0)), 0, 1e-5);
    EXPECT_TRUE(s.appearance.outdoor);
    EXPECT_NEAR(s.appearance.water.scattering, .4, 1e-6);
    // Camera: display names from config, ROS frames from the bridge, topics namespaced unless absolute.
    ASSERT_EQ(s.cameras.size(), 1u);
    const auto &c = s.cameras[0];
    EXPECT_EQ(c.title, "FRONT");
    EXPECT_EQ(c.model, "TESTCAM");
    EXPECT_EQ(c.rosOpticalFrame, "bot/cam_optical");
    EXPECT_EQ(c.rgbTopic, "/bot/cam/rgb/compressed");
    EXPECT_EQ(c.depthTopic, "/bot/cam/depth");
    EXPECT_EQ(c.infoTopic, "/abs/info");
    EXPECT_EQ(c.k.width, 1920);
    EXPECT_NEAR(c.maxRange, 4, 1e-9);
    EXPECT_NEAR(c.mountInBase[3].x, .4, 1e-6); // cad is at -base offset; mount at +0.3 in cad
    EXPECT_NEAR(c.mountInBase[3].z, .1, 1e-6);
    // Robot visuals are resolved through asset_paths and expressed in base_link.
    ASSERT_EQ(s.robotVisuals.size(), 1u);
    EXPECT_EQ(s.robotVisuals[0].path, "/abs/body.glb");
    EXPECT_NEAR(s.robotVisuals[0].inBase[3].x, .1, 1e-6);
    // Mechanism slots in base_link; projectile size from the pack.
    const auto *launcher = s.mechanism("launcher");
    ASSERT_TRUE(launcher);
    ASSERT_EQ(launcher->slotsInBase.size(), 1u);
    EXPECT_NEAR(launcher->slotsInBase[0][3].y, .2, 1e-6);
    EXPECT_NEAR(launcher->projectileLength, .08, 1e-6);
    // Task placement -> landmark and visual world transform (yaw 180).
    ASSERT_TRUE(s.landmarks.count("gate"));
    ASSERT_TRUE(s.landmarks.count("gate_left"));
    EXPECT_NEAR(s.landmarks.at("gate").world[0].x, -1, 1e-5);
    EXPECT_NEAR(s.landmarks.at("gate_left").world[3].y, 1, 1e-5); // task y=-1 flipped by yaw 180
    ASSERT_EQ(s.taskVisuals.size(), 1u);
    EXPECT_EQ(s.taskVisuals[0].path, "/abs/gate.dae");
    // ui: configured fallback survives when the task pack carries none.
    EXPECT_TRUE(s.ui["extra"]);
    EXPECT_EQ(s.absolute("x/y"), "/bot/x/y");
    EXPECT_EQ(s.absolute("/z"), "/z");
}

TEST(HostScenario, TaskPackUiOverridesConfiguredUiKeyByKey) {
    auto text = std::string(kDocument);
    const std::string needle = R"("tasks": {"id": "t",)";
    text.replace(text.find(needle), needle.size(), R"("tasks": {"ui": {"focus": ["Vehicle"]}, "id": "t",)");
    const auto s = parseScenario(text, YAML::Load("ui: {focus: [Course], keep: 2}"), {});
    EXPECT_EQ(s.ui["focus"][0].as<std::string>(), "Vehicle");
    EXPECT_EQ(s.ui["keep"].as<int>(), 2);
}

TEST(HostScenario, RejectsIncompleteDocuments) {
    EXPECT_THROW(parseScenario("{\"scenario\": {}}", YAML::Node(), {}), std::runtime_error);
}

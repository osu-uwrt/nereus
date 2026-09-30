#include <gtest/gtest.h>
#include "detection_pose.hpp"
#include <iostream>
using namespace robotics::ros_viewer::host;
#include <iostream>

TEST(HostDetectionPose, AcquisitionTimeAndImmutablePlacement) {
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf2_ros::Buffer tf(clock);
  auto set = [&](int sec, double x) {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "map";
    t.child_frame_id = "camera";
    t.header.stamp.sec = sec;
    t.transform.translation.x = x;
    t.transform.rotation.w = 1;
    ASSERT_TRUE(tf.setTransform(t, "test"));
  };
  set(10, 1);
  set(12, 3);
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "camera";
  m.header.stamp.sec = 11;
  m.pose.position.x = 5;
  m.pose.orientation.w = 1;
  glm::mat4 result(1);
  const auto close = [](const glm::vec3 &actual, const glm::vec3 &expected) {
    ASSERT_TRUE(glm::length(actual - expected) < 1e-5f);
  };

  // A delayed observation interpolates at acquisition time, not arrival time.
  ASSERT_TRUE(resolveDetectionPose(m, "map", tf, result));
  close(glm::vec3(result[3]), {7, 0, 0});
  set(13, 10);
  ASSERT_TRUE(resolveDetectionPose(m, "map", tf, result));
  close(glm::vec3(result[3]), {7, 0, 0});

  // No latest-pose fallback for future stamps: wait until TF catches up.
  m.header.stamp.sec = 14;
  ASSERT_TRUE(!resolveDetectionPose(m, "map", tf, result));
  set(14, 12);
  ASSERT_TRUE(resolveDetectionPose(m, "map", tf, result));
  close(glm::vec3(result[3]), {17, 0, 0});

  m.header.stamp.sec = 0;
  ASSERT_TRUE(resolveDetectionPose(m, "map", tf, result));
  close(glm::vec3(result[3]), {17, 0, 0});
  m.header.stamp.sec = 11;
  m.frame_locked = true;
  ASSERT_TRUE(resolveDetectionPose(m, "map", tf, result));
  close(glm::vec3(result[3]), {7, 0, 0});
  set(15, 14);
  ASSERT_TRUE(resolveDetectionPose(m, "map", tf, result));
  close(glm::vec3(result[3]), {7, 0, 0});

  m.frame_locked = false;
  m.header.frame_id = "map";
  ASSERT_TRUE(resolveDetectionPose(m, "map", tf, result));
  close(glm::vec3(result[3]), {5, 0, 0});
  m.header.frame_id = "unknown";
  ASSERT_TRUE(!resolveDetectionPose(m, "map", tf, result));

  m.header.frame_id = "camera";
  m.pose.position.x = 1;
  m.pose.position.y = 2;
  m.pose.position.z = 3;

  // Exercise the same stateful placement used by the viewer over many frames.
  // Subsequent TF updates must not move an existing observation, including
  // zero-stamped/frame-locked markers.
  int nextStamp = 16;
  for (bool locked : {false, true}) {
    for (int stamp : {0, 11}) {
      m.frame_locked = locked;
      m.header.stamp.sec = stamp;
      m.lifetime.sec = 5;
      DetectionPose observation;
      ASSERT_TRUE(observation.place(m, "map", tf));
      const auto initial = observation.world();
      set(nextStamp++, 30);
      for (int frame = 0; frame < 20; ++frame) {
        ASSERT_TRUE(observation.place(m, "map", tf));
        for (int column = 0; column < 4; ++column)
          ASSERT_TRUE(glm::length(observation.world()[column] - initial[column]) < 1e-5f);
      }
      ASSERT_TRUE(m.lifetime.sec == 5 && m.frame_locked == locked);
    }
  }
  // Unresolved observations can retry; replacement creates a new placement.
  m.header.stamp.sec = nextStamp;
  DetectionPose pending;
  ASSERT_TRUE(!pending.place(m, "map", tf));
  set(nextStamp, 40);
  ASSERT_TRUE(pending.place(m, "map", tf));
  close(glm::vec3(pending.world()[3]), {41, 2, 3});

  // Never substitute estimated TF when the simulator frame is unavailable.
  DetectionPose missingTruth;
  ASSERT_TRUE(!missingTruth.place(m, "map", tf, "simulator/camera"));

  // Repeated observations of a stationary simulator target must agree while the
  // robot translates and rotates, even when simulator and estimated poses
  // diverge. This covers replacement markers, not just a cached observation.
  const auto target = glm::translate(glm::mat4(1), glm::vec3(8, -3, -2)) *
      glm::rotate(glm::mat4(1), .7f, glm::vec3(0, 1, 0));
  const auto setPose = [&](const std::string &child, int sec, const glm::mat4 &pose) {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "map";
    t.child_frame_id = child;
    t.header.stamp.sec = sec;
    t.transform.translation.x = pose[3].x;
    t.transform.translation.y = pose[3].y;
    t.transform.translation.z = pose[3].z;
    const auto q = glm::quat_cast(pose);
    t.transform.rotation.w = q.w;
    t.transform.rotation.x = q.x;
    t.transform.rotation.y = q.y;
    t.transform.rotation.z = q.z;
    ASSERT_TRUE(tf.setTransform(t, "test"));
  };
  m.frame_locked = false;
  for (int step = 0; step < 20; ++step) {
    const int stamp = 40 + 2 * step;
    const auto camera = glm::translate(glm::mat4(1), glm::vec3(step * .2f, 1, -1)) *
        glm::rotate(glm::mat4(1), step * .1f, glm::vec3(0, 0, 1)) *
        opticalToLink();
    const auto drift = glm::translate(glm::mat4(1), glm::vec3(1 + step * .3f, -2, 0)) *
        glm::rotate(glm::mat4(1), step * .05f, glm::vec3(0, 0, 1));
    setPose("camera", stamp, drift * camera);
    setPose("simulator/camera", stamp, camera);
    // A newer robot pose must not affect a delayed detection.
    setPose("simulator/camera", stamp + 1,
            glm::translate(glm::mat4(1), glm::vec3(20, 30, 40)));
    const auto local = glm::inverse(camera) * target;
    const auto q = glm::quat_cast(local);
    m.header.frame_id = "camera";
    m.header.stamp.sec = stamp;
    m.pose.position.x = local[3].x;
    m.pose.position.y = local[3].y;
    m.pose.position.z = local[3].z;
    m.pose.orientation.w = q.w;
    m.pose.orientation.x = q.x;
    m.pose.orientation.y = q.y;
    m.pose.orientation.z = q.z;
    DetectionPose observation;
    ASSERT_TRUE(observation.place(m, "map", tf, "simulator/camera"));
    for (int column = 0; column < 4; ++column)
      ASSERT_TRUE(glm::length(observation.world()[column] - target[column]) < 1e-5f);

    // RViz's estimated placement differs from the actual simulated target.
    ASSERT_TRUE(resolveDetectionPose(m, "map", tf, result));
    ASSERT_TRUE(glm::length(result[3] - target[3]) > .1f);

    // Exact rendered poses take priority over TF's sampled/interpolated pose.
    const auto offset = glm::translate(glm::mat4(1), glm::vec3(.08f, -.04f, .02f));
    auto rendered = offset * camera;
    DetectionPose captured;
    ASSERT_TRUE(captured.place(m, "map", tf, "simulator/camera", &rendered));
    const auto renderedTarget = offset * target;
    for (int column = 0; column < 4; ++column)
      ASSERT_TRUE(glm::length(captured.world()[column] - renderedTarget[column]) < 1e-5f);
    rendered = glm::mat4(1);
    ASSERT_TRUE(captured.place(m, "map", tf, "simulator/camera", &rendered));
    for (int column = 0; column < 4; ++column)
      ASSERT_TRUE(glm::length(captured.world()[column] - renderedTarget[column]) < 1e-5f);

    // A marker explicitly in the simulator branch still uses that branch.
    m.header.frame_id = "simulator/camera";
    ASSERT_TRUE(resolveDetectionPose(m, "map", tf, result));
    for (int column = 0; column < 4; ++column)
      ASSERT_TRUE(glm::length(result[column] - target[column]) < 1e-5f);
  }
  // Zero stamps have no matching acquisition: use simulator TF once, then
  // keep the observation fixed even when newer simulator poses arrive.
  m.header.frame_id = "camera";
  m.header.stamp.sec = 0;
  const glm::mat4 unrelatedRender(1);
  DetectionPose unstamped;
  ASSERT_TRUE(unstamped.place(m, "map", tf, "simulator/camera", &unrelatedRender));
  ASSERT_TRUE(resolveDetectionPose(m, "map", tf, result, "simulator/camera"));
  for (int column = 0; column < 4; ++column)
    ASSERT_TRUE(glm::length(unstamped.world()[column] - result[column]) < 1e-5f);
  setPose("simulator/camera", 81, glm::mat4(1));
  ASSERT_TRUE(unstamped.place(m, "map", tf, "simulator/camera"));
  for (int column = 0; column < 4; ++column)
    ASSERT_TRUE(glm::length(unstamped.world()[column] - result[column]) < 1e-5f);
  std::cout << "Detection timestamps, immutable placement, simulator TF fallback, and render registration passed\n";
}

// New bridge: only the truth base link is published, so a camera detection is placed at the truth base
// pose at the marker stamp composed with the pack's fixed base-to-camera transform.
TEST(HostDetectionPose, TruthBaseAcquisition) {
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer tf(clock);
    const auto setTruth = [&](int sec, double x, double yaw) {
        geometry_msgs::msg::TransformStamped t;
        t.header.frame_id = "map";
        t.child_frame_id = "simulator/talos/base_link";
        t.header.stamp.sec = sec;
        t.transform.translation.x = x;
        t.transform.rotation.z = std::sin(yaw / 2);
        t.transform.rotation.w = std::cos(yaw / 2);
        ASSERT_TRUE(tf.setTransform(t, "test"));
    };
    setTruth(10, 1, 0);
    setTruth(12, 3, 0);
    // Optical frame 0.1 m ahead of base_link looking along +X.
    const glm::mat4 baseToOptical = glm::translate(glm::mat4(1), glm::vec3(.1f, 0, 0)) * opticalToLink();
    glm::mat4 world(1);
    ASSERT_TRUE(truthAcquisitionPose(tf, "map", "simulator/talos/base_link", rclcpp::Time(11, 0), baseToOptical, world));
    ASSERT_TRUE(glm::length(glm::vec3(world[3]) - glm::vec3(2.1f, 0, 0)) < 1e-5f); // interpolated at acquisition
    // Not yet available: the caller retries, it never falls back to estimated TF.
    ASSERT_TRUE(!truthAcquisitionPose(tf, "map", "simulator/talos/base_link", rclcpp::Time(20, 0), baseToOptical, world));
    // Placement stays fixed after the robot moves on.
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "talos/ffc_left_camera_optical_frame";
    marker.header.stamp.sec = 11;
    marker.pose.position.z = 2; // 2 m in front of the optical frame (optical +Z)
    marker.pose.orientation.w = 1;
    DetectionPose placed;
    ASSERT_TRUE(truthAcquisitionPose(tf, "map", "simulator/talos/base_link", rclcpp::Time(11, 0), baseToOptical, world));
    ASSERT_TRUE(placed.place(marker, "map", tf, "", &world));
    const auto first = placed.world();
    ASSERT_TRUE(glm::length(glm::vec3(first[3]) - glm::vec3(4.1f, 0, 0)) < 1e-4f);
    setTruth(13, 30, 1.f);
    ASSERT_TRUE(placed.place(marker, "map", tf, "", &world));
    ASSERT_TRUE(glm::length(placed.world()[3] - first[3]) < 1e-6f);
}

namespace {
struct TfFixture {
  std::shared_ptr<rclcpp::Clock> clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf2_ros::Buffer tf{clock};
  void set(int sec, double x) {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "map";
    t.child_frame_id = "camera";
    t.header.stamp.sec = sec;
    t.transform.translation.x = x;
    t.transform.rotation.w = 1;
    ASSERT_TRUE(tf.setTransform(t, "test"));
  }
};
visualization_msgs::msg::Marker markerAt(int sec) {
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "camera";
  m.header.stamp.sec = sec;
  m.pose.position.x = 1;
  m.pose.orientation.w = 1;
  return m;
}
} // namespace

TEST(HostDetectionPose, EstimatePlacementExactThenImmutable) {
  TfFixture f;
  f.set(10, 0);
  f.set(12, 2);
  DetectionPose p;
  ASSERT_TRUE(p.placeViaTf(markerAt(11), "map", f.tf, 0.));
  ASSERT_TRUE(!p.approximate());
  EXPECT_NEAR(p.world()[3].x, 2.0, 1e-5); // 1 (interpolated) + marker 1
  f.set(13, 50);
  ASSERT_TRUE(p.placeViaTf(markerAt(11), "map", f.tf, 9.));
  EXPECT_NEAR(p.world()[3].x, 2.0, 1e-5);
}

TEST(HostDetectionPose, EstimatePlacementRetriesThenApproximates) {
  TfFixture f;
  f.set(10, 3);
  DetectionPose p;
  // TF has not reached the stamp: wait inside the retry window...
  ASSERT_TRUE(!p.placeViaTf(markerAt(20), "map", f.tf, 0.1));
  ASSERT_TRUE(!p.placed());
  // ...and use the exact transform if it arrives in time.
  f.set(21, 5);
  ASSERT_TRUE(p.placeViaTf(markerAt(20), "map", f.tf, 0.3));
  ASSERT_TRUE(!p.approximate());

  // Never arrives: after the window fall back to the latest transform and mark it approximate, once.
  TfFixture g;
  g.set(10, 3);
  DetectionPose q;
  ASSERT_TRUE(!q.placeViaTf(markerAt(20), "map", g.tf, 0.49));
  ASSERT_TRUE(q.placeViaTf(markerAt(20), "map", g.tf, 0.51));
  ASSERT_TRUE(q.approximate());
  EXPECT_NEAR(q.world()[3].x, 4.0, 1e-5);
  g.set(30, 100);
  ASSERT_TRUE(q.placeViaTf(markerAt(20), "map", g.tf, 5.));
  EXPECT_NEAR(q.world()[3].x, 4.0, 1e-5);

  // Unknown frame never places, even after the window.
  auto bad = markerAt(20);
  bad.header.frame_id = "nowhere";
  DetectionPose r;
  ASSERT_TRUE(!r.placeViaTf(bad, "map", g.tf, 5.));
}

TEST(HostDetectionPose, ZeroStampUsesLatestExactly) {
  TfFixture f;
  f.set(10, 3);
  DetectionPose p;
  ASSERT_TRUE(p.placeViaTf(markerAt(0), "map", f.tf, 0.));
  ASSERT_TRUE(!p.approximate());
  EXPECT_NEAR(p.world()[3].x, 4.0, 1e-5);
}

TEST(HostDetectionPose, ModeResolution) {
  EXPECT_TRUE(parseDetectionMode("both") == DetectionMode::Both);
  EXPECT_TRUE(parseDetectionMode("pose_source") == DetectionMode::PoseSource);
  EXPECT_THROW(parseDetectionMode("nope"), std::invalid_argument);
  // Simulator present: pose source follows the active source; both shows both placements.
  auto s = resolveDetectionMode(DetectionMode::PoseSource, true, true);
  EXPECT_TRUE(s.truth && !s.estimate);
  s = resolveDetectionMode(DetectionMode::PoseSource, true, false);
  EXPECT_TRUE(!s.truth && s.estimate);
  s = resolveDetectionMode(DetectionMode::Both, true, true);
  EXPECT_TRUE(s.truth && s.estimate && !s.downgraded);
  s = resolveDetectionMode(DetectionMode::Estimate, true, true);
  EXPECT_TRUE(!s.truth && s.estimate);
  // Real robot: truth and both are never offered; they collapse to the estimate.
  for (auto mode : {DetectionMode::Truth, DetectionMode::Both, DetectionMode::PoseSource, DetectionMode::Estimate}) {
    s = resolveDetectionMode(mode, false, false);
    EXPECT_TRUE(!s.truth && s.estimate);
    EXPECT_EQ(s.downgraded, mode == DetectionMode::Truth || mode == DetectionMode::Both);
  }
}

TEST(HostDetectionPose, PlacementsAreIndependentPerKind) {
  // Mode switching: each kind is placed once, lazily, so switching modes never moves an existing placement
  // and a later-enabled kind places at the same acquisition stamp.
  TfFixture f;
  f.set(10, 0);
  f.set(12, 2);
  DetectionPose truth, estimate;
  glm::mat4 acquisition = glm::translate(glm::mat4(1), glm::vec3(7, 0, 0));
  auto m = markerAt(11);
  ASSERT_TRUE(truth.place(m, "map", f.tf, "", &acquisition));
  EXPECT_NEAR(truth.world()[3].x, 8.0, 1e-5);
  f.set(13, 99);
  ASSERT_TRUE(estimate.placeViaTf(m, "map", f.tf, 1.));
  EXPECT_NEAR(estimate.world()[3].x, 2.0, 1e-5);
  EXPECT_NEAR(truth.world()[3].x, 8.0, 1e-5);
}

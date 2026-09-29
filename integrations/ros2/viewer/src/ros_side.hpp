// ROS-facing half of the host: TF, visual-state subscriptions, detections, MPC path, camera feeds.
// Callbacks run from RosSide::spin() on the render thread, so the GUI reads plain data; only camera JPEG
// decoding happens on a worker (results are collected in spin()).
#pragma once
#include "detection_pose.hpp"
#include "jpeg_decode.hpp"
#include "scenario.hpp"
#include "status_lights.hpp"
#include "tf_tree.hpp"
#include "thruster_visuals.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <chrono>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <set>

namespace robotics::ros_viewer::host {
using Clock = std::chrono::steady_clock;
using MarkerKey = std::pair<std::string, int>;

struct MarkerRecord {
    visualization_msgs::msg::Marker marker;
    Clock::time_point received;
    glm::mat4 pose{1};
    bool attached = false; // pose is relative to the truth base link
    std::filesystem::path mesh;
};
struct PlacedDetection {
    glm::mat4 pose;
    visualization_msgs::msg::Marker marker;
};
struct CameraFeed {
    const SensorCamera *camera = nullptr;
    std::vector<std::uint8_t> rgb, depth; // decoded previews (RGB8)
    int rgbWidth = 0, rgbHeight = 0, depthWidth = 0, depthHeight = 0;
    bool rgbDirty = false, depthDirty = false, wantDepth = false;
    // false: the card shows the viewer's own render from the truth pose; true ("What the stack sees"):
    // the bridge's published images. Depth always comes from the topic.
    bool rosMode = true;
    double hz = 0;
    std::uint64_t frames = 0;
    Clock::time_point lastFrame{};
    std::unique_ptr<AsyncJpegDecoder> decoder; // JPEG decoding runs off the UI thread
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr rgbSub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthSub;
    bool connected() const {
        return frames > 0 && std::chrono::duration<double>(Clock::now() - lastFrame).count() < 2.0;
    }
};
struct TfSnapshot {
    std::map<std::string, glm::mat4> frames; // enabled, resolved frames in the fixed frame
    std::string difference;
    int resolved = 0, missing = 0;
};

class RosSide {
  public:
    explicit RosSide(rclcpp::Node::SharedPtr node);
    // Latched scenario document (std_msgs/String JSON, transient_local).
    void watchScenario(const std::string &topic, std::function<void(const std::string &)> callback);
    // Create every subscription that depends on scenario data; also usable for a demo without a node graph
    // (subscriptions are skipped when `live` is false).
    void attach(const Scenario &, const YAML::Node &config, StatusLights &, ThrusterVisuals &, bool live);
    void spin();
    double now() const {
        return node_ ? node_->now().seconds() : 0.;
    }
    rclcpp::Node &node() {
        return *node_;
    }
    bool useSimTime() const {
        return node_ && node_->get_parameter("use_sim_time").as_bool();
    }

    // Truth pose. Returns true when a new stamp arrived; `first` marks the first pose ever received.
    bool updateTruth(glm::mat4 &body, bool &first);
    const std::string &status() const {
        return status_;
    }
    bool truthFresh() const {
        return fresh_;
    }
    void captureDetections(bool show);
    void captureMpc(bool wanted);
    void captureTf(bool wanted, TfTree &tree, TfSnapshot &out);
    void setCamerasWanted(bool wanted);
    void refreshCameras(); // re-evaluate RGB/depth subscriptions after wantDepth changes
    void publishMechanism(const std::string &topic, bool value);

    // Published state, read by the GUI.
    std::array<float, 2> claw{0.f, 0.f};
    std::map<MarkerKey, MarkerRecord> props, projectiles, magnetLights;
    std::vector<PlacedDetection> placedDetections;
    std::vector<glm::mat4> mpcPath;
    std::deque<CameraFeed> feeds;
    std::size_t detectionCount() const {
        return detectionMarkers_.size();
    }
    std::string mpcTopic, detectionTopic;

  private:
    struct DetectionEntry {
        visualization_msgs::msg::Marker marker;
        Clock::time_point received;
        DetectionPose placement;
    };
    void receiveMarkers(std::map<MarkerKey, MarkerRecord> &, const visualization_msgs::msg::MarkerArray &);
    void receiveDetections(const visualization_msgs::msg::MarkerArray &);
    void expire(std::map<MarkerKey, MarkerRecord> &);
    void subscribeCamera(CameraFeed &);
    glm::mat4 matrixOf(const geometry_msgs::msg::Transform &) const;

    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::unique_ptr<tf2_ros::Buffer> buffer_;
    std::unique_ptr<tf2_ros::TransformListener> listener_;
    const Scenario *scenario_ = nullptr;
    StatusLights *lights_ = nullptr;
    ThrusterVisuals *thrusters_ = nullptr;
    bool live_ = false, camerasWanted_ = true;
    std::string status_ = "WAITING FOR PHYSICS";
    bool fresh_ = false;
    double lastPoseStamp_ = -1;
    Clock::time_point lastPoseWall_{};
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr scenarioSub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr mpcSub_;
    nav_msgs::msg::Path mpcMessage_;
    bool mpcPending_ = false, mpcFailing_ = false;
    Clock::time_point mpcReceived_{}, mpcFailingSince_{};
    std::map<MarkerKey, DetectionEntry> detectionMarkers_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> mechanismCommands_;
    std::set<std::string> warnedMeshes_;
};

std::filesystem::path resolveMeshResource(const std::string &uri);
} // namespace robotics::ros_viewer::host

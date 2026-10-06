// ROS-facing half of the host: TF, visual-state subscriptions, detections, MPC path, camera feeds.
// Callbacks run from RosSide::spin() on the render thread, so the GUI reads plain data; only camera JPEG
// decoding happens on a worker (results are collected in spin()).
#pragma once
#include "detection_pose.hpp"
#include "display_clock.hpp"
#include "jpeg_decode.hpp"
#include "point_cloud_convert.hpp"
#include "scenario.hpp"
#include "status_lights.hpp"
#include "tf_tree.hpp"
#include "thruster_visuals.hpp"
#include <atomic>
#include <chrono>
#include <deque>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <set>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>
#include <visualization_msgs/msg/marker_array.hpp>

namespace nereus::ros_viewer::host {

using Clock = std::chrono::steady_clock;
// (ns, id): a marker's identity, as RViz keys it.
using MarkerKey = std::pair<std::string, int>;

// Where the robot model, camera poses, follow/focus and the course map take the vehicle pose from.
enum class PoseSource { Auto, Truth, Estimate };
PoseSource parsePoseSource(const std::string &); // throws on anything but auto|truth|estimate

// One received marker (a simulator prop, projectile or magnet light), kept until deleted or its lifetime ends.
struct MarkerRecord {
    visualization_msgs::msg::Marker marker;
    Clock::time_point received;
    glm::mat4 pose{1};
    bool attached = false; // pose is relative to the truth base link
    std::filesystem::path mesh;
};

// A detection marker placed in the fixed frame, by truth or by estimate (approximate: TF fell back to the latest).
struct PlacedDetection {
    enum class Kind { Truth, Estimate, EstimateApprox };
    glm::mat4 pose;
    visualization_msgs::msg::Marker marker;
    Kind kind = Kind::Truth;
};

// One scenario camera's preview card: decoded RGB and colorized depth for the GUI, and its subscriptions.
struct CameraFeed {
    const SensorCamera *camera = nullptr;
    std::vector<std::uint8_t> rgb, depth; // decoded previews (RGB8)
    int rgbWidth = 0, rgbHeight = 0, depthWidth = 0, depthHeight = 0;
    bool rgbDirty = false, depthDirty = false, wantDepth = false;

    // false: the card shows the viewer's own render from the truth pose; true (card source button "ROS (stack)"):
    // the bridge's published images. Depth always comes from the topic.
    bool rosMode = true;

    // Arrival rate and count.
    double hz = 0;
    std::uint64_t frames = 0;
    Clock::time_point lastFrame{};

    std::unique_ptr<AsyncJpegDecoder> decoder; // JPEG decoding runs off the UI thread
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr rgbSub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthSub;

    // A frame arrived within the last 2 s.
    bool connected() const {
        return frames > 0 && std::chrono::duration<double>(Clock::now() - lastFrame).count() < 2.0;
    }
};

// One configured PointCloud2 topic (host config `point_clouds:`). Subscribed only while enabled; like RViz's
// default (decay 0) only the newest message is shown, placed once in the fixed frame and then left in place.
struct PointCloudLayer {
    std::string id, title, topic;
    float size = 3;
    bool enabled = false;
    Eigen::Vector3f fallback{.9f, .9f, .9f}; // color for clouds without an rgb field

    // The newest message and where it was placed (world: cloud frame to fixed frame).
    std::shared_ptr<const rendering::PointData> data;
    std::string frame;
    builtin_interfaces::msg::Time stamp;
    Clock::time_point received{};
    bool placed = false, approximate = false;
    glm::mat4 world{1};

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;
};

// The TF overlay for one frame (captureTf()): enabled frames' poses and the truth-vs-estimate readout.
struct TfSnapshot {
    std::map<std::string, glm::mat4> frames; // enabled, resolved frames in the fixed frame
    std::string difference;
    int resolved = 0, missing = 0;
};

// Owns the viewer's ROS node: the TF buffer, subscriptions and publishers. spin() and the capture*() calls run
// once per frame on the render thread.
class RosSide {
  public:
    explicit RosSide(rclcpp::Node::SharedPtr node);
    ~RosSide();

    std::string timingReport(); // display-clock diagnostics since the last call (for --profile)
    RosSide(const RosSide &) = delete;
    RosSide &operator=(const RosSide &) = delete;

    // Latched scenario document (std_msgs/String JSON, transient_local).
    void watchScenario(const std::string &topic, std::function<void(const std::string &)> callback);
    // The simulator supervisor (sim.launch.py): its latched JSON status, and a request to restart the
    // simulator in another scenario pack folder.
    void watchSupervisor(const std::string &topic, std::function<void(const std::string &)> callback);
    void requestScenario(const std::string &topic, const std::string &folder);

    // Create every subscription that depends on scenario data; also usable for a demo without a node graph
    // (subscriptions are skipped when `live` is false).
    void attach(const Scenario &, const YAML::Node &config, StatusLights &, ThrusterVisuals &, bool live);
    // Runs pending callbacks and collects finished JPEG decodes.
    void spin();

    // ROS time (sim time with use_sim_time), 0 without a node.
    double now() const {
        return node_ ? node_->now().seconds() : 0.;
    }
    rclcpp::Node &node() {
        return *node_;
    }
    bool useSimTime() const {
        return node_ && node_->get_parameter("use_sim_time").as_bool();
    }

    // Display sampling: the robot pose (and every TF frame drawn) is looked up at a smooth display time that
    // trails the newest data, `truthDelay` seconds for simulator truth and `otherDelay` for estimate / other
    // frames (a 30 Hz EKF needs more than its period to interpolate).
    void configurePose(PoseSource source, double truthDelay, double otherDelay);
    // Vehicle pose from the active source at the display time. Returns true when a new stamp arrived;
    // `first` marks the first pose of the run (or after a source switch).
    bool updatePose(glm::mat4 &body, bool &first);
    const std::string &status() const {
        return status_;
    }
    // The active source's pose is arriving (wall-clock staleness under 1 s).
    bool poseFresh() const {
        return fresh_;
    }
    // Active pose source is the simulator truth frame (false: the localization estimate, e.g. a real robot).
    bool truthActive() const {
        return usingTruth_;
    }
    // The simulator truth frame has appeared in TF at least once.
    bool truthSeen() const {
        return truth_.seen;
    }
    PoseSource poseSource() const {
        return source_;
    }
    // TF frame of the active pose source.
    const std::string &poseFrame() const {
        return usingTruth_ ? scenario_->truthBaseFrame : scenario_->estimateBaseFrame;
    }

    // Expires markers and, when `show`, places the detections into placedDetections.
    void captureDetections(bool show);

    // Point clouds: (un)subscribe per layer and place pending messages (truth camera pose when the simulator
    // truth is the pose source, else TF at the stamp, latest after 0.5 s). Called once per frame.
    void capturePointClouds();
    std::vector<rendering::PointSet> pointSets() const;

    // Latest transform of `frame` in the fixed frame (false while TF cannot resolve it).
    bool latestInFixed(const std::string &frame, glm::mat4 &world) const;
    // truth * estimate^-1 with both base links sampled at the same (estimate display) time, so motion between
    // the two display clocks does not leak into the offset: only the estimation error remains.
    bool truthFromEstimate(glm::mat4 &offset);

    // The host config's point_clouds layers (rebuilt by attach()).
    std::vector<PointCloudLayer> pointClouds;

    // Which placement(s) of each detection to draw (requested); detectionShow() is what is effective now.
    void setDetectionMode(DetectionMode mode) {
        detectionMode_ = mode;
    }
    // false: ignore DELETEALL so observations live out their lifetime (the detector clears on every frame).
    void setHonorDeleteAll(bool honor) {
        honorDeleteAll_ = honor;
    }
    bool honorDeleteAll() const {
        return honorDeleteAll_;
    }
    DetectionMode detectionMode() const {
        return detectionMode_;
    }
    // Simulator truth exists and is (or may be) the pose source: truth/both placements are offered.
    bool truthPlacementAvailable() const {
        return source_ != PoseSource::Estimate && truth_.seen && (source_ == PoseSource::Truth || usingTruth_);
    }
    DetectionShow detectionShow() const {
        return resolveDetectionMode(detectionMode_, truthPlacementAvailable(), usingTruth_);
    }

    // MPC prediction (mpcPath, over its horizon) and the whole follow_path plan (plannedPath, latched,
    // placed through TF in the map frame like the course estimate), subscribed only while wanted.
    void captureMpc(bool wanted);
    // Commanded thruster forces (topics.thruster_forces, what RViz drew as thruster wrenches), subscribed only
    // while wanted. `thrust` holds the newest array in bridge order, empty when none arrived for 0.5 s.
    void captureThrust(bool wanted);
    // The TF overlay: refreshes `tree` from the buffer and samples its enabled frames at the display time.
    void captureTf(bool wanted, TfTree &tree, TfSnapshot &out);
    // Turns all camera subscriptions on or off.
    void setCamerasWanted(bool wanted);
    void refreshCameras(); // re-evaluate RGB/depth subscriptions after wantDepth changes
    // Publishes a std_msgs/Bool on one of the scenario's mechanism_controls topics (unknown topics are ignored).
    void publishMechanism(const std::string &topic, bool value);

    // Published state, read by the GUI.
    std::array<float, 2> claw{0.f, 0.f};
    std::map<MarkerKey, MarkerRecord> props, projectiles, magnetLights;
    std::vector<PlacedDetection> placedDetections;
    std::vector<glm::mat4> mpcPath, plannedPath;
    std::vector<float> thrust;
    std::deque<CameraFeed> feeds;

    std::size_t detectionCount() const {
        return detectionMarkers_.size();
    }
    // Resolved topic names (set by attach(), shown in the GUI).
    std::string mpcTopic, plannedTopic, detectionTopic, thrustTopic;

  private:
    // A detection marker and its placements (each placed once, see captureDetections()).
    struct DetectionEntry {
        visualization_msgs::msg::Marker marker;
        Clock::time_point received;
        DetectionPose truth, estimate;
    };

    // Marker bookkeeping.
    void receiveMarkers(std::map<MarkerKey, MarkerRecord> &, const visualization_msgs::msg::MarkerArray &);
    void receiveDetections(const visualization_msgs::msg::MarkerArray &);
    void expire(std::map<MarkerKey, MarkerRecord> &);

    // (Un)subscribes a feed's RGB and depth topics to match what is wanted.
    void subscribeCamera(CameraFeed &);
    // A TF transform as a 4x4 matrix.
    glm::mat4 matrixOf(const geometry_msgs::msg::Transform &) const;

    // Arrival tracking of one base-link frame in TF (see probe()): newest stamp (s) and when it changed.
    struct Probe {
        bool seen = false, fresh = false, changed = false;
        double stamp = -1;
        Clock::time_point wall{};
    };
    void probe(Probe &, const std::string &frame);
    double wallSeconds() const;
    // Pose of `frame` in the fixed frame at stamp `t` (seconds), else the latest transform; false when unknown.
    bool lookupAt(const std::string &frame, double t, bool useTime, glm::mat4 &out);
    bool wantsRos(const CameraFeed &feed) const {
        return feed.rosMode || !usingTruth_; // no simulator truth: only the ROS image topics exist
    }

    // ROS plumbing; lookupFallbacks_ counts timed lookups that fell back to the latest (for timingReport()).
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::unique_ptr<tf2_ros::Buffer> buffer_;
    std::atomic<int> lookupFallbacks_{0};

    // Set by attach(); not owned.
    const Scenario *scenario_ = nullptr;
    StatusLights *lights_ = nullptr;
    ThrusterVisuals *thrusters_ = nullptr;

    bool live_ = false, camerasWanted_ = true;

    // Pose source state (updatePose()).
    std::string status_ = "WAITING FOR PHYSICS";
    bool fresh_ = false, usingTruth_ = true, delivered_ = false;
    PoseSource source_ = PoseSource::Auto;
    double truthDelay_ = .02, otherDelay_ = .06;
    Probe truth_, estimate_;

    // Display clocks fed with exact arrival times by a dedicated receive thread (timingThread_), so frame
    // pacing and UI-thread spinning never distort them.
    DisplayClock truthClock_, estimateClock_;
    // Guards truthFrame_ / estimateFrame_, which the timing thread reads.
    std::mutex framesMutex_;
    std::string truthFrame_, estimateFrame_;
    rclcpp::Node::SharedPtr timingNode_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> timingExecutor_;
    rclcpp::SubscriptionBase::SharedPtr timingSub_, timingStaticSub_;
    std::thread timingThread_;

    // This frame's display times (updatePose()); origin_ is wallSeconds()' zero.
    bool haveTime_ = false;
    double truthTime_ = 0, otherTime_ = 0; // display stamps of this frame (valid when haveTime_)
    Clock::time_point origin_ = Clock::now();

    // Subscriptions and publishers.
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr scenarioSub_, supervisorSub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr scenarioRequest_;

    // MPC prediction: the newest message, placed by captureMpc() (retried at its stamp while TF trails).
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr mpcSub_;
    nav_msgs::msg::Path mpcMessage_;
    bool mpcPending_ = false, mpcFailing_ = false;
    Clock::time_point mpcReceived_{}, mpcFailingSince_{};

    // The planned path (latched) and the thruster forces.
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plannedSub_;
    nav_msgs::msg::Path plannedMessage_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr thrustSub_;
    Clock::time_point thrustReceived_{};

    // Detections.
    std::map<MarkerKey, DetectionEntry> detectionMarkers_;
    DetectionMode detectionMode_ = DetectionMode::PoseSource;
    bool warnedDetectionDowngrade_ = false, honorDeleteAll_ = true;

    // Mechanism publishers by absolute topic; mesh URIs already warned about.
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> mechanismCommands_;
    std::set<std::string> warnedMeshes_;
};

// A marker mesh URI (file://, package:// or an absolute path) as a local path; empty when it cannot be resolved.
std::filesystem::path resolveMeshResource(const std::string &uri);

} // namespace nereus::ros_viewer::host

// RosSide: the viewer's ROS subscriptions, the TF buffer fed from a dedicated thread, display-clock pose sampling
// and the per-frame placement of detections, point clouds and paths. See ros_side.hpp.
#include "ros_side.hpp"
#include "jpeg_decode.hpp"
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstring>
#include <iostream>
#ifdef NEREUS_VIEWER_UWRT
#include <riptide_msgs2/msg/led_command.hpp>
#endif

namespace nereus::ros_viewer::host {
namespace {

// Preview width (px): JPEG previews decode to at least this; depth images are subsampled down toward it.
constexpr int kPreviewWidth = 480;

// A geometry_msgs pose as a 4x4 matrix (a zero quaternion is taken as identity).
glm::mat4 poseOf(const geometry_msgs::msg::Pose &p) {
    glm::quat q(float(p.orientation.w), float(p.orientation.x), float(p.orientation.y), float(p.orientation.z));
    if (glm::length(q) < 1e-6f)
        q = glm::quat(1, 0, 0, 0);
    return poseQuat({float(p.position.x), float(p.position.y), float(p.position.z)}, q);
}

double seconds(const builtin_interfaces::msg::Duration &d) {
    return d.sec + d.nanosec * 1e-9;
}

// Turbo-like ramp: near = warm, far = cool, invalid = dark.
void colorize(float meters, float lo, float hi, std::uint8_t *out) {
    if (!std::isfinite(meters) || meters <= 0) {
        out[0] = out[1] = out[2] = 8;
        return;
    }
    const float t = std::clamp((meters - lo) / std::max(1e-3f, hi - lo), 0.f, 1.f);
    static const float stops[5][3] = {{255, 214, 84}, {96, 214, 110}, {48, 200, 210}, {60, 100, 220}, {60, 40, 110}};
    const float x = t * 4;
    const int i = std::min(3, int(x));
    const float f = x - float(i);
    for (int c = 0; c < 3; ++c)
        out[c] = std::uint8_t(stops[i][c] + (stops[i + 1][c] - stops[i][c]) * f);
}

} // namespace

std::filesystem::path resolveMeshResource(const std::string &uri) {
    const std::string file = "file://", package = "package://";
    if (uri.rfind(file, 0) == 0)
        return uri.substr(file.size());
    if (uri.rfind(package, 0) == 0) {
        const auto rest = uri.substr(package.size());
        const auto slash = rest.find('/');
        if (slash == std::string::npos)
            return {};
        try {
            return std::filesystem::path(ament_index_cpp::get_package_share_directory(rest.substr(0, slash))) /
                   rest.substr(slash + 1);
        } catch (const std::exception &) {
            return {};
        }
    }
    return std::filesystem::path(uri).is_absolute() ? std::filesystem::path(uri) : std::filesystem::path();
}

RosSide::RosSide(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {
    if (!node_)
        return; // demo: no ROS graph

    // The main node, spun by spin() on the render thread.
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    // TF is received on a dedicated thread (not once per rendered frame): each transform enters the buffer
    // and only then feeds the display clocks, so a display time never runs ahead of the buffered data.
    // Lookups never wait (zero timeout).
    buffer_->setUsingDedicatedThread(true);

    // A second node (no parameter services, same use_sim_time) for TF, spun on its own thread.
    rclcpp::NodeOptions options;
    options.start_parameter_services(false).start_parameter_event_publisher(false);
    options.parameter_overrides({rclcpp::Parameter("use_sim_time", node_->get_parameter("use_sim_time").as_bool())});
    timingNode_ =
        std::make_shared<rclcpp::Node>(node_->get_name() + std::string("_timing"), node_->get_namespace(), options);

    timingStaticSub_ = timingNode_->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf_static", rclcpp::QoS(100).reliable().transient_local(), [this](const tf2_msgs::msg::TFMessage &msg) {
            for (const auto &t : msg.transforms)
                buffer_->setTransform(t, "tf_static", true);
        });

    // /tf: buffer every transform first, then feed each base link's arrival time to its display clock.
    timingSub_ = timingNode_->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", rclcpp::QoS(100), [this](const tf2_msgs::msg::TFMessage &msg) {
            const double wall = wallSeconds();
            for (const auto &t : msg.transforms) {
                try {
                    buffer_->setTransform(t, "tf", false);
                } catch (const tf2::TransformException &) {
                }
            }
            std::string truth, estimate;
            {
                std::lock_guard<std::mutex> lock(framesMutex_);
                truth = truthFrame_;
                estimate = estimateFrame_;
            }
            for (const auto &t : msg.transforms) {
                const double stamp = rclcpp::Time(t.header.stamp).seconds();
                if (!truth.empty() && t.child_frame_id == truth)
                    truthClock_.observe(stamp, wall);
                else if (!estimate.empty() && t.child_frame_id == estimate)
                    estimateClock_.observe(stamp, wall);
            }
        });

    timingExecutor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    timingExecutor_->add_node(timingNode_);
    timingThread_ = std::thread([this] { timingExecutor_->spin(); });
}

RosSide::~RosSide() {
    if (timingExecutor_)
        timingExecutor_->cancel();
    if (timingThread_.joinable())
        timingThread_.join();
}

glm::mat4 RosSide::matrixOf(const geometry_msgs::msg::Transform &t) const {
    const auto &q = t.rotation;
    return poseQuat({float(t.translation.x), float(t.translation.y), float(t.translation.z)},
                    glm::quat(float(q.w), float(q.x), float(q.y), float(q.z)));
}

void RosSide::watchScenario(const std::string &topic, std::function<void(const std::string &)> callback) {
    if (!node_)
        return;
    scenarioSub_ = node_->create_subscription<std_msgs::msg::String>(
        topic, rclcpp::QoS(1).reliable().transient_local(),
        [callback](const std_msgs::msg::String &msg) { callback(msg.data); });
}

void RosSide::watchSupervisor(const std::string &topic, std::function<void(const std::string &)> callback) {
    if (!node_)
        return;
    supervisorSub_ = node_->create_subscription<std_msgs::msg::String>(
        topic, rclcpp::QoS(1).reliable().transient_local(),
        [callback](const std_msgs::msg::String &msg) { callback(msg.data); });
}

void RosSide::requestScenario(const std::string &topic, const std::string &folder) {
    if (!node_)
        return;
    if (!scenarioRequest_ || scenarioRequest_->get_topic_name() != topic)
        scenarioRequest_ = node_->create_publisher<std_msgs::msg::String>(topic, rclcpp::QoS(1).reliable());

    // (Re)create the publisher when the topic changes.
    std_msgs::msg::String message;
    message.data = folder;
    scenarioRequest_->publish(message);
}

void RosSide::spin() {
    if (node_)
        executor_->spin_some();

    // Collect JPEGs the worker finished (newest only) for the GUI to upload.
    for (auto &feed : feeds) {
        DecodedImage image;
        if (feed.decoder && feed.decoder->take(image)) {
            feed.rgbWidth = image.width;
            feed.rgbHeight = image.height;
            feed.rgb = std::move(image.rgb);
            feed.rgbDirty = true;
        }
    }
}

std::string RosSide::timingReport() {
    const auto truth = truthClock_.takeCounts(), estimate = estimateClock_.takeCounts();
    char line[200];
    std::snprintf(line, sizeof(line),
                  "\n  pose timing: truth re-anchors %d holds %d delay %.0f ms | estimate re-anchors %d holds "
                  "%d delay %.0f ms | lookup fallbacks %d",
                  truth.first, truth.second, 1e3 * truthClock_.delay(truthDelay_), estimate.first, estimate.second,
                  1e3 * estimateClock_.delay(otherDelay_), lookupFallbacks_.exchange(0));
    return line;
}

void RosSide::attach(const Scenario &scenario, const YAML::Node &config, StatusLights &lights,
                     ThrusterVisuals &thrusters, bool live) {
    scenario_ = &scenario;
    {
        std::lock_guard<std::mutex> lock(framesMutex_);
        truthFrame_ = scenario.truthBaseFrame;
        estimateFrame_ = scenario.estimateBaseFrame;
    }

    lights_ = &lights;
    thrusters_ = &thrusters;
    live_ = live;

    // Drop the previous scenario's subscriptions and feeds.
    subscriptions_.clear();
    feeds.clear();
    // Camera feeds exist even without ROS (demo cards show PREVIEW ONLY).
    for (const auto &camera : scenario.cameras) {
        feeds.emplace_back();
        feeds.back().camera = &camera;
    }

    // Topics from the host config (`topics:`), resolved in the bridge namespace.
    mpcTopic =
        scenario.absolute(lookup(config, {"topics", "mpc_path"}).as<std::string>("controller/mpc/predicted_path"));
    plannedTopic =
        scenario.absolute(lookup(config, {"topics", "planned_path"}).as<std::string>("controller/mpc/planned_path"));

    // Point cloud layers; capturePointClouds() subscribes the enabled ones.
    pointClouds.clear();
    for (const auto &entry : config["point_clouds"]) {
        PointCloudLayer layer;
        layer.id = entry["id"].as<std::string>();
        layer.title = entry["title"].as<std::string>(layer.id);
        layer.topic = scenario.absolute(entry["topic"].as<std::string>());
        layer.size = std::clamp(entry["size"].as<float>(3), 1.f, 16.f);
        layer.enabled = entry["enabled"].as<bool>(false);
        if (entry["color"] && entry["color"].IsSequence() && entry["color"].size() == 3)
            layer.fallback = {entry["color"][0].as<float>(), entry["color"][1].as<float>(),
                              entry["color"][2].as<float>()};
        pointClouds.push_back(std::move(layer));
    }

    detectionTopic = scenario.absolute(
        lookup(config, {"topics", "detections"}).as<std::string>("yolo_orientation/visualization_marker_array"));
    thrustTopic = scenario.absolute(lookup(config, {"topics", "thruster_forces"}).as<std::string>("thruster_forces"));
    thrustSub_.reset(); // captureThrust subscribes again on the new topic
    thrust.clear();

    // Everything below needs a ROS graph.
    if (!live)
        return;

    const auto topic = [&](const char *key, const char *fallback) {
        return scenario.absolute(lookup(config, {"topics", key}).as<std::string>(fallback));
    };

    // Realized thruster forces animate the propellers.
    if (!thrusters.rotors.empty())
        subscriptions_.push_back(node_->create_subscription<std_msgs::msg::Float32MultiArray>(
            scenario.absolute(thrusters.topic), 10, [this](const std_msgs::msg::Float32MultiArray &msg) {
                if (!thrusters_->receive(msg.data, now()))
                    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                         "Ignoring invalid realized thruster forces for propeller animation");
            }));

    // Status lights: an LED command (UWRT builds) or a plain color.
    if (lights.input == "riptide_msgs2/msg/LedCommand") {
#ifdef NEREUS_VIEWER_UWRT
        subscriptions_.push_back(node_->create_subscription<riptide_msgs2::msg::LedCommand>(
            scenario.absolute(lights.topic), 10, [this](const riptide_msgs2::msg::LedCommand &msg) {
                using Command = riptide_msgs2::msg::LedCommand;
                LightMode mode;
                switch (msg.mode) {
                case Command::MODE_SOLID:
                    mode = LightMode::Solid;
                    break;
                case Command::MODE_SLOW_FLASH:
                    mode = LightMode::SlowFlash;
                    break;
                case Command::MODE_FAST_FLASH:
                    mode = LightMode::FastFlash;
                    break;
                case Command::MODE_BREATH:
                    mode = LightMode::Breath;
                    break;
                case Command::SINGLETON_FLASH:
                    mode = LightMode::Flash;
                    break;
                default:
                    return;
                }
                if (msg.target > Command::TARGET_ALL)
                    return;
                lights_->command(glm::vec3(msg.red, msg.green, msg.blue) / 255.f, mode, msg.target, now());
            }));
#else
        std::cerr << "nereus-viewer: built without riptide_msgs2; LED commands are not shown\n";
#endif
    } else if (lights.input == "std_msgs/msg/ColorRGBA") {
        subscriptions_.push_back(node_->create_subscription<std_msgs::msg::ColorRGBA>(
            scenario.absolute(lights.topic), 10, [this](const std_msgs::msg::ColorRGBA &msg) {
                if (!std::isfinite(msg.a))
                    return;
                lights_->command(glm::vec3(msg.r, msg.g, msg.b) * std::clamp(msg.a, 0.f, 1.f), LightMode::Solid,
                                 UINT32_MAX, now());
            }));
    }

    // The claw's two joint values.
    subscriptions_.push_back(node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        topic("claw_joints", "simulator/claw_joints"), 10, [this](const std_msgs::msg::Float64MultiArray &msg) {
            if (msg.data.size() == 2 && std::isfinite(msg.data[0]) && std::isfinite(msg.data[1]))
                claw = {float(msg.data[0]), float(msg.data[1])};
        }));

    // Simulator marker arrays: magnet lights, props and projectiles.
    const auto markers = [this](std::map<MarkerKey, MarkerRecord> &target) {
        return [this, &target](const visualization_msgs::msg::MarkerArray &msg) { receiveMarkers(target, msg); };
    };
    subscriptions_.push_back(node_->create_subscription<visualization_msgs::msg::MarkerArray>(
        topic("magnet_lights", "simulator/magnet_lights"), 10, markers(magnetLights)));
    subscriptions_.push_back(node_->create_subscription<visualization_msgs::msg::MarkerArray>(
        topic("task_objects", "simulator/task_objects"), 10, markers(props)));
    subscriptions_.push_back(node_->create_subscription<visualization_msgs::msg::MarkerArray>(
        topic("projectiles", "simulator/projectiles"), 10, markers(projectiles)));

    // Same marker array RViz shows; each observation is placed once at its acquisition pose.
    subscriptions_.push_back(node_->create_subscription<visualization_msgs::msg::MarkerArray>(
        detectionTopic, 10, [this](const visualization_msgs::msg::MarkerArray &msg) { receiveDetections(msg); }));

    // Mechanism buttons publish std_msgs/Bool on the scenario's mechanism_controls topics.
    for (const auto &control : scenario.ui["mechanism_controls"])
        mechanismCommands_[scenario.absolute(control["topic"].as<std::string>())] =
            node_->create_publisher<std_msgs::msg::Bool>(scenario.absolute(control["topic"].as<std::string>()), 10);

    for (auto &feed : feeds)
        subscribeCamera(feed);
}

// Creates or drops a feed's RGB and depth subscriptions to match what is wanted; safe to call repeatedly.
void RosSide::subscribeCamera(CameraFeed &feed) {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    if (!feed.camera->rgbTopic.empty() && !feed.rgbSub && camerasWanted_ && wantsRos(feed)) {
        if (!feed.decoder)
            feed.decoder = std::make_unique<AsyncJpegDecoder>(kPreviewWidth);
        feed.rgbSub = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
            feed.camera->rgbTopic, qos, [&feed](const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg) {
                // Arrival bookkeeping only; the decode (and its cost) belongs to the worker.
                const auto now = Clock::now();
                if (feed.frames > 0) {
                    const double dt = std::chrono::duration<double>(now - feed.lastFrame).count();
                    if (dt > 1e-4)
                        feed.hz = feed.hz > 0 ? feed.hz * .8 + .2 / dt : 1. / dt;
                }
                feed.lastFrame = now;
                ++feed.frames;
                feed.decoder->submit(msg->data);
            });
    }

    // Depth: 32FC1 meters, subsampled and colorized in the callback (on the render thread).
    if (!feed.camera->depthTopic.empty() && feed.wantDepth && camerasWanted_ && !feed.depthSub)
        feed.depthSub = node_->create_subscription<sensor_msgs::msg::Image>(
            feed.camera->depthTopic, qos, [&feed](const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
                if (msg->encoding != "32FC1" || msg->width == 0 || msg->height == 0 ||
                    msg->data.size() < std::size_t(msg->step) * msg->height)
                    return;
                const int stride = std::max(1, int(msg->width) / kPreviewWidth);
                const int w = int(msg->width) / stride, h = int(msg->height) / stride;
                feed.depth.resize(std::size_t(w) * std::size_t(h) * 3);
                for (int y = 0; y < h; ++y)
                    for (int x = 0; x < w; ++x) {
                        float meters;
                        std::memcpy(&meters,
                                    msg->data.data() + std::size_t(y * stride) * msg->step +
                                        std::size_t(x * stride) * sizeof(float),
                                    sizeof(float));
                        colorize(meters, float(feed.camera->minRange), float(feed.camera->maxRange),
                                 feed.depth.data() + (std::size_t(y) * std::size_t(w) + std::size_t(x)) * 3);
                    }
                feed.depthWidth = w;
                feed.depthHeight = h;
                feed.depthDirty = true;
            });

    // Drop what is no longer wanted.
    if (!feed.wantDepth || !camerasWanted_)
        feed.depthSub.reset();
    if (!camerasWanted_ || !wantsRos(feed))
        feed.rgbSub.reset();
}

void RosSide::setCamerasWanted(bool wanted) {
    camerasWanted_ = wanted;
    if (live_)
        for (auto &feed : feeds)
            subscribeCamera(feed);
}

void RosSide::refreshCameras() {
    if (live_)
        for (auto &feed : feeds)
            subscribeCamera(feed);
}

void RosSide::publishMechanism(const std::string &topic, bool value) {
    const auto it = mechanismCommands_.find(scenario_ ? scenario_->absolute(topic) : topic);
    if (it == mechanismCommands_.end())
        return;
    std_msgs::msg::Bool msg;
    msg.data = value;
    it->second->publish(msg);
}

PoseSource parsePoseSource(const std::string &text) {
    if (text == "auto")
        return PoseSource::Auto;
    if (text == "truth")
        return PoseSource::Truth;
    if (text == "estimate")
        return PoseSource::Estimate;
    throw std::runtime_error("pose_source must be auto, truth or estimate (got '" + text + "')");
}

void RosSide::configurePose(PoseSource source, double truthDelay, double otherDelay) {
    source_ = source;
    truthDelay_ = std::clamp(truthDelay, 0., 1.);
    otherDelay_ = std::clamp(otherDelay, 0., 1.);
    usingTruth_ = source != PoseSource::Estimate;
}

// Seconds on the steady clock since construction (the display clocks' wall time).
double RosSide::wallSeconds() const {
    return std::chrono::duration<double>(Clock::now() - origin_).count();
}

// Newest stamp of `frame` in the fixed frame; feeds the display clock and the wall-clock staleness test.
void RosSide::probe(Probe &p, const std::string &frame) {
    p.changed = false;
    try {
        const auto t = buffer_->lookupTransform(scenario_->mapFrame, frame, tf2::TimePointZero);
        const double stamp = rclcpp::Time(t.header.stamp).seconds();
        p.seen = true;
        if (stamp != p.stamp) {
            p.stamp = stamp;
            p.wall = Clock::now();
            p.changed = true;
        }
    } catch (const tf2::TransformException &) {
    }
    p.fresh = p.seen && std::chrono::duration<double>(Clock::now() - p.wall).count() < 1.;
}

bool RosSide::lookupAt(const std::string &frame, double t, bool useTime, glm::mat4 &out) {
    const auto &map = scenario_->mapFrame;
    if (frame == map) {
        out = glm::mat4(1);
        return true;
    }
    if (useTime) {
        try {
            out =
                matrixOf(buffer_->lookupTransform(map, frame, rclcpp::Time(int64_t(t * 1e9), RCL_ROS_TIME)).transform);
            return true;
        } catch (const tf2::TransformException &) {
            ++lookupFallbacks_;
        } // outside the buffered range for this frame: fall back to its latest transform
    }
    try {
        out = matrixOf(buffer_->lookupTransform(map, frame, tf2::TimePointZero).transform);
        return true;
    } catch (const tf2::TransformException &) {
        return false;
    }
}

bool RosSide::updatePose(glm::mat4 &body, bool &first) {
    first = false;
    haveTime_ = false;
    if (!scenario_ || !live_)
        return false;
    probe(truth_, scenario_->truthBaseFrame);
    probe(estimate_, scenario_->estimateBaseFrame);

    // Choose the source.
    bool truth;
    switch (source_) {
    case PoseSource::Truth:
        truth = true;
        break;
    case PoseSource::Estimate:
        truth = false;
        break;
    default: // simulator truth while it is alive, else the estimate; before any pose, truth until proven absent
        truth = truth_.fresh || (!estimate_.fresh && (truth_.seen || !estimate_.seen));
    }

    const bool switched = truth != usingTruth_;
    usingTruth_ = truth;
    if (switched) {
        delivered_ = false; // refocus on the new source
        refreshCameras();   // without truth the cards fall back to the ROS image topics
    }

    Probe &src = truth ? truth_ : estimate_;
    if (!src.seen) {
        fresh_ = false;
        status_ = source_ == PoseSource::Auto ? "WAITING FOR POSE"
                  : truth                     ? "WAITING FOR PHYSICS"
                                              : "WAITING FOR ESTIMATE";
        return false;
    }
    fresh_ = src.fresh;
    status_ =
        truth ? (src.fresh ? "PHYSICS CONNECTED" : "POSE STALE") : (src.fresh ? "ROBOT (ESTIMATE)" : "ESTIMATE STALE");

    // The active source's display clock places the robot; frames of the estimate stream (and the rest of
    // the TF tree) use the estimate clock, each sampled once per frame.
    const double wall = wallSeconds();
    auto &clock = truth ? truthClock_ : estimateClock_;
    if (!clock.valid()) // no arrival observed yet on the timing thread: fall back to the latest pose
        clock.observe(src.stamp, wall);
    truthTime_ = clock.at(wall, truth ? truthDelay_ : otherDelay_);
    otherTime_ = truth && estimateClock_.valid() ? estimateClock_.at(wall, otherDelay_)
                                                 : truthTime_ - (truth ? otherDelay_ - truthDelay_ : 0.);
    haveTime_ = true;

    glm::mat4 pose;
    if (lookupAt(poseFrame(), truth ? truthTime_ : otherTime_, true, pose))
        body = pose;

    if (src.changed && !delivered_) {
        first = true;
        delivered_ = true;
    }
    return src.changed;
}

// Drops records whose marker lifetime has run out (lifetime 0: forever).
void RosSide::expire(std::map<MarkerKey, MarkerRecord> &records) {
    const auto now = Clock::now();
    for (auto it = records.begin(); it != records.end();) {
        const double life = seconds(it->second.marker.lifetime);
        if (life > 0 && std::chrono::duration<double>(now - it->second.received).count() > life)
            it = records.erase(it);
        else
            ++it;
    }
}

// Applies a MarkerArray to `records` (add / DELETE / DELETEALL). Only markers in the map frame or on a base link
// (attached: they move with the robot) are kept.
void RosSide::receiveMarkers(std::map<MarkerKey, MarkerRecord> &records,
                             const visualization_msgs::msg::MarkerArray &msg) {
    using Marker = visualization_msgs::msg::Marker;
    for (const auto &m : msg.markers) {
        if (m.action == Marker::DELETEALL) {
            records.clear();
            continue;
        }
        if (m.action == Marker::DELETE) {
            records.erase({m.ns, m.id});
            continue;
        }

        const bool attached =
            m.header.frame_id == scenario_->estimateBaseFrame || m.header.frame_id == scenario_->truthBaseFrame;
        if (!attached && m.header.frame_id != scenario_->mapFrame)
            continue;

        MarkerRecord record;
        record.marker = m;
        record.received = Clock::now();
        record.pose = poseOf(m.pose);
        record.attached = attached;
        if (m.type == Marker::MESH_RESOURCE) {
            record.mesh = resolveMeshResource(m.mesh_resource);
            if (record.mesh.empty() && warnedMeshes_.insert(m.mesh_resource).second)
                std::cerr << "nereus-viewer: cannot resolve mesh resource " << m.mesh_resource << '\n';
        }
        records[{m.ns, m.id}] = std::move(record);
    }
}

// Stores detection markers; captureDetections() places them.
void RosSide::receiveDetections(const visualization_msgs::msg::MarkerArray &msg) {
    using Marker = visualization_msgs::msg::Marker;
    const auto now = Clock::now();
    for (const auto &m : msg.markers) {
        if (m.action == Marker::DELETEALL) {
            if (honorDeleteAll_)
                detectionMarkers_.clear();
        } else if (m.action == Marker::DELETE)
            detectionMarkers_.erase({m.ns, m.id});
        else
            detectionMarkers_[{m.ns, m.id}] = DetectionEntry{m, now, {}, {}};
    }
}

// Each observation is placed once per placement kind and then never follows later TF. Truth: the truth base
// link at the marker stamp composed with the pack's fixed base-to-camera transform. Estimate: TF of the marker
// frame at the stamp through the localization frames (RViz-like), approximate after the retry window.
void RosSide::captureDetections(bool show) {
    placedDetections.clear();
    expire(props);
    expire(projectiles);
    expire(magnetLights);

    // Subscription callbacks run even while the overlay is hidden. Expire markers independently of
    // drawing so unique IDs cannot pile up.
    const auto now = Clock::now();
    for (auto it = detectionMarkers_.begin(); it != detectionMarkers_.end();) {
        const double life = seconds(it->second.marker.lifetime);
        if (life > 0 && std::chrono::duration<double>(now - it->second.received).count() > life)
            it = detectionMarkers_.erase(it);
        else
            ++it;
    }

    if (!show || !live_ || !scenario_)
        return;

    const DetectionShow shown = detectionShow();
    if (shown.downgraded && !warnedDetectionDowngrade_) {
        warnedDetectionDowngrade_ = true;
        std::cerr << "nereus-viewer: no simulator truth; detection placement truth/both treated as estimate\n";
    }

    for (auto &[key, entry] : detectionMarkers_) {
        const auto &m = entry.marker;
        const double age = std::chrono::duration<double>(now - entry.received).count();
        if (shown.truth) {
            // Truth: through the camera's fixed mount when the marker is in a camera frame, else plain TF.
            const glm::mat4 *baseToCamera = nullptr;
            for (const auto &c : scenario_->cameras) {
                if (m.header.frame_id == c.rosOpticalFrame)
                    baseToCamera = &c.opticalInBase;
                else if (m.header.frame_id == scenario_->rosFrame(c.mountFrame))
                    baseToCamera = &c.mountInBase;
                if (baseToCamera)
                    break;
            }
            bool placed;
            if (baseToCamera) {
                const rclcpp::Time stamp(m.header.stamp);
                glm::mat4 acquisition;
                if (truthAcquisitionPose(*buffer_, scenario_->mapFrame, scenario_->truthBaseFrame, stamp, *baseToCamera,
                                         acquisition)) {
                    auto stamped = m;
                    if (stamp.nanoseconds() == 0)
                        stamped.header.stamp.nanosec = 1; // zero stamps use the latest truth for initial placement only
                    placed = entry.truth.place(stamped, scenario_->mapFrame, *buffer_, "", &acquisition);
                } else {
                    placed = entry.truth.placed(); // truth TF has not reached the stamp; retry next frame
                }
            } else {
                placed = entry.truth.place(m, scenario_->mapFrame, *buffer_);
            }
            if (placed)
                placedDetections.push_back({entry.truth.world(), m, PlacedDetection::Kind::Truth});
        }

        // Estimate: TF at the stamp, approximate after the retry window.
        if (shown.estimate && entry.estimate.placeViaTf(m, scenario_->mapFrame, *buffer_, age))
            placedDetections.push_back({entry.estimate.world(), m,
                                        entry.estimate.approximate() ? PlacedDetection::Kind::EstimateApprox
                                                                     : PlacedDetection::Kind::Estimate});
    }
}

void RosSide::capturePointClouds() {
    const auto now = Clock::now();
    for (std::size_t index = 0; index < pointClouds.size(); ++index) {
        auto &layer = pointClouds[index];

        const bool wanted = layer.enabled && live_ && scenario_;

        // Subscribed while enabled; disabling drops the data.
        if (wanted && !layer.subscription) {
            layer.subscription = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
                layer.topic, rclcpp::QoS(1).best_effort(), [this, index](const sensor_msgs::msg::PointCloud2 &msg) {
                    auto &target = pointClouds[index];
                    auto data = convertPointCloud(msg, target.fallback);
                    if (!data)
                        return;
                    target.data = std::move(data);
                    target.frame = msg.header.frame_id;
                    target.stamp = msg.header.stamp;
                    target.received = Clock::now();
                    target.placed = target.approximate = false;
                });
        } else if (!wanted && layer.subscription) {
            layer.subscription.reset();
            layer.data.reset();
        }

        if (!layer.data || layer.placed || !scenario_)
            continue;
        const rclcpp::Time stamp(layer.stamp);
        if (truthActive()) {
            // Simulator: place camera clouds from the truth pose at capture, like truth detections.
            const glm::mat4 *baseToCamera = nullptr;
            for (const auto &c : scenario_->cameras) {
                if (layer.frame == c.rosOpticalFrame)
                    baseToCamera = &c.opticalInBase;
                else if (layer.frame == scenario_->rosFrame(c.mountFrame))
                    baseToCamera = &c.mountInBase;
                if (baseToCamera)
                    break;
            }
            if (baseToCamera) {
                layer.placed = truthAcquisitionPose(*buffer_, scenario_->mapFrame, scenario_->truthBaseFrame, stamp,
                                                    *baseToCamera, layer.world);
                continue;
            }
        }

        // Else TF at the stamp; after 0.5 s, the latest transform (marked approximate).
        const auto lookup = [&](const rclcpp::Time &at) {
            try {
                layer.world = matrixOf(buffer_->lookupTransform(scenario_->mapFrame, layer.frame, at).transform);
                return true;
            } catch (const tf2::TransformException &) {
                return false;
            }
        };
        if (lookup(stamp))
            layer.placed = true;
        else if (std::chrono::duration<double>(now - layer.received).count() > .5 &&
                 lookup(rclcpp::Time(0, 0, stamp.get_clock_type())))
            layer.placed = layer.approximate = true;
    }
}

bool RosSide::truthFromEstimate(glm::mat4 &offset) {
    glm::mat4 truth, estimate;
    if (!usingTruth_ || !haveTime_ || !scenario_ ||
        !lookupAt(scenario_->estimateBaseFrame, otherTime_, true, estimate) ||
        !lookupAt(scenario_->truthBaseFrame, otherTime_, true, truth))
        return false;
    offset = truth * glm::inverse(estimate);
    return true;
}

bool RosSide::latestInFixed(const std::string &frame, glm::mat4 &world) const {
    if (!scenario_ || !buffer_)
        return false;
    try {
        world = matrixOf(buffer_->lookupTransform(scenario_->mapFrame, frame, tf2::TimePointZero).transform);
        return true;
    } catch (const tf2::TransformException &) {
        return false;
    }
}

// The enabled, placed clouds for the renderer.
std::vector<rendering::PointSet> RosSide::pointSets() const {
    std::vector<rendering::PointSet> sets;
    for (const auto &layer : pointClouds)
        if (layer.enabled && layer.data && layer.placed) {
            rendering::PointSet set;
            set.data = layer.data;
            for (int c = 0; c < 4; ++c)
                for (int r = 0; r < 4; ++r)
                    set.transform(r, c) = layer.world[c][r];
            set.size = layer.size;
            sets.push_back(std::move(set));
        }
    return sets;
}

// The controller steers its estimate, so the true vehicle follows the same motion relative to itself:
// place the prediction relative to the estimated base link, then re-root it at the truth base link.
void RosSide::captureMpc(bool wanted) {
    wanted = wanted && live_ && scenario_;

    // The prediction: the newest message, placed below.
    if (wanted && !mpcSub_) {
        mpcSub_ = node_->create_subscription<nav_msgs::msg::Path>(mpcTopic, 10, [this](const nav_msgs::msg::Path &msg) {
            mpcMessage_ = msg;
            mpcPending_ = true;
            mpcReceived_ = Clock::now();
        });
    } else if (!wanted && mpcSub_) {
        mpcSub_.reset();
        mpcPending_ = false;
        mpcPath.clear();
    }

    // The planned path: latched, cleared when unwanted.
    if (wanted && !plannedSub_) {
        plannedSub_ = node_->create_subscription<nav_msgs::msg::Path>(
            plannedTopic, rclcpp::QoS(1).transient_local(),
            [this](const nav_msgs::msg::Path &msg) { plannedMessage_ = msg; });
    } else if (!wanted && plannedSub_) {
        plannedSub_.reset();
        plannedMessage_ = nav_msgs::msg::Path();
        plannedPath.clear();
    }

    if (!wanted)
        return;
    // The plan is fixed in the odometry frame: re-place it with the latest TF (empty message: no path).
    if (plannedMessage_.poses.empty()) {
        plannedPath.clear();
    } else {
        try {
            const auto toMap = matrixOf(buffer_
                                            ->lookupTransform(scenario_->mapFrame, plannedMessage_.header.frame_id,
                                                              rclcpp::Time(0, 0, RCL_ROS_TIME))
                                            .transform);
            plannedPath.clear();
            for (const auto &pose : plannedMessage_.poses)
                plannedPath.push_back(toMap * poseOf(pose.pose));
        } catch (const tf2::TransformException &) {
        } // keep the last placement until TF has the frame
    }

    // The prediction, re-rooted at the active pose source's base link.
    const double age = std::chrono::duration<double>(Clock::now() - mpcReceived_).count();
    if (mpcPending_) {
        // Solves are stamped at compute time; TF can trail that by a few ms. Retry at the stamp briefly
        // (measured from the first failed attempt, not the latest message), then settle for the latest
        // transforms.
        const bool settle = mpcFailing_ && std::chrono::duration<double>(Clock::now() - mpcFailingSince_).count() > .2;
        const rclcpp::Time stamp = settle ? rclcpp::Time(0, 0, RCL_ROS_TIME) : rclcpp::Time(mpcMessage_.header.stamp);
        try {
            const auto estimated = matrixOf(
                buffer_->lookupTransform(scenario_->estimateBaseFrame, mpcMessage_.header.frame_id, stamp).transform);
            const auto truth = matrixOf(buffer_->lookupTransform(scenario_->mapFrame, poseFrame(), stamp).transform);
            mpcPath.clear();
            for (const auto &pose : mpcMessage_.poses)
                mpcPath.push_back(truth * estimated * poseOf(pose.pose));
            mpcPending_ = false;
            mpcFailing_ = false;
        } catch (const tf2::TransformException &) {
            if (!mpcFailing_) {
                mpcFailing_ = true;
                mpcFailingSince_ = Clock::now();
            }
        }
    }

    if (age > .5)
        mpcPath.clear(); // controller disabled or gone
}

void RosSide::captureThrust(bool wanted) {
    wanted = wanted && live_ && scenario_ && !scenario_->thrusterMounts.empty();
    if (wanted && !thrustSub_)
        thrustSub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
            thrustTopic, 10, [this](const std_msgs::msg::Float32MultiArray &msg) {
                if (msg.data.size() != scenario_->thrusterOrder.size() ||
                    !std::all_of(msg.data.begin(), msg.data.end(), [](float f) { return std::isfinite(f); })) {
                    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                         "Ignoring invalid thruster forces on %s", thrustTopic.c_str());
                    return;
                }
                thrust = msg.data;
                thrustReceived_ = Clock::now();
            });
    else if (!wanted && thrustSub_)
        thrustSub_.reset();

    if (!wanted || std::chrono::duration<double>(Clock::now() - thrustReceived_).count() > .5)
        thrust.clear(); // controller stopped publishing
}

// Samples the TF overlay for this frame, and the truth-vs-estimate readout when the simulator is the pose source.
void RosSide::captureTf(bool wanted, TfTree &tree, TfSnapshot &out) {
    out = {};
    if (!wanted || !live_ || !scenario_)
        return;

    // Rebuild the frame tree from the buffer (the map is a root; a parent not yet seen becomes a root).
    const auto &map = scenario_->mapFrame;
    std::map<std::string, std::string> parents;
    parents[map] = "";
    std::vector<std::string> frames;
    buffer_->_getFrameStrings(frames);
    for (const auto &name : frames) {
        std::string parent;
        buffer_->_getParent(name, tf2::TimePointZero, parent);
        parents[name] = parent;
        if (!parent.empty())
            parents.try_emplace(parent, "");
    }
    tree.update(parents);

    // Freeze the overlay before rendering: looking up each axis during drawing would sample a newer pose
    // than the already-rendered vehicle. Frames are sampled at the same display time as the robot model
    // (truth frames and their static children at the truth delay, everything else at the other delay),
    // falling back to the latest transform where the buffer has no data at that time.
    const auto &truthRoot = scenario_->truthBaseFrame;
    const auto underTruth = [&](std::string name) {
        for (int hops = 0; hops < 64 && !name.empty(); ++hops) {
            if (name == truthRoot)
                return true;
            const auto it = parents.find(name);
            if (it == parents.end())
                return false;
            name = it->second;
        }
        return false;
    };

    for (const auto &[name, parent] : parents) {
        auto &frame = tree.frames.at(name);
        glm::mat4 pose;
        if (lookupAt(name, underTruth(name) && usingTruth_ ? truthTime_ : otherTime_, haveTime_, pose)) {
            frame.available = true;
            if (frame.enabled) {
                out.frames[name] = pose;
                ++out.resolved;
            }
        } else if (frame.enabled)
            ++out.missing;
    }

    if (!usingTruth_ || !truth_.seen)
        return; // no simulator: nothing to compare the estimate against

    const auto fixed = [](double x, int decimals) {
        char text[32];
        std::snprintf(text, sizeof(text), "%.*f", decimals, x);
        return std::string(text);
    };

    // The readout: the estimate relative to the truth at the newest stamp both have.
    try {
        const auto estimate = buffer_->lookupTransform(map, scenario_->estimateBaseFrame, tf2::TimePointZero);
        const auto truth = buffer_->lookupTransform(map, scenario_->truthBaseFrame, tf2::TimePointZero);
        const auto stamp = std::min(rclcpp::Time(estimate.header.stamp), rclcpp::Time(truth.header.stamp));
        const auto at = matrixOf(buffer_->lookupTransform(map, scenario_->truthBaseFrame, stamp).transform);
        const auto relative =
            glm::inverse(at) * matrixOf(buffer_->lookupTransform(map, scenario_->estimateBaseFrame, stamp).transform);
        const float angle = glm::degrees(2 * std::acos(glm::clamp(std::abs(glm::quat_cast(relative).w), 0.f, 1.f)));
        out.difference = "ROS vs sim: " + fixed(glm::length(glm::vec3(relative[3])) * 100, 1) + " cm / " +
                         fixed(angle, 1) + " deg; forward " + fixed(relative[3].x * 100, 1) + " cm";
    } catch (const tf2::TransformException &) {
        out.difference = "Waiting for matching ROS / simulator poses";
    }
}

} // namespace nereus::ros_viewer::host

#include "ros_side.hpp"
#include "jpeg_decode.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstring>
#include <iostream>
#ifdef RP_VIEWER_UWRT
#include <riptide_msgs2/msg/led_command.hpp>
#endif

namespace robotics::ros_viewer::host {
namespace {
constexpr int kPreviewWidth = 480;

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
void colorize(float metres, float lo, float hi, std::uint8_t *out) {
    if (!std::isfinite(metres) || metres <= 0) {
        out[0] = out[1] = out[2] = 8;
        return;
    }
    const float t = std::clamp((metres - lo) / std::max(1e-3f, hi - lo), 0.f, 1.f);
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
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_, node_, false);
    // Data arrives via spin() on this thread and every lookup uses a zero timeout, so nothing ever waits.
    buffer_->setUsingDedicatedThread(true);
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

void RosSide::attach(const Scenario &scenario, const YAML::Node &config, StatusLights &lights,
                     ThrusterVisuals &thrusters, bool live) {
    scenario_ = &scenario;
    lights_ = &lights;
    thrusters_ = &thrusters;
    live_ = live;
    subscriptions_.clear();
    feeds.clear();
    // Camera feeds exist even without ROS (demo cards show PREVIEW ONLY).
    for (const auto &camera : scenario.cameras) {
        feeds.emplace_back();
        feeds.back().camera = &camera;
    }
    mpcTopic = scenario.absolute(lookup(config, {"topics", "mpc_path"}).as<std::string>("controller/mpc/predicted_path"));
    detectionTopic =
        scenario.absolute(lookup(config, {"topics", "detections"}).as<std::string>("yolo_orientation/visualization_marker_array"));
    if (!live)
        return;
    const auto topic = [&](const char *key, const char *fallback) {
        return scenario.absolute(lookup(config, {"topics", key}).as<std::string>(fallback));
    };
    if (!thrusters.rotors.empty())
        subscriptions_.push_back(node_->create_subscription<std_msgs::msg::Float32MultiArray>(
            scenario.absolute(thrusters.topic), 10, [this](const std_msgs::msg::Float32MultiArray &msg) {
                if (!thrusters_->receive(msg.data, now()))
                    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                         "Ignoring invalid realized thruster forces for propeller animation");
            }));
    if (lights.input == "riptide_msgs2/msg/LedCommand") {
#ifdef RP_VIEWER_UWRT
        subscriptions_.push_back(node_->create_subscription<riptide_msgs2::msg::LedCommand>(
            scenario.absolute(lights.topic), 10, [this](const riptide_msgs2::msg::LedCommand &msg) {
                using Command = riptide_msgs2::msg::LedCommand;
                LightMode mode;
                switch (msg.mode) {
                case Command::MODE_SOLID: mode = LightMode::Solid; break;
                case Command::MODE_SLOW_FLASH: mode = LightMode::SlowFlash; break;
                case Command::MODE_FAST_FLASH: mode = LightMode::FastFlash; break;
                case Command::MODE_BREATH: mode = LightMode::Breath; break;
                case Command::SINGLETON_FLASH: mode = LightMode::Flash; break;
                default: return;
                }
                if (msg.target > Command::TARGET_ALL)
                    return;
                lights_->command(glm::vec3(msg.red, msg.green, msg.blue) / 255.f, mode, msg.target, now());
            }));
#else
        std::cerr << "robotics-pool-viewer: built without riptide_msgs2; LED commands are not shown\n";
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
    subscriptions_.push_back(node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        topic("claw_joints", "simulator/claw_joints"), 10, [this](const std_msgs::msg::Float64MultiArray &msg) {
            if (msg.data.size() == 2 && std::isfinite(msg.data[0]) && std::isfinite(msg.data[1]))
                claw = {float(msg.data[0]), float(msg.data[1])};
        }));
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
    for (const auto &control : scenario.ui["mechanism_controls"])
        mechanismCommands_[scenario.absolute(control["topic"].as<std::string>())] =
            node_->create_publisher<std_msgs::msg::Bool>(scenario.absolute(control["topic"].as<std::string>()), 10);
    for (auto &feed : feeds)
        subscribeCamera(feed);
}

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
                        float metres;
                        std::memcpy(&metres, msg->data.data() + std::size_t(y * stride) * msg->step +
                                                 std::size_t(x * stride) * sizeof(float),
                                    sizeof(float));
                        colorize(metres, float(feed.camera->minRange), float(feed.camera->maxRange),
                                 feed.depth.data() + (std::size_t(y) * std::size_t(w) + std::size_t(x)) * 3);
                    }
                feed.depthWidth = w;
                feed.depthHeight = h;
                feed.depthDirty = true;
            });
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
            p.clock.observe(stamp, wallSeconds());
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
            out = matrixOf(buffer_->lookupTransform(map, frame, rclcpp::Time(int64_t(t * 1e9), RCL_ROS_TIME)).transform);
            return true;
        } catch (const tf2::TransformException &) {
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
    bool truth;
    switch (source_) {
    case PoseSource::Truth: truth = true; break;
    case PoseSource::Estimate: truth = false; break;
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
        status_ = source_ == PoseSource::Auto ? "WAITING FOR POSE" : truth ? "WAITING FOR PHYSICS" : "WAITING FOR ESTIMATE";
        return false;
    }
    fresh_ = src.fresh;
    status_ = truth ? (src.fresh ? "PHYSICS CONNECTED" : "POSE STALE") : (src.fresh ? "ROBOT (ESTIMATE)" : "ESTIMATE STALE");
    // One display clock (the active source's) serves the pose and every TF frame drawn this frame.
    const double wall = wallSeconds();
    truthTime_ = src.clock.at(wall, truthDelay_);
    otherTime_ = src.clock.at(wall, otherDelay_);
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
        const bool attached = m.header.frame_id == scenario_->estimateBaseFrame ||
                              m.header.frame_id == scenario_->truthBaseFrame;
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
                std::cerr << "robotics-pool-viewer: cannot resolve mesh resource " << m.mesh_resource << '\n';
        }
        records[{m.ns, m.id}] = std::move(record);
    }
}

void RosSide::receiveDetections(const visualization_msgs::msg::MarkerArray &msg) {
    using Marker = visualization_msgs::msg::Marker;
    const auto now = Clock::now();
    for (const auto &m : msg.markers) {
        if (m.action == Marker::DELETEALL)
            detectionMarkers_.clear();
        else if (m.action == Marker::DELETE)
            detectionMarkers_.erase({m.ns, m.id});
        else
            detectionMarkers_[{m.ns, m.id}] = DetectionEntry{m, now, {}};
    }
}

// Camera observations belong at their actual simulated acquisition pose: the truth base link at the
// marker stamp composed with the pack's fixed base-to-camera transform. Never falls back to estimated TF.
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
    for (auto &[key, entry] : detectionMarkers_) {
        const auto &m = entry.marker;
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
            if (!truthAcquisitionPose(*buffer_, scenario_->mapFrame, poseFrame(), stamp, *baseToCamera,
                                      acquisition))
                continue; // truth TF has not reached the stamp; retry next frame
            auto stamped = m;
            if (stamp.nanoseconds() == 0)
                stamped.header.stamp.nanosec = 1; // zero stamps use the latest truth for initial placement only
            placed = entry.placement.place(stamped, scenario_->mapFrame, *buffer_, "", &acquisition);
        } else {
            placed = entry.placement.place(m, scenario_->mapFrame, *buffer_);
        }
        if (placed)
            placedDetections.push_back({entry.placement.world(), m});
    }
}

// The controller steers its estimate, so the true vehicle follows the same motion relative to itself:
// place the prediction relative to the estimated base link, then re-root it at the truth base link.
void RosSide::captureMpc(bool wanted) {
    wanted = wanted && live_ && scenario_;
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
    if (!wanted)
        return;
    const double age = std::chrono::duration<double>(Clock::now() - mpcReceived_).count();
    if (mpcPending_) {
        // Solves are stamped at compute time; TF can trail that by a few ms. Retry at the stamp briefly
        // (measured from the first failed attempt, not the latest message), then settle for the latest
        // transforms.
        const bool settle = mpcFailing_ && std::chrono::duration<double>(Clock::now() - mpcFailingSince_).count() > .2;
        const rclcpp::Time stamp = settle ? rclcpp::Time(0, 0, RCL_ROS_TIME) : rclcpp::Time(mpcMessage_.header.stamp);
        try {
            const auto estimated =
                matrixOf(buffer_->lookupTransform(scenario_->estimateBaseFrame, mpcMessage_.header.frame_id, stamp).transform);
            const auto truth =
                matrixOf(buffer_->lookupTransform(scenario_->mapFrame, poseFrame(), stamp).transform);
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

void RosSide::captureTf(bool wanted, TfTree &tree, TfSnapshot &out) {
    out = {};
    if (!wanted || !live_ || !scenario_)
        return;
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
} // namespace robotics::ros_viewer::host

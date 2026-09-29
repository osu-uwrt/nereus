#include "app.hpp"
#include "frame_profiler.hpp"
#include "mapping_markers.hpp"
#include "overlay_draw.hpp"
#include "ros_side.hpp"
#include "scene_model.hpp"
#include "viewer_input.hpp"
#include "window.hpp"
#include "robotics/ros_viewer/panel_layout.hpp"
#include "robotics/ros_viewer/panels/composition.hpp"
#include "robotics/ros_viewer/panels/ros_providers.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <robotics/rendering/renderer.hpp>
#include <imgui_internal.h>
#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <thread>

namespace robotics::ros_viewer::host {
namespace {
namespace fs = std::filesystem;
namespace panels = robotics::ros_viewer::panels;

const ImVec4 cyan(.32f, .86f, .82f, 1), muted(.47f, .57f, .64f, 1), white(.87f, .92f, .95f, 1);
ImU32 color(ImVec4 c) {
    return ImGui::ColorConvertFloat4ToU32(c);
}
ImTextureID textureID(GLuint t) {
    return static_cast<ImTextureID>(t);
}
std::string fixed(double x, int decimals = 1) {
    std::ostringstream s;
    s << std::fixed << std::setprecision(decimals) << x;
    return s.str();
}
std::string trimSlashes(std::string s) {
    while (!s.empty() && s.front() == '/')
        s.erase(s.begin());
    return s;
}
fs::path contentDirectory() {
#ifdef RP_VIEWER_CONTENT
    return RP_VIEWER_CONTENT;
#else
    return fs::current_path();
#endif
}

// Adds the wall time of a scope to a profiler phase; with `sync` the GL queue is drained first so the
// cost of asynchronous draw calls lands in the phase that issued them (profiling only).
struct PhaseTimer {
    FrameProfiler &profiler;
    Phase phase;
    bool sync;
    Clock::time_point begin = Clock::now();
    ~PhaseTimer() {
        if (sync)
            glFinish();
        profiler.add(phase, std::chrono::duration<double>(Clock::now() - begin).count());
    }
};

// Removes the providers that only exist with the simulator (type sim.* / ros.simulation_rate) and every
// panel, tool or overlay bound to them, so a real-robot session shows no dead controls.
void dropSimulatorPanels(YAML::Node &document) {
    std::set<std::string> gone;
    if (auto providers = document["providers"]) {
        for (const auto &entry : providers) {
            const auto type = entry.second["type"].as<std::string>("");
            if (type.rfind("sim.", 0) == 0 || type == "ros.simulation_rate")
                gone.insert(entry.first.as<std::string>());
        }
        for (const auto &name : gone)
            providers.remove(name);
    }
    for (const char *group : {"panels", "toolbar", "overlays"}) {
        auto list = document[group];
        if (!list || !list.IsSequence())
            continue;
        YAML::Node kept(YAML::NodeType::Sequence);
        for (const auto &item : list)
            if (!gone.count(item["provider"].as<std::string>("")))
                kept.push_back(item);
        document[group] = kept;
    }
}

// Observer-only look: never touches the bridge's sensor renders.
struct Look {
    rendering::Appearance appearance;
    bool tag = true;
};
struct ObserverSettings {
    bool water = true, walls = true, floor = true, reflections = false, shadows = true;
    int lighting = 0; // 0 follows the scene, 1 indoor, 2 outdoor, 3 sterile
    float exposure = 1, brightness = 1, ambient = 1;
    void resetLighting() {
        shadows = true;
        lighting = 0;
        exposure = brightness = ambient = 1;
    }
    rendering::Appearance apply(rendering::Appearance scene) const {
        scene.reflections = reflections;
        scene.shadows = scene.shadows && shadows;
        scene.exposure *= exposure;
        scene.direct_light *= brightness;
        scene.ambient_light *= ambient;
        if (lighting)
            scene.outdoor = lighting == 2;
        if (lighting == 3) {
            // Ambient-only observer preset; independent of scene lighting.
            scene.outdoor = false;
            scene.shadows = false;
            scene.direct_light = 0;
            scene.ambient_light = .8f * ambient;
            scene.exposure = .8f * exposure;
            scene.caustics = 0;
            scene.glare = 0;
        }
        if (!water) {
            scene.surface = false;
            scene.caustics = 0;
            scene.water.absorption.setZero();
            scene.water.scattering = 0;
            scene.water.distance_scale = 0;
        }
        return scene;
    }
};

struct FocusPreset {
    std::string target = "landmark"; // course | vehicle | mechanism | mechanisms | landmark
    std::string mechanism, yawFrom = "landmark_facing";
    std::vector<std::string> mechanisms;
    glm::vec3 offset{0};
    float distance = 3.4f, distanceScale = 0, zOffset = 0, pitch = .28f, yaw = 0, yawOffset = 0;
    bool follow = true, labels = true;
};
void overlay(FocusPreset &p, const YAML::Node &n) {
    if (!n || !n.IsMap())
        return;
    p.target = n["target"].as<std::string>(p.target);
    p.mechanism = n["mechanism"].as<std::string>(p.mechanism);
    if (n["mechanisms"]) {
        p.mechanisms.clear();
        for (const auto &m : n["mechanisms"])
            p.mechanisms.push_back(m.as<std::string>());
    }
    if (n["offset"])
        p.offset = vec3(n["offset"]);
    p.distance = n["distance"].as<float>(p.distance);
    p.distanceScale = n["distance_scale"].as<float>(p.distanceScale);
    p.zOffset = n["z_offset"].as<float>(p.zOffset);
    p.pitch = n["pitch"].as<float>(p.pitch);
    p.yaw = n["yaw"].as<float>(p.yaw);
    p.yawOffset = n["yaw_offset"].as<float>(p.yawOffset);
    p.yawFrom = n["yaw_from"].as<std::string>(p.yawFrom);
    p.follow = n["follow"].as<bool>(p.follow);
    p.labels = n["labels"].as<bool>(p.labels);
}

struct CardTexture {
    GLuint rgb = 0, depth = 0;
    int rgbWidth = 0, rgbHeight = 0, depthWidth = 0, depthHeight = 0;
    bool flipped = false; // rendered (bottom-up) rather than decoded (top-down)
};
void upload(GLuint &texture, int &tw, int &th, const std::vector<std::uint8_t> &rgb, int w, int h) {
    if (!texture)
        glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, rgb.data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    tw = w;
    th = h;
}
} // namespace

class App {
  public:
    explicit App(const Options &, int argc, char **argv);
    ~App();
    int loop();

  private:
    // --- setup
    void loadScenario(const std::string &json);
    void buildPanels();
    FocusPreset presetFor(const std::string &name) const;
    bool focusable(const std::string &name) const;
    void focus(const std::string &name);
    glm::vec3 focusTarget(const std::string &name) const;
    void previewPose(const std::string &name);
    void applyInitialView();
    // --- per frame
    double clockSeconds() const;
    void step(double t);
    void updatePose();
    VisualState buildState();
    SensorView viewFor(float aspect, float dt, bool hovered, float viewportHeight, bool &dragging);
    void handleViewInput(float dt, bool hovered, float viewportHeight);
    void focusAtCursor(const SensorView &, const rendering::RenderedFrame &, ImVec2 origin, float w, float h);
    void demoTf();
    // --- UI
    void drawInterface(double time, float dt);
    void drawToolbar(float left, int &oldMode);
    void registerHostItems();
    void toolbarSceneSettings();
    void drawSceneSettingsPopup();
    void toolbarPoolViewer();
    void toolbarView();
    void toolbarFocus();
    void loadMappingMarkers();
    void drawPointCloudSettings();
    void toolbarFollow();
    void toolbarLabels();
    void toolbarTf();
    void toolbarDetections();
    void drawDetectionSettings(bool includeEnable);
    void toolbarMpcPath();
    void toolbarPreviewTask();
    void drawCameraCard(std::size_t index, float width, float maxHeight);
    void drawCourseMap(float width, float height, bool interactive);
    void drawMinimap(float width);
    void drawWaterControls();
    void pill(const std::string &text, ImVec4 tint);
    void sectionHeading(const char *text);
    std::string runTime() const;
    void saveCameraImages(const fs::path &screenshot);
    double cardPeriod(std::size_t camera) const;
    void renderLocalCards(double t, const rendering::Scene &mainScene);

    Options opt_;
    YAML::Node config_;
    fs::path configDir_;
    std::unique_ptr<Window> window_;
    std::unique_ptr<rendering::Renderer> renderer_;
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<RosSide> ros_;
    std::optional<Scenario> scenario_;
    std::unique_ptr<SceneModel> model_;
    StatusLights lights_;
    ThrusterVisuals thrusters_;
    std::string pendingScenario_;
    std::size_t loadedHash_ = 0;
    // panels
    panels::Registry registry_;
    panels::RosProviders panelRos_;
    std::unique_ptr<panels::Composition> composition_;
    std::shared_ptr<panels::Run> runTracking_;
    YAML::Node runScore_;
    // view state
    glm::mat4 body_{1};
    glm::vec3 target_{10, 4, -.8f}, freeEye_{-2, -5, 2};
    float yaw_ = -2.45f, pitch_ = .57f, distance_ = 19, freeYaw_ = .5f, freePitch_ = -.2f, freeRoll_ = 0;
    bool follow_ = false, labels_ = true, mouseCaptured_ = false;
    double lastMouseX_ = 0, lastMouseY_ = 0;
    int mode_ = 0; // 0 orbit, 1 free, 2+ sensor camera
    std::string focusName_ = "Vehicle";
    std::vector<std::string> focusNames_{"Course", "Vehicle"}, demoNames_;
    int selectedFocus_ = 0, selectedDemo_ = 0;
    bool orbitInteracting_ = false, orbitPanDrag_ = false;
    int orbitDragButton_ = -1;
    Clock::time_point orbitZoomUntil_{};
    SensorView viewportView_;
    rendering::RenderedFrame lastFrame_;
    bool haveFrame_ = false;
    GLuint readFbo_ = 0, drawFbo_ = 0;
    // settings
    Look look_;
    ObserverSettings observer_;
    bool openTfPopup_ = false, openObserverPopup_ = false, openDepth_ = false;
    bool openSceneSettings_ = false, showTf_ = false, tfNames_ = true, tfTreeOpen_ = false, detections_ = false,
         showMpc_ = false, largeMap_ = false, focusMap_ = false, demoMode_ = false;
    float tfAxisLength_ = .12f, mapZoom_ = 1, toolbarLeft_ = 0;
    int toolbarOldMode_ = 0;
    // Course source: 0 auto (pack layout with simulator truth, mapping markers otherwise), 1 pack, 2 mapping.
    std::vector<MappingMarker> mappingMarkers_;
    int courseMode_ = 0;
    bool mappingGhost_ = false;
    bool courseFromMapping() const;
    // Simulator only (truth is the pose source): the localization estimate at its display time, drawn as a
    // translucent robot ghost; the control gizmo and Follow are anchored together on it or on the truth robot.
    glm::mat4 estimateBody_{1};
    bool haveEstimate_ = false, robotGhost_ = false, anchorEstimate_ = false;
    glm::mat4 followBody() const {
        return anchorEstimate_ && haveEstimate_ ? estimateBody_ : body_;
    }
    ImVec2 mapPan_{0, 0};
    TfTree tfTree_;
    TfSnapshot tf_;
    std::deque<glm::vec3> trail_;
    std::string status_ = "WAITING FOR SCENARIO";
    std::vector<CardTexture> cards_;
    std::vector<double> cardDue_;        // next render time per card
    std::vector<char> cardVisible_;      // drawn on screen last frame (scrolled-out / hidden cards are skipped)
    std::size_t nextCardTurn_ = 0;       // round-robin start so cards share the frame budget evenly
    // layout
    float cameraSidebarWidth_ = 0, toolbarHeight_ = 80;
    bool cameraSidebarVisible_ = true, cameraSidebarResized_ = false;
    PanelEdge panelEdge_, cameraEdge_;
    Clock::time_point start_;
    double frameSeconds_ = 0;
    // profiling
    FrameProfiler profiler_;
    bool showProfile_ = false;
    Clock::time_point profileLogAt_{};
    double profileCachedAt_ = -1;
    std::string profileText_;
    bool profileSync() const {
        return opt_.profileSync;
    }
    std::vector<glm::mat4> lastLoaded_;
    int argc_;
    char **argv_;
};

App::App(const Options &options, int argc, char **argv) : opt_(options), argc_(argc), argv_(argv) {
    fs::path configPath = opt_.configPath.empty() ? contentDirectory() / "talos_uwrt_host.yaml" : fs::path(opt_.configPath);
    config_ = YAML::LoadFile(configPath.string());
    configDir_ = configPath.parent_path();
    demoMode_ = opt_.demo;
    for (const auto &name : opt_.open) {
        openSceneSettings_ |= name == "scene-settings";
        largeMap_ = focusMap_ = largeMap_ || name == "map";
        openTfPopup_ |= name == "tf";
        openObserverPopup_ |= name == "pool-viewer";
        openDepth_ |= name == "depth";
    }
    showTf_ = opt_.showTf;
    detections_ = opt_.detections.value_or(lookup(config_, {"detections", "enabled"}).as<bool>(true));
    showMpc_ = opt_.mpcPath;
    if (!demoMode_) {
        rclcpp::init(argc, argv);
        // Without a simulator (real robot) there is no /clock: default to wall time.
        const bool estimateOnly =
            (!opt_.poseSource.empty() ? opt_.poseSource : lookup(config_, {"pose", "source"}).as<std::string>("auto")) ==
            "estimate";
        const bool simTime = opt_.useSimTime.value_or(!estimateOnly);
        node_ = std::make_shared<rclcpp::Node>("robotics_pool_viewer",
                                               rclcpp::NodeOptions().parameter_overrides(
                                                   {rclcpp::Parameter("use_sim_time", simTime)}));
    }
    ros_ = std::make_unique<RosSide>(node_);
    ros_->configurePose(
        parsePoseSource(!opt_.poseSource.empty() ? opt_.poseSource
                                                 : lookup(config_, {"pose", "source"}).as<std::string>("auto")),
        opt_.truthDelay >= 0 ? opt_.truthDelay : lookup(config_, {"pose", "truth_delay_s"}).as<double>(.02),
        opt_.otherDelay >= 0 ? opt_.otherDelay : lookup(config_, {"pose", "other_delay_s"}).as<double>(.06));
    robotGhost_ = lookup(config_, {"pose", "robot_ghost"}).as<bool>(false);
    anchorEstimate_ = lookup(config_, {"pose", "anchor"}).as<std::string>("estimate") == "estimate";
    ros_->setDetectionMode(parseDetectionMode(
        !opt_.detectionPlacement.empty() ? opt_.detectionPlacement
                                         : lookup(config_, {"detections", "placement"}).as<std::string>("pose_source")));
    ros_->setHonorDeleteAll(!opt_.keepDetections && lookup(config_, {"detections", "honor_delete_all"}).as<bool>(true));
    const int width = lookup(config_, {"window", "width"}).as<int>(1480),
              height = lookup(config_, {"window", "height"}).as<int>(940);
    window_ = std::make_unique<Window>(width, height,
                                       lookup(config_, {"branding", "window_title"}).as<std::string>("Robotics Pool Viewer"),
                                       opt_.hidden, opt_.vsync);
    fs::path shaders = opt_.shaders;
#ifdef RP_RENDERING_SHADERS
    if (shaders.empty())
        shaders = RP_RENDERING_SHADERS;
#endif
    if (shaders.empty())
        shaders = fs::canonical("/proc/self/exe").parent_path().parent_path() / "share/robotics_platform/shaders";
    renderer_ = std::make_unique<rendering::Renderer>(shaders);
    glGenFramebuffers(1, &readFbo_);
    glGenFramebuffers(1, &drawFbo_);
    start_ = Clock::now();
    if (opt_.renderRate < 0 || opt_.renderRate > 240)
        throw std::runtime_error("render rate must be in [0,240] (0 = uncapped)");

    if (!opt_.scenarioFile.empty()) {
        std::ifstream file(opt_.scenarioFile);
        if (!file)
            throw std::runtime_error("cannot read scenario " + opt_.scenarioFile);
        std::stringstream text;
        text << file.rdbuf();
        loadScenario(text.str());
    } else if (demoMode_) {
        throw std::runtime_error("--demo needs --scenario FILE (there is no bridge to publish a scene)");
    } else {
        const std::string topic = !opt_.scenarioTopic.empty()
                                      ? opt_.scenarioTopic
                                      : lookup(config_, {"scenario_topic"}).as<std::string>("/talos/simulator/scenario");
        ros_->watchScenario(topic, [this](const std::string &json) { pendingScenario_ = json; });
    }
}

App::~App() {
    panelRos_.stop();
    composition_.reset();
    model_.reset();
    for (auto &card : cards_) {
        if (card.rgb)
            glDeleteTextures(1, &card.rgb);
        if (card.depth)
            glDeleteTextures(1, &card.depth);
    }
    if (readFbo_)
        glDeleteFramebuffers(1, &readFbo_);
    if (drawFbo_)
        glDeleteFramebuffers(1, &drawFbo_);
    renderer_.reset();
    ros_.reset();
    window_.reset();
    if (node_) {
        node_.reset();
        rclcpp::shutdown();
    }
}

FocusPreset App::presetFor(const std::string &name) const {
    FocusPreset preset;
    const auto presets = lookup(config_, {"focus_presets"});
    if (!presets)
        return preset;
    overlay(preset, presets["default"]);
    for (const auto &entry : presets) {
        const auto key = entry.first.as<std::string>();
        if (key == "default")
            continue;
        const bool glob = !key.empty() && key.back() == '*';
        if (key == name || (glob && name.rfind(key.substr(0, key.size() - 1), 0) == 0))
            overlay(preset, entry.second);
    }
    return preset;
}

bool App::focusable(const std::string &name) const {
    const auto p = presetFor(name);
    if (p.target == "landmark")
        return scenario_ && scenario_->landmarks.count(name);
    if (p.target == "mechanism")
        return scenario_ && scenario_->mechanism(p.mechanism);
    if (p.target == "mechanisms")
        return scenario_ && !p.mechanisms.empty() && scenario_->mechanism(p.mechanisms.front());
    return true;
}

glm::vec3 App::focusTarget(const std::string &name) const {
    const auto p = presetFor(name);
    const glm::mat4 body = followBody(); // truth robot, or the estimate when Follow is anchored on it
    if (!scenario_)
        return glm::vec3(body[3]);
    if (p.target == "mechanism")
        if (const auto *m = scenario_->mechanism(p.mechanism))
            return glm::vec3(body * m->frameInBase * glm::vec4(p.offset, 1));
    if (p.target == "mechanisms") {
        glm::vec3 sum(0);
        int count = 0;
        for (const auto &id : p.mechanisms)
            if (const auto *m = scenario_->mechanism(id))
                for (const auto &slot : m->slotsInBase) {
                    sum += glm::vec3(slot[3]);
                    ++count;
                }
        if (count)
            return glm::vec3(body * glm::vec4(sum / float(count), 1));
    }
    if (p.target == "landmark") {
        const auto it = scenario_->landmarks.find(name);
        if (it != scenario_->landmarks.end())
            return glm::vec3(it->second.world[3]);
    }
    return glm::vec3(body[3]);
}

void App::focus(const std::string &name) {
    if (!scenario_)
        return;
    const auto p = presetFor(name);
    const auto &s = *scenario_;
    if (p.target == "course") {
        const auto center = s.poolToWorld * glm::vec4(s.poolLength / 2, s.poolWidth / 2, 0, 1);
        target_ = glm::vec3(center.x, center.y, s.waterLevel + p.zOffset);
        distance_ = std::max(s.poolLength, s.poolWidth) * p.distanceScale;
    } else {
        target_ = focusTarget(name);
        distance_ = p.distance;
    }
    pitch_ = p.pitch;
    float base = p.yaw;
    if (p.yawFrom == "vehicle_heading")
        base = heading(body_);
    else if (p.yawFrom == "landmark_facing") {
        const auto it = s.landmarks.find(name);
        base = it == s.landmarks.end() ? yaw_ : std::atan2(it->second.world[0].y, it->second.world[0].x);
    }
    yaw_ = base + p.yawOffset;
    focusName_ = name;
    for (std::size_t i = 0; i < focusNames_.size(); ++i)
        if (focusNames_[i] == name)
            selectedFocus_ = int(i);
    mode_ = 0;
    follow_ = p.follow;
}

// Command-line view overrides survive the first-pose refocus.
void App::applyInitialView() {
    if (opt_.orbit.size() == 3) {
        yaw_ = opt_.orbit[0];
        pitch_ = opt_.orbit[1];
        distance_ = opt_.orbit[2];
    }
    if (!opt_.initialView.empty() && scenario_) {
        mode_ = opt_.initialView == "free" ? 1 : 0;
        for (std::size_t i = 0; i < scenario_->cameras.size(); ++i)
            if (scenario_->cameras[i].id == opt_.initialView)
                mode_ = int(i) + 2;
    }
}

void App::previewPose(const std::string &name) {
    if (!scenario_)
        return;
    for (std::size_t i = 0; i < demoNames_.size(); ++i)
        if (demoNames_[i] == name)
            selectedDemo_ = int(i);
    const auto it = scenario_->landmarks.find(name);
    if (it == scenario_->landmarks.end())
        throw std::runtime_error("Unknown demo task: " + name);
    const glm::vec3 at(it->second.world[3]), normal(it->second.world[0]);
    const float facing = std::atan2(-normal.y, -normal.x);
    glm::vec3 position = at + normal * 2.f;
    const auto preview = lookup(scenario_->ui, {"previews", name.c_str()});
    if (preview && preview["depth"])
        position.z = -preview["depth"].as<float>();
    if (preview && preview["camera"])
        if (const auto *camera = scenario_->camera(preview["camera"].as<std::string>())) {
            position.x = at.x;
            position.y = at.y;
            position -= glm::vec3(pose({}, {0, 0, facing}) * glm::vec4(glm::vec3(camera->mountInBase[3]), 0));
        }
    body_ = pose(position, {0, 0, facing});
}

void App::loadScenario(const std::string &json) {
    const std::size_t hash = std::hash<std::string>{}(json);
    if (hash == loadedHash_)
        return;
    auto parsed = parseScenario(json, config_, opt_.packDir);
    loadedHash_ = hash;
    // Replace dependents before the scenario they point to.
    model_.reset();
    lights_ = {};
    thrusters_ = {};
    scenario_.emplace(std::move(parsed));
    const auto resolve = [&](const char *key, const char *fallback) {
        return configDir_ / lookup(config_, {key}).as<std::string>(fallback);
    };
    const auto lightsPath = resolve("status_lights_config", "talos_uwrt_status_lights.yaml");
    // Robot-specific animation documents are optional: a scenario they do not fit runs without them.
    try {
        if (fs::exists(lightsPath))
            lights_ = StatusLights(YAML::LoadFile(lightsPath.string()));
    } catch (const std::exception &error) {
        lights_ = {};
        std::cerr << "robotics-pool-viewer: status lights disabled: " << error.what() << '\n';
    }
    const auto thrusterPath = resolve("thruster_visuals_config", "talos_uwrt_thruster_visuals.yaml");
    try {
        if (fs::exists(thrusterPath) && !scenario_->thrusterOrder.empty())
            thrusters_ = ThrusterVisuals(YAML::LoadFile(thrusterPath.string()), scenario_->thrusterOrder);
    } catch (const std::exception &error) {
        thrusters_ = {};
        std::cerr << "robotics-pool-viewer: thruster animation disabled: " << error.what() << '\n';
    }
    SceneModelOptions options;
    options.config = config_;
    options.configDirectory = configDir_;
    options.robotOnly = opt_.robotOnly;
    model_ = std::make_unique<SceneModel>(*scenario_, options, thrusters_, lights_);
    ros_->attach(*scenario_, config_, lights_, thrusters_, !demoMode_);
    loadMappingMarkers();
    look_.appearance = scenario_->appearance;
    look_.tag = lookup(config_, {"calibration_board", "visible"}).as<bool>(true);
    cards_.assign(scenario_->cameras.size(), {});
    cardDue_.assign(cards_.size(), 0.);
    cardVisible_.assign(cards_.size(), 1);
    for (auto &feed : ros_->feeds) {
        feed.wantDepth = openDepth_;
        feed.rosMode = !opt_.localCameras && !demoMode_; // local cards render from this viewer's scene
    }
    ros_->refreshCameras();
    // Focus / preview lists from the ui document, keeping only targets the scenario can resolve.
    focusNames_.clear();
    for (const auto &name : scenario_->ui["focus"])
        if (focusable(name.as<std::string>()))
            focusNames_.push_back(name.as<std::string>());
    if (focusNames_.empty())
        focusNames_ = {"Course", "Vehicle"};
    demoNames_.clear();
    for (const auto &name : scenario_->ui["demo_targets"])
        if (scenario_->landmarks.count(name.as<std::string>()))
            demoNames_.push_back(name.as<std::string>());
    window_->setTitle(lookup(config_, {"branding", "window_title"}).as<std::string>(scenario_->robotId + " | " + scenario_->poolId));
    if (demoMode_) {
        const auto p = lookup(config_, {"preview", "pose"});
        body_ = p ? pose(vec3(p), {0, 0, p[5].as<float>(0)}) : pose({3, -2, -.75f}, {0, 0, -.14f});
        const std::string task = !opt_.demoTask.empty() ? opt_.demoTask
                                                        : lookup(config_, {"preview", "task"}).as<std::string>("");
        if (!task.empty() && scenario_->landmarks.count(task))
            previewPose(task);
    }
    if (!composition_)
        buildPanels();
    std::string initial = !opt_.initialFocus.empty() ? opt_.initialFocus
                                                     : lookup(config_, {"initial_focus"}).as<std::string>("Vehicle");
    if (!focusable(initial))
        initial = "Vehicle";
    focus(initial);
    applyInitialView();
    status_ = demoMode_ ? "SCENE PREVIEW" : "WAITING FOR PHYSICS";
}

void App::buildPanels() {
    const std::string configured = opt_.panelsPath.empty()
                                       ? (configDir_ / lookup(config_, {"panels_config"}).as<std::string>("talos_uwrt_panels.yaml")).string()
                                       : opt_.panelsPath;
    const bool haveConfig = configured != "none" && fs::exists(configured);
    if (!haveConfig && configured != "none")
        std::cerr << "robotics-pool-viewer: panel composition " << configured << " not found; panels disabled\n";
    panels::registerPanels(registry_);
    registerHostItems();
    panelRos_.registerFactories(registry_);
    panels::Context context{trimSlashes(scenario_->ns), scenario_->mapFrame, demoMode_, ros_->useSimTime()};
    // Panel profile documents carry their scorecard schema under `ui` (run panel and sim.run provider).
    YAML::Node task(YAML::NodeType::Map);
    task["ui"] = YAML::Clone(scenario_->ui);
    context.documents.emplace("task", task);
    context.focus = [this](const std::string &name) { focus(name); };
    if (opt_.showScorecard)
        context.initialWindows.push_back("run");
    // Without panels the composition is empty (no sidebar) but still owns the default toolbar.
    auto document = haveConfig ? YAML::LoadFile(configured) : YAML::Load("{schema_version: 1, providers: {}}");
    if (ros_->poseSource() == PoseSource::Estimate)
        dropSimulatorPanels(document); // real robot: no simulator run / rate controls
    composition_ = std::make_unique<panels::Composition>(document, context, registry_);
    for (const auto &entry : composition_->providers())
        if (auto run = std::dynamic_pointer_cast<panels::Run>(entry.second)) {
            runTracking_ = run;
            break;
        }
    panelRos_.start();
}

double App::clockSeconds() const {
    return demoMode_ ? std::chrono::duration<double>(Clock::now() - start_).count() : ros_->now();
}

void App::updatePose() {
    if (demoMode_) {
        status_ = "SCENE PREVIEW";
        return;
    }
    bool first = false;
    const bool updated = ros_->updatePose(body_, first);
    // Estimate relative to the displayed truth robot, from one-time sampled offset (no display-clock lag).
    glm::mat4 truthFromEstimate;
    haveEstimate_ = ros_->truthFromEstimate(truthFromEstimate);
    if (haveEstimate_)
        estimateBody_ = glm::inverse(truthFromEstimate) * body_;
    if (updated) {
        if (first) {
            const auto p = presetFor(focusName_);
            if (p.target == "vehicle" || p.target == "mechanism" || p.target == "mechanisms")
                focus(focusName_);
            applyInitialView();
        }
        const glm::vec3 position(body_[3]);
        if (trail_.empty() || glm::distance(trail_.back(), position) > .06f) {
            trail_.push_back(position);
            if (trail_.size() > 1200)
                trail_.pop_front();
        }
    }
    status_ = ros_->status();
}

void App::demoTf() {
    tf_ = {};
    if (!scenario_)
        return;
    const auto &s = *scenario_;
    std::map<std::string, std::string> parents;
    parents[s.mapFrame] = "";
    parents[s.estimateBaseFrame] = s.mapFrame;
    for (const auto &frame : s.frames.names()) {
        if (frame == s.baseId)
            continue;
        parents[s.rosFrame(frame)] = s.estimateBaseFrame;
    }
    tfTree_.update(parents);
    const auto place = [&](const std::string &name, const glm::mat4 &pose) {
        auto &frame = tfTree_.frames.at(name);
        frame.available = true;
        if (frame.enabled) {
            tf_.frames[name] = pose;
            ++tf_.resolved;
        }
    };
    place(s.mapFrame, glm::mat4(1));
    place(s.estimateBaseFrame, body_);
    for (const auto &frame : s.frames.names())
        if (frame != s.baseId)
            place(s.rosFrame(frame), body_ * s.frames.relative(s.baseId, frame));
}

void App::step(double t) {
    if (!scenario_)
        return;
    updatePose();
    if (showTf_ || tfTreeOpen_) {
        if (demoMode_)
            demoTf();
        else
            ros_->captureTf(true, tfTree_, tf_);
    } else
        tf_ = {};
    ros_->captureDetections(detections_ && !demoMode_);
    if (!demoMode_)
        ros_->capturePointClouds();
    ros_->captureMpc(showMpc_ && !demoMode_);
    thrusters_.advance(clockSeconds());
    (void)t;
}

bool App::courseFromMapping() const {
    return !demoMode_ && !mappingMarkers_.empty() &&
           (courseMode_ == 2 || (courseMode_ == 0 && !ros_->truthActive()));
}

// Host config `mapping_markers:` {config: <package>/<path> or a file, meshes: optional local mesh folder,
// course: auto|pack|mapping, ghost: bool}. The file is the stack's RViz marker list (riptide_rviz markers.yaml).
void App::loadMappingMarkers() {
    mappingMarkers_.clear();
    const auto cfg = lookup(config_, {"mapping_markers"});
    if (!cfg || demoMode_)
        return;
    const auto share = [](const std::string &package) {
        return fs::path(ament_index_cpp::get_package_share_directory(package));
    };
    try {
        fs::path file = cfg["config"].as<std::string>();
        if (!file.is_absolute() && !fs::exists(file)) { // <package>/<path inside its share directory>
            const auto package = file.begin()->string();
            file = share(package) / file.lexically_relative(package);
        }
        mappingMarkers_ = host::loadMappingMarkers(file, share, cfg["meshes"].as<std::string>(""));
        const auto course = cfg["course"].as<std::string>("auto");
        courseMode_ = course == "pack" ? 1 : course == "mapping" ? 2 : 0;
        mappingGhost_ = cfg["ghost"].as<bool>(false);
    } catch (const std::exception &error) {
        std::cerr << "robotics-pool-viewer: mapping course disabled: " << error.what() << '\n';
        mappingMarkers_.clear();
    }
}

VisualState App::buildState() {
    VisualState state;
    state.body = body_;
    for (const auto &rotor : thrusters_.rotors)
        state.rotorSpin.push_back(rotor.transform());
    const double now = clockSeconds();
    for (const auto &light : lights_.lights)
        state.lightColor.push_back(light.state.color(now));
    state.claw = demoMode_ ? std::array<float, 2>{0.f, 0.f} : ros_->claw;
    state.showBoard = look_.tag;
    state.showWalls = observer_.walls;
    state.showFloor = observer_.floor;
    if (robotGhost_ && haveEstimate_)
        state.ghostBody = estimateBody_;
    const auto payloads = lookup(config_, {"payloads", "loaded_namespaces"});
    if (demoMode_) {
        for (const auto &entry : payloads)
            for (const auto &mount : model_->payloadMounts(entry.second.as<std::string>()))
                state.loadedPayloads.push_back(body_ * mount);
        return state;
    }
    // Mapping course (RViz markers at the mapping frames): the course itself on a real robot, or a translucent
    // ghost of the mapping estimate over the simulator's course.
    const bool mappingCourse = courseFromMapping();
    state.showCourse = !mappingCourse;
    if (mappingCourse || (mappingGhost_ && ros_->truthActive()))
        for (const auto &marker : mappingMarkers_) {
            glm::mat4 frame;
            if (!ros_->latestInFixed(marker.frame, frame))
                continue; // RViz does not draw a marker whose frame is unavailable
            MarkerDraw draw;
            draw.mesh = marker.path;
            draw.world = frame * marker.local;
            draw.ghost = !mappingCourse;
            draw.observerOnly = true;
            state.markers.push_back(std::move(draw));
        }
    for (const auto &[key, record] : ros_->props) {
        if (mappingCourse) // simulator props duplicate the mapped table items
            break;
        if (record.mesh.empty())
            continue;
        MarkerDraw draw;
        draw.mesh = record.mesh;
        draw.world = record.attached ? body_ * record.pose : record.pose;
        draw.scale = {float(record.marker.scale.x), float(record.marker.scale.y), float(record.marker.scale.z)};
        state.markers.push_back(std::move(draw));
    }
    for (const auto &[key, record] : ros_->projectiles) {
        const auto ns = lookup(payloads, {key.first.c_str()});
        if (ns) {
            // Loaded rounds follow this frame's robot pose like the launcher, not the sampled marker pose.
            const auto mounts = model_->payloadMounts(ns.as<std::string>());
            if (key.second >= 0 && std::size_t(key.second) < mounts.size()) {
                state.loadedPayloads.push_back(body_ * mounts[std::size_t(key.second)]);
                continue;
            }
        }
        if (record.mesh.empty())
            continue;
        MarkerDraw draw;
        draw.mesh = record.mesh;
        draw.world = record.attached ? body_ * record.pose : record.pose;
        draw.scale = {float(record.marker.scale.x), float(record.marker.scale.y), float(record.marker.scale.z)};
        state.markers.push_back(std::move(draw));
    }
    const auto radiance = lookup(config_, {"magnet_lights", "led_radiance"}).as<float>(60.f);
    for (const auto &[key, record] : ros_->magnetLights) {
        MarkerDraw draw;
        const bool green = record.marker.color.g > record.marker.color.r;
        // A pack LED visual follows the indicator itself (marker namespace = indicator region); the emissive
        // box is only the fallback for scenarios without one.
        state.indicatorLatched[key.first] = green;
        bool packLed = false;
        for (const auto &led : model_->pack().indicatorVisuals())
            packLed = packLed || led.region == key.first;
        if (packLed)
            continue;
        draw.emissive = true;
        draw.radiance = radiance;
        draw.tint = green ? glm::vec4(.002f, 1.f, .004f, 1.f) : glm::vec4(1.f, .001f, .002f, 1.f);
        draw.world = record.attached ? body_ * record.pose : record.pose;
        draw.scale = {float(record.marker.scale.x), float(record.marker.scale.y), float(record.marker.scale.z)};
        state.markers.push_back(std::move(draw));
    }
    return state;
}

// --------------------------------------------------------------------------------------------- input

void App::handleViewInput(float dt, bool hovered, float viewportHeight) {
    auto &io = ImGui::GetIO();
    GLFWwindow *glfw = window_->handle();
    if (mouseCaptured_ &&
        (mode_ != 1 || ImGui::IsKeyPressed(ImGuiKey_Escape) || !glfwGetWindowAttrib(glfw, GLFW_FOCUSED))) {
        mouseCaptured_ = false;
        glfwSetInputMode(glfw, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    }
    if (mode_ == 1 && hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !mouseCaptured_) {
        mouseCaptured_ = true;
        glfwSetInputMode(glfw, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        if (glfwRawMouseMotionSupported())
            glfwSetInputMode(glfw, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);
        glfwGetCursorPos(glfw, &lastMouseX_, &lastMouseY_);
    }
    if (mode_ == 1 && mouseCaptured_) {
        double x, y;
        glfwGetCursorPos(glfw, &x, &y);
        freeYaw_ -= float(x - lastMouseX_) * .003f;
        if (y != lastMouseY_)
            freePitch_ = glm::clamp(freePitch_ - float(y - lastMouseY_) * .003f, -1.55f, 1.55f);
        lastMouseX_ = x;
        lastMouseY_ = y;
        glm::vec3 forward(std::cos(freeYaw_), std::sin(freeYaw_), 0), left(-forward.y, forward.x, 0), motion(0);
        if (glfwGetKey(glfw, GLFW_KEY_W) == GLFW_PRESS)
            motion += forward;
        if (glfwGetKey(glfw, GLFW_KEY_S) == GLFW_PRESS)
            motion -= forward;
        if (glfwGetKey(glfw, GLFW_KEY_A) == GLFW_PRESS)
            motion += left;
        if (glfwGetKey(glfw, GLFW_KEY_D) == GLFW_PRESS)
            motion -= left;
        if (glfwGetKey(glfw, GLFW_KEY_SPACE) == GLFW_PRESS)
            motion.z += 1;
        if (glfwGetKey(glfw, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
            motion.z -= 1;
        if (glm::length(motion) > 0)
            freeEye_ += glm::normalize(motion) * dt * (io.KeyCtrl ? 8.f : 2.5f);
    }
    orbitInteracting_ = false;
    if (mode_ != 0 || !glfwGetWindowAttrib(glfw, GLFW_FOCUSED) ||
        (orbitDragButton_ >= 0 && !ImGui::IsMouseDown(orbitDragButton_)))
        orbitDragButton_ = -1;
    if (mode_ == 0) {
        if (hovered && orbitDragButton_ < 0)
            for (int button : {ImGuiMouseButton_Left, ImGuiMouseButton_Right, ImGuiMouseButton_Middle})
                if (ImGui::IsMouseClicked(button)) {
                    orbitDragButton_ = button;
                    orbitPanDrag_ = button != ImGuiMouseButton_Left || io.KeyShift;
                }
        orbitInteracting_ = orbitDragButton_ >= 0;
        if (orbitDragButton_ >= 0 && !ImGui::IsMouseClicked(orbitDragButton_)) {
            if (orbitPanDrag_ && (io.MouseDelta.x != 0 || io.MouseDelta.y != 0)) {
                follow_ = false; // panning detaches the camera from its target
                target_ += orbitPan(viewportView_.view, viewportView_.projection, distance_, viewportHeight,
                                    {io.MouseDelta.x, io.MouseDelta.y});
                ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
            } else if (!orbitPanDrag_) {
                yaw_ -= io.MouseDelta.x * .005f;
                pitch_ = glm::clamp(pitch_ + io.MouseDelta.y * .005f, -1.55f, 1.55f);
            }
        }
        if (hovered && io.MouseWheel != 0) {
            distance_ = glm::clamp(distance_ * std::exp(-io.MouseWheel * .1f), .15f, 75.f);
            orbitZoomUntil_ = Clock::now() + std::chrono::milliseconds(140);
        }
    }
    orbitInteracting_ = mode_ == 0 && (orbitInteracting_ || Clock::now() < orbitZoomUntil_);
}

SensorView App::viewFor(float aspect, float dt, bool hovered, float viewportHeight, bool &) {
    handleViewInput(dt, hovered, viewportHeight);
    if (follow_)
        target_ = focusTarget(focusName_);
    if (mode_ >= 2 && scenario_ && std::size_t(mode_ - 2) < scenario_->cameras.size()) {
        const auto &camera = scenario_->cameras[std::size_t(mode_ - 2)];
        (void)aspect; // the sensor keeps its own aspect; drawInterface resizes the intrinsics to the image
        return sensorView(body_ * camera.opticalInBase, camera.k);
    }
    glm::vec3 eye, at, up(0, 0, 1);
    if (mode_ == 1) {
        eye = freeEye_;
        at = eye + glm::vec3(std::cos(freeYaw_) * std::cos(freePitch_), std::sin(freeYaw_) * std::cos(freePitch_),
                             std::sin(freePitch_));
        const glm::vec3 right(std::sin(freeYaw_), -std::cos(freeYaw_), 0);
        up = std::cos(freeRoll_) * glm::cross(right, at - eye) + std::sin(freeRoll_) * right;
    } else {
        eye = target_ + distance_ * glm::vec3(std::cos(yaw_) * std::cos(pitch_), std::sin(yaw_) * std::cos(pitch_),
                                              std::sin(pitch_));
        at = target_;
    }
    return {eye, glm::lookAt(eye, at, up), glm::perspective(glm::radians(53.f), aspect, .05f, 100.f)};
}

void App::focusAtCursor(const SensorView &view, const rendering::RenderedFrame &frame, ImVec2 origin, float width,
                        float height) {
    if (mode_ != 0)
        return;
    const auto mouse = ImGui::GetIO().MousePos;
    const glm::vec2 cursor(mouse.x - origin.x, mouse.y - origin.y), size(width, height);
    OverlayFocusPicker picker(view.projection * view.view, size, cursor);
    if (showTf_)
        for (const auto &[name, frameMatrix] : tf_.frames)
            for (int axis = 0; axis < 3; ++axis)
                picker.segment(glm::vec3(frameMatrix[3]), glm::vec3(frameMatrix[3] + frameMatrix[axis] * tfAxisLength_),
                               glm::vec3(frameMatrix[3]));
    if (detections_)
        for (const auto &placed : ros_->placedDetections) {
            const auto &m = placed.marker;
            if (m.color.a <= 0)
                continue;
            using visualization_msgs::msg::Marker;
            if (m.type == Marker::CUBE)
                picker.quad(placed.pose, {float(m.scale.x) * .5f, float(m.scale.y) * .5f});
            else if (m.type == Marker::ARROW)
                picker.segment(glm::vec3(placed.pose[3]), glm::vec3(placed.pose * glm::vec4(float(m.scale.x), 0, 0, 1)),
                               glm::vec3(placed.pose[3]));
        }
    if (showMpc_)
        for (std::size_t i = 1; i < ros_->mpcPath.size(); ++i)
            picker.segment(glm::vec3(ros_->mpcPath[i - 1][3]), glm::vec3(ros_->mpcPath[i][3]),
                           glm::vec3(ros_->mpcPath[i - 1][3]));
    glm::vec3 point;
    if (!picker.result(point)) {
        const auto uv = cursor / size;
        if (uv.x < 0 || uv.x >= 1 || uv.y < 0 || uv.y >= 1 || !frame.depth_texture)
            return;
        GLint previous = 0;
        glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &previous);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, readFbo_);
        glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, frame.depth_texture, 0);
        float depth = 1;
        if (glCheckFramebufferStatus(GL_READ_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE)
            glReadPixels(int(uv.x * float(frame.width)), frame.height - 1 - int(uv.y * float(frame.height)), 1, 1,
                         GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
        glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, 0, 0);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, GLuint(previous));
        if (!depthPoint(view.projection * view.view, uv, depth, point) &&
            !focusPlanePoint(view.projection * view.view, view.eye, target_, uv, point))
            return;
    }
    const auto offset = view.eye - point;
    const float nextDistance = glm::length(offset);
    if (!std::isfinite(nextDistance) || nextDistance < 1e-4f)
        return;
    target_ = point;
    distance_ = nextDistance;
    follow_ = false;
    yaw_ = std::atan2(offset.y, offset.x);
    pitch_ = glm::clamp(std::asin(offset.z / distance_), -1.55f, 1.55f);
}

// ------------------------------------------------------------------------------------------- widgets

void App::pill(const std::string &text, ImVec4 tint) {
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(tint.x * .15f, tint.y * .15f, tint.z * .15f, 1));
    ImGui::PushStyleColor(ImGuiCol_Text, tint);
    ImGui::Button(text.c_str());
    ImGui::PopStyleColor(2);
}
void App::sectionHeading(const char *text) {
    ImGui::PushFont(window_->small);
    ImGui::TextColored(muted, "%s", text);
    ImGui::PopFont();
}
std::string App::runTime() const {
    const double seconds = runScore_ && runScore_["elapsed"] ? runScore_["elapsed"].as<double>() : 0.;
    char value[64];
    std::snprintf(value, sizeof(value), "%02d:%04.1f", int(seconds) / 60, std::fmod(seconds, 60.));
    return value;
}

void App::drawCameraCard(std::size_t index, float width, float maxHeight) {
    auto &feed = ros_->feeds[index];
    auto &tex = cards_[index];
    const auto &camera = *feed.camera;
    if (feed.rgbDirty) {
        upload(tex.rgb, tex.rgbWidth, tex.rgbHeight, feed.rgb, feed.rgbWidth, feed.rgbHeight);
        tex.flipped = false;
        feed.rgbDirty = false;
    }
    if (feed.depthDirty) {
        upload(tex.depth, tex.depthWidth, tex.depthHeight, feed.depth, feed.depthWidth, feed.depthHeight);
        feed.depthDirty = false;
    }
    ImGui::PushID(camera.id.c_str());
    ImGui::BeginChild("camera", {width, 0}, ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY);
    ImGui::AlignTextToFramePadding();
    ImGui::TextColored(cyan, "%s", camera.title.c_str());
    ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - 62);
    if (ImGui::Button(feed.wantDepth ? "DEPTH" : "RGB", {62, 0})) {
        feed.wantDepth = !feed.wantDepth;
        ros_->refreshCameras();
    }
    // Local rendering needs the simulator's truth pose; without it the card is the ROS image topic.
    const bool canLocal = demoMode_ || ros_->truthActive();
    if (!demoMode_ && !canLocal) {
        ImGui::PushFont(window_->small);
        ImGui::TextDisabled("ROS image topic (no simulator truth pose)");
        ImGui::PopFont();
    }
    ImGui::PushFont(window_->small);
    ImGui::TextColored(muted, "%s  /  %d x %d", camera.model.c_str(), camera.k.width, camera.k.height);
    const bool depthShown = feed.wantDepth && tex.depth;
    const bool local = demoMode_ || (canLocal && !feed.rosMode);
    ImGui::TextDisabled("Preview: %d x %d", depthShown ? tex.depthWidth : tex.rgbWidth,
                        depthShown ? tex.depthHeight : tex.rgbHeight);
    ImGui::PopFont();
    const float available = ImGui::GetContentRegionAvail().x;
    const float w = std::min(available, maxHeight * float(camera.k.width) / float(camera.k.height));
    const float h = w * float(camera.k.height) / float(camera.k.width);
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (available - w) / 2);
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    const GLuint texture = depthShown ? tex.depth : tex.rgb;
    if (texture) {
        const bool flip = !depthShown && tex.flipped;
        ImGui::Image(textureID(texture), {w, h}, flip ? ImVec2(0, 1) : ImVec2(0, 0), flip ? ImVec2(1, 0) : ImVec2(1, 1));
    } else {
        ImGui::Dummy({w, h});
        auto *draw = ImGui::GetWindowDrawList();
        draw->AddRectFilled(pos, {pos.x + w, pos.y + h}, IM_COL32(4, 13, 19, 255));
    }
    cardVisible_[index] = ImGui::IsItemVisible();
    const bool ready = demoMode_ || !local || ros_->poseFresh();
    if (!ready || !texture) {
        auto *draw = ImGui::GetWindowDrawList();
        if (texture)
            draw->AddRectFilled(pos, {pos.x + w, pos.y + h}, IM_COL32(4, 13, 19, 175));
        draw->AddText({pos.x + 18, pos.y + 18}, color(muted),
                      !ready ? "Awaiting vehicle pose" : local && !depthShown ? "Rendering local view"
                                                                              : "Awaiting camera image");
    }
    ImGui::PushFont(window_->small);
    const bool connected = !demoMode_ && feed.connected();
    const bool depthFromRos = feed.wantDepth && !demoMode_;
    ImGui::TextColored(
        local && !depthFromRos ? cyan : connected ? cyan : muted, "%s",
        demoMode_ ? "PREVIEW ONLY" : local && !depthFromRos ? "LOCAL VIEW" : connected ? "CONNECTED" : "NO SENSOR OUTPUT");
    ImGui::SameLine();
    const bool sourceToggle = canLocal && !demoMode_;
    if (sourceToggle) {
        // Simulator: the source label toggles the card between this viewer's truth-pose render and
        // the images the bridge publishes to the robot stack.
        const bool stack = feed.rosMode;
        ImGui::PushStyleColor(ImGuiCol_Button, stack ? ImVec4(.12f, .48f, .46f, 1) : ImVec4(.1f, .16f, .2f, 1));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, stack ? ImVec4(.16f, .6f, .56f, 1) : ImVec4(.16f, .25f, .31f, 1));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, stack ? ImVec4(.1f, .4f, .38f, 1) : ImVec4(.08f, .13f, .17f, 1));
        if (ImGui::SmallButton(stack ? "ROS (stack)###source" : "truth pose###source")) {
            feed.rosMode = !stack;
            ros_->refreshCameras();
            cardDue_[index] = 0; // render the local view immediately when switching back
        }
        ImGui::PopStyleColor(3);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Click to switch the card's source.\n"
                              "truth pose: rendered by this viewer at the truth pose (no sensor noise, no bridge latency).\n"
                              "ROS (stack): the images the bridge publishes, as the robot stack receives them.");
        ImGui::SameLine();
    }
    if (local && !depthFromRos)
        ImGui::TextDisabled(sourceToggle ? "|  RGB" : "  truth pose  |  RGB");
    else
        ImGui::TextDisabled("  %.1f Hz  |  %s", connected ? feed.hz : 0., feed.wantDepth ? "METRES" : "RECTIFIED RGB");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(local && !depthFromRos
                              ? "Rendered by this viewer from the truth pose (no sensor noise, no bridge latency)."
                              : "Camera images are rendered and published by the simulator bridge.\n"
                                "Depth is rendered geometry with the pack's sensor noise model.\n"
                                "Topic: %s",
                          feed.wantDepth ? camera.depthTopic.c_str() : camera.rgbTopic.c_str());
    ImGui::PopFont();
    ImGui::EndChild();
    ImGui::PopID();
}

void App::drawWaterControls() {
    auto edited = look_.appearance.water;
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(10, 3));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8, 3));
    ImGui::SetNextItemWidth(240);
    ImGui::ColorEdit3("Water tint", edited.tint.data());
    ImGui::SameLine();
    if (ImGui::SmallButton("Clear blue")) {
        edited = {};
        edited.tint = {.015f, .16f, .24f};
        edited.absorption = {.075f, .02f, .012f};
        edited.scattering = .045f;
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Pool"))
        edited = {};
    ImGui::SameLine();
    if (ImGui::SmallButton("Green / murky")) {
        edited = {};
        edited.tint = {.07f, .22f, .10f};
        edited.absorption = {.20f, .06f, .12f};
        edited.scattering = .25f;
    }
    ImGui::Columns(4, "water controls", false);
    auto slider = [&](const char *label, float &v, float lo, float hi, const char *format) {
        ImGui::TextUnformatted(label);
        ImGui::SetNextItemWidth(-10);
        ImGui::SliderFloat((std::string("##water ") + label).c_str(), &v, lo, hi, format);
        ImGui::NextColumn();
    };
    slider("Haze / scattering", edited.scattering, 0, 1, "%.3f /m");
    slider("Distance strength", edited.distance_scale, 0, 5, "%.2f x");
    slider("Distance exponent", edited.distance_power, .25f, 3, "%.2f");
    slider("Clear distance", edited.clear_distance, 0, 10, "%.2f m");
    slider("Red absorption", edited.absorption[0], 0, 1, "%.3f /m");
    slider("Green absorption", edited.absorption[1], 0, 1, "%.3f /m");
    slider("Blue absorption", edited.absorption[2], 0, 1, "%.3f /m");
    ImGui::TextDisabled("More red absorption\nmakes distant objects\nlook bluer.");
    ImGui::Columns(1);
    ImGui::PushStyleColor(ImGuiCol_Text, muted);
    ImGui::TextWrapped("Tint and haze accumulate along the underwater sightline. These settings change only this "
                       "viewer's rendering; the robot cameras are rendered by the bridge from the pool pack. "
                       "Exponent 1 / clear distance 0: exponential attenuation.");
    ImGui::PopStyleColor();
    look_.appearance.water = edited;
    ImGui::PopStyleVar(2);
}

void App::drawCourseMap(float width, float height, bool interactive) {
    const auto &s = *scenario_;
    const ImVec2 a = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("course canvas", {width, height});
    const bool hovered = ImGui::IsItemHovered();
    auto &io = ImGui::GetIO();
    if (interactive && hovered) {
        mapZoom_ = glm::clamp(mapZoom_ * std::exp(io.MouseWheel * .15f), 1.f, 8.f);
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            mapPan_.x += io.MouseDelta.x;
            mapPan_.y += io.MouseDelta.y;
        }
    }
    const float length = s.poolLength, poolWidth = s.poolWidth;
    const float scale = std::min((width - 36) / length, (height - 36) / poolWidth) * (interactive ? mapZoom_ : 1.f);
    const ImVec2 center(a.x + width / 2 + (interactive ? mapPan_.x : 0), a.y + height / 2 + (interactive ? mapPan_.y : 0));
    auto poolXY = [&](glm::vec2 p) {
        return ImVec2(center.x + (p.x - length / 2) * scale, center.y - (p.y - poolWidth / 2) * scale);
    };
    auto xy = [&](glm::vec3 p) { return poolXY(glm::vec2(s.worldToPool * glm::vec4(p, 1))); };
    auto *d = ImGui::GetWindowDrawList();
    d->AddRectFilled(a, {a.x + width, a.y + height}, IM_COL32(9, 24, 32, 255), 5);
    d->PushClipRect(a, {a.x + width, a.y + height}, true);
    d->AddRectFilled(poolXY({0, poolWidth}), poolXY({length, 0}), IM_COL32(13, 40, 50, 255));
    for (int i = 0; i <= length; i += 5)
        d->AddLine(poolXY({float(i), 0}), poolXY({float(i), poolWidth}), IM_COL32(35, 64, 74, 255));
    for (int i = 0; i <= poolWidth; i += 5)
        d->AddLine(poolXY({0, float(i)}), poolXY({length, float(i)}), IM_COL32(35, 64, 74, 255));
    d->AddRect(poolXY({0, poolWidth}), poolXY({length, 0}), IM_COL32(94, 154, 166, 255), 0, 0, 2);
    for (std::size_t i = 1; i < trail_.size(); ++i)
        d->AddLine(xy(trail_[i - 1]), xy(trail_[i]), IM_COL32(53, 134, 143, 200), 1.5f);
    int index = 0;
    for (const auto &entry : s.ui["map_landmarks"]) {
        const std::string key = entry.IsScalar() ? entry.as<std::string>() : entry["name"].as<std::string>();
        const std::string label = entry.IsScalar() ? key : entry["label"].as<std::string>(key);
        const bool hiddenInMinimap = !entry.IsScalar() && entry["hidden_in_minimap"].as<bool>(false);
        const bool relabelMinimap = !entry.IsScalar() && entry["minimap_label"];
        const auto found = s.landmarks.find(key);
        if (found == s.landmarks.end())
            continue;
        const auto p = xy(glm::vec3(found->second.world[3]));
        const float font = interactive ? 16 : 12;
        const ImVec2 textAt(p.x + 7, p.y + (index++ % 2 ? -19 : 4));
        d->AddCircleFilled(p, interactive ? 5 : 4, color(cyan));
        if (interactive || !hiddenInMinimap)
            d->AddText(interactive ? window_->normal : window_->small, font, textAt, color(white),
                       (!interactive && relabelMinimap ? entry["minimap_label"].as<std::string>() : key).c_str());
        (void)label;
        if (interactive && hovered && ImGui::IsMouseReleased(ImGuiMouseButton_Left) &&
            ImGui::GetMouseDragDelta().x * ImGui::GetMouseDragDelta().x +
                    ImGui::GetMouseDragDelta().y * ImGui::GetMouseDragDelta().y <
                9 &&
            std::hypot(io.MousePos.x - p.x, io.MousePos.y - p.y) < 12)
            focus(key);
    }
    const auto p = xy(glm::vec3(body_[3]));
    const auto tip = xy(glm::vec3(body_[3]) + glm::vec3(body_[0]) * 1.3f);
    d->AddCircleFilled(p, 6, IM_COL32(255, 208, 96, 255));
    d->AddLine(p, tip, IM_COL32(255, 208, 96, 255), 3);
    d->AddText(window_->small, 12, {p.x + 8, p.y - 15}, IM_COL32(255, 208, 96, 255), s.robotId.c_str());
    d->PopClipRect();
}

void App::drawMinimap(float width) {
    ImGui::BeginChild("map", {width, 0}, ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY);
    ImGui::AlignTextToFramePadding();
    sectionHeading("COURSE MAP");
    ImGui::SameLine();
    if (ImGui::Button("Expand"))
        largeMap_ = focusMap_ = true;
    const auto available = ImGui::GetContentRegionAvail();
    // Scale both canvas dimensions with the sidebar so height cannot cap the map's growth.
    const float aspect = scenario_->poolWidth / scenario_->poolLength;
    drawCourseMap(available.x, std::max(120.f, 36.f + (available.x - 36.f) * aspect), false);
    ImGui::EndChild();
}

void App::drawSceneSettingsPopup() {
    const float width = ImGui::GetContentRegionAvail().x;
    for (const auto &control : scenario_->ui["mechanism_controls"]) {
        ImGui::BeginDisabled(demoMode_ || !ros_->truthActive()); // simulator-only commands
        if (ImGui::Button(control["label"].as<std::string>().c_str()))
            ros_->publishMechanism(control["topic"].as<std::string>(), control["value"].as<bool>(true));
        ImGui::EndDisabled();
    }
    if (ImGui::BeginTabBar("Environment tabs")) {
        if (ImGui::BeginTabItem("Lighting")) {
            sectionHeading("UNDERWATER OPTICS");
            auto &a = look_.appearance;
            ImGui::Checkbox("Calibration board", &look_.tag);
            ImGui::SetNextItemWidth(width * .17f);
            ImGui::SliderFloat("Caustics", &a.caustics, 0, 1, "%.2f");
            ImGui::SameLine();
            ImGui::Checkbox("Surface", &a.surface);
            ImGui::SameLine();
            ImGui::Checkbox("Shadows", &a.shadows);
            ImGui::Separator();
            int profile = a.outdoor ? 1 : 0;
            ImGui::SetNextItemWidth(120);
            if (ImGui::Combo("Lighting", &profile, "Indoor\0Outdoor\0")) {
                a.outdoor = profile == 1;
                a.direct_light = a.outdoor ? 1.4f : 1.f;
                a.ambient_light = a.outdoor ? .6f : .9f;
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(130);
            ImGui::SliderFloat("Brightness", &a.direct_light, 0, 4, "%.2f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(125);
            ImGui::SliderFloat("Ambient", &a.ambient_light, 0, 2, "%.2f");
            if (a.outdoor) {
                ImGui::SetNextItemWidth(160);
                ImGui::SliderFloat("Sun azimuth", &a.sun_azimuth, 0, 360, "%.0f deg");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(140);
                ImGui::SliderFloat("Elevation", &a.sun_elevation, 5, 89, "%.0f deg");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(120);
                ImGui::SliderFloat("Glare", &a.glare, 0, 2, "%.2f");
            } else
                ImGui::TextDisabled("Diffuse indoor lighting. Switch to Outdoor to adjust sun and glare.");
            ImGui::PushStyleColor(ImGuiCol_Text, muted);
            ImGui::TextWrapped("Observer settings only: the bridge renders the robot cameras from the pool pack.");
            ImGui::PopStyleColor();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Water appearance")) {
            drawWaterControls();
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
}

void App::toolbarSceneSettings() {
    if (ImGui::Button("Scene settings") || openSceneSettings_) {
        openSceneSettings_ = false;
        ImGui::OpenPopup("scene_settings");
    }
    // Anchored under the button and kept inside the window. A plain popup (no child window) so colour-picker
    // sub-popups stack on it instead of dismissing it, and a fixed size so the tabs do not resize as they change.
    const auto *viewport = ImGui::GetMainViewport();
    const auto button = ImGui::GetItemRectMin();
    const float y = ImGui::GetItemRectMax().y + ImGui::GetStyle().ItemSpacing.y;
    const float width = std::min(780.f, viewport->WorkSize.x - 16.f);
    const float height = std::clamp(viewport->WorkPos.y + viewport->WorkSize.y - y - 8.f, 120.f, 400.f);
    ImGui::SetNextWindowPos({std::clamp(button.x, viewport->WorkPos.x + 8.f, viewport->WorkPos.x + viewport->WorkSize.x - width - 8.f), y});
    ImGui::SetNextWindowSize({width, height});
    if (ImGui::BeginPopup("scene_settings")) {
        drawSceneSettingsPopup();
        ImGui::EndPopup();
    }
}

void App::toolbarPoolViewer() {
    sameLineIfFits(ImGui::CalcTextSize("Pool Viewer").x + 2 * ImGui::GetStyle().FramePadding.x);
    if (ImGui::Button("Pool Viewer") || openObserverPopup_) {
        openObserverPopup_ = false;
        ImGui::OpenPopup("observer_visibility");
    }
    if (ImGui::BeginPopup("observer_visibility")) {
        ImGui::Checkbox("Water", &observer_.water);
        ImGui::Checkbox("Pool walls & deck", &observer_.walls);
        ImGui::Checkbox("Pool floor", &observer_.floor);
        if (!mappingMarkers_.empty() && !demoMode_) {
            ImGui::SeparatorText("Course");
            ImGui::SetNextItemWidth(170);
            ImGui::Combo("Source", &courseMode_, "Auto\0Pack layout\0Mapping (RViz)\0");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Auto: the pack layout with simulator truth, the mapping estimate otherwise.\n"
                                  "Mapping: riptide_meshes at the mapping TF frames, as RViz shows them.");
            if (ros_->truthActive()) {
                ImGui::BeginDisabled(courseFromMapping());
                ImGui::Checkbox("Mapping ghost", &mappingGhost_);
                ImGui::EndDisabled();
                if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                    ImGui::SetTooltip("Draw the mapping estimate translucent over the simulator course.");
            }
        }
        if (ros_->truthActive() && !demoMode_) { // simulator only: a real robot has the estimate alone
            ImGui::SeparatorText("Localization estimate");
            ImGui::Checkbox("Robot ghost", &robotGhost_);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Draw the robot translucent at the localization estimate (TF base_link).");
            int anchor = anchorEstimate_ ? 1 : 0;
            ImGui::SetNextItemWidth(170);
            if (ImGui::Combo("Anchor", &anchor, "Truth (sim)\0Estimate (TF)\0"))
                anchorEstimate_ = anchor == 1;
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Robot the control gizmo and the Follow camera centre on. Commands always go to\n"
                                  "the controller in its (estimate) frame; on Truth they are shown re-rooted at the\n"
                                  "sim robot.");
        }
        ImGui::Checkbox("Surface reflections", &observer_.reflections);
        ImGui::Checkbox("Frame stats (F3)", &showProfile_);
        ImGui::SeparatorText("Viewer lighting");
        ImGui::BeginDisabled(observer_.lighting == 3);
        ImGui::Checkbox("Shadows", &observer_.shadows);
        ImGui::EndDisabled();
        ImGui::SetNextItemWidth(160);
        ImGui::Combo("Lighting", &observer_.lighting, "Scene lighting\0Indoor\0Outdoor\0Sterile\0");
        ImGui::SetNextItemWidth(160);
        ImGui::SliderFloat("Exposure", &observer_.exposure, .4f, 2.f, "%.2fx");
        ImGui::BeginDisabled(observer_.lighting == 3);
        ImGui::SetNextItemWidth(160);
        ImGui::SliderFloat("Brightness", &observer_.brightness, 0.f, 4.f, "%.2fx");
        ImGui::EndDisabled();
        ImGui::SetNextItemWidth(160);
        ImGui::SliderFloat("Ambient", &observer_.ambient, 0.f, 3.f, "%.2fx");
        if (ImGui::Button("Reset lighting", {-1, 30}))
            observer_.resetLighting();
        ImGui::EndPopup();
    }
}

void App::toolbarView() {
    const auto &s = *scenario_;
    ImGui::SetNextItemWidth(110);
    toolbarOldMode_ = mode_;
    std::string views = "Orbit";
    views += '\0';
    views += "Free camera";
    views += '\0';
    for (const auto &camera : s.cameras) {
        views += camera.id;
        views += '\0';
    }
    views += '\0';
    ImGui::Combo("##view", &mode_, views.c_str());
    if (mode_ == 1 && toolbarOldMode_ != 1) {
        // Continue from the last displayed view, including sensor-camera roll.
        const glm::mat4 cameraPose = glm::inverse(viewportView_.view);
        const glm::vec3 forward = -glm::normalize(glm::vec3(cameraPose[2]));
        freeEye_ = viewportView_.eye;
        freeYaw_ = std::atan2(forward.y, forward.x);
        freePitch_ = std::atan2(forward.z, glm::length(glm::vec2(forward)));
        const glm::vec3 right(std::sin(freeYaw_), -std::cos(freeYaw_), 0);
        const glm::vec3 up(cameraPose[1]);
        freeRoll_ = std::atan2(glm::dot(up, right), glm::dot(up, glm::cross(right, forward)));
    }
}

void App::toolbarFocus() {
    sameLineIfFits(toolbarLeft_ > 640 ? 132 : 110);
    ImGui::SetNextItemWidth(toolbarLeft_ > 640 ? 132 : 110);
    std::string focuses;
    for (const auto &name : focusNames_) {
        focuses += name;
        focuses += '\0';
    }
    focuses += '\0';
    if (ImGui::Combo("##focus", &selectedFocus_, focuses.c_str()))
        focus(focusNames_.at(std::size_t(selectedFocus_)));
}

void App::toolbarFollow() {
    sameLineIfFits(ImGui::GetFrameHeight() + ImGui::GetStyle().ItemInnerSpacing.x + ImGui::CalcTextSize("Follow").x);
    if (!presetFor(focusName_).follow)
        follow_ = false;
    ImGui::BeginDisabled(!presetFor(focusName_).follow);
    ImGui::Checkbox("Follow", &follow_);
    ImGui::EndDisabled();
}

void App::toolbarLabels() {
    sameLineIfFits(ImGui::GetFrameHeight() + ImGui::GetStyle().ItemInnerSpacing.x + ImGui::CalcTextSize("Labels").x);
    ImGui::Checkbox("Labels", &labels_);
}

void App::toolbarTf() {
    sameLineIfFits(ImGui::CalcTextSize("TF").x + 2 * ImGui::GetStyle().FramePadding.x);
    if (ImGui::Button("TF") || openTfPopup_) {
        openTfPopup_ = false;
        ImGui::OpenPopup("TF display");
    }
    tfTreeOpen_ = false;
    if (ImGui::BeginPopup("TF display")) {
        tfTreeOpen_ = true;
        ImGui::Checkbox("Show TF frames", &showTf_);
        ImGui::Checkbox("Frame names", &tfNames_);
        ImGui::SetNextItemWidth(220);
        ImGui::SliderFloat("Axis length", &tfAxisLength_, .02f, 1.f, "%.2f m");
        ImGui::TextUnformatted("X: red   Y: green   Z: blue");
        drawTfTree(tfTree_, scenario_->mapFrame);
        ImGui::TextDisabled("Axes show through objects. Unavailable frames cannot reach the fixed frame.");
        ImGui::TextDisabled("%s", demoMode_ ? "Preview: robot-pack frames at the preview pose."
                                            : "Raw ROS TF in the fixed frame, including localization drift.");
        ImGui::EndPopup();
    }
}

void App::drawDetectionSettings(bool includeEnable) {
    if (includeEnable) {
        ImGui::Checkbox("Show detections", &detections_);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Detector markers (%s), placed once when observed and then fixed in the world.\n"
                              "Truth: simulator pose at image capture. Estimate: TF at the image stamp.",
                              ros_->detectionTopic.c_str());
    }
    if (ros_->truthPlacementAvailable()) {
        // Truth/both only exist with a simulator; a real robot has the estimate alone.
        ImGui::BeginDisabled(!detections_);
        ImGui::SetNextItemWidth(130);
        int placement = int(ros_->detectionMode());
        if (ImGui::Combo("Placement", &placement, "Pose source\0Truth\0Estimate\0Both\0"))
            ros_->setDetectionMode(DetectionMode(placement));
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip("Detection placement: follow the pose source, simulator truth, localization\n"
                              "estimate (RViz-like TF), or both (truth solid, estimate cyan outline).");
    }
    bool keep = !ros_->honorDeleteAll();
    if (ImGui::Checkbox("Keep detections (ignore DELETEALL)", &keep))
        ros_->setHonorDeleteAll(!keep);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("The detector clears its markers every frame; keeping them lets each observation live out\n"
                          "its lifetime.");
    drawPointCloudSettings();
}

// Point cloud layers from the host config: one toggle each (subscribes only while on) and a point size.
void App::drawPointCloudSettings() {
    if (ros_->pointClouds.empty() || demoMode_)
        return;
    ImGui::SeparatorText("Point clouds");
    for (auto &layer : ros_->pointClouds) {
        ImGui::PushID(layer.id.c_str());
        ImGui::Checkbox(layer.title.c_str(), &layer.enabled);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s\nNewest message only, placed once in the fixed frame%s.", layer.topic.c_str(),
                              ros_->truthActive() ? " (camera clouds at the simulator truth pose)" : "");
        if (layer.enabled) {
            ImGui::SameLine();
            ImGui::SetNextItemWidth(80);
            ImGui::SliderFloat("##size", &layer.size, 1, 8, "%.0f px");
            ImGui::SameLine();
            const bool stale = layer.data && std::chrono::duration<double>(Clock::now() - layer.received).count() > 2;
            ImGui::TextDisabled("%s", !layer.data ? "waiting"
                                      : !layer.placed ? "no transform"
                                      : stale ? "stale"
                                              : layer.approximate ? "approx" : "");
        }
        ImGui::PopID();
    }
}

// Sidebar form: full settings plus the legend. Toolbar form (below): the checkbox and a dropdown of the rest.
void App::toolbarDetections() {
    if (demoMode_)
        return;
    sameLineIfFits(ImGui::GetFrameHeight() + ImGui::GetStyle().ItemInnerSpacing.x +
                   ImGui::CalcTextSize("Detections").x + ImGui::GetFrameHeight() + 4);
    ImGui::Checkbox("Detections", &detections_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Detector markers (%s), placed once when observed and then fixed in the world.\n"
                          "Truth: simulator pose at image capture. Estimate: TF at the image stamp.",
                          ros_->detectionTopic.c_str());
    ImGui::SameLine();
    if (ImGui::ArrowButton("##detection_options", ImGuiDir_Down))
        ImGui::OpenPopup("detection_options");
    if (ImGui::BeginPopup("detection_options")) {
        drawDetectionSettings(false);
        ImGui::EndPopup();
    }
}

void App::toolbarMpcPath() {
    if (demoMode_)
        return;
    sameLineIfFits(ImGui::GetFrameHeight() + ImGui::GetStyle().ItemInnerSpacing.x +
                   ImGui::CalcTextSize("MPC path").x);
    ImGui::Checkbox("MPC path", &showMpc_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Predicted MPC trajectory over its horizon (%s).\n"
                          "Drawn relative to the simulator vehicle, so localization drift does not offset it.",
                          ros_->mpcTopic.c_str());
}

void App::toolbarPreviewTask() {
    if (demoMode_ && !demoNames_.empty()) {
        ImGui::SameLine();
        std::string choices;
        for (const auto &name : demoNames_) {
            choices += name;
            choices += '\0';
        }
        choices += '\0';
        if (ImGui::Combo("##previewtask", &selectedDemo_, choices.c_str()))
            previewPose(demoNames_.at(std::size_t(selectedDemo_)));
    }
}

void App::drawToolbar(float left, int &oldMode) {
    toolbarLeft_ = left;
    toolbarOldMode_ = mode_;
    if (composition_)
        composition_->drawToolbar();
    oldMode = toolbarOldMode_;
}

// Host-provided toolbar items and panels: an explicit table, no static self-registration. Each type can be
// listed in the composition's `toolbar:`; `detections` can also be a sidebar panel.
void App::registerHostItems() {
    const auto add = [this](const char *type, std::function<void()> bar, std::function<void()> body = {}) {
        panels::registerHostItem(registry_, type, std::move(bar), std::move(body));
    };
    add("scene_settings", [this] { toolbarSceneSettings(); });
    add("pool_viewer", [this] { toolbarPoolViewer(); });
    add("view", [this] { toolbarView(); });
    add("focus", [this] { toolbarFocus(); });
    add("follow", [this] { toolbarFollow(); });
    add("labels", [this] { toolbarLabels(); });
    add("tf", [this] { toolbarTf(); });
    add("mpc_path", [this] { toolbarMpcPath(); });
    add("preview_task", [this] { toolbarPreviewTask(); });
    add("detections", [this] { toolbarDetections(); },
        [this] {
            ImGui::BeginDisabled(demoMode_);
            drawDetectionSettings(true);
            ImGui::EndDisabled();
            if (demoMode_)
                ImGui::TextDisabled("Preview: no detector feed.");
            ImGui::SeparatorText("Legend");
            ImGui::TextUnformatted("Solid: simulator truth placement");
            ImGui::TextColored(ImVec4(.25f, .9f, 1.f, .95f), "Cyan outline: estimate (TF), when both are shown");
            ImGui::TextDisabled("Dim dashed: approximate estimate (TF lagged)");
        });
}

void App::drawInterface(double time, float dt) {
    auto &io = ImGui::GetIO();
    const float W = io.DisplaySize.x, H = io.DisplaySize.y;
    ImGui::SetNextWindowPos({0, 0});
    ImGui::SetNextWindowSize({W, H});
    ImGui::Begin("Riptide", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings |
                     ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoScrollbar |
                     ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::SetScrollY(0);
    const std::string headerTitle = lookup(config_, {"branding", "header"}).as<std::string>(scenario_ ? scenario_->robotId : "");
    const std::string headerSubtitle = lookup(config_, {"branding", "subtitle"}).as<std::string>(scenario_ ? scenario_->id : "");
    ImGui::PushFont(window_->title);
    ImGui::TextUnformatted(headerTitle.c_str());
    ImGui::PopFont();
    ImGui::SameLine();
    ImGui::TextColored(muted, " / ");
    ImGui::SameLine();
    ImGui::TextUnformatted(headerSubtitle.c_str());
    const float statusWidth = ImGui::CalcTextSize(status_.c_str()).x + 2 * ImGui::GetStyle().FramePadding.x;
    ImGui::SameLine(W - 18 - statusWidth);
    pill(status_, demoMode_ ? ImVec4(.94f, .73f, .35f, 1) : (ros_->poseFresh() ? cyan : muted));
    ImGui::Separator();
    if (!scenario_) {
        ImGui::Dummy({1, 40});
        ImGui::TextColored(muted, "Waiting for the bridge scenario document (%s) ...",
                           !opt_.scenarioTopic.empty()
                               ? opt_.scenarioTopic.c_str()
                               : lookup(config_, {"scenario_topic"}).as<std::string>("/talos/simulator/scenario").c_str());
        ImGui::End();
        return;
    }
    if (runTracking_)
        runScore_.reset(runTracking_->state().score);
    const auto contentOrigin = ImGui::GetCursorScreenPos();
    const float contentHeight = H - contentOrigin.y - 12;
    const bool configuredPanels = composition_ && !composition_->empty();
    bool hasPanels = configuredPanels && composition_->sidebarVisible();
    float panelWidth = configuredPanels ? composition_->width(W) : 0;
    if (!cameraSidebarResized_)
        cameraSidebarWidth_ = glm::clamp(W * .29f, 335.f, 445.f);
    const float sidebarBudget = W - 36 - (configuredPanels ? 16 : 0) - 16 - 360;
    const float maxPanelWidth =
        std::max(300.f, std::min(600.f, sidebarBudget - (cameraSidebarVisible_ ? cameraSidebarWidth_ : 0)));
    panelWidth = glm::clamp(panelWidth, 300.f, maxPanelWidth);
    if (configuredPanels) {
        const float previous = panelWidth;
        if (panelEdge_.draw(contentOrigin, contentHeight, hasPanels, panelWidth, maxPanelWidth) != hasPanels)
            composition_->toggleSidebar();
        composition_->setWidth(panelWidth, panelWidth != previous);
        hasPanels = composition_->sidebarVisible();
    }
    const float sidebar = configuredPanels ? (hasPanels ? panelWidth : 0) + 16 : 0;
    const float maxCameraWidth = std::max(300.f, std::min(600.f, sidebarBudget - (hasPanels ? panelWidth : 0)));
    cameraSidebarWidth_ = glm::clamp(cameraSidebarWidth_, 300.f, maxCameraWidth);
    const float previousCameraWidth = cameraSidebarWidth_;
    ImGui::PushID("camera_sidebar");
    cameraSidebarVisible_ = cameraEdge_.draw({W - 18, contentOrigin.y}, contentHeight, cameraSidebarVisible_,
                                             cameraSidebarWidth_, maxCameraWidth, true);
    ImGui::PopID();
    if (cameraSidebarWidth_ != previousCameraWidth)
        cameraSidebarResized_ = true;
    ros_->setCamerasWanted(cameraSidebarVisible_);
    const float side = cameraSidebarVisible_ ? cameraSidebarWidth_ : 0;
    const float left = W - 36 - sidebar - side - 16;
    if (hasPanels) {
        ImGui::SetCursorScreenPos(contentOrigin);
        composition_->drawSidebar(contentHeight);
    }
    ImGui::SetCursorScreenPos({contentOrigin.x + sidebar, contentOrigin.y});
    ImGui::BeginChild("left", {left, contentHeight}, ImGuiChildFlags_None,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::SetScrollY(0);
    ImGui::BeginChild("toolbar", {left, toolbarHeight_}, ImGuiChildFlags_None,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    int oldMode = mode_;
    drawToolbar(left, oldMode);
    toolbarHeight_ = std::max(80.f, ImGui::GetCursorPosY());
    ImGui::EndChild();
    const float viewHeight = std::max(1.f, ImGui::GetContentRegionAvail().y);
    const ImVec2 position = ImGui::GetCursorScreenPos();
    bool hovered = ImGui::IsMouseHoveringRect(position, {position.x + left, position.y + viewHeight});
    // Dropdowns can overlap the viewport. Selecting Free camera must not also consume that click as a
    // mouse-capture request.
    hovered = hovered && ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows) && mode_ == oldMode &&
              !ImGui::IsPopupOpen(nullptr, ImGuiPopupFlags_AnyPopupId);
    panels::Viewport panelView{viewportView_.projection,
                               viewportView_.view,
                               viewportView_.eye,
                               {position.x, position.y},
                               {left, viewHeight},
                               hovered && mode_ == 0,
                               bool(glfwGetWindowAttrib(window_->handle(), GLFW_FOCUSED))};
    // Commands live in the estimate frame; anchored on the truth robot, draw them re-rooted there (offset
    // sampled at one time for both poses, see RosSide::truthFromEstimate).
    if (!anchorEstimate_ && haveEstimate_)
        panelView.displayFromCommand = body_ * glm::inverse(estimateBody_);
    const bool dragging = composition_ && mode_ == 0 && composition_->input(panelView);
    bool unused = false;
    SensorView view = viewFor(left / viewHeight, dt, hovered && !dragging, viewHeight, unused);
    viewportView_ = view;
    // Keep sensor aspect ratios when a camera view is promoted to the large viewport.
    float iw = left, ih = viewHeight;
    const SensorCamera *sensor = mode_ >= 2 ? &scenario_->cameras[std::size_t(mode_ - 2)] : nullptr;
    if (sensor) {
        const float aspect = float(sensor->k.width) / float(sensor->k.height);
        ih = std::min(viewHeight, left / aspect);
        iw = ih * aspect;
    }
    const int rw = std::max(16, int(iw)), rh = std::max(16, int(ih));
    // A sensor view keeps its own field of view at any displayed size.
    if (sensor) {
        auto k = sensor->k;
        k.fx *= double(rw) / k.width;
        k.cx *= double(rw) / k.width;
        k.fy *= double(rh) / k.height;
        k.cy *= double(rh) / k.height;
        k.width = rw;
        k.height = rh;
        view = sensorView(body_ * sensor->opticalInBase, k);
        viewportView_ = view;
    }
    rendering::Scene scene;
    {
        PhaseTimer timer{profiler_, Phase::Scene, false};
        scene = model_->build(buildState());
    }
    renderLocalCards(time, scene);
    if (!demoMode_)
        scene.points = ros_->pointSets(); // main view only (cards render without points)
    const rendering::View renderView{toEigen(view.view), toEigen(view.projection), Eigen::Vector3f(view.eye.x, view.eye.y, view.eye.z)};
    auto appearance = observer_.apply(look_.appearance);
    // Original viewer: a 3D focus disc at the orbit target while orbiting/zooming without Follow.
    if (mode_ == 0 && (orbitInteracting_ || opt_.showFocus) && !follow_)
        appearance.focus = Eigen::Vector3f(target_.x, target_.y, target_.z);
    rendering::RenderedFrame frame;
    {
        PhaseTimer timer{profiler_, Phase::Main, profileSync()};
        frame = renderer_->draw(scene, renderView, appearance, float(time), rw, rh);
    }
    lastFrame_ = frame;
    haveFrame_ = true;
    if (mode_ == 0 && hovered && !dragging && !io.WantTextInput && ImGui::IsKeyPressed(ImGuiKey_F))
        focusAtCursor(view, frame, position, left, viewHeight);
    const ImVec2 imagePos(position.x + (left - iw) / 2, position.y + (viewHeight - ih) / 2);
    ImGui::SetCursorScreenPos(imagePos);
    // A promoted sensor view shows the bridge's depth image instead while its card is on DEPTH.
    const std::size_t sensorIndex = sensor ? std::size_t(mode_ - 2) : 0;
    if (sensor && sensorIndex < cards_.size() && sensorIndex < ros_->feeds.size() && ros_->feeds[sensorIndex].wantDepth &&
        cards_[sensorIndex].depth)
        ImGui::Image(textureID(cards_[sensorIndex].depth), {iw, ih});
    else
        ImGui::Image(textureID(frame.color_texture), {iw, ih}, {0, 1}, {1, 0});
    const ScreenRect rect{imagePos, iw, ih};
    const auto vp = view.projection * view.view;
    if (showTf_) {
        TfOverlay overlay;
        overlay.snapshot = &tf_;
        overlay.axisLength = tfAxisLength_;
        overlay.names = tfNames_;
        overlay.font = window_->small;
        overlay.caption = demoMode_ ? "TF preview from robot pack"
                                    : "ROS TF in " + scenario_->mapFrame + ": " + std::to_string(tf_.resolved) +
                                          " frames, " + std::to_string(tf_.missing) + " unavailable";
        drawTfAxes(overlay, vp, rect);
    }
    if (detections_ && !demoMode_)
        drawDetections(ros_->placedDetections, vp, rect, ros_->detectionShow().truth && ros_->detectionShow().estimate);
    if (showMpc_ && !demoMode_)
        drawMpcPath(ros_->mpcPath, vp, rect);
    if (composition_ && mode_ == 0) {
        panelView.projection = view.projection;
        panelView.view = view.view;
        panelView.eye = view.eye;
        composition_->drawOverlays(panelView);
    }
    auto *d = ImGui::GetWindowDrawList();
    d->AddRect(position, {position.x + left, position.y + viewHeight}, IM_COL32(38, 62, 72, 255), 5, 0, 1);
    d->AddRectFilled({position.x + 14, position.y + 14}, {position.x + 237, position.y + 43}, IM_COL32(8, 22, 29, 225), 4);
    d->AddText(window_->small, 12, {position.x + 25, position.y + 22}, color(white),
               (scenario_->poolId + " / " + fixed(scenario_->poolLength, 1) + " x " + fixed(scenario_->poolWidth, 2) + " m")
                   .c_str());
    if (showProfile_ && !profiler_.recent().empty()) {
        if (time - profileCachedAt_ > .5) {
            profileCachedAt_ = time;
            std::vector<FrameSample> samples(profiler_.recent().begin(), profiler_.recent().end());
            const auto f = frameDistribution(samples);
            char text[640];
            std::snprintf(text, sizeof(text), "%.0f fps  frame ms  mean %.1f  p50 %.1f  p95 %.1f  p99 %.1f  max %.1f\n",
                          f.mean > 0 ? 1000 / f.mean : 0., f.mean, f.p50, f.p95, f.p99, f.max);
            profileText_ = text;
            profileText_ += "phase ms mean/max:";
            for (int i = 0; i < kPhaseCount; ++i) {
                const auto d = phaseDistribution(samples, Phase(i));
                std::snprintf(text, sizeof(text), "  %s %.1f/%.1f", phaseName(Phase(i)), d.mean, d.max);
                profileText_ += text;
            }
        }
        ImGui::PushFont(window_->small);
        const ImVec2 size = ImGui::CalcTextSize(profileText_.c_str());
        ImGui::PopFont();
        d->AddRectFilled({position.x + 14, position.y + 50}, {position.x + 30 + size.x, position.y + 62 + size.y},
                         IM_COL32(8, 22, 29, 225), 4);
        d->AddText(window_->small, 12, {position.x + 22, position.y + 56}, color(white), profileText_.c_str());
    }
    if (runScore_ && runScore_["total"]) {
        std::string readout = fixed(runScore_["total"].as<double>(), 1) + " pts   /   " + runTime();
        if (runScore_["running"].as<bool>(false))
            readout += "  RUNNING";
        d->AddRectFilled({position.x + left - 310, position.y + 12}, {position.x + left - 12, position.y + 43},
                         IM_COL32(8, 22, 29, 225), 4);
        d->AddText(window_->small, 14, {position.x + left - 298, position.y + 21}, color(cyan), readout.c_str());
    }
    if (labels_ && mode_ < 2 && presetFor(focusName_).labels) {
        for (const auto &key : focusNames_) {
            const auto found = scenario_->landmarks.find(key);
            if (found == scenario_->landmarks.end())
                continue;
            glm::vec4 p = view.projection * view.view * (found->second.world * glm::vec4(0, 0, .5, 1));
            if (p.w <= 0)
                continue;
            p /= p.w;
            if (std::abs(p.x) > .94 || std::abs(p.y) > .85 || p.z > 1)
                continue;
            const ImVec2 at(position.x + (p.x * .5f + .5f) * left, position.y + (.5f - p.y * .5f) * viewHeight);
            d->AddCircleFilled(at, 3, color(cyan));
            d->AddLine(at, {at.x + 10, at.y - 14}, color(cyan));
            d->AddRectFilled({at.x + 9, at.y - 31}, {at.x + 105, at.y - 11}, IM_COL32(8, 22, 29, 215), 3);
            d->AddText(window_->small, 12, {at.x + 16, at.y - 28}, color(white), key.c_str());
        }
    }
    const char *controls = mode_ == 1 ? "CLICK  mouse look    WASD  move    SPACE / SHIFT  up / down    CTRL  fast    ESC  release"
                                      : "LEFT DRAG  orbit   RIGHT / MIDDLE DRAG  pan   SCROLL  zoom   F  focus cursor";
    d->AddRectFilled({position.x, position.y + viewHeight - 30}, {position.x + left, position.y + viewHeight},
                     IM_COL32(6, 18, 26, 205));
    d->AddText(window_->small, 12, {position.x + 14, position.y + viewHeight - 21}, color(white), controls);
    ImGui::EndChild();
    if (cameraSidebarVisible_) {
        ImGui::SetCursorScreenPos({W - 18 - side, contentOrigin.y});
        ImGui::BeginChild("right", {side, contentHeight}, ImGuiChildFlags_None);
        const float cardWidth = ImGui::GetContentRegionAvail().x;
        for (std::size_t i = 0; i < ros_->feeds.size(); ++i)
            drawCameraCard(i, cardWidth, cardWidth);
        drawMinimap(cardWidth);
        ImGui::PushFont(window_->small);
        ImGui::TextWrapped("%s", demoMode_ ? "Scene preview. Start the simulator bridge to stream live cameras."
                                            : "Images are rendered by the bridge at the physics pose. Observer controls "
                                              "do not move the vehicle.");
        ImGui::PopFont();
        ImGui::EndChild();
    }
    ImGui::End();
    if (composition_)
        composition_->drawWindows();
}

// Local camera cards: this viewer's own render from each sensor pose (the truth pose live, the fixed preview
// pose in demo mode). Scene preview has no bridge; live cards can switch to the bridge's images per card.
// Cost control: at most ONE card per frame (each card refreshes every 0.1 s, staggered by the round-robin), only
// cards drawn on screen whose RGB is not replaced by the depth image or the ROS feed, the main frame's scene
// is reused, and the renderer's preview mode skips shadow/bloom/reflection passes.
double App::cardPeriod(std::size_t camera) const {
    const double rate = opt_.cardRate >= 0 ? opt_.cardRate : lookup(config_, {"cards", "rate_hz"}).as<double>(0);
    if (rate > 0)
        return 1. / rate;
    const double own = camera < scenario_->cameras.size() ? scenario_->cameras[camera].periodS : 1. / 15;
    return own > 0 ? own : 1. / 15;
}

void App::renderLocalCards(double t, const rendering::Scene &mainScene) {
    if (!scenario_ || !model_ || !(demoMode_ || opt_.localCameras) || !cameraSidebarVisible_)
        return;
    if (!demoMode_ && !(ros_->truthActive() && ros_->poseFresh()))
        return; // no simulator truth pose to render from
    const std::size_t count = std::min(scenario_->cameras.size(), cards_.size());
    std::vector<std::size_t> todo;
    for (std::size_t n = 0; n < count; ++n) {
        const std::size_t i = (nextCardTurn_ + n) % count;
        const bool ros = !demoMode_ && i < ros_->feeds.size() && ros_->feeds[i].rosMode; // truthActive checked above
        const bool depthShown = i < ros_->feeds.size() && ros_->feeds[i].wantDepth && cards_[i].depth;
        if (opt_.legacyCards ? (!ros && t >= cardDue_[0]) : (!ros && !depthShown && cardVisible_[i] && t >= cardDue_[i])) {
            todo.push_back(i);
            if (!opt_.legacyCards)
                break;
        }
    }
    if (todo.empty())
        return;
    nextCardTurn_ = todo.back() + 1;
    for (auto i : todo) {
        // Refresh at the camera's own rate (or the configured card rate) on a fixed grid so updates are evenly
        // spaced; after a stall, resume from now instead of bursting to catch up.
        const double period = demoMode_ ? .25 : cardPeriod(i);
        auto &due = cardDue_[opt_.legacyCards ? 0 : i];
        due = due > 0 && t - due < period ? due + period : t + period;
    }
    PhaseTimer timer{profiler_, Phase::Cards, profileSync()};
    rendering::Scene ownScene;
    // The robot's camera sees the simulated pool and course whatever the observer hides or overlays.
    const bool observerOnly = !observer_.walls || !observer_.floor || courseFromMapping() ||
                              (robotGhost_ && haveEstimate_) || mappingGhost_;
    if (observerOnly || opt_.legacyCards) {
        auto state = buildState();
        state.showWalls = state.showFloor = state.showCourse = true;
        state.ghostBody.reset();
        state.markers.erase(std::remove_if(state.markers.begin(), state.markers.end(),
                                           [](const MarkerDraw &m) { return m.observerOnly; }),
                            state.markers.end());
        if (courseFromMapping()) // buildState skipped the simulator props for the mapped course
            for (const auto &[key, record] : ros_->props)
                if (!record.mesh.empty()) {
                    MarkerDraw draw;
                    draw.mesh = record.mesh;
                    draw.world = record.attached ? body_ * record.pose : record.pose;
                    draw.scale = {float(record.marker.scale.x), float(record.marker.scale.y),
                                  float(record.marker.scale.z)};
                    state.markers.push_back(std::move(draw));
                }
        ownScene = model_->build(state);
    }
    const rendering::Scene &scene = !observerOnly && !opt_.legacyCards ? mainScene : ownScene;
    for (const auto i : todo) {
        const auto &camera = scenario_->cameras[i];
        auto k = camera.k;
        const int w = 480, h = std::max(16, int(std::lround(480.0 * k.height / k.width)));
        k.fx *= double(w) / k.width;
        k.cx *= double(w) / k.width;
        k.fy *= double(h) / k.height;
        k.cy *= double(h) / k.height;
        k.width = w;
        k.height = h;
        const auto v = sensorView(body_ * camera.opticalInBase, k);
        const rendering::View renderView{toEigen(v.view), toEigen(v.projection), Eigen::Vector3f(v.eye.x, v.eye.y, v.eye.z)};
        auto appearance = look_.appearance;
        appearance.preview = !opt_.legacyCards;
        const auto frame = renderer_->draw(scene, renderView, appearance, float(t), w, h);
        auto &card = cards_[i];
        if (!card.rgb || card.rgbWidth != w || card.rgbHeight != h) {
            if (!card.rgb)
                glGenTextures(1, &card.rgb);
            glBindTexture(GL_TEXTURE_2D, card.rgb);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            card.rgbWidth = w;
            card.rgbHeight = h;
        }
        glBindFramebuffer(GL_READ_FRAMEBUFFER, readFbo_);
        glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, frame.color_texture, 0);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, drawFbo_);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, card.rgb, 0);
        glBlitFramebuffer(0, 0, w, h, 0, 0, w, h, GL_COLOR_BUFFER_BIT, GL_NEAREST);
        glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        card.flipped = true;
    }
}

void App::saveCameraImages(const fs::path &screenshot) {
    if (screenshot.empty() || !scenario_)
        return;
    for (std::size_t i = 0; i < ros_->feeds.size() && i < cards_.size(); ++i) {
        const auto &feed = ros_->feeds[i];
        if (feed.rgb.empty() && cards_[i].rgb && cards_[i].flipped) { // locally rendered card: read it back
            const auto &card = cards_[i];
            std::vector<std::uint8_t> bottomUp(std::size_t(card.rgbWidth) * std::size_t(card.rgbHeight) * 3), rgb;
            glBindTexture(GL_TEXTURE_2D, card.rgb);
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, bottomUp.data());
            const std::size_t row = std::size_t(card.rgbWidth) * 3;
            for (int y = card.rgbHeight - 1; y >= 0; --y)
                rgb.insert(rgb.end(), bottomUp.begin() + std::ptrdiff_t(std::size_t(y) * row),
                           bottomUp.begin() + std::ptrdiff_t(std::size_t(y + 1) * row));
            writePng(screenshot.parent_path() / (screenshot.stem().string() + "-" + feed.camera->id + ".png"),
                     card.rgbWidth, card.rgbHeight, rgb);
            continue;
        }
        if (feed.rgb.empty())
            continue;
        writePng(screenshot.parent_path() / (screenshot.stem().string() + "-" + feed.camera->id + ".png"),
                 feed.rgbWidth, feed.rgbHeight, feed.rgb);
    }
}

int App::loop() {
    int frames = 0;
    auto previous = start_;
    while (!window_->closing() && (demoMode_ || rclcpp::ok())) {
        const auto frameStart = Clock::now();
        const double t = std::chrono::duration<double>(frameStart - start_).count();
        const float dt = float(std::min(std::chrono::duration<double>(frameStart - previous).count(), .1));
        previous = frameStart;
        {
            PhaseTimer timer{profiler_, Phase::Spin, false};
            ros_->spin();
        }
        if (!pendingScenario_.empty()) {
            const auto json = std::move(pendingScenario_);
            pendingScenario_.clear();
            try {
                loadScenario(json);
            } catch (const std::exception &error) {
                std::cerr << "robotics-pool-viewer: rejecting scenario document: " << error.what() << '\n';
            }
        }
        if (composition_)
            composition_->touch();
        step(t);
        if (opt_.injectF.size() == 2 && opt_.frames > 0 && scenario_) {
            auto &io = ImGui::GetIO();
            if (frames == opt_.frames / 2)
                io.AddMousePosEvent(opt_.injectF[0], opt_.injectF[1]);
            if (frames == opt_.frames / 2 + 2)
                io.AddKeyEvent(ImGuiKey_F, true);
            if (frames == opt_.frames / 2 + 3)
                io.AddKeyEvent(ImGuiKey_F, false);
            if (frames == opt_.frames / 2 + 6)
                std::cout << "inject-f: target=(" << target_.x << "," << target_.y << "," << target_.z
                          << ") distance=" << distance_ << " follow=" << follow_ << "\n";
        }
        profiler_.setPosition(body_[3].x, body_[3].y, body_[3].z);
        window_->beginFrame();
        drawInterface(t, dt);
        if (largeMap_ && scenario_) {
            ImGui::SetNextWindowSize({1000, 620}, ImGuiCond_FirstUseEver);
            if (focusMap_) {
                ImGui::SetNextWindowFocus();
                ImGui::SetNextWindowCollapsed(false);
                focusMap_ = false;
            }
            if (ImGui::Begin("Course map", &largeMap_)) {
                ImGui::TextDisabled("SCROLL zoom / DRAG pan / CLICK a task to focus the pool view");
                ImGui::SameLine();
                if (ImGui::SmallButton("Fit pool")) {
                    mapZoom_ = 1;
                    mapPan_ = {0, 0};
                }
                auto space = ImGui::GetContentRegionAvail();
                drawCourseMap(space.x, std::max(100.f, space.y), true);
            }
            ImGui::End();
        }
        if (opt_.frames > 0 && window_->imguiErrors() > 0)
            throw std::runtime_error("ImGui validation failed during capture run");
        // A capture run starts counting once the scene exists (the scenario topic can arrive late).
        if (scenario_)
            ++frames;
        else if (opt_.frames > 0 && t > 30)
            throw std::runtime_error("no scenario document received within 30 s");
        const bool last = opt_.frames > 0 && frames >= opt_.frames;
        if (ImGui::IsKeyPressed(ImGuiKey_F3, false) && !ImGui::GetIO().WantTextInput)
            showProfile_ = !showProfile_;
        window_->present(last, opt_.screenshot);
        const auto preSwap = Clock::now();
        window_->swap();
        const auto postSwap = Clock::now();
        {
            const auto sec = [](Clock::duration d) { return std::chrono::duration<double>(d).count(); };
            profiler_.add(Phase::Swap, sec(postSwap - preSwap));
            const auto phases = profiler_.pendingPhases();
            profiler_.add(Phase::Ui, std::max(0., sec(preSwap - frameStart) - phases[int(Phase::Spin)] -
                                                       phases[int(Phase::Scene)] - phases[int(Phase::Main)] -
                                                       phases[int(Phase::Cards)]));
        }
        if (last) {
            if (scenario_)
                std::cout << "capture: status=" << status_ << " detections stored/placed=" << ros_->detectionCount() << "/"
                          << ros_->placedDetections.size() << " mpc_points=" << ros_->mpcPath.size()
                          << " props=" << ros_->props.size() << " projectiles=" << ros_->projectiles.size()
                          << " magnet_lights=" << ros_->magnetLights.size() << " tf_frames=" << tf_.frames.size()
                          << " camera_frames=" << (ros_->feeds.empty() ? 0 : ros_->feeds[0].frames) << " body=("
                          << body_[3].x << "," << body_[3].y << "," << body_[3].z << ")";
                for (const auto &layer : ros_->pointClouds)
                    if (layer.enabled)
                        std::cout << " cloud[" << layer.id << "]=" << (layer.data ? layer.data->xyzrgb.size() / 6 : 0)
                                  << (layer.placed ? " placed" : " unplaced");
                std::cout << "\n";
            saveCameraImages(opt_.screenshot);
            break;
        }
        const double cap = opt_.renderRate > 0 ? opt_.renderRate : opt_.hidden ? 30. : opt_.vsync ? 0. : 60.;
        if (cap > 0) {
            const auto before = Clock::now();
            std::this_thread::sleep_until(frameStart + std::chrono::duration_cast<Clock::duration>(
                                                           std::chrono::duration<double>(1. / cap)));
            profiler_.add(Phase::Sleep, std::chrono::duration<double>(Clock::now() - before).count());
        }
        profiler_.endFrame(Clock::now());
        if (opt_.profile) {
            const auto now = Clock::now();
            if (profileLogAt_ == Clock::time_point{})
                profileLogAt_ = now;
            if (now - profileLogAt_ >= std::chrono::seconds(5)) {
                std::cout << profileReport(profiler_.takeInterval(), std::chrono::duration<double>(now - profileLogAt_).count()) << (ros_ ? ros_->timingReport() : std::string())
                          << std::endl;
                profileLogAt_ = now;
            }
        }
    }
    if (opt_.profile && profiler_.pending() > 1)
        std::cout << profileReport(profiler_.takeInterval(), 0) << std::endl;
    return 0;
}

int run(const Options &options, int argc, char **argv) {
    App app(options, argc, argv);
    return app.loop();
}
} // namespace robotics::ros_viewer::host

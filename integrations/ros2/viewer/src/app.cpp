#include "app.hpp"
#include "frame_profiler.hpp"
#include "mapping_markers.hpp"
#include "nereus/ros_viewer/dock_layout.hpp"
#include "nereus/ros_viewer/panel_layout.hpp"
#include "nereus/ros_viewer/panels/composition.hpp"
#include "nereus/ros_viewer/panels/ros_providers.hpp"
#include "nereus/ros_viewer/pins.hpp"
#include "nereus/ros_viewer/theme.hpp"
#include "overlay_draw.hpp"
#include "ros_side.hpp"
#include "scene_model.hpp"
#include "viewer_input.hpp"
#include "window.hpp"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <array>
#include <cctype>
#include <cmath>
#include <cstring>
#include <deque>
#include <fstream>
#include <imgui_internal.h>
#include <iomanip>
#include <iostream>
#include <nereus/rendering/renderer.hpp>
#include <set>
#include <sstream>
#include <thread>

namespace nereus::ros_viewer::host {
namespace {
namespace fs = std::filesystem;
namespace panels = nereus::ros_viewer::panels;

// Drawn over the 3D view and the course map, so the same in every theme (the interface uses palette()).
const ImVec4 cyan(.32f, .86f, .82f, 1), muted(.47f, .57f, .64f, 1), white(.87f, .92f, .95f, 1);

// "Talos · Robosub 2026" from the robot and task-pack ids (underscores become spaces, words capitalized).
std::string scenarioLabel(const Scenario &scenario) {
    std::string label = scenario.robotId + " \u00b7 " + scenario.tasksId;
    bool word = true;
    for (auto &c : label) {
        if (c == '_')
            c = ' ';
        const bool letter = std::isalpha(static_cast<unsigned char>(c)) != 0;
        if (letter && word)
            c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        word = !letter && !std::isdigit(static_cast<unsigned char>(c));
    }
    return label;
}
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
#ifdef NEREUS_VIEWER_CONTENT
    return NEREUS_VIEWER_CONTENT;
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
    for (const char *group : {"panels", "toolbar", "header", "overlays"}) {
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
    bool equipment = true; // equipment pack visuals in the observer view
};
struct ObserverSettings {
    bool water = true, walls = true, floor = true, reflections = false, shadows = true;
    int lighting = 0; // 0 follows the scene, 1 indoor, 2 outdoor, 3 sterile
    float exposure = 1, brightness = 1, ambient = 1;
    int antialiasing = 1; // supersampling factor of the observer view and camera cards (1 = off)
    void resetLighting() {
        shadows = true;
        lighting = 0;
        exposure = brightness = ambient = 1;
    }
    rendering::Appearance apply(rendering::Appearance scene) const {
        scene.supersample = antialiasing;
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
    bool open = true;     // its camera window is shown
};

// Dockable host windows. The part after ### is the window's identity in saved layouts; the title can change.
constexpr const char *kPoolView = "Pool view###pool_view", *kCourseMap = "Course map###course_map",
                     *kSceneSettings = "Scene settings###scene_settings", *kDisplay = "Display###display",
                     *kTfFrames = "TF frames###tf", *kHelp = "Controls & shortcuts###help";
std::string cameraWindowName(const SensorCamera &camera) {
    return camera.title + "###camera." + camera.id;
}
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
    void drawMenuBar();
    void drawViewMenu();
    void drawThemeMenu();
    void drawWindowsMenu();
    void drawLayoutMenu();
    void drawCommandBar();
    void drawPoolView(double time, float dt);
    void drawCameraWindows();
    void drawMapWindow();
    void drawSceneSettingsWindow();
    void drawDisplayWindow();
    void drawTfWindow();
    void drawHelpWindow();
    void handleShortcuts();
    void setViewMode(int mode);
    void drawToolbar(float left, int &oldMode);
    void registerHostItems();
    void toolbarSceneSettings();
    void drawSceneSettings();
    void toolbarPoolViewer();
    void drawDisplaySettings();
    void toolbarView();
    void toolbarFocus();
    void loadMappingMarkers();
    void drawPointCloudSettings();
    void drawMappingMeshList();
    void toolbarFollow();
    void toolbarLabels();
    void toolbarTf();
    void toolbarDetections();
    void drawDetectionSettings(bool includeEnable);
    void toolbarMpcPath();
    void toolbarThrust();
    void toolbarPreviewTask();
    void drawCameraCard(std::size_t index);
    void drawCourseMap(float width, float height, bool compact);
    void drawWaterControls();
    void pill(const std::string &text, ImVec4 tint);
    void sectionHeading(const char *text);
    std::string runTime() const;
    void saveCameraImages(const fs::path &screenshot);
    double cardPeriod(std::size_t camera) const;
    void renderLocalCards(double t, const rendering::Scene &mainScene);
    // --- window layout
    struct WindowEntry {
        std::string key, name, label; // stable key ("panel.<id>", "camera.<id>", "map", ...), ImGui name, title
        bool *open;
    };
    std::vector<WindowEntry> windowEntries();
    WindowStates::Flags windowFlags();
    WindowStates::Flags toolbarFlags();
    void windowContextMenu(const std::string &key);
    void pinnedWindowButton(const WindowEntry &);
    void showWindow(const WindowEntry &);
    void drawToolbarCustomization();
    void resetToolbar();
    void drawWindowControls();
    void handleWindowEdges();
    void setTheme(const std::string &id);
    // Interface scale: 0 follows the desktop (the window's content scale); the title bar always does.
    float uiScaleSetting_ = 1, pendingUiScale_ = -1;
    float resolvedUiScale() const;
    void setUiScale(float setting); // remembered in viewer.yaml
    void applyPendingUiScale();     // between frames: fonts and style at the new scale
    void drawScaleMenu();
    void savePreference(const char *key, const YAML::Node &value);
    // Side columns snapped shut (dragged nearly closed, the View menu, Ctrl+[ / Ctrl+]): their windows close and
    // dragging the pool view's edge out brings them back, as the old sidebars' edges did.
    struct SideState {
        bool collapsed = false;
        std::vector<std::string> keys; // windows closed with the side
        float width = 0;               // last comfortable width
        float pending = 0;             // width to give it once its windows show again (dragged out)
        bool dragging = false;         // the viewer follows the held mouse: snap shut / pull back out live
    };
    std::array<SideState, 2> sides_;
    void updateSides();
    void collapseSide(Side);
    void expandSide(Side);
    void toggleSide(Side);
    bool sideShown(Side) const;
    void resetSides();
    bool drawSideHandle(Side, ImVec2 viewPos, ImVec2 viewSize, bool draw);
    std::string layoutSnapshot();
    void rememberDefaults();
    void applyCommandLineWindows();
    void requestLayout(const std::string &name);
    void applyPendingIni();
    void applyPreset(const std::string &id);
    void saveLayout(const std::string &name);
    void toggleMaximized();
    void persistLayout(bool force);
    void focusIfRequested(const std::string &window);
    std::string defaultPreset() const;

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
    bool openDepth_ = false;
    bool showTf_ = false, tfNames_ = true, tfTreeOpen_ = false, detections_ = false, showMpc_ = false,
         showThrust_ = false, demoMode_ = false;
    float thrustScale_ = .05f; // arrow metres per newton (host yaml thrust_arrows.metres_per_newton)
    float tfAxisLength_ = .12f, mapZoom_ = 1, toolbarLeft_ = 0;
    int toolbarOldMode_ = 0;
    // Course source: 0 auto (pack layout with simulator truth, mapping markers otherwise), 1 pack, 2 mapping.
    std::vector<MappingMarker> mappingMarkers_;
    int courseMode_ = 0;
    bool mappingGhost_ = false;
    char meshFilter_[64]{}; // Pool Viewer > Course > Meshes search
    bool courseFromMapping() const;
    // Simulator only (truth is the pose source): the localization estimate at its display time, drawn as a
    // translucent robot ghost; the control gizmo and Follow are anchored together on it or on the truth robot.
    glm::mat4 estimateBody_{1};
    bool haveEstimate_ = false, robotGhost_ = false, gizmoOnEstimate_ = true, followEstimate_ = false;
    // Follow on the estimate: truth * low-passed estimate error. The error is mostly EKF jitter frame to frame
    // but drifts slowly, so smoothing it steadies the camera without lagging the robot's own motion.
    glm::vec3 errorShift_{0};
    glm::quat errorTurn_{1, 0, 0, 0};
    bool haveError_ = false;
    Clock::time_point errorTime_{};
    void smoothEstimateError();
    glm::mat4 followBody() const {
        return followEstimate_ && haveEstimate_ && haveError_
                   ? glm::translate(glm::mat4(1), errorShift_) * glm::mat4_cast(errorTurn_) * body_
                   : body_;
    }
    ImVec2 mapPan_{0, 0};
    TfTree tfTree_;
    TfSnapshot tf_;
    std::deque<glm::vec3> trail_;
    std::string status_ = "WAITING FOR SCENARIO";
    std::vector<CardTexture> cards_;
    std::vector<double> cardDue_;   // next render time per card
    std::vector<char> cardVisible_; // drawn on screen last frame (scrolled-out / hidden cards are skipped)
    std::size_t nextCardTurn_ = 0;  // round-robin start so cards share the frame budget evenly
    // windows: open state of the host windows (camera windows: CardTexture::open; panels: the composition)
    bool mapOpen_ = true, sceneOpen_ = false, displayOpen_ = false, tfOpen_ = false, helpOpen_ = false;
    bool focusMap_ = false, camerasShown_ = true;
    ImVec2 sceneAnchor_{120, 140}, displayAnchor_{220, 140}, tfAnchor_{320, 140};
    float toolbarHeight_ = 46;
    // layout: dock space, built-in presets, the session layout saved in the config directory, named layouts
    ImGuiID dockspace_ = 0;
    WindowStates windowStates_{[this] {
        auto flags = windowFlags();
        for (const auto &flag : toolbarFlags())
            flags.push_back(flag);
        return flags;
    }};
    std::map<std::string, bool> pins_;        // windows pinned to the toolbar, by window key
    bool titleDrag_ = false;                  // a press on the empty title bar that becomes a window move once dragged
    std::map<std::string, bool> defaultOpen_; // window states a built-in layout resets to
    std::set<std::string> focusOnce_;         // windows brought to front (tab selected) on their next frame
    std::string pendingPreset_, pendingIni_, beforeMaximize_, layoutMessage_;
    fs::path sessionIni_, layoutDir_, preferencesFile_;
    bool persist_ = false, layoutReady_ = false, layoutLocked_ = false, maximized_ = false, commandLineDone_ = false,
         presetApplied_ = false;
    char layoutName_[64]{};
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
    fs::path configPath =
        opt_.configPath.empty() ? contentDirectory() / "talos_uwrt_host.yaml" : fs::path(opt_.configPath);
    config_ = YAML::LoadFile(configPath.string());
    configDir_ = configPath.parent_path();
    demoMode_ = opt_.demo;
    for (const auto &name : opt_.open)
        openDepth_ |= name == "depth";
    showTf_ = opt_.showTf;
    detections_ = opt_.detections.value_or(lookup(config_, {"detections", "enabled"}).as<bool>(true));
    showMpc_ = opt_.mpcPath;
    showThrust_ = opt_.thrust || lookup(config_, {"thrust_arrows", "enabled"}).as<bool>(false);
    thrustScale_ = lookup(config_, {"thrust_arrows", "metres_per_newton"}).as<float>(.05f);
    if (!std::isfinite(thrustScale_) || thrustScale_ <= 0)
        throw std::runtime_error("thrust_arrows.metres_per_newton must be positive");
    if (!demoMode_) {
        rclcpp::init(argc, argv);
        // Without a simulator (real robot) there is no /clock: default to wall time.
        const bool estimateOnly =
            (!opt_.poseSource.empty() ? opt_.poseSource
                                      : lookup(config_, {"pose", "source"}).as<std::string>("auto")) == "estimate";
        const bool simTime = opt_.useSimTime.value_or(!estimateOnly);
        node_ = std::make_shared<rclcpp::Node>(
            "nereus_viewer", rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", simTime)}));
    }
    ros_ = std::make_unique<RosSide>(node_);
    ros_->configurePose(
        parsePoseSource(!opt_.poseSource.empty() ? opt_.poseSource
                                                 : lookup(config_, {"pose", "source"}).as<std::string>("auto")),
        opt_.truthDelay >= 0 ? opt_.truthDelay : lookup(config_, {"pose", "truth_delay_s"}).as<double>(.02),
        opt_.otherDelay >= 0 ? opt_.otherDelay : lookup(config_, {"pose", "other_delay_s"}).as<double>(.06));
    robotGhost_ = lookup(config_, {"pose", "robot_ghost"}).as<bool>(false);
    gizmoOnEstimate_ = lookup(config_, {"pose", "gizmo_anchor"}).as<std::string>("estimate") == "estimate";
    followEstimate_ = lookup(config_, {"pose", "follow_anchor"}).as<std::string>("truth") == "estimate";
    ros_->setDetectionMode(
        parseDetectionMode(!opt_.detectionPlacement.empty()
                               ? opt_.detectionPlacement
                               : lookup(config_, {"detections", "placement"}).as<std::string>("pose_source")));
    ros_->setHonorDeleteAll(!opt_.keepDetections && lookup(config_, {"detections", "honor_delete_all"}).as<bool>(true));
    const int width = lookup(config_, {"window", "width"}).as<int>(1480),
              height = lookup(config_, {"window", "height"}).as<int>(940);
    const bool customTitleBar =
        !opt_.systemTitleBar.value_or(lookup(config_, {"window", "title_bar"}).as<std::string>("custom") == "system");
    window_ =
        std::make_unique<Window>(width, height, lookup(config_, {"branding", "window_title"}).as<std::string>("Nereus"),
                                 opt_.hidden, opt_.vsync, customTitleBar);
    // Window layout: the operator's last session (config directory), unless --layout names one. Capture runs
    // (--frames) neither read nor write it, so they always see the configured built-in layout.
    dockspace_ = ImHashStr("nereus_dockspace");
    defaultOpen_ = {{"map", true}, {"scene_settings", false}, {"display", false}, {"tf", false}, {"help", false}};
    windowStates_.install();
    pins::install();
    const auto configHome = configDirectory();
    persist_ = opt_.frames == 0 && !configHome.empty();
    if (!configHome.empty()) {
        sessionIni_ = configHome / "viewer_layout.ini";
        layoutDir_ = configHome / "layouts";
    }
    if (!opt_.layout.empty())
        requestLayout(opt_.layout);
    else if (persist_ && fs::exists(sessionIni_)) {
        ImGui::LoadIniSettingsFromDisk(sessionIni_.c_str());
        layoutReady_ = ImGui::DockBuilderGetNode(dockspace_) != nullptr;
    }
    applyPendingIni();
    if (!layoutReady_ && pendingPreset_.empty())
        pendingPreset_ = defaultPreset();
    // Theme: --theme, else the operator's last choice (viewer.yaml in the config directory), else the host config.
    if (!configHome.empty())
        preferencesFile_ = configHome / "viewer.yaml";
    std::string theme = opt_.theme;
    if (theme.empty() && persist_ && fs::exists(preferencesFile_))
        try {
            theme = YAML::LoadFile(preferencesFile_.string())["theme"].as<std::string>("");
        } catch (const std::exception &error) {
            std::cerr << "nereus-viewer: ignoring " << preferencesFile_ << ": " << error.what() << '\n';
        }
    if (theme.empty())
        theme = lookup(config_, {"theme"}).as<std::string>(themes().front().id);
    if (!applyTheme(theme)) {
        std::cerr << "nereus-viewer: unknown theme '" << theme << "'; using " << themes().front().id << '\n';
        applyTheme(themes().front().id);
    }
    // Interface scale: --ui-scale, else the operator's last choice, else the host config; "auto" follows the desktop.
    YAML::Node scaleSetting = lookup(config_, {"interface_scale"});
    if (persist_ && opt_.uiScale.empty() && fs::exists(preferencesFile_))
        try {
            if (auto saved = YAML::LoadFile(preferencesFile_.string())["interface_scale"])
                scaleSetting = saved;
        } catch (const std::exception &) {
        }
    const std::string scaleText = !opt_.uiScale.empty() ? opt_.uiScale : scaleSetting.as<std::string>("1");
    try {
        uiScaleSetting_ = scaleText == "auto" ? 0.f : std::clamp(std::stof(scaleText), .5f, 4.f);
    } catch (const std::exception &) {
        std::cerr << "nereus-viewer: interface scale '" << scaleText << "' is not a number or auto; using 1\n";
        uiScaleSetting_ = 1;
    }
    pendingUiScale_ = resolvedUiScale();
    applyPendingUiScale();
    fs::path shaders = opt_.shaders;
#ifdef NEREUS_RENDERING_SHADERS
    if (shaders.empty())
        shaders = NEREUS_RENDERING_SHADERS;
#endif
    if (shaders.empty())
        shaders = fs::canonical("/proc/self/exe").parent_path().parent_path() / "share/nereus/shaders";
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
        const std::string topic =
            !opt_.scenarioTopic.empty()
                ? opt_.scenarioTopic
                : lookup(config_, {"scenario_topic"}).as<std::string>("/talos/simulator/scenario");
        ros_->watchScenario(topic, [this](const std::string &json) { pendingScenario_ = json; });
    }
}

App::~App() {
    try {
        persistLayout(true); // before the panels (their window states) go
    } catch (const std::exception &error) {
        std::cerr << "nereus-viewer: layout not saved: " << error.what() << '\n';
    }
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
        std::cerr << "nereus-viewer: status lights disabled: " << error.what() << '\n';
    }
    const auto thrusterPath = resolve("thruster_visuals_config", "talos_uwrt_thruster_visuals.yaml");
    try {
        if (fs::exists(thrusterPath) && !scenario_->thrusterOrder.empty())
            thrusters_ = ThrusterVisuals(YAML::LoadFile(thrusterPath.string()), scenario_->thrusterOrder);
    } catch (const std::exception &error) {
        thrusters_ = {};
        std::cerr << "nereus-viewer: thruster animation disabled: " << error.what() << '\n';
    }
    SceneModelOptions options;
    options.config = config_;
    options.robotOnly = opt_.robotOnly;
    model_ = std::make_unique<SceneModel>(*scenario_, options, thrusters_, lights_);
    ros_->attach(*scenario_, config_, lights_, thrusters_, !demoMode_);
    loadMappingMarkers();
    look_.appearance = scenario_->appearance;
    look_.equipment = lookup(config_, {"equipment_visible"}).as<bool>(true);
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
    window_->setTitle(
        lookup(config_, {"branding", "window_title"}).as<std::string>("Nereus | " + scenarioLabel(*scenario_)));
    if (demoMode_) {
        const auto p = lookup(config_, {"preview", "pose"});
        body_ = p ? pose(vec3(p), {0, 0, p[5].as<float>(0)}) : pose({3, -2, -.75f}, {0, 0, -.14f});
        const std::string task =
            !opt_.demoTask.empty() ? opt_.demoTask : lookup(config_, {"preview", "task"}).as<std::string>("");
        if (!task.empty() && scenario_->landmarks.count(task))
            previewPose(task);
    }
    if (!composition_)
        buildPanels();
    // New windows (this scenario's cameras, the panels) start as configured, then as the saved layout has them.
    rememberDefaults();
    windowStates_.apply();
    if (!commandLineDone_)
        applyCommandLineWindows();
    std::string initial =
        !opt_.initialFocus.empty() ? opt_.initialFocus : lookup(config_, {"initial_focus"}).as<std::string>("Vehicle");
    if (!focusable(initial))
        initial = "Vehicle";
    focus(initial);
    applyInitialView();
    status_ = demoMode_ ? "SCENE PREVIEW" : "WAITING FOR PHYSICS";
}

void App::buildPanels() {
    const std::string configured =
        opt_.panelsPath.empty()
            ? (configDir_ / lookup(config_, {"panels_config"}).as<std::string>("talos_uwrt_panels.yaml")).string()
            : opt_.panelsPath;
    const bool haveConfig = configured != "none" && fs::exists(configured);
    if (!haveConfig && configured != "none")
        std::cerr << "nereus-viewer: panel composition " << configured << " not found; panels disabled\n";
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
    auto document = haveConfig ? YAML::LoadFile(configured) : YAML::Load("{providers: {}}");
    if (ros_->poseSource() == PoseSource::Estimate)
        dropSimulatorPanels(document); // real robot: no simulator run / rate controls
    composition_ = std::make_unique<panels::Composition>(document, context, registry_);
    composition_->setWindowMenu([this] { drawWindowsMenu(); });
    composition_->setWindowContextMenu([this](const std::string &id) { windowContextMenu("panel." + id); });
    // A saved layout from before these panels existed (or of another composition) does not place them: start
    // from the built-in layout instead of leaving every panel floating.
    if (layoutReady_ && pendingPreset_.empty()) {
        bool placed = composition_->empty();
        for (const auto &window : composition_->panelWindows())
            placed = placed || ImGui::FindWindowSettingsByID(ImHashStr(window.name.c_str())) != nullptr;
        if (!placed)
            pendingPreset_ = defaultPreset();
    }
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
    smoothEstimateError();
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
    ros_->captureThrust(showThrust_ && !demoMode_);
    thrusters_.advance(clockSeconds());
    (void)t;
}

void App::smoothEstimateError() {
    const auto now = Clock::now();
    if (!haveEstimate_) {
        haveError_ = false;
        return;
    }
    // World-frame error: estimate = error * truth.
    const glm::mat4 error = estimateBody_ * glm::inverse(body_);
    const glm::vec3 shift(error[3]);
    const glm::quat turn = glm::normalize(glm::quat_cast(glm::mat3(error)));
    const double dt = std::chrono::duration<double>(now - errorTime_).count();
    errorTime_ = now;
    // Take large changes (reset, placement, re-anchoring) at once; filter the rest with a 0.5 s time constant.
    if (!haveError_ || dt > 1 || glm::distance(shift, errorShift_) > .5f ||
        std::abs(glm::dot(turn, errorTurn_)) < std::cos(glm::radians(15.f) / 2)) {
        errorShift_ = shift;
        errorTurn_ = turn;
        haveError_ = true;
        return;
    }
    const float alpha = float(1 - std::exp(-dt / .5));
    errorShift_ += (shift - errorShift_) * alpha;
    errorTurn_ = glm::normalize(glm::slerp(errorTurn_, glm::dot(turn, errorTurn_) < 0 ? -turn : turn, alpha));
}

bool App::courseFromMapping() const {
    return !demoMode_ && !mappingMarkers_.empty() && (courseMode_ == 2 || (courseMode_ == 0 && !ros_->truthActive()));
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
        host::sortMappingMarkers(mappingMarkers_);
        const auto course = cfg["course"].as<std::string>("auto");
        courseMode_ = course == "pack" ? 1 : course == "mapping" ? 2 : 0;
        mappingGhost_ = cfg["ghost"].as<bool>(false);
        for (const auto &name : host::hideMappingMarkers(
                 mappingMarkers_, cfg["hidden"].as<std::vector<std::string>>(std::vector<std::string>{})))
            std::cerr << "nereus-viewer: mapping_markers.hidden: no marker frame, label or mesh '" << name << "'\n";
    } catch (const std::exception &error) {
        std::cerr << "nereus-viewer: mapping course disabled: " << error.what() << '\n';
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
    state.showEquipment = look_.equipment;
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
            if (!marker.visible || !ros_->latestInFixed(marker.frame, frame))
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
        const auto texel = depthTexel(uv, frame.depth_width, frame.depth_height);
        if (glCheckFramebufferStatus(GL_READ_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE)
            glReadPixels(texel.x, texel.y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
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
    const ImVec4 fill = tintedFill(tint);
    ImGui::PushStyleColor(ImGuiCol_Button, fill);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, fill);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, fill);
    ImGui::PushStyleColor(ImGuiCol_Text, tint);
    ImGui::Button(text.c_str());
    ImGui::PopStyleColor(4);
}
void App::sectionHeading(const char *text) {
    ImGui::PushFont(window_->small);
    ImGui::TextColored(palette().muted, "%s", text);
    ImGui::PopFont();
}
std::string App::runTime() const {
    const double seconds = runScore_ && runScore_["elapsed"] ? runScore_["elapsed"].as<double>() : 0.;
    char value[64];
    std::snprintf(value, sizeof(value), "%02d:%04.1f", int(seconds) / 60, std::fmod(seconds, 60.));
    return value;
}

// One camera window's contents: source controls, the image fitted to the window, and the feed status.
void App::drawCameraCard(std::size_t index) {
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
    // Local rendering needs the simulator's truth pose; without it the card is the ROS image topic.
    const bool canLocal = demoMode_ || ros_->truthActive();
    const bool depthShown = feed.wantDepth && tex.depth;
    const bool local = demoMode_ || (canLocal && !feed.rosMode);
    // RGB / Depth as a two-way switch, then "Main view" to show this camera in the pool view.
    for (const bool depth : {false, true}) {
        if (depth)
            ImGui::SameLine(0, 1);
        pushActiveColors(feed.wantDepth == depth);
        if (pins::Button(depth ? "Depth" : "RGB") && feed.wantDepth != depth) {
            feed.wantDepth = depth;
            ros_->refreshCameras();
        }
        popActiveColors();
    }
    ImGui::SameLine();
    const int cameraMode = int(index) + 2;
    pushActiveColors(mode_ == cameraMode);
    if (pins::Button("Main view"))
        setViewMode(mode_ == cameraMode ? 0 : cameraMode);
    popActiveColors();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(mode_ == cameraMode ? "Back to the orbit camera"
                                              : "Look through this camera in the pool view");
    ImGui::PushFont(window_->small);
    const std::string model =
        camera.model + "  /  " + std::to_string(camera.k.width) + " x " + std::to_string(camera.k.height);
    const float modelWidth = ImGui::CalcTextSize(model.c_str()).x;
    if (ImGui::GetContentRegionAvail().x > modelWidth + ui(120)) {
        ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - modelWidth); // takes the buttons' text baseline
        ImGui::TextColored(palette().muted, "%s", model.c_str());
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Preview: %d x %d%s", depthShown ? tex.depthWidth : tex.rgbWidth,
                              depthShown ? tex.depthHeight : tex.rgbHeight,
                              !demoMode_ && !canLocal ? "\nROS image topic (no simulator truth pose)" : "");
    }
    const float footer = ImGui::GetFontSize() + ImGui::GetStyle().ItemSpacing.y + 2;
    ImGui::PopFont();
    const auto available = ImGui::GetContentRegionAvail();
    const float aspect = float(camera.k.width) / float(camera.k.height);
    const float w = std::max(16.f, std::min(available.x, (available.y - footer) * aspect));
    const float h = w / aspect;
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (available.x - w) / 2);
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    const GLuint texture = depthShown ? tex.depth : tex.rgb;
    if (texture) {
        const bool flip = !depthShown && tex.flipped;
        ImGui::Image(textureID(texture), {w, h}, flip ? ImVec2(0, 1) : ImVec2(0, 0),
                     flip ? ImVec2(1, 0) : ImVec2(1, 1));
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
                      !ready                 ? "Awaiting vehicle pose"
                      : local && !depthShown ? "Rendering local view"
                                             : "Awaiting camera image");
    }
    ImGui::PushFont(window_->small);
    const bool connected = !demoMode_ && feed.connected();
    const bool depthFromRos = feed.wantDepth && !demoMode_;
    ImGui::TextColored(local && !depthFromRos ? palette().accent
                       : connected            ? palette().accent
                                              : palette().muted,
                       "%s",
                       demoMode_                ? "PREVIEW ONLY"
                       : local && !depthFromRos ? "LOCAL VIEW"
                       : connected              ? "CONNECTED"
                                                : "NO SENSOR OUTPUT");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("%s", demoMode_ ? "Scene preview. Start the simulator bridge to stream live cameras."
                                          : "Images are rendered by the bridge at the physics pose. Observer "
                                            "controls do not move the vehicle.");
    ImGui::SameLine();
    const bool sourceToggle = canLocal && !demoMode_;
    if (sourceToggle) {
        // Simulator: the source label toggles the card between this viewer's truth-pose render and
        // the images the bridge publishes to the robot stack.
        const bool stack = feed.rosMode;
        pushActiveColors(stack);
        if (ImGui::SmallButton(stack ? "ROS (stack)###source" : "truth pose###source")) {
            feed.rosMode = !stack;
            ros_->refreshCameras();
            cardDue_[index] = 0; // render the local view immediately when switching back
        }
        popActiveColors();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(
                "Click to switch the card's source.\n"
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
    ImGui::PopID();
}

void App::drawWaterControls() {
    auto edited = look_.appearance.water;
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ui(ImVec2(10, 3)));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ui(ImVec2(8, 3)));
    ImGui::SetNextItemWidth(ui(240));
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
    ImGui::PushStyleColor(ImGuiCol_Text, palette().muted);
    ImGui::TextWrapped("Tint and haze accumulate along the underwater sightline. These settings change only this "
                       "viewer's rendering; the robot cameras are rendered by the bridge from the pool pack. "
                       "Exponent 1 / clear distance 0: exponential attenuation.");
    ImGui::PopStyleColor();
    look_.appearance.water = edited;
    ImGui::PopStyleVar(2);
}

// Scroll zooms, drag pans, clicking a task focuses the pool view on it. Compact (a small window): smaller labels,
// the ui document's minimap labels, and landmarks marked hidden_in_minimap unlabelled.
void App::drawCourseMap(float width, float height, bool compact) {
    const auto &s = *scenario_;
    const ImVec2 a = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("course canvas", {width, height});
    const bool hovered = ImGui::IsItemHovered();
    auto &io = ImGui::GetIO();
    if (hovered) {
        mapZoom_ = glm::clamp(mapZoom_ * std::exp(io.MouseWheel * .15f), 1.f, 8.f);
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            mapPan_.x += io.MouseDelta.x;
            mapPan_.y += io.MouseDelta.y;
        }
    }
    const float length = s.poolLength, poolWidth = s.poolWidth;
    const float scale = std::min((width - 36) / length, (height - 36) / poolWidth) * mapZoom_;
    const ImVec2 center(a.x + width / 2 + mapPan_.x, a.y + height / 2 + mapPan_.y);
    auto poolXY = [&](glm::vec2 p) {
        return ImVec2(center.x + (p.x - length / 2) * scale, center.y - (p.y - poolWidth / 2) * scale);
    };
    auto xy = [&](glm::vec3 p) { return poolXY(glm::vec2(s.worldToPool * glm::vec4(p, 1))); };
    auto *d = ImGui::GetWindowDrawList();
    d->AddRectFilled(a, {a.x + width, a.y + height}, IM_COL32(9, 24, 32, 255), 5);
    d->PushClipRect(a, {a.x + width, a.y + height}, true);
    d->AddRectFilled(poolXY({0, poolWidth}), poolXY({length, 0}), IM_COL32(13, 40, 50, 255));
    // The pool's floor markings when it declares any, else a 5 m grid for scale.
    bool marked = false;
    if (model_)
        for (const auto &stripe : model_->pack().poolStripes())
            if (stripe.side == rendering::PoolSide::Floor) {
                d->AddLine(poolXY({stripe.from.x(), stripe.from.y()}), poolXY({stripe.to.x(), stripe.to.y()}),
                           IM_COL32(48, 88, 100, 255), std::max(1.f, stripe.width * scale));
                marked = true;
            }
    if (!marked) {
        for (int i = 0; i <= length; i += 5)
            d->AddLine(poolXY({float(i), 0}), poolXY({float(i), poolWidth}), IM_COL32(35, 64, 74, 255));
        for (int i = 0; i <= poolWidth; i += 5)
            d->AddLine(poolXY({0, float(i)}), poolXY({length, float(i)}), IM_COL32(35, 64, 74, 255));
    }
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
        const float font = compact ? ui(12) : ui(16);
        const ImVec2 textAt(p.x + 7, p.y + (index++ % 2 ? -19 : 4));
        d->AddCircleFilled(p, compact ? ui(4) : ui(5), color(cyan));
        if (!compact || !hiddenInMinimap)
            d->AddText(compact ? window_->small : window_->normal, font, textAt, color(white),
                       (compact && relabelMinimap ? entry["minimap_label"].as<std::string>() : key).c_str());
        (void)label;
        if (hovered && ImGui::IsMouseReleased(ImGuiMouseButton_Left) &&
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
    d->AddText(window_->small, window_->small->FontSize, {p.x + ui(8), p.y - ui(15)}, IM_COL32(255, 208, 96, 255),
               s.robotId.c_str());
    d->PopClipRect();
}

void App::drawMapWindow() {
    if (!mapOpen_)
        return;
    if (focusMap_) {
        ImGui::SetNextWindowFocus();
        focusMap_ = false;
    }
    focusIfRequested(kCourseMap);
    ImGui::SetNextWindowSize(ui(ImVec2(520, 340)), ImGuiCond_FirstUseEver);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ui(ImVec2(10, 8)));
    const bool visible = ImGui::Begin(kCourseMap, &mapOpen_);
    ImGui::PopStyleVar();
    if (visible) {
        windowContextMenu("map");
        ImGui::PushFont(window_->small);
        ImGui::AlignTextToFramePadding();
        // The hint as far as it fits beside Fit pool.
        const float room = ImGui::GetContentRegionAvail().x - buttonWidth("Fit pool") - ImGui::GetStyle().ItemSpacing.x;
        for (const char *hint : {"Scroll zoom  /  drag pan  /  click a task to focus", "Scroll zoom  /  drag pan", ""})
            if (ImGui::CalcTextSize(hint).x <= room) {
                ImGui::TextDisabled("%s", hint);
                break;
            }
        const bool fitted = mapZoom_ == 1 && mapPan_.x == 0 && mapPan_.y == 0;
        ImGui::SameLine(
            std::max(ImGui::GetCursorPosX(), ImGui::GetWindowContentRegionMax().x - buttonWidth("Fit pool")));
        ImGui::BeginDisabled(fitted);
        if (ImGui::SmallButton("Fit pool")) {
            mapZoom_ = 1;
            mapPan_ = {0, 0};
        }
        ImGui::EndDisabled();
        ImGui::PopFont();
        const auto space = ImGui::GetContentRegionAvail();
        drawCourseMap(space.x, std::max(80.f, space.y), space.x < ui(520) || space.y < ui(320));
    }
    ImGui::End();
}

void App::drawSceneSettings() {
    const float width = ImGui::GetContentRegionAvail().x;
    for (const auto &control : scenario_->ui["mechanism_controls"]) {
        ImGui::BeginDisabled(demoMode_ || !ros_->truthActive()); // simulator-only commands
        if (pins::Button(control["label"].as<std::string>().c_str()))
            ros_->publishMechanism(control["topic"].as<std::string>(), control["value"].as<bool>(true));
        ImGui::EndDisabled();
    }
    if (ImGui::BeginTabBar("Environment tabs")) {
        if (ImGui::BeginTabItem("Lighting")) {
            sectionHeading("UNDERWATER OPTICS");
            auto &a = look_.appearance;
            ImGui::SetNextItemWidth(width * .17f);
            ImGui::SliderFloat("Caustics", &a.caustics, 0, 1, "%.2f");
            ImGui::SameLine();
            pins::Checkbox("Surface", &a.surface);
            ImGui::SameLine();
            pins::Checkbox("Shadows", &a.shadows);
            ImGui::Separator();
            int profile = a.outdoor ? 1 : 0;
            ImGui::SetNextItemWidth(ui(120));
            if (pins::Combo("Lighting", &profile, "Indoor\0Outdoor\0")) {
                a.outdoor = profile == 1;
                a.direct_light = a.outdoor ? 1.4f : 1.f;
                a.ambient_light = a.outdoor ? .6f : .9f;
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(ui(130));
            ImGui::SliderFloat("Brightness", &a.direct_light, 0, 4, "%.2f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(ui(125));
            ImGui::SliderFloat("Ambient", &a.ambient_light, 0, 2, "%.2f");
            if (a.outdoor) {
                ImGui::SetNextItemWidth(ui(160));
                ImGui::SliderFloat("Sun azimuth", &a.sun_azimuth, 0, 360, "%.0f deg");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(ui(140));
                ImGui::SliderFloat("Elevation", &a.sun_elevation, 5, 89, "%.0f deg");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(ui(120));
                ImGui::SliderFloat("Glare", &a.glare, 0, 2, "%.2f");
            } else
                ImGui::TextDisabled("Diffuse indoor lighting. Switch to Outdoor to adjust sun and glare.");
            ImGui::PushStyleColor(ImGuiCol_Text, palette().muted);
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
    windowToggle("Scene", &sceneOpen_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Scene settings: mechanism buttons, lighting and water appearance");
    sceneAnchor_ = {ImGui::GetItemRectMin().x, ImGui::GetItemRectMax().y + ImGui::GetStyle().ItemSpacing.y};
}

void App::drawSceneSettingsWindow() {
    if (sceneOpen_) {
        focusIfRequested(kSceneSettings);
        if (beginToolWindow(kSceneSettings, &sceneOpen_, ui(ImVec2(780, 400)), sceneAnchor_)) {
            windowContextMenu("scene_settings");
            pins::beginScope("scene_settings", "Scene settings");
            drawSceneSettings();
            pins::endScope();
        }
        ImGui::End();
    }
    if (pins::needsDrawing("scene_settings"))
        pins::drawOffscreen("scene_settings", "Scene settings", [this] { drawSceneSettings(); });
}

void App::toolbarPoolViewer() {
    windowToggle("Display", &displayOpen_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("What this view shows: pool, course source, estimate ghost, viewer lighting");
    displayAnchor_ = {ImGui::GetItemRectMin().x, ImGui::GetItemRectMax().y + ImGui::GetStyle().ItemSpacing.y};
}

void App::drawDisplayWindow() {
    if (displayOpen_) {
        focusIfRequested(kDisplay);
        if (beginToolWindow(kDisplay, &displayOpen_, ui(ImVec2(340, 0)), displayAnchor_)) {
            windowContextMenu("display");
            pins::beginScope("display", "Display");
            drawDisplaySettings();
            pins::endScope();
        }
        ImGui::End();
    }
    if (pins::needsDrawing("display"))
        pins::drawOffscreen("display", "Display", [this] { drawDisplaySettings(); });
}

// Observer visibility and lighting: what this view draws, never what the robot cameras see.
void App::drawDisplaySettings() {
    pins::Checkbox("Water", &observer_.water);
    pins::Checkbox("Pool walls & deck", &observer_.walls);
    pins::Checkbox("Pool floor", &observer_.floor);
    if (!model_->pack().equipmentInstances().empty()) {
        pins::Checkbox("AprilTag board", &look_.equipment);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("The equipment pack (calibration board) in this view; camera cards always show it.");
    }
    if (!mappingMarkers_.empty() && !demoMode_) {
        ImGui::SeparatorText("Course");
        ImGui::SetNextItemWidth(ui(170));
        pins::Combo("Source", &courseMode_, "Auto\0Pack layout\0Mapping (RViz)\0");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Auto: the pack layout with simulator truth, the mapping estimate otherwise.\n"
                              "Mapping: riptide_meshes at the mapping TF frames, as RViz shows them.");
        if (ros_->truthActive()) {
            ImGui::BeginDisabled(courseFromMapping());
            pins::Checkbox("Mapping ghost", &mappingGhost_);
            ImGui::EndDisabled();
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                ImGui::SetTooltip("Draw the mapping estimate translucent over the simulator course.");
        }
        drawMappingMeshList();
    }
    if (ros_->truthActive() && !demoMode_) { // simulator only: a real robot has the estimate alone
        ImGui::SeparatorText("Localization estimate");
        pins::Checkbox("Robot ghost", &robotGhost_);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Draw the robot translucent at the localization estimate (TF base_link).");
        int gizmo = gizmoOnEstimate_ ? 1 : 0, follow = followEstimate_ ? 1 : 0;
        ImGui::SetNextItemWidth(ui(170));
        if (pins::Combo("Control gizmo", &gizmo, "Truth (sim)\0Estimate (TF)\0"))
            gizmoOnEstimate_ = gizmo == 1;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Where the drag axes and rings are drawn. Commands always go to the controller in its\n"
                              "(estimate) frame; on Truth they are shown re-rooted at the sim robot.");
        ImGui::SetNextItemWidth(ui(170));
        if (pins::Combo("Follow", &follow, "Truth (sim)\0Estimate (TF)\0"))
            followEstimate_ = follow == 1;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Robot the Follow camera tracks (the estimate with its jitter smoothed).");
    }
    pins::Checkbox("Surface reflections", &observer_.reflections);
    pins::Checkbox("Frame stats (F3)", &showProfile_);
    ImGui::SeparatorText("Viewer lighting");
    int antialiasing = std::clamp(observer_.antialiasing, 1, 4) - 1;
    ImGui::SetNextItemWidth(ui(160));
    if (pins::Combo("Anti-aliasing", &antialiasing,
                    "Off\0"
                    "2x\0"
                    "3x\0"
                    "4x\0"))
        observer_.antialiasing = antialiasing + 1;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Supersampling of this view and the camera cards: each pixel averages n x n samples.\n"
                          "Costs about n^2 in GPU time and memory; never changes the simulated camera images.");
    ImGui::BeginDisabled(observer_.lighting == 3);
    pins::Checkbox("Shadows", &observer_.shadows);
    ImGui::EndDisabled();
    ImGui::SetNextItemWidth(ui(160));
    pins::Combo("Lighting", &observer_.lighting, "Scene lighting\0Indoor\0Outdoor\0Sterile\0");
    ImGui::SetNextItemWidth(ui(160));
    ImGui::SliderFloat("Exposure", &observer_.exposure, .4f, 2.f, "%.2fx");
    ImGui::BeginDisabled(observer_.lighting == 3);
    ImGui::SetNextItemWidth(ui(160));
    ImGui::SliderFloat("Brightness", &observer_.brightness, 0.f, 4.f, "%.2fx");
    ImGui::EndDisabled();
    ImGui::SetNextItemWidth(ui(160));
    ImGui::SliderFloat("Ambient", &observer_.ambient, 0.f, 3.f, "%.2fx");
    if (pins::Button("Reset lighting", {-1, ui(30)}))
        observer_.resetLighting();
}

void App::setViewMode(int mode) {
    if (mode == 1 && mode_ != 1) {
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
    mode_ = mode;
}

void App::toolbarView() {
    const auto &s = *scenario_;
    sameLineIfFits(ui(130));
    ImGui::SetNextItemWidth(ui(130));
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
    int mode = mode_;
    if (ImGui::Combo("##view", &mode, views.c_str()))
        setViewMode(mode);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Camera: orbit, free camera (WASD), or look through a robot camera");
}

void App::toolbarFocus() {
    sameLineIfFits(toolbarLeft_ > 640 ? 150 : 120);
    ImGui::SetNextItemWidth(toolbarLeft_ > 640 ? 150 : 120);
    std::string focuses;
    for (const auto &name : focusNames_) {
        focuses += name;
        focuses += '\0';
    }
    focuses += '\0';
    if (ImGui::Combo("##focus", &selectedFocus_, focuses.c_str()))
        focus(focusNames_.at(std::size_t(selectedFocus_)));
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Focus: jump the camera to the vehicle, a mechanism or a course element");
}

void App::toolbarFollow() {
    if (!presetFor(focusName_).follow)
        follow_ = false;
    ImGui::BeginDisabled(!presetFor(focusName_).follow);
    toggleChip("Follow", &follow_);
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Keep the camera on the focus target as it moves (panning detaches it)");
}

void App::toolbarLabels() {
    toggleChip("Labels", &labels_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Name the course elements in the view");
}

void App::toolbarTf() {
    if (toggleChipWithMenu("TF", &showTf_, "tf_settings"))
        tfOpen_ = !tfOpen_;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("TF frames: names, axis length and the frame tree");
    tfAnchor_ = {ImGui::GetItemRectMin().x, ImGui::GetItemRectMax().y + ImGui::GetStyle().ItemSpacing.y};
}

void App::drawTfWindow() {
    tfTreeOpen_ = false;
    const auto toggles = [this] {
        pins::Checkbox("Show TF frames", &showTf_);
        pins::Checkbox("Frame names", &tfNames_);
    };
    if (tfOpen_) {
        focusIfRequested(kTfFrames);
        if (beginToolWindow(kTfFrames, &tfOpen_, ui(ImVec2(420, 520)), tfAnchor_)) {
            windowContextMenu("tf");
            tfTreeOpen_ = true;
            pins::beginScope("tf", "TF frames");
            toggles();
            pins::endScope();
            ImGui::SetNextItemWidth(ui(220));
            ImGui::SliderFloat("Axis length", &tfAxisLength_, .02f, 1.f, "%.2f m");
            ImGui::TextUnformatted("X: red   Y: green   Z: blue");
            drawTfTree(tfTree_, scenario_->mapFrame);
            ImGui::TextDisabled("Axes show through objects. Unavailable frames cannot reach the fixed frame.");
            ImGui::TextDisabled("%s", demoMode_ ? "Preview: robot-pack frames at the preview pose."
                                                : "Raw ROS TF in the fixed frame, including localization drift.");
        }
        ImGui::End();
    }
    if (pins::needsDrawing("tf"))
        pins::drawOffscreen("tf", "TF frames", toggles);
}

void App::drawDetectionSettings(bool includeEnable) {
    if (includeEnable) {
        pins::Checkbox("Show detections", &detections_);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Detector markers (%s), placed once when observed and then fixed in the world.\n"
                              "Truth: simulator pose at image capture. Estimate: TF at the image stamp.",
                              ros_->detectionTopic.c_str());
    }
    if (ros_->truthPlacementAvailable()) {
        // Truth/both only exist with a simulator; a real robot has the estimate alone.
        ImGui::BeginDisabled(!detections_);
        ImGui::SetNextItemWidth(ui(130));
        int placement = int(ros_->detectionMode());
        if (pins::Combo("Placement", &placement, "Pose source\0Truth\0Estimate\0Both\0"))
            ros_->setDetectionMode(DetectionMode(placement));
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip("Detection placement: follow the pose source, simulator truth, localization\n"
                              "estimate (RViz-like TF), or both (truth solid, estimate cyan outline).");
    }
    bool keep = !ros_->honorDeleteAll();
    if (pins::Checkbox("Keep detections (ignore DELETEALL)", &keep))
        ros_->setHonorDeleteAll(!keep);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("The detector clears its markers every frame; keeping them lets each observation live out\n"
                          "its lifetime.");
    drawPointCloudSettings();
}

// Per-mesh visibility of the mapping course (and ghost), folded by default: the stack lists ~25 markers.
void App::drawMappingMeshList() {
    const auto shown = std::count_if(mappingMarkers_.begin(), mappingMarkers_.end(),
                                     [](const MappingMarker &marker) { return marker.visible; });
    const std::string title =
        "Meshes (" + std::to_string(shown) + "/" + std::to_string(mappingMarkers_.size()) + " shown)###mapping_meshes";
    if (!ImGui::TreeNode(title.c_str()))
        return;
    ImGui::SetNextItemWidth(ui(260));
    ImGui::InputTextWithHint("##mesh_filter", "Search label, mesh or frame", meshFilter_, sizeof(meshFilter_));
    const bool filtering = meshFilter_[0] != 0;
    // While filtering, the buttons act on the matches only.
    const auto setAll = [this](bool visible) {
        for (auto &marker : mappingMarkers_)
            if (mappingMarkerMatches(marker, meshFilter_))
                marker.visible = visible;
    };
    if (ImGui::SmallButton(filtering ? "Show matches" : "Show all"))
        setAll(true);
    ImGui::SameLine();
    if (ImGui::SmallButton(filtering ? "Hide matches" : "Hide all"))
        setAll(false);
    const float rows = std::min<float>(float(mappingMarkers_.size()), 10.5f);
    ImGui::BeginChild("mapping_mesh_list", {260, rows * ImGui::GetFrameHeightWithSpacing()}, ImGuiChildFlags_Borders);
    bool any = false;
    for (auto &marker : mappingMarkers_) {
        if (!mappingMarkerMatches(marker, meshFilter_))
            continue;
        any = true;
        ImGui::PushID(marker.name.c_str());
        pins::Checkbox(marker.label.c_str(), &marker.visible);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Mesh %s at TF frame %s", marker.mesh.c_str(), marker.frame.c_str());
        ImGui::PopID();
    }
    if (!any)
        ImGui::TextDisabled("No mesh matches \"%s\"", meshFilter_);
    ImGui::EndChild();
    ImGui::TreePop();
}

// Point cloud layers from the host config: one toggle each (subscribes only while on) and a point size.
void App::drawPointCloudSettings() {
    if (ros_->pointClouds.empty() || demoMode_)
        return;
    ImGui::SeparatorText("Point clouds");
    for (auto &layer : ros_->pointClouds) {
        ImGui::PushID(layer.id.c_str());
        pins::Checkbox(layer.title.c_str(), &layer.enabled);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s\nNewest message only, placed once in the fixed frame%s.", layer.topic.c_str(),
                              ros_->truthActive() ? " (camera clouds at the simulator truth pose)" : "");
        if (layer.enabled) {
            ImGui::SameLine();
            ImGui::SetNextItemWidth(ui(80));
            ImGui::SliderFloat("##size", &layer.size, 1, 8, "%.0f px");
            ImGui::SameLine();
            const bool stale = layer.data && std::chrono::duration<double>(Clock::now() - layer.received).count() > 2;
            ImGui::TextDisabled("%s", !layer.data         ? "waiting"
                                      : !layer.placed     ? "no transform"
                                      : stale             ? "stale"
                                      : layer.approximate ? "approx"
                                                          : "");
        }
        ImGui::PopID();
    }
}

// Sidebar form: full settings plus the legend. Toolbar form (below): the checkbox and a dropdown of the rest.
void App::toolbarDetections() {
    if (demoMode_)
        return;
    if (toggleChipWithMenu("Detections", &detections_, "detection_options"))
        ImGui::OpenPopup("detection_options");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Detector markers (%s), placed once when observed and then fixed in the world.\n"
                          "Truth: simulator pose at image capture. Estimate: TF at the image stamp.",
                          ros_->detectionTopic.c_str());
    if (ImGui::BeginPopup("detection_options")) {
        drawDetectionSettings(false);
        ImGui::EndPopup();
    }
}

void App::toolbarMpcPath() {
    if (demoMode_)
        return;
    toggleChip("MPC path", &showMpc_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Predicted MPC trajectory over its horizon (%s, orange).\n"
                          "Drawn relative to the simulator vehicle, so localization drift does not offset it.\n"
                          "Blue: the whole follow_path plan with its headings (%s), placed through TF.",
                          ros_->mpcTopic.c_str(), ros_->plannedTopic.c_str());
}

void App::toolbarThrust() {
    if (demoMode_ || scenario_->thrusterMounts.empty())
        return;
    toggleChip("Thrust", &showThrust_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Commanded thruster forces (%s) as arrows from each thruster along its axis,\n"
                          "%.2f m per newton; nothing is drawn while the controller is not publishing.",
                          ros_->thrustTopic.c_str(), thrustScale_);
}

void App::toolbarPreviewTask() {
    if (demoMode_ && !demoNames_.empty()) {
        sameLineIfFits(ui(170));
        ImGui::SetNextItemWidth(ui(170));
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
    for (const auto &entry : windowEntries())
        if (pins_[entry.key])
            pinnedWindowButton(entry);
    pins::drawPinned(); // controls pinned from the panels
    // Customize: the trailing button or a right-click anywhere on the toolbar.
    sameLineIfFits(ImGui::GetFrameHeight());
    if (ImGui::Button("+", {ImGui::GetFrameHeight(), 0}))
        ImGui::OpenPopup("toolbar_customize");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Customize the toolbar (or right-click it): show or hide its buttons, pin windows");
    if (ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows) && ImGui::IsMouseReleased(ImGuiMouseButton_Right) &&
        !ImGui::IsPopupOpen(nullptr, ImGuiPopupFlags_AnyPopupId)) // a pinned control's own menu takes its click
        ImGui::OpenPopup("toolbar_customize");
    if (ImGui::BeginPopup("toolbar_customize")) {
        drawToolbarCustomization();
        ImGui::EndPopup();
    }
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
    add("thrust", [this] { toolbarThrust(); });
    add("preview_task", [this] { toolbarPreviewTask(); });
    add(
        "detections", [this] { toolbarDetections(); },
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

// The frame: menu bar and command bar (fixed), then the dock space holding the pool view and every window.
void App::drawInterface(double time, float dt) {
    pins::newFrame();
    handleShortcuts();
    drawMenuBar();
    drawCommandBar();
    auto *viewport = ImGui::GetMainViewport();
    if (!pendingPreset_.empty() && scenario_) {
        applyPreset(pendingPreset_);
        pendingPreset_.clear();
    }
    ImGuiDockNodeFlags dockFlags = ImGuiDockNodeFlags_NoCloseButton; // each tab has its own close box
    if (layoutLocked_)
        dockFlags |= ImGuiDockNodeFlags_NoUndocking | ImGuiDockNodeFlags_NoResize | ImGuiDockNodeFlags_NoDocking;
    ImGui::DockSpaceOverViewport(dockspace_, viewport, dockFlags);
    updateSides();
    if (scenario_ && runTracking_)
        runScore_.reset(runTracking_->state().score);
    drawPoolView(time, dt);
    if (scenario_) {
        drawCameraWindows();
        drawMapWindow();
        drawSceneSettingsWindow();
        drawDisplayWindow();
        drawTfWindow();
        if (composition_) {
            composition_->drawPanels();
            composition_->drawWindows();
        }
    }
    drawHelpWindow();
    pins::drawMenu();
    handleWindowEdges();
    windowStates_.update();
}

void App::drawMenuBar() {
    // 35 px at the desktop's scale whatever the interface scale, like other applications' title bars (VS Code's):
    // the padding sets the bar's height and centres the menus; font, spacing and the dropdowns at that scale too.
    const float t = window_->contentScale();
    ImGui::PushFont(window_->menu);
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(10 * t, 9 * t));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(14 * t, 12 * t));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemInnerSpacing, ImVec2(4 * t, 4 * t));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(10 * t, (35 * t - ImGui::GetFontSize()) * .5f));
    const bool open = ImGui::BeginMainMenuBar();
    ImGui::PopStyleVar();
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(10 * t, 7 * t)); // inside the dropdowns
    if (!open) {
        ImGui::PopStyleVar(4);
        ImGui::PopFont();
        return;
    }
    if (ImGui::BeginMenu("View")) {
        drawViewMenu();
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Windows")) {
        drawWindowsMenu();
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Layout")) {
        drawLayoutMenu();
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Help")) {
        ImGui::MenuItem("Controls & shortcuts", "F1", &helpOpen_);
        ImGui::EndMenu();
    }
    const char *state = maximized_ ? "Pool view maximized (Ctrl+Space restores)" : layoutLocked_ ? "Layout locked" : "";
    if (*state) {
        ImGui::SameLine(0, 24);
        ImGui::TextDisabled("%s", state);
    }
    if (window_->customTitleBar())
        drawWindowControls();
    ImGui::EndMainMenuBar();
    ImGui::PopStyleVar(4);
    ImGui::PopFont();
}

// The viewer's own title bar (no system decorations): the menu bar's empty space moves the window (drag) or
// maximizes it (double-click), with minimize / maximize / close at its right end.
void App::drawWindowControls() {
    const bool empty = ImGui::IsWindowHovered() && !ImGui::IsAnyItemHovered() && !ImGui::IsAnyItemActive();
    if (empty && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
        titleDrag_ = false;
        window_->toggleMaximized();
    } else if (empty && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
        titleDrag_ = true;
    if (titleDrag_ && ImGui::IsMouseDragging(ImGuiMouseButton_Left, 3)) {
        titleDrag_ = false;
        window_->beginMove(); // the window manager moves it (snapping, tiling) until the button is released
    }
    if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
        titleDrag_ = false;
    const float t = window_->contentScale(), buttonWidth = 46 * t, height = ImGui::GetWindowHeight();
    const ImVec2 origin(ImGui::GetWindowPos().x + ImGui::GetWindowWidth() - 3 * buttonWidth, ImGui::GetWindowPos().y);
    auto *draw = ImGui::GetWindowDrawList();
    const auto &p = palette();
    for (int i = 0; i < 3; ++i) {
        const ImVec2 min(origin.x + i * buttonWidth, origin.y), max(min.x + buttonWidth, min.y + height);
        ImGui::SetCursorScreenPos(min);
        ImGui::PushID(i);
        const bool clicked = ImGui::InvisibleButton("##window_button", {buttonWidth, height});
        ImGui::PopID();
        const bool hovered = ImGui::IsItemHovered(), held = ImGui::IsItemActive();
        const bool close = i == 2;
        if (hovered || held)
            draw->AddRectFilled(
                min, max,
                ImGui::GetColorU32(
                    close ? (held ? p.dangerPressed : p.dangerHovered)
                          : ImGui::GetStyle().Colors[held ? ImGuiCol_ButtonActive : ImGuiCol_ButtonHovered]));
        const auto ink = ImGui::GetColorU32(close && (hovered || held) ? p.dangerText : p.text);
        const ImVec2 c((min.x + max.x) * .5f, (min.y + max.y) * .5f);
        if (i == 0)
            draw->AddLine({c.x - 5 * t, c.y + .5f * t}, {c.x + 5 * t, c.y + .5f * t}, ink, 1.2f * t);
        else if (i == 1 && window_->maximized()) { // restore: two overlapping squares
            draw->AddRect({c.x - 5 * t, c.y - 3 * t}, {c.x + 3 * t, c.y + 5 * t}, ink, 0, 0, 1.2f * t);
            draw->AddLine({c.x - 3 * t, c.y - 5 * t}, {c.x + 5 * t, c.y - 5 * t}, ink, 1.2f * t);
            draw->AddLine({c.x + 5 * t, c.y - 5 * t}, {c.x + 5 * t, c.y + 3 * t}, ink, 1.2f * t);
        } else if (i == 1)
            draw->AddRect({c.x - 5 * t, c.y - 5 * t}, {c.x + 5 * t, c.y + 5 * t}, ink, 0, 0, 1.2f * t);
        else {
            draw->AddLine({c.x - 5 * t, c.y - 5 * t}, {c.x + 5 * t, c.y + 5 * t}, ink, 1.2f * t);
            draw->AddLine({c.x - 5 * t, c.y + 5 * t}, {c.x + 5 * t, c.y - 5 * t}, ink, 1.2f * t);
        }
        if (hovered)
            ImGui::SetTooltip("%s", i == 0   ? "Minimize"
                                    : i == 1 ? (window_->maximized() ? "Restore" : "Maximize")
                                             : "Close");
        if (clicked) {
            if (i == 0)
                window_->minimize();
            else if (i == 1)
                window_->toggleMaximized();
            else
                window_->requestClose();
        }
    }
}

// Without system decorations the window's own edges resize it (through the window manager), as borders would.
void App::handleWindowEdges() {
    const auto &io = ImGui::GetIO();
    if (!window_->customTitleBar() || window_->maximized() || !ImGui::IsMousePosValid() || ImGui::IsAnyItemActive())
        return;
    const auto m = io.MousePos;
    const float w = io.DisplaySize.x, h = io.DisplaySize.y, border = 5 * window_->contentScale();
    if (m.x < 0 || m.y < 0 || m.x >= w || m.y >= h)
        return;
    const bool left = m.x < border, right = m.x >= w - border, top = m.y < border, bottom = m.y >= h - border;
    const int edge = top && left       ? 0
                     : top && right    ? 2
                     : bottom && right ? 4
                     : bottom && left  ? 6
                     : top             ? 1
                     : right           ? 3
                     : bottom          ? 5
                     : left            ? 7
                                       : -1;
    if (edge < 0)
        return;
    ImGui::SetMouseCursor(edge == 1 || edge == 5   ? ImGuiMouseCursor_ResizeNS
                          : edge == 3 || edge == 7 ? ImGuiMouseCursor_ResizeEW
                          : edge == 0 || edge == 4 ? ImGuiMouseCursor_ResizeNWSE
                                                   : ImGuiMouseCursor_ResizeNESW);
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
        window_->beginResize(edge);
}

// A theme for the whole interface; the operator's choice is remembered (viewer.yaml in the config directory).
void App::setTheme(const std::string &id) {
    const auto before = currentThemeInfo();
    if (!applyTheme(id))
        return;
    savePreference("theme", YAML::Node(id));
    const auto &after = currentThemeInfo();
    if (after.fontFamily != before.fontFamily || after.fontPoints != before.fontPoints)
        pendingUiScale_ = resolvedUiScale(); // reload the fonts between frames
}

// The operator's interface preferences (theme, scale) in viewer.yaml in the config directory.
void App::savePreference(const char *key, const YAML::Node &value) {
    if (!persist_ || preferencesFile_.empty())
        return;
    try {
        YAML::Node preferences(YAML::NodeType::Map);
        if (fs::exists(preferencesFile_))
            preferences = YAML::LoadFile(preferencesFile_.string());
        preferences[key] = value;
        fs::create_directories(preferencesFile_.parent_path());
        std::ofstream(preferencesFile_) << preferences << '\n';
    } catch (const std::exception &error) {
        std::cerr << "nereus-viewer: " << key << " not saved: " << error.what() << '\n';
    }
}

float App::resolvedUiScale() const {
    return uiScaleSetting_ > 0 ? uiScaleSetting_ : window_->contentScale();
}

void App::setUiScale(float setting) {
    uiScaleSetting_ = setting;
    pendingUiScale_ = resolvedUiScale();
    savePreference("interface_scale", setting > 0 ? YAML::Node(setting) : YAML::Node("auto"));
}

void App::applyPendingUiScale() {
    if (pendingUiScale_ <= 0)
        return;
    const auto &theme = currentThemeInfo(); // a theme with its own font (the desktop's Qt font)
    window_->loadFonts(pendingUiScale_, window_->contentScale(), theme.fontFamily, theme.fontPoints);
    setInterfaceScale(pendingUiScale_);
    pendingUiScale_ = -1;
}

void App::drawScaleMenu() {
    if (!ImGui::BeginMenu("Interface scale"))
        return;
    for (const float scale : {1.f, 1.25f, 1.5f, 1.75f, 2.f}) {
        char label[16];
        std::snprintf(label, sizeof(label), "%d %%", int(std::lround(scale * 100)));
        if (ImGui::MenuItem(label, nullptr, uiScaleSetting_ == scale))
            setUiScale(scale);
    }
    char desktop[48];
    std::snprintf(desktop, sizeof(desktop), "Match desktop (%d %%)", int(std::lround(window_->contentScale() * 100)));
    if (ImGui::MenuItem(desktop, nullptr, uiScaleSetting_ <= 0))
        setUiScale(0);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Size everything like the desktop's other applications (GNOME's display scale).");
    ImGui::EndMenu();
}

void App::drawViewMenu() {
    if (!scenario_) {
        ImGui::TextDisabled("Waiting for the scenario");
        ImGui::Separator();
        drawThemeMenu();
        drawScaleMenu();
        return;
    }
    ImGui::SeparatorText("Camera");
    if (ImGui::MenuItem("Orbit", nullptr, mode_ == 0))
        setViewMode(0);
    if (ImGui::MenuItem("Free camera", nullptr, mode_ == 1))
        setViewMode(1);
    for (std::size_t i = 0; i < scenario_->cameras.size(); ++i)
        if (ImGui::MenuItem(scenario_->cameras[i].id.c_str(), nullptr, mode_ == int(i) + 2))
            setViewMode(int(i) + 2);
    if (ImGui::BeginMenu("Focus")) {
        for (const auto &name : focusNames_)
            if (ImGui::MenuItem(name.c_str(), nullptr, focusName_ == name))
                focus(name);
        ImGui::EndMenu();
    }
    ImGui::MenuItem("Follow", nullptr, &follow_, presetFor(focusName_).follow);
    ImGui::SeparatorText("Overlays");
    ImGui::MenuItem("Labels", nullptr, &labels_);
    ImGui::MenuItem("TF frames", nullptr, &showTf_);
    ImGui::MenuItem("Detections", nullptr, &detections_, !demoMode_);
    ImGui::MenuItem("MPC path", nullptr, &showMpc_, !demoMode_);
    ImGui::MenuItem("Thrust", nullptr, &showThrust_, !demoMode_ && !scenario_->thrusterMounts.empty());
    ImGui::Separator();
    if (ImGui::MenuItem("Maximize pool view", "Ctrl+Space", maximized_))
        toggleMaximized();
    ImGui::MenuItem("Frame stats", "F3", &showProfile_);
    ImGui::Separator();
    if (ImGui::MenuItem("Left panels", "Ctrl+[", sideShown(Side::Left)))
        toggleSide(Side::Left);
    if (ImGui::MenuItem("Right panels", "Ctrl+]", sideShown(Side::Right)))
        toggleSide(Side::Right);
    ImGui::Separator();
    drawThemeMenu();
    drawScaleMenu();
}

void App::drawThemeMenu() {
    if (!ImGui::BeginMenu("Theme"))
        return;
    for (const auto &theme : themes()) {
        if (ImGui::MenuItem(theme.label.c_str(), nullptr, currentTheme() == theme.id))
            setTheme(theme.id);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", theme.description.c_str());
    }
    ImGui::EndMenu();
}

void App::drawWindowsMenu() {
    if (!scenario_) {
        ImGui::TextDisabled("Waiting for the scenario");
        return;
    }
    if (composition_ && !composition_->empty()) {
        ImGui::SeparatorText("Panels");
        composition_->drawPanelMenuItems();
    }
    ImGui::SeparatorText("Cameras");
    for (std::size_t i = 0; i < scenario_->cameras.size() && i < cards_.size(); ++i)
        if (ImGui::MenuItem(scenario_->cameras[i].title.c_str(), nullptr, &cards_[i].open) && cards_[i].open)
            focusOnce_.insert(cameraWindowName(scenario_->cameras[i])); // reopened: shown on top
    ImGui::MenuItem("Course map", nullptr, &mapOpen_);
    ImGui::SeparatorText("Tools");
    ImGui::MenuItem("Scene settings", nullptr, &sceneOpen_);
    ImGui::MenuItem("Display", nullptr, &displayOpen_);
    ImGui::MenuItem("TF frames", nullptr, &tfOpen_);
    if (composition_)
        composition_->drawToolMenuItems();
}

void App::drawLayoutMenu() {
    ImGui::SeparatorText("Built-in");
    for (const auto &preset : layoutPresets()) {
        if (ImGui::MenuItem(preset.label.c_str(), preset.shortcut.c_str()))
            pendingPreset_ = preset.id;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s\nAlso reopens the default windows.", preset.description.c_str());
    }
    ImGui::SeparatorText("Saved");
    const auto names = savedLayouts(layoutDir_);
    if (ImGui::BeginMenu("Load", !names.empty())) {
        for (const auto &name : names)
            if (ImGui::MenuItem(name.c_str()))
                requestLayout(name);
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Save current as", !layoutDir_.empty())) {
        ImGui::SetNextItemWidth(ui(200));
        if (ImGui::IsWindowAppearing())
            ImGui::SetKeyboardFocusHere();
        const bool entered = ImGui::InputTextWithHint("##layout_name", "Layout name", layoutName_, sizeof(layoutName_),
                                                      ImGuiInputTextFlags_EnterReturnsTrue);
        ImGui::SameLine();
        ImGui::BeginDisabled(layoutFileStem(layoutName_).empty());
        if (ImGui::Button("Save") || (entered && !layoutFileStem(layoutName_).empty())) {
            saveLayout(layoutName_);
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndDisabled();
        if (std::find(names.begin(), names.end(), layoutFileStem(layoutName_)) != names.end())
            ImGui::TextDisabled("Replaces the saved layout of that name.");
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Delete", !names.empty())) {
        for (const auto &name : names)
            if (ImGui::MenuItem(name.c_str())) {
                std::error_code error;
                fs::remove(layoutDir_ / (name + ".ini"), error);
                layoutMessage_ = error ? "Could not delete " + name + ": " + error.message() : "Deleted " + name;
            }
        ImGui::EndMenu();
    }
    if (!layoutDir_.empty() && ImGui::IsItemHovered())
        ImGui::SetTooltip("Saved layouts live in %s", layoutDir_.c_str());
    ImGui::Separator();
    ImGui::MenuItem("Lock layout", nullptr, &layoutLocked_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Stop windows from being undocked, docked or resized by accident (closing still works).");
    if (ImGui::MenuItem("Maximize pool view", "Ctrl+Space", maximized_))
        toggleMaximized();
    if (!layoutMessage_.empty()) {
        ImGui::Separator();
        ImGui::TextDisabled("%s", layoutMessage_.c_str());
    }
}

// Always visible whatever the layout: branding, the pinned panel controls (Enable / KILL), robot status chips
// and the pose-source pill.
void App::drawCommandBar() {
    const float height = ui(52), padding = ui(9);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(16, padding));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, palette().bar);
    const bool open = ImGui::BeginViewportSideBar("##command_bar", ImGui::GetMainViewport(), ImGuiDir_Up, height,
                                                  ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoScrollbar |
                                                      ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::PopStyleColor();
    ImGui::PopStyleVar();
    if (open) {
        const float W = ImGui::GetWindowWidth(), inner = height - 2 * padding;
        const std::string headerTitle = lookup(config_, {"branding", "header"}).as<std::string>("NEREUS");
        std::string headerSubtitle = scenario_ ? scenarioLabel(*scenario_) : "";
        std::transform(headerSubtitle.begin(), headerSubtitle.end(), headerSubtitle.begin(),
                       [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
        headerSubtitle = lookup(config_, {"branding", "subtitle"}).as<std::string>(headerSubtitle);
        ImGui::PushFont(window_->title);
        ImGui::SetCursorPosY(padding + (inner - ImGui::GetFontSize()) * .5f);
        ImGui::TextUnformatted(headerTitle.c_str());
        ImGui::PopFont();
        ImGui::SameLine();
        ImGui::SetCursorPosY(padding + (inner - ImGui::GetFontSize()) * .5f);
        ImGui::TextColored(palette().muted, "/");
        ImGui::SameLine();
        ImGui::SetCursorPosY(padding + (inner - ImGui::GetFontSize()) * .5f);
        ImGui::TextUnformatted(headerSubtitle.c_str());
        if (composition_) { // pinned controls: Enable / KILL and the robot's state
            ImGui::SameLine(0, ui(32));
            ImGui::SetCursorPosY(padding);
            composition_->drawPinned();
        }
        const float statusWidth = ImGui::CalcTextSize(status_.c_str()).x + 2 * ImGui::GetStyle().FramePadding.x;
        if (composition_) { // header items (robot telemetry, recording) sit just left of the status pill
            ImGui::SameLine();
            ImGui::SetCursorPosY(padding + (inner - ImGui::GetFrameHeight()) * .5f);
            composition_->drawHeader(W - 16 - statusWidth - ImGui::GetStyle().ItemSpacing.x);
        }
        ImGui::SameLine(W - 16 - statusWidth);
        ImGui::SetCursorPosY(padding + (inner - ImGui::GetFrameHeight()) * .5f);
        pill(status_, demoMode_ ? palette().warn : (ros_->poseFresh() ? palette().accent : palette().muted));
    }
    ImGui::End();
}

// The 3D pool view: the configured toolbar on top, the rendered scene below with its overlays. It fills the
// dock space's central node; other windows dock around it.
void App::drawPoolView(double time, float dt) {
    ImGuiWindowClass single;
    single.DockNodeFlagsOverrideSet = ImGuiDockNodeFlags_AutoHideTabBar;
    ImGui::SetNextWindowClass(&single);
    ImGui::SetNextWindowDockID(dockspace_, ImGuiCond_FirstUseEver);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ui(ImVec2(0, 0)));
    const bool visible =
        ImGui::Begin(kPoolView, nullptr,
                     ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoCollapse);
    ImGui::PopStyleVar();
    if (!scenario_) {
        ImGui::SetCursorPos(ui(ImVec2(24, 24)));
        ImGui::TextColored(
            palette().muted, "Waiting for the bridge scenario document (%s) ...",
            !opt_.scenarioTopic.empty()
                ? opt_.scenarioTopic.c_str()
                : lookup(config_, {"scenario_topic"}).as<std::string>("/talos/simulator/scenario").c_str());
        ImGui::End();
        return;
    }
    if (!visible) { // covered by another tab: the camera windows still need their renders
        PhaseTimer timer{profiler_, Phase::Scene, false};
        const auto scene = model_->build(buildState());
        renderLocalCards(time, scene);
        ImGui::End();
        return;
    }
    const float width = std::max(16.f, ImGui::GetContentRegionAvail().x);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ui(ImVec2(10, 7)));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ui(ImVec2(6, 6)));
    ImGui::PushStyleColor(ImGuiCol_ChildBg, palette().toolbar);
    ImGui::BeginChild("toolbar", {width, toolbarHeight_}, ImGuiChildFlags_AlwaysUseWindowPadding,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    int oldMode = mode_;
    drawToolbar(width, oldMode);
    toolbarHeight_ =
        std::max(ImGui::GetFrameHeight() + ui(14), ImGui::GetCursorPosY() - ImGui::GetStyle().ItemSpacing.y + ui(7));
    ImGui::EndChild();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(2);
    auto &io = ImGui::GetIO();
    const float viewHeight = std::max(1.f, ImGui::GetContentRegionAvail().y);
    const ImVec2 position = ImGui::GetCursorScreenPos();
    bool hovered = ImGui::IsMouseHoveringRect(position, {position.x + width, position.y + viewHeight});
    // Dropdowns can overlap the viewport. Selecting Free camera must not also consume that click as a
    // mouse-capture request.
    hovered = hovered && ImGui::IsWindowHovered() && mode_ == oldMode &&
              !ImGui::IsPopupOpen(nullptr, ImGuiPopupFlags_AnyPopupId);
    // The tabs that bring back a snapped-shut side take their own clicks, not the camera's.
    for (const Side side : {Side::Left, Side::Right})
        hovered = hovered && !sides_[int(side)].dragging && !drawSideHandle(side, position, {width, viewHeight}, false);
    panels::Viewport panelView{viewportView_.projection,
                               viewportView_.view,
                               viewportView_.eye,
                               {position.x, position.y},
                               {width, viewHeight},
                               hovered && mode_ == 0,
                               bool(glfwGetWindowAttrib(window_->handle(), GLFW_FOCUSED))};
    // Commands live in the estimate frame; anchored on the truth robot, draw them re-rooted there (offset
    // sampled at one time for both poses, see RosSide::truthFromEstimate).
    if (!gizmoOnEstimate_ && haveEstimate_)
        panelView.displayFromCommand = body_ * glm::inverse(estimateBody_);
    const bool dragging = composition_ && mode_ == 0 && composition_->input(panelView);
    bool unused = false;
    SensorView view = viewFor(width / viewHeight, dt, hovered && !dragging, viewHeight, unused);
    viewportView_ = view;
    // Keep sensor aspect ratios when a camera view is promoted to the large viewport.
    float iw = width, ih = viewHeight;
    const SensorCamera *sensor = mode_ >= 2 ? &scenario_->cameras[std::size_t(mode_ - 2)] : nullptr;
    if (sensor) {
        const float aspect = float(sensor->k.width) / float(sensor->k.height);
        ih = std::min(viewHeight, width / aspect);
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
    const rendering::View renderView{toEigen(view.view), toEigen(view.projection),
                                     Eigen::Vector3f(view.eye.x, view.eye.y, view.eye.z)};
    auto appearance = observer_.apply(look_.appearance);
    // A window too large for the supersampled targets falls back to fewer samples.
    GLint maximumTexture = 0;
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maximumTexture);
    appearance.supersample = std::clamp(int(maximumTexture) / std::max(rw, rh), 1, appearance.supersample);
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
        focusAtCursor(view, frame, position, width, viewHeight);
    const ImVec2 imagePos(position.x + (width - iw) / 2, position.y + (viewHeight - ih) / 2);
    ImGui::SetCursorScreenPos(imagePos);
    // A promoted sensor view shows the bridge's depth image instead while its card is on DEPTH.
    const std::size_t sensorIndex = sensor ? std::size_t(mode_ - 2) : 0;
    if (sensor && sensorIndex < cards_.size() && sensorIndex < ros_->feeds.size() &&
        ros_->feeds[sensorIndex].wantDepth && cards_[sensorIndex].depth)
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
    if (showMpc_ && !demoMode_) {
        drawPlannedPath(ros_->plannedPath, vp, rect);
        drawMpcPath(ros_->mpcPath, vp, rect);
    }
    if (showThrust_ && !demoMode_)
        drawThrust(scenario_->thrusterMounts, ros_->thrust, body_, thrustScale_, vp, rect);
    if (composition_ && mode_ == 0) {
        panelView.projection = view.projection;
        panelView.view = view.view;
        panelView.eye = view.eye;
        composition_->drawOverlays(panelView);
    }
    auto *d = ImGui::GetWindowDrawList();
    d->AddRect(position, {position.x + width, position.y + viewHeight}, IM_COL32(38, 62, 72, 255), 5, 0, 1);
    d->AddRectFilled({position.x + ui(14), position.y + ui(14)}, {position.x + ui(237), position.y + ui(43)},
                     IM_COL32(8, 22, 29, 225), 4);
    d->AddText(
        window_->small, window_->small->FontSize, {position.x + ui(25), position.y + ui(22)}, color(white),
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
        d->AddRectFilled({position.x + ui(14), position.y + ui(50)},
                         {position.x + ui(30) + size.x, position.y + ui(62) + size.y}, IM_COL32(8, 22, 29, 225), 4);
        d->AddText(window_->small, window_->small->FontSize, {position.x + ui(22), position.y + ui(56)}, color(white),
                   profileText_.c_str());
    }
    if (runScore_ && runScore_["total"]) {
        std::string readout = fixed(runScore_["total"].as<double>(), 1) + " pts   /   " + runTime();
        if (runScore_["running"].as<bool>(false))
            readout += "  RUNNING";
        d->AddRectFilled({position.x + width - ui(310), position.y + ui(12)},
                         {position.x + width - ui(12), position.y + ui(43)}, IM_COL32(8, 22, 29, 225), 4);
        d->AddText(window_->small, window_->small->FontSize * 14 / 12,
                   {position.x + width - ui(298), position.y + ui(21)}, color(cyan), readout.c_str());
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
            const ImVec2 at(position.x + (p.x * .5f + .5f) * width, position.y + (.5f - p.y * .5f) * viewHeight);
            d->AddCircleFilled(at, 3, color(cyan));
            d->AddLine(at, {at.x + ui(10), at.y - ui(14)}, color(cyan));
            d->AddRectFilled({at.x + ui(9), at.y - ui(31)}, {at.x + ui(105), at.y - ui(11)}, IM_COL32(8, 22, 29, 215),
                             3);
            d->AddText(window_->small, window_->small->FontSize, {at.x + ui(16), at.y - ui(28)}, color(white),
                       key.c_str());
        }
    }
    const char *controls =
        mode_ == 1 ? "CLICK  mouse look    WASD  move    SPACE / SHIFT  up / down    CTRL  fast    ESC  release"
                   : "LEFT DRAG  orbit   RIGHT / MIDDLE DRAG  pan   SCROLL  zoom   F  focus cursor";
    d->AddRectFilled({position.x, position.y + viewHeight - ui(30)}, {position.x + width, position.y + viewHeight},
                     IM_COL32(6, 18, 26, 205));
    d->AddText(window_->small, window_->small->FontSize, {position.x + ui(14), position.y + viewHeight - ui(21)},
               color(white), controls);
    for (const Side side : {Side::Left, Side::Right})
        drawSideHandle(side, position, {width, viewHeight}, true);
    ImGui::End();
}

// Snap shut: a side dragged narrower than kSnapWidth closes, like the old sidebars' edges; pulled back past
// kOpenWidth it reopens; widths from kRememberWidth up are what it reopens at otherwise.
constexpr float kSnapWidth = 150, kOpenWidth = 180, kRememberWidth = 220;

bool App::sideShown(Side side) const {
    return !sides_[int(side)].collapsed && sideWidth(dockspace_, side) > 0;
}

// While the border is dragged: narrower than kSnapWidth snaps the side shut at once, and the drag carries on
// (pull back past kOpenWidth to reopen it, the width following the mouse) until the button is let go.
void App::updateSides() {
    if (!scenario_ || maximized_ || ImGui::GetMainViewport()->WorkSize.x < 500)
        return;
    const auto *root = ImGui::DockBuilderGetNode(dockspace_);
    const float mouse = ImGui::GetIO().MousePos.x;
    for (const Side side : {Side::Left, Side::Right}) {
        auto &state = sides_[int(side)];
        const float width = sideWidth(dockspace_, side);
        if (state.dragging) {
            if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                state.dragging = false;
                if (!state.collapsed && state.pending <= 0 && width >= kRememberWidth)
                    state.width = width;
            } else if (root) {
                ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
                const float pulled = side == Side::Left ? mouse - root->Pos.x : root->Pos.x + root->Size.x - mouse;
                if (state.collapsed && pulled >= kOpenWidth) {
                    expandSide(side);
                    state.pending = pulled;
                } else if (!state.collapsed && pulled < kSnapWidth)
                    collapseSide(side);
                else if (!state.collapsed)
                    state.pending = std::max(kSnapWidth, pulled);
            }
        }
        if (!state.collapsed && state.pending > 0 && width > 0) { // reopened: at the dragged width
            setSideWidth(dockspace_, side, state.pending);
            if (!state.dragging) {
                state.width = std::max(kRememberWidth, state.pending);
                state.pending = 0;
            }
            continue;
        }
        if (state.dragging || state.collapsed || width <= 0)
            continue;
        if (width < kSnapWidth && !layoutLocked_) { // the dock border dragged (or loaded) this narrow
            collapseSide(side);
            state.dragging = ImGui::IsMouseDown(ImGuiMouseButton_Left);
        } else if (!ImGui::IsMouseDown(ImGuiMouseButton_Left) && width >= kRememberWidth)
            state.width = width;
    }
}

void App::collapseSide(Side side) {
    auto &state = sides_[int(side)];
    const auto names = sideWindows(dockspace_, side);
    if (names.empty())
        return;
    // Back to a usable width first: the closed column keeps it for when it opens again (or is saved).
    if (state.width < kRememberWidth)
        state.width = std::max(kRememberWidth, ImGui::GetMainViewport()->WorkSize.x * .24f);
    setSideWidth(dockspace_, side, state.width);
    state.keys.clear();
    for (const auto &entry : windowEntries())
        if (*entry.open && std::find(names.begin(), names.end(), entry.name) != names.end()) {
            state.keys.push_back(entry.key);
            *entry.open = false;
        }
    state.collapsed = !state.keys.empty();
}

void App::expandSide(Side side) {
    auto &state = sides_[int(side)];
    for (const auto &entry : windowEntries())
        if (std::find(state.keys.begin(), state.keys.end(), entry.key) != state.keys.end())
            *entry.open = true;
    state.collapsed = false;
    state.keys.clear();
    state.pending = 0;
}

void App::toggleSide(Side side) {
    if (sides_[int(side)].collapsed)
        expandSide(side);
    else
        collapseSide(side);
}

void App::resetSides() {
    for (auto &state : sides_)
        state = {};
}

// A snapped-shut side's edge on the pool view, as the old sidebars had: invisible until hovered (resize cursor, a
// thin line and grip), dragged out to bring the panels back at that width. With draw false it only reports
// whether it has the pointer.
bool App::drawSideHandle(Side side, ImVec2 viewPos, ImVec2 viewSize, bool draw) {
    auto &state = sides_[int(side)];
    if (!state.collapsed)
        return false;
    const bool left = side == Side::Left;
    const float w = ui(10);
    const ImVec2 min(left ? viewPos.x : viewPos.x + viewSize.x - w, viewPos.y), max(min.x + w, viewPos.y + viewSize.y);
    if (!draw)
        return ImGui::IsMouseHoveringRect(min, max) || state.dragging;
    ImGui::SetCursorScreenPos(min);
    ImGui::InvisibleButton(left ? "##show_left_panels" : "##show_right_panels", {w, viewSize.y});
    const bool hovered = ImGui::IsItemHovered();
    if (ImGui::IsItemActivated())
        state.dragging = true; // updateSides follows the mouse from here
    if (hovered || state.dragging) {
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
        const float mouse = ImGui::GetIO().MousePos.x;
        const float x =
            state.dragging ? std::clamp(mouse, viewPos.x, viewPos.x + viewSize.x) : (left ? min.x + 2 : max.x - 2);
        const auto tint = ImGui::GetColorU32(palette().accent);
        const float middle = viewPos.y + viewSize.y * .5f;
        auto *d = ImGui::GetWindowDrawList();
        d->AddLine({x, viewPos.y}, {x, viewPos.y + viewSize.y}, tint, 2);
        d->AddRectFilled({x - 2, middle - 20}, {x + 2, middle + 20}, tint, 2);
    }
    if (hovered && !state.dragging)
        ImGui::SetTooltip(left ? "Drag right to show the panels" : "Drag left to show the panels");
    return hovered || state.dragging;
}

// The layout as saved: from before Maximize, and with snapped-shut sides open as they were.
std::string App::layoutSnapshot() {
    if (maximized_)
        return beforeMaximize_;
    std::vector<bool *> reopened;
    for (const auto &state : sides_)
        for (const auto &entry : windowEntries())
            if (!*entry.open && std::find(state.keys.begin(), state.keys.end(), entry.key) != state.keys.end()) {
                *entry.open = true;
                reopened.push_back(entry.open);
            }
    std::string text = ImGui::SaveIniSettingsToMemory();
    for (auto *flag : reopened)
        *flag = false;
    return text;
}

void App::drawCameraWindows() {
    camerasShown_ = false;
    for (std::size_t i = 0; i < ros_->feeds.size() && i < cards_.size(); ++i) {
        cardVisible_[i] = 0;
        if (!cards_[i].open)
            continue;
        camerasShown_ = true;
        const auto name = cameraWindowName(*ros_->feeds[i].camera);
        focusIfRequested(name);
        ImGui::SetNextWindowSize(ui(ImVec2(440, 360)), ImGuiCond_FirstUseEver);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ui(ImVec2(10, 8)));
        const bool visible = ImGui::Begin(name.c_str(), &cards_[i].open);
        ImGui::PopStyleVar();
        if (visible) {
            windowContextMenu("camera." + ros_->feeds[i].camera->id);
            pins::beginScope("camera." + ros_->feeds[i].camera->id, ros_->feeds[i].camera->title);
            drawCameraCard(i);
            pins::endScope();
        }
        ImGui::End();
    }
    for (std::size_t i = 0; i < ros_->feeds.size() && i < cards_.size(); ++i) // pinned controls of hidden ones
        if (pins::needsDrawing("camera." + ros_->feeds[i].camera->id))
            pins::drawOffscreen("camera." + ros_->feeds[i].camera->id, ros_->feeds[i].camera->title,
                                [&] { drawCameraCard(i); });
    ros_->setCamerasWanted(camerasShown_);
}

void App::drawHelpWindow() {
    if (!helpOpen_)
        return;
    const auto *viewport = ImGui::GetMainViewport();
    focusIfRequested(kHelp);
    if (beginToolWindow(kHelp, &helpOpen_, ui(ImVec2(600, 0)),
                        {viewport->WorkPos.x + viewport->WorkSize.x * .5f - 300, viewport->WorkPos.y + 80})) {
        windowContextMenu("help");
        const auto table = [](const char *id, std::initializer_list<std::pair<const char *, const char *>> rows) {
            if (!ImGui::BeginTable(id, 2, ImGuiTableFlags_RowBg))
                return;
            ImGui::TableSetupColumn("keys", ImGuiTableColumnFlags_WidthFixed, ui(170));
            ImGui::TableSetupColumn("action", ImGuiTableColumnFlags_WidthStretch);
            for (const auto &[keys, action] : rows) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextColored(palette().accent, "%s", keys);
                ImGui::TableNextColumn();
                ImGui::TextWrapped("%s", action);
            }
            ImGui::EndTable();
        };
        ImGui::SeparatorText("Pool view");
        table("view_keys",
              {{"Left drag", "orbit"},
               {"Right / middle drag", "pan (detaches Follow)"},
               {"Scroll", "zoom"},
               {"F", "focus on the point under the cursor"},
               {"Free camera", "click for mouse look, WASD move, Space / Shift up / down, Ctrl fast, "
                               "Esc release"},
               {"Gizmo", "drag an arrow to move, a ring to rotate; Esc during a drag restores the start"}});
        ImGui::SeparatorText("Shortcuts");
        table("shortcuts", {{"Ctrl+Space", "maximize the pool view / restore the layout"},
                            {"Ctrl+Shift+1 / 2 / 3", "Standard / Wide view / Camera wall layout"},
                            {"Ctrl+[ / Ctrl+]", "snap the left / right panels shut, or bring them back"},
                            {"F1", "this window"},
                            {"F3", "frame-time stats"}});
        ImGui::SeparatorText("Windows");
        ImGui::PushStyleColor(ImGuiCol_Text, palette().muted);
        ImGui::TextWrapped(
            "Every panel, camera, the course map and the tool windows can be moved: drag a window's tab onto "
            "another window to dock it there (the arrows show where), next to it to split the space, or away "
            "to float it. Drag the borders between windows to resize. Close a window with its x and reopen it "
            "from the Windows menu. The layout is saved when the viewer closes; Layout > Save current as keeps "
            "named layouts, and Layout > Standard puts everything back. Right-click a window's tab to pin it to the "
            "toolbar, or any button, checkbox or dropdown in a panel to pin that control; right-click the toolbar (or "
            "its +) to choose its buttons. View > Theme changes the colours, View > Interface scale the size.");
        ImGui::PopStyleColor();
    }
    ImGui::End();
}

void App::handleShortcuts() {
    if (ImGui::GetIO().WantTextInput)
        return;
    if (ImGui::Shortcut(ImGuiMod_Ctrl | ImGuiKey_Space, ImGuiInputFlags_RouteGlobal))
        toggleMaximized();
    const auto &presets = layoutPresets();
    for (std::size_t i = 0; i < presets.size() && i < 9; ++i)
        if (ImGui::Shortcut(ImGuiMod_Ctrl | ImGuiMod_Shift | ImGuiKey(ImGuiKey_1 + int(i)),
                            ImGuiInputFlags_RouteGlobal))
            pendingPreset_ = presets[i].id;
    if (ImGui::Shortcut(ImGuiKey_F1, ImGuiInputFlags_RouteGlobal))
        helpOpen_ = !helpOpen_;
    if (ImGui::Shortcut(ImGuiMod_Ctrl | ImGuiKey_LeftBracket, ImGuiInputFlags_RouteGlobal))
        toggleSide(Side::Left);
    if (ImGui::Shortcut(ImGuiMod_Ctrl | ImGuiKey_RightBracket, ImGuiInputFlags_RouteGlobal))
        toggleSide(Side::Right);
}

// --------------------------------------------------------------------------------------------- layout

std::vector<App::WindowEntry> App::windowEntries() {
    std::vector<WindowEntry> entries;
    if (composition_) {
        const auto flags = composition_->visibility();
        const auto windows = composition_->panelWindows(); // same order
        for (std::size_t i = 0; i < flags.size() && i < windows.size(); ++i)
            entries.push_back({flags[i].first, windows[i].name, windows[i].name.substr(0, windows[i].name.find("###")),
                               flags[i].second});
    }
    if (scenario_)
        for (std::size_t i = 0; i < scenario_->cameras.size() && i < cards_.size(); ++i)
            entries.push_back({"camera." + scenario_->cameras[i].id, cameraWindowName(scenario_->cameras[i]),
                               scenario_->cameras[i].title, &cards_[i].open});
    entries.push_back({"map", kCourseMap, "Course map", &mapOpen_});
    entries.push_back({"scene_settings", kSceneSettings, "Scene settings", &sceneOpen_});
    entries.push_back({"display", kDisplay, "Display", &displayOpen_});
    entries.push_back({"tf", kTfFrames, "TF frames", &tfOpen_});
    entries.push_back({"help", kHelp, "Controls & shortcuts", &helpOpen_});
    return entries;
}

WindowStates::Flags App::windowFlags() {
    WindowStates::Flags flags;
    for (const auto &entry : windowEntries())
        flags.emplace_back(entry.key, entry.open);
    return flags;
}

// The operator's toolbar: which configured items show ("toolbar.<id>") and which windows are pinned to it
// ("pin.<key>"). Saved with the layout; the built-in layouts leave it alone.
WindowStates::Flags App::toolbarFlags() {
    WindowStates::Flags flags;
    if (composition_)
        for (const auto &item : composition_->toolbarItems())
            flags.emplace_back("toolbar." + item.id, item.visible);
    for (const auto &entry : windowEntries())
        flags.emplace_back("pin." + entry.key, &pins_[entry.key]);
    return flags;
}

// The configured state of windows seen for the first time, which the built-in layouts restore.
void App::rememberDefaults() {
    for (const auto &[key, flag] : windowFlags())
        defaultOpen_.emplace(key, *flag);
    for (const auto &[key, flag] : toolbarFlags())
        defaultOpen_.emplace(key, *flag);
}

void App::resetToolbar() {
    pins::clear();
    for (const auto &[key, flag] : toolbarFlags()) {
        const auto value = defaultOpen_.find(key);
        *flag = value != defaultOpen_.end() && value->second;
    }
}

// Shows a closed window on top, or brings an open one forward (selects its tab).
void App::showWindow(const WindowEntry &entry) {
    if (entry.key.rfind("panel.", 0) == 0 && composition_)
        composition_->focusPanel(entry.key.substr(6));
    else if (!*entry.open) {
        *entry.open = true;
        focusOnce_.insert(entry.name);
    } else
        ImGui::SetWindowFocus(entry.name.c_str());
}

// Right-click on a window's tab (or title bar): pin it to the toolbar, or close it.
void App::windowContextMenu(const std::string &key) {
    if (!ImGui::BeginPopupContextItem("##window_menu"))
        return;
    bool &pinned = pins_[key];
    if (ImGui::MenuItem("Pin to toolbar", nullptr, pinned))
        pinned = !pinned;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("A toolbar button that shows this window, brings it to the front or hides it");
    if (ImGui::MenuItem("Close"))
        for (const auto &entry : windowEntries())
            if (entry.key == key)
                *entry.open = false;
    ImGui::EndPopup();
}

// A pinned window's toolbar button: lit while the window is in front; shows it, brings it forward, or hides it.
void App::pinnedWindowButton(const WindowEntry &entry) {
    const bool front = *entry.open && windowInFront(entry.name.c_str());
    ImGui::PushID(entry.key.c_str());
    sameLineIfFits(buttonWidth(entry.label.c_str()));
    pushActiveColors(front);
    if (ImGui::Button(entry.label.c_str())) {
        if (front)
            *entry.open = false;
        else
            showWindow(entry);
    }
    popActiveColors();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("%s %s (pinned: right-click the toolbar to unpin)",
                          front         ? "Hide"
                          : *entry.open ? "Bring forward"
                                        : "Show",
                          entry.label.c_str());
    ImGui::PopID();
}

namespace {
// Toolbar item names for the customization menu (a configured `title:` wins).
std::string toolbarLabel(const std::string &type) {
    static const std::map<std::string, std::string> labels{{"view", "Camera"},
                                                           {"focus", "Focus"},
                                                           {"follow", "Follow"},
                                                           {"labels", "Labels"},
                                                           {"tf", "TF"},
                                                           {"mpc_path", "MPC path"},
                                                           {"thrust", "Thrust"},
                                                           {"detections", "Detections"},
                                                           {"pool_viewer", "Display"},
                                                           {"scene_settings", "Scene"},
                                                           {"panels_menu", "Windows"},
                                                           {"preview_task", "Preview task"},
                                                           {"simulation", "Simulation"},
                                                           {"run", "Run tracking"},
                                                           {"motion", "Enable / KILL"},
                                                           {"telemetry", "Telemetry"},
                                                           {"recording", "Recording"}};
    const auto found = labels.find(type);
    return found == labels.end() ? type : found->second;
}
} // namespace

void App::drawToolbarCustomization() {
    if (composition_ && !composition_->toolbarItems().empty()) {
        ImGui::SeparatorText("Toolbar buttons");
        for (const auto &item : composition_->toolbarItems()) {
            ImGui::PushID(item.id.c_str());
            ImGui::Checkbox(item.title.empty() ? toolbarLabel(item.type).c_str() : item.title.c_str(), item.visible);
            ImGui::PopID();
        }
    }
    ImGui::SeparatorText("Pinned controls");
    pins::drawCustomization();
    ImGui::SeparatorText("Pinned windows");
    ImGui::TextDisabled("Also: right-click a window's tab > Pin to toolbar");
    for (const auto &entry : windowEntries()) {
        ImGui::PushID(entry.key.c_str());
        ImGui::Checkbox(entry.label.c_str(), &pins_[entry.key]);
        ImGui::PopID();
    }
    ImGui::Separator();
    if (ImGui::Button("Reset toolbar"))
        resetToolbar();
}

// --open: windows shown at start whatever the saved layout says.
void App::applyCommandLineWindows() {
    commandLineDone_ = true;
    for (const auto &name : opt_.open) {
        sceneOpen_ |= name == "scene-settings";
        if (name == "map")
            mapOpen_ = focusMap_ = true;
        tfOpen_ |= name == "tf";
        displayOpen_ |= name == "pool-viewer" || name == "display";
        helpOpen_ |= name == "help";
    }
}

std::string App::defaultPreset() const {
    const auto configured = lookup(config_, {"layout"}).as<std::string>("standard");
    if (findPreset(configured))
        return configured;
    std::cerr << "nereus-viewer: unknown layout '" << configured << "' in the host config; using standard\n";
    return "standard";
}

// A built-in preset by id, a saved layout by name, or an .ini file by path.
void App::requestLayout(const std::string &name) {
    if (findPreset(name)) {
        pendingPreset_ = name;
        return;
    }
    fs::path file = name;
    if (!fs::exists(file) && !layoutDir_.empty())
        file = layoutDir_ / (layoutFileStem(name) + ".ini");
    std::ifstream in(file);
    if (!in) {
        layoutMessage_ = "No layout '" + name + "'";
        std::cerr << "nereus-viewer: no built-in or saved layout '" << name << "'\n";
        if (!layoutReady_)
            pendingPreset_ = defaultPreset();
        return;
    }
    std::stringstream text;
    text << in.rdbuf();
    pendingIni_ = text.str();
    maximized_ = false;
    resetSides();
    layoutReady_ = true;
    layoutMessage_ = "Loaded " + file.stem().string();
}

// Ini text can only be loaded between frames (before NewFrame); the loop calls this.
void App::applyPendingIni() {
    if (pendingIni_.empty())
        return;
    const auto text = std::move(pendingIni_);
    pendingIni_.clear();
    ImGui::LoadIniSettingsFromMemory(text.c_str(), text.size());
}

void App::applyPreset(const std::string &id) {
    const auto *found = findPreset(id);
    LayoutPreset preset = found ? *found : layoutPresets().front();
    const auto size = ImGui::GetMainViewport()->WorkSize;
    if (preset.id == "standard" && composition_ && size.x > 0) // the composition's sidebar width
        preset.left = std::clamp(composition_->width(size.x) / size.x, .15f, .45f);
    for (const auto &[key, flag] : windowFlags()) {
        const auto value = defaultOpen_.find(key);
        if (value != defaultOpen_.end())
            *flag = value->second;
    }
    resetSides();
    if (!presetApplied_) // the session's first layout keeps the windows --open asked for
        applyCommandLineWindows();
    presetApplied_ = true;
    maximized_ = false;
    std::vector<LayoutWindow> windows;
    for (const auto &camera : scenario_->cameras)
        windows.push_back({cameraWindowName(camera), Dock::Right, true, false});
    windows.push_back({kCourseMap, Dock::RightBottom, false, false});
    if (composition_)
        for (const auto &panel : composition_->panelWindows())
            windows.push_back({panel.name, panel.dock, false, panel.selected});
    buildLayout(dockspace_, size, preset, kPoolView, windows);
    layoutReady_ = true;
    layoutMessage_ = preset.label + " layout";
}

void App::saveLayout(const std::string &name) {
    const auto stem = layoutFileStem(name);
    if (stem.empty() || layoutDir_.empty())
        return;
    try {
        fs::create_directories(layoutDir_);
        std::ofstream out(layoutDir_ / (stem + ".ini"));
        out << layoutSnapshot();
        if (!out)
            throw std::runtime_error("write failed");
        layoutMessage_ = "Saved " + stem;
    } catch (const std::exception &error) {
        layoutMessage_ = "Could not save " + stem + ": " + error.what();
    }
}

// Maximize: close every window but the pool view, remembering the layout; again: restore it exactly.
void App::toggleMaximized() {
    if (maximized_) {
        pendingIni_ = beforeMaximize_;
        maximized_ = false;
        return;
    }
    beforeMaximize_ = layoutSnapshot(); // restoring reopens snapped-shut sides too
    resetSides();
    for (const auto &[key, flag] : windowFlags())
        *flag = false;
    maximized_ = true;
}

// The session layout, written when ImGui reports a change (rate-limited) and on exit. While maximized the
// layout from before is what gets saved.
void App::persistLayout(bool force) {
    auto &io = ImGui::GetIO();
    if (!persist_ || !layoutReady_ || (!force && !io.WantSaveIniSettings))
        return;
    io.WantSaveIniSettings = false;
    std::error_code error;
    fs::create_directories(sessionIni_.parent_path(), error);
    std::ofstream out(sessionIni_);
    out << layoutSnapshot();
}

void App::focusIfRequested(const std::string &window) {
    if (focusOnce_.erase(window))
        ImGui::SetNextWindowFocus();
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
    if (!scenario_ || !model_ || !(demoMode_ || opt_.localCameras) || !camerasShown_)
        return;
    if (!demoMode_ && !(ros_->truthActive() && ros_->poseFresh()))
        return; // no simulator truth pose to render from
    const std::size_t count = std::min(scenario_->cameras.size(), cards_.size());
    std::vector<std::size_t> todo;
    for (std::size_t n = 0; n < count; ++n) {
        const std::size_t i = (nextCardTurn_ + n) % count;
        const bool ros = !demoMode_ && i < ros_->feeds.size() && ros_->feeds[i].rosMode; // truthActive checked above
        const bool depthShown = i < ros_->feeds.size() && ros_->feeds[i].wantDepth && cards_[i].depth;
        if (opt_.legacyCards ? (!ros && t >= cardDue_[0])
                             : (!ros && !depthShown && cardVisible_[i] && t >= cardDue_[i])) {
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
    const bool observerOnly = !observer_.walls || !observer_.floor || !look_.equipment || courseFromMapping() ||
                              (robotGhost_ && haveEstimate_) || mappingGhost_;
    if (observerOnly || opt_.legacyCards) {
        auto state = buildState();
        state.showWalls = state.showFloor = state.showCourse = state.showEquipment = true;
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
        const rendering::View renderView{toEigen(v.view), toEigen(v.projection),
                                         Eigen::Vector3f(v.eye.x, v.eye.y, v.eye.z)};
        auto appearance = look_.appearance;
        appearance.preview = !opt_.legacyCards;
        appearance.supersample = observer_.antialiasing;
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
                std::cerr << "nereus-viewer: rejecting scenario document: " << error.what() << '\n';
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
        applyPendingIni(); // a layout chosen last frame (saved layout, maximize restore)
        applyPendingUiScale();
        window_->beginFrame();
        drawInterface(t, dt);
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
        persistLayout(false);
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
                std::cout << "capture: status=" << status_ << " detections stored/placed=" << ros_->detectionCount()
                          << "/" << ros_->placedDetections.size() << " mpc_points=" << ros_->mpcPath.size()
                          << " planned_points=" << ros_->plannedPath.size() << " props=" << ros_->props.size()
                          << " projectiles=" << ros_->projectiles.size()
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
            std::this_thread::sleep_until(
                frameStart + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1. / cap)));
            profiler_.add(Phase::Sleep, std::chrono::duration<double>(Clock::now() - before).count());
        }
        profiler_.endFrame(Clock::now());
        if (opt_.profile) {
            const auto now = Clock::now();
            if (profileLogAt_ == Clock::time_point{})
                profileLogAt_ = now;
            if (now - profileLogAt_ >= std::chrono::seconds(5)) {
                std::cout << profileReport(profiler_.takeInterval(),
                                           std::chrono::duration<double>(now - profileLogAt_).count())
                          << (ros_ ? ros_->timingReport() : std::string()) << std::endl;
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
} // namespace nereus::ros_viewer::host

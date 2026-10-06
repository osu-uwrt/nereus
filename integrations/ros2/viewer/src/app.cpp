// The pool viewer's host application: one App owns the window, the ImGui interface (docked windows, toolbar,
// menus, command palette), the observer 3D view, the camera cards, the map editor and the ROS side feeding them.

#include "app.hpp"
#include "frame_profiler.hpp"
#include "mapping_markers.hpp"
#include "nereus/ros_viewer/dock_layout.hpp"
#include "nereus/ros_viewer/panel_layout.hpp"
#include "nereus/ros_viewer/panels/composition.hpp"
#include "nereus/ros_viewer/panels/ros_providers.hpp"
#include "nereus/ros_viewer/panels/pose_math.hpp"
#include "nereus/ros_viewer/pins.hpp"
#include "nereus/ros_viewer/plots/plots.hpp"
#include "nereus/ros_viewer/theme.hpp"
#include "overlay_draw.hpp"
#include "pose_command.hpp"
#include "prior_map_editor.hpp"
#include "top_down.hpp"
#include "scenario_packs.hpp"
#include "ros_side.hpp"
#include "scene_model.hpp"
#include "viewer_input.hpp"
#include "window.hpp"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <array>
#include <atomic>
#include <cctype>
#include <ctime>
#include <sys/wait.h>
#include <spawn.h>
#include <cmath>
#include <cfloat>
#include <cstring>
#include <functional>
#include <deque>
#include <fstream>
#include <imgui_internal.h>
#include <iomanip>
#include <iostream>
#include <nereus/rendering/renderer.hpp>
#include <set>
#include <sstream>
#include <mutex>
#include <thread>

// The process environment, handed to posix_spawnp (openFolder).
extern char **environ;

namespace nereus::ros_viewer::host {
namespace {
namespace fs = std::filesystem;
namespace panels = nereus::ros_viewer::panels;

// Keyboard driving's steps ([ / ] choose): meters along the heading and degrees of turn per key press
constexpr float kDriveMeters[] = {.05f, .1f, .25f, .5f, 1.f};
constexpr float kDriveDegrees[] = {2.f, 5.f, 15.f, 30.f, 45.f};

// Drawn over the 3D view and the course map, so the same in every theme (the interface uses palette()).
const ImVec4 muted(.47f, .57f, .64f, 1); // on the camera cards' dark video area, in every theme

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

// Overlay chips on the pool view and the course map: the theme's window color, nearly opaque.
ImU32 chipFill() {
    const ImVec4 bg = ImGui::GetStyle().Colors[ImGuiCol_WindowBg];
    return ImGui::GetColorU32(ImVec4(bg.x, bg.y, bg.z, .88f));
}

// Small ImGui, GL and string helpers.
ImU32 color(ImVec4 c) {
    return ImGui::ColorConvertFloat4ToU32(c);
}

ImTextureID textureID(GLuint t) {
    return static_cast<ImTextureID>(t);
}

// `x` as text with a fixed number of decimals.
std::string fixed(double x, int decimals = 1) {
    std::ostringstream s;
    s << std::fixed << std::setprecision(decimals) << x;
    return s.str();
}

// `s` without leading slashes (the scenario namespace as a panel context name).
std::string trimSlashes(std::string s) {
    while (!s.empty() && s.front() == '/')
        s.erase(s.begin());
    return s;
}

// The viewer's content folder (host yaml, themes, fonts, icons): the build-time path, else the working directory.
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

// The HDR value the renderer's post pass (exposure 1, ACES fit, 1/2.2 gamma) shows as display value `display`,
// kept below the bloom threshold (1.2).
float hdrFor(float display) {
    const float v = std::pow(std::clamp(display, 0.f, .999f), 2.2f);
    // ACES: x(2.51x + .03) / (x(2.43x + .59) + .14) = v, a quadratic in x with a < 0: the positive root
    const float a = 2.43f * v - 2.51f, b = .59f * v - .03f, c = .14f * v;
    const float x = (-b - std::sqrt(std::max(0.f, b * b - 4 * a * c))) / (2 * a);
    return std::clamp(x, 0.f, 1.15f);
}

// How well `text` matches a typed `query` (letters in order, any case; lower is better), or -1: earlier, closer
// together and at word starts ranks first, as VS Code's quick open.
int fuzzyScore(const std::string &text, const std::string &query) {
    int score = 0;
    std::size_t at = 0;
    long last = -1;
    for (const char wanted : query) {
        if (wanted == ' ')
            continue;
        const auto lower = [](char c) { return char(std::tolower(static_cast<unsigned char>(c))); };
        while (at < text.size() && lower(text[at]) != lower(wanted))
            ++at;
        if (at == text.size())
            return -1;
        const bool wordStart = at == 0 || !std::isalnum(static_cast<unsigned char>(text[at - 1]));
        score += int(at) - int(last) - 1 + (wordStart ? 0 : 2);
        last = long(at++);
    }
    return score;
}

// A folder in the desktop's file manager (xdg-open), without waiting for it.
void openFolder(const fs::path &folder) {
    pid_t pid = 0;
    const std::string path = folder.string();
    char *argv[] = {const_cast<char *>("xdg-open"), const_cast<char *>(path.c_str()), nullptr};
    if (posix_spawnp(&pid, "xdg-open", nullptr, nullptr, argv, environ) == 0)
        std::thread([pid] { waitpid(pid, nullptr, 0); }).detach();
}

// Where screenshots are saved: ~/Pictures/Nereus, or ~/Nereus without a Pictures folder.
fs::path screenshotDirectory() {
    const char *home = std::getenv("HOME");
    const fs::path base = home ? fs::path(home) : fs::temp_directory_path();
    return (fs::exists(base / "Pictures") ? base / "Pictures" : base) / "Nereus";
}

const fs::path kRunRecords = "/tmp/nereus_sim"; // sim.launch.py's run records

// Map editing's 2D chart colors, from the theme: the canvas around the pool, a floor one step from it toward
// the accent, lane lines a quiet stroke on the floor, the pool rim.
struct PlanPalette {
    ImVec4 canvas, floor, line, rim;
};

PlanPalette planPalette() {
    const ImVec4 bg = ImGui::GetStyle().Colors[ImGuiCol_WindowBg];
    const auto &p = palette();
    const auto mix = [](ImVec4 a, ImVec4 b, float t) {
        return ImVec4(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, 1);
    };
    const bool dark = .2126f * bg.x + .7152f * bg.y + .0722f * bg.z < .5f;
    PlanPalette out;
    out.canvas = ImVec4(bg.x, bg.y, bg.z, 1);
    // a ruled theme keeps the chart in its sheet's own grays (no tinted floor); the others lean to the accent
    out.floor =
        ruledTheme() ? mix(out.canvas, p.text, dark ? .07f : .06f) : mix(out.canvas, p.accent, dark ? .16f : .12f);
    out.line = mix(out.floor, p.text, dark ? .22f : .20f);
    out.rim = mix(out.canvas, p.text, dark ? .16f : .20f);
    return out;
}

// The observer view's own display options (Scene settings / Display windows), layered over the scene's appearance.
struct ObserverSettings {
    bool water = true, walls = true, floor = true, reflections = false, shadows = true;
    bool tiles = true;      // the pool's tile grout (off: plain walls and floor; the lane lines stay)
    bool planColors = true; // map editing's 2D view drawn as a chart (theme colors, flat light)
    int lighting = 0;       // 0 follows the scene, 1 indoor, 2 outdoor, 3 sterile
    float exposure = 1, brightness = 1, ambient = 1;
    int antialiasing = 1; // supersampling factor of the observer view and camera cards (1 = off)

    void resetLighting() {
        shadows = true;
        lighting = 0;
        exposure = brightness = ambient = 1;
    }

    // The scene's appearance with these overrides applied; only the observer view and camera cards use it.
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

// A camera framing for focus(): what to look at and from where. Host yaml presets overlay these defaults by name.
struct FocusPreset {
    std::string target = "landmark"; // course | vehicle | mechanism | mechanisms | landmark
    std::string mechanism, yawFrom = "landmark_facing";
    std::vector<std::string> mechanisms;
    glm::vec3 offset{0};
    float distance = 3.4f, distanceScale = 0, zOffset = 0, pitch = .28f, yaw = 0, yawOffset = 0;
    bool follow = true, labels = true;
};

// Copies the fields present in a YAML map onto the preset; absent keys keep their current value.
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

// The GL textures of one camera card (color and depth images) and its window state.
struct CardTexture {
    GLuint rgb = 0, depth = 0;
    int rgbWidth = 0, rgbHeight = 0, depthWidth = 0, depthHeight = 0;
    bool flipped = false;   // rendered (bottom-up) rather than decoded (top-down)
    bool open = true;       // its camera window is shown
    bool metaInline = true; // the camera / resolution line sits on the button row (else its own line)
};

// Dockable host windows. The part after ### is the window's identity in saved layouts; the title can change.
constexpr const char *kPoolView = "Pool view###pool_view", *kCourseMap = "Course map###course_map",
                     *kSceneSettings = "Scene settings###scene_settings", *kDisplay = "Display###display",
                     *kTfFrames = "TF frames###tf", *kHelp = "Controls & shortcuts###help",
                     *kMapObjects = "Map objects###map_objects", *kMapInspector = "Inspector###map_inspector";

std::string cameraWindowName(const SensorCamera &camera) {
    return camera.title + "###camera." + camera.id;
}

// Uploads an RGB8 image into `texture` (created on first use) and records its size in tw x th.
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

// ----------------------------------------------------------------------------------------------- app

// The viewer. Owns the window, renderer, ROS side and every window's state; one instance per process
// (run()). Its member functions follow below, grouped by topic.
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
    // The scene point drawn under `cursor` (pixels from the view's corner), from the frame's depth.
    std::optional<glm::vec3> depthPoint(const SensorView &, const rendering::RenderedFrame &, glm::vec2 cursor,
                                        glm::vec2 size);
    void demoTf();

    // --- UI
    void drawInterface(double time, float dt);
    void drawMenuBar();
    void drawViewMenu();
    void drawThemeMenu();
    void drawPoolMenu();
    void switchPool(const ScenarioPack &);
    void updatePoolSwitch();
    void drawPoolSwitchStatus(ImVec2 position);
    void drawWindowsMenu();
    void drawLayoutMenu();
    void drawCommandBar();
    void drawPoolView(double time, float dt);
    void drawCameraWindows();
    void drawMapWindow();
    void drawSceneSettingsWindow();
    void drawDisplayWindow();
    void drawTfWindow();
    void drawMapWindows();

    // --- workspaces: Operate (the robot) and Map (the prior map editor), each with its own window layout
    enum class Workspace { Operate, Map };
    void setWorkspace(Workspace);
    void applyMapLayout();
    float editMapButtonWidth() const;
    void drawEditMapButton();
    void drawMapButtons(); // Save / Done while editing the map
    void drawMapToolbar(float width);
    void drawUnsavedMapPrompt();

    // the Map workspace's 2D view: the scene from straight above (orthographic), panned and zoomed in x / y
    bool planActive() const {
        return workspace_ == Workspace::Map && planView_;
    }
    void fitPlan();
    void drawPlanFrame(const glm::mat4 &vp, glm::vec2 origin, glm::vec2 size); // the chart's canvas, rim, scale
    void handlePlanInput(bool hovered);
    SensorView planCamera(float aspect) const;

    void setupPriorMap();
    void drawHelpWindow();
    void handleShortcuts();

    // Toolbar items and the settings they open.
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
    // Where a window is listed in the Windows menu (Help has its own menu).
    enum class MenuSection { Panels, Cameras, Tools, Plots, None };
    struct WindowEntry {
        std::string key, name, label; // stable key ("panel.<id>", "camera.<id>", "map", ...), ImGui name, title
        bool *open;
        MenuSection section = MenuSection::Tools;
        const char *tooltip = nullptr; // Windows menu hint
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
    void drawFloatingEdges(); // hairline edges and soft shadows for undocked windows

    void setTheme(const std::string &id);

    // Interface scale: 0 follows the desktop (the window's content scale); the title bar always does.
    float uiScaleSetting_ = 1, pendingUiScale_ = -1;
    float resolvedUiScale() const;
    void setUiScale(float setting); // remembered in viewer.yaml
    void applyPendingUiScale();     // between frames: fonts and style at the new scale
    void applyPendingTheme();       // between frames: a theme chosen in the menu

    // Until the scenario is in: the mark pinging over rising water, and what the viewer is waiting for.
    void drawLoadingScreen();
    void releaseCardTextures(); // the camera cards' GL textures (a new scenario, shutdown)
    // Runs `work` (the scene's build) here while another thread keeps the loading screen moving: that thread has
    // the GL context and ImGui meanwhile, so `work` must touch neither. Only over the loading screen, and not in
    // capture runs (one thread, frame for frame).
    void animateLoadingWhile(const std::function<void()> &work);
    bool loadingOnThread() const;
    std::string loadingTopic_;   // the scenario topic the loading screen names (read on this thread, first)
    bool buildingShown_ = false; // the scenario arrived: one frame says "Building the scene" before the load
    float loadingLevel_ = 0;     // the loading screen's water, 0..1 of its height

    // File menu: the pool, the prior map's file, screenshots, folders, quit.
    void drawFileMenu();
    void drawFilePopups();
    void openPriorMap(const fs::path &file);
    void requestScreenshot();
    void notify(const std::string &message, bool error = false); // the pool view's status line, for a few seconds
    std::vector<fs::path> recentMaps_;
    char openMapPath_[512] = {};
    bool openMapPopup_ = false, reloadMapPopup_ = false;
    int screenshotCountdown_ = 0; // frames until the window is saved (the menu that asked has closed by then)
    fs::path screenshotPath_;
    std::string pendingTheme_;

    // The command center (VS Code's): a search box in the title bar (Ctrl+P) over everything the menus do.
    struct PaletteCommand {
        std::string group, label;
        bool checked = false;
        std::function<void()> run; // none: a note (how to finish a move, why it cannot run now)
        std::string fill;          // instead of running: the search box takes this, for the rest to be typed
    };
    std::string paletteFill_; // text the search box takes next frame (a command that needs more typing)

    // The robot from the keyboard: moves typed in the palette ("forward 0.5", "turn 30", "go 1 2 -1 90") and
    // keyboard driving (WASD, R / F, Q / E, one step a press). Enable / KILL stay off the keyboard.
    std::shared_ptr<panels::Motion> motion() const;
    std::shared_ptr<panels::Autonomy> autonomy() const;
    // Whether the robot takes a pose now (else why not, in `why`); `wait`: only for a moment (a mode change).
    bool commandable(std::string *why, bool *wait = nullptr) const;
    PoseTarget poseBase() const; // what a move starts from: the last command, else where the robot is
    void commandPose(const PoseTarget &);
    void driveWithKeys();
    void setDriving(bool on); // on only while the robot takes poses in Position control (else says why)
    bool driving_ = false;
    int driveStep_ = 2;              // index into kDriveMeters / kDriveDegrees
    std::optional<PoseTarget> sent_; // the last keyboard target, until the robot reports it as its command
    Clock::time_point sentAt_{};

    // The command center's commands, its title-bar search box and the results dropping from it.
    std::vector<PaletteCommand> paletteCommands();
    void drawCommandCenter(float titleScale);
    void drawCommandPalette();
    bool paletteOpen_ = false, paletteFocus_ = false, paletteTyping_ = false, paletteEnter_ = false;
    char paletteQuery_[128] = {};
    int paletteIndex_ = 0;
    ImVec2 paletteBox_{0, 0}, paletteBoxSize_{0, 0}; // the title bar's search box, the results drop from it

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

    // Layouts: built-in presets, named layouts, the session layout, the windows asked for on the command line.
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

    // --- state: configuration, window, renderer, ROS and the loaded scenario
    Options opt_;
    YAML::Node config_;
    fs::path configDir_;
    std::unique_ptr<Window> window_;
    std::unique_ptr<rendering::Renderer> renderer_;
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<RosSide> ros_;
    std::optional<Scenario> scenario_;
    std::unique_ptr<SceneModel> model_;

    // The course map's top-down images, baked once per scene model (top_down.hpp): the course's props in pool
    // coordinates, and the robot in its base frame (drawn turned to its heading).
    struct TopDownTexture {
        GLuint texture = 0;
        glm::vec2 low{0}, high{0};
    };
    TopDownTexture courseImage_, robotImage_;
    const SceneModel *topDownFor_ = nullptr;
    void bakeTopDownImages();

    StatusLights lights_;
    ThrusterVisuals thrusters_;
    std::string pendingScenario_;

    // View > Pool. A scenario from the simulator's topic is switched by its supervisor (sim.launch.py restarts the
    // simulator in the new pool and the topic brings the new scene); a scenario file (real robot, preview) is
    // resolved here, off the UI thread, and reloaded in this viewer only.
    struct PoolSwitch {
        std::vector<ScenarioPack> packs;
        bool fromTopic = false;
        std::string topicBase;                // <namespace>/simulator
        std::string supervisorState, message; // supervisor: running | switching | stopped | error
        bool messageError = false;
        std::string target, targetLabel; // the pool id being switched to (empty: none)
        Clock::time_point messageUntil{};
        std::thread worker;
        std::mutex mutex;
        bool done = false;
        std::string resolved, error;
    } poolSwitch_;
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
    GLuint logoTrident_ = 0, logoSonar_ = 0; // the title bar's mark (white masks, tinted)

    // settings
    Look look_;
    ObserverSettings observer_;
    // Map editing's own look: no water, even light without shadows or caustics (Sterile), so the floor reads plainly.
    ObserverSettings mapObserver_ = [] {
        ObserverSettings s;
        s.water = false;
        s.lighting = 3;
        s.shadows = false;
        return s;
    }();
    // The settings the 3D view uses now: map editing's own look in the Map workspace, else the observer's.
    ObserverSettings &viewSettings() {
        if (workspace_ != Workspace::Map)
            return observer_;
        mapObserver_.antialiasing = observer_.antialiasing; // a performance setting, shared
        return mapObserver_;
    }

    void drawMapLighting();
    bool openDepth_ = false;
    bool showTf_ = false, tfNames_ = true, tfTreeOpen_ = false, detections_ = false, showMpc_ = false,
         showThrust_ = false, demoMode_ = false;
    float thrustScale_ = .05f; // arrow meters per newton (host yaml thrust_arrows.meters_per_newton)
    float tfAxisLength_ = .12f, mapZoom_ = 1, toolbarLeft_ = 0;
    int toolbarOldMode_ = 0;

    // Course source: 0 auto (pack layout with simulator truth, mapping markers otherwise), 1 pack, 2 mapping.
    std::vector<MappingMarker> mappingMarkers_;
    int courseMode_ = 0;
    bool mappingGhost_ = false;
    char meshFilter_[64]{}; // Pool Viewer > Course > Meshes search

    // Prior map editor (Dead Reckoning in 3D): the robot's riptide_mapping config.yaml laid out in the pool.
    std::unique_ptr<PriorMapEditor> priorMap_;
    std::optional<glm::vec3> priorMapPointer_; // the scene point under the pointer last frame
    Workspace workspace_ = Workspace::Operate;
    std::string operateIni_, mapIni_; // the hidden workspace's layout (ini text)
    fs::path mapIniFile_;             // the Map workspace's layout between sessions
    bool pendingMapLayout_ = false, planView_ = true, planFitted_ = false, closeConfirmed_ = false,
         closePrompt_ = false;
    glm::vec2 planCenter_{0};                             // world x / y at the view's center
    float planHeight_ = 30;                               // meters shown top to bottom
    int planDragButton_ = -1;                             // the button panning the 2D view
    ImVec2 viewPos_{0, 0}, viewSize_{1, 1};               // the pool view's image rect this frame
    bool mapPausedSim_ = false, followBeforeMap_ = false; // what entering the Map workspace changed
    std::shared_ptr<panels::Simulation> simulation() const;
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

    void loadLogo();
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

// --------------------------------------------------------------------------------------------- setup

// Reads the host config, starts ROS (unless --demo), opens the window, restores the layout, theme and interface
// scale, creates the renderer, then loads --scenario or subscribes to the bridge's scenario topic.
App::App(const Options &options, int argc, char **argv) : opt_(options), argc_(argc), argv_(argv) {
    // Host config and the command line's display toggles
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
    thrustScale_ = lookup(config_, {"thrust_arrows", "meters_per_newton"}).as<float>(.05f);
    if (!std::isfinite(thrustScale_) || thrustScale_ <= 0)
        throw std::runtime_error("thrust_arrows.meters_per_newton must be positive");

    // ROS node (sim time unless only the estimate is shown) and the pose / detection sources
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

    // The window, with the viewer's own title bar unless the system one is asked for
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
    plots::install();
    loadLogo();
    const auto configHome = configDirectory();
    persist_ = opt_.frames == 0 && !configHome.empty();
    profiler_.keepInterval(opt_.profile); // only --profile reports intervals
    if (!configHome.empty()) {
        sessionIni_ = configHome / "viewer_layout.ini";
        mapIniFile_ = configHome / "map_layout.ini";
        layoutDir_ = configHome / "layouts";
        if (persist_ && fs::exists(mapIniFile_)) {
            std::ifstream in(mapIniFile_);
            std::stringstream text;
            text << in.rdbuf();
            mapIni_ = text.str();
        }
    }

    // Host config `prior_map: {config: <file>}`: the config.yaml the editor opens first (--prior-map overrides).
    priorMap_ = std::make_unique<PriorMapEditor>(
        persist_ ? configHome / "prior_map" : fs::path(),
        !opt_.priorMap.empty() ? opt_.priorMap : lookup(config_, {"prior_map", "config"}).as<std::string>(""));
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
    // themes are data (content/viewer/themes); a file that cannot be read is reported and skipped
    for (const auto &warning : loadThemes(contentDirectory() / "themes"))
        std::cerr << "nereus-viewer: theme: " << warning << '\n';
    std::string theme = opt_.theme;
    if (theme.empty() && persist_ && fs::exists(preferencesFile_))
        try {
            theme = YAML::LoadFile(preferencesFile_.string())["theme"].as<std::string>("");
        } catch (const std::exception &error) {
            std::cerr << "nereus-viewer: ignoring " << preferencesFile_ << ": " << error.what() << '\n';
        }
    if (persist_ && fs::exists(preferencesFile_)) // File > Open recent
        try {
            for (const auto &path : YAML::LoadFile(preferencesFile_.string())["recent_prior_maps"])
                if (fs::is_regular_file(path.as<std::string>()))
                    recentMaps_.push_back(path.as<std::string>());
        } catch (const std::exception &) {
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

    // Renderer: --shaders, else the build's shader folder, else share/nereus/shaders beside the executable
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

    // Scenario: --scenario now, else the bridge's latched topic (loaded between frames); the simulator supervisor's
    // status feeds File > Switch pool.
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
        poolSwitch_.fromTopic = true;
        poolSwitch_.topicBase = topic.substr(0, topic.rfind('/'));
        ros_->watchSupervisor(poolSwitch_.topicBase + "/supervisor", [this](const std::string &json) {
            try {
                const auto status = YAML::Load(json); // JSON is YAML
                auto &s = poolSwitch_;
                s.supervisorState = status["state"].as<std::string>("");
                const auto message = status["message"].as<std::string>("");
                if (s.supervisorState == "error" || s.supervisorState == "stopped") {
                    s.message =
                        s.supervisorState == "error" ? "Pool not switched: " + message : "Simulator: " + message;
                    s.messageError = true;
                    s.messageUntil = Clock::now() + std::chrono::seconds(12);
                    s.target.clear();
                } else if (s.supervisorState == "running" && !message.empty()) {
                    s.message = message; // e.g. the MPC's generated model is for the old pool's water
                    s.messageError = true;
                    s.messageUntil = Clock::now() + std::chrono::seconds(12);
                }
            } catch (const std::exception &error) {
                std::cerr << "nereus-viewer: ignoring simulator supervisor status: " << error.what() << '\n';
            }
        });
    }
}

// The logo (content/viewer/icons) as the window's icon, at every size.
void App::loadLogo() {
    const auto icons = contentDirectory() / "icons";
    std::vector<fs::path> sizes;
    for (const int size : {16, 24, 32, 48, 64, 128, 256})
        sizes.push_back(icons / ("nereus-" + std::to_string(size) + ".png"));
    window_->setIcon(sizes);
    // and in the title bar: the bare mark (no tile) as two white masks the bar tints, mipmapped to stay crisp small
    const auto mask = [&](const char *file, GLuint &texture) {
        int width = 0, height = 0;
        std::vector<unsigned char> rgba;
        if (!readPng(icons / file, width, height, rgba))
            return;
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
        glGenerateMipmap(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, 0);
    };
    mask("nereus-trident-128.png", logoTrident_);
    mask("nereus-sonar-128.png", logoSonar_);
}

// Teardown in dependency order: the layout while the panels exist, GL objects while the window's context
// lives, ROS last.
App::~App() {
    if (poolSwitch_.worker.joinable())
        poolSwitch_.worker.join();
    try {
        persistLayout(true); // before the panels (their window states) go
    } catch (const std::exception &error) {
        std::cerr << "nereus-viewer: layout not saved: " << error.what() << '\n';
    }
    plots::shutdown(); // its node goes before the panels' runtime and rclcpp
    panelRos_.stop();
    composition_.reset();
    model_.reset();
    releaseCardTextures();
    if (readFbo_)
        glDeleteFramebuffers(1, &readFbo_);
    if (drawFbo_)
        glDeleteFramebuffers(1, &drawFbo_);
    for (GLuint texture : {logoTrident_, logoSonar_, courseImage_.texture, robotImage_.texture})
        if (texture)
            glDeleteTextures(1, &texture);
    renderer_.reset();
    ros_.reset();
    window_.reset();
    if (node_) {
        node_.reset();
        rclcpp::shutdown();
    }
}

// --------------------------------------------------------------------------------------------- focus

// The host yaml's focus preset for `name`: `default`, then every matching key (a trailing * matches a prefix).
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

// Whether `name`'s preset has something to look at in this scenario (its landmark or mechanism exists).
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

// The world point `name` orbits: a mechanism frame plus offset, the mechanisms' slot centroid, a landmark, else
// the robot.
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

// Points the orbit camera at `name` with its preset's distance, pitch and yaw (course: the pool's center).
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

// --demo: places the robot 2 m in front of landmark `name`, facing it. The ui document's `previews` can set the
// depth, or put a named camera right over the landmark.
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

// ------------------------------------------------------------------------------------------ scenario

// Loads a scenario document (resolved JSON) unless it is the one already loaded: the scene model and the ROS
// attachments are built off the UI thread, then the cards, focus lists, panels and windows are reset for it.
void App::loadScenario(const std::string &json) {
    const std::size_t hash = std::hash<std::string>{}(json);
    if (hash == loadedHash_)
        return;
    animateLoadingWhile([&] { // the heavy part: packs, meshes, textures; no ImGui or GL in here
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

        // The scene model (meshes, textures) and this robot's ROS subscriptions
        SceneModelOptions options;
        options.config = config_;
        options.robotOnly = opt_.robotOnly;
        model_ = std::make_unique<SceneModel>(*scenario_, options, thrusters_, lights_);
        ros_->attach(*scenario_, config_, lights_, thrusters_, !demoMode_);
        loadMappingMarkers();
        setupPriorMap();
    });

    // Back on the UI thread: the observer look and a fresh card per camera
    look_.appearance = scenario_->appearance;
    look_.equipment = lookup(config_, {"equipment_visible"}).as<bool>(true);
    releaseCardTextures(); // the last scenario's cards
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
    // --demo: the host yaml's preview pose, or in front of the preview task
    if (demoMode_) {
        const auto p = lookup(config_, {"preview", "pose"});
        body_ = p ? pose(vec3(p), {0, 0, p[5].as<float>(0)}) : pose({3, -2, -.75f}, {0, 0, -.14f});
        const std::string task =
            !opt_.demoTask.empty() ? opt_.demoTask : lookup(config_, {"preview", "task"}).as<std::string>("");
        if (!task.empty() && scenario_->landmarks.count(task))
            previewPose(task);
    }

    // Panels (built once, with the first scenario)
    if (!composition_)
        buildPanels();

    // New windows (this scenario's cameras, the panels) start as configured, then as the saved layout has them.
    rememberDefaults();
    windowStates_.apply();
    if (!commandLineDone_)
        applyCommandLineWindows();

    // The initial camera: --focus, else the host yaml's initial_focus
    std::string initial =
        !opt_.initialFocus.empty() ? opt_.initialFocus : lookup(config_, {"initial_focus"}).as<std::string>("Vehicle");
    if (!focusable(initial))
        initial = "Vehicle";
    focus(initial);
    applyInitialView();
    status_ = demoMode_ ? "SCENE PREVIEW" : "WAITING FOR PHYSICS";
}

void App::releaseCardTextures() {
    for (auto &card : cards_) {
        if (card.rgb)
            glDeleteTextures(1, &card.rgb);
        if (card.depth)
            glDeleteTextures(1, &card.depth);
        card.rgb = card.depth = 0;
    }
}

// Builds the operator panel composition (--panels, else the host yaml's panels_config) and starts its ROS
// providers.
void App::buildPanels() {
    const std::string configured =
        opt_.panelsPath.empty()
            ? (configDir_ / lookup(config_, {"panels_config"}).as<std::string>("talos_uwrt_panels.yaml")).string()
            : opt_.panelsPath;
    const bool haveConfig = configured != "none" && fs::exists(configured);
    if (!haveConfig && configured != "none")
        std::cerr << "nereus-viewer: panel composition " << configured << " not found; panels disabled\n";

    // Panel types: the library's, the host's own toolbar items, the ROS-backed providers
    panels::registerPanels(registry_);
    registerHostItems();
    panelRos_.registerFactories(registry_);

    panels::Context context{trimSlashes(scenario_->ns), scenario_->mapFrame, demoMode_, ros_->useSimTime()};
    // Panel profile documents carry their scorecard schema under `ui` (run panel and sim.run provider).
    YAML::Node task(YAML::NodeType::Map);
    task["ui"] = YAML::Clone(scenario_->ui);
    context.documents.emplace("task", task);
    context.focus = [this](const std::string &name) { focus(name); };
    context.keyboardDriving = [this] { return driving_; };
    context.setKeyboardDriving = [this](bool on) { setDriving(on); };
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

    // The run tracker (scorecard), if the composition has one
    for (const auto &entry : composition_->providers())
        if (auto run = std::dynamic_pointer_cast<panels::Run>(entry.second)) {
            runTracking_ = run;
            break;
        }
    panelRos_.start();

    // Plots: their own node in the robot namespace, the panels' figures from these providers (no ROS in --demo).
    if (!demoMode_) {
        plots::Options options;
        options.robotNamespace = trimSlashes(scenario_->ns);
        options.useSimTime = ros_->useSimTime();
        if (!layoutDir_.empty())
            options.savedDir = layoutDir_.parent_path() / "plots";
        if (const char *home = std::getenv("HOME"))
            options.exportDir = fs::path(home) / "Documents" / "Nereus" / "plots";
        plots::configure(options, composition_->providers());
    }
}

// ----------------------------------------------------------------------------------------- per frame

// Scene time in seconds: the ROS clock (sim time under the simulator), or wall time since start in --demo.
double App::clockSeconds() const {
    return demoMode_ ? std::chrono::duration<double>(Clock::now() - start_).count() : ros_->now();
}

// Takes the latest robot pose and estimate from ROS; refocuses on the first pose and extends the trail.
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

        // The trail: a point every 6 cm, the last 1200 kept
        const glm::vec3 position(body_[3]);
        if (trail_.empty() || glm::distance(trail_.back(), position) > .06f) {
            trail_.push_back(position);
            if (trail_.size() > 1200)
                trail_.pop_front();
        }
    }
    status_ = ros_->status();
}

// --demo's TF tree: the scenario's frames placed on the preview pose (there is no ROS graph to read).
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

// Per-frame ROS capture: pose, TF (only while shown), detections, point clouds, MPC path and thrust, then the
// thruster animation.
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

// Low-passes the estimate's error against truth, for Follow on the estimate (see errorShift_).
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

// Whether the course comes from the mapping markers: Course > Mapping, or Auto without simulator truth.
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

// What the scene draws this frame: the robot's pose and animation state, and markers for the prior map, the
// mapping course, simulator props, projectiles and magnet-light LEDs.
VisualState App::buildState() {
    VisualState state;
    priorMap_->setPlan(planActive()); // 2D: stand-alone props without a mesh are badges, not boxes
    // The Map workspace draws the robot at a robot-frame origin (its start pose), else where it is (the simulator
    // is paused while the map is edited).
    const bool mapping = workspace_ == Workspace::Map;
    // level: position and heading only (a pitched or rolled robot says nothing about the map)
    const auto level = [](const glm::mat4 &m) {
        return glm::rotate(glm::translate(glm::mat4(1), glm::vec3(m[3])), std::atan2(m[0].y, m[0].x),
                           glm::vec3(0, 0, 1));
    };
    const glm::mat4 body = mapping ? priorMap_->robotStart().value_or(level(body_)) : body_;
    state.body = body;

    // Robot animation (rotor spin, status lights, claw) and the view's toggles
    for (const auto &rotor : thrusters_.rotors)
        state.rotorSpin.push_back(rotor.transform());
    const double now = clockSeconds();
    for (const auto &light : lights_.lights)
        state.lightColor.push_back(light.state.color(now));
    state.claw = demoMode_ ? std::array<float, 2>{0.f, 0.f} : ros_->claw;
    state.showEquipment = look_.equipment;
    state.showWalls = viewSettings().walls;
    state.showFloor = viewSettings().floor;
    if (robotGhost_ && haveEstimate_ && !mapping)
        state.ghostBody = estimateBody_;

    // The prior map being edited, in place of the course when the editor hides it.
    const bool priorMapOnly = priorMap_->hidesCourse();
    priorMap_->addMarkers(state.markers);
    const auto payloads = lookup(config_, {"payloads", "loaded_namespaces"});
    if (demoMode_) {
        for (const auto &entry : payloads)
            for (const auto &mount : model_->payloadMounts(entry.second.as<std::string>()))
                state.loadedPayloads.push_back(body * mount);
        state.showCourse = !priorMapOnly;
        return state;
    }

    // Mapping course (RViz markers at the mapping frames): the course itself on a real robot, or a translucent
    // ghost of the mapping estimate over the simulator's course.
    const bool mappingCourse = courseFromMapping();
    state.showCourse = !mappingCourse && !priorMapOnly;
    if (!priorMapOnly && (mappingCourse || (mappingGhost_ && ros_->truthActive())))
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

    // Simulator props (marker meshes), unless the mapped course or the prior map stands in for them
    for (const auto &[key, record] : ros_->props) {
        if (mappingCourse || priorMapOnly) // simulator props duplicate the mapped table items
            break;
        if (record.mesh.empty())
            continue;
        MarkerDraw draw;
        draw.mesh = record.mesh;
        draw.world = record.attached ? body * record.pose : record.pose;
        draw.scale = {float(record.marker.scale.x), float(record.marker.scale.y), float(record.marker.scale.z)};
        state.markers.push_back(std::move(draw));
    }

    // Projectiles: loaded rounds ride on their launcher's payload mounts, the rest are drawn where the marker says
    for (const auto &[key, record] : ros_->projectiles) {
        const auto ns = lookup(payloads, {key.first.c_str()});
        if (ns) {
            // Loaded rounds follow this frame's robot pose like the launcher, not the sampled marker pose.
            const auto mounts = model_->payloadMounts(ns.as<std::string>());
            if (key.second >= 0 && std::size_t(key.second) < mounts.size()) {
                state.loadedPayloads.push_back(body * mounts[std::size_t(key.second)]);
                continue;
            }
        }
        if (record.mesh.empty())
            continue;
        MarkerDraw draw;
        draw.mesh = record.mesh;
        draw.world = record.attached ? body * record.pose : record.pose;
        draw.scale = {float(record.marker.scale.x), float(record.marker.scale.y), float(record.marker.scale.z)};
        state.markers.push_back(std::move(draw));
    }

    // Magnet lights: a green / red latch on the pack's indicator LED, else an emissive box
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
        draw.world = record.attached ? body * record.pose : record.pose;
        draw.scale = {float(record.marker.scale.x), float(record.marker.scale.y), float(record.marker.scale.z)};
        state.markers.push_back(std::move(draw));
    }
    return state;
}

// --------------------------------------------------------------------------------------------- input

// Mouse and keyboard on the 3D view. Free fly (mode 1): a click captures the mouse, WASD / Space / Shift move,
// Ctrl is faster. Orbit (mode 0): left drag turns, right / middle / Shift drag pans, the wheel zooms.
void App::handleViewInput(float dt, bool hovered, float viewportHeight) {
    if (planActive()) {
        handlePlanInput(hovered);
        return;
    }
    auto &io = ImGui::GetIO();
    GLFWwindow *glfw = window_->handle();

    // Free fly: Escape, another mode or losing focus releases the mouse; a click in the view captures it
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

    // Orbit: the button that started a drag owns it until released
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

// This frame's observer camera, after input: the 2D plan, a sensor camera (mode 2+), free fly or the orbit.
SensorView App::viewFor(float aspect, float dt, bool hovered, float viewportHeight, bool &) {
    handleViewInput(dt, hovered, viewportHeight);
    if (planActive())
        return planCamera(aspect);
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

std::optional<glm::vec3> App::depthPoint(const SensorView &view, const rendering::RenderedFrame &frame,
                                         glm::vec2 cursor, glm::vec2 size) {
    const auto uv = cursor / size;
    if (uv.x < 0 || uv.x >= 1 || uv.y < 0 || uv.y >= 1 || !frame.depth_texture)
        return std::nullopt;

    // Read one depth texel through readFbo_, then restore the caller's read framebuffer
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
    glm::vec3 point;
    if (!host::depthPoint(view.projection * view.view, uv, depth, point))
        return std::nullopt;
    return point;
}

// F over the orbit view: re-centers the orbit on what is under the cursor (an overlay line, else the drawn depth,
// else the plane through the current target) without moving the camera.
void App::focusAtCursor(const SensorView &view, const rendering::RenderedFrame &frame, ImVec2 origin, float width,
                        float height) {
    if (mode_ != 0)
        return;
    const auto mouse = ImGui::GetIO().MousePos;
    const glm::vec2 cursor(mouse.x - origin.x, mouse.y - origin.y), size(width, height);

    // Overlay geometry under the cursor: TF axes, detections, the MPC path
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
        if (uv.x < 0 || uv.x >= 1 || uv.y < 0 || uv.y >= 1)
            return;
        if (const auto drawn = depthPoint(view, frame, cursor, size))
            point = *drawn;
        else if (!focusPlanePoint(view.projection * view.view, view.eye, target_, uv, point))
            return;
    }

    // The eye stays put: the new orbit is the offset from the point to the eye
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
    statusChip(text.c_str(), tint);
}

void App::sectionHeading(const char *text) {
    ImGui::PushFont(window_->small);
    ImGui::TextColored(palette().muted, "%s", text);
    ImGui::PopFont();
}

// The run's elapsed time as mm:ss.s (from the run panel's score).
std::string App::runTime() const {
    const double seconds = runScore_ && runScore_["elapsed"] ? runScore_["elapsed"].as<double>() : 0.;
    char value[64];
    std::snprintf(value, sizeof(value), "%02d:%04.1f", int(seconds) / 60, std::fmod(seconds, 60.));
    return value;
}

// -------------------------------------------------------------------------------------- camera cards

// One camera window's contents: source controls, the image fitted to the window, and the feed status.
void App::drawCameraCard(std::size_t index) {
    auto &feed = ros_->feeds[index];
    auto &tex = cards_[index];
    const auto &camera = *feed.camera;
    // New images from the feed into the card's textures
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
    // RGB / Depth, in the simulator Truth / ROS (the card's source: this viewer's render at the truth pose, or
    // the images the bridge publishes to the robot stack), then "Main view" to show this camera in the pool view.
    int image = feed.wantDepth ? 1 : 0;
    if (pins::Switch("Image##image", &image, {"RGB", "Depth"})) {
        feed.wantDepth = image == 1;
        ros_->refreshCameras();
    }
    const bool sourceToggle = canLocal && !demoMode_;
    if (sourceToggle) {
        ImGui::SameLine();
        int source = feed.rosMode ? 1 : 0;
        if (pins::Switch("Source##source", &source, {"Truth", "ROS"})) {
            feed.rosMode = source == 1;
            ros_->refreshCameras();
            cardDue_[index] = 0; // render the local view immediately when switching back
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Truth: rendered by this viewer at the truth pose (no sensor noise, no bridge latency).\n"
                              "ROS: the images the bridge publishes, as the robot stack receives them.");
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
    // the camera, its resolution, and the live rate of the images the bridge publishes (when that is the source)
    std::string model =
        camera.model + "  /  " + std::to_string(camera.k.width) + " x " + std::to_string(camera.k.height);
    if (!demoMode_ && !local && feed.connected()) {
        char rate[24];
        std::snprintf(rate, sizeof(rate), "  /  %.1f Hz", feed.hz);
        model += rate;
    }
    const float modelWidth = ImGui::CalcTextSize(model.c_str()).x;
    // the model at the right end of the button row, only where the row leaves room for it after the buttons
    const float rowEnd = ImGui::GetItemRectMax().x - ImGui::GetWindowPos().x; // the last button's right edge
    // (on its own line under the buttons when the row is too narrow: the resolution is never dropped)
    // with some hysteresis, so a width right at the edge cannot flip it every frame
    const float room = ImGui::GetWindowContentRegionMax().x - rowEnd;
    tex.metaInline = room >= modelWidth + ui(tex.metaInline ? 16 : 40);
    if (tex.metaInline)
        ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - modelWidth); // takes the buttons' text baseline
    ImGui::TextColored(palette().muted, "%s", model.c_str());
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Preview: %d x %d%s", depthShown ? tex.depthWidth : tex.rgbWidth,
                          depthShown ? tex.depthHeight : tex.rgbHeight,
                          !demoMode_ && !canLocal ? "\nROS image topic (no simulator truth pose)" : "");
    ImGui::PopFont();

    // The image, as large as the window allows at the camera's aspect, centered
    const auto available = ImGui::GetContentRegionAvail();
    const float aspect = float(camera.k.width) / float(camera.k.height);
    const float w = std::max(16.f, std::min(available.x, available.y * aspect));
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
    const ImVec2 below = ImGui::GetCursorScreenPos();
    // No overlay on the image: the source is the Truth / ROS switch, the rate is in the header. While there is
    // nothing to show, the reason sits centered on a veil in the board's colors.
    beginSurface(Surface::Board);
    auto *draw = ImGui::GetWindowDrawList();
    const ImVec4 strip = palette().bar;
    draw->PushClipRect(pos, {pos.x + w, pos.y + h}, true);
    if (!ready || !texture) {
        draw->AddRectFilled(pos, {pos.x + w, pos.y + h},
                            ImGui::GetColorU32(ImVec4(strip.x, strip.y, strip.z, texture ? .7f : 1.f)));
        const char *waiting = !ready                 ? "Waiting for pose"
                              : local && !depthShown ? "Rendering the local view"
                                                     : "Waiting for camera images";
        const ImVec2 size = ImGui::CalcTextSize(waiting);
        draw->AddText({pos.x + (w - size.x) * .5f, pos.y + (h - size.y) * .5f}, ImGui::GetColorU32(palette().muted),
                      waiting);
    }
    draw->PopClipRect();
    endSurface();
    ImGui::SetCursorScreenPos(below);
    ImGui::Dummy({0, 0});
    ImGui::PopID();
}

// ------------------------------------------------------------------------------------ scene settings

// Scene settings > Water appearance: the observer view's water tint, haze and absorption, with three presets.
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

    // The sliders, four to a row
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

// ---------------------------------------------------------------------------------------- course map

// Scroll zooms, drag pans, clicking a task focuses the pool view on it. Compact (a small window): smaller labels,
// the ui document's minimap labels, and landmarks marked hidden_in_minimap unlabeled.
void App::drawCourseMap(float width, float height, bool compact) {
    const auto &s = *scenario_;
    const ImVec2 a = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("course canvas", {width, height});
    const bool hovered = ImGui::IsItemHovered();
    auto &io = ImGui::GetIO();

    // Zoom (wheel, 1-8x) and pan (left drag)
    if (hovered) {
        mapZoom_ = glm::clamp(mapZoom_ * std::exp(io.MouseWheel * .15f), 1.f, 8.f);
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            mapPan_.x += io.MouseDelta.x;
            mapPan_.y += io.MouseDelta.y;
        }
    }

    const float length = s.poolLength, poolWidth = s.poolWidth, margin = ui(28);
    // the pool lengthwise across the well, or turned a quarter (its x up) when that draws it larger (a tall well)
    const float across = std::min((width - margin) / length, (height - margin) / poolWidth),
                upright = std::min((height - margin) / length, (width - margin) / poolWidth);
    const bool turned = upright > across * 1.15f;
    const float scale = std::max(across, turned ? upright : 0.f) * mapZoom_;
    const ImVec2 center(a.x + width / 2 + mapPan_.x, a.y + height / 2 + mapPan_.y);
    auto poolXY = [&](glm::vec2 p) {
        const glm::vec2 c = (p - glm::vec2(length, poolWidth) * .5f) * scale;
        return turned ? ImVec2(center.x - c.y, center.y - c.x) : ImVec2(center.x + c.x, center.y - c.y);
    };
    auto xy = [&](glm::vec3 p) { return poolXY(glm::vec2(s.worldToPool * glm::vec4(p, 1))); };

    // The canvas and the pool's floor
    auto *d = ImGui::GetWindowDrawList();
    const auto chart = planPalette(); // the same chart colors as map editing's 2D view, from the theme
    const ImU32 lineColor = ImGui::GetColorU32(chart.line);
    d->AddRectFilled(a, {a.x + width, a.y + height}, ImGui::GetColorU32(chart.canvas), ImGui::GetStyle().FrameRounding);
    d->PushClipRect(a, {a.x + width, a.y + height}, true);
    const ImVec2 cornerA = poolXY({0, 0}), cornerB = poolXY({length, poolWidth});
    const ImVec2 poolMin(std::min(cornerA.x, cornerB.x), std::min(cornerA.y, cornerB.y)),
        poolMax(std::max(cornerA.x, cornerB.x), std::max(cornerA.y, cornerB.y));
    d->AddRectFilled(poolMin, poolMax, ImGui::GetColorU32(chart.floor));

    // The pool's floor markings when it declares any, else a 5 m grid for scale.
    bool marked = false;
    if (model_)
        for (const auto &stripe : model_->pack().poolStripes())
            if (stripe.side == rendering::PoolSide::Floor) {
                d->AddLine(poolXY({stripe.from.x(), stripe.from.y()}), poolXY({stripe.to.x(), stripe.to.y()}),
                           lineColor, std::max(1.f, stripe.width * scale));
                marked = true;
            }
    if (!marked) {
        for (int i = 0; i <= length; i += 5)
            d->AddLine(poolXY({float(i), 0}), poolXY({float(i), poolWidth}), lineColor);
        for (int i = 0; i <= poolWidth; i += 5)
            d->AddLine(poolXY({0, float(i)}), poolXY({length, float(i)}), lineColor);
    }
    d->AddRect(poolMin, poolMax, ImGui::GetColorU32(chart.rim), 0, 0, 2);

    const ImVec4 accent = palette().accent;
    // The course's props as their baked top-down image (as map editing's 2D view shows them), under the labels.
    if (model_ && topDownFor_ != model_.get())
        bakeTopDownImages();
    if (courseImage_.texture) {
        const glm::vec2 lo = courseImage_.low, hi = courseImage_.high; // image row 0 is the top (pool y high)
        d->AddImageQuad(textureID(courseImage_.texture), poolXY({lo.x, hi.y}), poolXY(hi), poolXY({hi.x, lo.y}),
                        poolXY(lo), {0, 0}, {1, 0}, {1, 1}, {0, 1});
    }

    // The robot's trail
    for (std::size_t i = 1; i < trail_.size(); ++i)
        d->AddLine(xy(trail_[i - 1]), xy(trail_[i]), ImGui::GetColorU32(ImVec4(accent.x, accent.y, accent.z, .6f)),
                   1.5f);

    // Labels take the first free spot beside their dot (right, then left, below then above), clear of the other
    // labels, the dots and the vehicle; one with no free spot is left out (hover the dot for its name).
    const auto vehicle = xy(glm::vec3(body_[3]));
    std::vector<std::pair<ImVec2, ImVec2>> taken{
        {{vehicle.x - ui(8), vehicle.y - ui(8)}, {vehicle.x + ui(8), vehicle.y + ui(8)}}};
    const ImVec2 robotText = window_->small->CalcTextSizeA(window_->small->FontSize, 1e9f, 0, s.robotId.c_str());
    taken.push_back(
        {{vehicle.x + ui(8), vehicle.y - ui(15)}, {vehicle.x + ui(8) + robotText.x, vehicle.y - ui(15) + robotText.y}});
    for (const auto &entry : s.ui["map_landmarks"]) {
        const auto found =
            s.landmarks.find(entry.IsScalar() ? entry.as<std::string>() : entry["name"].as<std::string>());
        if (found != s.landmarks.end()) {
            const auto p = xy(glm::vec3(found->second.world[3]));
            taken.push_back({{p.x - ui(6), p.y - ui(6)}, {p.x + ui(6), p.y + ui(6)}});
        }
    }
    const auto free = [&](ImVec2 min, ImVec2 max) {
        return std::none_of(taken.begin(), taken.end(), [&](const auto &r) {
            return min.x < r.second.x && max.x > r.first.x && min.y < r.second.y && max.y > r.first.y;
        });
    };

    // Each map landmark: its label, a tooltip on hover, and a click (not a drag) to focus on it
    for (const auto &entry : s.ui["map_landmarks"]) {
        const std::string key = entry.IsScalar() ? entry.as<std::string>() : entry["name"].as<std::string>();
        const std::string label = entry.IsScalar() ? key : entry["label"].as<std::string>(key);
        const bool hiddenInMinimap = !entry.IsScalar() && entry["hidden_in_minimap"].as<bool>(false);
        const bool relabelMinimap = !entry.IsScalar() && entry["minimap_label"];
        const auto found = s.landmarks.find(key);
        if (found == s.landmarks.end())
            continue;
        const auto p = xy(glm::vec3(found->second.world[3]));
        // no marker: the prop draws itself (the baked image); its label sits beside it and the spot takes the click
        if (!compact || !hiddenInMinimap) {
            ImFont *font = compact ? window_->small : window_->normal;
            const std::string text = compact && relabelMinimap ? entry["minimap_label"].as<std::string>() : key;
            const ImVec2 size = font->CalcTextSizeA(font->FontSize, 1e9f, 0, text.c_str());
            const float gap = ui(7);
            for (const ImVec2 at :
                 {ImVec2(p.x + gap, p.y + ui(2)), ImVec2(p.x + gap, p.y - ui(2) - size.y),
                  ImVec2(p.x - gap - size.x, p.y + ui(2)), ImVec2(p.x - gap - size.x, p.y - ui(2) - size.y)})
                if (free(at, {at.x + size.x, at.y + size.y})) {
                    taken.push_back({at, {at.x + size.x, at.y + size.y}});
                    d->AddText(font, font->FontSize, at, color(palette().text), text.c_str());
                    break;
                }
        }
        if (hovered && std::hypot(io.MousePos.x - p.x, io.MousePos.y - p.y) < ui(12))
            ImGui::SetTooltip("%s: click to focus the view", label.c_str());
        if (hovered && ImGui::IsMouseReleased(ImGuiMouseButton_Left) &&
            ImGui::GetMouseDragDelta().x * ImGui::GetMouseDragDelta().x +
                    ImGui::GetMouseDragDelta().y * ImGui::GetMouseDragDelta().y <
                9 &&
            std::hypot(io.MousePos.x - p.x, io.MousePos.y - p.y) < ui(12))
            focus(key);
    }

    // The robot: its top-down image turned to its heading (else a dot and a heading line), and its name
    const auto p = xy(glm::vec3(body_[3]));
    const ImU32 robot = ImGui::GetColorU32(palette().warn); // the vehicle's name: the theme's warm status color
    if (robotImage_.texture) {                              // the robot's own top-down image, turned to its heading
        const glm::vec2 lo = robotImage_.low, hi = robotImage_.high;
        const auto at = [&](float x, float y) { return xy(glm::vec3(body_ * glm::vec4(x, y, 0, 1))); };
        d->AddImageQuad(textureID(robotImage_.texture), at(lo.x, hi.y), at(hi.x, hi.y), at(hi.x, lo.y), at(lo.x, lo.y),
                        {0, 0}, {1, 0}, {1, 1}, {0, 1});
        d->AddCircle(p, ui(8), robot, 0, ui(1.5f)); // findable at the whole pool's scale
    } else {
        d->AddCircleFilled(p, 6, robot);
        d->AddLine(p, xy(glm::vec3(body_[3]) + glm::vec3(body_[0]) * 1.3f), robot, 3);
    }
    d->AddText(window_->small, window_->small->FontSize, {p.x + ui(8), p.y - ui(15)}, robot, s.robotId.c_str());
    d->PopClipRect();
}

// Bakes the course map's top-down images (once per scene model): the task props in pool coordinates (as placed at
// reset), and the robot's visuals in its base frame.
void App::bakeTopDownImages() {
    topDownFor_ = model_.get();
    // each image with its own smaller sizes (topDownLevels: thin props stay solid and outlined when drawn small)
    const auto upload = [](TopDownImage image, TopDownTexture &out) {
        if (out.texture)
            glDeleteTextures(1, &out.texture);
        out = {};
        const auto levels = topDownLevels(std::move(image));
        if (levels.empty())
            return;
        glGenTextures(1, &out.texture);
        glBindTexture(GL_TEXTURE_2D, out.texture);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        for (std::size_t i = 0; i < levels.size(); ++i)
            glTexImage2D(GL_TEXTURE_2D, GLint(i), GL_RGBA, levels[i].width, levels[i].height, 0, GL_RGBA,
                         GL_UNSIGNED_BYTE, levels[i].rgba.data());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, GLint(levels.size() - 1));
        const auto &image0 = levels.front();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glBindTexture(GL_TEXTURE_2D, 0);
        out.low = image0.low;
        out.high = image0.high;
    };

    if (!model_ || !scenario_)
        return;
    const auto &pack = model_->pack();
    const auto &instances = pack.staticScene().instances;
    const auto &sources = pack.staticSources();
    const auto toGlm = [](const Eigen::Matrix4f &m) {
        glm::mat4 out;
        for (int c = 0; c < 4; ++c)
            for (int r = 0; r < 4; ++r)
                out[c][r] = m(r, c);
        return out;
    };

    // The course: visible task instances and the props at their reset poses, in pool coordinates
    std::vector<TopDownPart> course;
    for (std::size_t i = 0; i < instances.size() && i < sources.size(); ++i)
        if (sources[i].role == "task" && instances[i].mesh && instances[i].visible)
            course.push_back({instances[i].mesh, scenario_->worldToPool * toGlm(instances[i].transform)});
    for (const auto &prop : pack.propVisuals())
        if (prop.mesh)
            course.push_back({prop.mesh, scenario_->worldToPool * toGlm(prop.world_from_asset_at_reset.cast<float>())});
    const float margin = 1;
    const auto courseImage = bakeTopDown(course, {-margin, -margin},
                                         {scenario_->poolLength + margin, scenario_->poolWidth + margin}, 48, readPng);
    upload(courseImage, courseImage_);

    // the robot: its visuals in base_link (root placed as the pack's convention puts it under base_link)
    const glm::mat4 baseFromRoot = toGlm(model_->worldFromRoot(glm::mat4(1)).cast<float>());
    std::vector<TopDownPart> robot;
    for (const auto &visual : pack.robotVisuals())
        if (visual.mesh)
            robot.push_back({visual.mesh, baseFromRoot * toGlm(visual.rootFromAsset().cast<float>())});
    const auto [low, high] = topDownBounds(robot);
    if (low.x < high.x)
        upload(bakeTopDown(robot, low - glm::vec2(.02f), high + glm::vec2(.02f), 400, readPng, 1024), robotImage_);
}

// The Course map window: the hint line and Fit pool above the map, which fills the rest.
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
        if (ImGui::Button("Fit pool")) {
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

// ---------------------------------------------------------------------------------- settings windows

// Scene settings contents: the scenario's mechanism buttons (simulator only), then the observer look's Lighting
// and Water tabs.
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

// The toolbar's Scene toggle; its window opens anchored under it.
void App::toolbarSceneSettings() {
    windowToggle("Scene", &sceneOpen_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Scene settings: mechanism buttons, lighting and water appearance");
    sceneAnchor_ = {ImGui::GetItemRectMin().x, ImGui::GetItemRectMax().y + ImGui::GetStyle().ItemSpacing.y};
}

// The Scene settings window. When it is closed but has pinned controls, its contents are drawn unseen so the
// pins still work.
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

// The toolbar's Display toggle; its window opens anchored under it.
void App::toolbarPoolViewer() {
    windowToggle("Display", &displayOpen_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("What this view shows: pool, course source, estimate ghost, viewer lighting");
    displayAnchor_ = {ImGui::GetItemRectMin().x, ImGui::GetItemRectMax().y + ImGui::GetStyle().ItemSpacing.y};
}

// The Display window (drawn unseen for its pins while closed, as Scene settings).
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
    pins::Checkbox("Pool tiles", &observer_.tiles);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("The tile grout on the walls and floor (this view only); the lane lines stay.");

    if (!model_->pack().equipmentInstances().empty()) {
        pins::Checkbox("AprilTag board", &look_.equipment);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("The equipment pack (calibration board) in this view; camera cards always show it.");
    }

    if (!mappingMarkers_.empty() && !demoMode_) {
        sectionTitle("Course");
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
        sectionTitle("Localization estimate");
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

    sectionTitle("Viewer lighting");
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

// ------------------------------------------------------------------------------------------- toolbar

// Switches the pool view's camera (0 orbit, 1 free, 2+ a sensor camera). Free fly starts from the view shown.
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

// A toolbar dropdown that names what it chooses: "Camera \u00b7 Orbit". Returns the chosen index (or -1).
static int labeledCombo(const char *id, const char *what, const std::vector<std::string> &options, int current) {
    const std::string preview = std::string(what) + "  \u00b7  " +
                                (current >= 0 && current < int(options.size()) ? options[std::size_t(current)] : "");
    const float width = ImGui::CalcTextSize(preview.c_str()).x + 2 * ImGui::GetStyle().FramePadding.x +
                        ImGui::GetFrameHeight() + ImGui::GetStyle().ItemInnerSpacing.x;
    sameLineIfFits(width);
    ImGui::SetNextItemWidth(width);
    int chosen = -1;
    if (ImGui::BeginCombo(id, preview.c_str())) {
        for (int i = 0; i < int(options.size()); ++i)
            if (ImGui::Selectable(options[std::size_t(i)].c_str(), i == current))
                chosen = i;
        ImGui::EndCombo();
    }
    return chosen;
}

// Toolbar items: each draws its control and tooltip; window toggles also remember where their window anchors.
// Camera: orbit, free camera, or one of the robot's cameras.
void App::toolbarView() {
    const auto &s = *scenario_;
    toolbarOldMode_ = mode_;
    std::vector<std::string> views{"Orbit", "Free camera"};
    for (const auto &camera : s.cameras)
        views.push_back(camera.id);
    if (const int mode = labeledCombo("##view", "Camera", views, mode_); mode >= 0)
        setViewMode(mode);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Camera: orbit, free camera (WASD), or look through a robot camera");
}

void App::toolbarFocus() {
    if (const int chosen = labeledCombo("##focus", "Focus", focusNames_, selectedFocus_); chosen >= 0) {
        selectedFocus_ = chosen;
        focus(focusNames_.at(std::size_t(selectedFocus_)));
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Focus: jump the camera to the vehicle, a mechanism or a course element");
}

// Follow is off (and disabled) while the focus preset does not follow.
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

// TF: the chip shows the axes, its arrow opens the TF frames window.
void App::toolbarTf() {
    if (toggleChipWithMenu("TF", &showTf_, "tf_settings"))
        tfOpen_ = !tfOpen_;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("TF frames: names, axis length and the frame tree");
    tfAnchor_ = {ImGui::GetItemRectMin().x, ImGui::GetItemRectMax().y + ImGui::GetStyle().ItemSpacing.y};
}

// ------------------------------------------------------------------------------ map and tool windows

// The prior map editor's pool: the scenario's pool and floor lines, and where the scenario has the map (the
// world frame is the map: its origin is the calibration board's tag), which the editor starts from.
void App::setupPriorMap() {
    const auto &s = *scenario_;
    PriorMapEditor::Pool pool;
    pool.id = s.poolId;
    pool.poolToWorld = s.poolToWorld;
    pool.length = s.poolLength;
    pool.width = s.poolWidth;
    pool.waterLevel = s.waterLevel;
    for (const auto &stripe : model_->pack().poolStripes())
        if (stripe.side == rendering::PoolSide::Floor)
            pool.lines.push_back({stripe.from.x(), stripe.from.y(), stripe.to.x(), stripe.to.y()});

    // The scenario's world origin (the tag) in pool coordinates, and the wall the tag faces
    const glm::mat4 &map = s.worldToPool;
    auto &origin = pool.scenarioOrigin;
    origin.x = map[3].x;
    origin.y = map[3].y;
    origin.z = map[3].z;
    const double yaw = std::atan2(map[0].y, map[0].x) * 180 / M_PI;
    const int quarter = (int(std::lround(yaw / 90)) % 4 + 4) % 4; // the wall the tag is on faces this way
    origin.basePhi = quarter * 90;
    origin.yawOffset = prior_map::wrapDegrees(yaw - origin.basePhi);
    origin.wall = "WSEN"[quarter];
    priorMap_->setPool(pool);

    // Prop meshes: the RViz course markers (also in demo mode, when the packages resolve).
    auto meshes = mappingMarkers_;
    if (meshes.empty())
        if (const auto cfg = lookup(config_, {"mapping_markers"}))
            try {
                const auto share = [](const std::string &package) {
                    return fs::path(ament_index_cpp::get_package_share_directory(package));
                };
                fs::path file = cfg["config"].as<std::string>();
                if (!file.is_absolute() && !fs::exists(file)) {
                    const auto package = file.begin()->string();
                    file = share(package) / file.lexically_relative(package);
                }
                meshes = host::loadMappingMarkers(file, share, cfg["meshes"].as<std::string>(""));
            } catch (const std::exception &error) {
                std::cerr << "nereus-viewer: prior map props drawn as boxes: " << error.what() << '\n';
            }
    for (auto &marker : meshes)
        marker.visible = true; // the course's hidden list is about the live course, not the prior map

    // What a click on a prop hits: its mesh's bounds through the marker pose, in the prop's frame.
    std::map<std::string, PriorMapEditor::Extent> extents;
    for (const auto &marker : meshes)
        if (const auto mesh = model_->mesh(marker.path)) {
            PriorMapEditor::Extent extent{glm::vec3(1e9f), glm::vec3(-1e9f)};
            for (int corner = 0; corner < 8; ++corner) {
                const glm::vec3 p(corner & 1 ? mesh->maximum.x() : mesh->minimum.x(),
                                  corner & 2 ? mesh->maximum.y() : mesh->minimum.y(),
                                  corner & 4 ? mesh->maximum.z() : mesh->minimum.z());
                const glm::vec3 q(marker.local * glm::vec4(p, 1));
                extent.low = glm::min(extent.low, q);
                extent.high = glm::max(extent.high, q);
            }
            extents[marker.frame] = extent;
        }
    priorMap_->setExtents(std::move(extents));
    priorMap_->setMeshes(std::move(meshes));
}

// The prior map editor's Map objects and Inspector windows (Map workspace).
void App::drawMapWindows() {
    focusIfRequested(kMapObjects);
    priorMap_->drawObjectsWindow(kMapObjects, [this] { windowContextMenu("map_objects"); });
    focusIfRequested(kMapInspector);
    priorMap_->drawInspectorWindow(kMapInspector, [this] { windowContextMenu("map_inspector"); });
}

// The TF frames window: toggles, axis length and the frame tree. tfTreeOpen_ tells step() to capture TF.
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

// Detection options (the Detections chip's menu; the sidebar panel form adds the enable toggle): placement,
// DELETEALL handling and point clouds.
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
    sectionTitle("Point clouds");
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

// --demo: a dropdown of the preview tasks; picking one moves the robot in front of it.
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

// The toolbar row: the composition's items, pinned windows and controls, then the + (customize) button.
// `oldMode` returns the camera mode from before the toolbar's items changed it this frame.
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
            sectionTitle("Legend");
            // a swatch drawn in each style, the words in the theme's text color
            const auto legend = [](ImU32 swatch, bool dashed, const char *text) {
                const ImVec2 at = ImGui::GetCursorScreenPos();
                const float h = ImGui::GetTextLineHeight(), w = ui(22), y = at.y + h * .5f;
                auto *d = ImGui::GetWindowDrawList();
                for (float x = 0; x < w; x += dashed ? ui(6) : w)
                    d->AddLine({at.x + x, y}, {at.x + std::min(w, x + (dashed ? ui(3.5f) : w)), y}, swatch, ui(3));
                ImGui::Dummy({w + ui(6), h});
                ImGui::SameLine(0, 0);
                ImGui::TextWrapped("%s", text);
            };
            legend(ImGui::GetColorU32(palette().text), false, "Solid: simulator truth placement");
            legend(IM_COL32(64, 230, 255, 242), false, "Cyan outline: estimate (TF), when both are shown");
            legend(ImGui::GetColorU32(palette().muted), true, "Dim dashed: approximate estimate (TF lagged)");
        });
}

// ----------------------------------------------------------------------------------------- interface

// The frame: menu bar and command bar (fixed), then the dock space holding the pool view and every window.
void App::drawInterface(double time, float dt) {
    pins::newFrame();
    plots::frame();
    handleShortcuts();
    driveWithKeys();
    drawMenuBar();
    drawCommandBar();
    auto *viewport = ImGui::GetMainViewport();

    // Layout and workspace requests (menus, commands, the command line), once a scenario is in
    if (!pendingPreset_.empty() && scenario_) {
        applyPreset(pendingPreset_);
        pendingPreset_.clear();
    }
    if (pendingMapLayout_ && scenario_) {
        applyMapLayout();
        pendingMapLayout_ = false;
    }
    if (!opt_.workspace.empty() && scenario_ && layoutReady_ && pendingPreset_.empty()) {
        if (opt_.workspace == "map")
            setWorkspace(Workspace::Map);
        opt_.workspace.clear();
    }

    // The dock space over the whole window, then the pool view and this workspace's windows
    ImGuiDockNodeFlags dockFlags = ImGuiDockNodeFlags_NoCloseButton; // each tab has its own close box
    if (layoutLocked_)
        dockFlags |= ImGuiDockNodeFlags_NoUndocking | ImGuiDockNodeFlags_NoResize | ImGuiDockNodeFlags_NoDocking;
    ImGui::DockSpaceOverViewport(dockspace_, viewport, dockFlags);
    updateSides();
    if (scenario_ && runTracking_)
        runScore_.reset(runTracking_->state().score);
    drawPoolView(time, dt);
    if (scenario_ && workspace_ == Workspace::Operate) {
        drawCameraWindows();
        drawMapWindow();
        drawSceneSettingsWindow();
        drawDisplayWindow();
        drawTfWindow();
        if (composition_) {
            composition_->drawPanels();
            composition_->drawWindows();
        }
        plots::drawWindows();
    } else if (scenario_)
        drawMapWindows();

    // Prompts, popups and edges on top of everything
    drawUnsavedMapPrompt();
    if (!scenario_)
        drawLoadingScreen();
    drawHelpWindow();
    drawFilePopups();
    drawCommandPalette();
    drawFloatingEdges();
    pins::drawMenu();
    handleWindowEdges();
    windowStates_.update();
}

// ----------------------------------------------------------------------------------------- title bar

// The title bar: the logo, File / View / Windows / Layout / Help, the command center and, with the viewer's own
// title bar, the window buttons.
void App::drawMenuBar() {
    // 35 px at the desktop's scale with 13 px menus, as VS Code's and other applications' title bars: the padding
    // sets the bar's height and centers the menus. The dropdowns are content (the panels' font and spacing at the
    // interface scale), never taller than the window.
    const float t = window_->contentScale();
    const ImGuiStyle content = ImGui::GetStyle();
    const auto menu = [&](const char *label, const std::function<void()> &body) {
        ImGui::SetNextWindowSizeConstraints({0, 0}, {FLT_MAX, ImGui::GetMainViewport()->WorkSize.y - ui(12)});
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, content.WindowPadding); // read as the dropdown begins
        pushPopupColors(Surface::Sheet);                                         // the dropdown is a sheet
        const bool shown = ImGui::BeginMenu(label);
        popPopupColors();
        ImGui::PopStyleVar();
        if (!shown)
            return;
        beginSurface(Surface::Sheet); // the dropdown is a sheet, not the board
        ImGui::PushFont(window_->normal);
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, content.FramePadding);
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, content.ItemSpacing);
        ImGui::PushStyleVar(ImGuiStyleVar_ItemInnerSpacing, content.ItemInnerSpacing);
        body();
        ImGui::PopStyleVar(3);
        ImGui::PopFont();
        endSurface();
        ImGui::EndMenu();
    };

    // The bar itself, sized at the desktop's scale (t) rather than the interface scale
    beginSurface(Surface::Board);
    ImGui::PushFont(window_->menu);
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(10 * t, 9 * t));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(14 * t, 12 * t));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemInnerSpacing, ImVec2(4 * t, 4 * t));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(10 * t, (35 * t - ImGui::GetFontSize()) * .5f));
    // the title strip a step darker than the command bar below it, with a hairline between (as VS Code's)
    const ImVec4 bar = palette().bar;
    const float shade = .2126f * bar.x + .7152f * bar.y + .0722f * bar.z < .5f ? .32f : .06f;
    ImGui::PushStyleColor(ImGuiCol_MenuBarBg, ImVec4(bar.x * (1 - shade), bar.y * (1 - shade), bar.z * (1 - shade), 1));
    const bool open = ImGui::BeginMainMenuBar();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar();
    if (open) {
        const ImVec2 at = ImGui::GetWindowPos();
        const float bottom = at.y + ImGui::GetWindowHeight() - 1;
        auto *draw = ImGui::GetWindowDrawList(); // the bar's own list: open menus draw over it
        draw->PushClipRect(at, {at.x + ImGui::GetWindowWidth(), bottom + 1}, false);
        draw->AddLine({at.x, bottom + .5f}, {at.x + ImGui::GetWindowWidth(), bottom + .5f},
                      ImGui::GetColorU32(ImGuiCol_Border), std::max(1.f, t));
        draw->PopClipRect();
    }
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(10 * t, 7 * t)); // inside the dropdowns
    if (!open) {
        ImGui::PopStyleVar(4);
        ImGui::PopFont();
        endSurface();
        return;
    }

    if (logoTrident_) { // the mark at the bar's left end, before the menus: drawn on the bar, its width reserved;
        // the trident in the bar's text color, the sonar pings in its accent
        const float size = 23 * t, height = ImGui::GetWindowHeight();
        const ImVec2 at(ImGui::GetCursorScreenPos().x + 2 * t, ImGui::GetWindowPos().y + (height - size) * .5f);
        auto *draw = ImGui::GetWindowDrawList();
        if (logoSonar_)
            draw->AddImage(textureID(logoSonar_), at, {at.x + size, at.y + size}, {0, 0}, {1, 1},
                           ImGui::GetColorU32(palette().accent));
        draw->AddImage(textureID(logoTrident_), at, {at.x + size, at.y + size}, {0, 0}, {1, 1},
                       ImGui::GetColorU32(palette().text));
        ImGui::Dummy({size + 4 * t, 1});
        if (ImGui::IsMouseHoveringRect(at, {at.x + size, at.y + size}))
            ImGui::SetTooltip("Nereus");
    }

    menu("File", [&] { drawFileMenu(); });
    menu("View", [&] { drawViewMenu(); });
    menu("Windows", [&] { drawWindowsMenu(); });
    menu("Layout", [&] { drawLayoutMenu(); });
    menu("Help", [&] { ImGui::MenuItem("Controls & shortcuts", "F1", &helpOpen_); });
    const char *state = maximized_ ? "Pool view maximized (Ctrl+Space restores)" : layoutLocked_ ? "Layout locked" : "";
    if (*state) {
        ImGui::SameLine(0, 24);
        ImGui::TextDisabled("%s", state);
    }
    drawCommandCenter(t);
    if (window_->customTitleBar())
        drawWindowControls();
    ImGui::EndMainMenuBar();
    ImGui::PopStyleVar(4);
    ImGui::PopFont();
    endSurface();
}

// The command center: a search box in the title bar, centered between the menus and the window buttons (as VS
// Code's). Typing in it (click, or Ctrl+P) drops the matches down from it.
void App::drawCommandCenter(float t) {
    if (!scenario_)
        return;
    const float barWidth = ImGui::GetWindowWidth(), buttons = window_->customTitleBar() ? 3 * 46 * t : 0;
    const float left = ImGui::GetCursorPosX() + 24 * t, right = barWidth - buttons - 24 * t;
    const float width = std::min(420 * t, right - left);
    if (width < 180 * t) { // no room in the bar: the results still open (Ctrl+P) under where it would be
        paletteBox_ = {ImGui::GetWindowPos().x + (barWidth - 420 * t) * .5f, ImGui::GetWindowPos().y};
        paletteBoxSize_ = {420 * t, ImGui::GetWindowHeight()};
        return;
    }

    // The box: centered in the bar, at VS Code's height
    const float height = std::min(ImGui::GetWindowHeight() - 8 * t, 22 * t); // VS Code's command center
    const float x = std::clamp((barWidth - width) * .5f, left, right - width);
    const ImVec2 at(ImGui::GetWindowPos().x + x, ImGui::GetWindowPos().y + (ImGui::GetWindowHeight() - height) * .5f);
    paletteBox_ = at;
    paletteBoxSize_ = {width, height};
    const float rounding = std::min(height * .5f, ImGui::GetStyle().FrameRounding * t + 2 * t);
    ImGui::SetCursorScreenPos(at);
    ImGui::PushFont(window_->titleSmall);
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, {26 * t, (height - ImGui::GetFontSize()) * .5f});
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, rounding);
    ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, std::max(1.f, t * .75f));
    if (paletteFocus_) { // Ctrl+P: the menu bar takes no keyboard focus itself, so activate the box by its id
        ImGui::FocusWindow(ImGui::GetCurrentWindow());
        ImGui::ActivateItemByID(ImGui::GetID("##command_center"));
        paletteFocus_ = false;
    }

    ImGui::SetNextItemWidth(width);
    const auto takeFill = [](ImGuiInputTextCallbackData *data) {
        auto &fill = *static_cast<std::string *>(data->UserData);
        if (!fill.empty()) { // a command that needs more typing: its start, the cursor after it
            data->DeleteChars(0, data->BufTextLen);
            data->InsertChars(0, fill.c_str());
            data->SelectionStart = data->SelectionEnd = data->CursorPos = data->BufTextLen;
            fill.clear();
        }
        return 0;
    };
    if (ImGui::InputTextWithHint("##command_center", "Search Nereus", paletteQuery_, sizeof(paletteQuery_),
                                 ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_AutoSelectAll |
                                     ImGuiInputTextFlags_CallbackAlways,
                                 takeFill, &paletteFill_))
        paletteEnter_ = true;
    if (ImGui::IsItemActivated()) {
        paletteOpen_ = true;
        paletteIndex_ = 0;
        pins::requestCensus(); // every window's controls into the index, closed ones too
    }
    if (ImGui::IsItemEdited())
        paletteIndex_ = 0;
    paletteTyping_ = ImGui::IsItemActive();
    const bool hovered = ImGui::IsItemHovered();
    ImGui::PopStyleVar(3);
    if (hovered && !paletteTyping_)
        ImGui::SetTooltip("Search windows, layouts, themes, controls and trees, or type a move: forward 0.5, "
                          "turn 30, go 2 1 -1.5 90 (Ctrl+P)");

    // a magnifier at its left; the shortcut at its right while it is empty and idle
    auto *draw = ImGui::GetWindowDrawList();
    const ImU32 ink = ImGui::GetColorU32(palette().muted);
    const ImVec2 lens(at.x + 13 * t, at.y + height * .5f - .7f * t);
    draw->AddCircle(lens, 3.8f * t, ink, 0, 1.2f * t);
    draw->AddLine({lens.x + 2.7f * t, lens.y + 2.7f * t}, {lens.x + 5.5f * t, lens.y + 5.5f * t}, ink, 1.2f * t);
    if (!paletteTyping_ && !paletteQuery_[0]) {
        const char *keys = "Ctrl+P";
        const float keysWidth = ImGui::CalcTextSize(keys).x;
        draw->AddText({at.x + width - keysWidth - 10 * t, at.y + (height - ImGui::GetFontSize()) * .5f}, ink, keys);
    }
    ImGui::PopFont();
}

// ------------------------------------------------------------------------------------ command center

// Everything the palette can run: the menus' items, as commands. Enable and KILL are not here on purpose: a
// fuzzy match must never start or stop the robot.
std::vector<App::PaletteCommand> App::paletteCommands() {
    std::vector<PaletteCommand> out;
    const auto add = [&](const std::string &group, const std::string &label, bool checked, std::function<void()> run) {
        out.push_back({group, label, checked, std::move(run)});
    };

    // Windows, then the workspace's own commands: layouts, focus targets, cameras and view toggles in Operate
    for (const auto &entry : windowEntries())
        add("Window", entry.label, *entry.open, [this, entry] { showWindow(entry); });
    if (workspace_ == Workspace::Operate) {
        for (const auto &preset : layoutPresets())
            add("Layout", preset.label, false, [this, id = preset.id] { pendingPreset_ = id; });
        for (const auto &name : savedLayouts(layoutDir_))
            add("Layout", name, false, [this, name] { requestLayout(name); });
        for (const auto &name : focusNames_)
            add("Focus", name, focusName_ == name, [this, name] { focus(name); });
        add("Camera", "Orbit", mode_ == 0, [this] { setViewMode(0); });
        add("Camera", "Free camera", mode_ == 1, [this] { setViewMode(1); });
        for (std::size_t i = 0; i < scenario_->cameras.size(); ++i)
            add("Camera", "Look through the " + scenario_->cameras[i].title, mode_ == int(i) + 2,
                [this, i] { setViewMode(int(i) + 2); });
        add("View", "Follow", follow_, [this] { follow_ = !follow_; });
        add("View", "Labels", labels_, [this] { labels_ = !labels_; });
        add("View", "TF frames", showTf_, [this] { showTf_ = !showTf_; });
        if (!demoMode_) {
            add("View", "Detections", detections_, [this] { detections_ = !detections_; });
            add("View", "MPC path", showMpc_, [this] { showMpc_ = !showMpc_; });
            if (!scenario_->thrusterMounts.empty())
                add("View", "Thrust", showThrust_, [this] { showThrust_ = !showThrust_; });
        }
        add("Map", "Edit prior map", false, [this] { setWorkspace(Workspace::Map); });
    } else {
        add("Map", "2D view", planView_, [this] {
            planView_ = !planView_;
            if (planView_ && !planFitted_)
                fitPlan();
        });
        add("Map", "Done editing the map", false, [this] { setWorkspace(Workspace::Operate); });
    }

    // In both workspaces: view, theme, interface scale, pool and file commands
    add("View", "Maximize pool view", maximized_, [this] { toggleMaximized(); });
    add("View", "Left panels", sideShown(Side::Left), [this] { toggleSide(Side::Left); });
    add("View", "Right panels", sideShown(Side::Right), [this] { toggleSide(Side::Right); });
    add("View", "Frame stats", showProfile_, [this] { showProfile_ = !showProfile_; });
    for (const auto &theme : themes())
        add("Theme", theme.label, currentTheme() == theme.id, [this, id = theme.id] { setTheme(id); });
    add("Interface scale", "Match desktop", uiScaleSetting_ <= 0, [this] { setUiScale(0); });
    for (const float scale : {1.f, 1.25f, 1.5f, 1.75f, 2.f})
        add("Interface scale", std::to_string(int(std::lround(scale * 100))) + " %",
            uiScaleSetting_ > 0 && std::abs(uiScaleSetting_ - scale) < .01f, [this, scale] { setUiScale(scale); });
    if (poolSwitch_.packs.empty())
        poolSwitch_.packs = scenarioPacks();
    for (const auto &pack : poolSwitch_.packs)
        if (pack.poolId != scenario_->poolId)
            add("Pool", pack.poolLabel, false, [this, pack] { switchPool(pack); });
    add("File", "Open prior map...", false, [this] {
        std::snprintf(openMapPath_, sizeof(openMapPath_), "%s", priorMap_->file().c_str());
        openMapPopup_ = true;
    });
    for (const auto &path : recentMaps_)
        add("File", "Open recent: " + path.filename().string() + "  (" + path.parent_path().string() + ")",
            path == priorMap_->file(), [this, path] { openPriorMap(path); });
    if (priorMap_->dirty())
        add("File", "Save prior map", false, [this] { priorMap_->save(); });
    if (priorMap_->loaded())
        add("File", "Reload prior map", false, [this] {
            if (priorMap_->dirty())
                reloadMapPopup_ = true;
            else
                priorMap_->reload();
        });
    add("File", "Save screenshot", false, [this] { requestScreenshot(); });
    if (fs::exists(kRunRecords))
        add("File", "Open run records", false, [] { openFolder(kRunRecords); });
    add("File", "Open settings folder", false, [this] { openFolder(configDirectory()); });
    add("File", "Quit", false, [this] { window_->requestClose(); });
    add("Help", "Controls & shortcuts", helpOpen_, [this] { helpOpen_ = true; });

    // The robot: moves to type out (each fills the box with its first word), and keyboard driving
    if (workspace_ == Workspace::Operate && motion()) {
        const auto type = [&](const std::string &label, const std::string &fill) {
            out.push_back({"Robot", label, false, nullptr, fill});
        };
        type("Move forward / back / left / right / up / down...", "forward ");
        type("Turn left / right...", "turn ");
        type("Go to x y z yaw...", "go ");
        type("Set x / y / z / roll / pitch / yaw...", "z ");
        add("Robot", "Drive with the keyboard", driving_, [this] { setDriving(!driving_); });
    }

    // Autonomy: each tree to run (as the panel's Start: only while the robot it drives is enabled), Stop
    if (const auto mission = autonomy()) {
        const auto s = mission->state();
        if (s.connected && !s.busy && !s.pending) {
            const bool may = composition_->mayStart(mission);
            for (const auto &tree : s.trees)
                if (may)
                    add("Autonomy", "Run " + fs::path(tree).filename().string(), false,
                        [mission, tree] { mission->start(tree); });
            if (!may && !s.trees.empty())
                out.push_back({"Autonomy", "Run a tree: enable the robot first", false, nullptr, {}});
        }
        if (s.busy)
            add("Autonomy",
                "Stop " + (s.activeTree.empty() ? std::string("the tree") : fs::path(s.activeTree).filename().string()),
                false, [mission] { mission->stop(); });
        if (s.connected && !s.refreshing)
            add("Autonomy", "Refresh the tree list", false, [mission] { mission->refresh(); });
    }

    // Every control in every window (as pinning knows them): run as a click on it would, in its window or, when
    // that is closed or behind a tab, off screen. Unavailable ones are left out, and so are Enable / KILL.
    for (const auto &control : pins::controls()) {
        if (control.disabled || control.key.find("enable_kill") != std::string::npos || control.window.empty())
            continue;
        if (control.kind == pins::Control::Kind::Choice) {
            for (std::size_t i = 0; i < control.items.size(); ++i)
                add(control.window, control.label + ": " + control.items[i], int(i) == control.current,
                    [key = control.key, i] { pins::trigger(key, int(i)); });
        } else
            add(control.window, control.label, control.kind == pins::Control::Kind::Checkbox && control.checked,
                [key = control.key] { pins::trigger(key); });
    }
    // Plots: New plot, the saved plots and "Plot a topic field…"; once the search starts with "plot ", every
    // topic field, panel figure and Motion axis too (Shift adds to the focused plot)
    if (workspace_ == Workspace::Operate) {
        std::string query = paletteQuery_;
        query.erase(0, query.find_first_not_of(' '));
        std::transform(query.begin(), query.end(), query.begin(),
                       [](unsigned char c) { return char(std::tolower(c)); });
        for (auto &command : plots::commands(query.rfind("plot ", 0) == 0)) {
            if (command.run)
                add(command.group, command.label, false, std::move(command.run));
            else
                out.push_back({command.group, command.label, false, nullptr, command.fill});
        }
    }
    return out;
}

// The matches, dropped down from the title bar's search box while it is in use: the arrows choose, Enter or a
// click runs one, Esc or a click elsewhere closes them.
void App::drawCommandPalette() {
    if (!paletteOpen_ || !scenario_)
        return;
    const auto close = [&] {
        paletteOpen_ = paletteEnter_ = false;
        paletteQuery_[0] = 0;
        paletteIndex_ = 0;
    };
    if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
        close();
        return;
    }

    // The results window, under the box and at least 520 px wide
    const auto *viewport = ImGui::GetMainViewport();
    const float width = std::min(std::max(paletteBoxSize_.x, ui(520)), viewport->Size.x - ui(24));
    const float x = std::clamp(paletteBox_.x + (paletteBoxSize_.x - width) * .5f, viewport->Pos.x + ui(12),
                               viewport->Pos.x + viewport->Size.x - width - ui(12));
    ImGui::SetNextWindowPos({x, paletteBox_.y + paletteBoxSize_.y + ui(3)});
    ImGui::SetNextWindowSizeConstraints({width, 0}, {width, viewport->Size.y * .7f});
    ImGui::Begin("##command_results", nullptr,
                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav);
    ImGui::BringWindowToDisplayFront(ImGui::GetCurrentWindow()); // over every window, as a dropdown

    // a move typed out comes first: where it takes the robot, or how to finish it / why it cannot run now
    std::vector<PaletteCommand> commands;
    int typedScore = -1; // first; how to finish a move comes after any matches ("left panels" is a search)
    if (workspace_ == Workspace::Operate && motion()) {
        const auto parsed = parseMove(paletteQuery_, poseBase());
        std::string why;
        bool wait = false;
        if (parsed.isMove && !parsed.command) {
            commands.push_back({"Robot", parsed.error, false, nullptr, {}});
            typedScore = 1 << 20;
        } else if (parsed.isMove && !commandable(&why, &wait))
            commands.push_back({"Robot", wait ? "Waiting for the controller..." : why, false, nullptr, {}});
        else if (parsed.isMove)
            commands.push_back({"Robot",
                                parsed.command->summary + "   \u2192   " + describe(parsed.command->target),
                                false,
                                [this, target = parsed.command->target] { commandPose(target); },
                                {}});
    }

    const std::size_t typed = commands.size();
    for (auto &command : paletteCommands())
        commands.push_back(std::move(command));

    // the matches, best first (ties keep the menus' order); with nothing typed, everything in that order
    std::vector<std::pair<int, std::size_t>> ranked;
    for (std::size_t i = 0; i < commands.size(); ++i) {
        const int label = fuzzyScore(commands[i].label, paletteQuery_),
                  full = fuzzyScore(commands[i].group + " " + commands[i].label, paletteQuery_);
        const int score = i < typed ? typedScore : label >= 0 ? label : full >= 0 ? full + 50 : -2;
        if (score >= -1)
            ranked.push_back({score, i});
    }
    std::stable_sort(ranked.begin(), ranked.end(), [](const auto &a, const auto &b) { return a.first < b.first; });
    const int shownCount = int(std::min<std::size_t>(ranked.size(), 14));

    // The arrows move the highlight; Enter runs the highlighted row
    if (paletteTyping_ && ImGui::IsKeyPressed(ImGuiKey_DownArrow))
        paletteIndex_ = std::min(paletteIndex_ + 1, std::max(0, shownCount - 1));
    if (paletteTyping_ && ImGui::IsKeyPressed(ImGuiKey_UpArrow))
        paletteIndex_ = std::max(paletteIndex_ - 1, 0);
    paletteIndex_ = std::clamp(paletteIndex_, 0, std::max(0, shownCount - 1));
    const PaletteCommand *chosen = nullptr;
    if (paletteEnter_ && shownCount > 0)
        chosen = &commands[ranked[std::size_t(paletteIndex_)].second];
    paletteEnter_ = false;

    // The rows: the group in a left column, the label, and "on" at the right for a current choice
    if (shownCount == 0)
        ImGui::TextDisabled("No match");
    float groupWidth = ImGui::CalcTextSize("Interface scale").x;
    for (int row = 0; row < shownCount; ++row)
        groupWidth =
            std::max(groupWidth, ImGui::CalcTextSize(commands[ranked[std::size_t(row)].second].group.c_str()).x);
    groupWidth = std::min(groupWidth + ui(16), width * .4f);
    for (int row = 0; row < shownCount; ++row) {
        const auto &command = commands[ranked[std::size_t(row)].second];
        ImGui::PushID(row);
        const ImVec2 at = ImGui::GetCursorScreenPos();
        const bool selected = row == paletteIndex_;
        if (ImGui::Selectable("##row", selected, 0, {0, ImGui::GetTextLineHeight() + ui(6)}))
            chosen = &command;
        const bool hovered = ImGui::IsItemHovered(), held = ImGui::IsItemActive();
        if (hovered && (ImGui::GetIO().MouseDelta.x != 0 || ImGui::GetIO().MouseDelta.y != 0))
            paletteIndex_ = row;
        // Inks that read on the row's highlight, as the Selectable filled it (Classic's is navy: white ink there).
        const auto &colors = ImGui::GetStyle().Colors;
        const bool lit = hovered || selected;
        const ImVec4 fill = colors[held && hovered ? ImGuiCol_HeaderActive
                                   : hovered       ? ImGuiCol_HeaderHovered
                                                   : ImGuiCol_Header];
        const auto ink = [&](ImVec4 c, float ratio) {
            if (!lit)
                return c;
            const ImVec4 on = inkOn(fill, c, ratio);
            // a secondary ink that had to change keeps its rank: the new ink, softened
            return on.x == c.x && on.y == c.y && on.z == c.z ? c : ImVec4(on.x, on.y, on.z, ratio < 4 ? .75f : 1);
        };
        auto *draw = ImGui::GetWindowDrawList();
        const float y = at.y + ui(3);
        draw->AddText({at.x + ui(4), y}, ImGui::GetColorU32(ink(palette().muted, 3)), command.group.c_str());
        // a note (nothing to run) in muted ink
        const bool runnable = command.run || !command.fill.empty();
        draw->AddText({at.x + groupWidth, y},
                      ImGui::GetColorU32(runnable ? ink(palette().text, 4.5f) : ink(palette().muted, 3)),
                      command.label.c_str());
        if (command.checked) { // the current choice / an option that is on
            const char *on = "on";
            draw->AddText({at.x + ImGui::GetContentRegionAvail().x - ImGui::CalcTextSize(on).x - ui(4), y},
                          ImGui::GetColorU32(ink(palette().accent, 3)), on);
        }
        ImGui::PopID();
    }
    if (ranked.size() > std::size_t(shownCount))
        ImGui::TextDisabled("%zu more: keep typing", ranked.size() - std::size_t(shownCount));
    const bool overResults = ImGui::IsWindowHovered(ImGuiHoveredFlags_RootAndChildWindows);
    ImGui::End();

    // a click on a row keeps them open until it lands; anywhere else (the box no longer in use) closes them
    const bool overBox =
        ImGui::IsMouseHoveringRect(paletteBox_, {paletteBox_.x + paletteBoxSize_.x, paletteBox_.y + paletteBoxSize_.y});
    // a command that needs more typing fills the box (a note leaves it as it is) and the box keeps the keyboard
    std::function<void()> run = chosen ? chosen->run : nullptr;
    if (chosen && !run) {
        paletteFill_ = chosen->fill.empty() ? std::string(paletteQuery_) : chosen->fill;
        std::snprintf(paletteQuery_, sizeof(paletteQuery_), "%s", paletteFill_.c_str());
        paletteIndex_ = 0;
        paletteFocus_ = true;
    }
    if (run ||
        (!paletteTyping_ && !paletteFocus_ && !overResults && !overBox && !ImGui::IsMouseDown(ImGuiMouseButton_Left)))
        close();
    if (run)
        run();
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

    // Minimize, maximize / restore and close at the bar's right end, 46 px wide at the desktop's scale
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

// Floating windows (undocked tool windows, the scorecard, Help) stand off the docked sheets: a hairline edge and
// a soft shadow below, drawn only outside each window so nothing covers its content.
void App::drawFloatingEdges() {
    auto &g = *ImGui::GetCurrentContext();
    const ImU32 edge = ImGui::GetColorU32(ImGuiCol_Border);
    const bool light = palette().text.x < .5f;
    for (ImGuiWindow *window : g.Windows) {
        if (!window->Active || window->Hidden || window->DockIsActive || window->DockNodeAsHost ||
            (window->Flags & (ImGuiWindowFlags_ChildWindow | ImGuiWindowFlags_Popup | ImGuiWindowFlags_Tooltip |
                              ImGuiWindowFlags_NoTitleBar)))
            continue;
        auto *draw = window->DrawList;
        const ImRect box = window->Rect();
        const ImVec2 view = ImGui::GetMainViewport()->Size;
        // the four bands around the window, each clipped to its side so the shadow never lands on the window
        const ImRect sides[] = {{{0, 0}, {view.x, box.Min.y}},
                                {{0, box.Max.y}, view},
                                {{0, box.Min.y}, {box.Min.x, box.Max.y}},
                                {{box.Max.x, box.Min.y}, {view.x, box.Max.y}}};
        const float rounding = window->WindowRounding, drop = ui(4);
        for (const auto &side : sides) {
            draw->PushClipRect(side.Min, side.Max, false);
            for (int i = 1; i <= 10; ++i) {
                const float spread = ui(float(i) * 1.4f), fade = 1 - float(i) / 11;
                draw->AddRect({box.Min.x - spread, box.Min.y - spread + drop},
                              {box.Max.x + spread, box.Max.y + spread + drop},
                              IM_COL32(0, 0, 0, int((light ? 22 : 48) * fade * fade)), rounding + spread, 0, ui(1.6f));
            }
            draw->PopClipRect();
        }
        draw->PushClipRect({0, 0}, view, false);
        draw->AddRect(box.Min, box.Max, edge, rounding, 0, 1);
        draw->PopClipRect();
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

    // The edge or corner under the pointer, numbered as Window::beginResize takes them (0 top-left, clockwise)
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

// ----------------------------------------------------------------------------------- theme and scale

// A theme for the whole interface; the operator's choice is remembered (viewer.yaml in the config directory).
// The theme changes between frames (applyPendingTheme): mid-frame, colors pushed by the board and menus would
// be popped back as the old theme's when they close, leaving a mix of two themes.
void App::setTheme(const std::string &id) {
    pendingTheme_ = id;
}

void App::applyPendingTheme() {
    if (pendingTheme_.empty())
        return;
    const std::string id = std::move(pendingTheme_);
    pendingTheme_.clear();
    const auto before = currentThemeInfo();
    if (!applyTheme(id))
        return;
    savePreference("theme", YAML::Node(id));
    const auto &after = currentThemeInfo();
    if (after.fontFamily != before.fontFamily || after.fontPoints != before.fontPoints ||
        after.fontRegular != before.fontRegular)
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

// The interface scale in use: the setting, or the desktop's content scale when it follows the desktop (0).
float App::resolvedUiScale() const {
    return uiScaleSetting_ > 0 ? uiScaleSetting_ : window_->contentScale();
}

// Sets and remembers the interface scale (0: match the desktop); the fonts reload between frames.
void App::setUiScale(float setting) {
    uiScaleSetting_ = setting;
    pendingUiScale_ = resolvedUiScale();
    savePreference("interface_scale", setting > 0 ? YAML::Node(setting) : YAML::Node("auto"));
}

void App::applyPendingUiScale() {
    if (pendingUiScale_ <= 0)
        return;
    const auto &theme = currentThemeInfo(); // a theme with its own font (the desktop's Qt font)
    window_->loadFonts(pendingUiScale_, window_->contentScale(), theme, contentDirectory() / "fonts");
    setInterfaceScale(pendingUiScale_);
    pendingUiScale_ = -1;
}

// View > Interface scale: fixed steps, or match the desktop.
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

// ------------------------------------------------------------------------------------ loading screen

// The loading screen, over everything under the title bar: the Nereus mark with its sonar pinging outward, water
// rising from the bottom (further once the scene is being built), the name and the status. Drawn, not rendered:
// it costs nothing while the simulator starts.
void App::drawLoadingScreen() {
    const auto *viewport = ImGui::GetMainViewport();
    const float top = 35 * window_->contentScale(); // the title bar stays usable
    const ImVec2 origin(viewport->Pos.x, viewport->Pos.y + top), size(viewport->Size.x, viewport->Size.y - top);
    ImGui::SetNextWindowPos(origin);
    ImGui::SetNextWindowSize(size);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {0, 0});
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0);
    ImGui::Begin("##loading", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings |
                     ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav |
                     ImGuiWindowFlags_NoBringToFrontOnFocus);
    ImGui::PopStyleVar(2);
    ImGui::BringWindowToDisplayFront(ImGui::GetCurrentWindow()); // over the dock space (focused at start)
    beginSurface(Surface::Board);                                // the board's colors: ink by day, black by night

    const auto &p = palette();
    auto *draw = ImGui::GetWindowDrawList();
    const float t = float(ImGui::GetTime());
    const ImVec2 end(origin.x + size.x, origin.y + size.y);
    draw->AddRectFilled(origin, end, ImGui::GetColorU32(p.bar));

    // the water: rising toward a third of the height while waiting, to over half once the scene is building
    auto &level = loadingLevel_;
    const float target = buildingShown_ ? .55f : .34f;
    level += (target - level) * (1 - std::exp(-ImGui::GetIO().DeltaTime * (buildingShown_ ? 3.f : .35f)));
    const ImVec4 bar = p.bar, pool(.10f, .62f, .78f, 1);
    const auto water = [&](float amount, float alpha) {
        return ImGui::GetColorU32(ImVec4(bar.x + (pool.x - bar.x) * amount, bar.y + (pool.y - bar.y) * amount,
                                         bar.z + (pool.z - bar.z) * amount, alpha));
    };
    const float surface = end.y - size.y * level, step = ui(4);
    for (int layer = 0; layer < 2; ++layer) { // a slower back wave, then the front one with its bright edge
        const float amplitude = ui(layer ? 7 : 10), speed = layer ? 1.3f : -.8f, wave = layer ? .011f : .007f;
        std::vector<ImVec2> crest;
        for (float x = origin.x; x <= end.x + step; x += step) {
            const float y = surface + (layer ? 0 : -ui(6)) +
                            amplitude * std::sin((x - origin.x) * wave / ui(1) + t * speed) +
                            amplitude * .45f * std::sin((x - origin.x) * wave * 2.3f / ui(1) - t * speed * 1.7f);
            draw->AddRectFilled({x, y}, {std::min(x + step, end.x), end.y}, water(layer ? .42f : .26f, 1));
            crest.push_back({x, y});
        }
        if (layer)
            draw->AddPolyline(crest.data(), int(crest.size()), water(.75f, .8f), 0, ui(2));
    }

    // the mark, above the water: its sonar pings rising from the trident's foot, then the trident
    const float mark = ui(128);
    const ImVec2 corner(origin.x + (size.x - mark) * .5f,
                        std::min(surface - mark - ui(90), origin.y + size.y * .5f - mark));
    const ImVec2 foot(corner.x + mark * .5f, corner.y + mark * .815f);
    const ImVec4 accent = p.accent;
    for (int ping = 0; ping < 3; ++ping) {
        const float phase = std::fmod(t * .45f + float(ping) / 3, 1.f);
        const float radius = mark * (.14f + .42f * phase), fade = std::pow(1 - phase, 1.5f);
        draw->PathArcTo(foot, radius, -2.65f, -.49f, 32);
        draw->PathStroke(ImGui::GetColorU32(ImVec4(accent.x, accent.y, accent.z, .9f * fade)), 0, ui(4));
    }
    if (logoTrident_)
        draw->AddImage(textureID(logoTrident_), corner, {corner.x + mark, corner.y + mark}, {0, 0}, {1, 1},
                       ImGui::GetColorU32(p.text));

    // the name and what the viewer is waiting for
    ImFont *title = window_->title, *small = window_->small;
    const char *name = "Nereus";
    const ImVec2 nameSize = title->CalcTextSizeA(title->FontSize, 1e9f, 0, name);
    float y = corner.y + mark + ui(18);
    draw->AddText(title, title->FontSize, {origin.x + (size.x - nameSize.x) * .5f, y}, ImGui::GetColorU32(p.text),
                  name);
    y += nameSize.y + ui(10);
    if (loadingTopic_.empty()) // first drawn on the main thread (config_ is not the loading thread's to read)
        loadingTopic_ = !opt_.scenarioTopic.empty()
                            ? opt_.scenarioTopic
                            : lookup(config_, {"scenario_topic"}).as<std::string>("/talos/simulator/scenario");
    const std::string &topic = loadingTopic_;
    const std::string status = buildingShown_ ? "Building the scene" : "Waiting for the scenario from the simulator";
    const ImVec2 statusSize = ImGui::CalcTextSize(status.c_str());
    draw->AddText({origin.x + (size.x - statusSize.x) * .5f, y}, ImGui::GetColorU32(p.text), status.c_str());
    y += statusSize.y + ui(6);
    std::string detail = buildingShown_ ? "meshes, textures and the course map" : topic;
    if (!buildingShown_ && ImGui::GetTime() > 20)
        detail = topic + "  \u00b7  is the simulator running? (ros2 launch integrations/uwrt/launch/sim.launch.py)";
    const ImVec2 detailSize = small->CalcTextSizeA(small->FontSize, 1e9f, 0, detail.c_str());
    draw->AddText(small, small->FontSize, {origin.x + (size.x - detailSize.x) * .5f, y}, ImGui::GetColorU32(p.muted),
                  detail.c_str());
    endSurface();
    ImGui::End();
}

// Whether the scene is built while another thread animates the loading screen (see animateLoadingWhile).
bool App::loadingOnThread() const {
    return !scenario_ && buildingShown_ && opt_.frames == 0 && !opt_.hidden;
}

void App::animateLoadingWhile(const std::function<void()> &work) {
    if (!loadingOnThread()) {
        work();
        return;
    }

    // Hand the GL context to a thread that draws the loading screen at about 60 Hz until `work` is done
    std::atomic<bool> done{false};
    window_->detach();
    std::thread animation([&] {
        window_->currentOnThisThread(true);
        for (auto previous = Clock::now(); !done.load();) {
            const auto now = Clock::now();
            window_->beginDetachedFrame(float(std::chrono::duration<double>(now - previous).count()));
            previous = now;
            drawLoadingScreen();
            window_->presentDetached();
            std::this_thread::sleep_until(now + std::chrono::microseconds(16667)); // 60 Hz (vsync may be off)
        }
        window_->currentOnThisThread(false);
    });

    const auto finish = [&] {
        done = true;
        animation.join();
        window_->reattach();
    };
    try {
        work();
    } catch (...) {
        finish();
        throw;
    }
    finish();
}

// ----------------------------------------------------------------------------------------- file menu

void App::notify(const std::string &message, bool error) {
    poolSwitch_.message = message;
    poolSwitch_.messageError = error;
    poolSwitch_.messageUntil = Clock::now() + std::chrono::seconds(5);
}

// Queues a PNG of the whole window, saved two frames later (once the menu that asked has closed).
void App::requestScreenshot() {
    std::error_code error;
    fs::create_directories(screenshotDirectory(), error);
    const auto now = std::time(nullptr);
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", std::localtime(&now));
    screenshotPath_ = screenshotDirectory() / ("nereus-" + std::string(stamp) + ".png");
    screenshotCountdown_ = 2;
}

// Opens a prior map config in the editor, reports how that went and puts it first in Open recent (six kept).
void App::openPriorMap(const fs::path &file) {
    priorMap_->open(file);
    const bool ok = priorMap_->loaded() && priorMap_->file() == file;
    notify(priorMap_->message().empty() ? file.filename().string() : priorMap_->message(), !ok);
    if (!ok)
        return;
    recentMaps_.erase(std::remove(recentMaps_.begin(), recentMaps_.end(), file), recentMaps_.end());
    recentMaps_.insert(recentMaps_.begin(), file);
    if (recentMaps_.size() > 6)
        recentMaps_.resize(6);
    YAML::Node list(YAML::NodeType::Sequence);
    for (const auto &path : recentMaps_)
        list.push_back(path.string());
    savePreference("recent_prior_maps", list);
}

// File: what the viewer reads and writes. The pool (a scenario), the prior map's file, screenshots, the folders
// its records and settings live in, and quitting.
void App::drawFileMenu() {
    if (scenario_) {
        sectionTitle("Pool");
        if (ImGui::BeginMenu("Switch pool")) {
            drawPoolMenu();
            ImGui::EndMenu();
        }
    }

    sectionTitle("Prior map");
    if (ImGui::MenuItem("Open...")) {
        std::snprintf(openMapPath_, sizeof(openMapPath_), "%s", priorMap_->file().c_str());
        openMapPopup_ = true;
    }
    if (ImGui::BeginMenu("Open recent", !recentMaps_.empty())) {
        for (const auto &path : recentMaps_) {
            ImGui::PushID(path.c_str());
            if (ImGui::MenuItem(path.parent_path().parent_path().filename().empty()
                                    ? path.filename().c_str()
                                    : (path.parent_path().parent_path().filename() / path.filename()).c_str(),
                                nullptr, path == priorMap_->file()))
                openPriorMap(path);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("%s", path.c_str());
            ImGui::PopID();
        }
        ImGui::EndMenu();
    }
    if (ImGui::MenuItem("Save", "Ctrl+S", false, priorMap_->loaded() && priorMap_->dirty()))
        priorMap_->save();
    if (ImGui::MenuItem("Reload", nullptr, false, priorMap_->loaded())) {
        if (priorMap_->dirty())
            reloadMapPopup_ = true;
        else
            priorMap_->reload();
    }
    if (priorMap_->loaded() && ImGui::IsItemHovered())
        ImGui::SetTooltip("Read %s again", priorMap_->file().c_str());

    sectionTitle("Capture");
    if (ImGui::MenuItem("Save screenshot", "F12"))
        requestScreenshot();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("The whole window, as a PNG in %s", screenshotDirectory().c_str());
    if (ImGui::MenuItem("Open screenshots folder", nullptr, false, fs::exists(screenshotDirectory())))
        openFolder(screenshotDirectory());

    sectionTitle("Folders");
    if (ImGui::MenuItem("Open run records", nullptr, false, fs::exists(kRunRecords)))
        openFolder(kRunRecords);
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("%s: one folder per simulator run (log, scores, bags)", kRunRecords.c_str());
    if (ImGui::MenuItem("Open settings folder", nullptr, false, fs::exists(configDirectory())))
        openFolder(configDirectory());
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("%s: preferences, the saved arrangement and named layouts", configDirectory().c_str());

    ImGui::Separator();
    if (ImGui::MenuItem("Quit", "Ctrl+Q"))
        window_->requestClose(); // asks first when the prior map has unsaved edits
}

// The File menu's dialogs: a prior map's path to open, and confirming a reload over unsaved edits.
void App::drawFilePopups() {
    if (openMapPopup_) {
        ImGui::OpenPopup("Open prior map");
        openMapPopup_ = false;
    }
    if (reloadMapPopup_) {
        ImGui::OpenPopup("Reload prior map?");
        reloadMapPopup_ = false;
    }

    ImGui::SetNextWindowSize({ui(620), 0});
    if (ImGui::BeginPopupModal("Open prior map", nullptr, ImGuiWindowFlags_NoSavedSettings)) {
        ImGui::TextWrapped("A riptide_mapping config.yaml (the robot workspace's source tree).");
        ImGui::SetNextItemWidth(-1);
        if (ImGui::IsWindowAppearing())
            ImGui::SetKeyboardFocusHere();
        const bool entered = ImGui::InputText("##path", openMapPath_, sizeof(openMapPath_),
                                              ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_AutoSelectAll);
        const fs::path path = openMapPath_;
        const bool exists = fs::is_regular_file(path);
        if (!exists && openMapPath_[0])
            ImGui::TextColored(palette().error, "No such file");
        ImGui::BeginDisabled(!exists);
        if (ImGui::Button("Open") || (entered && exists)) {
            openPriorMap(path);
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("Cancel") || ImGui::IsKeyPressed(ImGuiKey_Escape))
            ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    if (ImGui::BeginPopupModal("Reload prior map?", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted("Discard the unsaved changes and read the file again?");
        if (ImGui::Button("Reload")) {
            priorMap_->reload();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel") || ImGui::IsKeyPressed(ImGuiKey_Escape))
            ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
}

// --------------------------------------------------------------------------------------------- menus

// View: the camera, focus and overlays (Operate) or the 2D view (Map), then the pool view's size, the side
// panels, the theme and the interface scale.
void App::drawViewMenu() {
    if (!scenario_) {
        ImGui::TextDisabled("Waiting for the scenario");
        ImGui::Separator();
        drawThemeMenu();
        drawScaleMenu();
        return;
    }

    if (workspace_ == Workspace::Map) { // the map tools are in the pool view's toolbar
        sectionTitle("Map");
        if (ImGui::MenuItem("2D view", nullptr, planView_)) {
            planView_ = !planView_;
            if (planView_ && !planFitted_)
                fitPlan();
        }
    } else {
        sectionTitle("Camera");
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
        sectionTitle("Overlays");
        ImGui::MenuItem("Labels", nullptr, &labels_);
        ImGui::MenuItem("TF frames", nullptr, &showTf_);
        ImGui::MenuItem("Detections", nullptr, &detections_, !demoMode_);
        ImGui::MenuItem("MPC path", nullptr, &showMpc_, !demoMode_);
        ImGui::MenuItem("Thrust", nullptr, &showThrust_, !demoMode_ && !scenario_->thrusterMounts.empty());
    }

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

// View > Theme, each theme's description as its tooltip.
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
    // Every window in windowEntries(), by section: a ticked one closes, an unticked one opens on top.
    const auto entries = windowEntries();
    const std::pair<MenuSection, const char *> sections[] = {
        {MenuSection::Panels, "Panels"}, {MenuSection::Cameras, "Cameras"}, {MenuSection::Tools, "Tools"}};
    for (const auto &[section, title] : sections) {
        bool heading = false;
        for (const auto &entry : entries) {
            if (entry.section != section)
                continue;
            if (!heading)
                sectionTitle(title);
            heading = true;
            ImGui::PushID(entry.key.c_str());
            if (ImGui::MenuItem(entry.label.c_str(), nullptr, *entry.open)) {
                if (*entry.open)
                    *entry.open = false;
                else
                    showWindow(entry);
            }
            if (entry.tooltip && ImGui::IsItemHovered())
                ImGui::SetTooltip("%s", entry.tooltip);
            ImGui::PopID();
        }
        // windows the panels open themselves (Simulation, Run tracking) follow the host's tools
        if (section == MenuSection::Tools && composition_)
            composition_->drawToolMenuItems();
    }
    // Plots: the plot windows, the saved plots, New plot
    ImGui::Separator();
    if (ImGui::BeginMenu("Plots")) {
        plots::drawMenu();
        ImGui::EndMenu();
    }
}

// Layout: built-in presets, saved layouts (load, save, delete), lock and maximize. The Map workspace has one
// arrangement, so only its reset and lock.
void App::drawLayoutMenu() {
    if (workspace_ == Workspace::Map) { // the Map workspace has one arrangement
        if (ImGui::MenuItem("Reset map layout"))
            pendingMapLayout_ = true;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Map objects on the left, the inspector on the right, the pool view between");
        ImGui::MenuItem("Lock layout", nullptr, &layoutLocked_);
        return;
    }

    sectionTitle("Built-in");
    for (const auto &preset : layoutPresets()) {
        if (ImGui::MenuItem(preset.label.c_str(), preset.shortcut.c_str()))
            pendingPreset_ = preset.id;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s\nAlso reopens the default windows.", preset.description.c_str());
    }

    sectionTitle("Saved");
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

// --------------------------------------------------------------------------------------- command bar

// Always visible whatever the layout: branding, the pinned panel controls (Enable / KILL), robot status chips
// and the pose-source pill.
void App::drawCommandBar() {
    beginSurface(Surface::Board); // the board: its own colors in a theme that has them
    const float height = ui(52), padding = ui(9);

    // Editing the map: the bar takes on the accent color, as far as its text stays legible (muted 5:1, text 7:1;
    // at most half way, so Enable keeps its contrast).
    const bool editingMap = scenario_ && workspace_ == Workspace::Map;
    const auto &bar = palette().bar, &accent = palette().active;
    const auto tinted = [&](float t) {
        return ImVec4(bar.x + (accent.x - bar.x) * t, bar.y + (accent.y - bar.y) * t, bar.z + (accent.z - bar.z) * t,
                      1);
    };
    float tint = ruledTheme() ? 0.f : .45f; // a ruled board keeps its ink and takes a lane-blue rule instead
    while (tint > .1f &&
           (contrastRatio(palette().muted, tinted(tint)) < 5 || contrastRatio(palette().text, tinted(tint)) < 7))
        tint -= .025f;
    const ImVec4 editBar = tinted(tint);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(16, padding));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, editingMap ? editBar : bar);
    const bool open = ImGui::BeginViewportSideBar("##command_bar", ImGui::GetMainViewport(), ImGuiDir_Up, height,
                                                  ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoScrollbar |
                                                      ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::PopStyleColor();
    ImGui::PopStyleVar();
    if (open) {
        const float W = ImGui::GetWindowWidth(), inner = height - 2 * padding;
        if (editingMap && tint == 0) { // editing the map on a ruled board: a lane-blue rule along its foot
            const ImVec2 at = ImGui::GetWindowPos();
            ImGui::GetWindowDrawList()->AddRectFilled({at.x, at.y + height - ui(4)}, {at.x + W, at.y + height},
                                                      ImGui::GetColorU32(palette().active));
        }

        // Header text: the scenario's label (host branding may override it), or the map being edited
        const std::string headerTitle = lookup(config_, {"branding", "header"}).as<std::string>("NEREUS");
        std::string headerSubtitle = scenario_ ? scenarioLabel(*scenario_) : "";
        std::transform(headerSubtitle.begin(), headerSubtitle.end(), headerSubtitle.begin(),
                       [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
        headerSubtitle = lookup(config_, {"branding", "subtitle"}).as<std::string>(headerSubtitle);
        if (editingMap) {
            headerSubtitle = "EDITING PRIOR MAP  \u00b7  " +
                             (priorMap_->loaded() ? priorMap_->file().filename().string() : std::string("no file"));
            if (priorMap_->dirty())
                headerSubtitle += "  \u00b7  unsaved";
            if (mapPausedSim_)
                headerSubtitle += "  \u00b7  simulator paused";
            ImGui::PushStyleColor(ImGuiCol_Text, palette().text);
        }
        // the robot and pool (or the map being edited) lead the board; the product's name is the title bar's logo,
        // or the host config's branding header when it sets one
        if (lookup(config_, {"branding", "header"})) {
            ImGui::PushFont(window_->title);
            ImGui::SetCursorPosY(padding + (inner - ImGui::GetFontSize()) * .5f);
            ImGui::TextUnformatted(headerTitle.c_str());
            ImGui::PopFont();
            ImGui::SameLine();
        }
        if (window_->strong)
            ImGui::PushFont(window_->strong);
        ImGui::SetCursorPosY(padding + (inner - ImGui::GetFontSize()) * .5f);
        ImGui::TextUnformatted(headerSubtitle.c_str());
        if (window_->strong)
            ImGui::PopFont();
        if (editingMap)
            ImGui::PopStyleColor();

        if (composition_) { // pinned controls: Enable / KILL and the robot's state
            ImGui::SameLine(0, ui(32));
            ImGui::SetCursorPosY(padding);
            composition_->drawPinned();
        }

        // The right end, right to left: the status pill, Edit map (or Save / Done), the run's clock and score, then the
        // composition's header items
        const float statusWidth = statusChipWidth(status_.c_str());
        const float spacing = ImGui::GetStyle().ItemSpacing.x, edit = editMapButtonWidth();
        // The run on the board: its clock and score as board figures (amber while it runs), left of the chips.
        const bool run = runScore_ && runScore_["total"], running = run && runScore_["running"].as<bool>(false);
        const std::string clock = run ? runTime() : "", points = run ? fixed(runScore_["total"].as<double>(), 1) : "";
        float clockWidth = 0, clockText = 0, pointsText = 0;
        if (run) {
            ImGui::PushFont(window_->number);
            clockText = ImGui::CalcTextSize(clock.c_str()).x;
            pointsText = ImGui::CalcTextSize(points.c_str()).x;
            ImGui::PopFont();
            clockWidth = clockText + ui(16) + pointsText + ui(6) + ImGui::CalcTextSize("pts").x + ui(28);
        }
        const float right = W - 16 - statusWidth - spacing - (edit > 0 ? edit + spacing : 0);
        if (composition_) { // header items (robot telemetry, recording) sit just left of the run, Edit map, status
            ImGui::SetCursorPosY(padding + (inner - ImGui::GetFrameHeight()) * .5f); // a fresh line (no SameLine)
            composition_->drawHeader(right - clockWidth);
        }
        if (run) {
            ImGui::PushFont(window_->number);
            const float figureTop = padding + (inner - ImGui::GetFontSize()) * .5f, baseline = ImGui::GetFontSize();
            ImGui::SameLine(right - clockWidth + ui(12));
            ImGui::SetCursorPosY(figureTop);
            ImGui::TextColored(running ? palette().change : palette().text, "%s", clock.c_str()); // lit while running
            const bool hovered = ImGui::IsItemHovered();
            ImGui::SameLine(0, ui(16));
            ImGui::SetCursorPosY(figureTop);
            ImGui::TextUnformatted(points.c_str());
            ImGui::PopFont();
            ImGui::SameLine(0, ui(6));
            ImGui::SetCursorPosY(figureTop + baseline - ImGui::GetFontSize() - ui(3)); // on the figures' baseline
            ImGui::TextColored(palette().muted, "pts");
            if (hovered || ImGui::IsItemHovered())
                ImGui::SetTooltip("%s", running ? "Run time (simulation time) and score: running"
                                                : "Run time (simulation time) and score: stopped");
        }

        if (edit > 0) {
            ImGui::SameLine(W - 16 - statusWidth - spacing - edit);
            ImGui::SetCursorPosY(padding + (inner - ImGui::GetFrameHeight()) * .5f);
            if (editingMap)
                drawMapButtons();
            else
                drawEditMapButton();
        }

        ImGui::SameLine(W - 16 - statusWidth);
        ImGui::SetCursorPosY(padding + (inner - ImGui::GetFrameHeight()) * .5f);
        pill(status_, ros_->poseFresh() && !demoMode_ ? palette().accent : palette().muted); // a mode, not a warning
    }
    ImGui::End();
    endSurface();
}

// ----------------------------------------------------------------------------------------- pool view

// The 3D pool view: the configured toolbar on top, the rendered scene below with its overlays. It fills the
// dock space's central node; other windows dock around it.
void App::drawPoolView(double time, float dt) {
    ImGuiWindowClass single;
    // no tab bar at all (an auto-hidden one leaves ImGui's little corner marker on the view)
    single.DockNodeFlagsOverrideSet = ImGuiDockNodeFlags_NoTabBar;
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

    // The toolbar strip (the Map workspace has its own), sized to what it drew
    const float width = std::max(16.f, ImGui::GetContentRegionAvail().x);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ui(ImVec2(10, 7)));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ui(ImVec2(6, 6)));
    ImGui::PushStyleColor(ImGuiCol_ChildBg, palette().toolbar);
    ImGui::BeginChild("toolbar", {width, toolbarHeight_}, ImGuiChildFlags_AlwaysUseWindowPadding,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    int oldMode = mode_;
    if (workspace_ == Workspace::Map)
        drawMapToolbar(width);
    else
        drawToolbar(width, oldMode);
    toolbarHeight_ =
        std::max(ImGui::GetFrameHeight() + ui(14), ImGui::GetCursorPosY() - ImGui::GetStyle().ItemSpacing.y + ui(7));
    ImGui::EndChild();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(2);

    // The view below it, and whether the pointer is really on it this frame
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

    // Panel input on the view (the control gizmo), with last frame's camera
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
    const bool operating = workspace_ == Workspace::Operate;
    const bool dragging = operating && composition_ && mode_ == 0 && composition_->input(panelView);

    viewPos_ = position;
    viewSize_ = {width, viewHeight};
    if (planActive() && !planFitted_)
        fitPlan(); // the first 2D view shows the whole pool

    // The prior map editor: its picks, handles and drags come before the camera's (last frame's view).
    const bool editing = mode_ == 0 && priorMap_->active();
    PriorMapEditor::View editorView{viewportView_.projection * viewportView_.view,
                                    {position.x, position.y},
                                    {width, viewHeight},
                                    hovered && !dragging,
                                    planActive(),
                                    priorMapPointer_};
    const bool editorInput = editing && !dragging && priorMap_->input(editorView);
    if (editing)
        priorMap_->shortcuts(hovered);

    if (const auto look = priorMap_->takeFocus(); look && mode_ == 0) { // a prop double-clicked: look at it
        if (planActive()) {
            planCenter_ = glm::vec2(*look);
            planHeight_ = std::min(planHeight_, 8.f);
        } else {
            target_ = *look;
            distance_ = std::min(distance_, 6.f);
            follow_ = false;
        }
    }

    // This frame's camera (input first, unless a panel or the editor took the pointer)
    bool unused = false;
    SensorView view = viewFor(width / viewHeight, dt, hovered && !dragging && !editorInput, viewHeight, unused);
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

    // The scene, built once: the camera cards render from it, then the main view adds its own changes
    rendering::Scene scene;
    {
        PhaseTimer timer{profiler_, Phase::Scene, false};
        scene = model_->build(buildState());
    }
    renderLocalCards(time, scene);
    if (!demoMode_)
        scene.points = ros_->pointSets();     // main view only (cards render without points)
    if (!viewSettings().tiles && scene.water) // main view only: the cameras see the pool as it is
        scene.water->tile_size = 0;

    const bool planLook = planActive() && mapObserver_.planColors;
    if (planLook) { // a chart: flat theme-colored floor and lines, no walls (the overlay draws the rim)
        const auto pal = planPalette();
        const auto hdr = [](ImVec4 c) { return Eigen::Vector3f(hdrFor(c.x), hdrFor(c.y), hdrFor(c.z)); };
        Eigen::Vector3f stripe(.093f, .14f, .16f); // the floor stripes' own color (tint multiplies it)
        for (const auto &s : model_->pack().poolStripes())
            if (s.side == rendering::PoolSide::Floor) {
                stripe = s.color;
                break;
            }
        // Lamp surfaces show 1.65 x their color; markings are lit by the upward ambient term only (flat light).
        const Eigen::Vector3f floorTint = hdr(pal.floor) / 1.65f,
                              lineTint = hdr(pal.line).cwiseQuotient(
                                  Eigen::Vector3f(.48f, .55f, .56f)
                                      .cwiseProduct(stripe.cwiseMax(Eigen::Vector3f::Constant(.01f))));
        for (const auto i : model_->pack().poolFloorInstances()) {
            if (i >= scene.instances.size())
                continue;
            auto &instance = scene.instances[i];
            if (instance.material == rendering::SurfaceMaterial::Tiles) {
                instance.material = rendering::SurfaceMaterial::Lamp;
                instance.tint.head<3>() = floorTint;
                instance.casts_shadow = false;
            } else if (instance.material == rendering::SurfaceMaterial::Marking)
                instance.tint.head<3>() = lineTint;
        }
        for (const auto i : model_->pack().poolWallInstances())
            if (i < scene.instances.size())
                scene.instances[i].visible = false;
        if (scene.water)
            scene.water->tile_size = 0;
    }

    // Render the main view
    const rendering::View renderView{toEigen(view.view), toEigen(view.projection),
                                     Eigen::Vector3f(view.eye.x, view.eye.y, view.eye.z)};
    auto appearance = viewSettings().apply(look_.appearance);
    if (planLook) { // flat light so the chart colors come out exactly (see hdrFor)
        appearance.direct_light = 0;
        appearance.ambient_light = 1;
        appearance.exposure = 1;
        appearance.shadows = appearance.reflections = appearance.surface = false;
        appearance.caustics = appearance.glare = 0;
        appearance.water.absorption.setZero();
        appearance.water.scattering = 0;
        appearance.water.distance_scale = 0;
    }

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

    // The scene point under the pointer (map editor), and F to focus on it (or fit the 2D view)
    priorMapPointer_.reset();
    if (editing && hovered) {
        const auto mouse = io.MousePos;
        priorMapPointer_ = depthPoint(view, frame, {mouse.x - position.x, mouse.y - position.y}, {width, viewHeight});
    }
    if (mode_ == 0 && hovered && !dragging && !io.WantTextInput && ImGui::IsKeyPressed(ImGuiKey_F))
        planActive() ? fitPlan() : focusAtCursor(view, frame, position, width, viewHeight);

    // The image, centered in the view
    const ImVec2 imagePos(position.x + (width - iw) / 2, position.y + (viewHeight - ih) / 2);
    ImGui::SetCursorScreenPos(imagePos);
    // A promoted sensor view shows the bridge's depth image instead while its card is on DEPTH.
    const std::size_t sensorIndex = sensor ? std::size_t(mode_ - 2) : 0;
    if (sensor && sensorIndex < cards_.size() && sensorIndex < ros_->feeds.size() &&
        ros_->feeds[sensorIndex].wantDepth && cards_[sensorIndex].depth)
        ImGui::Image(textureID(cards_[sensorIndex].depth), {iw, ih});
    else
        ImGui::Image(textureID(frame.color_texture), {iw, ih}, {0, 1}, {1, 0});

    // Overlays over the image: TF, detections, MPC path, thrust, panel gizmos, the 2D chart's frame, the map editor
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
    if (operating && composition_ && mode_ == 0) {
        panelView.projection = view.projection;
        panelView.view = view.view;
        panelView.eye = view.eye;
        composition_->drawOverlays(panelView);
    }
    if (planLook)
        drawPlanFrame(vp, {imagePos.x, imagePos.y}, {iw, ih});
    if (editing) {
        editorView.viewProjection = vp;
        priorMap_->drawOverlay(editorView, window_->small);
    }

    auto *d = ImGui::GetWindowDrawList();
    d->AddRect(position, {position.x + width, position.y + viewHeight}, ImGui::GetColorU32(ImGuiCol_Border),
               ImGui::GetStyle().WindowRounding, 0, 1); // the theme's corners (square in a ruled theme)
    // (no pool chip over the scene: the board names the robot and pool, View > Pool lists and switches pools)
    if (poolSwitch_.packs.empty())
        poolSwitch_.packs = scenarioPacks();
    drawPoolSwitchStatus(position);

    // Frame stats (F3), the text refreshed every half second
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
                         {position.x + ui(30) + size.x, position.y + ui(62) + size.y}, chipFill(), 4);
        d->AddText(window_->small, window_->small->FontSize, {position.x + ui(22), position.y + ui(56)},
                   color(palette().text), profileText_.c_str());
    }

    // Course labels: a leader line and a name plate for each focus landmark in view
    if (labels_ && mode_ < 2 && presetFor(focusName_).labels && !priorMap_->hidesCourse()) {
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
            // white over the water whatever the theme (the accent's blue sinks into it)
            d->AddCircleFilled(at, 3, IM_COL32_WHITE);
            d->AddLine(at, {at.x + ui(10), at.y - ui(14)}, IM_COL32_WHITE);
            // a plate fitted to the name, edged in white like its leader line
            ImFont *font = typeRamp().smallStrong ? typeRamp().smallStrong : window_->small;
            const float text = font->CalcTextSizeA(font->FontSize, 1e9f, 0, key.c_str()).x;
            const ImVec2 min(at.x + ui(9), at.y - ui(31)), max(at.x + ui(23) + text, at.y - ui(11));
            d->AddRectFilled(min, max, chipFill(), ui(3));
            d->AddRect(min, max, IM_COL32(255, 255, 255, 140), ui(3), 0, ui(1));
            d->AddText(font, font->FontSize, {at.x + ui(16), at.y - ui(28)}, color(palette().text), key.c_str());
        }
    }

    // the controls strip: the longest wording that fits the view's width
    const char *const hints[3][3] = {
        {"LEFT DRAG  orbit   RIGHT / MIDDLE DRAG  pan   SCROLL  zoom   F  focus cursor",
         "DRAG  orbit   RIGHT DRAG  pan   SCROLL  zoom   F  focus", "DRAG  orbit   SCROLL  zoom"},
        {"CLICK  mouse look    WASD  move    SPACE / SHIFT  up / down    CTRL  fast    ESC  release",
         "WASD  move   SPACE / SHIFT  up / down   ESC  release", "WASD  move   ESC  release"},
        {"DRAG FLOOR / RIGHT / MIDDLE DRAG  pan   SCROLL  zoom   F  fit pool   DOUBLE-CLICK  center a prop",
         "DRAG  pan   SCROLL  zoom   F  fit pool", "DRAG  pan   SCROLL  zoom"}};
    const int hintSet = planActive() ? 2 : mode_ == 1 ? 1 : 0;
    std::vector<std::string> wordings(std::begin(hints[hintSet]), std::end(hints[hintSet]));
    if (driving_ && !planActive() && !mouseCaptured_) { // keyboard driving: its keys and step instead
        char step[48];
        std::snprintf(step, sizeof(step), "%g m, %g\u00b0", double(kDriveMeters[driveStep_]),
                      double(kDriveDegrees[driveStep_]));
        wordings = {std::string("DRIVING   W / S  forward / back   A / D  left / right   SPACE / SHIFT  up / down   "
                                "Q / E  turn   [ / ]  step ") +
                        step + "   ESC  stop",
                    std::string("DRIVING   WASD  move   SPACE / SHIFT  up / down   Q / E  turn   [ ]  ") + step +
                        "   ESC  stop",
                    std::string("DRIVING   ") + step + "   ESC  stop"};
    }
    const char *controls = wordings.back().c_str();
    for (const auto &hint : wordings)
        if (window_->small->CalcTextSizeA(window_->small->FontSize, 1e9f, 0, hint.c_str()).x + ui(28) <= width) {
            controls = hint.c_str();
            break;
        }
    d->AddRectFilled({position.x, position.y + viewHeight - ui(30)}, {position.x + width, position.y + viewHeight},
                     chipFill());
    d->AddText(window_->small, window_->small->FontSize, {position.x + ui(14), position.y + viewHeight - ui(21)},
               color(palette().text), controls);

    for (const Side side : {Side::Left, Side::Right})
        drawSideHandle(side, position, {width, viewHeight}, true);
    ImGui::End();
}

// -------------------------------------------------------------------------------------- side columns

// Snap shut: a side dragged narrower than kSnapWidth closes, like the old sidebars' edges; pulled back past
// kOpenWidth it reopens; widths from kRememberWidth up are what it reopens at otherwise.
constexpr float kSnapWidth = 150, kOpenWidth = 180, kRememberWidth = 220;

// Whether a side column is open: not snapped shut, and with some width in the dock layout.
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

// Snaps a side shut: closes its windows (remembered in `keys`) and keeps a usable width for when it reopens.
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

// Reopens the windows a side closed with.
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

// Forgets the sides' snapped-shut state (a layout or workspace replaces the windows).
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

// --------------------------------------------------------------------------- camera and help windows

// One window per open camera card (closed ones with pinned controls are drawn unseen); the ROS side's camera
// subscriptions follow whether any is shown.
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
        // no scrollbar: the image is fitted to the window (a scrollbar coming and going would change the width the
        // card lays out in, and the card would jitter between two sizes)
        const bool visible = ImGui::Begin(name.c_str(), &cards_[i].open,
                                          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
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

// Controls & shortcuts (F1): the pool view's mouse and keys, robot moves, shortcuts and window handling.
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
                if (typeRamp().strong) // the keys in bold ink (color stays for state)
                    ImGui::PushFont(typeRamp().strong);
                ImGui::TextUnformatted(keys);
                if (typeRamp().strong)
                    ImGui::PopFont();
                ImGui::TableNextColumn();
                ImGui::TextWrapped("%s", action);
            }
            ImGui::EndTable();
        };

        sectionTitle("Pool view");
        table("view_keys",
              {{"Left drag", "orbit"},
               {"Right / middle drag", "pan (detaches Follow)"},
               {"Scroll", "zoom"},
               {"F", "focus on the point under the cursor"},
               {"Free camera", "click for mouse look, WASD move, Space / Shift up / down, Ctrl fast, "
                               "Esc release"},
               {"Gizmo", "drag an arrow to move, a ring to rotate; Esc during a drag restores the start"}});

        sectionTitle("Robot from the keyboard");
        table("robot_keys",
              {{"Ctrl+P, then a move", "forward / back / left / right / up / down 0.5 (meters, along the heading), "
                                       "turn 30 (left positive), x / y / z -1.5, roll / pitch / yaw 90, "
                                       "go 2 1 -1.5 90, level; several in a row; Enter sends it"},
               {"Drive with keys", "Ctrl+P or the Motion panel (Position control): W / S forward / back, A / D "
                                   "left / right, Space / Shift up / down, Q / E turn: one step a press; [ / ] step "
                                   "size; Esc stops"},
               {"Ctrl+P, Run <tree>", "start an autonomy tree (robot enabled); Stop the tree from there too"},
               {"Enable / KILL", "never from the keyboard: the button in the command bar"}});

        sectionTitle("Shortcuts");
        table("shortcuts", {{"Ctrl+P", "search: windows, layouts, themes, controls, trees; robot moves"},
                            {"Ctrl+P, plot odom z", "a field or Motion axis in a plot (Shift+Enter: the focused one)"},
                            {"Ctrl+M", "edit the prior map / done"},
                            {"Ctrl+S", "save the prior map"},
                            {"F12", "save a screenshot (Pictures/Nereus)"},
                            {"Ctrl+Q", "quit (asks first if the prior map has unsaved edits)"},
                            {"Ctrl+Space", "maximize the pool view / restore the layout"},
                            {"Ctrl+Shift+1 / 2 / 3", "Standard / Wide view / Camera wall layout"},
                            {"Ctrl+[ / Ctrl+]", "snap the left / right panels shut, or bring them back"},
                            {"F1", "this window"},
                            {"F3", "frame-time stats"}});

        sectionTitle("Windows");
        table("windows", {{"Move", "drag a tab onto a window to dock it (the arrows show where), beside it to split, "
                                   "away to float it"},
                          {"Resize", "drag the borders between windows"},
                          {"Close / reopen", "a tab's x / the Windows menu"},
                          {"Layouts", "saved when the viewer closes; Layout > Save current as, Layout > Standard"},
                          {"Pin", "right-click a tab, or any control in a panel, to put it on the toolbar"},
                          {"Toolbar", "right-click it (or its +) to choose its buttons"},
                          {"Look", "View > Theme, View > Interface scale"}});
    }
    ImGui::End();
}

// Global shortcuts (none while a text field has the keyboard); the Help window lists them.
void App::handleShortcuts() {
    if (ImGui::GetIO().WantTextInput)
        return;
    if (ImGui::Shortcut(ImGuiMod_Ctrl | ImGuiKey_Space, ImGuiInputFlags_RouteGlobal))
        toggleMaximized();
    if (ImGui::Shortcut(ImGuiMod_Ctrl | ImGuiKey_M, ImGuiInputFlags_RouteGlobal))
        setWorkspace(workspace_ == Workspace::Map ? Workspace::Operate : Workspace::Map);
    const auto &presets = layoutPresets();
    for (std::size_t i = 0; i < presets.size() && i < 9; ++i)
        if (ImGui::Shortcut(ImGuiMod_Ctrl | ImGuiMod_Shift | ImGuiKey(ImGuiKey_1 + int(i)),
                            ImGuiInputFlags_RouteGlobal) &&
            workspace_ == Workspace::Operate)
            pendingPreset_ = presets[i].id;
    if (ImGui::Shortcut(ImGuiKey_F1, ImGuiInputFlags_RouteGlobal))
        helpOpen_ = !helpOpen_;
    if (ImGui::Shortcut(ImGuiMod_Ctrl | ImGuiKey_Q, ImGuiInputFlags_RouteGlobal))
        window_->requestClose(); // asks first when the prior map has unsaved edits
    if (ImGui::Shortcut(ImGuiKey_F12, ImGuiInputFlags_RouteGlobal))
        requestScreenshot();
    // Ctrl+S saves the prior map from anywhere (map editing handles its own)
    if (workspace_ != Workspace::Map && priorMap_->dirty() &&
        ImGui::Shortcut(ImGuiMod_Ctrl | ImGuiKey_S, ImGuiInputFlags_RouteGlobal))
        priorMap_->save();
    if (ImGui::Shortcut(ImGuiMod_Ctrl | ImGuiKey_P, ImGuiInputFlags_RouteGlobal) && scenario_)
        paletteFocus_ = true; // the title bar's search box takes the keyboard
    if (ImGui::Shortcut(ImGuiMod_Ctrl | ImGuiKey_LeftBracket, ImGuiInputFlags_RouteGlobal))
        toggleSide(Side::Left);
    if (ImGui::Shortcut(ImGuiMod_Ctrl | ImGuiKey_RightBracket, ImGuiInputFlags_RouteGlobal))
        toggleSide(Side::Right);
}

// --------------------------------------------------------------------------------------------- layout

// Every host window of this workspace, as the Windows menu, pins, layouts and the palette see them: the panels,
// the camera cards, then the tool windows (the Map workspace: the editor's windows and Help).
std::vector<App::WindowEntry> App::windowEntries() {
    std::vector<WindowEntry> entries;
    if (workspace_ == Workspace::Map) {
        entries.push_back({"map_objects", kMapObjects, "Map objects", &priorMap_->objectsOpen()});
        entries.push_back({"map_inspector", kMapInspector, "Inspector", &priorMap_->inspectorOpen()});
        entries.push_back({"help", kHelp, "Controls & shortcuts", &helpOpen_, MenuSection::None});
        return entries;
    }
    if (composition_) {
        const auto flags = composition_->visibility();
        const auto windows = composition_->panelWindows(); // same order
        for (std::size_t i = 0; i < flags.size() && i < windows.size(); ++i)
            entries.push_back({flags[i].first, windows[i].name, windows[i].name.substr(0, windows[i].name.find("###")),
                               flags[i].second, MenuSection::Panels});
    }
    if (scenario_)
        for (std::size_t i = 0; i < scenario_->cameras.size() && i < cards_.size(); ++i)
            entries.push_back({"camera." + scenario_->cameras[i].id, cameraWindowName(scenario_->cameras[i]),
                               scenario_->cameras[i].title, &cards_[i].open, MenuSection::Cameras});
    entries.push_back({"map", kCourseMap, "Course map", &mapOpen_, MenuSection::Cameras});
    entries.push_back({"scene_settings", kSceneSettings, "Scene settings", &sceneOpen_});
    entries.push_back({"display", kDisplay, "Display", &displayOpen_});
    entries.push_back({"tf", kTfFrames, "TF frames", &tfOpen_});
    for (const auto &window : plots::windows())
        entries.push_back({window.key, window.name, window.label, window.open,
                           window.plot ? MenuSection::Plots : MenuSection::Tools});
    entries.push_back({"help", kHelp, "Controls & shortcuts", &helpOpen_, MenuSection::None});
    return entries;
}

// Each window's open flag by key, saved and restored with the layout (WindowStates).
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

// Back to the configured toolbar: its default items, nothing pinned.
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
                                                           {"recording", "Recording"},
                                                           {"bagging", "Bags"}};
    const auto found = labels.find(type);
    return found == labels.end() ? type : found->second;
}
} // namespace

// The toolbar's customize popup: which items show, pinned controls, pinned windows, and a reset.
void App::drawToolbarCustomization() {
    if (composition_ && !composition_->toolbarItems().empty()) {
        sectionTitle("Toolbar buttons");
        for (const auto &item : composition_->toolbarItems()) {
            if (item.type == "separator") // dividers come and go with the groups around them
                continue;
            ImGui::PushID(item.id.c_str());
            ImGui::Checkbox(item.title.empty() ? toolbarLabel(item.type).c_str() : item.title.c_str(), item.visible);
            ImGui::PopID();
        }
    }

    sectionTitle("Pinned controls");
    pins::drawCustomization();

    sectionTitle("Pinned windows");
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

// The host yaml's `layout` preset, else standard.
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

// Rebuilds the dock layout from a built-in preset (the configured one if `id` is unknown), with the windows back
// in their configured open states.
void App::applyPreset(const std::string &id) {
    const auto *found = findPreset(id);
    LayoutPreset preset = found ? *found : layoutPresets().front();
    const auto size = ImGui::GetMainViewport()->WorkSize;
    if (preset.id == "standard" && composition_ && size.x > 0) // the composition's sidebar width
        preset.left = std::clamp(composition_->width(size.x) / size.x, .15f, .45f);
    // The side columns and the upper left panel never go below what their content needs at this interface scale
    // (a small window or a large scale), so tabs and tables keep their content instead of truncating; a large
    // window keeps the preset's shares.
    if (size.x > 0 && size.y > 0) {
        preset.left =
            std::clamp(ui(400) / size.x, preset.left, std::max(preset.left, .32f)); // the pool view keeps 36 %+
        preset.right = std::clamp(ui(400) / size.x, preset.right, std::max(preset.right, .32f));
        preset.leftTop = std::clamp(ui(620) / size.y, preset.leftTop, std::max(preset.leftTop, .82f));
    }

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

    // Where each window docks: the cameras right, the course map lower right, panels where the composition says
    std::vector<LayoutWindow> windows;
    for (const auto &camera : scenario_->cameras)
        windows.push_back({cameraWindowName(camera), Dock::Right, true, false});
    windows.push_back({kCourseMap, Dock::RightBottom, false, false});
    if (composition_)
        for (const auto &panel : composition_->panelWindows())
            windows.push_back({panel.name, panel.dock, false, panel.selected});
    // open plot windows share a strip under the pool view; the Topics browser joins the lower left panels
    for (const auto &window : plots::windows())
        if (*window.open)
            windows.push_back({window.name, window.plot ? Dock::Bottom : Dock::Left, false, false});
    buildLayout(dockspace_, size, preset, kPoolView, windows);
    layoutReady_ = true;
    layoutMessage_ = preset.label + " layout";
}

// Layout > Save current as: the current layout as <layout directory>/<name>.ini.
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
    if (workspace_ == Workspace::Map) { // the session layout is Operate's; the Map one has its own file
        std::ofstream(sessionIni_) << operateIni_;
        if (!mapIniFile_.empty())
            std::ofstream(mapIniFile_) << layoutSnapshot();
        return;
    }
    std::ofstream out(sessionIni_);
    out << layoutSnapshot();
}

// Brings `window` to the front on this frame's Begin when showWindow asked for it.
void App::focusIfRequested(const std::string &window) {
    if (focusOnce_.erase(window))
        ImGui::SetNextWindowFocus();
}

// ----------------------------------------------------------------------------- camera card rendering

// Local camera cards: this viewer's own render from each sensor pose (the truth pose live, the fixed preview
// pose in demo mode). Scene preview has no bridge; live cards can switch to the bridge's images per card.
// Cost control: at most ONE card per frame (each card refreshes every 0.1 s, staggered by the round-robin), only
// cards drawn on screen whose RGB is not replaced by the depth image or the ROS feed, the main frame's scene
// is reused, and the renderer's preview mode skips shadow/bloom/reflection passes.

// A card's refresh period in seconds: --card-rate or cards.rate_hz, else the camera's own rate (15 Hz without one).
double App::cardPeriod(std::size_t camera) const {
    const double rate = opt_.cardRate >= 0 ? opt_.cardRate : lookup(config_, {"cards", "rate_hz"}).as<double>(0);
    if (rate > 0)
        return 1. / rate;
    const double own = camera < scenario_->cameras.size() ? scenario_->cameras[camera].periodS : 1. / 15;
    return own > 0 ? own : 1. / 15;
}

// Renders the camera cards that are due this frame from the robot's sensor poses, into their card textures.
void App::renderLocalCards(double t, const rendering::Scene &mainScene) {
    if (!scenario_ || !model_ || !(demoMode_ || opt_.localCameras) || !camerasShown_)
        return;
    if (!demoMode_ && !(ros_->truthActive() && ros_->poseFresh()))
        return; // no simulator truth pose to render from

    // The due cards: the next visible RGB one in round-robin order (every due one with --legacy-cards)
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

    // Each card at 480 px wide (the camera's aspect), its intrinsics scaled to match
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

        // Copy the frame into the card's texture (rows bottom-up: the card draws it flipped)
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

// Saves each camera card beside a screenshot as <screenshot>-<camera id>.png (local renders read back from GL).
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

// ----------------------------------------------------------------------------------------- main loop

// The frame loop, until the window closes or ROS shuts down: spin ROS, take a pending scenario, step, draw the
// interface, present and swap, then pace the frame. A capture run (--frames) stops after its last frame.
int App::loop() {
    int frames = 0;
    auto previous = start_;
    while (true) {
        // Closing with unsaved prior map edits asks first (drawUnsavedMapPrompt).
        if (window_->closing() && priorMap_->dirty() && !closeConfirmed_ && opt_.frames == 0) {
            window_->cancelClose();
            closePrompt_ = true;
        }
        if (window_->closing() || !(demoMode_ || rclcpp::ok()))
            break;

        const auto frameStart = Clock::now();
        const double t = std::chrono::duration<double>(frameStart - start_).count();
        const float dt = float(std::min(std::chrono::duration<double>(frameStart - previous).count(), .1));
        previous = frameStart;

        // ROS callbacks, then the pool switch's progress
        {
            PhaseTimer timer{profiler_, Phase::Spin, false};
            ros_->spin();
        }
        updatePoolSwitch();

        // the first scenario: one frame of "Building the scene" first (kept: the loading thread draws over it)
        if (!pendingScenario_.empty() && !scenario_ && !buildingShown_)
            buildingShown_ = true;
        else if (!pendingScenario_.empty()) {
            const auto json = std::move(pendingScenario_);
            pendingScenario_.clear();
            try {
                loadScenario(json); // the loading screen keeps moving meanwhile (animateLoadingWhile)
            } catch (const std::exception &error) {
                std::cerr << "nereus-viewer: rejecting scenario document: " << error.what() << '\n';
            }
            buildingShown_ = false;
        }

        if (composition_)
            composition_->touch();
        step(t);

        // --inject-f test aid: hover a point and press F halfway through the capture run
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
        applyPendingIni();   // a layout chosen last frame (saved layout, maximize restore)
        applyPendingTheme(); // before the scale: a theme with its own fonts reloads them
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

        // Present (saving a screenshot when one is due), keep the layout saved, swap
        const bool shot = screenshotCountdown_ > 0 && --screenshotCountdown_ == 0; // File > Save screenshot
        window_->present(last || shot, shot ? screenshotPath_ : fs::path(opt_.screenshot));
        if (shot)
            notify("Saved screenshot " + screenshotPath_.string());
        if (loadingOnThread())
            window_->keepFrame(); // the "Building the scene" frame: under the loading thread's frames
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

        // A capture run's last frame: a summary line on stdout, then the camera images beside the screenshot
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

        // Frame cap: --render-rate, else 30 Hz hidden, uncapped with vsync (it paces), 60 Hz otherwise
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

// The viewer's entry point (app.hpp).
int run(const Options &options, int argc, char **argv) {
    App app(options, argc, argv);
    return app.loop();
}
} // namespace nereus::ros_viewer::host

namespace nereus::ros_viewer::host {
// ------------------------------------------------------------------------------------------- pool switch

// File > Switch pool: the scenario packs found, the current pool ticked; disabled while a switch is under way, or
// when the simulator was not started by its supervisor.
void App::drawPoolMenu() {
    auto &s = poolSwitch_;
    if (ImGui::IsWindowAppearing() || s.packs.empty())
        s.packs = scenarioPacks();
    const bool supervised = !s.fromTopic || !s.supervisorState.empty();
    const bool busy = !s.target.empty();
    if (s.packs.empty())
        ImGui::TextDisabled("No scenario packs in %s", (packContent() / "scenarios").c_str());
    for (const auto &pack : s.packs) {
        // two scenarios in one pool: tell them apart by their folder
        const auto same = std::count_if(s.packs.begin(), s.packs.end(),
                                        [&](const ScenarioPack &p) { return p.poolId == pack.poolId; });
        const std::string label =
            same > 1 ? pack.poolLabel + " (" + pack.folder.filename().string() + ")" : pack.poolLabel;
        const bool current = scenario_ && pack.poolId == scenario_->poolId;
        ImGui::PushID(pack.folder.c_str());
        if (ImGui::MenuItem(label.c_str(), nullptr, current, supervised && !busy) && !current)
            switchPool(pack);
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip("%s\n%s\n%s", pack.poolDescription.c_str(), pack.folder.c_str(),
                              !s.fromTopic ? "Reloads this view in that pool (the robot is not affected)"
                              : supervised ? "Restarts the simulator in that pool: the run starts over, the robot "
                                             "stack keeps running"
                                           : "Start the simulator with sim.launch.py to switch its pool");
        ImGui::PopID();
    }
    if (busy)
        ImGui::TextDisabled("Switching to %s ...", s.targetLabel.c_str());
    else if (!supervised)
        ImGui::TextDisabled("The simulator was not started by sim.launch.py:\nrestart it with pool:= to switch");
}

// Starts switching to `pack`'s pool: through the simulator's supervisor when the scenario comes from its topic,
// else by resolving the pack on a worker thread (picked up by updatePoolSwitch).
void App::switchPool(const ScenarioPack &pack) {
    auto &s = poolSwitch_;
    if (!s.target.empty() || s.worker.joinable())
        return;
    s.target = pack.poolId;
    s.targetLabel = pack.poolLabel;
    s.messageError = false;
    s.messageUntil = {};
    if (s.fromTopic) { // the supervisor restarts the simulator; its new scenario arrives on the topic
        s.message = "Restarting the simulator in " + pack.poolLabel + " ...";
        ros_->requestScenario(s.topicBase + "/load_scenario", pack.folder.string());
        return;
    }
    s.message = "Loading " + pack.poolLabel + " ...";
    s.done = false;
    s.worker = std::thread([this, folder = pack.folder] {
        std::string resolved, error;
        try {
            resolved = resolveScenarioPack(folder);
        } catch (const std::exception &e) {
            error = e.what();
        }
        std::lock_guard<std::mutex> lock(poolSwitch_.mutex);
        poolSwitch_.resolved = std::move(resolved);
        poolSwitch_.error = std::move(error);
        poolSwitch_.done = true;
    });
}

// Each frame: a locally resolved pack goes to the loader; a switch ends when the new pool is shown.
void App::updatePoolSwitch() {
    auto &s = poolSwitch_;
    if (s.worker.joinable()) {
        bool done;
        {
            std::lock_guard<std::mutex> lock(s.mutex);
            done = s.done;
        }
        if (done) {
            s.worker.join();
            if (!s.error.empty()) {
                s.message = "Pool not switched: " + s.error;
                s.messageError = true;
                s.messageUntil = Clock::now() + std::chrono::seconds(12);
                s.target.clear();
            } else
                pendingScenario_ = std::move(s.resolved);
            s.resolved.clear();
            s.error.clear();
        }
    }
    if (!s.target.empty() && scenario_ && scenario_->poolId == s.target) {
        if (!s.messageError || Clock::now() >= s.messageUntil) {
            s.message = "Pool: " + s.targetLabel;
            s.messageError = false;
            s.messageUntil = Clock::now() + std::chrono::seconds(4);
        }
        s.target.clear();
    }
}

// The pool switch's (and notify()'s) status chip at the pool view's top left, while its time lasts.
void App::drawPoolSwitchStatus(ImVec2 position) {
    const auto &s = poolSwitch_;
    if (s.message.empty() || (s.target.empty() && Clock::now() >= s.messageUntil))
        return;
    auto *d = ImGui::GetWindowDrawList();
    const ImVec2 size = window_->small->CalcTextSizeA(window_->small->FontSize, 1e9f, 0, s.message.c_str());
    const ImVec2 at(position.x + ui(14), position.y + ui(50));
    d->AddRectFilled(at, {at.x + size.x + ui(16), at.y + size.y + ui(10)},
                     ImGui::GetColorU32(s.messageError ? palette().dangerPressed : palette().active), ui(4));
    d->AddText(window_->small, window_->small->FontSize, {at.x + ui(8), at.y + ui(5)},
               ImGui::GetColorU32(s.messageError ? palette().dangerText : palette().activeText), s.message.c_str());
}
} // namespace nereus::ros_viewer::host

namespace nereus::ros_viewer::host {
// ------------------------------------------------------------------------------------------- workspaces

// Map editing is a mode stepped into from Operate: "Edit map" in the command bar; while editing, the command bar
// takes the accent color, says so, and holds Save and Done. Each keeps its own window layout; Enable / KILL stay in
// the command bar. The map editor: its two windows, the map tools and a 2D view.

// The command bar's room for Edit map (Operate) or Save + Done (Map).
float App::editMapButtonWidth() const {
    if (!scenario_)
        return 0;
    if (workspace_ == Workspace::Map)
        return buttonWidth("Save") + buttonWidth("Done") + ImGui::GetStyle().ItemSpacing.x;
    return buttonWidth(priorMap_->dirty() ? "Edit map*" : "Edit map");
}

void App::drawMapButtons() {
    const float y = ImGui::GetCursorPosY();
    ImGui::BeginDisabled(!priorMap_->loaded() || !priorMap_->dirty());
    if (ImGui::Button("Save###bar_save"))
        priorMap_->save();
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Write the prior map (Ctrl+S)");
    ImGui::SameLine();
    ImGui::SetCursorPosY(y);
    if (ImGui::Button("Done###bar_done"))
        setWorkspace(Workspace::Operate);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(priorMap_->dirty() ? "Back to operating; unsaved edits stay here until you save or quit "
                                               "(Ctrl+M)"
                                             : "Back to operating the robot (Ctrl+M)");
}

void App::drawEditMapButton() {
    if (ImGui::Button(priorMap_->dirty() ? "Edit map*###edit_map" : "Edit map###edit_map"))
        setWorkspace(Workspace::Map);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(priorMap_->dirty() ? "Edit the robot's prior map: it has unsaved changes  (Ctrl+M)"
                                             : "Edit the robot's prior map (riptide_mapping config.yaml)  (Ctrl+M)");
}

// Switches between Operate and Map. Each keeps its own window layout; entering Map pauses a running simulator and
// Follow, leaving restores them and saves the Map layout.
void App::setWorkspace(Workspace next) {
    if (next == workspace_ || !scenario_)
        return;
    const auto snapshot = layoutSnapshot(); // the layout from before when maximized
    maximized_ = false;
    resetSides();

    if (next == Workspace::Map) {
        operateIni_ = snapshot;
        // The map is edited against a still scene: pause a running simulator, stop following the robot.
        followBeforeMap_ = follow_;
        follow_ = false;
        mapPausedSim_ = false;
        if (const auto sim = simulation()) {
            const auto state = sim->state();
            if (state.connected && !state.pending && state.rate > 0) {
                sim->setPaused(true);
                mapPausedSim_ = true;
                poolSwitch_.message = "Simulator paused while the map is edited (Operate resumes it)";
                poolSwitch_.messageError = false;
                poolSwitch_.messageUntil = Clock::now() + std::chrono::seconds(5);
            }
        }
        workspace_ = next;
        if (mode_ != 0)
            setViewMode(0);
        if (!mapIni_.empty())
            pendingIni_ = mapIni_;
        else
            pendingMapLayout_ = true;
    } else {
        mapIni_ = snapshot;
        workspace_ = next;
        pendingIni_ = operateIni_;
        follow_ = followBeforeMap_;
        if (mapPausedSim_)
            if (const auto sim = simulation())
                sim->setPaused(false);
        mapPausedSim_ = false;
        if (persist_ && !mapIniFile_.empty()) {
            std::error_code error;
            fs::create_directories(mapIniFile_.parent_path(), error);
            std::ofstream(mapIniFile_) << mapIni_;
        }
    }
    priorMap_->setActive(workspace_ == Workspace::Map);
}

// The composition's providers the host drives itself: Simulation (paused for map editing), Motion and Autonomy
// (the palette and keyboard driving; none in --demo).
std::shared_ptr<panels::Simulation> App::simulation() const {
    if (!composition_)
        return nullptr;
    for (const auto &[name, provider] : composition_->providers())
        if (provider && provider->kind() == panels::Kind::Simulation)
            return std::dynamic_pointer_cast<panels::Simulation>(provider);
    return nullptr;
}

std::shared_ptr<panels::Motion> App::motion() const {
    if (!composition_ || demoMode_)
        return nullptr;
    for (const auto &[name, provider] : composition_->providers())
        if (provider && provider->kind() == panels::Kind::Motion)
            return std::dynamic_pointer_cast<panels::Motion>(provider);
    return nullptr;
}

std::shared_ptr<panels::Autonomy> App::autonomy() const {
    if (!composition_ || demoMode_)
        return nullptr;
    for (const auto &[name, provider] : composition_->providers())
        if (provider && provider->kind() == panels::Kind::Autonomy)
            return std::dynamic_pointer_cast<panels::Autonomy>(provider);
    return nullptr;
}

// As the motion panel's Command: the robot enabled, its pose fresh, and nobody else (autonomy, another viewer)
// commanding it. A mode change in flight is only a wait.
bool App::commandable(std::string *why, bool *wait) const {
    const auto m = motion();
    const auto s = m ? m->state() : panels::MotionState{};
    const char *reason = !m            ? "No motion control in this session"
                         : !s.fresh    ? "Waiting for the robot's pose"
                         : !s.enabled  ? "Enable the robot to command a pose (Enable is not in the search)"
                         : s.blocked   ? "Autonomy is driving: stop the tree to command a pose"
                         : s.competing ? "Another operator is commanding the robot"
                         : s.mode == panels::Mode::Feedforward
                             ? "In Feedforward control: switch the Motion panel to Position to command a pose"
                             : nullptr;
    if (wait)
        *wait = !reason && s.pending;
    if (why)
        *why = reason ? reason : "";
    return !reason && !s.pending;
}

PoseTarget App::poseBase() const {
    const auto m = motion();
    if (!m)
        return {};
    const auto s = m->state();
    const glm::mat4 &from = s.hasCommand && s.mode == panels::Mode::Position ? s.commanded : s.actual;
    PoseTarget base;
    base.position = glm::vec3(from[3]);
    base.degrees = glm::degrees(glm::eulerAngles(glm::quat_cast(glm::mat3(from))));
    // the last keyboard target until the robot reports it (it arrives a few frames later), so presses add up
    if (sent_ && Clock::now() - sentAt_ < std::chrono::seconds(1))
        return *sent_;
    return base;
}

// Sends a pose target in Position control, if the robot takes commands now.
void App::commandPose(const PoseTarget &target) {
    if (const auto m = motion(); m && commandable(nullptr)) {
        m->activate(panels::Mode::Position, nereus::ros_viewer::rpyPose(target.position, glm::radians(target.degrees)));
        sent_ = target;
        sentAt_ = Clock::now();
    }
}

void App::setDriving(bool on) {
    std::string why;
    bool wait = false;
    if (on && !commandable(&why, &wait) && !wait) {
        notify(why, true);
        return;
    }
    driving_ = on;
}

// Keyboard driving: each press moves the target one step from where it was told to be (no repeat while held,
// so a held key never runs it away). Off when a text field has the keyboard, the mouse looks around (free
// camera), or in the Map workspace; it ends on Esc or when the robot stops taking poses (killed, autonomy).
void App::driveWithKeys() {
    if (!driving_)
        return;
    std::string why;
    bool wait = false;
    if (workspace_ != Workspace::Operate || (!commandable(&why, &wait) && !wait)) {
        driving_ = false;
        notify(why.empty() ? "Keyboard driving off" : "Keyboard driving off: " + why, !why.empty());
        return;
    }

    const auto &io = ImGui::GetIO();
    if (io.WantTextInput || mouseCaptured_ || io.KeyCtrl || io.KeyAlt)
        return;
    if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
        driving_ = false;
        return;
    }

    // [ / ] change the step size
    if (ImGui::IsKeyPressed(ImGuiKey_LeftBracket, false))
        driveStep_ = std::max(0, driveStep_ - 1);
    if (ImGui::IsKeyPressed(ImGuiKey_RightBracket, false))
        driveStep_ = std::min(int(std::size(kDriveMeters)) - 1, driveStep_ + 1);
    if (wait)
        return;

    // One step per key press, along and about the robot's heading
    const float meters = kDriveMeters[driveStep_], degrees = kDriveDegrees[driveStep_];
    float forward = 0, left = 0, up = 0, turn = 0;
    const auto pressed = [](ImGuiKey key) { return ImGui::IsKeyPressed(key, false); };
    forward += pressed(ImGuiKey_W) ? meters : 0;
    forward -= pressed(ImGuiKey_S) ? meters : 0;
    left += pressed(ImGuiKey_A) ? meters : 0;
    left -= pressed(ImGuiKey_D) ? meters : 0;
    up += pressed(ImGuiKey_Space) ? meters : 0; // as the free camera
    up -= pressed(ImGuiKey_LeftShift) || pressed(ImGuiKey_RightShift) ? meters : 0;
    turn += pressed(ImGuiKey_Q) ? degrees : 0;
    turn -= pressed(ImGuiKey_E) ? degrees : 0;
    if (forward != 0 || left != 0 || up != 0 || turn != 0)
        commandPose(stepped(poseBase(), forward, left, up, turn));
}

// The Map workspace's arrangement: the object tree left, the inspector right, the pool view between.
void App::applyMapLayout() {
    LayoutPreset preset;
    preset.id = "map";
    preset.label = "Map";
    preset.left = .36f; // the object table with its pose columns
    preset.right = .25f;
    priorMap_->objectsOpen() = priorMap_->inspectorOpen() = true;
    resetSides();
    maximized_ = false;
    buildLayout(dockspace_, ImGui::GetMainViewport()->WorkSize, preset, kPoolView,
                {{kMapObjects, Dock::Left, false, true}, {kMapInspector, Dock::Right, false, true}});
    layoutReady_ = true;
}

// The pool view toolbar in the Map workspace: 3D / 2D, the map tools, the pool.
void App::drawMapToolbar(float width) {
    (void)width;
    int view = planView_ ? 1 : 0;
    if (pins::Switch("View##map_view", &view, {"3D", "2D"})) {
        planView_ = view == 1;
        if (planView_ && !planFitted_)
            fitPlan();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("3D: orbit, and move props in height too.\n2D: straight down at the pool; drag props across "
                          "it, height stays put.");
    if (planView_) {
        ImGui::SameLine();
        if (ImGui::Button("Fit"))
            fitPlan();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("The whole pool (F)");
    }

    // Display: map editing's own look
    sameLineIfFits(buttonWidth("Display"));
    if (ImGui::Button("Display"))
        ImGui::OpenPopup("map_display");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("What the pool view draws, and its lighting (map editing has its own)");
    if (ImGui::BeginPopup("map_display")) {
        ImGui::Checkbox("Water", &mapObserver_.water);
        ImGui::Checkbox("Pool walls & deck", &mapObserver_.walls);
        ImGui::Checkbox("Pool floor", &mapObserver_.floor);
        ImGui::Checkbox("Plan colors (2D)", &mapObserver_.planColors);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(
                "The 2D view as a chart: flat theme colors, the floor, its lines and the props' outlines");
        ImGui::Checkbox("Pool tiles", &mapObserver_.tiles);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("The tile grout on the walls and floor; the lane lines stay.");
        if (!model_->pack().equipmentInstances().empty())
            ImGui::Checkbox("AprilTag board", &look_.equipment);
        ImGui::Checkbox("Surface reflections", &mapObserver_.reflections);
        drawMapLighting();
        ImGui::EndPopup();
    }

    sameLineIfFits(buttonWidth("Place origin"));
    priorMap_->drawViewTools();

    // The pool dropdown (the same list as File > Switch pool)
    if (poolSwitch_.packs.empty())
        poolSwitch_.packs = scenarioPacks();
    std::string current = scenario_->poolId;
    for (const auto &pack : poolSwitch_.packs)
        if (pack.poolId == scenario_->poolId)
            current = pack.poolLabel;
    const float poolWidth = std::min(ui(260), ImGui::CalcTextSize(current.c_str()).x + ui(40));
    sameLineIfFits(poolWidth);
    ImGui::SetNextItemWidth(poolWidth);
    if (ImGui::BeginCombo("##pool", current.c_str(), ImGuiComboFlags_HeightLarge)) {
        drawPoolMenu();
        ImGui::EndCombo();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("The pool (View > Pool)");
}

// Map editing's lighting (its own; Operate's look is unchanged). Sterile by default: even light, no shadows.
void App::drawMapLighting() {
    auto &s = mapObserver_;
    sectionTitle("Lighting");
    ImGui::SetNextItemWidth(ui(160));
    ImGui::Combo("Lighting", &s.lighting, "Scene lighting\0Indoor\0Outdoor\0Sterile\0");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Sterile: even light with no shadows, caustics or glare (the default here)");
    ImGui::BeginDisabled(s.lighting == 3);
    ImGui::Checkbox("Shadows", &s.shadows);
    ImGui::SetNextItemWidth(ui(160));
    ImGui::SliderFloat("Brightness", &s.brightness, 0.f, 4.f, "%.2fx");
    ImGui::EndDisabled();
    ImGui::SetNextItemWidth(ui(160));
    ImGui::SliderFloat("Exposure", &s.exposure, .4f, 2.f, "%.2fx");
    ImGui::SetNextItemWidth(ui(160));
    ImGui::SliderFloat("Ambient", &s.ambient, 0.f, 3.f, "%.2fx");
    if (ImGui::Button("Reset lighting")) {
        s.resetLighting();
        s.lighting = 3;
        s.shadows = false;
    }
}

// Quitting with unsaved prior map edits asks first.
void App::drawUnsavedMapPrompt() {
    if (closePrompt_) {
        closePrompt_ = false;
        ImGui::OpenPopup("Unsaved prior map");
    }
    if (!ImGui::BeginPopupModal("Unsaved prior map", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        return;
    ImGui::Text("The prior map has unsaved changes.");
    if (ImGui::Button("Save and quit")) {
        priorMap_->save();
        if (!priorMap_->dirty()) {
            closeConfirmed_ = true;
            window_->requestClose();
        }
        ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Quit without saving")) {
        closeConfirmed_ = true;
        window_->requestClose();
        ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel"))
        ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
}

// ------------------------------------------------------------------------------------------- the 2D view

namespace {
// The pool's axes in world x / y: screen right is the pool's +X, screen up its +Y.
std::pair<glm::vec2, glm::vec2> planAxes(const Scenario &s) {
    glm::vec2 x(s.poolToWorld[0]), y(s.poolToWorld[1]);
    x = glm::length(x) > 1e-6f ? glm::normalize(x) : glm::vec2(1, 0);
    y = glm::length(y) > 1e-6f ? glm::normalize(y) : glm::vec2(0, 1);
    return {x, y};
}
} // namespace

// Fits the whole pool into the 2D view, with an 8 % margin.
void App::fitPlan() {
    if (!scenario_)
        return;
    const auto &s = *scenario_;
    const auto [x, y] = planAxes(s);
    glm::vec2 low(1e9f), high(-1e9f); // along the screen axes
    for (const glm::vec2 corner : {glm::vec2(0, 0), glm::vec2(s.poolLength, 0), glm::vec2(0, s.poolWidth),
                                   glm::vec2(s.poolLength, s.poolWidth)}) {
        const glm::vec2 world(s.poolToWorld * glm::vec4(corner, 0, 1));
        const glm::vec2 along(glm::dot(world, x), glm::dot(world, y));
        low = glm::min(low, along);
        high = glm::max(high, along);
    }

    const glm::vec2 middle = (low + high) * .5f;
    planCenter_ = x * middle.x + y * middle.y;
    const float aspect = viewSize_.x / std::max(1.f, viewSize_.y);
    planHeight_ = std::max(high.y - low.y, (high.x - low.x) / aspect) * 1.08f;
    planFitted_ = true;
}

// Drag on the floor (left, once the prop editor has passed on it), right or middle drag: pan. Scroll: zoom
// about the pointer.
void App::handlePlanInput(bool hovered) {
    auto &io = ImGui::GetIO();
    const auto [x, y] = planAxes(*scenario_);
    const float metersPerPixel = planHeight_ / std::max(1.f, viewSize_.y);
    if (planDragButton_ >= 0 && !ImGui::IsMouseDown(planDragButton_))
        planDragButton_ = -1;
    if (hovered && planDragButton_ < 0)
        for (int button : {ImGuiMouseButton_Left, ImGuiMouseButton_Right, ImGuiMouseButton_Middle})
            if (ImGui::IsMouseClicked(button))
                planDragButton_ = button;
    if (planDragButton_ >= 0 && (io.MouseDelta.x != 0 || io.MouseDelta.y != 0)) {
        planCenter_ += (-x * io.MouseDelta.x + y * io.MouseDelta.y) * metersPerPixel;
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
    }
    if (hovered && io.MouseWheel != 0) {
        const glm::vec2 offset(io.MousePos.x - (viewPos_.x + viewSize_.x / 2),
                               io.MousePos.y - (viewPos_.y + viewSize_.y / 2));
        const glm::vec2 under = planCenter_ + (x * offset.x - y * offset.y) * metersPerPixel;
        planHeight_ = std::clamp(planHeight_ * std::exp(-io.MouseWheel * .15f), 1.f, 300.f);
        const float next = planHeight_ / std::max(1.f, viewSize_.y);
        planCenter_ = under - (x * offset.x - y * offset.y) * next; // the point under the pointer stays put
    }
}

// The 2D view's camera: orthographic, straight down from above the deck, the pool's +Y up on screen.
SensorView App::planCamera(float aspect) const {
    const auto [x, y] = planAxes(*scenario_);
    const float above = scenario_->waterLevel + scenario_->deckHeight + 6; // over the deck and equipment
    const glm::vec3 eye(planCenter_, above);
    const float h = planHeight_ * .5f, w = h * aspect;
    return {eye, glm::lookAt(eye, eye - glm::vec3(0, 0, 1), glm::vec3(y, 0)),
            glm::ortho(-w, w, -h, h, .05f, 6 + scenario_->deckHeight + scenario_->poolDepth + 20)};
}
} // namespace nereus::ros_viewer::host

namespace nereus::ros_viewer::host {
// The 2D chart's frame: the theme canvas around the pool (hiding the deck and the sky), the pool's rim, a scale bar,
// and the robot's name at its marker.
void App::drawPlanFrame(const glm::mat4 &vp, glm::vec2 origin, glm::vec2 size) {
    const auto &s = *scenario_;
    const auto pal = planPalette();
    auto *d = ImGui::GetWindowDrawList();
    const auto project = [&](glm::vec3 w, ImVec2 &out) {
        const auto clip = vp * glm::vec4(w, 1);
        if (clip.w <= 1e-6f)
            return false;
        out = {origin.x + (clip.x / clip.w * .5f + .5f) * size.x, origin.y + (.5f - clip.y / clip.w * .5f) * size.y};
        return true;
    };

    // The pool's outline on screen, from its floor corners
    ImVec2 lo(1e9f, 1e9f), hi(-1e9f, -1e9f);
    for (const glm::vec2 corner : {glm::vec2(0, 0), glm::vec2(s.poolLength, 0), glm::vec2(0, s.poolWidth),
                                   glm::vec2(s.poolLength, s.poolWidth)}) {
        ImVec2 at;
        if (!project(glm::vec3(s.poolToWorld * glm::vec4(corner, -s.poolDepth, 1)), at))
            return;
        lo = {std::min(lo.x, at.x), std::min(lo.y, at.y)};
        hi = {std::max(hi.x, at.x), std::max(hi.y, at.y)};
    }

    // The canvas over everything outside the pool, then the rim
    const ImVec2 a(origin.x, origin.y), b(origin.x + size.x, origin.y + size.y);
    d->PushClipRect(a, b, true);
    const ImU32 canvas = ImGui::GetColorU32(pal.canvas);
    d->AddRectFilled(a, {b.x, lo.y}, canvas);
    d->AddRectFilled({a.x, hi.y}, b, canvas);
    d->AddRectFilled({a.x, lo.y}, {lo.x, hi.y}, canvas);
    d->AddRectFilled({hi.x, lo.y}, {b.x, hi.y}, canvas);
    const float rim = ui(5);
    d->AddRect({lo.x - rim * .5f, lo.y - rim * .5f}, {hi.x + rim * .5f, hi.y + rim * .5f}, ImGui::GetColorU32(pal.rim),
               ui(2), 0, rim);

    // the robot, named (map editing draws it frozen and level)
    if (!s.robotId.empty()) {
        ImVec2 at;
        const glm::mat4 body = priorMap_->robotStart().value_or(body_);
        if (project(glm::vec3(body[3]), at)) {
            const ImVec2 text = window_->small->CalcTextSizeA(window_->small->FontSize, 1e9f, 0, s.robotId.c_str());
            const ImVec2 box(at.x - (text.x + ui(10)) * .5f, at.y + ui(16));
            d->AddRectFilled(box, {box.x + text.x + ui(10), box.y + text.y + ui(6)},
                             ImGui::GetColorU32(ImVec4(pal.canvas.x, pal.canvas.y, pal.canvas.z, .85f)), ui(3));
            d->AddText(window_->small, window_->small->FontSize, {box.x + ui(5), box.y + ui(3)},
                       ImGui::GetColorU32(palette().muted), s.robotId.c_str());
        }
    }

    // a scale bar in the top right corner (clear of the scene chip and the hints): the longest round length under 160
    // px
    const float metersPerPixel = planHeight_ / std::max(1.f, size.y);
    float meters = .5f;
    for (const float step : {1.f, 2.f, 5.f, 10.f, 20.f, 50.f})
        if (step / metersPerPixel <= ui(160))
            meters = step;
    const float length = meters / metersPerPixel;
    char label[16];
    std::snprintf(label, sizeof(label), meters < 1 ? "%.1f m" : "%.0f m", meters);
    const ImVec2 text = window_->small->CalcTextSizeA(window_->small->FontSize, 1e9f, 0, label);
    const ImVec2 base(b.x - ui(18) - length, a.y + ui(18) + text.y + ui(12));
    const ImU32 ink = ImGui::GetColorU32(palette().text);
    d->AddRectFilled({base.x - ui(8), base.y - text.y - ui(12)}, {base.x + length + ui(8), base.y + ui(8)},
                     ImGui::GetColorU32(ImVec4(pal.canvas.x, pal.canvas.y, pal.canvas.z, .85f)), ui(3));
    d->AddLine(base, {base.x + length, base.y}, ink, ui(2));
    for (const float x : {0.f, length})
        d->AddLine({base.x + x, base.y - ui(5)}, {base.x + x, base.y + ui(3)}, ink, ui(2));
    d->AddText(window_->small, window_->small->FontSize, {base.x + (length - text.x) * .5f, base.y - text.y - ui(6)},
               ink, label);
    d->PopClipRect();
}
} // namespace nereus::ros_viewer::host

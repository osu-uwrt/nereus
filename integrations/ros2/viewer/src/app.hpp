// nereus-viewer: ROS 2 client GUI for the nereus simulator bridge.
#pragma once
#include <optional>
#include <string>
#include <vector>

namespace robotics::ros_viewer::host {
struct Options {
    std::string scenarioFile;  // resolved.json instead of the latched scenario topic
    std::string packDir;       // pack folder for documents without asset_paths
    std::string configPath;    // host viewer document
    std::string panelsPath;    // operator panel composition ("none" disables panels)
    std::string shaders;       // renderer shader directory
    std::string screenshot;    // PNG written on the last frame
    std::string initialFocus;  // overrides the config
    std::string initialView;   // orbit | free | <camera id>
    std::string demoTask;      // preview target landmark
    std::string scenarioTopic; // overrides the config
    bool demo = false;         // no ROS graph: fixed preview pose, no providers
    bool showFocus = false;    // keep the orbit focus marker visible (screenshots/tests)
    bool localCameras = true;  // camera cards render from the viewer's scene at the truth pose
    double cardRate = -1;      // local card refresh Hz; <0 = host yaml cards.rate_hz, else the camera's rate
    bool hidden = false;
    std::optional<bool> detections; // default: host yaml detections.enabled (on)
    bool keepDetections = false;    // ignore marker DELETEALL (host yaml detections.honor_delete_all: false)
    std::string detectionPlacement; // pose_source | truth | estimate | both (empty: host yaml)
    bool showTf = false, mpcPath = false, showScorecard = false;
    std::vector<std::string> open; // initial windows/popups: scene-settings, map, tf, pool-viewer
    std::vector<float> injectF;    // test aid: hover this window position and press F mid-run
    std::vector<float> orbit;      // optional initial orbit: yaw pitch distance (radians, metres)
    int frames = 0;                // render N frames, save the screenshot, exit
    double renderRate = 0;         // frame cap in Hz; 0 = 60 (vsync: uncapped, vsync paces; hidden runs use 30)
    // Swap interval 1 (ignored for --hidden). Off by default: under Wayland/XWayland the compositor throttles
    // swaps of a covered or unfocused window (~1 Hz), which stalls the whole UI loop.
    bool vsync = false;
    bool profile = false;           // log frame-time statistics every few seconds
    bool legacyCards = false;       // A/B aid: both cards every 0.1 s through the full render pipeline
    bool profileSync = false;       // also glFinish after main/card draws so phases include GPU time
    double truthDelay = -1;         // seconds behind the latest truth stamp (<0: host yaml, default 0.02)
    double otherDelay = -1;         // same for estimate / other TF frames (default 0.06)
    std::string poseSource;         // auto | truth | estimate (empty: host yaml, default auto)
    bool robotOnly = false;         // draw only the robot (no pool, water or course)
    std::optional<bool> useSimTime; // default: on unless --demo
    std::vector<std::string> rosArgs;
};
int run(const Options &, int argc, char **argv);
} // namespace robotics::ros_viewer::host

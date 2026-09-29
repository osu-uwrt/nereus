// robotics-pool-viewer: ROS 2 client GUI for the robotics-platform simulator bridge.
#pragma once
#include <optional>
#include <string>
#include <vector>

namespace robotics::ros_viewer::host {
struct Options {
    std::string scenarioFile;       // resolved.json instead of the latched scenario topic
    std::string packDir;            // pack folder for documents without asset_paths
    std::string configPath;         // host viewer document
    std::string panelsPath;         // operator panel composition ("none" disables panels)
    std::string shaders;            // renderer shader directory
    std::string screenshot;         // PNG written on the last frame
    std::string initialFocus;       // overrides the config
    std::string initialView;        // orbit | free | <camera id>
    std::string demoTask;           // preview target landmark
    std::string scenarioTopic;      // overrides the config
    bool demo = false;              // no ROS graph: fixed preview pose, no providers
    bool localCameras = true;       // camera cards render from the viewer's scene at the truth pose
    bool hidden = false;
    bool showTf = false, detections = false, mpcPath = false, showScorecard = false;
    std::vector<std::string> open;  // initial windows/popups: scene-settings, map, tf, pool-viewer
    std::vector<float> injectF;     // test aid: hover this window position and press F mid-run
    std::vector<float> orbit;       // optional initial orbit: yaw pitch distance (radians, metres)
    int frames = 0;                 // render N frames, save the screenshot, exit
    double renderRate = 30;
    std::optional<bool> useSimTime; // default: on unless --demo
    std::vector<std::string> rosArgs;
};
int run(const Options &, int argc, char **argv);
} // namespace robotics::ros_viewer::host

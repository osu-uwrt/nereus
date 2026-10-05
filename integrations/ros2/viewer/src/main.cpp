#include "app.hpp"
#include <cstdlib>
#include <cstring>
#include <iostream>

namespace {
void usage() {
    std::cout
        << "nereus-viewer: ROS 2 pool viewer for the nereus simulator bridge\n"
           "  --scenario FILE        load the scene from a resolved.json (default: latched scenario topic)\n"
           "  --pack-dir DIR         scenario pack folder used to resolve assets of a --scenario file\n"
           "  --config FILE          host viewer document (default content/viewer/talos_uwrt_host.yaml)\n"
           "  --panels FILE|none     operator panel composition (default content/viewer/talos_uwrt_panels.yaml)\n"
           "  --scenario-topic NAME  latched scenario topic (default from the config)\n"
           "  --local-cameras [true|false]  camera cards render from this viewer's scene at the truth pose\n"
           "                         (default true); each card can switch to the bridge's images (ROS)\n"
           "  --demo                 scene preview without ROS: fixed vehicle pose, landmark picker\n"
           "  --show-focus           keep the orbit focus marker visible (screenshots/tests)\n"
           "  --demo-task NAME       preview target landmark (demo only)\n"
           "  --focus NAME           initial focus (Course, Vehicle, a landmark, ...)\n"
           "  --view NAME            initial view: orbit, free, or a camera id\n"
           "  --inject-f X Y         test aid: hover window position (X,Y) and press F halfway through a capture run\n"
           "  --orbit YAW PITCH DIST initial orbit angles (radians) and distance (metres) after the focus\n"
           "  --open NAME            open scene-settings, map, tf, display (pool-viewer), help or depth (camera cards)\n"
           "                         at start (repeatable)\n"
           "  --workspace NAME       start in operate (default) or map (the prior map editor; Ctrl+M switches)\n"
           "  --prior-map FILE       the riptide_mapping config.yaml the prior map editor opens (default: host config\n"
           "                         prior_map.config)\n"
           "  --layout NAME          start layout: standard, wide, cameras, a saved layout's name or an .ini file\n"
           "                         (default: the last session's layout, else the host config's `layout`)\n"
           "  --theme NAME           heat-sheet (default), timing-board, abyss, midnight, daylight, contrast, ocean,\n"
           "                         arctic, ember, sonar, paper, classic (default: the last one chosen)\n"
           "  --ui-scale X|auto      interface scale, 1 = 100 % (default: the last one chosen); auto follows the "
           "desktop\n"
           "  --system-title-bar | --custom-title-bar   window decorations: the desktop's, or the viewer's own\n"
           "                         title bar with the menus in it (default; host yaml window.title_bar)\n"
           "  --show-tf --mpc-path --thrust --show-scorecard   initial toggle states\n"
           "  --detections | --no-detections   detection overlay on/off at start (default on)\n"
           "  --keep-detections      ignore detector DELETEALL; observations live out their lifetime\n"
           "  --detections-placement MODE   pose_source (default) | truth | estimate | both; truth/both need the "
           "simulator\n"
           "  --hidden --frames N --screenshot out.png   render N frames, save a PNG, exit\n"
           "  --render-rate HZ       frame cap (default 60; with --vsync: none, vsync paces)\n"
           "  --card-rate HZ         local camera card refresh (default: each camera's own rate; 0 = camera rate)\n"
           "  --vsync | --no-vsync   swap interval 1 / 0 (default 0: a covered or unfocused window's vsync\n"
           "                         swaps are throttled by the compositor and stall the UI)\n"
           "  --profile              log frame time mean/p50/p95/p99/max and per-phase CPU cost every 5 s\n"
           "                         (F3 toggles the on-screen readout)\n"
           "  --legacy-cards         A/B aid: render both local cards together every 0.1 s with the full pipeline\n"
           "  --profile-sync         --profile plus glFinish after the main/card draws so phases include GPU time\n"
           "                         (serializes CPU and GPU: use for attribution, not for the frame-time numbers)\n"
           "  --pose-source auto|truth|estimate  robot pose: simulator truth when fresh (auto), always truth, or\n"
           "                         the localization estimate (real robot: no simulator, no bridge)\n"
           "  --display-delay S      seconds behind the latest truth stamp shown (default 0.02)\n"
           "  --estimate-delay S     same for estimate / other TF frames (default 0.06)\n"
           "  --robot-only           draw only the robot: no pool, water or course (real-robot use)\n"
           "  --shaders DIR          renderer shader folder\n"
           "  --use-sim-time [true|false]   follow /clock (default true unless --demo)\n"
           "  --ros-args ...         passed to rclcpp\n";
}
} // namespace

int main(int argc, char **argv) {
    nereus::ros_viewer::host::Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto value = [&]() -> std::string {
            if (i + 1 >= argc) {
                std::cerr << arg << " needs a value\n";
                std::exit(2);
            }
            return argv[++i];
        };
        if (arg == "--help" || arg == "-h") {
            usage();
            return 0;
        } else if (arg == "--ros-args") {
            for (; i < argc; ++i)
                options.rosArgs.push_back(argv[i]);
        } else if (arg == "--scenario")
            options.scenarioFile = value();
        else if (arg == "--pack-dir")
            options.packDir = value();
        else if (arg == "--config")
            options.configPath = value();
        else if (arg == "--panels")
            options.panelsPath = value();
        else if (arg == "--scenario-topic")
            options.scenarioTopic = value();
        else if (arg == "--shaders")
            options.shaders = value();
        else if (arg == "--screenshot")
            options.screenshot = value();
        else if (arg == "--focus")
            options.initialFocus = value();
        else if (arg == "--view")
            options.initialView = value();
        else if (arg == "--inject-f")
            options.injectF = {std::stof(value()), std::stof(value())};
        else if (arg == "--open")
            options.open.push_back(value());
        else if (arg == "--layout")
            options.layout = value();
        else if (arg == "--prior-map")
            options.priorMap = value();
        else if (arg == "--workspace") {
            options.workspace = value();
            if (options.workspace != "operate" && options.workspace != "map")
                throw std::runtime_error("--workspace takes operate or map");
        }
        else if (arg == "--theme")
            options.theme = value();
        else if (arg == "--ui-scale")
            options.uiScale = value();
        else if (arg == "--system-title-bar")
            options.systemTitleBar = true;
        else if (arg == "--custom-title-bar")
            options.systemTitleBar = false;
        else if (arg == "--orbit") {
            options.orbit = {std::stof(value()), std::stof(value()), std::stof(value())};
        } else if (arg == "--demo-task")
            options.demoTask = value();
        else if (arg == "--demo")
            options.demo = true;
        else if (arg == "--show-focus")
            options.showFocus = true;
        else if (arg == "--no-local-cameras")
            options.localCameras = false;
        else if (arg == "--local-cameras") {
            std::string v = "true";
            if (i + 1 < argc && argv[i + 1][0] != '-')
                v = argv[++i];
            options.localCameras = v != "false" && v != "0";
        } else if (arg == "--no-vsync")
            options.vsync = false;
        else if (arg == "--vsync")
            options.vsync = true;
        else if (arg == "--legacy-cards")
            options.legacyCards = true;
        else if (arg == "--profile")
            options.profile = true;
        else if (arg == "--profile-sync") // phase costs include GPU time; drains the GL queue every frame
            options.profile = options.profileSync = true;
        else if (arg == "--robot-only")
            options.robotOnly = true;
        else if (arg == "--pose-source")
            options.poseSource = value();
        else if (arg == "--display-delay")
            options.truthDelay = std::atof(value().c_str());
        else if (arg == "--estimate-delay")
            options.otherDelay = std::atof(value().c_str());
        else if (arg == "--hidden")
            options.hidden = true;
        else if (arg == "--show-tf")
            options.showTf = true;
        else if (arg == "--detections")
            options.detections = true;
        else if (arg == "--no-detections")
            options.detections = false;
        else if (arg == "--keep-detections")
            options.keepDetections = true;
        else if (arg == "--detections-placement")
            options.detectionPlacement = value();
        else if (arg == "--mpc-path")
            options.mpcPath = true;
        else if (arg == "--thrust")
            options.thrust = true;
        else if (arg == "--show-scorecard")
            options.showScorecard = true;
        else if (arg == "--frames")
            options.frames = std::atoi(value().c_str());
        else if (arg == "--render-rate")
            options.renderRate = std::atof(value().c_str());
        else if (arg == "--card-rate")
            options.cardRate = std::atof(value().c_str());
        else if (arg == "--use-sim-time") {
            std::string v = "true";
            if (i + 1 < argc && argv[i + 1][0] != '-')
                v = argv[++i];
            options.useSimTime = v != "false" && v != "0";
        } else {
            std::cerr << "unknown argument " << arg << "\n";
            usage();
            return 2;
        }
    }
    try {
        return nereus::ros_viewer::host::run(options, argc, argv);
    } catch (const std::exception &error) {
        std::cerr << "nereus-viewer: " << error.what() << '\n';
        return 1;
    }
}

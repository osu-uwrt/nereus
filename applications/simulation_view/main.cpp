#include "interface.hpp"
#ifdef RP_VIEWER_SCENES
#include "scene_view.hpp"
#endif
#include "running_scenario.hpp"
#include <charconv>
#include <iostream>

namespace {
std::int64_t frameCount(const std::string &text) {
    std::int64_t value = 0;
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
    if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() || value <= 0)
        throw std::invalid_argument("--frames requires a positive integer");
    return value;
}
} // namespace
int main(int argc, char **argv) {
    try {
        std::filesystem::path scenario_path, screenshot;
        std::filesystem::path shaders, scene;
        bool hidden = false;
        std::int64_t frames = 0;
        for (int i = 1; i < argc; ++i) {
            const std::string argument(argv[i]);
            if (argument == "--help") {
                std::cout << "robotics-sim-view SCENARIO.yaml [--hidden --frames N] "
                             "[--screenshot OUTPUT.ppm]\n";
                std::cout << "Optional scene build: --scene SCENE.yaml --shaders DIRECTORY\n";
                return 0;
            }
            if ((argument == "--shaders" || argument == "--scene") && i + 1 < argc) {
#ifdef RP_VIEWER_SCENES
                if (argument == "--shaders")
                    shaders = argv[++i];
                else
                    scene = std::filesystem::absolute(argv[++i]);
#else
                throw std::invalid_argument("scene rendering unavailable in this build");
#endif
            } else if (argument == "--hidden")
                hidden = true;
            else if ((argument == "--frames" || argument == "--screenshot") && i + 1 < argc) {
                const std::string value(argv[++i]);
                if (argument == "--frames")
                    frames = frameCount(value);
                else
                    screenshot = value;
            } else if (argument.rfind("--", 0) != 0 && scenario_path.empty())
                scenario_path = std::filesystem::canonical(argument);
            else
                throw std::invalid_argument("unknown or incomplete argument: " + argument);
        }
        if (scenario_path.empty())
            throw std::invalid_argument("a scenario is required; see --help");
        if ((hidden || !screenshot.empty()) && frames == 0)
            throw std::invalid_argument("--hidden/--screenshot require --frames");
        const auto scenario = robotics::config::loadScenario(scenario_path);
        robotics::visualization::LivePoseOptions pose_options{"simulation", "simulation_clock"};
        pose_options.body_frame = scenario.body_frames.root();
        pose_options.fixed_frames = scenario.body_frames.edges();
        auto source =
            std::make_shared<robotics::visualization::LivePoseSource>(std::move(pose_options));
        auto workspace = robotics::viewer::emptyWorkspace();
        workspace.sources.push_back({"simulation", "simulation", scenario_path});
        workspace.selected_source = "simulation";
        workspace.camera.target = scenario.initial.position;
        robotics::visualization::DisplaySettings body;
        body.id = "body";
        body.type = "pose";
        body.source = "simulation";
        body.stream = "pose";
        workspace.displays.push_back(body);
        auto path = body;
        path.id = "trajectory";
        path.type = "trajectory";
        workspace.displays.push_back(path);
        auto sources = robotics::viewer::localSources();
        sources.emplace("simulation", [source, scenario_path](const auto &file, const auto &id) {
            if (id != "simulation" || std::filesystem::weakly_canonical(file) != scenario_path)
                throw std::invalid_argument("this application is bound to its startup scenario");
            return robotics::viewer::Connection{source, nullptr, 0,
                                                [source] { source->reconnect(); }};
        });
        workspace.scene = scene;
        robotics::viewer::Desktop desktop(hidden);
        robotics::viewer::SceneDraw scene_draw;
#ifdef RP_VIEWER_SCENES
        scene_draw = robotics::viewer::sceneDrawer(shaders);
#endif
        robotics::viewer::Interface interface(std::move(workspace), std::move(sources),
                                              robotics::visualization::standardDisplays(),
                                              std::move(scene_draw));
        robotics::runner::RunningScenario execution(scenario, source);
        std::int64_t rendered = 0;
        auto previous = std::chrono::steady_clock::now();
        while (!desktop.closing() && (frames == 0 || rendered < frames)) {
            const auto started = std::chrono::steady_clock::now();
            interface.advance(
                std::chrono::duration_cast<std::chrono::nanoseconds>(started - previous).count());
            previous = started;
            if (execution.finished())
                execution.join(); // Reports failure; completed state remains inspectable.
            desktop.begin();
            interface.draw();
            ++rendered;
            desktop.finish(rendered == frames ? screenshot : std::filesystem::path{});
            std::this_thread::sleep_until(started + std::chrono::milliseconds(16));
        }
        execution.stop();
        execution.join();
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "robotics-sim-view: " << error.what() << '\n';
        return 1;
    }
}

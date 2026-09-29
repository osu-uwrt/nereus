#include "interface.hpp"
#ifdef RP_VIEWER_SCENES
#include "scene_view.hpp"
#endif

#include <charconv>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {
std::int64_t integer(const std::string &text) {
    std::int64_t value = 0;
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
    if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() || value < 0)
        throw std::invalid_argument("expected nonnegative integer: " + text);
    return value;
}
} // namespace
int main(int argc, char **argv) {
    try {
        std::filesystem::path workspace, screenshot;
        std::filesystem::path shaders, scene;
        bool hidden = false;
        std::int64_t frames = 0;
        std::optional<robotics::visualization::Time> time;
        for (int i = 1; i < argc; ++i) {
            const std::string argument(argv[i]);
            if (argument == "--help") {
                std::cout << "robotics-viewer [WORKSPACE.yaml] [--hidden --frames N] [--time-ns N] "
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
            else if ((argument == "--frames" || argument == "--time-ns" ||
                      argument == "--screenshot") &&
                     i + 1 < argc) {
                const std::string value(argv[++i]);
                if (argument == "--frames")
                    frames = integer(value);
                else if (argument == "--time-ns")
                    time = integer(value);
                else
                    screenshot = value;
            } else if (argument.rfind("--", 0) != 0 && workspace.empty())
                workspace = argument;
            else
                throw std::invalid_argument("unknown or incomplete argument: " + argument);
        }
        if ((hidden || !screenshot.empty()) && frames == 0)
            throw std::invalid_argument("--hidden/--screenshot require a positive --frames count");
        robotics::viewer::Desktop desktop(hidden);
        robotics::viewer::SceneDraw scene_draw;
#ifdef RP_VIEWER_SCENES
        scene_draw = robotics::viewer::sceneDrawer(shaders);
#endif
        robotics::viewer::Interface interface(workspace, std::move(scene_draw), scene);
        if (time)
            interface.seek(*time);
        using Clock = std::chrono::steady_clock;
        auto previous = Clock::now();
        std::int64_t rendered = 0;
        while (!desktop.closing() && (frames == 0 || rendered < frames)) {
            const auto started = Clock::now();
            interface.advance(
                std::chrono::duration_cast<std::chrono::nanoseconds>(started - previous).count());
            previous = started;
            desktop.begin();
            interface.draw();
            ++rendered;
            desktop.finish(rendered == frames ? screenshot : std::filesystem::path{});
            std::this_thread::sleep_until(started + std::chrono::milliseconds(16));
        }
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "robotics-viewer: " << error.what() << '\n';
        return 1;
    }
}

// Bag recording: the recorder scripts against a stand-in ros2 (this computer) and a stand-in ssh (the "robot", its
// own home and state), then the ros.bagging provider and bagging panel through a composition.
#include "nereus/ros_viewer/panels/composition.hpp"
#include "nereus/ros_viewer/panels/ros_providers.hpp"
#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <imgui.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <thread>
#include <unistd.h>
using namespace nereus::ros_viewer::panels;
namespace fs = std::filesystem;
namespace {
void write(const fs::path &path, const std::string &text, bool executable = false) {
    std::ofstream(path) << text;
    if (executable)
        fs::permissions(path, fs::perms::owner_all, fs::perm_options::add);
}
std::string read(const fs::path &path) {
    std::ifstream in(path);
    std::stringstream text;
    text << in.rdbuf();
    return text.str();
}
bool waitFor(const std::function<bool()> &done, double seconds) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (!done()) {
        if (std::chrono::steady_clock::now() > end)
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return true;
}
// ros2 bag record stand-in: writes its arguments into the bag, appends data until SIGINT, then closes the bag after
// close_delay seconds by writing metadata.yaml. Files beside it change its behaviour: die (fail at once with the
// file's text), exit_after (exit on its own after that many ticks), close_delay.
constexpr const char *fakeRos2 = R"SH(#!/bin/sh
d=$(cd "$(dirname "$0")" && pwd)
[ "$1" = bag ] && [ "$2" = record ] || exit 2
shift 2
bag=
: > "$d/args.$$"
while [ $# -gt 0 ]; do
  if [ "$1" = -o ]; then bag=$2; shift 2; continue; fi
  printf '%s\n' "$1" >> "$d/args.$$"; shift
done
if [ -f "$d/die" ]; then echo "boom: $(cat "$d/die")"; rm -f "$d/args.$$"; exit 1; fi
mkdir -p "$bag" && mv "$d/args.$$" "$bag/args"
close=0; [ -f "$d/close_delay" ] && close=$(cat "$d/close_delay")
trap 'echo closing; sleep "$close"; echo "version: 5" > "$bag/metadata.yaml"; exit 0' INT
n=0
while :; do
  echo data >> "$bag/data.db3"
  sleep 0.05
  n=$((n + 1))
  if [ -f "$d/exit_after" ] && [ "$n" -ge "$(cat "$d/exit_after")" ]; then echo "write failed"; echo "disk full"; exit 3; fi
done
)SH";
} // namespace

int main(int argc, char **argv) {
    char pattern[] = "/tmp/nereus_bagging_XXXXXX";
    const fs::path root = mkdtemp(pattern);
    const fs::path bin = root / "bin", robotHome = root / "robot_home";
    fs::create_directories(bin);
    fs::create_directories(robotHome);
    write(bin / "ros2", fakeRos2, true);
    // The robot: ssh's options skipped, the destination logged, the command run by a login shell with the robot's
    // own home and state directory.
    write(root / "ssh",
          "#!/bin/sh\nwhile [ $# -gt 0 ]; do case \"$1\" in -o) shift 2 ;; -T) shift ;; *) break ;; esac; done\n"
          "echo \"$1\" >> " + (root / "ssh_hosts").string() + "\nshift\nexport HOME=" + robotHome.string() +
              " XDG_STATE_HOME=" + (root / "robot_state").string() + "\nexec sh -c \"$*\"\n",
          true);
    write(root / "ssh_down", "#!/bin/sh\necho 'ssh: connect to host down port 22: No route to host' >&2\nexit 255\n",
          true);
    setenv("XDG_STATE_HOME", (root / "host_state").c_str(), 1);
    const std::string setup = "export PATH=" + bin.string() + ":$PATH";

    // Pure helpers.
    assert(shellQuote("it's") == "'it'\\''s'");
    assert(validBagName("talos_20261005_101500") && validBagName("a.b-c"));
    assert(!validBagName("") && !validBagName("..") && !validBagName("a/b") && !validBagName("a b"));
    assert(bagName("talos", false) == "talos" && bagName("talos", true).size() == std::string("talos_").size() + 15);
    assert(byteSize(512) == "512 B" && byteSize(1.43e9) == "1.4 GB" && byteSize(212e9) == "212 GB");
    const auto report = parseBagReport("Sourcing zenoh\nNEREUS_BAG state recording\nNEREUS_BAG bag /a b\nnoise\n"
                                       "NEREUS_BAG bytes 12\nNEREUS_BAG empty\n");
    assert(report.get("state") == "recording" && report.get("bag") == "/a b" && report.number("bytes") == 12);
    assert(report.has("empty") && report.get("empty").empty() && report.number("missing", -1) == -1);
    BagHost robot{"ros@orin2", "~/bags", setup, {(root / "ssh").string()}, {"--max-cache-size", "100"}};
    const auto command = bagCommand(robot, "echo hi");
    assert(command.front() == (root / "ssh").string() && command[command.size() - 2] == "ros@orin2" &&
           command.back() == "bash -c 'echo hi'");
    assert(bagCommand(BagHost{}, "echo hi") == (std::vector<std::string>{"bash", "-c", "echo hi"}));
    const auto timed = runProcess({"sh", "-c", "echo started; sleep 5"}, .3);
    assert(timed.timedOut && timed.output == "started\n");
    // A child left holding the output (as an ssh connection master does) does not hold up the result.
    const auto leftover = runProcess({"sh", "-c", "sleep 3 & echo done"}, 2);
    assert(!leftover.timedOut && leftover.status == 0 && leftover.output == "done\n");
    assert(runProcess({"/nonexistent/program"}, 1).status != 0);

    // Scripts on this computer.
    const fs::path hostBags = root / "host_bags";
    BagHost host{"", hostBags.string(), setup, {"ssh"}, {}};
    auto run = [](const BagHost &h, const std::string &script) {
        const auto result = runProcess(bagCommand(h, script), 20);
        assert(!result.timedOut);
        return parseBagReport(result.output);
    };
    auto r = run(host, bagStatusScript(host));
    assert(r.get("state") == "idle" && r.number("free") > 0 && !r.has("last_bag"));
    BagRequest request{"first", "", false, {"/talos/odometry/filtered", "/tf"}, "image|point cloud"};
    r = run(host, bagStartScript(host, request));
    const auto firstBag = (hostBags / "first").string();
    assert(r.get("state") == "recording" && r.get("bag") == firstBag && r.number("pid") > 0);
    assert(read(hostBags / "first" / "args") == "-x\nimage|point cloud\n/talos/odometry/filtered\n/tf\n");
    assert(run(host, bagStartScript(host, {"second", "", true, {}, ""})).get("error").find("already recording") !=
           std::string::npos);
    r = run(host, bagStatusScript(host));
    assert(r.get("state") == "recording" && r.number("bytes") > 0 && r.number("now") >= r.number("start"));
    r = run(host, bagStopScript(host, 5));
    assert(r.get("state") == "stopped" && r.get("last_bag") == firstBag && r.has("last_complete"));
    assert(fs::exists(hostBags / "first" / "metadata.yaml"));
    r = run(host, bagStatusScript(host));
    assert(r.get("state") == "idle" && r.get("last_state") == "stopped" && r.number("last_bytes") > 0);
    assert(run(host, bagStartScript(host, request)).get("error").find("already exists") != std::string::npos);
    // ros2 failing at once: its output comes back.
    write(bin / "die", "unknown storage");
    r = run(host, bagStartScript(host, {"dies", "", true, {}, ""}));
    assert(r.get("state") == "error" && r.get("error").find("boom: unknown storage") != std::string::npos);
    fs::remove(bin / "die");
    // A recorder that exits on its own is reported as died, with its last words.
    write(bin / "exit_after", "4");
    assert(run(host, bagStartScript(host, {"short", "", true, {}, ""})).get("error").find("disk full") !=
           std::string::npos); // within the start check
    write(bin / "exit_after", "40");
    assert(run(host, bagStartScript(host, {"short2", "", true, {}, ""})).get("state") == "recording");
    assert(waitFor([&] { return run(host, bagStatusScript(host)).get("last_bag") == (hostBags / "short2").string(); },
                   8));
    fs::remove(bin / "exit_after");
    r = run(host, bagStatusScript(host));
    assert(r.get("state") == "idle" && r.get("last_state") == "died" &&
           r.get("last_note").find("disk full") != std::string::npos && !r.has("last_complete"));
    // Slow to close: stopping until it finishes; a second stop does not interrupt it.
    write(bin / "close_delay", "1.5");
    assert(run(host, bagStartScript(host, {"slow", "", true, {}, ""})).get("state") == "recording");
    assert(run(host, bagStopScript(host, .2)).get("state") == "stopping");
    assert(run(host, bagStatusScript(host)).get("state") == "stopping");
    assert(run(host, bagStopScript(host, 5)).get("state") == "stopped");
    assert(fs::exists(hostBags / "slow" / "metadata.yaml"));
    // Never closing: kill.
    write(bin / "close_delay", "100");
    assert(run(host, bagStartScript(host, {"stuck", "", true, {}, ""})).get("state") == "recording");
    assert(run(host, bagStopScript(host, .2)).get("state") == "stopping");
    r = run(host, bagStopScript(host, .2, true));
    assert(r.get("state") == "stopped" && r.get("last_note").find("killed") != std::string::npos &&
           !r.has("last_complete"));
    fs::remove(bin / "close_delay");

    // The robot over ssh: ~ is the robot's home, its state is its own.
    r = run(robot, bagStartScript(robot, {"remote", "", true, {}, ""}));
    assert(r.get("state") == "recording" && r.get("bag") == (robotHome / "bags" / "remote").string());
    assert(read(robotHome / "bags" / "remote" / "args") == "--max-cache-size\n100\n-a\n");
    assert(read(root / "ssh_hosts").find("ros@orin2") != std::string::npos);
    assert(run(host, bagStatusScript(host)).get("state") == "idle");
    assert(run(robot, bagStopScript(robot, 5)).get("state") == "stopped");
    BagHost down = robot;
    down.ssh = {(root / "ssh_down").string()};
    const auto unreachable = runProcess(bagCommand(down, bagStatusScript(down)), 5);
    assert(unreachable.status == 255 && unreachable.output.find("No route to host") != std::string::npos);

    // The provider and panel.
    setenv("ROS_DOMAIN_ID", "185", 1);
    setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp", 1);
    rclcpp::init(argc, argv);
    const auto ns = "bagging_" + std::to_string(getpid());
    Registry registry;
    registerPanels(registry);
    RosProviders ros;
    ros.registerFactories(registry);
    auto config = YAML::Load(R"(
providers:
  bags:
    type: ros.bagging
    options:
      stop_timeout: 3
      presets: {Nav: [odometry/filtered, /tf]}
      targets:
        - {id: host, label: This computer}
        - {id: robot, label: Robot, host: ros@orin2, directory: ~/bags}
        - {id: down, label: Down, host: ros@down}
panels:
  - {id: bags, type: bagging, provider: bags, options: {name: talos, timestamp: false, target: robot}}
header:
  - {id: bag_chips, type: bagging, provider: bags}
)");
    auto targets = config["providers"]["bags"]["options"]["targets"];
    targets[0]["directory"] = hostBags.string();
    for (int i = 0; i < 3; ++i)
        targets[i]["setup"] = setup;
    targets[1]["ssh"].push_back((root / "ssh").string());
    targets[2]["ssh"].push_back((root / "ssh_down").string());
    Context ctx{ns, "world", false, false};
    auto composition = std::make_unique<Composition>(config, ctx, registry);
    auto bagging = std::dynamic_pointer_cast<Bagging>(composition->providers().at("bags"));
    ros.start();
    auto target = [&](const std::string &id) {
        for (const auto &t : bagging->state().targets)
            if (t.id == id)
                return t;
        assert(false);
        return BagTargetState{};
    };
    assert(bagging->state().presets.size() == 1 &&
           bagging->state().presets[0].topics ==
               (std::vector<std::string>{"/" + ns + "/odometry/filtered", "/tf"}));
    assert(waitFor([&] { return target("host").known && target("robot").known && target("down").known; }, 10));
    assert(target("host").reachable && target("robot").reachable && !target("down").reachable);
    assert(target("down").message.find("Can't reach ros@down") != std::string::npos &&
           target("down").message.find("No route to host") != std::string::npos);
    assert(target("robot").lastBag == (robotHome / "bags" / "remote").string());
    // Another machine for the robot target: it starts over there (the stand-in ssh logs the destination);
    // this computer has no destination to change.
    bagging->setHost("host", "ros@elsewhere");
    assert(target("host").host.empty());
    bagging->setHost("robot", "");
    assert(target("robot").host == "ros@orin2");
    bagging->setHost("robot", "ros@orin3");
    assert(target("robot").host == "ros@orin3" && !target("robot").known && target("robot").lastBag.empty());
    assert(waitFor([&] { return target("robot").known; }, 10));
    assert(target("robot").reachable && read(root / "ssh_hosts").find("ros@orin3") != std::string::npos);
    bagging->setHost("robot", "ros@orin2");
    assert(waitFor([&] { return target("robot").known; }, 10));
    // Invalid requests are refused.
    bagging->start("robot", {"bad name", "", true, {}, ""});
    bagging->start("robot", {"none", "", false, {}, ""});
    bagging->kill("robot");
    assert(!target("robot").pending);
    BagRequest nav{"nav", "", false, bagging->state().presets[0].topics, ""};
    bagging->start("robot", nav);
    assert(target("robot").pending);
    bagging->start("host", {"local", "", true, {}, ""});
    assert(waitFor([&] { return target("robot").recording && target("host").recording; }, 10));
    assert(target("robot").bag == (robotHome / "bags" / "nav").string() && target("robot").message == "Recording");
    bagging->setHost("robot", "ros@orin3"); // not while it records
    assert(target("robot").host == "ros@orin2" && target("robot").recording);
    // Draw the panel (both target choices) and the header chips.
    ImGui::CreateContext();
    auto &io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.DisplaySize = {1000, 900};
    io.DeltaTime = 1.f / 30;
    unsigned char *pixels;
    int width, height;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
    auto draw = [&] {
        ImGui::NewFrame();
        composition->drawPanels();
        ImGui::Begin("test");
        composition->drawHeader(600);
        ImGui::End();
        ImGui::Render();
    };
    for (int frame = 0; frame < 3; ++frame)
        draw();
    // Elapsed time keeps counting between polls.
    const double before = target("robot").elapsed;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    assert(target("robot").elapsed >= before + .25);
    // A restarted viewer finds the robot's recording; this computer's ended with the old viewer.
    ros.stop();
    composition.reset();
    bagging.reset();
    assert(fs::exists(hostBags / "local" / "metadata.yaml"));
    assert(!fs::exists(robotHome / "bags" / "nav" / "metadata.yaml"));
    composition = std::make_unique<Composition>(config, ctx, registry);
    bagging = std::dynamic_pointer_cast<Bagging>(composition->providers().at("bags"));
    ros.start();
    assert(waitFor([&] { return target("robot").recording && target("host").known; }, 10));
    assert(!target("host").recording && target("host").lastBag == (hostBags / "local").string());
    for (int frame = 0; frame < 3; ++frame)
        draw();
    bagging->stop("robot");
    assert(waitFor([&] { return !target("robot").recording && !target("robot").pending; }, 10));
    const auto stopped = target("robot");
    assert(!stopped.failed && stopped.message.find("Saved " + (robotHome / "bags" / "nav").string()) == 0);
    assert(read(robotHome / "bags" / "nav" / "args") ==
           "/" + ns + "/odometry/filtered\n/tf\n");
    for (int frame = 0; frame < 2; ++frame)
        draw();
    ImGui::DestroyContext();
    ros.stop();
    composition.reset();
    bagging.reset();
    rclcpp::shutdown();
    fs::remove_all(root);
    std::cout << "Bagging checks passed\n";
}

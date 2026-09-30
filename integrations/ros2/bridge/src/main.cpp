// nereus-sim <resolved.json> --output <run dir> [--sensors a,b] [--duration s]
//                  [--no-cameras] [--always-cameras] [--validate-only]
// The simulator bridge: validates the bridge pack
// against the installed ROS types before the first step, then runs the session in real time and
// writes resolved.json, execution.json, and on exit tasks.json and summary.json.
#include "camera_sink.hpp"
#include "node.hpp"
#include "records.hpp"
#include "session_adapter.hpp"

#include <rules/registry.hpp>

#include <csignal>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <sstream>

namespace fs = std::filesystem;
using namespace nereus;
using namespace nereus::ros_bridge;

namespace {
struct Arguments {
    fs::path scenario, output;
    std::optional<std::string> sensors;
    std::optional<double> duration;
    bool no_cameras{false}, validate_only{false}, always_cameras{false};
};

std::vector<std::string> split(const std::string &text) {
    std::vector<std::string> out;
    std::stringstream stream(text);
    std::string item;
    while (std::getline(stream, item, ','))
        if (!item.empty())
            out.push_back(item);
    return out;
}

Arguments parse(int argc, char **argv) {
    Arguments arguments;
    for (int k = 1; k < argc; ++k) {
        const std::string arg = argv[k];
        const auto value = [&]() -> std::string {
            if (k + 1 >= argc)
                throw std::invalid_argument(arg + " needs a value");
            return argv[++k];
        };
        if (arg == "--output")
            arguments.output = value();
        else if (arg == "--sensors")
            arguments.sensors = value();
        else if (arg == "--duration")
            arguments.duration = std::stod(value());
        else if (arg == "--no-cameras")
            arguments.no_cameras = true;
        else if (arg == "--always-cameras")
            arguments.always_cameras = true;
        else if (arg == "--validate-only")
            arguments.validate_only = true;
        else if (!arg.empty() && arg[0] == '-')
            throw std::invalid_argument("unknown option " + arg);
        else
            arguments.scenario = arg;
    }
    if (arguments.scenario.empty() || arguments.output.empty())
        throw std::invalid_argument(
            "usage: nereus-sim <resolved.json> --output <run dir> "
            "[--sensors a,b] [--duration s] [--no-cameras] [--always-cameras] [--validate-only]");
    return arguments;
}

bool isCamera(const Json &sensor) {
    return sensor.at("type") == "stereo_camera";
}

// Drops every camera sensor stream and image stream from the bridge (no GPU run).
void withoutCameras(session::ResolvedScenario &resolved) {
    if (resolved.bridge.is_null())
        return;
    std::set<std::string> cameras;
    for (const auto &sensor : resolved.robot.at("sensors"))
        if (isCamera(sensor))
            cameras.insert(sensor.at("id").get<std::string>());
    Json streams = Json::array();
    for (const auto &stream : resolved.bridge.at("streams")) {
        const std::string native = stream.at("native");
        const bool camera = native.rfind("sensor:", 0) == 0 &&
                            cameras.count(native.substr(
                                7, native.find('.') == std::string::npos ? std::string::npos : native.find('.') - 7));
        if (!stream.contains("image") && !camera)
            streams.push_back(stream);
    }
    resolved.bridge["streams"] = streams;
    if (resolved.document.contains("bridge"))
        resolved.document["bridge"]["streams"] = streams;
}
} // namespace

int main(int argc, char **argv) {
    Arguments arguments;
    try {
        arguments = parse(argc, argv);
    } catch (const std::exception &error) {
        std::cerr << "nereus-sim: " << error.what() << "\n";
        return 2;
    }

    std::optional<session::ResolvedScenario> resolved;
    std::unique_ptr<SessionAdapter> adapter;
    std::unique_ptr<CameraSink> cameras;
    std::unique_ptr<BridgeCore> core;
    std::vector<std::string> sensors, deferred;
    std::optional<std::int64_t> duration_ns;
    std::int64_t epoch_ns = 0;
    try {
        resolved = session::loadResolvedScenario(arguments.scenario);
        if (arguments.no_cameras)
            withoutCameras(*resolved);
        std::map<std::string, Json> by_id;
        std::vector<std::string> enabled;
        for (const auto &sensor : resolved->robot.at("sensors")) {
            by_id[sensor.at("id")] = sensor;
            if (sensor.value("enabled", true))
                enabled.push_back(sensor.at("id"));
        }
        std::vector<std::string> selection = enabled;
        if (arguments.sensors) {
            selection = split(*arguments.sensors);
            if (arguments.no_cameras) {
                std::vector<std::string> kept;
                for (const auto &id : selection)
                    if (by_id.count(id) && !isCamera(by_id.at(id)))
                        kept.push_back(id);
                selection = kept;
            }
        } else if (arguments.no_cameras) {
            selection.clear();
            for (const auto &id : enabled)
                if (!isCamera(by_id.at(id)))
                    selection.push_back(id);
        }
        std::set<std::string> unique(selection.begin(), selection.end());
        if (unique.size() != selection.size())
            throw std::invalid_argument("selected sensors must have unique ids from the robot pack");
        std::vector<std::string> native_ids, camera_ids;
        for (const auto &id : selection) {
            if (!by_id.count(id))
                throw std::invalid_argument("selected sensors must have unique ids from the robot pack");
            if (!by_id.at(id).value("enabled", true))
                throw std::invalid_argument("selected sensor is disabled in the robot pack");
            (isCamera(by_id.at(id)) ? camera_ids : native_ids).push_back(id);
        }
        adapter = std::make_unique<SessionAdapter>(*resolved, rules::standardRules(), &native_ids);
        if (!camera_ids.empty()) {
            CameraSinkOptions camera_options;
            camera_options.always = arguments.always_cameras;
            cameras = createCameraSink(*resolved, camera_ids, *adapter, camera_options);
            if (!cameras)
                throw std::runtime_error("camera sensors need the camera runtime, which this build does not "
                                         "include; run with --no-cameras");
        }
        sensors = selection;
        for (const auto &id : adapter->deferredSensorIds())
            if (std::find(camera_ids.begin(), camera_ids.end(), id) == camera_ids.end())
                deferred.push_back(id);
        epoch_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())
                .count();
        if (arguments.duration)
            duration_ns = static_cast<std::int64_t>(std::llround(*arguments.duration * 1e9));
        if (fs::exists(arguments.output))
            throw std::runtime_error("output directory exists: " + arguments.output.string());
        fs::create_directories(arguments.output);
        writeJson(arguments.output / "resolved.json", resolved->document);
        // Validate every mapping against installed ROS types before any middleware exists.
        core = std::make_unique<BridgeCore>(*resolved, *adapter, epoch_ns, Lookup{}, cameras.get());
        writeJson(arguments.output / "execution.json",
                  executionRecord(*resolved, *core, sensors, deferred, duration_ns, cameras.get()));
    } catch (const std::exception &error) {
        std::cerr << "nereus-sim: " << error.what() << "\n";
        return 1;
    }
    if (arguments.validate_only) {
        std::cout << "validated " << arguments.scenario.string() << "; records in " << arguments.output.string()
                  << "\n";
        return 0;
    }

    rclcpp::init(argc, argv);
    std::string reason = "duration reached";
    int status = 0;
    std::unique_ptr<BridgeNode> node;
    try {
        node = std::make_unique<BridgeNode>(*core, cameras.get());
        node->startExecutor();
        const auto ticks = node->run(duration_ns);
        if (!(duration_ns && ticks * core->timestepNs() >= *duration_ns))
            reason = "interrupted";
    } catch (const std::exception &error) {
        reason = std::string("failed: ") + error.what();
        std::cerr << "nereus-sim: " << reason << "\n";
        status = 1;
    }
    std::signal(SIGINT, SIG_IGN); // a repeated Ctrl-C must not cut the records
    if (node)
        node->stopExecutor();
    try {
        if (cameras)
            cameras->close(); // finish workers before writing counters or destroying publishers
        writeJson(arguments.output / "tasks.json", tasksRecord(*core));
        Json summary = summaryRecord(*core, reason, cameras.get());
        if (node) {
            const auto &perf = node->performance();
            summary["performance"] = {
                {"ticks", perf.ticks},
                {"mean_tick_us", perf.meanTickUs()},
                {"max_tick_us", perf.tick_ns_max / 1e3},
                {"session_advance_us_per_tick",
                 core->timing().advance_ns / 1e3 / std::max<std::int64_t>(perf.ticks, 1)},
                {"sensor_mapping_us_per_tick", core->timing().sensors_ns / 1e3 / std::max<std::int64_t>(perf.ticks, 1)},
                {"streams_us_per_tick", core->timing().timed_ns / 1e3 / std::max<std::int64_t>(perf.ticks, 1)},
                {"wall_s", perf.wall_ns / 1e9},
                {"sim_s", perf.sim_ns / 1e9},
                {"real_time_factor", perf.realTimeFactor()},
                {"max_behind_ms", perf.max_behind_ns / 1e6},
                {"catchup_bursts", perf.catchup_bursts}};
            const auto phase = [&](const nereus::ros_bridge::BridgeNode::Phase &item) {
                return Json{{"mean_us", item.total_ns / 1e3 / std::max<std::int64_t>(perf.ticks, 1)},
                            {"max_ms", item.max_ns / 1e6},
                            {"over_1ms", item.over_1ms},
                            {"over_5ms", item.over_5ms}};
            };
            summary["skipped_without_subscribers"] = node->skippedPublications();
            summary["performance"]["phases"] = {{"drain", phase(perf.drain)},
                                                {"step", phase(perf.step)},
                                                {"publish", phase(perf.publish)},
                                                {"cameras", phase(perf.cameras)},
                                                {"oversleep", phase(perf.oversleep)}};
        }
        writeJson(arguments.output / "summary.json", summary);
    } catch (const std::exception &error) {
        std::cerr << "nereus-sim: " << error.what() << "\n";
        status = 1;
    }
    node.reset();
    if (rclcpp::ok())
        rclcpp::shutdown();
    return status;
}

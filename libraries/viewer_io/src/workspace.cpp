#include <robotics/viewer/workspace.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <set>
#include <stdexcept>

namespace robotics::viewer {
namespace v = visualization;
namespace {
void fields(const YAML::Node &node, std::initializer_list<std::string> allowed) {
    if (!node.IsMap())
        throw std::invalid_argument("expected mapping");
    std::set<std::string> seen;
    for (const auto &entry : node) {
        const auto key = entry.first.as<std::string>();
        if (!seen.insert(key).second ||
            std::find(allowed.begin(), allowed.end(), key) == allowed.end())
            throw std::invalid_argument("duplicate or unknown field: " + key);
    }
}
void sequence(const YAML::Node &node, std::size_t limit) {
    if (!node.IsSequence() || node.size() > limit)
        throw std::invalid_argument("expected sequence with at most " + std::to_string(limit) +
                                    " entries");
}
std::string name(const YAML::Node &node) {
    const auto text = node.as<std::string>();
    if (text.empty() || text.size() > 256)
        throw std::invalid_argument("name must contain 1..256 characters");
    return text;
}
Eigen::Vector3d vector(const YAML::Node &node) {
    sequence(node, 3);
    if (node.size() != 3)
        throw std::invalid_argument("expected three coordinates");
    Eigen::Vector3d result(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
    if (!result.allFinite() || result.cwiseAbs().maxCoeff() > 1e12)
        throw std::invalid_argument("coordinates must be finite and within 1e12 metres");
    return result;
}
v::Pose pose(const YAML::Node &node) {
    fields(node, {"position", "orientation_wxyz"});
    const auto q = node["orientation_wxyz"];
    sequence(q, 4);
    if (q.size() != 4)
        throw std::invalid_argument("orientation requires four wxyz values");
    v::Pose result{vector(node["position"]),
                   Eigen::Quaterniond(q[0].as<double>(), q[1].as<double>(), q[2].as<double>(),
                                      q[3].as<double>())};
    v::validate(result);
    return result;
}
YAML::Node document(const std::filesystem::path &path) {
    if (std::filesystem::file_size(path) > 16 * 1024 * 1024)
        throw std::invalid_argument("document exceeds 16 MiB");
    auto result = YAML::LoadFile(path.string());
    if (result["version"].as<int>() != 1)
        throw std::invalid_argument("unsupported document version");
    return result;
}
void validateCamera(const Camera &camera) {
    if (!camera.target.allFinite() || camera.target.cwiseAbs().maxCoeff() > 1e12 ||
        !std::isfinite(camera.yaw) || !std::isfinite(camera.pitch) || camera.pitch < -1.5 ||
        camera.pitch > 1.5 || !std::isfinite(camera.distance) || camera.distance < 0.1 ||
        camera.distance > 10000)
        throw std::invalid_argument("invalid camera target, angles, or distance");
}
YAML::Node vectorNode(const Eigen::Vector3d &vector) {
    YAML::Node result;
    for (int i = 0; i < 3; ++i)
        result.push_back(vector[i]);
    result.SetStyle(YAML::EmitterStyle::Flow);
    return result;
}
} // namespace
Workspace emptyWorkspace() {
    Workspace result;
    v::DisplaySettings grid;
    grid.id = "Grid";
    grid.type = "grid";
    grid.color = {45, 65, 80};
    result.displays.push_back(grid);
    return result;
}
Workspace loadWorkspace(const std::filesystem::path &path) {
    try {
        const auto node = document(path);
        fields(node,
               {"version", "sources", "selected_source", "fixed_frame", "displays", "camera"});
        Workspace result;
        result.fixed_frame = name(node["fixed_frame"]);
        result.selected_source = node["selected_source"].as<std::string>();
        sequence(node["sources"], 16);
        std::set<std::string> ids;
        for (const auto &source : node["sources"]) {
            fields(source, {"id", "type", "file"});
            SourceBinding binding{name(source["id"]), name(source["type"]),
                                  source["file"].as<std::string>()};
            if (binding.file.empty() || binding.file.string().size() > 4096)
                throw std::invalid_argument(
                    "source file requires a path of at most 4096 characters");
            if (!ids.insert(binding.id).second)
                throw std::invalid_argument("duplicate source: " + binding.id);
            binding.file =
                std::filesystem::absolute(path.parent_path() / binding.file).lexically_normal();
            result.sources.push_back(std::move(binding));
        }
        sequence(node["displays"], 64);
        ids.clear();
        for (const auto &display : node["displays"]) {
            fields(display, {"id", "type", "source", "stream", "frame", "enabled", "history_limit",
                             "max_age_ns", "scale", "color"});
            v::DisplaySettings settings;
            settings.id = name(display["id"]);
            settings.type = name(display["type"]);
            if (!ids.insert(settings.id).second)
                throw std::invalid_argument("duplicate display: " + settings.id);
            if (display["source"])
                settings.source = display["source"].as<std::string>();
            if (display["stream"])
                settings.stream = display["stream"].as<std::string>();
            if (display["frame"])
                settings.frame = display["frame"].as<std::string>();
            if (display["enabled"])
                settings.enabled = display["enabled"].as<bool>();
            if (display["history_limit"])
                settings.history_limit = display["history_limit"].as<std::size_t>();
            if (display["max_age_ns"])
                settings.max_age_ns = display["max_age_ns"].as<v::Time>();
            if (display["scale"])
                settings.scale = display["scale"].as<double>();
            if (display["color"]) {
                sequence(display["color"], 3);
                if (display["color"].size() != 3)
                    throw std::invalid_argument("color requires RGB");
                for (std::size_t i = 0; i < 3; ++i)
                    settings.color[i] = display["color"][i].as<int>();
            }
            v::validate(settings);
            result.displays.push_back(std::move(settings));
        }
        if (node["camera"]) {
            const auto camera = node["camera"];
            fields(camera, {"target", "yaw", "pitch", "distance"});
            result.camera = {vector(camera["target"]), camera["yaw"].as<double>(),
                             camera["pitch"].as<double>(), camera["distance"].as<double>()};
        }
        validateCamera(result.camera);
        return result;
    } catch (const std::exception &error) {
        throw std::invalid_argument(path.string() + ": " + error.what());
    }
}
std::string serializeWorkspace(const Workspace &workspace,
                               const std::filesystem::path &destination) {
    validateCamera(workspace.camera);
    YAML::Node result;
    result["version"] = 1;
    result["fixed_frame"] = workspace.fixed_frame;
    result["selected_source"] = workspace.selected_source;
    result["sources"] = YAML::Node(YAML::NodeType::Sequence);
    for (const auto &source : workspace.sources) {
        YAML::Node node;
        node["id"] = source.id;
        node["type"] = source.type;
        auto relative =
            source.file.lexically_relative(std::filesystem::absolute(destination).parent_path());
        node["file"] = (relative.empty() ? source.file : relative).string();
        result["sources"].push_back(node);
    }
    result["displays"] = YAML::Node(YAML::NodeType::Sequence);
    for (const auto &display : workspace.displays) {
        v::validate(display);
        YAML::Node node;
        node["id"] = display.id;
        node["type"] = display.type;
        node["source"] = display.source;
        node["stream"] = display.stream;
        node["frame"] = display.frame;
        node["enabled"] = display.enabled;
        node["history_limit"] = display.history_limit;
        node["max_age_ns"] = display.max_age_ns;
        node["scale"] = display.scale;
        for (int value : display.color)
            node["color"].push_back(value);
        result["displays"].push_back(node);
    }
    result["camera"]["target"] = vectorNode(workspace.camera.target);
    result["camera"]["yaw"] = workspace.camera.yaw;
    result["camera"]["pitch"] = workspace.camera.pitch;
    result["camera"]["distance"] = workspace.camera.distance;
    YAML::Emitter emitter;
    emitter << result;
    if (!emitter.good())
        throw std::runtime_error(emitter.GetLastError());
    return std::string(emitter.c_str()) + "\n";
}
v::Recording loadRecording(const std::filesystem::path &path) {
    try {
        const auto node = document(path);
        fields(node, {"version", "clock", "duration_ns", "root", "frames", "streams"});
        v::Recording result;
        result.data.clock = name(node["clock"]);
        result.duration_ns = node["duration_ns"].as<v::Time>();
        sequence(node["frames"], 128);
        std::vector<v::FrameEdge> edges;
        for (const auto &edge : node["frames"]) {
            fields(edge, {"parent", "child", "static", "samples"});
            v::FrameEdge parsed;
            parsed.parent = name(edge["parent"]);
            parsed.child = name(edge["child"]);
            parsed.is_static = edge["static"].as<bool>();
            sequence(edge["samples"], 10000);
            for (const auto &sample : edge["samples"]) {
                fields(sample, {"time_ns", "pose"});
                parsed.samples.push_back({sample["time_ns"].as<v::Time>(), pose(sample["pose"])});
            }
            edges.push_back(std::move(parsed));
        }
        result.data.frames =
            std::make_shared<const v::FrameGraph>(name(node["root"]), std::move(edges));
        sequence(node["streams"], 64);
        for (const auto &stream : node["streams"]) {
            fields(stream, {"id", "samples"});
            sequence(stream["samples"], 10000);
            v::PoseHistory samples;
            for (const auto &sample : stream["samples"]) {
                fields(sample, {"time_ns", "frame", "pose"});
                samples.push_back(
                    {sample["time_ns"].as<v::Time>(), name(sample["frame"]), pose(sample["pose"])});
            }
            if (!result.data.streams.emplace(name(stream["id"]), std::move(samples)).second)
                throw std::invalid_argument("duplicate stream id");
        }
        v::validate(result);
        return result;
    } catch (const std::exception &error) {
        throw std::invalid_argument(path.string() + ": " + error.what());
    }
}
} // namespace robotics::viewer

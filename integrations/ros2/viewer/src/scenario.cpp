#include "scenario.hpp"
#include <algorithm>
#include <cctype>
#include <fstream>
#include <regex>
#include <set>
#include <sstream>

namespace nereus::ros_viewer::host {
namespace {
namespace fs = std::filesystem;

fs::path locatePackDirectory(const YAML::Node &doc, const fs::path &hint) {
    const auto scenario = doc["scenario"];
    auto usable = [&](const fs::path &dir) {
        if (dir.empty() || !fs::is_directory(dir))
            return false;
        for (const char *role : {"robot", "tasks", "pool"})
            if (!fs::exists(dir / scenario[role].as<std::string>("")))
                return false;
        return true;
    };
    if (usable(hint))
        return hint;
#ifdef NEREUS_PACK_CONTENT
    const fs::path root = fs::path(NEREUS_PACK_CONTENT) / "scenarios";
    if (fs::is_directory(root))
        for (const auto &entry : fs::directory_iterator(root))
            if (entry.is_directory() &&
                fs::exists(entry.path() / doc["scenario_file"].as<std::string>("scenario.yaml")) &&
                usable(entry.path()))
                return entry.path();
#endif
    return {};
}

// pack role -> {asset id -> absolute path}
std::map<std::string, std::map<std::string, fs::path>> resolveAssets(const YAML::Node &doc, const fs::path &hint) {
    std::map<std::string, std::map<std::string, fs::path>> result;
    const auto given = doc["asset_paths"];
    fs::path packDir;
    for (const char *role : {"robot", "pool", "tasks"}) {
        for (const auto &asset : doc[role]["assets"]) {
            const auto id = asset["id"].as<std::string>();
            if (given && given[role] && given[role][id]) {
                result[role][id] = given[role][id].as<std::string>();
                continue;
            }
            if (packDir.empty())
                packDir = locatePackDirectory(doc, hint);
            if (packDir.empty())
                continue; // unresolved assets are skipped with a warning by the caller
            result[role][id] = fs::weakly_canonical(packDir / doc["scenario"][role].as<std::string>() /
                                                    asset["path"].as<std::string>());
        }
    }
    return result;
}

std::string upper(std::string s) {
    for (auto &c : s)
        c = char(std::toupper(static_cast<unsigned char>(c)));
    return s;
}
} // namespace

std::string Scenario::absolute(const std::string &relative) const {
    if (relative.empty() || relative[0] == '/')
        return relative;
    return (ns.empty() || ns == "/" ? std::string("") : ns) + "/" + relative;
}
const SensorCamera *Scenario::camera(const std::string &name) const {
    for (const auto &c : cameras)
        if (c.id == name)
            return &c;
    return nullptr;
}
const Mechanism *Scenario::mechanism(const std::string &name) const {
    const auto it = mechanisms.find(name);
    return it == mechanisms.end() ? nullptr : &it->second;
}
std::string Scenario::rosFrame(const std::string &packFrame) const {
    const auto it = frameNames.find(packFrame);
    if (it != frameNames.end())
        return it->second;
    std::string prefix = ns;
    while (!prefix.empty() && prefix.front() == '/')
        prefix.erase(prefix.begin());
    return prefix.empty() ? packFrame : prefix + "/" + packFrame;
}

Scenario parseScenario(const std::string &json, const YAML::Node &config, const fs::path &packDirHint) {
    Scenario s;
    s.document = YAML::Load(json);
    auto d = s.document;
    for (const char *key : {"scenario", "robot", "pool", "tasks", "task_definitions", "bridge"})
        if (!d[key])
            throw std::runtime_error(std::string("scenario document lacks '") + key + "'");
    s.bridge = d["bridge"];
    const auto scenario = d["scenario"], robot = d["robot"], pool = d["pool"];
    s.id = scenario["id"].as<std::string>("scenario");
    s.robotId = robot["id"].as<std::string>("robot");
    s.tasksId = d["tasks"]["id"].as<std::string>("tasks");
    s.ns = s.bridge["namespace"].as<std::string>("");
    s.baseId = robot["reference_frame"].as<std::string>("base_link");
    for (const auto &item : s.bridge["frame_names"])
        s.frameNames[item.first.as<std::string>()] = item.second.as<std::string>();
    s.mapFrame = s.frameNames.count("world") ? s.frameNames["world"] : scenario["world_frame"].as<std::string>("map");
    s.estimateBaseFrame = s.rosFrame(s.baseId);
    // Truth pose: the bridge tf.publish entry driven by the robot reference pose.
    for (const auto &entry : s.bridge["tf"]["publish"])
        if (entry["native"].as<std::string>("").find("reference_pose") != std::string::npos) {
            s.truthBaseFrame = entry["child"].as<std::string>();
            if (entry["parent"])
                s.mapFrame = entry["parent"].as<std::string>();
        }
    if (s.truthBaseFrame.empty())
        s.truthBaseFrame = "simulator/" + s.estimateBaseFrame;
    // Bridge thruster order (force array layout).
    for (const auto &id : s.bridge["thrusters"]["order"])
        s.thrusterOrder.push_back(id.as<std::string>());

    // Configured ui (fallback) overlaid by the task pack ui.
    s.ui = config["ui"] ? YAML::Clone(config["ui"]) : YAML::Node(YAML::NodeType::Map);
    if (d["tasks"]["ui"])
        for (const auto &item : d["tasks"]["ui"])
            s.ui[item.first.as<std::string>()] = YAML::Clone(item.second);
    s.runOptions = d["run_options"] ? YAML::Clone(d["run_options"]) : YAML::Node(YAML::NodeType::Map);

    // Frame tree of the robot pack.
    for (const auto &edge : robot["frames"]["transforms"])
        s.frames.add(edge["parent"].as<std::string>(), edge["child"].as<std::string>(), packPose(edge));
    const auto assets = resolveAssets(d, packDirHint);
    const auto assetPath = [&](const char *role, const std::string &id) -> fs::path {
        const auto r = assets.find(role);
        if (r == assets.end())
            return {};
        const auto a = r->second.find(id);
        return a == r->second.end() ? fs::path() : a->second;
    };
    if (assets.count("robot"))
        s.robotAssets = assets.at("robot");
    try {
        auto document = nlohmann::json::parse(json);
        if (!document.contains("format"))
            document["format"] = "nereus.resolved_scenario";
        nlohmann::json paths = nlohmann::json::object();
        for (const auto &[role, byId] : assets)
            for (const auto &[id, path] : byId)
                paths[role][id] = path.string();
        document["asset_paths"] = std::move(paths);
        s.resolved = std::make_shared<const session::ResolvedScenario>(session::parseResolvedScenario(document));
    } catch (const std::exception &error) {
        s.resolvedError = error.what();
    }

    // Pool geometry, placement and appearance.
    const auto pp = pool["parameters"];
    s.poolId = pool["id"].as<std::string>("pool");
    s.poolLength = pp["length_m"].as<float>();
    s.poolWidth = pp["width_m"].as<float>();
    s.poolDepth = pp["depth_m"].as<float>();
    s.deckHeight = pp["deck_height_m"].as<float>();
    const auto placement = scenario["pool_placement"];
    const glm::vec3 poolAt = vec3(placement["position_m"]);
    s.waterLevel = pp["water_level_m"].as<float>(0) + poolAt.z;
    s.poolToWorld = pose({poolAt.x, poolAt.y, 0}, {0, 0, glm::radians(placement["yaw_deg"].as<float>(0))});
    s.worldToPool = glm::inverse(s.poolToWorld);
    if (pool["water_optics"] && pool["lighting"]) {
        const auto o = pool["water_optics"], l = pool["lighting"];
        auto &w = s.appearance.water;
        if (o["tint_rgb"])
            w.tint = {o["tint_rgb"][0].as<float>(), o["tint_rgb"][1].as<float>(), o["tint_rgb"][2].as<float>()};
        w.absorption = {o["absorption_per_m_rgb"][0].as<float>(), o["absorption_per_m_rgb"][1].as<float>(),
                        o["absorption_per_m_rgb"][2].as<float>()};
        w.scattering = o["scattering"].as<float>();
        w.distance_scale = o["distance_scale"].as<float>();
        w.distance_power = o["distance_power"].as<float>();
        w.clear_distance = o["clear_distance_m"].as<float>();
        s.appearance.outdoor = l["profile"].as<std::string>() == "outdoor";
        s.appearance.direct_light = l["direct_light"].as<float>();
        s.appearance.ambient_light = l["ambient_light"].as<float>();
        s.appearance.sun_azimuth = l["sun_azimuth_deg"].as<float>();
        s.appearance.sun_elevation = l["sun_elevation_deg"].as<float>();
        s.appearance.glare = l["glare"].as<float>();
    }

    // Robot visuals in base_link.
    for (const auto &visual : robot["visuals"]) {
        RobotVisual v;
        v.asset = visual["asset"].as<std::string>();
        v.frame = visual["frame"].as<std::string>();
        v.local = packPose(visual);
        v.frameInBase = s.frames.relative(s.baseId, v.frame);
        v.inBase = v.frameInBase * v.local;
        v.path = assetPath("robot", v.asset);
        s.robotVisuals.push_back(std::move(v));
    }

    // Mechanisms.
    for (const auto &item : robot["mechanisms"]) {
        Mechanism m;
        m.id = item["id"].as<std::string>();
        m.type = item["type"].as<std::string>();
        m.frame = item["frame"].as<std::string>();
        m.frameInBase = s.frames.relative(s.baseId, m.frame);
        const auto p = item["parameters"];
        for (const auto &slot : p["slots"])
            m.slotsInBase.push_back(m.frameInBase * packPose(slot));
        if (p["projectile"]) {
            m.projectileLength = p["projectile"]["length_m"].as<float>(0);
            m.projectileRadius = p["projectile"]["radius_m"].as<float>(0);
        }
        m.minGap = p["min_gap_m"].as<float>(0);
        if (p["tip_position_m"])
            m.tip = vec3(p["tip_position_m"]);
        s.mechanisms[m.id] = std::move(m);
    }

    // Camera sensors and their bridge topics.
    const auto streamFor = [&](const std::string &sensor, const std::string &output) -> std::string {
        const std::string key = "sensor:" + sensor + "." + output;
        for (const auto &stream : s.bridge["streams"])
            if (stream["native"].as<std::string>("") == key && stream["direction"].as<std::string>("") == "publish")
                return s.absolute(stream["topic"].as<std::string>());
        return {};
    };
    int index = 0;
    for (const auto &sensor : robot["sensors"]) {
        if (sensor["type"].as<std::string>() != "stereo_camera" || !sensor["enabled"].as<bool>(true))
            continue;
        SensorCamera c;
        c.id = sensor["id"].as<std::string>();
        c.mountFrame = sensor["mount_frame"].as<std::string>(sensor["frame"].as<std::string>());
        c.opticalFrame = sensor["frame"].as<std::string>();
        c.rosOpticalFrame = s.rosFrame(c.opticalFrame);
        const auto p = sensor["parameters"], left = p["intrinsics_left"];
        c.k.width = p["resolution_px"][0].as<int>();
        c.k.height = p["resolution_px"][1].as<int>();
        c.k.fx = left["fx"].as<double>();
        c.k.fy = left["fy"].as<double>();
        c.k.cx = left["cx"].as<double>();
        c.k.cy = left["cy"].as<double>();
        c.k.validate();
        c.minRange = p["depth"]["min_range_m"].as<double>(c.minRange);
        c.maxRange = p["depth"]["max_range_m"].as<double>(c.maxRange);
        c.periodS = sensor["period_ns"].as<double>(66666667.) * 1e-9;
        c.mountInBase = s.frames.relative(s.baseId, c.mountFrame);
        c.opticalInBase = s.frames.relative(s.baseId, c.opticalFrame);
        c.rgbTopic = streamFor(c.id, "rgb_left");
        c.depthTopic = streamFor(c.id, "depth_left");
        c.infoTopic = streamFor(c.id, "camera_info");
        const auto display = lookup(config, {"cameras", c.id.c_str()});
        char number[16];
        std::snprintf(number, sizeof(number), "%02d", ++index);
        c.title = display["title"].as<std::string>(std::string(number) + "  " + upper(c.id));
        c.model = display["model"].as<std::string>("CAMERA");
        s.cameras.push_back(std::move(c));
    }

    // Task visuals and landmarks.
    std::map<std::string, YAML::Node> definitions;
    for (const auto &def : d["task_definitions"])
        definitions[def["id"].as<std::string>()] = static_cast<const YAML::Node &>(def);
    for (const auto &place : scenario["task_placements"]) {
        const auto task = place["task"].as<std::string>();
        const glm::vec3 at = vec3(place["position_m"]);
        const glm::mat4 world = pose(at, {0, 0, glm::radians(place["yaw_deg"].as<float>(0))});
        s.landmarks[task] = {world};
        const auto it = definitions.find(task);
        if (it == definitions.end())
            continue;
        const auto &def = it->second;
        std::map<std::string, glm::mat4> frames{{"task", glm::mat4(1)}};
        for (const auto &f : def["frames"]) {
            frames[f["id"].as<std::string>()] = packPose(f);
            s.landmarks[f["id"].as<std::string>()] = {world * frames[f["id"].as<std::string>()]};
        }
        std::map<std::string, YAML::Node> regions;
        for (const auto &r : def["regions"])
            regions[r["id"].as<std::string>()] = static_cast<const YAML::Node &>(r);
        for (const auto &prop : def["props"]) {
            if (prop["type"].as<std::string>() != "static_body")
                continue; // moving props are drawn from simulator/task_objects
            const auto params = prop["parameters"];
            const auto cutouts = params["cutouts"];
            for (const auto &visual : params["visuals"]) {
                TaskVisual v;
                v.task = task;
                v.prop = prop["id"].as<std::string>();
                v.asset = visual["asset"].as<std::string>();
                v.taskFromAsset = frames.at(visual["frame"].as<std::string>()) * packPose(visual);
                v.world = world * v.taskFromAsset;
                v.path = assetPath("tasks", v.asset);
                if (cutouts) {
                    const auto region = regions.at(cutouts["region"].as<std::string>())["parameters"];
                    if (region["plane"]["axis"].as<std::string>() != "x")
                        throw std::runtime_error("cutout faces are local x offsets; region plane axis must be x");
                    CutoutSpec spec;
                    for (const auto &face : cutouts["faces_local_x_m"])
                        spec.facesX.push_back(face.as<float>());
                    spec.halfSize = region["half_size_m"].as<float>();
                    for (const auto &hole : region["holes"])
                        spec.holes.push_back(
                            {{hole["uv"][0].as<float>(), hole["uv"][1].as<float>()}, hole["radius_uv"].as<float>()});
                    v.cutouts = std::move(spec);
                }
                s.taskVisuals.push_back(std::move(v));
            }
        }
    }
    return s;
}
} // namespace nereus::ros_viewer::host

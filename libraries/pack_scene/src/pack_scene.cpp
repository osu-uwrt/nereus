#include <array>
#include <robotics/pack_scene/pack_scene.hpp>

#include <robotics/rendering/assets.hpp>

#include <cmath>
#include <iostream>
#include <set>
#include <stdexcept>

namespace robotics::pack_scene {
namespace {
namespace r = robotics::rendering;
using session::Json;
constexpr double kPanelToleranceM = 5e-4; // declared panel faces are sub-millimetre data

Eigen::Vector3d vector3(const Json &value) {
    return {value.at(0).get<double>(), value.at(1).get<double>(), value.at(2).get<double>()};
}

std::vector<spatial::FixedFrame> edges(const Json &robot) {
    std::vector<spatial::FixedFrame> result;
    for (const auto &item : robot.at("frames").at("transforms"))
        result.push_back({item.at("parent").get<std::string>(), item.at("child").get<std::string>(),
                          placement(item)});
    return result;
}

rendering::Appearance appearanceFrom(const Json &pool, bool strict) {
    r::Appearance appearance;
    if (!pool.contains("water_optics") || !pool.contains("lighting")) {
        if (strict)
            throw std::runtime_error("pool '" + pool.value("id", std::string("?")) +
                                     "' lacks camera appearance: water_optics, lighting");
        return appearance;
    }
    const auto &optics = pool.at("water_optics"), &lighting = pool.at("lighting");
    auto &water = appearance.water;
    if (optics.contains("tint_rgb"))
        water.tint = vector3(optics.at("tint_rgb")).cast<float>();
    water.absorption = vector3(optics.at("absorption_per_m_rgb")).cast<float>();
    water.scattering = optics.at("scattering").get<float>();
    water.distance_scale = optics.at("distance_scale").get<float>();
    water.distance_power = optics.at("distance_power").get<float>();
    water.clear_distance = optics.at("clear_distance_m").get<float>();
    appearance.outdoor = lighting.at("profile").get<std::string>() == "outdoor";
    appearance.direct_light = lighting.at("direct_light").get<float>();
    appearance.ambient_light = lighting.at("ambient_light").get<float>();
    appearance.sun_azimuth = lighting.at("sun_azimuth_deg").get<float>();
    appearance.sun_elevation = lighting.at("sun_elevation_deg").get<float>();
    appearance.glare = lighting.at("glare").get<float>();
    return appearance;
}
} // namespace

Matrix4d toMatrix(const spatial::Pose &pose) {
    Matrix4d result = Matrix4d::Identity();
    result.topLeftCorner<3, 3>() = pose.rotation.toRotationMatrix();
    result.topRightCorner<3, 1>() = pose.translation;
    return result;
}

spatial::Pose placement(const Json &item) {
    spatial::Pose pose;
    pose.translation = vector3(item.at("position_m"));
    const auto &q = item.at("orientation_wxyz");
    pose.rotation = Eigen::Quaterniond(q.at(0).get<double>(), q.at(1).get<double>(), q.at(2).get<double>(),
                                       q.at(3).get<double>());
    spatial::validate(pose);
    return pose;
}

spatial::Pose upright(const Json &position_m, double yaw_deg) {
    const double half = yaw_deg * M_PI / 180 / 2;
    spatial::Pose pose;
    pose.translation = vector3(position_m);
    pose.rotation = Eigen::Quaterniond(std::cos(half), 0, 0, std::sin(half));
    return pose;
}

void PackScene::warn(const std::string &text) const {
    std::lock_guard<std::mutex> lock(mutex_);
    warnings_.push_back(text);
    std::cerr << "pack_scene: " << text << '\n';
}

PackScene::PackScene(const session::ResolvedScenario &resolved, Options options)
    : resolved_(resolved), options_(options),
      frames_(resolved.robot.at("frames").at("root").get<std::string>(), edges(resolved.robot)) {
    appearance_ = appearanceFrom(resolved_.pool, options_.strict);
    buildPool();
    buildTasks();
    for (const auto &visual : resolved_.robot.value("visuals", Json::array())) {
        RobotVisual item;
        item.asset = visual.at("asset").get<std::string>();
        item.frame = visual.at("frame").get<std::string>();
        item.root_from_frame = toMatrix(frames_.fromRoot(item.frame));
        item.frame_from_asset = toMatrix(placement(visual));
        item.mesh = mesh("robot", item.asset, visual.value("texture", std::string()));
        robot_.push_back(std::move(item));
    }
    // Reject content that silently vanished from a strict scene (mesh() throws when strict).
}

std::shared_ptr<const r::MeshAsset> PackScene::mesh(const std::string &role, const std::string &asset,
                                                    const std::string &texture) const {
    const std::string key = role + '\0' + asset + '\0' + texture;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto found = cache_.find(key);
        if (found != cache_.end())
            return found->second;
    }
    std::shared_ptr<const r::MeshAsset> loaded;
    try {
        r::MeshAsset value = r::loadMesh(resolved_.asset(role, asset));
        if (!texture.empty()) {
            const auto path = resolved_.asset(role, texture);
            bool textured = false;
            for (const auto &part : value.submeshes)
                textured = textured || part.material.diffuse_texture.has_value();
            for (auto &part : value.submeshes)
                if (!textured || part.material.diffuse_texture)
                    part.material.diffuse_texture = path;
        }
        loaded = std::make_shared<const r::MeshAsset>(std::move(value));
    } catch (const std::exception &error) {
        const std::string text = role + " pack asset '" + asset + "'" +
                                 (texture.empty() ? "" : " with texture '" + texture + "'") + ": " + error.what();
        if (options_.strict)
            throw std::runtime_error(text);
        warn(text);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    cache_[key] = loaded;
    return loaded;
}

r::Instance PackScene::instance(const std::string &role, const std::string &asset,
                                const Matrix4d &world_from_asset) const {
    r::Instance result;
    result.mesh = mesh(role, asset);
    result.transform = world_from_asset.cast<float>();
    return result;
}

void PackScene::buildPool() {
    const auto &pool = resolved_.pool;
    // Interactive (non-strict) previews of hand-written documents may omit the type; packs never do.
    const auto type = options_.strict ? pool.at("type").get<std::string>() : pool.value("type", std::string("rectangular_pool"));
    if (type != "rectangular_pool")
        throw std::runtime_error("pool type '" + type + "' has no camera scene");
    const auto &p = pool.at("parameters");
    const auto &placementJson = resolved_.scenario.at("pool_placement");
    const auto &at = placementJson.at("position_m");
    r::PoolGeometry geometry;
    geometry.dimensions = {p.at("length_m").get<float>(), p.at("width_m").get<float>(), p.at("depth_m").get<float>()};
    // Placement height raises the water level (pack_runtime).
    geometry.water_level = static_cast<float>(p.at("water_level_m").get<double>() + at.at(2).get<double>());
    geometry.deck_height = p.at("deck_height_m").get<float>();
    const double yaw = placementJson.at("yaw_deg").get<double>();
    geometry.local_to_world =
        toMatrix(upright(Json::array({at.at(0), at.at(1), 0.0}), yaw)).cast<float>();
    static_ = r::makePoolScene(geometry);
    pool_instances_ = static_.instances.size();
    pool_record_ = {{"dimensions_m", {geometry.dimensions[0], geometry.dimensions[1], geometry.dimensions[2]}},
                    {"water_level_world_m", geometry.water_level},
                    {"deck_height_m", geometry.deck_height},
                    {"placement", {{"position_m", at}, {"yaw_deg", yaw}}}};
}

void PackScene::buildTasks() {
    std::map<std::string, Json> placements;
    for (const auto &item : resolved_.scenario.at("task_placements"))
        placements[item.at("task").get<std::string>()] = item;
    for (const auto &task : resolved_.task_definitions) {
        const auto id = task.at("id").get<std::string>();
        const auto world_task = upright(placements.at(id).at("position_m"), placements.at(id).at("yaw_deg").get<double>());
        std::map<std::string, spatial::Pose> frames{{"task", spatial::Pose{}}};
        for (const auto &item : task.at("frames"))
            frames[item.at("id").get<std::string>()] = placement(item);
        std::map<std::string, Json> regions;
        for (const auto &item : task.at("regions"))
            regions[item.at("id").get<std::string>()] = item;
        for (const auto &prop : task.at("props")) {
            const auto propId = prop.at("id").get<std::string>();
            const auto type = prop.at("type").get<std::string>();
            const std::string where = "task '" + id + "' prop '" + propId + "'";
            const auto &parameters = prop.at("parameters");
            if (type == "rigid_body" || type == "contact_world") {
                // Moving props are drawn from caller-supplied poses.
                if (type == "rigid_body" && parameters.contains("visual_asset")) {
                    PropVisual visual;
                    visual.task = id;
                    visual.prop = propId;
                    visual.asset = parameters.at("visual_asset").get<std::string>();
                    visual.mesh = mesh("tasks", visual.asset);
                    visual.world_from_asset_at_reset = toMatrix(spatial::compose(
                        world_task, frames.at(parameters.at("frame").get<std::string>())));
                    props_.push_back(std::move(visual));
                } else {
                    unrendered_.push_back(id + "/" + propId);
                }
                continue;
            }
            if (type != "static_body")
                throw std::runtime_error(where + ": prop type '" + type + "' has no camera visuals");
            const auto visuals = parameters.value("visuals", Json::array());
            const auto cutouts = parameters.find("cutouts");
            if (visuals.empty()) {
                if (cutouts != parameters.end())
                    throw std::runtime_error(where + ": cutouts declared without visuals");
                unrendered_.push_back(id + "/" + propId);
                continue;
            }
            std::optional<r::PanelCutouts> panel;
            if (cutouts != parameters.end()) {
                const auto &region = regions.at(cutouts->at("region").get<std::string>()).at("parameters");
                if (region.at("plane").at("axis").get<std::string>() != "x")
                    throw std::runtime_error(where + ": cutout faces are local x offsets; region plane axis is '" +
                                             region.at("plane").at("axis").get<std::string>() + "'");
                r::PanelCutouts spec;
                for (const auto &face : cutouts->at("faces_local_x_m"))
                    spec.faces_x.push_back(face.get<float>());
                spec.half_size = region.at("half_size_m").get<float>();
                for (const auto &hole : region.at("holes"))
                    spec.cutouts.push_back({{hole.at("uv").at(0).get<float>(), hole.at("uv").at(1).get<float>()},
                                            hole.at("radius_uv").get<float>()});
                spec.tolerance = static_cast<float>(kPanelToleranceM);
                panel = std::move(spec);
            }
            std::vector<std::size_t> counts(panel ? panel->faces_x.size() : 0, 0);
            for (const auto &visual : visuals) {
                const auto asset = visual.at("asset").get<std::string>();
                const auto texture = visual.value("texture", std::string());
                if (!texture.empty())
                    textures_.push_back({{"task", id}, {"prop", propId}, {"asset", asset}, {"texture", texture}});
                auto meshAsset = mesh("tasks", asset, texture);
                if (!meshAsset) {
                    warn(where + " visual '" + asset + "' has no readable asset");
                    continue;
                }
                const auto task_asset = spatial::compose(frames.at(visual.at("frame").get<std::string>()), placement(visual));
                if (panel) {
                    panel->asset_to_panel = toMatrix(task_asset).cast<float>();
                    try {
                        auto perforated = r::perforatePanel(*meshAsset, *panel);
                        meshAsset = std::make_shared<const r::MeshAsset>(std::move(perforated.mesh));
                        for (std::size_t i = 0; i < counts.size(); ++i)
                            counts[i] += perforated.face_triangles.at(i);
                    } catch (const std::exception &error) {
                        if (options_.strict)
                            throw;
                        warn(where + " cutouts failed: " + error.what());
                        continue;
                    }
                }
                r::Instance item;
                item.mesh = meshAsset;
                item.transform = toMatrix(spatial::compose(world_task, task_asset)).cast<float>();
                const auto material = visual.value("material", std::string("asset"));
                if (material == "liner")
                    item.material = r::SurfaceMaterial::Liner;
                else if (material == "clear")
                    item.material = r::SurfaceMaterial::Clear;
                else if (material == "emissive") {
                    item.material = r::SurfaceMaterial::Emissive;
                    item.radiance = visual.value("radiance", 60.f);
                    item.casts_shadow = false;
                }
                if (visual.contains("indicator")) {
                    // The tint follows the region's indicator: initial colour at reset, latched colour once latched.
                    const auto &indicator = visual.at("indicator");
                    const auto &names = regions.at(indicator.at("region").get<std::string>()).at("parameters").at("indicator");
                    IndicatorVisual follower;
                    follower.task = id;
                    follower.region = indicator.at("region").get<std::string>();
                    follower.instance = static_.instances.size();
                    for (const auto &[state, target] : {std::pair{"initial", &follower.initial}, {"latched", &follower.latched}}) {
                        const auto rgb = vector3(indicator.at("color_rgb").at(names.at(state).get<std::string>()));
                        *target = Eigen::Vector4f(float(rgb.x()), float(rgb.y()), float(rgb.z()), 1.f);
                    }
                    item.tint = follower.initial;
                    indicators_.push_back(std::move(follower));
                }
                static_.instances.push_back(std::move(item));
            }
            if (panel) {
                Json faces = Json::array(), triangles = Json::array();
                for (std::size_t i = 0; i < counts.size(); ++i) {
                    if (counts[i] == 0 && options_.strict)
                        throw std::runtime_error(where + ": no visual geometry on cutout face x=" +
                                                 std::to_string(panel->faces_x[i]));
                    faces.push_back(panel->faces_x[i]);
                    triangles.push_back(counts[i]);
                }
                cutouts_.push_back({{"task", id}, {"prop", propId}, {"region", cutouts->at("region")},
                                    {"faces_local_x_m", faces}, {"face_triangles", triangles}});
            }
        }
    }
}

r::Scene PackScene::compose(const Matrix4d &world_from_root, const std::vector<r::Instance> &dynamic,
                            const std::vector<RobotOverride> &overrides,
                            const std::map<std::string, bool> &latched) const {
    r::Scene scene = static_;
    for (const auto &item : indicators_) {
        const auto found = latched.find(item.region);
        if (found != latched.end())
            scene.instances[item.instance].tint = found->second ? item.latched : item.initial;
    }
    scene.instances.reserve(static_.instances.size() + robot_.size() + dynamic.size());
    for (std::size_t i = 0; i < robot_.size(); ++i) {
        if (!robot_[i].mesh)
            continue;
        Matrix4d root_from_asset = robot_[i].rootFromAsset();
        for (const auto &item : overrides)
            if (item.index == i)
                root_from_asset = item.root_from_asset;
        r::Instance instance;
        instance.mesh = robot_[i].mesh;
        instance.transform = (world_from_root * root_from_asset).cast<float>();
        scene.instances.push_back(std::move(instance));
    }
    scene.instances.insert(scene.instances.end(), dynamic.begin(), dynamic.end());
    return scene;
}

Json PackScene::describe() const {
    Json props = Json::array();
    for (const auto &item : props_)
        props.push_back(item.task + "/" + item.prop);
    return {{"pool", pool_record_},
            {"static_instances", static_.instances.size()},
            {"robot_visuals", robot_.size()},
            {"cutouts", cutouts_},
            {"texture_overrides", textures_},
            {"moving_props_with_visuals", props},
            {"props_without_visuals", unrendered_},
            {"indicator_visuals", indicators_.size()},
            {"warnings", warnings_}};
}
} // namespace robotics::pack_scene

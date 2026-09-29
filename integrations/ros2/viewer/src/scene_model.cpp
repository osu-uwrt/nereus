#include "scene_model.hpp"
#include <robotics/rendering/assets.hpp>
#include <algorithm>
#include <iostream>
#include <set>

namespace robotics::ros_viewer::host {
namespace {
namespace r = robotics::rendering;
} // namespace

void SceneModel::warn(const std::string &text) const {
    warnings_.push_back(text);
    std::cerr << "robotics-pool-viewer: " << text << '\n';
}

std::shared_ptr<const r::MeshAsset> SceneModel::mesh(const std::filesystem::path &path) const {
    if (path.empty())
        return nullptr;
    const auto found = cache_.find(path);
    if (found != cache_.end())
        return found->second;
    std::shared_ptr<const r::MeshAsset> loaded;
    try {
        loaded = std::make_shared<const r::MeshAsset>(r::loadMesh(path));
    } catch (const std::exception &error) {
        warn("cannot load mesh " + path.string() + ": " + error.what());
    }
    cache_[path] = loaded;
    return loaded;
}

SceneModel::SceneModel(const Scenario &scenario, const SceneModelOptions &options, const ThrusterVisuals &thrusters,
                       const StatusLights &lights)
    : robotOnly_(options.robotOnly), scenario_(scenario), thrusters_(thrusters), lights_(lights) {
    // Pool, task visuals (cutouts, texture overrides) and robot visuals: the shared pack composition.
    if (!scenario.resolved)
        throw std::runtime_error("scenario document cannot be composed: " + scenario.resolvedError);
    pack_scene::Options packOptions;
    packOptions.strict = false; // a missing asset draws the rest of the scene and warns
    pack_ = std::make_unique<pack_scene::PackScene>(*scenario.resolved, packOptions);
    for (const auto &text : pack_->warnings())
        warnings_.push_back(text);
    baseFromRoot_ = pack_->rootFromFrame(scenario.baseId).inverse();
    box_ = r::makeBoxMesh();

    // Rotor and claw parts are recognised by asset id from viewer data.
    std::set<std::string> leftAssets, rightAssets;
    for (const auto &id : lookup(options.config, {"claw", "left_assets"}))
        leftAssets.insert(id.as<std::string>());
    for (const auto &id : lookup(options.config, {"claw", "right_assets"}))
        rightAssets.insert(id.as<std::string>());
    for (const auto &visual : pack_->robotVisuals()) {
        RobotAnimation item;
        for (std::size_t i = 0; i < thrusters.rotors.size(); ++i)
            if (thrusters.rotors[i].asset == visual.asset)
                item.rotor = int(i);
        item.clawSide = leftAssets.count(visual.asset) ? 1 : rightAssets.count(visual.asset) ? -1 : 0;
        animation_.push_back(item);
    }
    // Viewer-only robot visuals that the pack lists as assets but not as visuals (launcher, magnet).
    for (const auto &entry : options.config["extra_visuals"]) {
        const auto assetId = entry["asset"].as<std::string>();
        const auto it = scenario.robotAssets.find(assetId);
        auto asset = it == scenario.robotAssets.end() ? nullptr : mesh(it->second);
        if (!asset) {
            warn("extra visual '" + assetId + "' has no readable asset");
            continue;
        }
        glm::mat4 base(1);
        if (entry["mechanism"]) {
            const auto *m = scenario.mechanism(entry["mechanism"].as<std::string>());
            if (!m) {
                warn("extra visual '" + assetId + "' names an unknown mechanism");
                continue;
            }
            base = m->frameInBase;
            if (entry["at_tip"].as<bool>(false))
                base = base * glm::translate(glm::mat4(1), m->tip);
        } else {
            base = scenario.frames.relative(scenario.baseId, entry["frame"].as<std::string>(scenario.baseId));
        }
        extras_.emplace_back(asset, base);
    }
    const auto payloadAsset = lookup(options.config, {"payloads", "mesh_asset"});
    if (payloadAsset) {
        const auto it = scenario.robotAssets.find(payloadAsset.as<std::string>());
        if (it != scenario.robotAssets.end())
            payloadMesh_ = it->second;
    }

    // Calibration board: local +X is the printed face normal.
    const auto board = lookup(options.config, {"calibration_board"});
    if (board && board["texture"]) {
        const auto texture = options.configDirectory / board["texture"].as<std::string>();
        const auto size = board["size_m"];
        const float hw = size[0].as<float>() / 2, hh = size[1].as<float>() / 2;
        auto asset = std::make_shared<r::MeshAsset>();
        r::Submesh sub;
        sub.vertices = {{{0, -hw, -hh}, {1, 0, 0}, {0, 0}},
                        {{0, hw, -hh}, {1, 0, 0}, {1, 0}},
                        {{0, hw, hh}, {1, 0, 0}, {1, 1}},
                        {{0, -hw, hh}, {1, 0, 0}, {0, 1}}};
        sub.indices = {0, 1, 2, 0, 2, 3};
        sub.material.diffuse_texture = texture;
        asset->submeshes.push_back(std::move(sub));
        asset->minimum = {0, -hw, -hh};
        asset->maximum = {0, hw, hh};
        board_ = asset;
        boardPose_ = poseQuat(vec3(board["position_m"]), glm::quat(1, 0, 0, 0));
        if (board["yaw_deg"])
            boardPose_ = pose(vec3(board["position_m"]), {0, 0, glm::radians(board["yaw_deg"].as<float>())});
    }
}

std::vector<glm::mat4> SceneModel::payloadMounts(const std::string &mechanism) const {
    std::vector<glm::mat4> result;
    if (const auto *m = scenario_.mechanism(mechanism))
        for (const auto &slot : m->slotsInBase)
            result.push_back(slot * glm::scale(glm::mat4(1), glm::vec3(m->projectileLength, 2 * m->projectileRadius,
                                                                        2 * m->projectileRadius)));
    return result;
}

Eigen::Matrix4d SceneModel::worldFromRoot(const glm::mat4 &worldFromBase) const {
    return toEigen(worldFromBase).cast<double>() * baseFromRoot_;
}

r::Scene SceneModel::build(const VisualState &state) const {
    const auto add = [&](std::vector<r::Instance> &into, const std::shared_ptr<const r::MeshAsset> &asset,
                         const glm::mat4 &world) {
        r::Instance instance;
        instance.mesh = asset;
        instance.transform = toEigen(world);
        into.push_back(std::move(instance));
    };
    std::vector<r::Instance> dynamic;
    if (board_ && state.showBoard)
        add(dynamic, board_, boardPose_);
    // Rotor spin and claw travel replace the reset placement of those robot visuals.
    std::vector<pack_scene::RobotOverride> overrides;
    for (std::size_t i = 0; i < animation_.size(); ++i) {
        const auto &part = pack_->robotVisuals()[i];
        const auto &item = animation_[i];
        if (item.rotor < 0 && item.clawSide == 0)
            continue;
        Eigen::Matrix4d local = part.frame_from_asset;
        if (item.rotor >= 0 && std::size_t(item.rotor) < state.rotorSpin.size())
            local = local * toEigen(state.rotorSpin[std::size_t(item.rotor)]).cast<double>();
        Eigen::Matrix4d rootFromFrame = part.root_from_frame;
        if (item.clawSide) {
            Eigen::Matrix4d slide = Eigen::Matrix4d::Identity();
            slide(1, 3) = item.clawSide > 0 ? state.claw[0] : -state.claw[1];
            rootFromFrame = rootFromFrame * slide;
        }
        overrides.push_back({i, rootFromFrame * local});
    }
    for (const auto &[asset, base] : extras_)
        add(dynamic, asset, state.body * base);
    for (std::size_t i = 0; i < lights_.lights.size(); ++i) {
        const auto &light = lights_.lights[i];
        r::Instance instance;
        instance.mesh = box_;
        instance.material = r::SurfaceMaterial::Emissive;
        instance.radiance = light.radiance;
        instance.casts_shadow = false;
        const glm::vec3 color = i < state.lightColor.size() ? state.lightColor[i] : glm::vec3(0);
        instance.tint << color.x, color.y, color.z, 1.f;
        instance.transform =
            toEigen(state.body * scenario_.frames.relative(scenario_.baseId, light.frame) * light.mount *
                    glm::scale(glm::mat4(1), light.size));
        dynamic.push_back(std::move(instance));
    }
    if (!payloadMesh_.empty())
        if (auto payload = mesh(payloadMesh_))
            for (const auto &world : state.loadedPayloads)
                add(dynamic, payload, world);
    for (const auto &marker : state.markers) {
        r::Instance instance;
        instance.mesh = marker.mesh.empty() ? box_ : mesh(marker.mesh);
        if (!instance.mesh)
            continue;
        instance.transform = toEigen(marker.world * glm::scale(glm::mat4(1), marker.scale));
        if (marker.emissive) {
            instance.material = r::SurfaceMaterial::Emissive;
            instance.radiance = marker.radiance;
            instance.casts_shadow = false;
            instance.tint = Eigen::Vector4f(marker.tint.x, marker.tint.y, marker.tint.z, marker.tint.w);
        }
        dynamic.push_back(std::move(instance));
    }
    r::Scene scene = pack_->compose(worldFromRoot(state.body), dynamic, overrides, state.indicatorLatched);
    if (robotOnly_) { // static scene = pool + task visuals; robot and dynamic instances follow it
        for (std::size_t i = 0; i < pack_->staticScene().instances.size() && i < scene.instances.size(); ++i)
            scene.instances[i].visible = false;
        scene.water.reset();
    }
    // Pool instance order (rendering/scene.hpp): floor, four walls, four decks, four coping strips.
    const std::size_t pool = std::min(pack_->poolInstanceCount(), scene.instances.size());
    if (!state.showFloor && pool > 0)
        scene.instances[0].visible = false;
    if (!state.showWalls)
        for (std::size_t i = 1; i <= 12 && i < pool; ++i)
            scene.instances[i].visible = false;
    return scene;
}
} // namespace robotics::ros_viewer::host

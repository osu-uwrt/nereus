#include "scene_model.hpp"
#include <robotics/rendering/assets.hpp>
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
    : scenario_(scenario), thrusters_(thrusters), lights_(lights) {
    // Pool, water and deck.
    r::PoolGeometry geometry;
    geometry.dimensions = {scenario.poolLength, scenario.poolWidth, scenario.poolDepth};
    geometry.water_level = scenario.waterLevel;
    geometry.deck_height = scenario.deckHeight;
    geometry.local_to_world = toEigen(scenario.poolToWorld);
    static_ = r::makePoolScene(geometry);
    poolInstances_ = static_.instances.size();
    box_ = r::makeBoxMesh();

    // Task visuals (with perforated-panel cutouts) at their placements.
    for (const auto &visual : scenario.taskVisuals) {
        auto asset = mesh(visual.path);
        if (!asset) {
            warn("task '" + visual.task + "' visual '" + visual.asset + "' has no readable asset");
            continue;
        }
        if (visual.cutouts) {
            r::PanelCutouts panel;
            panel.asset_to_panel = toEigen(visual.taskFromAsset);
            panel.faces_x = visual.cutouts->facesX;
            panel.half_size = visual.cutouts->halfSize;
            panel.cutouts = visual.cutouts->holes;
            try {
                asset = std::make_shared<const r::MeshAsset>(r::perforatePanel(*asset, panel).mesh);
            } catch (const std::exception &error) {
                warn("task '" + visual.task + "' cutouts failed: " + error.what());
                continue;
            }
        }
        r::Instance instance;
        instance.mesh = asset;
        instance.transform = toEigen(visual.world);
        static_.instances.push_back(std::move(instance));
    }

    // Robot visuals; rotor and claw parts are recognised by asset id from viewer data.
    std::set<std::string> leftAssets, rightAssets;
    for (const auto &id : lookup(options.config, {"claw", "left_assets"}))
        leftAssets.insert(id.as<std::string>());
    for (const auto &id : lookup(options.config, {"claw", "right_assets"}))
        rightAssets.insert(id.as<std::string>());
    for (const auto &visual : scenario.robotVisuals) {
        RobotInstance item;
        item.mesh = mesh(visual.path);
        if (!item.mesh) {
            warn("robot visual '" + visual.asset + "' has no readable asset");
            continue;
        }
        item.frameInBase = visual.frameInBase;
        item.local = visual.local;
        for (std::size_t i = 0; i < thrusters.rotors.size(); ++i)
            if (thrusters.rotors[i].asset == visual.asset)
                item.rotor = int(i);
        item.clawSide = leftAssets.count(visual.asset) ? 1 : rightAssets.count(visual.asset) ? -1 : 0;
        robot_.push_back(std::move(item));
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

r::Scene SceneModel::build(const VisualState &state) const {
    r::Scene scene = static_;
    if (!state.showWalls)
        for (std::size_t i = 1; i <= 4 && i < poolInstances_; ++i)
            scene.instances[i].visible = false;
    if (board_ && state.showBoard) {
        r::Instance instance;
        instance.mesh = board_;
        instance.transform = toEigen(boardPose_);
        scene.instances.push_back(std::move(instance));
    }
    const auto add = [&](const std::shared_ptr<const r::MeshAsset> &asset, const glm::mat4 &world) {
        r::Instance instance;
        instance.mesh = asset;
        instance.transform = toEigen(world);
        scene.instances.push_back(std::move(instance));
    };
    for (const auto &item : robot_) {
        glm::mat4 local = item.local;
        if (item.rotor >= 0 && std::size_t(item.rotor) < state.rotorSpin.size())
            local = local * state.rotorSpin[std::size_t(item.rotor)];
        glm::mat4 world = state.body * item.frameInBase;
        if (item.clawSide)
            world = world * glm::translate(glm::mat4(1), {0, item.clawSide > 0 ? state.claw[0] : -state.claw[1], 0});
        add(item.mesh, world * local);
    }
    for (const auto &[asset, base] : extras_)
        add(asset, state.body * base);
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
        scene.instances.push_back(std::move(instance));
    }
    if (!payloadMesh_.empty())
        if (auto payload = mesh(payloadMesh_))
            for (const auto &world : state.loadedPayloads)
                add(payload, world);
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
        scene.instances.push_back(std::move(instance));
    }
    return scene;
}
} // namespace robotics::ros_viewer::host

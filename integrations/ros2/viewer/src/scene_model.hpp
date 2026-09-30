// Composes the renderer scene (pool, course, robot, mechanisms, lights) from a Scenario and the
// per-frame VisualState. Pool, task visuals (cutouts, texture overrides) and robot visuals come from
// nereus_pack_scene, the same composition the simulator's cameras use; this class adds the viewer-only
// content (rotor/claw animation, status lights, calibration board, markers, payloads).
#pragma once
#include "scenario.hpp"
#include "status_lights.hpp"
#include "thruster_visuals.hpp"
#include "visual_state.hpp"
#include <robotics/pack_scene/pack_scene.hpp>
#include <robotics/rendering/scene.hpp>
#include <map>
#include <memory>

namespace robotics::ros_viewer::host {
struct ExtraVisual {
    std::string id, asset, frame;
    glm::mat4 local{1};
    bool atTip = false;
    std::string mechanism;
};
struct SceneModelOptions {
    YAML::Node config;                   // host viewer document
    std::filesystem::path configDirectory; // for relative resources (calibration board texture)
    bool robotOnly = false;                // hide pool, water and course visuals (real-robot use)
};
class SceneModel {
  public:
    SceneModel(const Scenario &, const SceneModelOptions &, const ThrusterVisuals &, const StatusLights &);
    rendering::Scene build(const VisualState &) const;
    std::shared_ptr<const rendering::MeshAsset> mesh(const std::filesystem::path &) const; // cached; null on failure
    const std::vector<std::string> &warnings() const {
        return warnings_;
    }
    std::size_t robotVisualCount() const {
        std::size_t count = 0;
        for (const auto &visual : pack_->robotVisuals())
            count += visual.mesh != nullptr;
        return count;
    }
    const pack_scene::PackScene &pack() const {
        return *pack_;
    }
    // Truth base_link pose -> robot frame root (the pack's pose convention).
    Eigen::Matrix4d worldFromRoot(const glm::mat4 &worldFromBase) const;
    // Slot poses for `mechanism` in base_link, scaled to the projectile size (drawn by loadedPayloads).
    std::vector<glm::mat4> payloadMounts(const std::string &mechanism) const;
    std::filesystem::path payloadMesh() const {
        return payloadMesh_;
    }

  private:
    struct RobotAnimation {
        int rotor = -1;
        int clawSide = 0; // -1 right, +1 left
    };
    bool robotOnly_ = false;
    const Scenario &scenario_;
    const ThrusterVisuals &thrusters_;
    const StatusLights &lights_;
    std::unique_ptr<pack_scene::PackScene> pack_;
    std::vector<RobotAnimation> animation_; // parallel to pack_->robotVisuals()
    Eigen::Matrix4d baseFromRoot_ = Eigen::Matrix4d::Identity();
    std::vector<std::pair<std::shared_ptr<const rendering::MeshAsset>, glm::mat4>> extras_; // base_link poses
    std::shared_ptr<const rendering::MeshAsset> box_, board_;
    glm::mat4 boardPose_{1};
    std::filesystem::path payloadMesh_;
    mutable std::map<std::filesystem::path, std::shared_ptr<const rendering::MeshAsset>> cache_;
    mutable std::vector<std::string> warnings_;
    void warn(const std::string &) const;
};
} // namespace robotics::ros_viewer::host

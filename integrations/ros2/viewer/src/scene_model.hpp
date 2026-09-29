// Composes the renderer scene (pool, course, robot, mechanisms, lights) from a Scenario and the
// per-frame VisualState. Mirrors python/src/robotics_platform/pack_cameras.py composition.
#pragma once
#include "scenario.hpp"
#include "status_lights.hpp"
#include "thruster_visuals.hpp"
#include "visual_state.hpp"
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
        return robot_.size();
    }
    // Slot poses for `mechanism` in base_link, scaled to the projectile size (drawn by loadedPayloads).
    std::vector<glm::mat4> payloadMounts(const std::string &mechanism) const;
    std::filesystem::path payloadMesh() const {
        return payloadMesh_;
    }

  private:
    struct RobotInstance {
        std::shared_ptr<const rendering::MeshAsset> mesh;
        glm::mat4 frameInBase{1}, local{1};
        int rotor = -1;
        int clawSide = 0; // -1 right, +1 left
    };
    const Scenario &scenario_;
    const ThrusterVisuals &thrusters_;
    const StatusLights &lights_;
    rendering::Scene static_;
    std::size_t poolInstances_ = 0;
    std::vector<RobotInstance> robot_;
    std::vector<std::pair<std::shared_ptr<const rendering::MeshAsset>, glm::mat4>> extras_; // base_link poses
    std::shared_ptr<const rendering::MeshAsset> box_, board_;
    glm::mat4 boardPose_{1};
    std::filesystem::path payloadMesh_;
    mutable std::map<std::filesystem::path, std::shared_ptr<const rendering::MeshAsset>> cache_;
    mutable std::vector<std::string> warnings_;
    void warn(const std::string &) const;
};
} // namespace robotics::ros_viewer::host

// Composes the renderer scene (pool, course, robot, mechanisms, lights) from a Scenario and the
// per-frame VisualState. Pool, task visuals (cutouts, texture overrides) and robot visuals come from
// nereus_pack_scene, the same composition the simulator's cameras use (equipment such as the calibration
// board included); this class adds the viewer-only content (rotor/claw animation, status lights, markers,
// payloads).
#pragma once
#include "scenario.hpp"
#include "status_lights.hpp"
#include "thruster_visuals.hpp"
#include "visual_state.hpp"
#include <map>
#include <memory>
#include <nereus/pack_scene/pack_scene.hpp>
#include <nereus/rendering/scene.hpp>

namespace nereus::ros_viewer::host {
// An extra_visuals entry of the host yaml (currently unused: SceneModel reads the YAML directly).
struct ExtraVisual {
    std::string id, asset, frame;
    glm::mat4 local{1};
    bool atTip = false;
    std::string mechanism;
};

struct SceneModelOptions {
    YAML::Node config;      // host viewer document
    bool robotOnly = false; // hide pool, water and course visuals (real-robot use)
};

// Built once per scenario; build() is called per frame. Holds references to the scenario, thruster visuals
// and status lights, which must outlive it.
class SceneModel {
  public:
    SceneModel(const Scenario &, const SceneModelOptions &, const ThrusterVisuals &, const StatusLights &);

    // The full scene for this frame: the pack composition at the body pose plus the viewer's dynamic instances,
    // with the visibility toggles applied.
    rendering::Scene build(const VisualState &) const;

    std::shared_ptr<const rendering::MeshAsset> mesh(const std::filesystem::path &) const; // cached; null on failure
    const std::vector<std::string> &warnings() const {
        return warnings_;
    }

    // Robot visuals whose mesh loaded.
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

    // Mesh file drawn for loaded payloads (host yaml payloads.mesh_asset); empty when none.
    std::filesystem::path payloadMesh() const {
        return payloadMesh_;
    }

  private:
    // How one pack robot visual animates: the rotor it spins with (index into ThrusterVisuals::rotors) and/or
    // the claw jaw it slides with.
    struct RobotAnimation {
        int rotor = -1;
        int clawSide = 0; // -1 right, +1 left
    };

    bool robotOnly_ = false;
    const Scenario &scenario_;
    const ThrusterVisuals &thrusters_;
    const StatusLights &lights_;

    // Pack composition and the per-visual animation, base_link <- robot frame root.
    std::unique_ptr<pack_scene::PackScene> pack_;
    std::vector<RobotAnimation> animation_; // parallel to pack_->robotVisuals()
    Eigen::Matrix4d baseFromRoot_ = Eigen::Matrix4d::Identity();

    // Viewer-only meshes: extra visuals, the unit box (status lights, markers) and the payload mesh.
    std::vector<std::pair<std::shared_ptr<const rendering::MeshAsset>, glm::mat4>> extras_; // base_link poses
    std::shared_ptr<const rendering::MeshAsset> box_;
    std::filesystem::path payloadMesh_;

    // Loaded meshes by path (null entries remember failures) and warnings; filled lazily from const methods.
    mutable std::map<std::filesystem::path, std::shared_ptr<const rendering::MeshAsset>> cache_;
    mutable std::vector<std::string> warnings_;
    void warn(const std::string &) const;
};
} // namespace nereus::ros_viewer::host

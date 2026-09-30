// Viewer-side view of the bridge's latched scenario document (resolved.json layout plus asset_paths).
// Everything the host draws or names (poses, frames, cameras, meshes, topics) is read from here.
#pragma once
#include "camera_geometry.hpp"
#include "frame_graph.hpp"
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <robotics/rendering/scene.hpp>
#include <robotics/session/scenario.hpp>
#include <string>
#include <vector>

namespace robotics::ros_viewer::host {
struct SensorCamera {
    std::string id, mountFrame, opticalFrame, rosOpticalFrame, title, model;
    Intrinsics k;
    double periodS = 1. / 15, minRange = .15, maxRange = 4;
    std::string rgbTopic, depthTopic, infoTopic; // absolute; empty when the bridge does not publish them
    glm::mat4 mountInBase{1}, opticalInBase{1};
};
struct RobotVisual {
    std::string asset, frame;
    glm::mat4 inBase{1};      // frame pose in base_link composed with the visual placement
    glm::mat4 frameInBase{1}; // the visual's frame in base_link
    glm::mat4 local{1};       // placement in `frame`
    std::filesystem::path path;
};
struct Mechanism {
    std::string id, type, frame;
    glm::mat4 frameInBase{1};
    std::vector<glm::mat4> slotsInBase; // launchers/droppers: payload seating in base_link
    float projectileLength = 0, projectileRadius = 0;
    float minGap = 0; // claw
    glm::vec3 tip{0}; // magnet
};
struct CutoutSpec {
    std::vector<float> facesX;
    float halfSize = 0;
    std::vector<rendering::UvCutout> holes;
};
struct TaskVisual {
    std::string task, prop, asset;
    glm::mat4 taskFromAsset{1}, world{1}; // world = task placement * taskFromAsset
    std::filesystem::path path;
    std::optional<CutoutSpec> cutouts;
};
struct Landmark {
    glm::mat4 world{1};
};
struct Scenario {
    std::string id, robotId, ns, mapFrame, truthBaseFrame, estimateBaseFrame, baseId;
    YAML::Node document, bridge, ui;
    FrameGraph frames;
    glm::mat4 poolToWorld{1}, worldToPool{1};
    float poolLength = 50, poolWidth = 22.86f, poolDepth = 2.1336f, deckHeight = .305f, waterLevel = 0;
    std::string poolId;
    rendering::Appearance appearance;
    std::vector<SensorCamera> cameras;
    std::vector<RobotVisual> robotVisuals;
    std::vector<std::string> thrusterOrder;
    std::map<std::string, Mechanism> mechanisms;
    std::vector<TaskVisual> taskVisuals;
    std::map<std::string, Landmark> landmarks;
    std::map<std::string, std::filesystem::path> robotAssets;
    std::map<std::string, std::string> frameNames; // pack frame id -> ROS frame
    YAML::Node runOptions;
    // The same document as the session's resolved scenario (asset_paths filled from the pack folder for
    // local previews); null with `resolvedError` set when it lacks what scene composition needs.
    std::shared_ptr<const session::ResolvedScenario> resolved;
    std::string resolvedError;

    std::string absolute(const std::string &relative) const; // bridge-namespaced topic
    const SensorCamera *camera(const std::string &id) const;
    const Mechanism *mechanism(const std::string &id) const;
    // ROS frame name for a pack frame id (bridge frame_names, else <namespace>/<frame>).
    std::string rosFrame(const std::string &packFrame) const;
};

// `config`: the host viewer document (fallback ui, camera display names). `packDirHint` locates the
// pack folder for documents without asset_paths (local previews); empty searches NEREUS_PACK_CONTENT.
Scenario parseScenario(const std::string &json, const YAML::Node &config, const std::filesystem::path &packDirHint);
} // namespace robotics::ros_viewer::host

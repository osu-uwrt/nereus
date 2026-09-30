#pragma once
#include "robotics/rendering/assets.hpp"
#include <memory>
#include <optional>
#include <string>

namespace robotics::rendering {
enum class SurfaceMaterial { Asset = 0, Tiles = 1, Deck = 2, Lamp = 3, Liner = 4, Clear = 5, Emissive = 6 };
struct Instance {
    std::shared_ptr<const MeshAsset> mesh;
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    Eigen::Vector4f tint = Eigen::Vector4f::Ones();
    SurfaceMaterial material = SurfaceMaterial::Asset;
    float radiance = 60;
    bool visible = true, casts_shadow = true;
};
struct WaterSurface {
    Instance surface;
    Eigen::Vector3f dimensions = {50, 22.86f, 2.1336f};
    float level = 0;                                              // World Z; surface is horizontal.
    Eigen::Matrix4f local_to_world = Eigen::Matrix4f::Identity(); // XY translation and yaw only.
};
// Coloured points (e.g. a point cloud), drawn unlit and depth-tested after the water surface in
// observer views only (never in Appearance::preview draws). Each distinct PointData is uploaded once and
// kept on the GPU while a scene references it, so share one object across frames until it changes.
struct PointData {
    std::vector<float> xyzrgb; // per point: x, y, z in the set's local frame (metres), r, g, b in [0, 1]
};
struct PointSet {
    std::shared_ptr<const PointData> data;
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity(); // local -> world
    float size = 3;                                          // pixels
};
struct Scene {
    std::vector<Instance> instances;
    std::vector<PointSet> points;
    std::optional<WaterSurface> water;
    Eigen::Vector3f lighting_center = Eigen::Vector3f::Zero();
};
struct View {
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Vector3f eye = Eigen::Vector3f::Zero();
};
struct WaterOptics {
    Eigen::Vector3f tint = {.025f, .22f, .29f}, absorption = {.095f, .035f, .025f};
    float scattering = .10f, distance_scale = 1, distance_power = 1, clear_distance = 0;
};
struct Appearance {
    WaterOptics water;
    float caustics = 1, exposure = 1;
    bool surface = true, shadows = true, reflections = true, outdoor = false;
    float sun_azimuth = 225, sun_elevation = 55; // degrees
    float direct_light = 1, ambient_light = .7f, glare = .5f;
    // Cheap preview draw (camera cards, thumbnails): renders into its own targets (no resize churn against
    // the main view), reuses the shadow map of the last full draw when one exists, and skips the surface
    // reflection and bloom passes. Default false keeps every existing draw bit-identical.
    bool preview = false;
    // Observer-only orbit focus marker: a shaded, depth-tested yellow disc at this world
    // point, sized to the camera distance. Unset (default) keeps every draw bit-identical; never set it
    // for sensor renders.
    std::optional<Eigen::Vector3f> focus;
};
struct PoolGeometry {
    Eigen::Vector3f dimensions = {50, 22.86f, 2.1336f};
    float water_level = 0, deck_height = .305288888f;
    Eigen::Matrix4f local_to_world = Eigen::Matrix4f::Identity();
};
// Unit cube centered on origin, six separate normal/UV faces. CPU-only geometry.
std::shared_ptr<const MeshAsset> makeBoxMesh();
// Optional original pool appearance geometry. No robot, task, fluid dynamics or transport.
// Instance order: floor, four walls, four decks, four coping strips.
Scene makePoolScene(const PoolGeometry &parameters = {});
} // namespace robotics::rendering

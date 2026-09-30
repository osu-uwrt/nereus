#pragma once
#include "nereus/rendering/assets.hpp"
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace nereus::rendering {
// Marking: an alpha-blended decal whose UVs span [-1, 1] across the painted area (the mesh may extend past
// it so the edge can be anti-aliased). It is drawn in instance order without writing depth, so it must follow
// the surface it lies on and precede anything that may sit in front of that surface.
enum class SurfaceMaterial {
    Asset = 0,
    Tiles = 1,
    Deck = 2,
    Lamp = 3,
    Liner = 4,
    Clear = 5,
    Emissive = 6,
    Marking = 7
};
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
    // Tiles material finish: grout pitch (0 = plain) and the wall band around the waterline, as
    // [bottom, top] metres relative to the water surface (an empty band draws nothing).
    float tile_size = .1524f;
    Eigen::Vector2f waterline_band = {-.13f, .04f};
    Eigen::Vector3f waterline_color = {.065f, .20f, .27f};
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
enum class PoolSide { Floor, XMin, XMax, YMin, YMax };
// A painted stripe (lane line, T bar, wall target). On the floor, from/to are pool-local (x, y); on a wall
// they are (coordinate along the wall, z relative to the water surface). Metres.
struct PoolStripe {
    PoolSide side = PoolSide::Floor;
    Eigen::Vector2f from = Eigen::Vector2f::Zero(), to = Eigen::Vector2f::Zero();
    float width = .254f;
    Eigen::Vector3f color = {.093f, .14f, .16f};
};
struct PoolGeometry {
    Eigen::Vector3f dimensions = {50, 22.86f, 2.1336f};
    float water_level = 0, deck_height = .305288888f;
    Eigen::Matrix4f local_to_world = Eigen::Matrix4f::Identity();
    Eigen::Vector3f tile_color = {.68f, .85f, .87f};
    float tile_size = .1524f;
    Eigen::Vector2f waterline_band = {-.13f, .04f};
    Eigen::Vector3f waterline_color = {.065f, .20f, .27f};
    std::vector<PoolStripe> markings;
};
// Unit cube centered on origin, six separate normal/UV faces. CPU-only geometry.
std::shared_ptr<const MeshAsset> makeBoxMesh();
// Optional original pool appearance geometry. No robot, task, fluid dynamics or transport.
// Instance order: floor, four walls, four decks, four coping strips, then one Marking instance for the
// floor stripes and one for the wall stripes, each only when there are any.
Scene makePoolScene(const PoolGeometry &parameters = {});
} // namespace nereus::rendering

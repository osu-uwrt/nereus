#pragma once
#include "robotics/rendering/assets.hpp"
#include <memory>
#include <string>

namespace robotics::rendering {
enum class SurfaceMaterial {
    Asset = 0,
    Tiles = 1,
    Deck = 2,
    Lamp = 3,
    Liner = 4,
    Clear = 5,
    Emissive = 6
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
    float level = 0;                                              // World Z; surface is horizontal.
    Eigen::Matrix4f local_to_world = Eigen::Matrix4f::Identity(); // XY translation and yaw only.
};
struct Scene {
    std::vector<Instance> instances;
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
};
struct PoolGeometry {
    Eigen::Vector3f dimensions = {50, 22.86f, 2.1336f};
    float water_level = 0, deck_height = .305288888f;
    Eigen::Matrix4f local_to_world = Eigen::Matrix4f::Identity();
};
// Optional original pool appearance geometry. No robot, task, fluid dynamics or transport.
// Instance order: floor, four walls, four decks, four coping strips.
Scene makePoolScene(const PoolGeometry &parameters = {});
} // namespace robotics::rendering

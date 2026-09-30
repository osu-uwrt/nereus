#include "nereus/rendering/scene.hpp"
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <limits>
#include <stdexcept>

namespace nereus::rendering {
namespace {
Eigen::Matrix4f eigen(const glm::mat4 &m) {
    return Eigen::Map<const Eigen::Matrix4f>(glm::value_ptr(m));
}
bool unitColor(const Eigen::Vector3f &c) {
    return c.allFinite() && (c.array() >= 0).all() && (c.array() <= 1).all();
}
// Decal quads for one pool side, one submesh per colour, in pool-local coordinates relative to the water
// surface. Each quad extends kPad past its stripe so the Marking shader can fade the edge.
std::shared_ptr<const MeshAsset> stripeMesh(const std::vector<PoolStripe> &stripes, bool floor, float length,
                                            float width, float depth) {
    constexpr float kPad = .05f, kLift = .002f; // the lift keeps decals off the surface they lie on
    auto result = std::make_shared<MeshAsset>();
    result->minimum.setConstant(std::numeric_limits<float>::max());
    result->maximum.setConstant(std::numeric_limits<float>::lowest());
    for (const auto &stripe : stripes) {
        if ((stripe.side == PoolSide::Floor) != floor)
            continue;
        const Eigen::Vector2f along = stripe.to - stripe.from;
        const float half = along.norm() / 2, halfWidth = stripe.width / 2;
        const Eigen::Vector2f u = along / (2 * half), across(-u.y(), u.x()), center = (stripe.from + stripe.to) / 2;
        Eigen::Vector3f normal;
        const auto place = [&](Eigen::Vector2f q) -> Eigen::Vector3f {
            switch (stripe.side) {
            case PoolSide::Floor:
                normal = {0, 0, 1};
                return {q.x(), q.y(), -depth + kLift};
            case PoolSide::XMin:
                normal = {1, 0, 0};
                return {kLift, q.x(), q.y()};
            case PoolSide::XMax:
                normal = {-1, 0, 0};
                return {length - kLift, q.x(), q.y()};
            case PoolSide::YMin:
                normal = {0, 1, 0};
                return {q.x(), kLift, q.y()};
            case PoolSide::YMax:
                normal = {0, -1, 0};
                return {q.x(), width - kLift, q.y()};
            }
            throw std::invalid_argument("unknown pool side");
        };
        auto found = std::find_if(result->submeshes.begin(), result->submeshes.end(), [&](const Submesh &part) {
            return part.material.base_color.head<3>() == stripe.color;
        });
        if (found == result->submeshes.end()) {
            Submesh part;
            part.material.base_color << stripe.color, 1;
            result->submeshes.push_back(std::move(part));
            found = std::prev(result->submeshes.end());
        }
        const auto start = static_cast<std::uint32_t>(found->vertices.size());
        const Eigen::Vector2f uv((half + kPad) / half, (halfWidth + kPad) / halfWidth);
        for (Eigen::Vector2f corner :
             {Eigen::Vector2f(-1, -1), Eigen::Vector2f(1, -1), Eigen::Vector2f(1, 1), Eigen::Vector2f(-1, 1)}) {
            const Eigen::Vector3f position =
                place(center + corner.x() * (half + kPad) * u + corner.y() * (halfWidth + kPad) * across);
            found->vertices.push_back({position, normal, corner.cwiseProduct(uv)});
            result->minimum = result->minimum.cwiseMin(position);
            result->maximum = result->maximum.cwiseMax(position);
        }
        for (auto i : {0U, 1U, 2U, 0U, 2U, 3U})
            found->indices.push_back(start + i);
    }
    if (result->submeshes.empty())
        return nullptr;
    return result;
}
} // namespace
std::shared_ptr<const MeshAsset> makeBoxMesh() {
    auto result = std::make_shared<MeshAsset>();
    Submesh mesh;
    for (int axis = 0; axis < 3; ++axis)
        for (int sign : {-1, 1}) {
            Eigen::Vector3f n = Eigen::Vector3f::Zero(), u = n, w = n;
            n[axis] = static_cast<float>(sign);
            u[(axis + 1) % 3] = 1;
            w[(axis + 2) % 3] = static_cast<float>(sign);
            const auto start = static_cast<std::uint32_t>(mesh.vertices.size());
            for (Eigen::Vector2f uv :
                 {Eigen::Vector2f(0, 0), Eigen::Vector2f(1, 0), Eigen::Vector2f(1, 1), Eigen::Vector2f(0, 1)})
                mesh.vertices.push_back({n * .5f + (uv.x() - .5f) * u + (uv.y() - .5f) * w, n, uv});
            for (auto i : {0U, 1U, 2U, 0U, 2U, 3U})
                mesh.indices.push_back(start + i);
        }
    result->submeshes.push_back(std::move(mesh));
    result->minimum.setConstant(-.5f);
    result->maximum.setConstant(.5f);
    return result;
}
Scene makePoolScene(const PoolGeometry &p) {
    if (!p.dimensions.allFinite() || (p.dimensions.array() <= 0).any() || !std::isfinite(p.water_level) ||
        !std::isfinite(p.deck_height) || p.deck_height < 0 || !p.local_to_world.allFinite() ||
        !p.local_to_world.row(3).isApprox(Eigen::RowVector4f(0, 0, 0, 1)) ||
        !p.local_to_world.col(2).isApprox(Eigen::Vector4f(0, 0, 1, 0)) ||
        !p.local_to_world.row(2).isApprox(Eigen::RowVector4f(0, 0, 1, 0)) ||
        !(p.local_to_world.topLeftCorner<3, 3>().transpose() * p.local_to_world.topLeftCorner<3, 3>())
             .isApprox(Eigen::Matrix3f::Identity(), 1e-5f) ||
        std::abs(p.local_to_world.topLeftCorner<3, 3>().determinant() - 1) > 1e-5f)
        throw std::invalid_argument("pool appearance requires positive dimensions and a horizontal rigid frame");
    if (!unitColor(p.tile_color) || !unitColor(p.waterline_color) || !std::isfinite(p.tile_size) || p.tile_size < 0 ||
        !p.waterline_band.allFinite())
        throw std::invalid_argument("pool finish requires unit colours, a non-negative tile size and a finite band");
    for (const auto &stripe : p.markings)
        if (!stripe.from.allFinite() || !stripe.to.allFinite() || (stripe.to - stripe.from).norm() <= 0 ||
            !std::isfinite(stripe.width) || stripe.width <= 0 || !unitColor(stripe.color))
            throw std::invalid_argument(
                "pool stripes require distinct finite ends, a positive width and a unit colour");
    Scene scene;
    const auto geometry = makeBoxMesh();
    const float length = p.dimensions.x(), width = p.dimensions.y(), depth = p.dimensions.z(), deck = p.deck_height;
    const auto pool = glm::make_mat4(p.local_to_world.data());
    const auto at = [&](float x, float y, float z) {
        return pool * glm::translate(glm::mat4(1), {x, y, z + p.water_level});
    };
    const auto box = [&](const glm::mat4 &matrix, glm::vec3 size, Eigen::Vector3f color,
                         SurfaceMaterial material = SurfaceMaterial::Asset) {
        Instance instance;
        instance.mesh = geometry;
        instance.transform = eigen(glm::scale(matrix, size));
        instance.tint << color, 1;
        instance.material = material;
        scene.instances.push_back(std::move(instance));
    };
    box(at(length / 2, width / 2, -depth - .12f), {length, width, .24f}, p.tile_color, SurfaceMaterial::Tiles);
    box(at(length / 2, -.15f, (deck - depth) / 2), {length, .3f, depth + deck}, p.tile_color, SurfaceMaterial::Tiles);
    box(at(length / 2, width + .15f, (deck - depth) / 2), {length, .3f, depth + deck}, p.tile_color,
        SurfaceMaterial::Tiles);
    box(at(-.15f, width / 2, (deck - depth) / 2), {.3f, width, depth + deck}, p.tile_color, SurfaceMaterial::Tiles);
    box(at(length + .15f, width / 2, (deck - depth) / 2), {.3f, width, depth + deck}, p.tile_color,
        SurfaceMaterial::Tiles);
    for (float y : {-1.5f, width + 1.5f})
        box(at(length / 2, y < 0 ? -1.8f : width + 1.8f, deck - .15f), {length + 6.6f, 3, .3f}, {.73f, .76f, .73f},
            SurfaceMaterial::Deck);
    for (float x : {-1.5f, length + 1.5f})
        box(at(x < 0 ? -1.8f : length + 1.8f, width / 2, deck - .15f), {3, width + .6f, .3f}, {.73f, .76f, .73f},
            SurfaceMaterial::Deck);
    for (float y : {-.10f, width + .10f})
        box(at(length / 2, y, deck + .02f), {length, .22f, .055f}, {.9f, .91f, .86f});
    for (float x : {-.10f, length + .10f})
        box(at(x, width / 2, deck + .02f), {.22f, width, .055f}, {.9f, .91f, .86f});
    for (bool floor : {true, false})
        if (auto mesh = stripeMesh(p.markings, floor, length, width, depth)) {
            Instance instance;
            instance.mesh = std::move(mesh);
            instance.transform = eigen(at(0, 0, 0));
            instance.material = SurfaceMaterial::Marking;
            instance.casts_shadow = false;
            scene.instances.push_back(std::move(instance));
        }
    auto surface = std::make_shared<MeshAsset>();
    Submesh mesh;
    mesh.vertices = {{{0, 0, 0}, {0, 0, 1}, {0, 0}},
                     {{length, 0, 0}, {0, 0, 1}, {1, 0}},
                     {{length, width, 0}, {0, 0, 1}, {1, 1}},
                     {{0, width, 0}, {0, 0, 1}, {0, 1}}};
    mesh.indices = {0, 1, 2, 0, 2, 3};
    surface->submeshes.push_back(std::move(mesh));
    surface->minimum.setZero();
    surface->maximum = {length, width, 0};
    WaterSurface water;
    water.surface.mesh = std::move(surface);
    water.surface.transform = eigen(at(0, 0, 0));
    water.dimensions = p.dimensions;
    water.tile_size = p.tile_size;
    water.waterline_band = p.waterline_band;
    water.waterline_color = p.waterline_color;
    water.level = p.water_level;
    water.local_to_world = p.local_to_world;
    scene.water = std::move(water);
    scene.lighting_center = (p.local_to_world * Eigen::Vector4f(length / 2, width / 2, p.water_level, 1)).head<3>();
    return scene;
}
} // namespace nereus::rendering

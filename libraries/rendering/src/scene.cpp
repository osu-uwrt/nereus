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
// Depth of one floor profile (see PoolGeometry::floor_profiles) at a position along its axis.
float profileDepth(const std::vector<Eigen::Vector2f> &profile, float s) {
    if (!(s > profile.front().x()))
        return profile.front().y();
    if (s >= profile.back().x())
        return profile.back().y();
    const auto upper = std::upper_bound(profile.begin(), profile.end(), s,
                                        [](float value, const Eigen::Vector2f &v) { return value < v.x(); });
    const Eigen::Vector2f &a = *(upper - 1), &b = *upper;
    return a.y() + (b.y() - a.y()) * (s - a.x()) / (b.x() - a.x());
}
// Depth of a profiled floor under pool-local (x, y): the shallowest profile there.
float floorDepth(const PoolGeometry &p, float x, float y) {
    float depth = std::numeric_limits<float>::max();
    for (const auto &profile : p.floor_profiles)
        depth = std::min(depth, profileDepth(profile.polyline, profile.along_x ? x : y));
    return depth;
}
// Up normal of a profiled floor from its depth slopes (z = -depth).
Eigen::Vector3f floorNormal(const PoolGeometry &p, float x, float y) {
    constexpr float h = .05f;
    const float dx = (floorDepth(p, x + h, y) - floorDepth(p, x - h, y)) / (2 * h);
    const float dy = (floorDepth(p, x, y + h) - floorDepth(p, x, y - h)) / (2 * h);
    return Eigen::Vector3f(dx, dy, 1).normalized();
}
// Grid lines of a profiled floor: the pool edges and every profile vertex, per axis.
std::pair<std::vector<float>, std::vector<float>> floorGrid(const PoolGeometry &p) {
    std::vector<float> xs{0, p.dimensions.x()}, ys{0, p.dimensions.y()};
    for (const auto &profile : p.floor_profiles)
        for (const auto &v : profile.polyline)
            (profile.along_x ? xs : ys).push_back(v.x());
    for (auto *axis : {&xs, &ys}) {
        std::sort(axis->begin(), axis->end());
        axis->erase(std::unique(axis->begin(), axis->end()), axis->end());
    }
    return {xs, ys};
}
// Floor mesh over a profiled floor, exact for its profiles' polylines. The grid runs through every profile
// vertex, so on each cell every profile is a plane; the cell splits into convex pieces, one per profile, where
// that profile is the shallowest. A crease between profiles then lies on piece edges instead of being cut
// across (which would lift the mesh above the floor there and bury the stripes draped on it).
std::shared_ptr<const MeshAsset> floorMesh(const PoolGeometry &p) {
    const auto [xs, ys] = floorGrid(p);
    struct Plane { // depth = a + bx * x + by * y over one cell
        float a, bx, by;
        float at(const Eigen::Vector2f &q) const {
            return a + bx * q.x() + by * q.y();
        }
    };
    auto result = std::make_shared<MeshAsset>();
    Submesh mesh;
    std::vector<Plane> planes(p.floor_profiles.size());
    for (std::size_t j = 0; j + 1 < ys.size(); ++j)
        for (std::size_t i = 0; i + 1 < xs.size(); ++i) {
            const float x0 = xs[i], x1 = xs[i + 1], y0 = ys[j], y1 = ys[j + 1];
            for (std::size_t k = 0; k < planes.size(); ++k) {
                const auto &profile = p.floor_profiles[k];
                const float s0 = profile.along_x ? x0 : y0, s1 = profile.along_x ? x1 : y1;
                const float d0 = profileDepth(profile.polyline, s0), d1 = profileDepth(profile.polyline, s1);
                const float slope = (d1 - d0) / (s1 - s0);
                planes[k] = profile.along_x ? Plane{d0 - slope * s0, slope, 0} : Plane{d0 - slope * s0, 0, slope};
            }
            for (std::size_t k = 0; k < planes.size(); ++k) {
                // Counter-clockwise seen from above; clipping keeps the order.
                std::vector<Eigen::Vector2f> piece{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
                for (std::size_t m = 0; m < planes.size() && piece.size() >= 3; ++m) {
                    if (m == k)
                        continue;
                    // Keep where profile k is shallower; ties go to the earlier profile.
                    const auto excess = [&](const Eigen::Vector2f &q) { return planes[k].at(q) - planes[m].at(q); };
                    const auto inside = [&](float f) { return m < k ? f < 0 : f <= 0; };
                    std::vector<Eigen::Vector2f> clipped;
                    for (std::size_t v = 0; v < piece.size(); ++v) {
                        const Eigen::Vector2f &a = piece[v], &b = piece[(v + 1) % piece.size()];
                        const float fa = excess(a), fb = excess(b);
                        if (inside(fa))
                            clipped.push_back(a);
                        if (inside(fa) != inside(fb))
                            clipped.push_back(a + (b - a) * (fa / (fa - fb)));
                    }
                    piece = std::move(clipped);
                }
                if (piece.size() < 3)
                    continue;
                const auto start = static_cast<std::uint32_t>(mesh.vertices.size());
                for (const auto &q : piece)
                    mesh.vertices.push_back({{q.x(), q.y(), -planes[k].at(q)}, floorNormal(p, q.x(), q.y()), q});
                for (std::uint32_t v = 1; v + 1 < piece.size(); ++v)
                    for (auto index : {start, start + v, start + v + 1})
                        mesh.indices.push_back(index);
            }
        }
    result->minimum = {0, 0, -p.dimensions.z()};
    result->maximum = {p.dimensions.x(), p.dimensions.y(), 0};
    for (const auto &v : mesh.vertices) {
        result->minimum = result->minimum.cwiseMin(v.position);
        result->maximum = result->maximum.cwiseMax(v.position);
    }
    result->submeshes.push_back(std::move(mesh));
    return result;
}
// Decal quads for one pool side, one submesh per colour, in pool-local coordinates relative to the water
// surface. Each quad extends kPad past its stripe so the Marking shader can fade the edge. Over a profiled
// floor (`profiled`), floor quads are split along their length and each vertex sits on the floor below it.
std::shared_ptr<const MeshAsset> stripeMesh(const std::vector<PoolStripe> &stripes, bool floor, float length,
                                            float width, float depth, const PoolGeometry *profiled = nullptr) {
    constexpr float kPad = .05f, kLift = .002f; // the lift keeps decals off the surface they lie on
    constexpr float kPiece = .1f;               // longest split piece over a profiled floor
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
        if (floor && profiled) {
            const int pieces = std::max(1, static_cast<int>(std::ceil(2 * (half + kPad) / kPiece)));
            for (int i = 0; i <= pieces; ++i)
                for (float side : {-1.f, 1.f}) {
                    const Eigen::Vector2f corner(-1 + 2.f * static_cast<float>(i) / static_cast<float>(pieces), side);
                    const Eigen::Vector2f q =
                        center + corner.x() * (half + kPad) * u + corner.y() * (halfWidth + kPad) * across;
                    const Eigen::Vector3f position(q.x(), q.y(), -floorDepth(*profiled, q.x(), q.y()) + kLift);
                    found->vertices.push_back(
                        {position, floorNormal(*profiled, q.x(), q.y()), corner.cwiseProduct(uv)});
                    result->minimum = result->minimum.cwiseMin(position);
                    result->maximum = result->maximum.cwiseMax(position);
                }
            for (int i = 0; i < pieces; ++i) {
                const auto a = start + static_cast<std::uint32_t>(2 * i);
                for (auto k : {a, a + 2, a + 3, a, a + 3, a + 1})
                    found->indices.push_back(k);
            }
            continue;
        }
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
    for (const auto &profile : p.floor_profiles) {
        const auto &line = profile.polyline;
        const float extent = profile.along_x ? p.dimensions.x() : p.dimensions.y();
        bool valid = line.size() >= 2 && line.front().x() == 0 &&
                     std::abs(line.back().x() - extent) <= 1e-4f * std::max(1.f, extent);
        for (std::size_t k = 0; valid && k < line.size(); ++k)
            valid = line[k].allFinite() && line[k].y() > 0 && line[k].y() <= p.dimensions.z() * (1 + 1e-5f) &&
                    (k == 0 || line[k].x() > line[k - 1].x());
        if (!valid)
            throw std::invalid_argument("pool floor profile must run from 0 to the pool extent with increasing "
                                        "positions and depths in (0, depth]");
    }
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
    const bool profiled = !p.floor_profiles.empty();
    if (!profiled) {
        box(at(length / 2, width / 2, -depth - .12f), {length, width, .24f}, p.tile_color, SurfaceMaterial::Tiles);
    } else {
        Instance instance;
        instance.mesh = floorMesh(p);
        instance.transform = eigen(at(0, 0, 0));
        instance.tint << p.tile_color, 1;
        instance.material = SurfaceMaterial::Tiles;
        scene.instances.push_back(std::move(instance));
    }
    // Each wall reaches the deepest floor along its foot (the floor hides any of it lying below).
    float yMin = depth, yMax = depth, xMin = depth, xMax = depth;
    if (profiled) {
        const auto [xs, ys] = floorGrid(p);
        yMin = yMax = xMin = xMax = 0;
        for (float x : xs)
            yMin = std::max(yMin, floorDepth(p, x, 0)), yMax = std::max(yMax, floorDepth(p, x, width));
        for (float y : ys)
            xMin = std::max(xMin, floorDepth(p, 0, y)), xMax = std::max(xMax, floorDepth(p, length, y));
    }
    box(at(length / 2, -.15f, (deck - yMin) / 2), {length, .3f, yMin + deck}, p.tile_color, SurfaceMaterial::Tiles);
    box(at(length / 2, width + .15f, (deck - yMax) / 2), {length, .3f, yMax + deck}, p.tile_color,
        SurfaceMaterial::Tiles);
    box(at(-.15f, width / 2, (deck - xMin) / 2), {.3f, width, xMin + deck}, p.tile_color, SurfaceMaterial::Tiles);
    box(at(length + .15f, width / 2, (deck - xMax) / 2), {.3f, width, xMax + deck}, p.tile_color,
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
        if (auto mesh = stripeMesh(p.markings, floor, length, width, depth, profiled ? &p : nullptr)) {
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

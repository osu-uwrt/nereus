#include "nereus/rendering/scene.hpp"
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <limits>
#include <stdexcept>
#include <utility>

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
// A box whose top face (top_l x top_w) is smaller than its base (l x w): four sides sloping in, centred on
// the origin, z from -h/2 to h/2. Top and sides are separate submeshes so they can differ in colour.
std::shared_ptr<const MeshAsset> frustumMesh(const Eigen::Vector3f &base, const Eigen::Vector2f &top,
                                             const Eigen::Vector3f &top_color, const Eigen::Vector3f &side_color) {
    const float bl = base.x() / 2, bw = base.y() / 2, tl = top.x() / 2, tw = top.y() / 2, h = base.z() / 2;
    const Eigen::Vector3f b[4] = {{-bl, -bw, -h}, {bl, -bw, -h}, {bl, bw, -h}, {-bl, bw, -h}};
    const Eigen::Vector3f t[4] = {{-tl, -tw, h}, {tl, -tw, h}, {tl, tw, h}, {-tl, tw, h}};
    auto result = std::make_shared<MeshAsset>();
    Submesh topFace, sides;
    topFace.material.base_color << top_color, 1;
    sides.material.base_color << side_color, 1;
    const auto quad = [](Submesh &mesh, const Eigen::Vector3f &p0, const Eigen::Vector3f &p1, const Eigen::Vector3f &p2,
                         const Eigen::Vector3f &p3) { // counter-clockwise from outside
        const Eigen::Vector3f n = (p1 - p0).cross(p2 - p0).normalized();
        const auto start = static_cast<std::uint32_t>(mesh.vertices.size());
        for (const auto &[p, uv] : {std::pair{p0, Eigen::Vector2f(0, 0)}, std::pair{p1, Eigen::Vector2f(1, 0)},
                                    std::pair{p2, Eigen::Vector2f(1, 1)}, std::pair{p3, Eigen::Vector2f(0, 1)}})
            mesh.vertices.push_back({p, n, uv});
        for (auto i : {0U, 1U, 2U, 0U, 2U, 3U})
            mesh.indices.push_back(start + i);
    };
    quad(topFace, t[0], t[1], t[2], t[3]);
    for (int k = 0; k < 4; ++k)
        quad(sides, b[k], b[(k + 1) % 4], t[(k + 1) % 4], t[k]);
    quad(sides, b[0], b[3], b[2], b[1]); // bottom
    result->submeshes = {std::move(topFace), std::move(sides)};
    result->minimum = -base / 2;
    result->maximum = base / 2;
    return result;
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
Scene makePoolScene(const PoolGeometry &p, PoolLayout *layout) {
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
    for (const auto &b : p.boxes)
        if (b.top && (!b.top->allFinite() || (b.top->array() <= 0).any() || b.top->x() > b.size.x() ||
                      b.top->y() > b.size.y() || (b.side_color && !unitColor(*b.side_color))))
            throw std::invalid_argument("a pool box's top must be positive and no larger than its base");
    for (const auto &b : p.boxes)
        if (!b.center.allFinite() || !b.size.allFinite() || (b.size.array() <= 0).any() || !b.rotation.allFinite() ||
            !(b.rotation.transpose() * b.rotation).isApprox(Eigen::Matrix3f::Identity(), 1e-4f) ||
            b.rotation.determinant() < 0 || !unitColor(b.color))
            throw std::invalid_argument(
                "pool boxes require a finite centre, a rotation, a positive size and a unit colour");
    for (const auto &r : p.recesses)
        if (r.side == PoolSide::Floor || !r.from.allFinite() || !r.to.allFinite() || r.from.x() == r.to.x() ||
            r.from.y() == r.to.y() || !std::isfinite(r.depth) || r.depth <= 0)
            throw std::invalid_argument("pool recesses need a wall, distinct finite corners and a positive depth");
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
    // Recesses cut the wall, deck and coping on their side. In a side's frame (along the wall, into it from
    // the pool face, z) a recess removes a box: its opening, `depth` into the wall, open through the deck when
    // it reaches the deck. A side without recesses keeps its boxes exactly; a cut box keeps its first piece in
    // place and the rest, with the recess linings, follow the coping strips.
    struct Piece {
        glm::mat4 matrix;
        glm::vec3 size;
        Eigen::Vector3f color;
        SurfaceMaterial material;
    };
    std::vector<Piece> extraWalls;
    using Aabb = std::pair<Eigen::Vector3f, Eigen::Vector3f>;
    const auto toSide = [&](PoolSide side, const Eigen::Vector3f &q) -> Eigen::Vector3f {
        switch (side) {
        case PoolSide::YMin:
            return {q.x(), -q.y(), q.z()};
        case PoolSide::YMax:
            return {q.x(), q.y() - width, q.z()};
        case PoolSide::XMin:
            return {q.y(), -q.x(), q.z()};
        default:
            return {q.y(), q.x() - length, q.z()};
        }
    };
    const auto fromSide = [&](PoolSide side, const Eigen::Vector3f &s) -> Eigen::Vector3f {
        switch (side) {
        case PoolSide::YMin:
            return {s.x(), -s.y(), s.z()};
        case PoolSide::YMax:
            return {s.x(), s.y() + width, s.z()};
        case PoolSide::XMin:
            return {-s.y(), s.x(), s.z()};
        default:
            return {s.y() + length, s.x(), s.z()};
        }
    };
    const auto convert = [](const auto &map, PoolSide side, const Aabb &b) -> Aabb {
        const Eigen::Vector3f a = map(side, b.first), c = map(side, b.second);
        return {a.cwiseMin(c), a.cwiseMax(c)};
    };
    const auto holes = [&](PoolSide side) {
        std::vector<Aabb> out;
        for (const auto &r : p.recesses)
            if (r.side == side) {
                const float z1 = std::max(r.from.y(), r.to.y());
                out.push_back({{std::min(r.from.x(), r.to.x()), -.05f, std::min(r.from.y(), r.to.y())},
                               {std::max(r.from.x(), r.to.x()), r.depth,
                                z1 >= deck ? std::numeric_limits<float>::infinity() : z1}});
            }
        return out;
    };
    const auto subtract = [](const std::vector<Aabb> &boxes, const Aabb &hole) {
        std::vector<Aabb> out;
        for (auto b : boxes) {
            if ((b.first.array() >= hole.second.array()).any() || (b.second.array() <= hole.first.array()).any()) {
                out.push_back(b);
                continue;
            }
            for (int axis = 0; axis < 3; ++axis) {
                if (b.first[axis] < hole.first[axis]) {
                    Aabb below = b;
                    below.second[axis] = hole.first[axis];
                    out.push_back(below);
                    b.first[axis] = hole.first[axis];
                }
                if (b.second[axis] > hole.second[axis]) {
                    Aabb above = b;
                    above.first[axis] = hole.second[axis];
                    out.push_back(above);
                    b.second[axis] = hole.second[axis];
                }
            } // what is left of b lies inside the hole
        }
        return out;
    };
    const auto sideBox = [&](PoolSide side, const Eigen::Vector3f &c, const Eigen::Vector3f &s,
                             const Eigen::Vector3f &color, SurfaceMaterial material) {
        const auto cuts = holes(side);
        if (cuts.empty()) {
            box(at(c.x(), c.y(), c.z()), {s.x(), s.y(), s.z()}, color, material);
            return;
        }
        std::vector<Aabb> pieces{convert(toSide, side, {c - s / 2, c + s / 2})};
        for (const auto &hole : cuts)
            pieces = subtract(pieces, hole);
        if (pieces.empty()) // keep the box's instance slot
            pieces.push_back({c, c + Eigen::Vector3f::Constant(1e-4f)});
        else
            for (auto &piece : pieces)
                piece = convert(fromSide, side, piece);
        for (std::size_t k = 0; k < pieces.size(); ++k) {
            const Eigen::Vector3f center = (pieces[k].first + pieces[k].second) / 2,
                                  size = pieces[k].second - pieces[k].first;
            const auto matrix = at(center.x(), center.y(), center.z());
            if (k == 0)
                box(matrix, {size.x(), size.y(), size.z()}, color, material);
            else
                extraWalls.push_back({matrix, {size.x(), size.y(), size.z()}, color, material});
        }
    };
    const Eigen::Vector3f deckColor(.73f, .76f, .73f), copingColor(.9f, .91f, .86f);
    sideBox(PoolSide::YMin, {length / 2, -.15f, (deck - yMin) / 2}, {length, .3f, yMin + deck}, p.tile_color,
            SurfaceMaterial::Tiles);
    sideBox(PoolSide::YMax, {length / 2, width + .15f, (deck - yMax) / 2}, {length, .3f, yMax + deck}, p.tile_color,
            SurfaceMaterial::Tiles);
    sideBox(PoolSide::XMin, {-.15f, width / 2, (deck - xMin) / 2}, {.3f, width, xMin + deck}, p.tile_color,
            SurfaceMaterial::Tiles);
    sideBox(PoolSide::XMax, {length + .15f, width / 2, (deck - xMax) / 2}, {.3f, width, xMax + deck}, p.tile_color,
            SurfaceMaterial::Tiles);
    for (float y : {-1.5f, width + 1.5f})
        sideBox(y < 0 ? PoolSide::YMin : PoolSide::YMax, {length / 2, y < 0 ? -1.8f : width + 1.8f, deck - .15f},
                {length + 6.6f, 3, .3f}, deckColor, SurfaceMaterial::Deck);
    for (float x : {-1.5f, length + 1.5f})
        sideBox(x < 0 ? PoolSide::XMin : PoolSide::XMax, {x < 0 ? -1.8f : length + 1.8f, width / 2, deck - .15f},
                {3, width + .6f, .3f}, deckColor, SurfaceMaterial::Deck);
    for (float y : {-.10f, width + .10f})
        sideBox(y < 0 ? PoolSide::YMin : PoolSide::YMax, {length / 2, y, deck + .02f}, {length, .22f, .055f},
                copingColor, SurfaceMaterial::Asset);
    for (float x : {-.10f, length + .10f})
        sideBox(x < 0 ? PoolSide::XMin : PoolSide::XMax, {x, width / 2, deck + .02f}, {.22f, width, .055f}, copingColor,
                SurfaceMaterial::Asset);
    // Line each recess: back, ends, sill and (below the deck) lintel, .3 m thick like the walls. The wall's own
    // .3 m (and the deck's) already bound the opening there, so the lining only covers what lies behind them:
    // overlapping a wall or deck face would make the two fight.
    for (const auto &r : p.recesses) {
        const float a0 = std::min(r.from.x(), r.to.x()), a1 = std::max(r.from.x(), r.to.x());
        const float z0 = std::min(r.from.y(), r.to.y()), z1 = std::max(r.from.y(), r.to.y()), d = r.depth;
        constexpr float wall = .3f;
        if (d <= wall)
            continue;                                           // the wall pieces line it entirely
        const float top = z1 >= deck ? deck - wall : z1 + wall; // under the deck slab when open through it
        const auto lining = [&](float b0, float b1, float n0, float n1, float c0, float c1) {
            const Aabb pool = convert(fromSide, r.side, {{b0, n0, c0}, {b1, n1, c1}});
            const Eigen::Vector3f center = (pool.first + pool.second) / 2, size = pool.second - pool.first;
            extraWalls.push_back({at(center.x(), center.y(), center.z()),
                                  {size.x(), size.y(), size.z()},
                                  p.tile_color,
                                  SurfaceMaterial::Tiles});
        };
        lining(a0 - wall, a1 + wall, d, d + wall, z0 - wall, top);
        lining(a0 - wall, a0, wall, d, z0 - wall, top);
        lining(a1, a1 + wall, wall, d, z0 - wall, top);
        lining(a0, a1, wall, d, z0 - wall, z0);
        if (z1 < deck)
            lining(a0, a1, wall, d, z1, z1 + wall);
    }
    PoolLayout groups;
    groups.floor = {0};
    for (std::size_t i = 1; i <= 12; ++i)
        groups.walls.push_back(i);
    // Wall pieces before the markings: a decal is drawn after the surface it lies on.
    for (const auto &piece : extraWalls) {
        groups.walls.push_back(scene.instances.size());
        box(piece.matrix, piece.size, piece.color, piece.material);
    }
    for (bool floor : {true, false})
        if (auto mesh = stripeMesh(p.markings, floor, length, width, depth, profiled ? &p : nullptr)) {
            Instance instance;
            instance.mesh = std::move(mesh);
            instance.transform = eigen(at(0, 0, 0));
            instance.material = SurfaceMaterial::Marking;
            instance.casts_shadow = false;
            (floor ? groups.floor : groups.walls).push_back(scene.instances.size());
            scene.instances.push_back(std::move(instance));
        }
    for (const auto &b : p.boxes) {
        (b.on_floor ? groups.floor : groups.walls).push_back(scene.instances.size());
        glm::mat4 rotation(1);
        for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 3; ++col)
                rotation[col][row] = b.rotation(row, col);
        const Eigen::Vector3f color = b.tiled ? p.tile_color : b.color;
        const auto material = b.tiled ? SurfaceMaterial::Tiles : SurfaceMaterial::Asset;
        if (!b.top) {
            box(at(b.center.x(), b.center.y(), b.center.z()) * rotation, {b.size.x(), b.size.y(), b.size.z()}, color,
                material);
            continue;
        }
        Instance instance;
        instance.mesh = frustumMesh(b.size, *b.top, color, b.side_color.value_or(color));
        instance.transform = eigen(at(b.center.x(), b.center.y(), b.center.z()) * rotation);
        instance.material = material;
        scene.instances.push_back(std::move(instance));
    }
    if (layout)
        *layout = std::move(groups);
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

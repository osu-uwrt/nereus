#include <nereus/pack_scene/pack_scene.hpp>
#include <nereus/session/pool.hpp>

#include <gtest/gtest.h>

#include <set>

namespace ps = nereus::pack_scene;
namespace rs = nereus::session;
namespace r = nereus::rendering;

namespace {
const rs::ResolvedScenario &talos() {
    static const auto resolved = rs::loadResolvedScenario(NEREUS_RESOLVED_TALOS);
    return resolved;
}
std::size_t texturedWith(const r::Scene &scene, const std::string &name) {
    std::size_t count = 0;
    for (const auto &instance : scene.instances)
        for (const auto &part : instance.mesh->submeshes)
            if (part.material.diffuse_texture && part.material.diffuse_texture->filename().string().find(name) == 0) {
                ++count;
                break;
            }
    return count;
}
} // namespace

TEST(PackScene, ComposesPoolTasksAndRobot) {
    ps::PackScene pack(talos());
    EXPECT_EQ(pack.poolInstanceCount(), 15u); // floor, 4 walls, 4 decks, 4 coping strips, floor/wall stripes
    EXPECT_GT(pack.staticScene().instances.size(), pack.poolInstanceCount());
    EXPECT_TRUE(pack.staticScene().water.has_value());
    EXPECT_EQ(pack.rootFrame(), "com");
    EXPECT_EQ(pack.robotVisuals().size(), talos().robot.at("visuals").size());
    for (const auto &visual : pack.robotVisuals())
        EXPECT_TRUE(visual.mesh);

    ps::Matrix4d root = ps::Matrix4d::Identity();
    root(0, 3) = 3;
    const auto scene = pack.compose(root);
    EXPECT_EQ(scene.instances.size(), pack.staticScene().instances.size() + pack.robotVisuals().size());
    const auto &first = scene.instances[pack.staticScene().instances.size()];
    const ps::Matrix4d expected = root * pack.robotVisuals()[0].rootFromAsset();
    EXPECT_LT((first.transform.cast<double>() - expected).cwiseAbs().maxCoeff(), 1e-6);
}

// The equipment pack's board is the last static instance, where the scenario hangs it (on the pool's y = 0 wall at
// the map origin, back face on the wall), with the board texture; a scenario without equipment has none.
TEST(PackScene, EquipmentVisualsAreLastInTheWorldFrame) {
    ps::PackScene pack(talos());
    ASSERT_EQ(pack.equipmentInstances(), (std::vector<std::size_t>{pack.staticScene().instances.size() - 1}));
    const auto &board = pack.staticScene().instances.back();
    const Eigen::Vector3f position = board.transform.topRightCorner<3, 1>();
    const Eigen::Matrix3f rotation = board.transform.topLeftCorner<3, 3>();
    EXPECT_LT((position - Eigen::Vector3f(0, 0, -.3394f)).norm(), 1e-6f);
    EXPECT_TRUE(rotation.isIdentity(1e-6f));
    EXPECT_EQ(texturedWith(pack.staticScene(), "calibration_board"), 1u);
    EXPECT_EQ(pack.describe().at("equipment_visuals"), 1);
    auto without = talos();
    without.equipment = nullptr;
    ps::PackScene bare(without);
    EXPECT_TRUE(bare.equipmentInstances().empty());
    EXPECT_EQ(bare.staticScene().instances.size(), pack.staticScene().instances.size() - 1);
}

TEST(PackScene, DynamicInstancesAndOverrides) {
    ps::PackScene pack(talos());
    ASSERT_FALSE(pack.propVisuals().empty()); // table props name display meshes
    std::vector<r::Instance> dynamic;
    ps::Matrix4d at = ps::Matrix4d::Identity();
    at(2, 3) = -1;
    dynamic.push_back(pack.instance("tasks", pack.propVisuals()[0].asset, at));
    ps::Matrix4d moved = ps::Matrix4d::Identity();
    moved(1, 3) = 0.25;
    const auto scene = pack.compose(ps::Matrix4d::Identity(), dynamic, {{1, moved}});
    ASSERT_EQ(scene.instances.size(),
              pack.staticScene().instances.size() + pack.robotVisuals().size() + dynamic.size());
    const auto base = pack.staticScene().instances.size();
    EXPECT_NEAR(scene.instances[base + 1].transform(1, 3), 0.25f, 1e-6f);
    EXPECT_NEAR(scene.instances.back().transform(2, 3), -1.f, 1e-6f);
    EXPECT_EQ(scene.instances.back().mesh, pack.propVisuals()[0].mesh); // one shared mesh per asset
}

TEST(PackScene, CutoutsAreRecorded) {
    ps::PackScene pack(talos());
    const auto record = pack.describe();
    ASSERT_TRUE(record.at("cutouts").is_array());
    ASSERT_FALSE(record.at("cutouts").empty());
    for (const auto &cutout : record.at("cutouts"))
        for (const auto &count : cutout.at("face_triangles"))
            EXPECT_GT(count.get<int>(), 0);
}

TEST(PackScene, BinsUseCrateClassTextures) {
    ps::PackScene pack(talos());
    EXPECT_EQ(texturedWith(pack.staticScene(), "Task3_Blood"), 2u);
    EXPECT_EQ(texturedWith(pack.staticScene(), "Task3_Fire"), 2u);
    EXPECT_EQ(pack.describe().at("texture_overrides").size(), 4u);
}

TEST(PackScene, TextureOverrideReplacesOnlyThatPlacement) {
    ps::PackScene pack(talos());
    const auto plain = pack.mesh("tasks", "bin_vinyl_mesh");
    const auto blood = pack.mesh("tasks", "bin_vinyl_mesh", "bin_vinyl_blood_texture");
    const auto fire = pack.mesh("tasks", "bin_vinyl_mesh", "bin_vinyl_fire_texture");
    ASSERT_TRUE(plain && blood && fire);
    EXPECT_NE(plain, blood);
    EXPECT_EQ(blood, pack.mesh("tasks", "bin_vinyl_mesh", "bin_vinyl_blood_texture")); // cached
    ASSERT_FALSE(blood->submeshes.empty());
    EXPECT_EQ(blood->submeshes[0].vertices.size(), plain->submeshes[0].vertices.size());
    const auto textures = [](const r::MeshAsset &mesh) {
        std::set<std::string> names;
        for (const auto &part : mesh.submeshes)
            if (part.material.diffuse_texture)
                names.insert(part.material.diffuse_texture->filename().string());
        return names;
    };
    EXPECT_EQ(textures(*plain).size(), 2u); // the shipped mesh carries both class textures
    EXPECT_EQ(textures(*blood), std::set<std::string>{"Task3_Blood_Fixed.png"});
    EXPECT_EQ(textures(*fire), std::set<std::string>{"Task3_Fire_Fixed.png"});
    EXPECT_THROW(pack.mesh("tasks", "bin_vinyl_mesh", "no_such_texture"), std::runtime_error);
}

TEST(PackScene, StrictRejectsMissingAssetsNonStrictWarns) {
    auto broken = talos();
    broken.asset_paths["tasks"].erase("bin_mesh");
    EXPECT_THROW(ps::PackScene{broken}, std::runtime_error);
    ps::PackScene tolerant(broken, {false});
    EXPECT_FALSE(tolerant.warnings().empty());
    EXPECT_GT(tolerant.staticScene().instances.size(), tolerant.poolInstanceCount());
}

TEST(PackScene, PoseHelpers) {
    const auto pose = ps::upright(rs::Json::array({1.0, 2.0, 3.0}), 90);
    const auto m = ps::toMatrix(pose);
    EXPECT_NEAR(m(0, 1), -1, 1e-12);
    EXPECT_NEAR(m(1, 0), 1, 1e-12);
    EXPECT_NEAR(m(2, 3), 3, 1e-12);
    EXPECT_THROW(ps::placement({{"position_m", {0, 0, 0}}, {"orientation_wxyz", {2, 0, 0, 0}}}), std::exception);
}

TEST(PackScene, TableTokenBodiesAreUntexturedDarkPlastic) {
    // The repaired pill/nut_and_bolt exports keep their body faces in the untextured Material.006; no textured
    // triangle may sit entirely at UV (0,0) (it would sample the vinyl's transparent corner and vanish).
    ps::PackScene pack(talos());
    for (const auto *asset : {"pill_visual", "nut_and_bolt_visual"}) {
        const auto mesh = pack.mesh("tasks", asset,
                                    std::string(asset) == "pill_visual" ? "pill_visual_task5_pill_fixed"
                                                                        : "nut_and_bolt_visual_task5_nutbolt_fixed");
        ASSERT_TRUE(mesh) << asset;
        std::size_t body = 0, top = 0;
        for (const auto &part : mesh->submeshes) {
            if (!part.material.diffuse_texture) {
                EXPECT_LT(part.material.base_color.head<3>().maxCoeff(), .01f) << asset;
                EXPECT_FLOAT_EQ(part.material.base_color.w(), 1.f) << asset;
                body += part.indices.size() / 3;
                continue;
            }
            for (std::size_t k = 0; k + 2 < part.indices.size(); k += 3) {
                bool unmapped = true;
                for (std::size_t c = 0; c < 3; ++c)
                    unmapped = unmapped && part.vertices[part.indices[k + c]].uv.squaredNorm() < 1e-14f;
                EXPECT_FALSE(unmapped) << asset << ": textured triangle at UV (0,0)";
                ++top;
            }
        }
        EXPECT_EQ(body, 260u) << asset;
        EXPECT_EQ(top, 36u) << asset;
    }
}

TEST(PackScene, RoboSubLaneGridExpandsToStripesOnFloorAndEndWalls) {
    ps::PackScene pack(talos());
    const auto &stripes = pack.poolStripes();
    std::size_t floor = 0, walls = 0;
    for (const auto &stripe : stripes)
        (stripe.side == r::PoolSide::Floor ? floor : walls)++;
    EXPECT_EQ(floor, (8u + 17u) * 3); // each line plus a T bar at both ends
    EXPECT_EQ(walls, (8u + 17u) * 2); // each line continues up both end walls
    // First along_x line: centred across the 22.86 m width, stopping 2 m short of the 50 m walls.
    EXPECT_NEAR(stripes[0].from.x(), 2, 1e-5);
    EXPECT_NEAR(stripes[0].to.x(), 48, 1e-5);
    EXPECT_NEAR(stripes[0].from.y(), (22.86 - 7 * 2.7432) / 2, 1e-5);
    EXPECT_NEAR(stripes[0].width, .254, 1e-6);
    // Its T bar at x = 2: 1 m across the line.
    EXPECT_NEAR(stripes[1].from.x(), 2, 1e-5);
    EXPECT_NEAR((stripes[1].to - stripes[1].from).norm(), 1, 1e-5);
    EXPECT_EQ(pack.poolFloorInstances(), (std::vector<std::size_t>{0, 13}));
    EXPECT_EQ(pack.poolWallInstances().back(), 14u);
    EXPECT_EQ(pack.poolWallInstances().size(), 13u);
    EXPECT_EQ(pack.describe().at("pool").at("stripes").get<std::size_t>(), stripes.size());
}

TEST(PackScene, LaneGridTargetsSitOnBothEndWallsAboveEachLine) {
    const auto pool = rs::Json::parse(R"({
        "parameters": {"length_m": 25, "width_m": 12, "depth_m": 2},
        "markings": {"lane_grid": {"along_x": {"count": 2, "spacing_m": 4},
                                   "targets": {"stem_m": [-1, 0], "bar_z_m": -0.5, "bar_length_m": 0.6,
                                               "width_m": 0.3, "color_rgb": [0, 0, 0]}}}
    })");
    std::vector<r::PoolStripe> walls;
    for (const auto &stripe : ps::poolStripes(pool))
        if (stripe.side != r::PoolSide::Floor)
            walls.push_back(stripe);
    ASSERT_EQ(walls.size(), 2u * 2 * 2); // 2 lines x 2 end walls x (stem, bar)
    EXPECT_EQ(walls[0].side, r::PoolSide::XMin);
    EXPECT_EQ(walls[4].side, r::PoolSide::XMax);
    EXPECT_EQ(walls[0].from, Eigen::Vector2f(4, -1)); // lines at y = 4 and 8
    EXPECT_EQ(walls[0].to, Eigen::Vector2f(4, 0));
    EXPECT_EQ(walls[1].from, Eigen::Vector2f(3.7f, -.5f));
    EXPECT_EQ(walls[1].to, Eigen::Vector2f(4.3f, -.5f));
    EXPECT_FLOAT_EQ(walls[1].width, .3f);
    EXPECT_EQ(walls[1].color, Eigen::Vector3f::Zero());
}

TEST(PackScene, IrregularLinesKeepTheirPlacementAndStyleOverrides) {
    const auto pool = rs::Json::parse(R"({
        "parameters": {"length_m": 25, "width_m": 12, "depth_m": 2},
        "markings": {
            "color_rgb": [0.1, 0.1, 0.1], "width_m": 0.2,
            "lane_grid": {"along_y": {"count": 2, "spacing_m": 3, "first_m": 4}},
            "lines": [
                {"from": [1, 1], "to": [7, 4], "ends": ["t", "none"], "t_length_m": 0.5},
                {"from": [10, 2.5], "to": [20, 2.5], "width_m": 0.4, "color_rgb": [1, 0, 0]}
            ],
            "wall_lines": [{"wall": "y_max", "from": [6, -2], "to": [6, -0.5], "ends": "t"}]
        }
    })");
    const auto stripes = ps::poolStripes(pool);
    ASSERT_EQ(stripes.size(), 2u + 2u + 1u + 3u);
    // Grid lines start at first_m, not centred.
    EXPECT_FLOAT_EQ(stripes[0].from.x(), 4);
    EXPECT_FLOAT_EQ(stripes[1].from.x(), 7);
    EXPECT_FLOAT_EQ(stripes[0].to.y(), 12);
    // A diagonal line with a T only at `from`, perpendicular to the line.
    const auto &diagonal = stripes[2], &bar = stripes[3];
    EXPECT_EQ(bar.from + bar.to, 2 * diagonal.from);
    EXPECT_NEAR((bar.to - bar.from).dot(diagonal.to - diagonal.from), 0, 1e-5);
    EXPECT_NEAR((bar.to - bar.from).norm(), .5, 1e-6);
    EXPECT_FLOAT_EQ(bar.width, .2f);
    // Per-line overrides.
    EXPECT_FLOAT_EQ(stripes[4].width, .4f);
    EXPECT_EQ(stripes[4].color, Eigen::Vector3f(1, 0, 0));
    EXPECT_EQ(stripes[0].color, Eigen::Vector3f(.1f, .1f, .1f));
    // A wall stripe with T ends at both ends.
    EXPECT_EQ(stripes[5].side, r::PoolSide::YMax);
    EXPECT_EQ(stripes[6].side, r::PoolSide::YMax);
    EXPECT_EQ(stripes[7].side, r::PoolSide::YMax);
    EXPECT_TRUE(ps::poolStripes(rs::Json::parse(R"({"parameters": {}})")).empty());
}

TEST(PackScene, ProfiledFloorDrapesStripesAndMeetsTheWalls) {
    // Deep (5 m) beside y = 0 at the x = 0 end, rising to 3 m along x and to 4 m across y.
    const auto pool = rs::Json::parse(R"({
        "parameters": {"length_m": 20, "width_m": 8, "depth_m": 5, "floor_profile": [
            {"along": "x", "points_m": [[0, 5], [8, 5], [14, 3], [20, 3]]},
            {"along": "y", "points_m": [[0, 5], [3, 5], [6, 4], [8, 4]]}]},
        "markings": {"lane_grid": {"along_x": {"count": 2, "spacing_m": 4}, "ends": "t", "on_walls": true}}
    })");
    const auto floor = rs::poolFloor(pool);
    const auto depth = [&](const Eigen::Vector3f &v) { return floor.depthAt(Eigen::Vector2d(v.x(), v.y())); };
    const auto stripes = ps::poolStripes(pool);
    // Wall stripes run from the floor where they meet their wall: lines at y = 2 and y = 6.
    std::size_t walls = 0;
    for (const auto &stripe : stripes) {
        if (stripe.side == r::PoolSide::XMin) {
            EXPECT_FLOAT_EQ(stripe.from.y(), stripe.from.x() < 4 ? -5 : -4);
            ++walls;
        }
        if (stripe.side == r::PoolSide::XMax) {
            EXPECT_FLOAT_EQ(stripe.from.y(), -3);
            ++walls;
        }
    }
    EXPECT_EQ(walls, 4u);
    r::PoolGeometry geometry;
    geometry.dimensions = {20, 8, 5};
    geometry.markings = stripes;
    for (const auto &profile : floor.profiles()) {
        r::FloorSlope slope;
        slope.along_x = profile.axis() == nereus::simulation::FloorProfile::Axis::X;
        for (const auto &vertex : profile.polyline())
            slope.polyline.push_back(vertex.cast<float>());
        geometry.floor_profiles.push_back(slope);
    }
    const auto scene = r::makePoolScene(geometry);
    // Floor mesh: exactly the shallower profile everywhere, creases included. Every vertex is on the floor,
    // and so is every point inside every triangle (a mesh cutting across a crease would sit above it).
    const auto &floorMesh = scene.instances[0].mesh->submeshes.at(0);
    for (const auto &v : floorMesh.vertices) {
        EXPECT_NEAR(v.position.z(), -depth(v.position), 1e-5);
        EXPECT_GT(v.normal.z(), .85); // this test floor peaks near 27 degrees
    }
    double worst = 0;
    for (std::size_t t = 0; t + 2 < floorMesh.indices.size(); t += 3)
        for (const Eigen::Vector3f &weights :
             {Eigen::Vector3f(1.f / 3, 1.f / 3, 1.f / 3), Eigen::Vector3f(.6f, .2f, .2f),
              Eigen::Vector3f(.2f, .6f, .2f), Eigen::Vector3f(.2f, .2f, .6f)}) {
            Eigen::Vector3f q = Eigen::Vector3f::Zero();
            for (int c = 0; c < 3; ++c)
                q += weights[c] * floorMesh.vertices[floorMesh.indices[t + c]].position;
            worst = std::max(worst, std::abs(q.z() + depth(q)));
        }
    EXPECT_LT(worst, 1e-4);
    // Each wall reaches the deepest floor along its foot, plus the deck.
    const auto height = [&](std::size_t i) { return scene.instances[i].transform.col(2).head<3>().norm(); };
    EXPECT_NEAR(height(1), 5 + geometry.deck_height, 1e-5); // y = 0
    EXPECT_NEAR(height(2), 4 + geometry.deck_height, 1e-5); // y = 8
    EXPECT_NEAR(height(3), 5 + geometry.deck_height, 1e-5); // x = 0
    EXPECT_NEAR(height(4), 3 + geometry.deck_height, 1e-5); // x = 20
    // Floor stripes drape over both slopes, 2 mm above the floor.
    const auto &floorStripes = scene.instances[13];
    ASSERT_EQ(floorStripes.material, r::SurfaceMaterial::Marking);
    std::size_t onSlope = 0;
    for (const auto &part : floorStripes.mesh->submeshes)
        for (const auto &v : part.vertices) {
            EXPECT_NEAR(v.position.z(), -depth(v.position) + .002, 1e-4);
            onSlope += v.position.x() > 8 && v.position.x() < 14;
        }
    EXPECT_GT(onSlope, 2u * 2 * 50);
    // A profile that does not span the pool is rejected.
    geometry.floor_profiles[1].polyline.back().x() = 7;
    EXPECT_THROW(r::makePoolScene(geometry), std::invalid_argument);
}

TEST(PackScene, RpacDiveWellBuildsItsSlopedFloor) {
    const auto resolved = rs::loadResolvedScenario(NEREUS_RESOLVED_RPAC);
    ps::PackScene pack(resolved);
    const auto floor = rs::poolFloor(resolved.pool);
    const auto &mesh = pack.staticScene().instances.at(0).mesh->submeshes.at(0);
    float deepest = 0, shallowest = 100;
    for (const auto &v : mesh.vertices) {
        deepest = std::max(deepest, -v.position.z());
        shallowest = std::min(shallowest, -v.position.z());
    }
    EXPECT_NEAR(deepest, 5.1816, 1e-4); // 17 ft
    // The shallow end's far corner.
    const auto &dimensions = resolved.pool.at("parameters");
    EXPECT_NEAR(shallowest,
                floor.depthAt({dimensions.at("length_m").get<double>(), dimensions.at("width_m").get<double>()}), 1e-4);
    EXPECT_LT(shallowest, 4.6);
    EXPECT_TRUE(pack.describe().at("pool").contains("floor_profile"));
}

TEST(PackScene, RecessesOpenTheirWallAndBoxesJoinTheirGroup) {
    r::PoolGeometry geometry;
    geometry.dimensions = {20, 8, 4};
    geometry.deck_height = .3f;
    r::PoolRecess recess;
    recess.side = r::PoolSide::YMin; // the y = 0 wall
    recess.from = {15, -1.2f};
    recess.to = {16, .3f}; // up through the deck
    recess.depth = .5f;
    geometry.recesses = {recess};
    r::PoolBox grate;
    grate.center = {5, 2, -3.98f};
    grate.size = {1.2f, .6f, .04f};
    grate.on_floor = true;
    grate.top = Eigen::Vector2f(1.f, .4f); // sloped sides
    grate.side_color = Eigen::Vector3f(.3f, .4f, .4f);
    r::PoolBox tread;
    tread.center = {15.5f, -.25f, -.6f};
    tread.size = {1, .5f, .05f};
    geometry.boxes = {grate, tread};
    r::PoolStripe band; // across the wall either side of the recess
    band.side = r::PoolSide::YMin;
    band.from = {0, -1.5f};
    band.to = {20, -1.5f};
    geometry.markings = {band};
    r::PoolLayout layout;
    const auto scene = r::makePoolScene(geometry, &layout);
    // Nothing of the y = 0 wall's tiles is left inside the opening.
    const auto inside = [&](const r::Instance &instance) {
        const Eigen::Vector3f c = instance.transform.col(3).head<3>();
        const Eigen::Vector3f half =
            Eigen::Vector3f(instance.transform.col(0).head<3>().norm(), instance.transform.col(1).head<3>().norm(),
                            instance.transform.col(2).head<3>().norm()) /
            2;
        // Overlap with the opening: x 15..16, y -0.49..0 (in front of the recess back), z -1.2..0.3.
        return c.x() - half.x() < 16 - 1e-4f && c.x() + half.x() > 15 + 1e-4f && c.y() - half.y() < -1e-4f &&
               c.y() + half.y() > -.49f && c.z() - half.z() < .3f - 1e-4f && c.z() + half.z() > -1.2f + 1e-4f;
    };
    // Nor of its deck or coping: a recess reaching the deck is an open stair well.
    std::size_t lining = 0;
    for (const auto i : layout.walls) {
        const auto &instance = scene.instances[i];
        if (instance.material == r::SurfaceMaterial::Marking || i + 1 == scene.instances.size()) // the tread
            continue;
        EXPECT_FALSE(inside(instance)) << "wall, deck or coping instance " << i;
        lining += i > 12 && instance.material == r::SurfaceMaterial::Tiles;
    }
    EXPECT_GE(lining, 4u); // the wall's other pieces and the lining
    // No two tiled wall pieces share volume: overlapping faces would fight (the lining showing through the wall).
    std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> tiled;
    for (const auto i : layout.walls) {
        const auto &instance = scene.instances[i];
        if (instance.material != r::SurfaceMaterial::Tiles)
            continue;
        const Eigen::Vector3f c = instance.transform.col(3).head<3>();
        const Eigen::Vector3f half(instance.transform.col(0).head<3>().norm() / 2,
                                   instance.transform.col(1).head<3>().norm() / 2,
                                   instance.transform.col(2).head<3>().norm() / 2);
        tiled.emplace_back(c - half, c + half);
    }
    for (std::size_t a = 0; a < tiled.size(); ++a)
        for (std::size_t b = a + 1; b < tiled.size(); ++b) {
            const Eigen::Vector3f overlap =
                (tiled[a].second.cwiseMin(tiled[b].second) - tiled[a].first.cwiseMax(tiled[b].first)).cwiseMax(0);
            EXPECT_LT(overlap.prod(), 1e-6f) << "tiled pieces " << a << " and " << b << " overlap";
        }
    // Every wall piece comes before the wall markings, which are drawn after the surface they lie on.
    std::size_t markings = 0;
    for (std::size_t i = 0; i < scene.instances.size(); ++i)
        if (scene.instances[i].material == r::SurfaceMaterial::Marking)
            markings = i;
    for (const auto i : layout.walls)
        if (scene.instances[i].material == r::SurfaceMaterial::Tiles) {
            EXPECT_LT(i, markings);
        }
    // The tread sits in the opening, grouped with the walls; the grate with the floor.
    EXPECT_EQ(layout.floor.back(), scene.instances.size() - 2);
    EXPECT_EQ(layout.walls.back(), scene.instances.size() - 1);
    EXPECT_TRUE(inside(scene.instances.back()));
    // The grate is a sloped-sided box: its top face is the smaller one, its sides their own colour.
    const auto &grateMesh = *scene.instances[layout.floor.back()].mesh;
    ASSERT_EQ(grateMesh.submeshes.size(), 2u);
    for (const auto &v : grateMesh.submeshes[0].vertices) {
        EXPECT_NEAR(std::abs(v.position.x()), .5f, 1e-6);
        EXPECT_NEAR(v.position.z(), .02f, 1e-6);
        EXPECT_NEAR(v.normal.z(), 1, 1e-6);
    }
    EXPECT_EQ(grateMesh.submeshes[1].material.base_color.head<3>(), Eigen::Vector3f(.3f, .4f, .4f));
    for (const auto &v : grateMesh.submeshes[1].vertices) {
        if (v.normal.z() < -.99f)
            continue;               // the bottom
        EXPECT_GT(v.normal.z(), 0); // a side sloping in, so facing out and up
        EXPECT_LT(v.normal.z(), .99f);
        EXPECT_GT(v.normal.head<2>().dot(v.position.head<2>()), 0);
    }
    // Without recesses or boxes the scene keeps its usual 13 boxes.
    EXPECT_EQ(r::makePoolScene({}).instances.size(), 13u);
    geometry.recesses[0].depth = 0;
    EXPECT_THROW(r::makePoolScene(geometry), std::invalid_argument);
}

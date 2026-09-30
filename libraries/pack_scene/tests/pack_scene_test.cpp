#include <nereus/pack_scene/pack_scene.hpp>

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

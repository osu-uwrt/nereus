#include <robotics/pack_scene/pack_scene.hpp>

#include <gtest/gtest.h>

#include <set>

namespace ps = robotics::pack_scene;
namespace rs = robotics::session;
namespace r = robotics::rendering;

namespace {
const rs::ResolvedScenario &talos() {
    static const auto resolved = rs::loadResolvedScenario(RP_RESOLVED_TALOS);
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
    EXPECT_EQ(pack.poolInstanceCount(), 13u); // floor, 4 walls, 4 decks, 4 coping strips
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

TEST(PackScene, UnmappedUvFacesAreDrawnInTheDeclaredColor) {
    ps::PackScene pack(talos());
    const auto raw = r::loadMesh(talos().asset("tasks", "pill_visual"));
    std::size_t rawTriangles = 0, rawUnmapped = 0;
    for (const auto &part : raw.submeshes)
        for (std::size_t k = 0; k + 2 < part.indices.size(); k += 3) {
            ++rawTriangles;
            bool all = part.material.diffuse_texture.has_value();
            for (std::size_t c = 0; c < 3; ++c)
                all = all && part.vertices[part.indices[k + c]].uv.squaredNorm() < 1e-14f;
            rawUnmapped += all;
        }
    ASSERT_GT(rawUnmapped, 0u) << "the shipped pill mesh has unmapped body faces";
    for (const auto *asset : {"pill_visual", "nut_and_bolt_visual"}) {
        const auto repaired = pack.mesh("tasks", asset);
        ASSERT_TRUE(repaired) << asset;
        std::size_t body = 0, triangles = 0;
        for (const auto &part : repaired->submeshes) {
            triangles += part.indices.size() / 3;
            if (!part.material.diffuse_texture && part.material.base_color.isApprox(Eigen::Vector4f(.003442196f, .003442196f, .003442196f, 1)))
                body += part.indices.size() / 3;
        }
        EXPECT_GT(body, 0u) << asset << ": dark-plastic body submesh";
        if (std::string(asset) == "pill_visual") {
            EXPECT_EQ(body, rawUnmapped);
            EXPECT_EQ(triangles, rawTriangles); // nothing lost, nothing duplicated
        }
    }
    // Assets without the repair are unchanged (bandage has no unmapped_uv_color).
    const auto bandage = pack.mesh("tasks", "bandage_visual");
    const auto rawBandage = r::loadMesh(talos().asset("tasks", "bandage_visual"));
    ASSERT_TRUE(bandage);
    EXPECT_EQ(bandage->submeshes.size(), rawBandage.submeshes.size());
}

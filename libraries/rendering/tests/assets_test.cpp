#include "robotics/rendering/assets.hpp"
#include <fstream>
#include <functional>
#include <gtest/gtest.h>
#include <limits>
#include <sstream>

using namespace robotics::rendering;
TEST(MeshAssets, OriginalTalosBodyAndRotorsKeepTriangleCountsAndTransparency) {
    const std::filesystem::path root = std::filesystem::path(NEREUS_PACK_CONTENT) / "robots/talos/assets/visual";
    const auto body = loadMesh(root / "Talos3_body.glb");
    std::size_t triangles = 0, transparent = 0;
    for (const auto &part : body.submeshes) {
        triangles += part.indices.size() / 3;
        transparent += part.material.base_color.w() < 1;
        EXPECT_FALSE(part.material.diffuse_texture);
    }
    EXPECT_EQ(triangles, 1053293U);
    EXPECT_GT(transparent, 0U);
    std::size_t rotor_triangles = 0;
    for (const auto *id : {"VUS", "VUP", "HUS", "HUP", "HLS", "HLP", "VLS", "VLP"}) {
        const auto rotor = loadMesh(root / "rotors" / (std::string(id) + ".glb"));
        for (const auto &part : rotor.submeshes)
            rotor_triangles += part.indices.size() / 3;
    }
    EXPECT_EQ(rotor_triangles, 33937U);
}
TEST(MeshAssets, RejectsMissingMalformedAndOversizedAssets) {
    EXPECT_THROW(loadMesh("/does/not/exist.glb"), std::runtime_error);
    EXPECT_THROW(loadMesh(std::filesystem::path(NEREUS_PACK_CONTENT) / "robots/talos/robot.yaml"), std::runtime_error);
    const auto body = std::filesystem::path(NEREUS_PACK_CONTENT) / "robots/talos/assets/visual/Talos3_body.glb";
    AssetLimits limits;
    limits.file_bytes = 1;
    EXPECT_THROW(loadMesh(body, limits), std::runtime_error);
    limits.file_bytes = 0;
    EXPECT_THROW(loadMesh(body, limits), std::invalid_argument);
    limits = {};
    limits.vertices = 1;
    EXPECT_THROW(loadMesh(body, limits), std::runtime_error);
}

TEST(MeshAssets, NestedTransformsUseAuthoredAxesAndInverseTransposeNormals) {
    const auto asset = loadMesh(std::filesystem::path(NEREUS_ASSET_FIXTURES) / "nested_mesh.gltf");
    ASSERT_EQ(asset.submeshes.size(), 2U);
    const auto &part = asset.submeshes[0];
    ASSERT_EQ(part.vertices.size(), 3U);
    EXPECT_TRUE(part.vertices[0].position.isApprox(Eigen::Vector3f(1, 2, 3), 1e-6f));
    EXPECT_TRUE(part.vertices[1].position.isApprox(Eigen::Vector3f(4, 4, 3), 1e-6f));
    EXPECT_TRUE(part.vertices[2].position.isApprox(Eigen::Vector3f(1, 2, 7), 1e-6f));
    const Eigen::Vector3f normal = Eigen::Vector3f(-2, 3, 0).normalized();
    for (const auto &vertex : part.vertices)
        EXPECT_TRUE(vertex.normal.isApprox(normal, 1e-6f));
    // Assimp converts glTF's texture V convention, as in the original loading path.
    EXPECT_EQ(part.vertices[1].uv, Eigen::Vector2f(1, 1));
    EXPECT_EQ(part.vertices[2].uv, Eigen::Vector2f(0, 0));
    EXPECT_TRUE(part.material.base_color.isApprox(Eigen::Vector4f(.2f, .3f, .4f, .25f)));
    EXPECT_TRUE(asset.submeshes[1].vertices[0].position.isApprox(Eigen::Vector3f(1, 12, 3), 1e-6f));
    EXPECT_TRUE(asset.minimum.isApprox(Eigen::Vector3f(1, 2, 3), 1e-6f));
    EXPECT_TRUE(asset.maximum.isApprox(Eigen::Vector3f(4, 13, 7), 1e-6f));
    AssetLimits limits;
    limits.nodes = 1;
    EXPECT_THROW(loadMesh(std::filesystem::path(NEREUS_ASSET_FIXTURES) / "nested_mesh.gltf", limits),
                 std::runtime_error);
}

TEST(MeshAssets, EveryOriginalSubmeshKeepsOrderTopologyMaterialsAndVertexStatistics) {
    std::ifstream reference(std::filesystem::path(NEREUS_ASSET_FIXTURES) / "legacy_mesh_assets.csv");
    ASSERT_TRUE(reference);
    std::string line, previous;
    ASSERT_TRUE(std::getline(reference, line));
    MeshAsset asset;
    std::size_t part_count = 0, file_count = 0;
    while (std::getline(reference, line)) {
        std::istringstream stream(line);
        std::vector<std::string> fields;
        std::string field;
        while (std::getline(stream, field, ','))
            fields.push_back(field);
        ASSERT_EQ(fields.size(), 23U);
        if (fields[0] != previous) {
            if (!previous.empty()) {
                EXPECT_EQ(part_count, asset.submeshes.size());
            }
            const auto root = std::filesystem::path(NEREUS_PACK_CONTENT) / "robots/talos/assets/visual";
            asset = loadMesh(fields[0] == "Talos3_body.glb" ? root / fields[0] : root / "rotors" / fields[0]);
            previous = fields[0];
            part_count = 0;
            ++file_count;
        }
        ASSERT_EQ(std::stoull(fields[1]), part_count);
        ASSERT_LT(part_count, asset.submeshes.size());
        const auto &part = asset.submeshes[part_count++];
        ASSERT_EQ(part.vertices.size(), std::stoull(fields[2]));
        ASSERT_EQ(part.indices.size(), std::stoull(fields[3]));
        Eigen::Vector3f minimum = part.vertices.front().position, maximum = minimum;
        Eigen::Vector3d position = Eigen::Vector3d::Zero(), normal = position;
        Eigen::Vector2d uv = Eigen::Vector2d::Zero();
        for (const auto &v : part.vertices) {
            minimum = minimum.cwiseMin(v.position);
            maximum = maximum.cwiseMax(v.position);
            position += v.position.cast<double>();
            normal += v.normal.cast<double>();
            uv += v.uv.cast<double>();
        }
        position /= static_cast<double>(part.vertices.size());
        normal /= static_cast<double>(part.vertices.size());
        uv /= static_cast<double>(part.vertices.size());
        Eigen::Matrix<double, 18, 1> actual;
        actual << part.material.base_color.cast<double>(), minimum.cast<double>(), maximum.cast<double>(), position,
            normal, uv;
        ASSERT_TRUE(actual.allFinite());
        for (Eigen::Index i = 0; i < actual.size(); ++i)
            EXPECT_NEAR(actual[i], std::stod(fields[i + 4]), 1e-6)
                << previous << " part " << part_count - 1 << " field " << i;
        std::uint64_t hash = 14695981039346656037ULL;
        for (auto index : part.indices)
            hash = (hash ^ index) * 1099511628211ULL;
        EXPECT_EQ(hash, std::stoull(fields[22]));
    }
    EXPECT_EQ(part_count, asset.submeshes.size());
    EXPECT_EQ(file_count, 9U);
}

namespace {
Vertex vertex(float x, float y, float z, Eigen::Vector2f uv = {-1, -1}) {
    return {{x, y, z}, {1, 0, 0}, uv};
}
Eigen::Vector2f planar(float y, float z, float half = 1) {
    return {y / (2 * half) + 0.5f, z / (2 * half) + 0.5f};
}
PanelCutouts panelOptions() {
    PanelCutouts panel;
    panel.faces_x = {0.f};
    panel.half_size = 1;
    panel.cutouts = {{{0.25f, 0.5f}, 0.1f}, {{0.75f, 0.5f}, 0.2f}};
    return panel;
}
// Triangle 0 lies on the panel plane, triangle 1 is 0.1 m in front of it.
Submesh mixedSubmesh() {
    Submesh part;
    part.vertices = {vertex(0, -.5f, -.5f),   vertex(0, .5f, -.5f),   vertex(0, 0, .5f),
                     vertex(.1f, -.5f, -.5f), vertex(.1f, .5f, -.5f), vertex(.1f, 0, .5f)};
    part.indices = {0, 1, 2, 3, 4, 5};
    part.material.base_color = {.1f, .2f, .3f, 1};
    return part;
}
MeshAsset asset(std::vector<Submesh> parts) {
    MeshAsset result;
    result.submeshes = std::move(parts);
    result.minimum = {-1, -2, -3};
    result.maximum = {4, 5, 6};
    return result;
}
} // namespace

TEST(PerforatePanel, MixedSubmeshSplitsRemainderFirstAndCopiesPanelVertices) {
    Submesh other = mixedSubmesh();
    other.indices = {3, 4, 5};
    const auto result = perforatePanel(asset({other, mixedSubmesh()}), panelOptions());
    ASSERT_EQ(result.face_triangles, std::vector<std::size_t>({1}));
    ASSERT_EQ(result.mesh.submeshes.size(), 3U);
    EXPECT_EQ(result.mesh.submeshes[0].indices, other.indices);
    EXPECT_TRUE(result.mesh.submeshes[0].material.cutouts.empty());
    const auto &remainder = result.mesh.submeshes[1];
    EXPECT_EQ(remainder.vertices.size(), 6U);
    EXPECT_EQ(remainder.indices, std::vector<std::uint32_t>({3, 4, 5}));
    EXPECT_TRUE(remainder.material.cutouts.empty());
    EXPECT_EQ(remainder.vertices[0].uv, Eigen::Vector2f(-1, -1));
    const auto &panel = result.mesh.submeshes[2];
    ASSERT_EQ(panel.vertices.size(), 3U);
    EXPECT_EQ(panel.indices, std::vector<std::uint32_t>({0, 1, 2}));
    EXPECT_TRUE(panel.material.base_color.isApprox(remainder.material.base_color));
    ASSERT_EQ(panel.material.cutouts.size(), 2U);
    EXPECT_EQ(panel.material.cutouts[1].center, Eigen::Vector2f(0.75f, 0.5f));
    EXPECT_EQ(panel.material.cutouts[1].radius, 0.2f);
    const auto source = mixedSubmesh();
    for (std::size_t i = 0; i < 3; ++i) {
        EXPECT_EQ(panel.vertices[i].position, source.vertices[i].position);
        EXPECT_EQ(panel.vertices[i].normal, source.vertices[i].normal);
        EXPECT_TRUE(
            panel.vertices[i].uv.isApprox(planar(source.vertices[i].position.y(), source.vertices[i].position.z())));
    }
    EXPECT_EQ(result.mesh.minimum, Eigen::Vector3f(-1, -2, -3));
    EXPECT_EQ(result.mesh.maximum, Eigen::Vector3f(4, 5, 6));
}

TEST(PerforatePanel, UntexturedPanelSubmeshGetsPlanarUvInPlace) {
    Submesh part = mixedSubmesh();
    part.indices = {0, 1, 2};
    const auto result = perforatePanel(asset({part}), panelOptions());
    ASSERT_EQ(result.mesh.submeshes.size(), 1U);
    const auto &out = result.mesh.submeshes[0];
    ASSERT_EQ(out.vertices.size(), 6U);
    EXPECT_EQ(out.indices, part.indices);
    EXPECT_EQ(out.material.cutouts.size(), 2U);
    for (std::size_t i = 0; i < 3; ++i)
        EXPECT_TRUE(out.vertices[i].uv.isApprox(planar(part.vertices[i].position.y(), part.vertices[i].position.z())));
    EXPECT_EQ(out.vertices[3].uv, Eigen::Vector2f(-1, -1));
}

TEST(PerforatePanel, TexturedPanelKeepsAuthoredUvAndRejectsMismatch) {
    Submesh part = mixedSubmesh();
    part.indices = {0, 1, 2};
    part.material.diffuse_texture = "texture.png";
    for (std::size_t i = 0; i < 3; ++i)
        part.vertices[i].uv =
            planar(part.vertices[i].position.y(), part.vertices[i].position.z()) + Eigen::Vector2f(5e-5f, -5e-5f);
    const auto result = perforatePanel(asset({part}), panelOptions());
    ASSERT_EQ(result.mesh.submeshes.size(), 1U);
    for (std::size_t i = 0; i < 3; ++i)
        EXPECT_EQ(result.mesh.submeshes[0].vertices[i].uv, part.vertices[i].uv);
    EXPECT_TRUE(result.mesh.submeshes[0].material.diffuse_texture);
    EXPECT_EQ(result.mesh.submeshes[0].material.cutouts.size(), 2U);

    part.vertices[1].uv.x() += 2e-4f;
    EXPECT_THROW(perforatePanel(asset({part}), panelOptions()), std::invalid_argument);
    Submesh mixed = mixedSubmesh();
    mixed.material.diffuse_texture = "texture.png";
    try {
        perforatePanel(asset({mixed}), panelOptions());
        FAIL();
    } catch (const std::invalid_argument &error) {
        EXPECT_STREQ(error.what(), "textured panel face UVs differ from the panel mapping");
    }
    // A mismatched vertex that no panel triangle references is not checked.
    part.vertices[1].uv.x() -= 2e-4f;
    part.vertices[4].uv = {9, 9};
    EXPECT_NO_THROW(perforatePanel(asset({part}), panelOptions()));
}

TEST(PerforatePanel, RejectsExistingCutoutsAndSelectsFirstMatchingFaceOnly) {
    Submesh part = mixedSubmesh();
    part.material.cutouts = {{{.5f, .5f}, .1f}};
    EXPECT_THROW(perforatePanel(asset({part}), panelOptions()), std::invalid_argument);
    // Faces at 0 and 0.0004 both match the triangle; only the first counts it.
    auto panel = panelOptions();
    panel.faces_x = {0.f, 4e-4f, 0.1f};
    const auto result = perforatePanel(asset({mixedSubmesh()}), panel);
    EXPECT_EQ(result.face_triangles, std::vector<std::size_t>({1, 0, 1}));
    EXPECT_EQ(result.mesh.submeshes.size(), 1U);
}

TEST(PerforatePanel, IgnoresTrianglesOutsideExtentOrOffFace) {
    Submesh part;
    part.vertices = {vertex(0, -.5f, -.5f), vertex(0, .5f, -.5f),   vertex(0, 0, .5f),     vertex(0, 0, 0),
                     vertex(0, 1.0004f, 0), vertex(0, 0, .5f),      vertex(0, 0, 0),       vertex(0, 1.001f, 0),
                     vertex(0, 0, .5f),     vertex(6e-4f, 0, 0),    vertex(0, .5f, 0),     vertex(0, 0, .5f),
                     vertex(4e-4f, 0, 0),   vertex(-4e-4f, .5f, 0), vertex(0, 0, -1.0004f)};
    part.indices = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
    const auto result = perforatePanel(asset({part}), panelOptions());
    // Triangles 0, 1 (within tolerance of the extent) and 4 (within tolerance of the plane).
    EXPECT_EQ(result.face_triangles, std::vector<std::size_t>({3}));
    ASSERT_EQ(result.mesh.submeshes.size(), 2U);
    EXPECT_EQ(result.mesh.submeshes[0].indices, std::vector<std::uint32_t>({6, 7, 8, 9, 10, 11}));
    EXPECT_EQ(result.mesh.submeshes[1].vertices.size(), 9U);

    Submesh far = mixedSubmesh();
    far.indices = {3, 4, 5};
    const auto untouched = perforatePanel(asset({far}), panelOptions());
    ASSERT_EQ(untouched.mesh.submeshes.size(), 1U);
    EXPECT_TRUE(untouched.mesh.submeshes[0].material.cutouts.empty());
    EXPECT_EQ(untouched.mesh.submeshes[0].vertices[0].uv, Eigen::Vector2f(-1, -1));
    EXPECT_EQ(untouched.face_triangles, std::vector<std::size_t>({0}));
}

TEST(PerforatePanel, AppliesAssetToPanelTransform) {
    // Panel frame: p = Rz(90 deg) * a + t, so panel x = 0.2 - a.y, y = a.x + 0.3, z = a.z + 0.1.
    auto panel = panelOptions();
    panel.asset_to_panel << 0, -1, 0, 0.2f, 1, 0, 0, 0.3f, 0, 0, 1, 0.1f, 0, 0, 0, 1;
    Submesh part;
    part.vertices = {vertex(-.5f, .2f, -.5f), vertex(.5f, .2f, -.5f), vertex(0, .2f, .5f),
                     vertex(-.5f, 0, -.5f),   vertex(.5f, 0, -.5f),   vertex(0, 0, .5f)};
    part.indices = {0, 1, 2, 3, 4, 5};
    const auto result = perforatePanel(asset({part}), panel);
    ASSERT_EQ(result.face_triangles, std::vector<std::size_t>({1}));
    ASSERT_EQ(result.mesh.submeshes.size(), 2U);
    const auto &out = result.mesh.submeshes[1];
    ASSERT_EQ(out.vertices.size(), 3U);
    for (std::size_t i = 0; i < 3; ++i) {
        EXPECT_EQ(out.vertices[i].position, part.vertices[i].position);
        EXPECT_TRUE(out.vertices[i].uv.isApprox(
            planar(part.vertices[i].position.x() + .3f, part.vertices[i].position.z() + .1f), 1e-6f));
    }
    // The identity transform sees neither triangle on the plane x = 0.
    EXPECT_EQ(perforatePanel(asset({part}), panelOptions()).face_triangles, std::vector<std::size_t>({0}));
}

TEST(PerforatePanel, RejectsInvalidParameters) {
    const auto source = asset({mixedSubmesh()});
    const auto rejects = [&](const std::function<void(PanelCutouts &)> &edit) {
        auto panel = panelOptions();
        edit(panel);
        EXPECT_THROW(perforatePanel(source, panel), std::invalid_argument);
    };
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    EXPECT_NO_THROW(perforatePanel(source, panelOptions()));
    rejects([](PanelCutouts &p) { p.faces_x.clear(); });
    rejects([&](PanelCutouts &p) { p.faces_x = {0.f, nan}; });
    rejects([&](PanelCutouts &p) { p.faces_x = {inf}; });
    rejects([](PanelCutouts &p) { p.half_size = 0; });
    rejects([](PanelCutouts &p) { p.half_size = -1; });
    rejects([&](PanelCutouts &p) { p.half_size = nan; });
    rejects([&](PanelCutouts &p) { p.half_size = inf; });
    rejects([&](PanelCutouts &p) { p.tolerance = nan; });
    rejects([&](PanelCutouts &p) { p.tolerance = inf; });
    rejects([](PanelCutouts &p) { p.tolerance = -1e-6f; });
    rejects([](PanelCutouts &p) { p.tolerance = p.half_size; });
    rejects([](PanelCutouts &p) { p.cutouts.clear(); });
    rejects([](PanelCutouts &p) { p.cutouts.assign(5, {{.5f, .5f}, .1f}); });
    rejects([&](PanelCutouts &p) { p.cutouts[0].center.x() = nan; });
    rejects([&](PanelCutouts &p) { p.cutouts[1].center.y() = inf; });
    rejects([](PanelCutouts &p) { p.cutouts[0].radius = 0; });
    rejects([](PanelCutouts &p) { p.cutouts[0].radius = -.1f; });
    rejects([&](PanelCutouts &p) { p.cutouts[1].radius = nan; });
    rejects([&](PanelCutouts &p) { p.cutouts[1].radius = inf; });
    rejects([&](PanelCutouts &p) { p.asset_to_panel(0, 3) = nan; });
    rejects([](PanelCutouts &p) { p.asset_to_panel(3, 0) = 1e-3f; });
    rejects([](PanelCutouts &p) { p.asset_to_panel(3, 3) = 2; });
    rejects([](PanelCutouts &p) { p.asset_to_panel(0, 0) = 1.001f; });
    rejects([](PanelCutouts &p) { p.asset_to_panel(0, 1) = 1e-3f; });
    rejects([](PanelCutouts &p) { p.asset_to_panel(0, 0) = -1; });
    rejects([](PanelCutouts &p) { p.asset_to_panel.topLeftCorner<3, 3>().setZero(); });
    Submesh broken = mixedSubmesh();
    broken.indices.push_back(6);
    EXPECT_THROW(perforatePanel(asset({broken}), panelOptions()), std::invalid_argument);
    broken.indices = {0, 1, 2, 3};
    EXPECT_THROW(perforatePanel(asset({broken}), panelOptions()), std::invalid_argument);
}

TEST(PerforatePanel, RobosubTorpedoSeparatesFrameBackingAndTexturedFront) {
    const auto path = std::filesystem::path(NEREUS_PACK_CONTENT) / "tasks/robosub_2026/assets/torpedo/model.dae";
    const auto source = loadMesh(path);
    PanelCutouts panel;
    panel.faces_x = {0.0f, -0.004f};
    panel.half_size = 0.3048f;
    panel.cutouts = {{{0.16142625f, 0.58890478f}, 0.10469f},
                     {{0.51378955f, 0.81221886f}, 0.08365f},
                     {{0.85499477f, 0.14589262f}, 0.10469f},
                     {{0.50045453f, 0.18734768f}, 0.08365f}};
    const auto result = perforatePanel(source, panel);
    EXPECT_EQ(result.face_triangles, std::vector<std::size_t>({2, 2}));
    // loadMesh yields the frame (which also holds the two backing triangles), then the front quad.
    ASSERT_EQ(source.submeshes.size(), 2U);
    EXPECT_FALSE(source.submeshes[0].material.diffuse_texture);
    EXPECT_TRUE(source.submeshes[1].material.diffuse_texture);
    ASSERT_EQ(result.mesh.submeshes.size(), 3U);
    EXPECT_EQ(result.mesh.minimum, source.minimum);
    EXPECT_EQ(result.mesh.maximum, source.maximum);
    const auto &frame = result.mesh.submeshes[0];
    const auto &backing = result.mesh.submeshes[1];
    const auto &front = result.mesh.submeshes[2];
    EXPECT_EQ(frame.indices.size() / 3, 27724U);
    EXPECT_TRUE(frame.material.cutouts.empty());
    EXPECT_EQ(backing.indices.size() / 3, 2U);
    EXPECT_FALSE(backing.material.diffuse_texture);
    EXPECT_EQ(backing.material.cutouts.size(), 4U);
    EXPECT_TRUE(backing.material.base_color.isApprox(frame.material.base_color));
    ASSERT_EQ(backing.vertices.size(), 6U);
    for (const auto &v : backing.vertices) {
        EXPECT_NEAR(v.position.x(), -0.003988f, 1e-5f);
        EXPECT_TRUE(v.uv.isApprox(planar(v.position.y(), v.position.z(), 0.3048f), 1e-6f));
    }
    EXPECT_EQ(front.indices.size() / 3, 2U);
    EXPECT_TRUE(front.material.diffuse_texture);
    EXPECT_EQ(front.material.cutouts.size(), 4U);
    for (const auto &v : front.vertices)
        EXPECT_NEAR(v.position.x(), 0.f, 5e-4f);
}

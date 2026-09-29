#include "robotics/rendering/assets.hpp"
#include <fstream>
#include <gtest/gtest.h>
#include <sstream>

using namespace robotics::rendering;
TEST(MeshAssets, OriginalTalosBodyAndRotorsKeepTriangleCountsAndTransparency) {
    const std::filesystem::path root = std::filesystem::path(RP_VISUAL_CONTENT) / "talos";
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
    EXPECT_THROW(loadMesh(std::filesystem::path(RP_VISUAL_CONTENT) / "talos/inventory.json"),
                 std::runtime_error);
    const auto body = std::filesystem::path(RP_VISUAL_CONTENT) / "talos/Talos3_body.glb";
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
    const auto asset = loadMesh(std::filesystem::path(RP_ASSET_FIXTURES) / "nested_mesh.gltf");
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
    EXPECT_THROW(loadMesh(std::filesystem::path(RP_ASSET_FIXTURES) / "nested_mesh.gltf", limits),
                 std::runtime_error);
}

TEST(MeshAssets, EveryOriginalSubmeshKeepsOrderTopologyMaterialsAndVertexStatistics) {
    std::ifstream reference(std::filesystem::path(RP_ASSET_FIXTURES) / "legacy_mesh_assets.csv");
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
            const auto root = std::filesystem::path(RP_VISUAL_CONTENT) / "talos";
            asset = loadMesh(fields[0] == "Talos3_body.glb" ? root / fields[0]
                                                            : root / "rotors" / fields[0]);
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
        actual << part.material.base_color.cast<double>(), minimum.cast<double>(),
            maximum.cast<double>(), position, normal, uv;
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

#pragma once
#include <Eigen/Core>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <vector>

namespace robotics::rendering {
struct Vertex {
    Eigen::Vector3f position, normal;
    Eigen::Vector2f uv;
};
struct Material {
    Eigen::Vector4f base_color = Eigen::Vector4f::Ones();
    // External image reference only; decoding/GPU ownership belongs to the renderer.
    std::optional<std::filesystem::path> diffuse_texture;
};
struct Submesh {
    std::vector<Vertex> vertices;
    std::vector<std::uint32_t> indices; // Triangle list, indexing this submesh.
    Material material;
};
struct MeshAsset {
    std::vector<Submesh> submeshes;   // Original depth-first node/mesh order.
    Eigen::Vector3f minimum, maximum; // Indexed bounds in authored asset coordinates.
};
struct AssetLimits {
    std::uintmax_t file_bytes = 128 * 1024 * 1024;
    std::size_t vertices = 8'000'000, triangles = 4'000'000;
    std::size_t submeshes = 10'000, nodes = 100'000;
};
// Owns all returned data; no GL, window, simulation, cache or source lifetime dependency.
// Bakes node transforms and inverse-transpose normals, retaining authored axes/units.
// External textures must use relative paths; embedded textures are not supported yet.
// Limits bound source size and output data, not allocations internal to Assimp.
MeshAsset loadMesh(const std::filesystem::path &path, AssetLimits limits = {});
} // namespace robotics::rendering

#pragma once
#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <vector>

namespace robotics::rendering {
struct Vertex {
    Eigen::Vector3f position, normal;
    Eigen::Vector2f uv;
};
// Circular opening in texture coordinates: fragments with |uv - center| < radius are
// discarded from color, depth and shadow passes (for example perforated panels).
struct UvCutout {
    Eigen::Vector2f center = Eigen::Vector2f::Zero();
    float radius = 0;
};
struct Material {
    Eigen::Vector4f base_color = Eigen::Vector4f::Ones();
    // External image reference only; decoding/GPU ownership belongs to the renderer.
    std::optional<std::filesystem::path> diffuse_texture;
    // Per-submesh render data supplied by scene composition; the loader leaves it empty.
    // The renderer accepts at most four cutouts per submesh.
    std::vector<UvCutout> cutouts;
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
// Planar perforated panel in its own frame: faces are planes x = faces_x[i] with |y|, |z| <=
// half_size; panel UV is u = y / (2 half_size) + 0.5, v = z / (2 half_size) + 0.5.
struct PanelCutouts {
    Eigen::Matrix4f asset_to_panel = Eigen::Matrix4f::Identity(); // Rigid.
    std::vector<float> faces_x;
    float half_size = 0;
    std::vector<UvCutout> cutouts; // Panel UV; the renderer accepts at most four.
    float tolerance = 5e-4f;       // Metres, for face distance and panel extent.
};
struct PerforatedMesh {
    MeshAsset mesh;
    std::vector<std::size_t> face_triangles; // Selected triangles per faces_x entry.
};
// Moves triangles lying on the panel faces into submeshes carrying the cutouts. Positions and
// normals are unchanged; a mixed submesh becomes its remainder followed by a panel submesh with
// copied vertices. Throws std::invalid_argument for invalid parameters or inconsistent UVs.
PerforatedMesh perforatePanel(const MeshAsset &mesh, const PanelCutouts &panel);
} // namespace robotics::rendering

#include "robotics/rendering/assets.hpp"
#include <Eigen/LU>
#include <algorithm>
#include <assimp/Importer.hpp>
#include <assimp/config.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

namespace robotics::rendering {
namespace {
Eigen::Matrix4f matrix(const aiMatrix4x4 &m) {
    Eigen::Matrix4f result;
    result << m.a1, m.a2, m.a3, m.a4, m.b1, m.b2, m.b3, m.b4, m.c1, m.c2, m.c3, m.c4, m.d1, m.d2,
        m.d3, m.d4;
    return result;
}
Eigen::Vector3f vector(const aiVector3D &v) {
    return {v.x, v.y, v.z};
}
Eigen::Vector2f panelUv(const Eigen::Vector3f &panel_position, float half_size) {
    return {panel_position.y() / (2 * half_size) + 0.5f,
            panel_position.z() / (2 * half_size) + 0.5f};
}
} // namespace
MeshAsset loadMesh(const std::filesystem::path &path, AssetLimits limits) {
    const auto fail = [&path](const std::string &message) -> void {
        throw std::runtime_error(path.string() + ": " + message);
    };
    if (!limits.file_bytes || !limits.vertices || !limits.triangles || !limits.submeshes ||
        !limits.nodes)
        throw std::invalid_argument("asset limits must be positive");
    std::error_code error;
    const auto bytes = std::filesystem::file_size(path, error);
    if (error || !bytes || bytes > limits.file_bytes)
        fail("missing, empty or oversized asset");
    Assimp::Importer importer;
    importer.SetPropertyBool(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, true);
    const auto *scene =
        importer.ReadFile(path.string(), aiProcess_Triangulate | aiProcess_GenSmoothNormals |
                                             aiProcess_JoinIdenticalVertices);
    if (!scene || !scene->mRootNode)
        fail(importer.GetErrorString());
    MeshAsset result;
    result.minimum.setConstant(std::numeric_limits<float>::infinity());
    result.maximum = -result.minimum;
    struct Pending {
        const aiNode *node;
        Eigen::Matrix4f parent;
    };
    std::vector<Pending> pending{{scene->mRootNode, Eigen::Matrix4f::Identity()}};
    std::size_t nodes = 0, vertices = 0, triangles = 0;
    while (!pending.empty()) {
        const auto current = pending.back();
        pending.pop_back();
        if (++nodes > limits.nodes)
            fail("node limit exceeded");
        const auto *node = current.node;
        const Eigen::Matrix4f transform = current.parent * matrix(node->mTransformation);
        const Eigen::Matrix3f linear = transform.topLeftCorner<3, 3>();
        if (!transform.allFinite() || !transform.row(3).isApprox(Eigen::RowVector4f(0, 0, 0, 1)) ||
            !linear.fullPivLu().isInvertible())
            fail("invalid or singular affine node transform");
        const Eigen::Matrix3f normal = linear.inverse().transpose();
        for (unsigned j = 0; j < node->mNumMeshes; ++j) {
            if (node->mMeshes[j] >= scene->mNumMeshes)
                fail("invalid node mesh index");
            const auto *input = scene->mMeshes[node->mMeshes[j]];
            if (!input->mNumFaces)
                continue;
            if (!input->HasPositions() || !input->HasNormals())
                fail("mesh requires positions and normals");
            if (input->mNumVertices > limits.vertices - vertices ||
                input->mNumFaces > limits.triangles - triangles ||
                result.submeshes.size() >= limits.submeshes)
                fail("mesh allocation limit exceeded");
            vertices += input->mNumVertices;
            triangles += input->mNumFaces;
            Submesh mesh;
            mesh.vertices.reserve(input->mNumVertices);
            mesh.indices.reserve(static_cast<std::size_t>(input->mNumFaces) * 3);
            for (unsigned k = 0; k < input->mNumVertices; ++k) {
                const auto &p = input->mVertices[k];
                const Eigen::Vector4f position = transform * Eigen::Vector4f(p.x, p.y, p.z, 1);
                const Eigen::Vector3f n = normal * vector(input->mNormals[k]);
                // Retain original float dot order and reciprocal multiplication. Equivalent
                // normalizations can move shadow-bias thresholds in unoptimized builds.
                const float norm = std::sqrt((n.x() * n.x() + n.y() * n.y()) + n.z() * n.z());
                Eigen::Vector2f uv = Eigen::Vector2f::Zero();
                if (input->HasTextureCoords(0))
                    uv = {input->mTextureCoords[0][k].x, input->mTextureCoords[0][k].y};
                if (!position.allFinite() || !n.allFinite() || !std::isfinite(norm) || norm <= 0 ||
                    !uv.allFinite())
                    fail("invalid transformed vertex");
                mesh.vertices.push_back({position.head<3>(), n * (1.f / norm), uv});
            }
            for (unsigned k = 0; k < input->mNumFaces; ++k) {
                const auto &face = input->mFaces[k];
                if (face.mNumIndices != 3)
                    fail("mesh contains non-triangle primitives");
                for (unsigned l = 0; l < 3; ++l) {
                    if (face.mIndices[l] >= mesh.vertices.size())
                        fail("invalid triangle index");
                    mesh.indices.push_back(face.mIndices[l]);
                    const auto &position = mesh.vertices[face.mIndices[l]].position;
                    result.minimum = result.minimum.cwiseMin(position);
                    result.maximum = result.maximum.cwiseMax(position);
                }
            }
            if (input->mMaterialIndex >= scene->mNumMaterials)
                fail("invalid material index");
            const auto *material = scene->mMaterials[input->mMaterialIndex];
            aiColor4D color(1, 1, 1, 1);
            aiGetMaterialColor(material, AI_MATKEY_COLOR_DIFFUSE, &color);
            float opacity = 1;
            material->Get(AI_MATKEY_OPACITY, opacity);
            mesh.material.base_color = {color.r, color.g, color.b, std::min(color.a, opacity)};
            if (!mesh.material.base_color.allFinite() || !std::isfinite(opacity) ||
                (mesh.material.base_color.array() < 0).any() || mesh.material.base_color.w() > 1)
                fail("invalid material color/opacity");
            aiString texture;
            if (material->GetTexture(aiTextureType_DIFFUSE, 0, &texture) == AI_SUCCESS) {
                const std::filesystem::path reference(texture.C_Str());
                if (reference.empty() || reference.is_absolute() ||
                    reference.string().front() == '*')
                    fail("texture must be an external relative resource");
                mesh.material.diffuse_texture = (path.parent_path() / reference).lexically_normal();
            }
            result.submeshes.push_back(std::move(mesh));
        }
        if (node->mNumChildren > limits.nodes - nodes ||
            pending.size() > limits.nodes - nodes - node->mNumChildren)
            fail("node limit exceeded");
        // Reverse push preserves the original recursive depth-first draw order.
        for (unsigned k = node->mNumChildren; k > 0; --k)
            pending.push_back({node->mChildren[k - 1], transform});
    }
    if (result.submeshes.empty())
        fail("asset contains no triangles");
    return result;
}
PerforatedMesh perforatePanel(const MeshAsset &mesh, const PanelCutouts &panel) {
    const float half = panel.half_size;
    if (panel.faces_x.empty() || !std::all_of(panel.faces_x.begin(), panel.faces_x.end(),
                                              [](float x) { return std::isfinite(x); }))
        throw std::invalid_argument("panel faces must be non-empty and finite");
    if (!std::isfinite(half) || half <= 0)
        throw std::invalid_argument("panel half size must be finite and positive");
    if (!std::isfinite(panel.tolerance) || panel.tolerance < 0 || panel.tolerance >= half)
        throw std::invalid_argument(
            "panel tolerance must be finite, non-negative and below half size");
    if (panel.cutouts.empty() || panel.cutouts.size() > 4 ||
        !std::all_of(panel.cutouts.begin(), panel.cutouts.end(), [](const UvCutout &cutout) {
            return cutout.center.allFinite() && std::isfinite(cutout.radius) && cutout.radius > 0;
        }))
        throw std::invalid_argument("panel needs one to four finite cutouts with positive radius");
    const Eigen::Matrix4f &transform = panel.asset_to_panel;
    const Eigen::Matrix3f rotation = transform.topLeftCorner<3, 3>();
    if (!transform.allFinite() || transform.row(3) != Eigen::RowVector4f(0, 0, 0, 1) ||
        !(rotation.transpose() * rotation).isApprox(Eigen::Matrix3f::Identity(), 1e-5f) ||
        std::abs(rotation.determinant() - 1) > 1e-5f)
        throw std::invalid_argument("panel transform must be rigid");
    const auto toPanel = [&transform](const Eigen::Vector3f &p) -> Eigen::Vector3f {
        return (transform * Eigen::Vector4f(p.x(), p.y(), p.z(), 1)).head<3>();
    };
    const auto faceOf = [&](const Submesh &part,
                            std::size_t triangle) -> std::optional<std::size_t> {
        Eigen::Vector3f corners[3];
        for (std::size_t k = 0; k < 3; ++k) {
            const auto index = part.indices[triangle * 3 + k];
            if (index >= part.vertices.size())
                throw std::invalid_argument("submesh index out of range");
            corners[k] = toPanel(part.vertices[index].position);
        }
        for (std::size_t i = 0; i < panel.faces_x.size(); ++i) {
            const bool inside = std::all_of(corners, corners + 3, [&](const Eigen::Vector3f &c) {
                return std::abs(c.x() - panel.faces_x[i]) <= panel.tolerance &&
                       std::abs(c.y()) <= half + panel.tolerance &&
                       std::abs(c.z()) <= half + panel.tolerance;
            });
            if (inside)
                return i;
        }
        return std::nullopt;
    };
    const auto checkTexture = [&](const Submesh &part, const Vertex &vertex) {
        if (part.material.diffuse_texture &&
            (vertex.uv - panelUv(toPanel(vertex.position), half)).cwiseAbs().maxCoeff() > 1e-4f)
            throw std::invalid_argument("textured panel face UVs differ from the panel mapping");
    };
    PerforatedMesh result;
    result.mesh.minimum = mesh.minimum;
    result.mesh.maximum = mesh.maximum;
    result.mesh.submeshes.reserve(mesh.submeshes.size());
    result.face_triangles.assign(panel.faces_x.size(), 0);
    for (const auto &part : mesh.submeshes) {
        if (part.indices.size() % 3)
            throw std::invalid_argument("submesh indices are not a triangle list");
        std::vector<std::uint32_t> remainder, selected;
        for (std::size_t t = 0; t < part.indices.size() / 3; ++t) {
            const auto face = faceOf(part, t);
            auto &target = face ? selected : remainder;
            target.insert(target.end(), part.indices.begin() + t * 3,
                          part.indices.begin() + t * 3 + 3);
            if (face)
                ++result.face_triangles[*face];
        }
        if (selected.empty()) {
            result.mesh.submeshes.push_back(part);
            continue;
        }
        if (!part.material.cutouts.empty())
            throw std::invalid_argument("panel submesh already has cutouts");
        if (remainder.empty()) {
            Submesh panel_part = part;
            for (auto index : selected) {
                auto &vertex = panel_part.vertices[index];
                checkTexture(part, vertex);
                if (!part.material.diffuse_texture)
                    vertex.uv = panelUv(toPanel(vertex.position), half);
            }
            panel_part.material.cutouts = panel.cutouts;
            result.mesh.submeshes.push_back(std::move(panel_part));
            continue;
        }
        Submesh kept{part.vertices, std::move(remainder), part.material};
        Submesh panel_part;
        panel_part.material = part.material;
        panel_part.material.cutouts = panel.cutouts;
        panel_part.vertices.reserve(selected.size());
        panel_part.indices.reserve(selected.size());
        for (auto index : selected) {
            Vertex vertex = part.vertices[index];
            checkTexture(part, vertex);
            if (!part.material.diffuse_texture)
                vertex.uv = panelUv(toPanel(vertex.position), half);
            panel_part.indices.push_back(static_cast<std::uint32_t>(panel_part.vertices.size()));
            panel_part.vertices.push_back(vertex);
        }
        result.mesh.submeshes.push_back(std::move(kept));
        result.mesh.submeshes.push_back(std::move(panel_part));
    }
    return result;
}
} // namespace robotics::rendering

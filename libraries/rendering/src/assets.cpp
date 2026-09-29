#include "robotics/rendering/assets.hpp"
#include <Eigen/LU>
#include <algorithm>
#include <assimp/Importer.hpp>
#include <assimp/config.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <cmath>
#include <limits>
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
} // namespace robotics::rendering

#pragma once
// Minimal Wavefront OBJ reader for collision meshes (private to nereus_session). Reads `v` lines and
// triangulates `f` polygons as fans (collision assets are triangle meshes). Throws
// std::runtime_error with the path in the message.
#include <Eigen/Core>

#include <array>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace nereus::session {
struct ObjMesh {
    std::vector<Eigen::Vector3d> vertices;     // every `v` line, in file order
    std::vector<std::array<int, 3>> triangles; // indices into `vertices`

    // Vertices referenced by at least one face, in first-use order (a viewer-side loader keeps
    // one vertex per face corner; a hull over these equals a hull over the corners).
    std::vector<Eigen::Vector3d> referenced() const {
        std::vector<int> slot(vertices.size(), -1);
        std::vector<Eigen::Vector3d> result;
        for (const auto &triangle : triangles)
            for (const int index : triangle)
                if (slot[static_cast<std::size_t>(index)] < 0) {
                    slot[static_cast<std::size_t>(index)] = static_cast<int>(result.size());
                    result.push_back(vertices[static_cast<std::size_t>(index)]);
                }
        return result;
    }
};

inline ObjMesh loadObj(const std::filesystem::path &path) {
    std::ifstream stream(path);
    if (!stream)
        throw std::runtime_error("cannot open mesh " + path.string());

    ObjMesh mesh;
    std::string line;
    while (std::getline(stream, line)) {
        // Every other line type (vn, vt, o, g, usemtl, comments, ...) is ignored.
        if (line.size() > 2 && line[0] == 'v' && (line[1] == ' ' || line[1] == '\t')) {
            std::istringstream fields(line.substr(2));
            double x, y, z;
            if (!(fields >> x >> y >> z))
                throw std::runtime_error("bad vertex line in " + path.string());
            mesh.vertices.emplace_back(x, y, z);
        } else if (line.size() > 2 && line[0] == 'f' && (line[1] == ' ' || line[1] == '\t')) {
            std::istringstream fields(line.substr(2));
            std::vector<int> corner;
            std::string token;
            // Corners are `v`, `v/vt`, `v//vn` or `v/vt/vn`; only the 1-based (or negative, relative) vertex index
            // is used.
            while (fields >> token) {
                int index = std::stoi(token.substr(0, token.find('/')));
                index = index < 0 ? static_cast<int>(mesh.vertices.size()) + index : index - 1;
                if (index < 0 || index >= static_cast<int>(mesh.vertices.size()))
                    throw std::runtime_error("face index out of range in " + path.string());
                corner.push_back(index);
            }

            // Fan-triangulate the polygon around its first corner.
            for (std::size_t i = 2; i < corner.size(); ++i)
                mesh.triangles.push_back({corner[0], corner[i - 1], corner[i]});
        }
    }

    if (mesh.vertices.empty())
        throw std::runtime_error("mesh has no vertices: " + path.string());
    return mesh;
}
} // namespace nereus::session

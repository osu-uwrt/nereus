// Offline CPU wrapper for the unchanged original Renderer::load; never linked to platform.
#include <algorithm>
#include <assimp/Importer.hpp>
#include <assimp/config.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <glm/gtc/type_ptr.hpp>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <vector>
struct Vertex {
    glm::vec3 p, n;
    glm::vec2 uv;
};
struct Mesh {
    std::vector<Vertex> vertices;
    std::vector<unsigned> indices;
    glm::vec4 color{1};
    std::vector<glm::vec3> holes;
    unsigned texture = 0;
    Mesh(const std::vector<Vertex> &v, const std::vector<unsigned> &i) : vertices(v), indices(i) {}
};
struct Renderer {
    std::string meshRoot;
    std::map<std::string, std::vector<std::shared_ptr<Mesh>>> cache;
    std::vector<glm::vec3> torpedoHoles;
    std::size_t triangles = 0;
    unsigned texture(const std::string &) {
        throw std::runtime_error("texture-free reference unexpectedly requested texture");
    }
    std::vector<std::shared_ptr<Mesh>> load(const std::string &, const std::string & = "");
};
#include "mesh_loader.inc"
int main(int argc, char **argv) {
    std::cout << "asset,part,vertices,indices,r,g,b,a,min_x,min_y,min_z,max_x,max_y,max_z,mean_x,"
                 "mean_y,mean_z,mean_nx,mean_ny,mean_nz,mean_u,mean_v,index_hash\n";
    std::cout << std::setprecision(17);
    for (int file = 1; file < argc; ++file) {
        Renderer renderer;
        const std::filesystem::path path(argv[file]);
        const auto meshes = renderer.load(std::filesystem::absolute(path).string());
        for (std::size_t j = 0; j < meshes.size(); ++j) {
            const auto &mesh = *meshes[j];
            glm::vec3 minimum(std::numeric_limits<float>::infinity()), maximum(-minimum);
            glm::dvec3 positions(0), normals(0);
            glm::dvec2 uv(0);
            for (const auto &v : mesh.vertices) {
                minimum = glm::min(minimum, v.p);
                maximum = glm::max(maximum, v.p);
                positions += glm::dvec3(v.p);
                normals += glm::dvec3(v.n);
                uv += glm::dvec2(v.uv);
            }
            positions /= double(mesh.vertices.size());
            normals /= double(mesh.vertices.size());
            uv /= double(mesh.vertices.size());
            std::uint64_t hash = 14695981039346656037ULL;
            for (auto index : mesh.indices)
                hash = (hash ^ index) * 1099511628211ULL;
            std::cout << path.filename().string() << ',' << j << ',' << mesh.vertices.size() << ','
                      << mesh.indices.size();
            for (int k = 0; k < 4; ++k)
                std::cout << ',' << mesh.color[k];
            for (const auto &v : {glm::dvec3(minimum), glm::dvec3(maximum), positions, normals})
                for (int k = 0; k < 3; ++k)
                    std::cout << ',' << v[k];
            std::cout << ',' << uv.x << ',' << uv.y << ',' << hash << '\n';
        }
    }
}

#include <robotics/scene_view/scene.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace robotics::scene_view {
namespace {
void fields(const YAML::Node &node, std::initializer_list<std::string> allowed) {
    if (!node.IsMap())
        throw std::invalid_argument("expected mapping");
    std::set<std::string> seen;
    for (const auto &item : node) {
        const auto key = item.first.as<std::string>();
        if (!seen.insert(key).second ||
            std::find(allowed.begin(), allowed.end(), key) == allowed.end())
            throw std::invalid_argument("duplicate or unknown field: " + key);
    }
}
std::string name(const YAML::Node &node) {
    const auto value = node.as<std::string>();
    if (value.empty() || value.size() > 256)
        throw std::invalid_argument("name must contain 1..256 characters");
    return value;
}
template <int N> Eigen::Matrix<float, N, 1> vector(const YAML::Node &node) {
    if (!node.IsSequence() || node.size() != N)
        throw std::invalid_argument("wrong vector length");
    Eigen::Matrix<float, N, 1> value;
    for (int i = 0; i < N; ++i)
        value[i] = node[i].as<float>();
    if (!value.allFinite())
        throw std::invalid_argument("nonfinite vector");
    return value;
}
Eigen::Matrix4f pose(const YAML::Node &node) {
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    if (!node)
        return result;
    fields(node, {"position", "orientation_wxyz"});
    const auto q = vector<4>(node["orientation_wxyz"]);
    const Eigen::Quaternionf rotation(q[0], q[1], q[2], q[3]);
    if (std::abs(rotation.norm() - 1) > 1e-5f)
        throw std::invalid_argument("orientation must be a unit quaternion");
    result.topLeftCorner<3, 3>() = rotation.toRotationMatrix();
    result.topRightCorner<3, 1>() = vector<3>(node["position"]);
    return result;
}
void sequence(const YAML::Node &node, std::size_t maximum) {
    if (!node.IsSequence() || node.size() > maximum)
        throw std::invalid_argument("expected bounded sequence");
}
} // namespace
Document load(const std::filesystem::path &path) {
    try {
        if (std::filesystem::file_size(path) > 1024 * 1024)
            throw std::invalid_argument("scene document exceeds 1 MiB");
        const auto root = YAML::LoadFile(path.string());
        fields(root, {"version", "groups"});
        if (root["version"].as<int>() != 1)
            throw std::invalid_argument("unsupported scene version");
        sequence(root["groups"], 128);
        Document result;
        std::set<std::string> ids;
        std::map<std::filesystem::path, std::shared_ptr<const rendering::MeshAsset>> meshes;
        std::size_t instance_count = 0;
        bool has_water = false;
        for (const auto &node : root["groups"]) {
            fields(node, {"id", "source", "frame", "pool", "instances"});
            Group group;
            group.id = name(node["id"]);
            if (!ids.insert(group.id).second)
                throw std::invalid_argument("duplicate group: " + group.id);
            if (node["source"])
                group.source = name(node["source"]);
            group.frame = name(node["frame"]);
            if (node["pool"]) {
                if (has_water)
                    throw std::invalid_argument("scene supports one water surface");
                has_water = true;
                const auto pool = node["pool"];
                fields(pool, {"dimensions", "water_level", "deck_height", "pose"});
                rendering::PoolGeometry geometry;
                geometry.dimensions = vector<3>(pool["dimensions"]);
                geometry.water_level = pool["water_level"].as<float>();
                geometry.deck_height = pool["deck_height"].as<float>();
                geometry.local_to_world = pose(pool["pose"]);
                group.content = rendering::makePoolScene(geometry);
            }
            if (node["instances"]) {
                sequence(node["instances"], 4096);
                instance_count += node["instances"].size();
                if (instance_count > 4096)
                    throw std::invalid_argument("scene exceeds 4096 mesh instances");
                for (const auto &item : node["instances"]) {
                    fields(item, {"mesh", "pose", "tint", "material", "radiance", "casts_shadow"});
                    const auto filename = item["mesh"].as<std::string>();
                    if (filename.empty() || filename.size() > 4096)
                        throw std::invalid_argument("invalid mesh path");
                    const auto resource =
                        std::filesystem::weakly_canonical(path.parent_path() / filename);
                    auto &mesh = meshes[resource];
                    if (!mesh)
                        mesh = std::make_shared<const rendering::MeshAsset>(
                            rendering::loadMesh(resource));
                    rendering::Instance instance;
                    instance.mesh = mesh;
                    instance.transform = pose(item["pose"]);
                    if (item["tint"])
                        instance.tint = vector<4>(item["tint"]);
                    if ((instance.tint.array() < 0).any() || (instance.tint.array() > 1).any())
                        throw std::invalid_argument("tint components must be in [0,1]");
                    if (item["material"]) {
                        const std::map<std::string, rendering::SurfaceMaterial> materials{
                            {"asset", rendering::SurfaceMaterial::Asset},
                            {"clear", rendering::SurfaceMaterial::Clear},
                            {"emissive", rendering::SurfaceMaterial::Emissive}};
                        instance.material = materials.at(item["material"].as<std::string>());
                    }
                    if (item["radiance"])
                        instance.radiance = item["radiance"].as<float>();
                    if (!std::isfinite(instance.radiance) || instance.radiance < 0)
                        throw std::invalid_argument("radiance must be finite and nonnegative");
                    if (item["casts_shadow"])
                        instance.casts_shadow = item["casts_shadow"].as<bool>();
                    group.content.instances.push_back(std::move(instance));
                }
            }
            if (group.content.instances.empty())
                throw std::invalid_argument("empty group: " + group.id);
            result.groups.push_back(std::move(group));
        }
        return result;
    } catch (const std::exception &error) {
        throw std::invalid_argument(path.string() + ": " + error.what());
    }
}
} // namespace robotics::scene_view

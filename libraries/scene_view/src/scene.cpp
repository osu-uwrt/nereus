#include <robotics/scene_view/scene.hpp>
#include <stdexcept>

namespace robotics::scene_view {
Resolved resolve(const Document &document, const std::string &fixed_frame,
                 const visualization::SourceSnapshot *source) {
    Resolved result;
    for (const auto &group : document.groups) {
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        if (group.source.empty()) {
            if (group.frame != fixed_frame) {
                result.issues[group.id] = "Static group requires fixed frame: " + group.frame;
                continue;
            }
        } else {
            if (!source || source->id != group.source || !source->data || !source->data->frames) {
                result.issues[group.id] = "Awaiting source: " + group.source;
                continue;
            }
            const auto found =
                source->data->frames->lookup(fixed_frame, group.frame, source->time_ns);
            if (!found.pose) {
                result.issues[group.id] = found.issue;
                continue;
            }
            transform.topLeftCorner<3, 3>() = found.pose->rotation.toRotationMatrix().cast<float>();
            transform.topRightCorner<3, 1>() = found.pose->translation.cast<float>();
        }
        std::map<std::size_t, Eigen::Vector3f> colors;
        for (const auto &binding : group.colors) {
            if (binding.instance >= group.content.instances.size() || binding.channel.empty() ||
                colors.count(binding.instance)) {
                result.issues[group.id] = "Invalid instance color binding";
                break;
            }
            if (group.source.empty() || !source || !source->data) {
                result.issues[group.id] = "Color binding requires a source";
                break;
            }
            const auto channel = source->data->colors.find(binding.channel);
            const auto color = channel == source->data->colors.end()
                                   ? std::nullopt
                                   : visualization::colorAt(channel->second, source->time_ns);
            if (!color) {
                result.issues[group.id] = "Awaiting color channel: " + binding.channel;
                break;
            }
            try {
                visualization::validateColor(*color);
            } catch (const std::invalid_argument &error) {
                result.issues[group.id] = error.what();
                break;
            }
            colors.emplace(binding.instance, *color);
        }
        if (result.issues.count(group.id))
            continue;
        auto water = group.content.water;
        if (water) {
            if (result.scene.water) {
                result.issues[group.id] = "Only one water surface is supported";
                continue;
            }
            const Eigen::Matrix3f rotation = transform.topLeftCorner<3, 3>();
            if (!(rotation * Eigen::Vector3f::UnitZ()).isApprox(Eigen::Vector3f::UnitZ(), 1e-5f)) {
                result.issues[group.id] = "Water requires a Z-up fixed frame";
                continue;
            }
            water->surface.transform = transform * water->surface.transform;
            water->level += transform(2, 3);
            water->local_to_world = transform * water->local_to_world;
            water->local_to_world(2, 3) = 0;
        }
        for (std::size_t i = 0; i < group.content.instances.size(); ++i) {
            const auto &instance = group.content.instances[i];
            auto resolved = instance;
            if (colors.count(i))
                resolved.tint.head<3>() = resolved.tint.head<3>().cwiseProduct(colors.at(i));
            resolved.transform = transform * instance.transform;
            result.scene.instances.push_back(std::move(resolved));
        }
        if (water) {
            result.scene.lighting_center =
                transform.topLeftCorner<3, 3>() * group.content.lighting_center +
                transform.topRightCorner<3, 1>();
            result.scene.water = std::move(water);
        }
    }
    return result;
}
} // namespace robotics::scene_view

#include <nereus/session/scenario.hpp>

#include <string>
#include <vector>

namespace nereus::session {

simulation::FloorProfile poolFloor(const Json &pool) {
    const Json &p = pool.at("parameters");
    if (!p.contains("floor_profile"))
        return simulation::FloorProfile::flat(p.at("depth_m").get<double>(), p.at("length_m").get<double>());
    const Json &profile = p.at("floor_profile");
    std::vector<Eigen::Vector2d> points;
    for (const auto &point : profile.at("points_m"))
        points.emplace_back(point.at(0).get<double>(), point.at(1).get<double>());
    const auto axis = profile.at("along").get<std::string>() == "y" ? simulation::FloorProfile::Axis::Y
                                                                    : simulation::FloorProfile::Axis::X;
    return simulation::FloorProfile::smooth(axis, points);
}

Json poolCollisionBoxes(const Json &pool) {
    Json boxes = pool.at("collision_boxes");
    const Json &p = pool.at("parameters");
    if (!p.contains("floor_profile"))
        return boxes;
    const auto floor = poolFloor(pool);
    const double span = floor.axis() == simulation::FloorProfile::Axis::X ? p.at("width_m").get<double>()
                                                                          : p.at("length_m").get<double>();
    const auto generated = simulation::floorBoxes(floor, span, p.at("water_level_m").get<double>());
    for (std::size_t k = 0; k < generated.size(); ++k) {
        const auto &box = generated[k];
        const auto &q = box.orientation;
        boxes.push_back({{"id", "floor_" + std::to_string(k)},
                         {"size_m", {box.size.x(), box.size.y(), box.size.z()}},
                         {"center_m", {box.center.x(), box.center.y(), box.center.z()}},
                         {"orientation_wxyz", {q.w(), q.x(), q.y(), q.z()}},
                         {"floor", true}});
    }
    return boxes;
}

} // namespace nereus::session

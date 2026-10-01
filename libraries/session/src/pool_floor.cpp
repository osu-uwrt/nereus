#include <nereus/session/scenario.hpp>

#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace nereus::session {
namespace {
simulation::FloorProfile profileFrom(const Json &profile) {
    std::vector<Eigen::Vector2d> points;
    for (const auto &point : profile.at("points_m"))
        points.emplace_back(point.at(0).get<double>(), point.at(1).get<double>());
    const auto axis = profile.at("along").get<std::string>() == "y" ? simulation::FloorProfile::Axis::Y
                                                                    : simulation::FloorProfile::Axis::X;
    return simulation::FloorProfile::smooth(axis, points);
}
} // namespace

simulation::PoolFloor poolFloor(const Json &pool) {
    const Json &p = pool.at("parameters");
    if (!p.contains("floor_profile"))
        return simulation::PoolFloor::flat(p.at("depth_m").get<double>(), p.at("length_m").get<double>());
    const Json &floor = p.at("floor_profile");
    std::vector<simulation::FloorProfile> profiles;
    if (floor.is_array())
        for (const auto &profile : floor)
            profiles.push_back(profileFrom(profile));
    else
        profiles.push_back(profileFrom(floor));
    return simulation::PoolFloor(std::move(profiles));
}

std::vector<PoolFixtureBox> poolFixtureBoxes(const Json &pool) {
    std::vector<PoolFixtureBox> out;
    if (!pool.contains("fixtures"))
        return out;
    const auto floor = poolFloor(pool);
    const double surface = pool.at("parameters").at("water_level_m").get<double>();
    for (const auto &fixture : pool.at("fixtures")) {
        if (fixture.at("type").get<std::string>() != "box")
            continue;
        PoolFixtureBox box;
        box.id = fixture.at("id").get<std::string>();
        const auto &center = fixture.at("center_m"), &size = fixture.at("size_m");
        box.size = {size.at(0).get<double>(), size.at(1).get<double>(), size.at(2).get<double>()};
        box.on_floor = center.size() == 2;
        const double x = center.at(0).get<double>(), y = center.at(1).get<double>();
        box.center = {x, y,
                      box.on_floor ? surface - floor.depthAt(Eigen::Vector2d(x, y)) + box.size.z() / 2
                                   : surface + center.at(2).get<double>()};
        if (fixture.contains("rpy_deg")) {
            const auto &rpy = fixture.at("rpy_deg");
            const double radians = 3.14159265358979323846 / 180;
            box.orientation = Eigen::AngleAxisd(rpy.at(2).get<double>() * radians, Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(rpy.at(1).get<double>() * radians, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(rpy.at(0).get<double>() * radians, Eigen::Vector3d::UnitX());
        }
        box.contact = fixture.value("contact", false);
        out.push_back(std::move(box));
    }
    return out;
}

Json poolCollisionBoxes(const Json &pool) {
    Json boxes = pool.at("collision_boxes");
    for (const auto &box : poolFixtureBoxes(pool))
        if (box.contact)
            boxes.push_back({{"id", box.id},
                             {"size_m", {box.size.x(), box.size.y(), box.size.z()}},
                             {"center_m", {box.center.x(), box.center.y(), box.center.z()}},
                             {"orientation_wxyz",
                              {box.orientation.w(), box.orientation.x(), box.orientation.y(), box.orientation.z()}}});
    const Json &p = pool.at("parameters");
    if (!p.contains("floor_profile"))
        return boxes;
    const auto generated = simulation::floorBoxes(poolFloor(pool), p.at("length_m").get<double>(),
                                                  p.at("width_m").get<double>(), p.at("water_level_m").get<double>());
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

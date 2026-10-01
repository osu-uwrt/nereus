#include <nereus/session/pool.hpp>

#include "json_util.hpp"

#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace nereus::session {
namespace {
constexpr double kFlatFloorToleranceM = 1e-3;

simulation::FloorProfile profileFrom(const Json &profile) {
    std::vector<Eigen::Vector2d> points;
    for (const auto &point : profile.at("points_m"))
        points.emplace_back(point.at(0).get<double>(), point.at(1).get<double>());
    const auto axis = profile.at("along").get<std::string>() == "y" ? simulation::FloorProfile::Axis::Y
                                                                    : simulation::FloorProfile::Axis::X;
    return simulation::FloorProfile::smooth(axis, points);
}

// Where a fixture sits: on the floor under x, y (lifted by `lift`) when only those are given, else at z
// relative to the water surface; turned by rpy_deg (yaw, then pitch, then roll about the fixture's axes).
void place(const Json &fixture, const simulation::PoolFloor &floor, double surface, double lift,
           Eigen::Vector3d &center, Eigen::Quaterniond &orientation, bool &on_floor) {
    const auto &c = fixture.at("center_m");
    on_floor = c.size() == 2;
    const double x = c.at(0).get<double>(), y = c.at(1).get<double>();
    center = {x, y, on_floor ? surface - floor.depthAt(Eigen::Vector2d(x, y)) + lift : surface + c.at(2).get<double>()};
    orientation = Eigen::Quaterniond::Identity();
    if (fixture.contains("rpy_deg")) {
        const auto &rpy = fixture.at("rpy_deg");
        orientation = Eigen::AngleAxisd(rpy.at(2).get<double>() * (detail::kPi / 180), Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(rpy.at(1).get<double>() * (detail::kPi / 180), Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(rpy.at(0).get<double>() * (detail::kPi / 180), Eigen::Vector3d::UnitX());
    }
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

PoolModel poolModel(const Json &pool) {
    const Json &p = pool.at("parameters");
    PoolModel model;
    model.surface_z = p.at("water_level_m").get<double>();
    model.profiled = p.contains("floor_profile");
    model.floor = poolFloor(pool);

    for (const auto &fixture : pool.value("fixtures", Json::array())) {
        const auto type = fixture.at("type").get<std::string>();
        if (type == "box") {
            PoolFixtureBox box;
            box.id = fixture.at("id").get<std::string>();
            const auto &size = fixture.at("size_m");
            box.size = {size.at(0).get<double>(), size.at(1).get<double>(), size.at(2).get<double>()};
            place(fixture, model.floor, model.surface_z, box.size.z() / 2, box.center, box.orientation, box.on_floor);
            box.contact = fixture.value("contact", false);
            model.boxes.push_back(std::move(box));
        } else if (type == "mesh") {
            PoolFixtureMesh mesh;
            mesh.id = fixture.at("id").get<std::string>();
            mesh.asset = fixture.at("asset").get<std::string>();
            place(fixture, model.floor, model.surface_z, 0, mesh.center, mesh.orientation, mesh.on_floor);
            model.meshes.push_back(std::move(mesh));
        }
    }

    // The one rule for which boxes are floor: generated from the profile, or topped at the flat floor depth.
    const double flat_floor_z = model.surface_z - p.at("depth_m").get<double>();
    const auto atFlatFloor = [&](const PoolContactBox &box) {
        return std::abs(box.center.z() + box.size.z() / 2 - flat_floor_z) < kFlatFloorToleranceM;
    };
    // collision_boxes is required by the pool schema; interactive previews of hand-written documents may omit it.
    for (const auto &item : pool.value("collision_boxes", Json::array())) {
        PoolContactBox box;
        box.id = item.at("id").get<std::string>();
        box.size = detail::vec3(item.at("size_m"), "size_m");
        box.center = detail::vec3(item.at("center_m"), "center_m");
        box.orientation = detail::quat(item.at("orientation_wxyz"), "orientation_wxyz");
        box.floor = atFlatFloor(box);
        model.contacts.push_back(std::move(box));
    }
    for (const auto &fixture : model.boxes)
        if (fixture.contact) {
            PoolContactBox box{fixture.id, fixture.size, fixture.center, fixture.orientation, false};
            box.floor = atFlatFloor(box);
            model.contacts.push_back(std::move(box));
        }
    if (model.profiled) {
        const auto generated = simulation::floorBoxes(model.floor, p.at("length_m").get<double>(),
                                                      p.at("width_m").get<double>(), model.surface_z);
        for (std::size_t k = 0; k < generated.size(); ++k)
            model.contacts.push_back(
                {"floor_" + std::to_string(k), generated[k].size, generated[k].center, generated[k].orientation, true});
    }
    return model;
}

} // namespace nereus::session

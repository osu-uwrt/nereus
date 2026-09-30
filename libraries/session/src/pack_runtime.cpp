// Port of python/src/nereus/pack_runtime.py: physics and sensors from resolved packs.
#include <robotics/sensors/models.hpp>
#include <robotics/session/session.hpp>

#include "json_util.hpp"

#include <cmath>
#include <set>

namespace robotics::session {
namespace {
using namespace detail;
using simulation::BoxProxy;

bool has(const Json &object, const char *key) {
    return object.is_object() && object.contains(key);
}
const Json &at(const Json &object, const char *key) {
    if (!object.is_object() || !object.contains(key))
        throw std::invalid_argument(std::string("missing field '") + key + "'");
    return object.at(key);
}
const Json &getOr(const Json &object, const char *key, const Json &fallback) {
    return has(object, key) ? object.at(key) : fallback;
}
const Json kEmpty = Json::object();

void assignNumber(double &target, const Json &values, const char *key) {
    if (has(values, key))
        target = num(values.at(key), key);
}
void assignVec3(Eigen::Vector3d &target, const Json &values, const char *key) {
    if (has(values, key))
        target = vec3(values.at(key), key);
}
std::optional<Eigen::Vector3d> optionalVec3(const Json &values, const char *key) {
    if (!has(values, key) || values.at(key).is_null())
        return std::nullopt;
    return vec3(values.at(key), key);
}

spatial::FixedFrames makeFrames(const Json &config) {
    std::vector<spatial::FixedFrame> edges;
    for (const auto &entry : at(config, "transforms")) {
        spatial::FixedFrame edge;
        edge.parent = at(entry, "parent").get<std::string>();
        edge.child = at(entry, "child").get<std::string>();
        edge.pose = makePose(at(entry, "position_m"), at(entry, "orientation_wxyz"), "transform");
        edges.push_back(std::move(edge));
    }
    return spatial::FixedFrames(at(config, "root").get<std::string>(), std::move(edges));
}

BoxProxy makeBox(const Json &config, const Eigen::Vector3d &position,
                 const Eigen::Quaterniond &orientation) {
    BoxProxy box;
    box.id = at(config, "id").get<std::string>();
    box.size = vec3(at(config, "size_m"), "size_m");
    const spatial::Pose local{vec3(at(config, "center_m"), "center_m"),
                              has(config, "orientation_wxyz")
                                  ? quat(config.at("orientation_wxyz"), "orientation_wxyz")
                                  : Eigen::Quaterniond::Identity()};
    const auto pose = composeChecked({position, orientation}, local);
    box.center = pose.translation;
    box.orientation = pose.rotation;
    return box;
}

sensors::NoiseParameters makeNoise(const Json &config, bool enabled) {
    sensors::NoiseParameters result;
    if (enabled) {
        assignVec3(result.bias, config, "bias");
        assignVec3(result.white_stddev, config, "white_stddev");
        assignVec3(result.walk_stddev, config, "walk_stddev");
    }
    return result;
}
sensors::ScalarNoiseParameters makeScalarNoise(const Json &config, bool enabled) {
    sensors::ScalarNoiseParameters result;
    if (enabled) {
        assignNumber(result.bias, config, "bias");
        assignNumber(result.white_stddev, config, "white_stddev");
        assignNumber(result.walk_stddev, config, "walk_stddev");
    }
    return result;
}
sensors::ImuReporting makeReporting(const Json &config) {
    sensors::ImuReporting result;
    if (has(config, "gravity_magnitude_m_s2"))
        result.gravity_magnitude = num(config.at("gravity_magnitude_m_s2"), "gravity_magnitude");
    if (has(config, "force_variance"))
        result.force_variance = vec3(config.at("force_variance"), "force_variance");
    if (has(config, "angular_variance"))
        result.angular_variance = vec3(config.at("angular_variance"), "angular_variance");
    return result;
}
sensors::AttitudeParameters makeAttitude(const Json &config, bool noise) {
    sensors::AttitudeParameters result;
    assignVec3(result.heading_axis_world, config, "heading_axis_world");
    if (has(config, "reported_variance"))
        result.reported_variance = vec3(config.at("reported_variance"), "reported_variance");
    if (noise) {
        assignNumber(result.angle_stddev, config, "angle_stddev_rad");
        assignNumber(result.heading_drift_rate, config, "heading_drift_rad_s");
    }
    return result;
}

// Adds one sensor to the runtime and returns its type-erased handle name check.
void addSensor(sensors::Runtime &runtime, const Json &config, const spatial::FixedFrames &frames,
               const simulation::Pool &pool, double surface_pressure, bool noise) {
    const auto &mount_pose = frames.fromRoot(at(config, "mount_frame").get<std::string>());
    sensors::Mount mount;
    mount.position_body = mount_pose.translation;
    mount.sensor_to_body = mount_pose.rotation;
    const Json &p = at(config, "parameters");
    const std::string kind = at(config, "type").get<std::string>();

    sensors::Device device;
    device.id = at(config, "id").get<std::string>();
    device.frame = at(config, "frame").get<std::string>();
    device.period = sensors::Nanoseconds(at(config, "period_ns").get<std::int64_t>());
    device.latency = sensors::Nanoseconds(has(config, "latency_ns") ? config.at("latency_ns").get<std::int64_t>() : 0);
    device.capacity = has(config, "capacity") ? config.at("capacity").get<std::size_t>() : 64;
    const std::string overflow = has(config, "overflow") ? config.at("overflow").get<std::string>() : "fail";
    if (overflow == "fail")
        device.overflow = sensors::OverflowPolicy::Fail;
    else if (overflow == "drop_oldest")
        device.overflow = sensors::OverflowPolicy::DropOldest;
    else
        throw std::invalid_argument("sensor " + repr(device.id) + ": unknown overflow policy");

    if (kind == "imu") {
        runtime.add(device, sensors::Imu(mount, makeNoise(getOr(p, "acceleration_noise", kEmpty), noise),
                                         makeNoise(getOr(p, "gyro_noise", kEmpty), noise),
                                         makeReporting(getOr(p, "reporting", kEmpty))));
    } else if (kind == "attitude") {
        runtime.add(device, sensors::Attitude(mount, makeAttitude(p, noise)));
    } else if (kind == "ahrs") {
        sensors::AhrsParameters ahrs;
        const Json &inertial = at(p, "inertial");
        ahrs.acceleration_noise = makeNoise(getOr(inertial, "acceleration_noise", kEmpty), noise);
        ahrs.gyro_noise = makeNoise(getOr(inertial, "gyro_noise", kEmpty), noise);
        ahrs.inertial_reporting = makeReporting(getOr(inertial, "reporting", kEmpty));
        ahrs.attitude = makeAttitude(at(p, "attitude"), noise);
        runtime.add(device, sensors::Ahrs(mount, ahrs));
    } else if (kind == "fog") {
        std::vector<Eigen::Vector3d> axes;
        for (const auto &axis : at(p, "axes"))
            axes.push_back(vec3(axis, "axes"));
        runtime.add(device, sensors::Fog(mount, axes, makeNoise(getOr(p, "gyro_noise", kEmpty), noise),
                                         optionalVec3(p, "reported_variance")));
    } else if (kind == "reference_velocity") {
        sensors::ReferenceVelocityParameters velocity;
        velocity.mount = mount;
        velocity.noise = makeNoise(getOr(p, "velocity_noise", kEmpty), noise);
        assignVec3(velocity.reference_velocity_world, p, "reference_velocity_world_m_s");
        if (has(p, "reported_variance"))
            velocity.reported_variance = vec3(p.at("reported_variance"), "reported_variance");
        if (has(p, "inclination_limit")) {
            sensors::InclinationLimit limit;
            const Json &l = p.at("inclination_limit");
            assignVec3(limit.sensor_axis, l, "axis");
            assignNumber(limit.maximum_angle, l, "maximum_angle_rad");
            velocity.inclination_limit = limit;
        }
        runtime.add(device, sensors::ReferenceVelocity(velocity));
    } else if (kind == "reference_altitude") {
        sensors::ReferenceAltitudeParameters altitude;
        altitude.mount = mount;
        altitude.noise = makeScalarNoise(getOr(p, "noise", kEmpty), noise);
        if (has(p, "target_position_body_m"))
            altitude.target_position_body = vec3(p.at("target_position_body_m"), "target_position_body_m");
        if (has(p, "reported_variance"))
            altitude.reported_variance = num(p.at("reported_variance"), "reported_variance");
        runtime.add(device, sensors::ReferenceAltitude(altitude));
    } else if (kind == "dvl") {
        sensors::DvlParameters dvl;
        dvl.mount = mount;
        dvl.velocity_noise = makeNoise(getOr(p, "velocity_noise", kEmpty), noise);
        assignVec3(dvl.bottom_axis, p, "bottom_axis");
        assignNumber(dvl.minimum_range, p, "minimum_range_m");
        assignNumber(dvl.maximum_range, p, "maximum_range_m");
        runtime.add(device, sensors::Dvl(dvl, sensors::PoolBottom(pool)));
    } else if (kind == "pressure") {
        sensors::PressureParameters pressure;
        pressure.mount = mount;
        pressure.noise = makeScalarNoise(getOr(p, "noise", kEmpty), noise);
        assignNumber(pressure.reference_pressure, p, "reference_pressure_pa");
        assignNumber(pressure.reference_density, p, "reference_density_kg_m3");
        assignNumber(pressure.reference_gravity, p, "reference_gravity_m_s2");
        assignNumber(pressure.minimum_pressure, p, "minimum_pressure_pa");
        assignNumber(pressure.maximum_pressure, p, "maximum_pressure_pa");
        runtime.add(device, sensors::Pressure(pressure, sensors::HydrostaticPressure(
                                                            pool.water_level, pool.water_density,
                                                            surface_pressure)));
    } else {
        throw std::invalid_argument("sensor " + repr(device.id) +
                                    ": no native physics implementation for " + repr(kind));
    }
}
} // namespace

PackRuntime createRuntime(const ResolvedScenario &resolved, const std::vector<std::string> *sensor_ids) {
    const Json &robot = resolved.robot, &world = resolved.pool, &scenario = resolved.scenario;
    simulation::PlantParameters parameters;
    const Json &body = at(at(robot, "body"), "parameters");
    auto &b = parameters.body;
    assignNumber(b.mass, body, "mass_kg");
    if (has(body, "inertia_matrix"))
        b.inertia = matrix(body.at("inertia_matrix"), 3, "inertia_matrix");
    if (has(body, "added_mass_matrix"))
        b.added_mass = matrix(body.at("added_mass_matrix"), 6, "added_mass_matrix");
    if (has(body, "linear_damping_matrix"))
        b.linear_damping = matrix(body.at("linear_damping_matrix"), 6, "linear_damping_matrix");
    if (has(body, "quadratic_damping"))
        b.quadratic_damping = vec(body.at("quadratic_damping"), 6, "quadratic_damping");
    assignVec3(b.damping_center, body, "damping_center_m");
    assignNumber(b.displaced_volume, body, "displaced_volume_m3");
    assignVec3(b.buoyancy_center, body, "buoyancy_center_m");
    assignVec3(b.buoyancy_radii, body, "buoyancy_radii_m");
    parameters.command_timeout = num(at(body, "command_timeout_s"), "command_timeout_s");
    parameters.timestep = std::chrono::nanoseconds(at(scenario, "timestep_ns").get<std::int64_t>());

    simulation::Pool pool;
    const Json &wp = at(world, "parameters");
    assignNumber(pool.length, wp, "length_m");
    assignNumber(pool.width, wp, "width_m");
    assignNumber(pool.depth, wp, "depth_m");
    assignNumber(pool.water_level, wp, "water_level_m");
    assignNumber(pool.water_density, wp, "water_density_kg_m3");
    assignVec3(pool.current_velocity, wp, "current_m_s");
    assignVec3(pool.current_oscillation_amplitude, wp, "current_oscillation_amplitude_m_s");
    assignNumber(pool.current_oscillation_frequency, wp, "current_oscillation_frequency_hz");
    const Json &placement = at(scenario, "pool_placement");
    const Eigen::Vector3d placement_position = vec3(at(placement, "position_m"), "position_m");
    pool.yaw_world = radians(num(at(placement, "yaw_deg"), "yaw_deg"));
    pool.origin_xy_world = placement_position.head<2>();
    pool.water_level += placement_position[2];
    parameters.pool = pool;

    for (const auto &entry : at(robot, "thrusters")) {
        const Json &tp = at(entry, "parameters");
        // {**entry, **entry["parameters"]}: parameters win.
        const auto field = [&](const char *key) -> const Json * {
            if (has(tp, key))
                return &tp.at(key);
            if (has(entry, key))
                return &entry.at(key);
            return nullptr;
        };
        simulation::Thruster t;
        const auto number = [&](const char *key, double &target) {
            if (const Json *value = field(key))
                target = num(*value, key);
        };
        if (const Json *value = field("id"))
            t.id = value->get<std::string>();
        if (const Json *value = field("position_m"))
            t.position = vec3(*value, "position_m");
        if (const Json *value = field("direction"))
            t.direction = vec3(*value, "direction");
        number("delay_s", t.delay);
        number("rise_time_s", t.rise_time);
        number("fall_time_s", t.fall_time);
        number("slew_rate_n_s", t.slew_rate);
        number("forward_limit_n", t.forward_limit);
        number("reverse_limit_n", t.reverse_limit);
        number("deadband_n", t.deadband);
        number("forward_scale", t.forward_scale);
        number("reverse_scale", t.reverse_scale);
        number("efficiency", t.efficiency);
        if (const Json *value = field("propeller_radius_m"))
            t.propeller_radius = value->is_null() ? std::nullopt
                                                  : std::optional<double>(num(*value, "propeller_radius_m"));
        parameters.thrusters.push_back(std::move(t));
    }

    simulation::ContactParameters contacts;
    const Json &cm = at(scenario, "contacts");
    const std::string model = at(cm, "model").get<std::string>();
    if (model == "disabled")
        contacts.model = simulation::ContactModel::Disabled;
    else if (model == "sphere_pool")
        contacts.model = simulation::ContactModel::SpherePool;
    else if (model == "box_scene")
        contacts.model = simulation::ContactModel::BoxScene;
    else
        throw std::invalid_argument("unknown contact model " + repr(model));
    contacts.restitution = num(at(cm, "restitution"), "restitution");
    contacts.friction = num(at(cm, "friction"), "friction");
    for (const auto &box : at(robot, "collision_boxes"))
        contacts.body_boxes.push_back(makeBox(box, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()));
    const double half_yaw = parameters.pool.yaw_world / 2;
    const Eigen::Quaterniond pool_quaternion(std::cos(half_yaw), 0, 0, std::sin(half_yaw));
    for (const auto &box : at(world, "collision_boxes"))
        contacts.world_boxes.push_back(makeBox(box, placement_position, pool_quaternion));
    for (const auto &instance : at(scenario, "task_placements")) {
        const double yaw = radians(num(at(instance, "yaw_deg"), "yaw_deg")) / 2;
        const Eigen::Quaterniond quaternion(std::cos(yaw), 0, 0, std::sin(yaw));
        const std::string task_id = at(instance, "task").get<std::string>();
        const Json &definition = resolved.task(task_id);
        for (const auto &prop : at(definition, "props")) {
            const std::string type = at(prop, "type").get<std::string>();
            if (type == "rigid_body" || type == "contact_world")
                continue; // owned by the task's prop contact world
            if (type != "static_body")
                throw std::invalid_argument("prop " + repr(at(prop, "id").get<std::string>()) +
                                            ": no native contact implementation for " + repr(type));
            for (const auto &box_definition : at(at(prop, "parameters"), "collision_boxes")) {
                auto box = makeBox(box_definition, vec3(at(instance, "position_m"), "position_m"), quaternion);
                box.id = task_id + "/" + at(prop, "id").get<std::string>() + "/" + box.id;
                contacts.world_boxes.push_back(std::move(box));
            }
        }
    }
    parameters.contacts = contacts;

    auto frames = std::make_shared<const spatial::FixedFrames>(makeFrames(at(robot, "frames")));
    simulation::BodyState initial;
    const Json &start = at(scenario, "initial");
    assignVec3(initial.position, start, "position_m");
    if (has(start, "orientation_wxyz"))
        initial.orientation = quat(start.at("orientation_wxyz"), "orientation_wxyz");
    assignVec3(initial.linear_velocity, start, "linear_velocity_m_s");
    assignVec3(initial.angular_velocity, start, "angular_velocity_rad_s");
    const std::string initial_frame = at(start, "frame").get<std::string>();
    if (initial_frame != frames->root()) {
        const auto com_pose = composeChecked({initial.position, initial.orientation},
                                             inverseChecked(frames->fromRoot(initial_frame)));
        initial.position = com_pose.translation;
        initial.orientation = com_pose.rotation;
    }

    PackRuntime result;
    result.runtime = std::make_unique<sensors::Runtime>(parameters, initial,
                                                        at(scenario, "seed").get<std::uint64_t>());
    std::map<std::string, const Json *> configured;
    std::vector<std::string> order;
    for (const auto &entry : at(robot, "sensors")) {
        const std::string id = at(entry, "id").get<std::string>();
        configured[id] = &entry;
        order.push_back(id);
    }
    const auto enabled = [](const Json &entry) {
        return !has(entry, "enabled") || entry.at("enabled").get<bool>();
    };
    std::vector<std::string> selected;
    if (sensor_ids) {
        selected = *sensor_ids;
    } else {
        for (const auto &id : order) {
            const Json &entry = *configured.at(id);
            if (enabled(entry) && at(entry, "type").get<std::string>().find("camera") == std::string::npos)
                selected.push_back(id);
        }
    }
    const std::set<std::string> unique(selected.begin(), selected.end());
    bool known = unique.size() == selected.size();
    for (const auto &id : selected)
        known = known && configured.count(id);
    if (!known)
        throw std::invalid_argument("selected sensors must have unique ids from the robot pack");
    const double surface_pressure = num(at(wp, "surface_pressure_pa"), "surface_pressure_pa");
    const bool noise = at(scenario, "sensor_noise").get<bool>();
    for (const auto &id : selected) {
        const Json &config = *configured.at(id);
        if (!enabled(config))
            throw std::invalid_argument("selected sensor " + repr(id) + " is disabled in the robot pack");
        addSensor(*result.runtime, config, *frames, pool, surface_pressure, noise);
        result.sensor_types[id] = at(config, "type").get<std::string>();
    }
    result.sensor_ids = selected;
    for (const auto &id : order)
        if (!unique.count(id))
            result.deferred_sensor_ids.push_back(id);
    result.parameters = std::move(parameters);
    result.frames = std::move(frames);
    result.initial = initial;
    return result;
}
} // namespace robotics::session

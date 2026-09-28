#include "robotics/sensors/models.hpp"
#include "yaml_helpers.hpp"
#include <map>

namespace robotics::config::detail {
namespace {
using Attach = decltype(SensorPlan::attach);
using Decode =
    std::function<Attach(const sensors::Mount &, const YAML::Node &, const std::string &)>;

sensors::NoiseParameters noise(const YAML::Node &node, const std::string &field) {
    sensors::NoiseParameters result;
    if (!node.IsDefined()) {
        return result;
    }
    keys(node, {"bias", "white_stddev", "walk_stddev"}, field);
    if (node["bias"]) {
        result.bias = vector(node, "bias", 3, field);
    }
    if (node["white_stddev"]) {
        result.white_stddev = vector(node, "white_stddev", 3, field);
    }
    if (node["walk_stddev"]) {
        result.walk_stddev = vector(node, "walk_stddev", 3, field);
    }
    return result;
}
sensors::Nanoseconds duration(const YAML::Node &node, const char *key, const std::string &field) {
    const auto value = integer(node, key, field);
    if (value > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
        throw std::invalid_argument(field + "." + key + ": duration overflows nanoseconds");
    }
    return sensors::Nanoseconds(value);
}
Attach imu(const sensors::Mount &mount, const YAML::Node &node, const std::string &field) {
    keys(node, {"acceleration_noise", "gyro_noise"}, field);
    const sensors::Imu model(mount,
                             noise(node["acceleration_noise"], field + ".acceleration_noise"),
                             noise(node["gyro_noise"], field + ".gyro_noise"));
    return [model](auto &runtime, const auto &device, const auto &, double) {
        runtime.add(device, model);
    };
}
Attach fog(const sensors::Mount &mount, const YAML::Node &node, const std::string &field) {
    keys(node, {"axes", "gyro_noise"}, field);
    const auto list = node["axes"];
    if (!list.IsSequence() || list.size() == 0 || list.size() > 3) {
        throw std::invalid_argument(field + ".axes must contain one to three axes");
    }
    std::vector<Eigen::Vector3d> axes;
    for (std::size_t i = 0; i < list.size(); ++i) {
        YAML::Node wrapper;
        wrapper["axis"] = list[i];
        axes.push_back(vector(wrapper, "axis", 3, field + ".axes[" + std::to_string(i) + "]"));
    }
    const sensors::Fog model(mount, axes, noise(node["gyro_noise"], field + ".gyro_noise"));
    return [model](auto &runtime, const auto &device, const auto &, double) {
        runtime.add(device, model);
    };
}
Attach dvl(const sensors::Mount &mount, const YAML::Node &node, const std::string &field) {
    keys(node, {"bottom_axis", "minimum_range_m", "maximum_range_m", "velocity_noise"}, field);
    sensors::DvlParameters parameters;
    parameters.mount = mount;
    if (node["bottom_axis"]) {
        parameters.bottom_axis = vector(node, "bottom_axis", 3, field);
    }
    if (node["minimum_range_m"]) {
        parameters.minimum_range = number(node, "minimum_range_m", field);
    }
    if (node["maximum_range_m"]) {
        parameters.maximum_range = number(node, "maximum_range_m", field);
    }
    parameters.velocity_noise = noise(node["velocity_noise"], field + ".velocity_noise");
    return [parameters](auto &runtime, const auto &device, const auto &pool, double) {
        runtime.add(device, sensors::Dvl(parameters, sensors::PoolBottom(pool)));
    };
}
Attach pressure(const sensors::Mount &mount, const YAML::Node &node, const std::string &field) {
    keys(node,
         {"noise", "reference_pressure_pa", "reference_density_kg_m3", "reference_gravity_m_s2",
          "minimum_pressure_pa", "maximum_pressure_pa"},
         field);
    sensors::PressureParameters p;
    p.mount = mount;
    if (node["noise"]) {
        const auto n = node["noise"];
        keys(n, {"bias", "white_stddev", "walk_stddev"}, field + ".noise");
        if (n["bias"]) {
            p.noise.bias = number(n, "bias", field + ".noise");
        }
        if (n["white_stddev"]) {
            p.noise.white_stddev = number(n, "white_stddev", field + ".noise");
        }
        if (n["walk_stddev"]) {
            p.noise.walk_stddev = number(n, "walk_stddev", field + ".noise");
        }
    }
    if (node["reference_pressure_pa"]) {
        p.reference_pressure = number(node, "reference_pressure_pa", field);
    }
    if (node["reference_density_kg_m3"]) {
        p.reference_density = number(node, "reference_density_kg_m3", field);
    }
    if (node["reference_gravity_m_s2"]) {
        p.reference_gravity = number(node, "reference_gravity_m_s2", field);
    }
    if (node["minimum_pressure_pa"]) {
        p.minimum_pressure = number(node, "minimum_pressure_pa", field);
    }
    if (node["maximum_pressure_pa"]) {
        p.maximum_pressure = number(node, "maximum_pressure_pa", field);
    }
    return [p](auto &runtime, const auto &device, const auto &pool, double atmosphere) {
        runtime.add(device,
                    sensors::Pressure(p, sensors::HydrostaticPressure(
                                             pool.water_level, pool.water_density, atmosphere)));
    };
}
} // namespace

SensorPlan parseSensor(const YAML::Node &node, const std::filesystem::path &declaring,
                       const std::string &field, std::vector<std::filesystem::path> &sources) {
    keys(node,
         {"id", "frame", "period_ns", "latency_ns", "capacity", "overflow", "mount", "profile",
          "model", "parameters"},
         field);
    SensorPlan result;
    auto &device = result.device;
    device.id = text(node, "id", field);
    device.frame = text(node, "frame", field);
    device.period = duration(node, "period_ns", field);
    if (node["latency_ns"]) {
        device.latency = duration(node, "latency_ns", field);
    }
    if (node["capacity"]) {
        const auto capacity = integer(node, "capacity", field);
        if (capacity > std::numeric_limits<std::size_t>::max()) {
            throw std::invalid_argument(field + ".capacity overflow");
        }
        device.capacity = static_cast<std::size_t>(capacity);
    }
    if (node["overflow"]) {
        const auto policy = text(node, "overflow", field);
        if (policy == "drop_oldest") {
            device.overflow = sensors::OverflowPolicy::DropOldest;
        } else if (policy != "fail") {
            throw std::invalid_argument(field + ".overflow must be fail or drop_oldest");
        }
    }
    const auto m = node["mount"];
    keys(m, {"position_m", "orientation_wxyz"}, field + ".mount");
    sensors::Mount mount;
    mount.position_body = vector(m, "position_m", 3, field + ".mount");
    const auto q = vector(m, "orientation_wxyz", 4, field + ".mount");
    mount.sensor_to_body = Eigen::Quaterniond(q[0], q[1], q[2], q[3]);
    YAML::Node definition = node;
    std::string definition_field = field;
    if (node["profile"]) {
        if (node["model"] || node["parameters"]) {
            throw std::invalid_argument(field +
                                        ": profile cannot be combined with model/parameters");
        }
        const auto profile = reference(node, "profile", declaring, "sensor", sources);
        keys(profile.root, {"schema_version", "kind", "model", "parameters"},
             profile.path.string());
        definition.reset(profile.root);
        definition_field = profile.path.string();
    }
    const auto model = text(definition, "model", definition_field);
    result.model = model;
    // Configuration-edge registry only; no family enum or branch in the runtime.
    const std::map<std::string, Decode> decoders{
        {"imu", imu}, {"fog", fog}, {"dvl", dvl}, {"pressure", pressure}};
    const auto decoder = decoders.find(model);
    if (decoder == decoders.end()) {
        throw std::invalid_argument(definition_field + ": unsupported sensor model " + model);
    }
    try {
        result.attach =
            decoder->second(mount, definition["parameters"], definition_field + ".parameters");
    } catch (const std::exception &e) {
        throw std::invalid_argument(definition_field + ": " + e.what());
    }
    return result;
}
} // namespace robotics::config::detail

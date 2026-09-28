#include "yaml_helpers.hpp"

namespace robotics::config {
namespace {
using namespace detail;
void parseVehicle(const YAML::Node &v, simulation::PlantParameters &plant,
                  const std::string &path) {
    keys(v,
         {"mass_kg", "inertia_diagonal", "added_mass_diagonal", "linear_damping_diagonal",
          "quadratic_damping", "displaced_volume_m3", "buoyancy_center_m", "buoyancy_radii_m",
          "collision_radius_m", "command_timeout_s", "thrusters", "inertia_matrix",
          "added_mass_matrix", "linear_damping_matrix", "damping_center_m"},
         path);
    auto &b = plant.body;
    b.mass = number(v, "mass_kg", path);
    b.inertia = matrixOrDiagonal(v, "inertia_diagonal", "inertia_matrix", 3, path);
    b.added_mass = matrixOrDiagonal(v, "added_mass_diagonal", "added_mass_matrix", 6, path);
    b.linear_damping =
        matrixOrDiagonal(v, "linear_damping_diagonal", "linear_damping_matrix", 6, path);
    if (v["damping_center_m"]) {
        b.damping_center = vector(v, "damping_center_m", 3, path);
    }
    b.quadratic_damping = vector(v, "quadratic_damping", 6, path);
    b.displaced_volume = number(v, "displaced_volume_m3", path);
    b.buoyancy_center = vector(v, "buoyancy_center_m", 3, path);
    b.buoyancy_radii = vector(v, "buoyancy_radii_m", 3, path);
    b.collision_radius = number(v, "collision_radius_m", path);
    plant.command_timeout = number(v, "command_timeout_s", path);
    const auto thrusters = v["thrusters"];
    if (!thrusters.IsSequence()) {
        throw std::invalid_argument(path + ".thrusters must be a sequence (may be empty)");
    }
    for (std::size_t i = 0; i < thrusters.size(); ++i) {
        const auto t = thrusters[i];
        const std::string field = path + ".thrusters[" + std::to_string(i) + "]";
        keys(t,
             {"id", "position_m", "direction", "delay_s", "rise_time_s", "fall_time_s",
              "slew_rate_n_s", "forward_limit_n", "reverse_limit_n", "propeller_radius_m"},
             field);
        simulation::Thruster thruster;
        thruster.id = text(t, "id", field);
        thruster.position = vector(t, "position_m", 3, field);
        thruster.direction = vector(t, "direction", 3, field);
        thruster.delay = number(t, "delay_s", field);
        thruster.rise_time = number(t, "rise_time_s", field);
        thruster.fall_time = number(t, "fall_time_s", field);
        thruster.slew_rate = number(t, "slew_rate_n_s", field);
        thruster.forward_limit = number(t, "forward_limit_n", field);
        thruster.reverse_limit = number(t, "reverse_limit_n", field);
        if (t["propeller_radius_m"])
            thruster.propeller_radius = number(t, "propeller_radius_m", field);
        plant.thrusters.push_back(thruster);
    }
}
void parsePool(const YAML::Node &pool, simulation::Pool &result, const std::string &path) {
    keys(pool,
         {"length_m", "width_m", "depth_m", "water_level_m", "water_density_kg_m3", "current_m_s",
          "current_oscillation_amplitude_m_s", "current_oscillation_frequency_hz"},
         path);
    auto &p = result;
    p.length = number(pool, "length_m", path);
    p.width = number(pool, "width_m", path);
    p.depth = number(pool, "depth_m", path);
    p.water_level = number(pool, "water_level_m", path);
    p.water_density = number(pool, "water_density_kg_m3", path);
    p.current_velocity = vector(pool, "current_m_s", 3, path);
    if (pool["current_oscillation_amplitude_m_s"])
        p.current_oscillation_amplitude =
            vector(pool, "current_oscillation_amplitude_m_s", 3, path);
    if (pool["current_oscillation_frequency_hz"])
        p.current_oscillation_frequency = number(pool, "current_oscillation_frequency_hz", path);
}
void parseInitial(const YAML::Node &initial, simulation::BodyState &result,
                  const std::string &path) {
    keys(initial,
         {"position_m", "orientation_wxyz", "linear_velocity_m_s", "angular_velocity_rad_s"}, path);
    result.position = vector(initial, "position_m", 3, path);
    const auto q = vector(initial, "orientation_wxyz", 4, path);
    result.orientation = Eigen::Quaterniond(q[0], q[1], q[2], q[3]);
    result.linear_velocity = vector(initial, "linear_velocity_m_s", 3, path);
    result.angular_velocity = vector(initial, "angular_velocity_rad_s", 3, path);
}
void parseCommands(const YAML::Node &root, Scenario &s) {
    const auto commands = root["commands"];
    if (!commands.IsSequence()) {
        throw std::invalid_argument("commands must be a sequence (may be empty)");
    }
    for (std::size_t i = 0; i < commands.size(); ++i) {
        const auto command = commands[i];
        const auto field = "commands[" + std::to_string(i) + "]";
        keys(command, {"tick", "forces_n"}, field);
        const auto tick = integer(command, "tick", field);
        if (tick >= s.ticks || (!s.commands.empty() && tick <= s.commands.back().tick)) {
            throw std::invalid_argument(field +
                                        ".tick must increase strictly and be less than ticks");
        }
        s.commands.push_back(
            {tick, vector(command, "forces_n", static_cast<Eigen::Index>(s.plant.thrusters.size()),
                          field)});
    }
}
Scenario parse(const Document &document, std::vector<std::filesystem::path> sources) {
    const auto &root = document.root;
    const auto version = integer(root, "schema_version", "scenario");
    Scenario s;
    s.sources = std::move(sources);
    if (version == 1) {
        keys(root,
             {"schema_version", "timestep_ns", "ticks", "vehicle", "pool", "initial", "commands"},
             "scenario");
        parseVehicle(root["vehicle"], s.plant, "vehicle");
        parsePool(root["pool"], s.plant.pool, "pool");
    } else if (version == 2) {
        keys(root,
             {"schema_version", "timestep_ns", "ticks", "seed", "robot", "world", "initial",
              "commands"},
             "scenario");
        s.seed = integer(root, "seed", "scenario");
        const auto robot = reference(root, "robot", document.path, "robot", s.sources);
        keys(robot.root, {"schema_version", "kind", "vehicle", "sensors"}, robot.path.string());
        parseVehicle(robot.root["vehicle"], s.plant, robot.path.string() + ": vehicle");
        const auto world = reference(root, "world", document.path, "world", s.sources);
        keys(world.root, {"schema_version", "kind", "pool", "surface_pressure_pa"},
             world.path.string());
        parsePool(world.root["pool"], s.plant.pool, world.path.string() + ": pool");
        s.surface_pressure = number(world.root, "surface_pressure_pa", world.path.string());
        if (s.surface_pressure <= 0) {
            throw std::invalid_argument(world.path.string() +
                                        ": surface_pressure_pa must be positive");
        }
        const auto devices = robot.root["sensors"];
        if (!devices.IsSequence()) {
            throw std::invalid_argument(robot.path.string() +
                                        ": sensors must be a sequence (may be empty)");
        }
        for (std::size_t index = 0; index < devices.size(); ++index) {
            s.sensors.push_back(parseSensor(
                devices[index], robot.path,
                robot.path.string() + ": sensors[" + std::to_string(index) + "]", s.sources));
        }
    } else {
        throw std::invalid_argument("scenario.schema_version must be 1 or 2");
    }
    const auto step = integer(root, "timestep_ns", "scenario");
    if (step == 0 || step > 100'000'000) {
        throw std::invalid_argument("scenario.timestep_ns must be in [1, 100000000]");
    }
    s.plant.timestep = std::chrono::nanoseconds(step);
    s.ticks = integer(root, "ticks", "scenario");
    if (s.ticks > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) / step) {
        throw std::invalid_argument("scenario.ticks overflows elapsed nanoseconds");
    }
    parseInitial(root["initial"], s.initial, "initial");
    parseCommands(root, s);
    makeRuntime(s); // Validate physical and sensor invariants before returning resolved data.
    return s;
}
} // namespace
Scenario loadScenario(const std::filesystem::path &path) {
    try {
        std::vector<std::filesystem::path> sources;
        const auto document = detail::read(path, sources);
        return parse(document, std::move(sources));
    } catch (const std::exception &e) {
        throw std::invalid_argument(path.string() + ": " + e.what());
    }
}
std::unique_ptr<sensors::Runtime> makeRuntime(const Scenario &scenario) {
    auto runtime =
        std::make_unique<sensors::Runtime>(scenario.plant, scenario.initial, scenario.seed);
    for (const auto &sensor : scenario.sensors) {
        if (!sensor.attach) {
            throw std::invalid_argument("sensor plan has no factory: " + sensor.device.id);
        }
        try {
            sensor.attach(*runtime, sensor.device, scenario.plant.pool, scenario.surface_pressure);
        } catch (const std::exception &e) {
            throw std::invalid_argument("sensor " + sensor.device.id + ": " + e.what());
        }
    }
    return runtime;
}
} // namespace robotics::config

#include "robotics/config/scenario.hpp"
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>

namespace robotics::config {
namespace {
void keys(const YAML::Node &node, std::initializer_list<const char *> allowed,
          const std::string &path) {
    if (!node.IsMap()) {
        throw std::invalid_argument(path + " must be a mapping");
    }
    const std::set<std::string> permitted(allowed.begin(), allowed.end());
    std::set<std::string> seen;
    for (const auto &item : node) {
        const auto key = item.first.as<std::string>();
        if (!permitted.count(key) || !seen.insert(key).second) {
            throw std::invalid_argument(path + "." + key + ": unknown or duplicate field");
        }
    }
}

double number(const YAML::Node &n, const char *key, const std::string &path) {
    try {
        const double value = n[key].as<double>();
        if (!std::isfinite(value)) {
            throw std::invalid_argument("must be finite");
        }
        return value;
    } catch (const std::exception &e) {
        throw std::invalid_argument(path + "." + key + ": " + e.what());
    }
}

std::uint64_t integer(const YAML::Node &n, const char *key, const std::string &path) {
    try {
        const auto text = n[key].as<std::string>();
        if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos) {
            throw std::invalid_argument("expected a nonnegative decimal integer");
        }
        return std::stoull(text);
    } catch (const std::exception &e) {
        throw std::invalid_argument(path + "." + key + ": " + e.what());
    }
}

Eigen::VectorXd vector(const YAML::Node &n, const char *key, Eigen::Index size,
                       const std::string &path) {
    const auto list = n[key];
    if (!list.IsSequence() || list.size() != static_cast<std::size_t>(size)) {
        throw std::invalid_argument(path + "." + key + ": expected " + std::to_string(size) +
                                    " values");
    }
    Eigen::VectorXd values(size);
    for (Eigen::Index i = 0; i < size; ++i) {
        try {
            values[i] = list[static_cast<std::size_t>(i)].as<double>();
        } catch (const YAML::Exception &e) {
            throw std::invalid_argument(path + "." + key + ": " + e.what());
        }
    }
    if (!values.allFinite()) {
        throw std::invalid_argument(path + "." + key + ": values must be finite");
    }
    return values;
}

Scenario parse(const YAML::Node &root) {
    keys(root, {"schema_version", "timestep_ns", "ticks", "vehicle", "pool", "initial", "commands"},
         "scenario");
    if (integer(root, "schema_version", "scenario") != 1) {
        throw std::invalid_argument("scenario.schema_version must be 1");
    }
    Scenario s;
    const auto step = integer(root, "timestep_ns", "scenario");
    if (step == 0 || step > 100'000'000) {
        throw std::invalid_argument("scenario.timestep_ns must be in [1, 100000000]");
    }
    s.plant.timestep = std::chrono::nanoseconds(step);
    s.ticks = integer(root, "ticks", "scenario");
    if (s.ticks > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) / step) {
        throw std::invalid_argument("scenario.ticks overflows elapsed nanoseconds");
    }
    const auto v = root["vehicle"];
    keys(v,
         {"mass_kg", "inertia_diagonal", "added_mass_diagonal", "linear_damping_diagonal",
          "quadratic_damping", "displaced_volume_m3", "buoyancy_center_m", "buoyancy_radii_m",
          "collision_radius_m", "command_timeout_s", "thrusters"},
         "vehicle");
    auto &b = s.plant.body;
    b.mass = number(v, "mass_kg", "vehicle");
    b.inertia = vector(v, "inertia_diagonal", 3, "vehicle").asDiagonal();
    b.added_mass = vector(v, "added_mass_diagonal", 6, "vehicle").asDiagonal();
    b.linear_damping = vector(v, "linear_damping_diagonal", 6, "vehicle").asDiagonal();
    b.quadratic_damping = vector(v, "quadratic_damping", 6, "vehicle");
    b.displaced_volume = number(v, "displaced_volume_m3", "vehicle");
    b.buoyancy_center = vector(v, "buoyancy_center_m", 3, "vehicle");
    b.buoyancy_radii = vector(v, "buoyancy_radii_m", 3, "vehicle");
    b.collision_radius = number(v, "collision_radius_m", "vehicle");
    s.plant.command_timeout = number(v, "command_timeout_s", "vehicle");
    const auto thrusters = v["thrusters"];
    if (!thrusters.IsSequence()) {
        throw std::invalid_argument("vehicle.thrusters must be a sequence (may be empty)");
    }
    for (std::size_t i = 0; i < thrusters.size(); ++i) {
        const auto t = thrusters[i];
        const std::string field = "vehicle.thrusters[" + std::to_string(i) + "]";
        keys(t,
             {"id", "position_m", "direction", "delay_s", "rise_time_s", "fall_time_s",
              "slew_rate_n_s", "forward_limit_n", "reverse_limit_n"},
             field);
        simulation::Thruster thruster;
        thruster.id = t["id"].as<std::string>();
        thruster.position = vector(t, "position_m", 3, field);
        thruster.direction = vector(t, "direction", 3, field);
        thruster.delay = number(t, "delay_s", field);
        thruster.rise_time = number(t, "rise_time_s", field);
        thruster.fall_time = number(t, "fall_time_s", field);
        thruster.slew_rate = number(t, "slew_rate_n_s", field);
        thruster.forward_limit = number(t, "forward_limit_n", field);
        thruster.reverse_limit = number(t, "reverse_limit_n", field);
        s.plant.thrusters.push_back(thruster);
    }
    const auto pool = root["pool"];
    keys(pool,
         {"length_m", "width_m", "depth_m", "water_level_m", "water_density_kg_m3", "current_m_s"},
         "pool");
    auto &p = s.plant.pool;
    p.length = number(pool, "length_m", "pool");
    p.width = number(pool, "width_m", "pool");
    p.depth = number(pool, "depth_m", "pool");
    p.water_level = number(pool, "water_level_m", "pool");
    p.water_density = number(pool, "water_density_kg_m3", "pool");
    p.current_velocity = vector(pool, "current_m_s", 3, "pool");
    const auto initial = root["initial"];
    keys(initial,
         {"position_m", "orientation_wxyz", "linear_velocity_m_s", "angular_velocity_rad_s"},
         "initial");
    s.initial.position = vector(initial, "position_m", 3, "initial");
    const auto q = vector(initial, "orientation_wxyz", 4, "initial");
    s.initial.orientation = Eigen::Quaterniond(q[0], q[1], q[2], q[3]);
    s.initial.linear_velocity = vector(initial, "linear_velocity_m_s", 3, "initial");
    s.initial.angular_velocity = vector(initial, "angular_velocity_rad_s", 3, "initial");
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
    // Validate physical invariants using the same constructors as direct API users.
    simulation::Plant validated(s.plant, s.initial);
    return s;
}
} // namespace

Scenario loadScenario(const std::filesystem::path &path) {
    try {
        return parse(YAML::LoadFile(path.string()));
    } catch (const std::exception &e) {
        throw std::invalid_argument(path.string() + ": " + e.what());
    }
}
} // namespace robotics::config

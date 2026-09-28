#pragma once
#include "robotics/sensors/runtime.hpp"
#include <filesystem>
#include <functional>
#include <vector>

namespace robotics::config {
struct ScheduledCommand {
    std::uint64_t tick;
    Eigen::VectorXd forces;
};
// Resolved, typed model factory; no YAML objects survive loading. The supplied
// device and world values are authoritative at construction, not hidden globals.
struct SensorPlan {
    sensors::Device device;
    std::string model; // Configuration/export identifier; the runtime does not dispatch on it.
    std::function<void(sensors::Runtime &, const sensors::Device &, const simulation::Pool &,
                       double)>
        attach;
};
struct Scenario {
    simulation::PlantParameters plant;
    simulation::BodyState initial;
    std::uint64_t ticks = 0;
    std::vector<ScheduledCommand> commands;
    std::uint64_t seed = 0;
    double surface_pressure = 101325;
    std::vector<SensorPlan> sensors;
    std::vector<std::filesystem::path> sources; // Declaring files, in first-use order.
};
// v1: inline standalone plant. v2: local robot/world/sensor profile composition.
// Unknown/duplicate keys and malformed fields are rejected with a source path.
Scenario loadScenario(const std::filesystem::path &path);
std::unique_ptr<sensors::Runtime> makeRuntime(const Scenario &scenario);
} // namespace robotics::config

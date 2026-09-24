#pragma once
#include "robotics/simulation/plant.hpp"
#include <filesystem>
#include <vector>

namespace robotics::config {
struct ScheduledCommand {
    std::uint64_t tick;
    Eigen::VectorXd forces;
};
struct Scenario {
    simulation::PlantParameters plant;
    simulation::BodyState initial;
    std::uint64_t ticks = 0;
    std::vector<ScheduledCommand> commands;
};
// Schema v1 is a small standalone-run format, not the final pack composition schema.
// Unknown/duplicate keys and malformed fields are rejected with a source path.
Scenario loadScenario(const std::filesystem::path &path);
} // namespace robotics::config

#pragma once
#include "robotics/config/scenario.hpp"
#include <iosfwd>

namespace robotics::runner {
// Drain each sensor once per tick; null output deliberately discards observations.
std::vector<std::function<void()>> telemetry(sensors::Runtime &runtime,
                                             const std::vector<config::SensorPlan> &plans,
                                             std::ostream *output);
void sensorHeader(std::ostream &output);
} // namespace robotics::runner

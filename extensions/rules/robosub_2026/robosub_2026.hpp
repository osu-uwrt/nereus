#pragma once
// RoboSub 2026 competition rules (tasks pack robosub_2026, `rules: robosub_2026`).
#include <robotics/session/tasks.hpp>

#include <memory>

namespace robotics::rules {
std::unique_ptr<session::Rules> makeRobosub2026();
} // namespace robotics::rules

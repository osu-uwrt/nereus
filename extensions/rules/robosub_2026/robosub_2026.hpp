#pragma once
// RoboSub 2026 competition rules (tasks pack robosub_2026, `rules: robosub_2026`).
#include <nereus/session/tasks.hpp>

#include <memory>

namespace nereus::rules {
// Factory registered as "robosub_2026" in standardRules().
std::unique_ptr<session::Rules> makeRobosub2026();
} // namespace nereus::rules

#pragma once
// RoboSub 2026 competition rules: port of content/packs/tasks/robosub_2026/hooks/rules_2026.py.
#include <robotics/session/tasks.hpp>

#include <memory>

namespace robotics::rules {
std::unique_ptr<session::Rules> makeRobosub2026();
} // namespace robotics::rules

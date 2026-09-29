#pragma once
// Compiled rules available to task packs, by name. The composition root passes this registry to
// the session; nothing in libraries/ knows competition names.
#include <robotics/session/tasks.hpp>

namespace robotics::rules {
session::RulesRegistry standardRules();
} // namespace robotics::rules

#pragma once
// Compiled rules available to task packs, by name. The composition root passes this registry to
// the session; nothing in libraries/ knows competition names.
#include <nereus/session/tasks.hpp>

namespace nereus::rules {
// A fresh registry of every built-in rules module (name -> factory).
session::RulesRegistry standardRules();
} // namespace nereus::rules

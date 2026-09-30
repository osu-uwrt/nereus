#pragma once
// Compiled rules available to task packs, by name. The composition root passes this registry to
// the session; nothing in libraries/ knows competition names.
#include <nereus/session/tasks.hpp>

namespace nereus::rules {
session::RulesRegistry standardRules();
} // namespace nereus::rules

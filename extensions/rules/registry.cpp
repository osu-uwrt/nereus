#include "rules/registry.hpp"

#include "rules/robosub_2026/robosub_2026.hpp"

namespace robotics::rules {
session::RulesRegistry standardRules() {
    session::RulesRegistry registry;
    // Each rules module adds itself here.
    registry["robosub_2026"] = makeRobosub2026;
    return registry;
}
} // namespace robotics::rules

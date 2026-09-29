#include "rules/registry.hpp"

namespace robotics::rules {
session::RulesRegistry standardRules() {
    session::RulesRegistry registry;
    // Each rules module adds itself here, e.g. registry["robosub_2026"] = makeRobosub2026;
    return registry;
}
} // namespace robotics::rules

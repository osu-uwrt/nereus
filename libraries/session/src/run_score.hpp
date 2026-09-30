#pragma once
// Operator scorecard document, split out so it can be tested without a TaskRuntime.
#include <robotics/session/scenario.hpp>

#include <cstdint>
#include <string>

namespace robotics::session {
// task_pack: resolved `tasks` pack document; snapshot/extra: TaskRuntime::snapshot()/describe().
Json buildRunSnapshot(const Json &task_pack, const Json &snapshot, Json extra, std::int64_t now_ns, double adjustment,
                      const std::string &message);
} // namespace robotics::session

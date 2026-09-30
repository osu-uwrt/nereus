#pragma once
// Run records written next to resolved.json: execution.json, tasks.json, summary.json.
#include "core.hpp"

#include <filesystem>

namespace nereus::ros_bridge {
void writeJson(const std::filesystem::path &path, const Json &document);

Json executionRecord(const session::ResolvedScenario &resolved, const BridgeCore &core,
                     const std::vector<std::string> &sensors, const std::vector<std::string> &deferred,
                     std::optional<std::int64_t> duration_ns, const CameraSink *cameras);
Json tasksRecord(BridgeCore &core);
Json summaryRecord(BridgeCore &core, const std::string &reason, const CameraSink *cameras);
} // namespace nereus::ros_bridge

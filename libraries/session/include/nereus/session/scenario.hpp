#pragma once
// Resolved scenario as produced by the pack tool (`python -m nereus.packs`), i.e. the
// bridge's resolved.json layout plus `asset_paths`. Validation is the pack tool's job; this
// loader only checks the fields it reads. Pack documents stay JSON: robot/pool/task data is
// consumed by the component that owns it, never by name switches.
#include <nlohmann/json.hpp>

#include <filesystem>
#include <map>
#include <string>

namespace nereus::session {
using Json = nlohmann::json;

struct ResolvedScenario {
    Json scenario, robot, pool, tasks, bridge; // bridge is null when no bridge pack is selected
    Json equipment;                            // null when no equipment pack is selected
    Json task_definitions;                     // array of kind: task documents
    Json run_options;
    // role ("robot" | "pool" | "tasks" | "equipment") -> asset id -> absolute path (present assets only).
    std::map<std::string, std::map<std::string, std::filesystem::path>> asset_paths;
    Json document; // the complete source document, for records and the viewer's scenario topic

    const Json &task(const std::string &id) const; // throws std::out_of_range
    std::filesystem::path asset(const std::string &role, const std::string &id) const;
};

// Throws std::runtime_error with file/field context.
ResolvedScenario loadResolvedScenario(const std::filesystem::path &resolved_json);
ResolvedScenario parseResolvedScenario(const Json &document);
} // namespace nereus::session

#include <robotics/session/scenario.hpp>

#include <fstream>
#include <stdexcept>

namespace robotics::session {
namespace {
const Json &member(const Json &document, const char *key) {
    const auto found = document.find(key);
    if (found == document.end())
        throw std::runtime_error(std::string("resolved scenario: missing '") + key + "'");
    return *found;
}
} // namespace

const Json &ResolvedScenario::task(const std::string &id) const {
    for (const auto &definition : task_definitions)
        if (definition.at("id") == id)
            return definition;
    throw std::out_of_range("resolved scenario: unknown task '" + id + "'");
}

std::filesystem::path ResolvedScenario::asset(const std::string &role, const std::string &id) const {
    const auto pack = asset_paths.find(role);
    if (pack != asset_paths.end()) {
        const auto found = pack->second.find(id);
        if (found != pack->second.end())
            return found->second;
    }
    throw std::out_of_range("resolved scenario: asset '" + id + "' is not present in pack '" + role +
                            "'");
}

ResolvedScenario parseResolvedScenario(const Json &document) {
    if (!document.is_object() || member(document, "format") != "robotics_platform.resolved_scenario")
        throw std::runtime_error("resolved scenario: unexpected format (write it with "
                                 "`python -m robotics_platform.packs resolve`)");
    ResolvedScenario result;
    result.scenario = member(document, "scenario");
    result.robot = member(document, "robot");
    result.pool = member(document, "pool");
    result.tasks = member(document, "tasks");
    result.bridge = document.value("bridge", Json());
    result.task_definitions = member(document, "task_definitions");
    result.run_options = document.value("run_options", Json::object());
    if (!result.task_definitions.is_array())
        throw std::runtime_error("resolved scenario: task_definitions must be an array");
    for (const auto &[role, assets] : member(document, "asset_paths").items())
        for (const auto &[id, path] : assets.items()) {
            const std::filesystem::path absolute(path.get<std::string>());
            if (!absolute.is_absolute())
                throw std::runtime_error("resolved scenario: asset path of " + role + ":" + id +
                                         " is not absolute");
            result.asset_paths[role][id] = absolute;
        }
    result.document = document;
    return result;
}

ResolvedScenario loadResolvedScenario(const std::filesystem::path &resolved_json) {
    std::ifstream stream(resolved_json);
    if (!stream)
        throw std::runtime_error("resolved scenario: cannot read " + resolved_json.string());
    Json document;
    try {
        document = Json::parse(stream);
    } catch (const Json::parse_error &error) {
        throw std::runtime_error(resolved_json.string() + ": " + error.what());
    }
    try {
        return parseResolvedScenario(document);
    } catch (const std::exception &error) {
        throw std::runtime_error(resolved_json.string() + ": " + error.what());
    }
}
} // namespace robotics::session

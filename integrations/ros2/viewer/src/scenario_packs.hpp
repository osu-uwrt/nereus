// The scenario packs a pool can be switched to (View > Pool): content/packs/scenarios/*, each with its pool, and
// the pack resolver (Python's nereus.packs) to turn one into the document the viewer loads.
#pragma once
#include <filesystem>
#include <string>
#include <vector>

namespace nereus::ros_viewer::host {
// One entry of the View > Pool menu.
struct ScenarioPack {
    std::filesystem::path folder; // the pack folder (holds scenario.yaml)
    std::string scenarioId;       // the scenario's own id
    std::string robot;            // robot pack id
    std::string poolId, poolLabel, poolDescription;
};

// The packs' content folder (content/packs of the source tree this viewer was built from).
std::filesystem::path packContent();

// Every scenario pack in `scenarios` (default: packContent()/scenarios), by pool label. Unreadable ones are skipped.
std::vector<ScenarioPack> scenarioPacks(const std::filesystem::path &scenarios = {});

// A pool's menu label: its description up to the first comma, without a trailing full stop.
std::string poolLabel(const std::string &description, const std::string &id);

// Runs `python -m nereus.packs resolve <folder>` (the project's .venv when there is one) and returns the resolved
// JSON. Throws with the resolver's last line of output when it fails. Blocks for a few seconds.
std::string resolveScenarioPack(const std::filesystem::path &folder);

// Applies a course edit (`python -m nereus.packs set-course <folder> --course <edit>`: moved tasks, loose objects,
// fixed run options as JSON) to the scenario pack. The tools write it only when the edited scenario resolves;
// throws with their last line of output otherwise. Blocks for a few seconds.
void setScenarioCourse(const std::filesystem::path &folder, const std::string &edit);
} // namespace nereus::ros_viewer::host

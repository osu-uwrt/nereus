// Keyboard pose commands: what the command palette reads in "forward 0.5", "turn 30", "go 1 2 -1 90" or "z -1.5",
// and the steps of keyboard driving. Targets start from the pose the robot was last told to hold, so moves add up
// while it travels; forward / left run along its heading, level with the water.
#pragma once
#include <glm/glm.hpp>
#include <optional>
#include <string>

namespace nereus::ros_viewer::host {
// A pose to hold in the world (map) frame: meters and degrees.
struct PoseTarget {
    glm::vec3 position{0};
    glm::vec3 degrees{0}; // roll, pitch, yaw (yaw in (-180, 180])
};

// A parsed move: the resulting target plus a human-readable summary.
struct PoseCommand {
    PoseTarget target;
    std::string summary; // the move as read back: "forward 0.5 m, turn 30°"
};

// A move typed in the palette. Not a move (`isMove` false) unless the first word is one of:
//   forward / back / left / right / up / down <meters>     along the heading (up is +z)
//   turn [left | right] <degrees>                          yaw, left positive
//   x / y / z <meters>, roll / pitch / yaw <degrees>       absolute
//   go <x> <y> <z> [<yaw> | <roll> <pitch> <yaw>]          absolute
//   level                                                  roll and pitch to zero
// Several may follow each other ("x 2 yaw 90", "forward 1 turn -45"), applied in order. A move word without its
// numbers gives `error` (the palette shows how to finish it) and no command. Units ("0.5m", "30deg", "30°") are
// accepted and ignored.
struct ParsedMove {
    bool isMove = false;
    std::optional<PoseCommand> command;
    std::string error;
};
ParsedMove parseMove(const std::string &text, const PoseTarget &from);

// One keyboard-drive step: meters forward / left along the heading, up, and degrees of turn (left positive).
PoseTarget stepped(const PoseTarget &from, float forward, float left, float up, float turn);

// The target as the palette shows it: "x 1.20  y 0.40  z -1.00  yaw 120°" (roll / pitch when not level).
std::string describe(const PoseTarget &);
} // namespace nereus::ros_viewer::host

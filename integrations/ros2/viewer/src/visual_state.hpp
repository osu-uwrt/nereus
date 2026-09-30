// Dynamic drawable state handed to the scene model each frame (everything that is not static pack content).
#pragma once
#include "math.hpp"
#include <array>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace nereus::ros_viewer::host {
// A marker-driven mesh (task prop, projectile) or emissive box (magnet light) at a world pose.
struct MarkerDraw {
    std::filesystem::path mesh; // empty: unit box
    glm::mat4 world{1};
    glm::vec3 scale{1};
    glm::vec4 tint{1};
    float radiance = 0;
    bool emissive = false;
    bool ghost = false;        // translucent overlay (mapping estimate over the simulator course)
    bool observerOnly = false; // not seen by the robot's cameras (mapping course / ghosts)
};
struct VisualState {
    glm::mat4 body{1};                  // truth base_link in the fixed frame
    std::optional<glm::mat4> ghostBody; // simulator: the localization estimate, drawn as a translucent robot
    std::vector<glm::mat4> rotorSpin;   // parallel to ThrusterVisuals::rotors
    std::vector<glm::vec3> lightColor;  // parallel to StatusLights::lights
    std::array<float, 2> claw{0.f, 0.f};
    std::vector<MarkerDraw> markers;
    std::map<std::string, bool> indicatorLatched;              // task indicator region -> latched (magnet target LEDs)
    std::vector<glm::mat4> loadedPayloads;                     // world poses (body already applied), unit length scaled
    bool showBoard = true, showWalls = true, showFloor = true; // walls include the deck and coping
    bool showCourse = true; // the pack's task visuals (false: the course comes from mapping markers)
};
} // namespace nereus::ros_viewer::host

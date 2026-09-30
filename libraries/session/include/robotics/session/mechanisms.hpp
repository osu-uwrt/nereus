#pragma once
// Robot mechanisms compiled from robot-pack data:
// launcher / dropper (slot release with shared cooldown groups, spring launch), claw (timed jaw
// travel), magnet (passive). Arming guards and kill semantics come from robot.safety.
// Released bodies belong to the caller; reload and reset never touch them.
#include <robotics/session/scenario.hpp>
#include <robotics/spatial/frames.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cstdint>
#include <memory>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace robotics::session {
using Pose = spatial::Pose;

struct CommandResult {
    bool accepted{false};
    std::string message;
};

struct PayloadRelease {
    std::string mechanism_id, mechanism_type, slot_id;
    int slot_index{0};
    std::int64_t time_ns{0};
    Eigen::Vector3d position_world;           // visible mesh centre, NOT centre of mass
    Eigen::Quaterniond orientation_world;     // world from payload body
    Eigen::Vector3d velocity_com_world;
    Eigen::Vector3d angular_velocity_world;
    const Json *projectile{nullptr};          // robot-pack projectile parameters (owned by scenario)
};

struct ReleaseState {
    std::string state; // disarmed | busy | loaded | empty
    int available{0};
};
struct ClawState {
    std::string state; // disarmed | opening | closing | opened | closed
    double gap_m{0}, target_gap_m{0};
    std::array<double, 2> joint_positions_m{0, 0};
};
struct MechanismState {
    std::int64_t time_ns{0};
    bool armed{false}, any_busy{false};
    std::map<std::string, ReleaseState> releases;
    std::map<std::string, ClawState> claws;
};

class Mechanisms {
  public:
    explicit Mechanisms(const Json &robot); // throws on invalid mechanism data
    ~Mechanisms();
    Mechanisms(Mechanisms &&) noexcept;
    Mechanisms &operator=(Mechanisms &&) noexcept;
    void reset(bool killed);
    CommandResult setArmed(bool armed, bool killed);
    CommandResult reloadAll(bool killed);
    // Velocities are at the named reference-frame origin, in its axes. On success `release`
    // is filled; ammunition and cooldown change only then.
    CommandResult fire(const std::string &id, const Pose &world_from_reference,
                       const Eigen::Vector3d &linear_velocity_reference,
                       const Eigen::Vector3d &angular_velocity_reference,
                       const std::string &reference_frame, double water_density, bool killed,
                       PayloadRelease &release);
    CommandResult commandClaw(const std::string &id, bool open, bool killed);
    CommandResult moveClaw(const std::string &id, double signed_duration_s, bool killed);
    void advance(std::int64_t dt_ns, bool killed);
    MechanismState snapshot(bool killed) const;
    int slotCount(const std::string &id) const;
    Pose slotMount(const std::string &id, int index) const; // in the robot frame root
    const std::string &type(const std::string &id) const;
    std::vector<std::string> ids() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace robotics::session

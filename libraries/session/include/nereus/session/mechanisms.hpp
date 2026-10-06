#pragma once
// Robot mechanisms compiled from robot-pack data:
// launcher / dropper (slot release with shared cooldown groups, spring launch), claw (timed jaw
// travel), magnet (passive). Arming guards and kill semantics come from robot.safety.
// Released bodies belong to the caller; reload and reset never touch them.
#include <nereus/session/scenario.hpp>
#include <nereus/spatial/frames.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace nereus::session {
using Pose = spatial::Pose;

// Outcome of a mechanism command; `message` is the human-readable reason either way.
struct CommandResult {
    bool accepted{false};
    std::string message;
};

// A payload that just left a launcher/dropper slot, for the caller to spawn as a free body.
struct PayloadRelease {
    std::string mechanism_id, mechanism_type, slot_id;
    int slot_index{0}; // 0-based, in firing order
    std::int64_t time_ns{0};
    Eigen::Vector3d position_world;       // visible mesh center, NOT center of mass
    Eigen::Quaterniond orientation_world; // world from payload body
    Eigen::Vector3d velocity_com_world;
    Eigen::Vector3d angular_velocity_world;
    const Json *projectile{nullptr}; // robot-pack projectile parameters (owned by scenario)
};

// Per-mechanism state reported by Mechanisms::snapshot().
struct ReleaseState {
    std::string state; // disarmed | busy | loaded | empty
    int available{0};  // rounds left
};

struct ClawState {
    std::string state; // disarmed | opening | closing | opened | closed
    double gap_m{0}, target_gap_m{0};
    std::array<double, 2> joint_positions_m{0, 0}; // each jaw's opening from closed (gap = min gap + both)
};

struct MechanismState {
    std::int64_t time_ns{0};
    bool armed{false}, any_busy{false}; // any_busy: a cooldown is running or a claw is still moving
    std::map<std::string, ReleaseState> releases;
    std::map<std::string, ClawState> claws;
};

// Every mechanism of one robot, on its own nanosecond clock advanced by the session. Each command also takes
// the vehicle's current kill state and applies it first (a kill stops the claws and may disarm).
class Mechanisms {
  public:
    explicit Mechanisms(const Json &robot); // throws on invalid mechanism data
    ~Mechanisms();
    Mechanisms(Mechanisms &&) noexcept;
    Mechanisms &operator=(Mechanisms &&) noexcept;

    // Back to the start of a run: initial arming, full magazines, no cooldowns, claws at their initial state.
    void reset(bool killed);
    CommandResult setArmed(bool armed, bool killed);
    // Refill every magazine, clear cooldowns, and disarm.
    CommandResult reloadAll(bool killed);

    // Velocities are at the named reference-frame origin, in its axes. On success `release`
    // is filled; ammunition and cooldown change only then.
    CommandResult fire(const std::string &id, const Pose &world_from_reference,
                       const Eigen::Vector3d &linear_velocity_reference,
                       const Eigen::Vector3d &angular_velocity_reference, const std::string &reference_frame,
                       double water_density, bool killed, PayloadRelease &release);

    // Drive a claw fully open or closed.
    CommandResult commandClaw(const std::string &id, bool open, bool killed);
    // Drive a claw for |signed_duration_s| seconds: positive opens, otherwise closes; 0 stops it.
    CommandResult moveClaw(const std::string &id, double signed_duration_s, bool killed);
    // Step the mechanism clock by dt_ns (claw travel, timed moves, cooldowns).
    void advance(std::int64_t dt_ns, bool killed);
    // Current state; also applies the kill state, like every command (so it is not truly const).
    MechanismState snapshot(bool killed) const;

    // Static configuration queries.
    int slotCount(const std::string &id) const;
    Pose slotMount(const std::string &id, int index) const; // in the robot frame root
    const std::string &type(const std::string &id) const;
    std::vector<std::string> ids() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace nereus::session

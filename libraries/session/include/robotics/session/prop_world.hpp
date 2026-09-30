#pragma once
// Rigid-body prop contact world (Bullet) for one task with a `contact_world` prop.
// The plant stays authoritative for the robot; jaws are placed kinematically from the robot
// pose and claw joint positions. One private btDiscreteDynamicsWorld per instance.
#include <robotics/session/tasks.hpp>
#include <robotics/simulation/contacts.hpp>

#include <array>
#include <map>
#include <memory>
#include <string>

namespace robotics::session {
struct Water {
    Eigen::Vector3d velocity_world{Eigen::Vector3d::Zero()};
    double density{998.2};
};
struct PropState {
    Eigen::Vector3d position;       // mesh origin in world
    Eigen::Quaterniond orientation; // world from mesh
    bool attached{false};
    std::string mechanism_id; // holder when attached
    std::string basket;       // basket region id when resting in one
};

class PropWorld {
  public:
    PropWorld(const ResolvedScenario &scenario, const std::string &task, const std::string &mechanism_id = {});
    ~PropWorld();
    PropWorld(const PropWorld &) = delete;
    PropWorld &operator=(const PropWorld &) = delete;

    // robot_root_pose: pose of the robot frame-tree root (COM) in world; velocities of that
    // origin in world axes; claw joints from ClawState::joint_positions_m.
    Events step(double dt_s, std::int64_t time_ns, const spatial::Pose &robot_root_pose,
                const Eigen::Vector3d &linear_velocity_world, const Eigen::Vector3d &angular_velocity_world,
                const std::array<double, 2> &claw_joint_positions, const Water &water, bool enabled);
    void reset();
    const std::string &task() const;
    const std::string &mechanismId() const;
    std::map<std::string, PropState> props() const;
    std::map<std::string, std::string> basketContents() const;
    double jawPosition() const; // physical jaw travel (lags the mechanism while blocked)
    // Robot-side contacts for the plant: the claw pads and a
    // held prop move with the robot and push it back from this task's scenery, the pool boxes and
    // props resting on scenery. Updated by step()/reset(); install with Runtime::setContactResolver.
    // friction: Coulomb coefficient of the robot contacts (the plant's contact friction).
    std::shared_ptr<simulation::ContactResolver> vehicleContacts(double friction);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace robotics::session

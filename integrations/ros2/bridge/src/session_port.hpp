#pragma once
// The slice of robotics::session::Session the bridge core drives (see session.hpp), as a virtual
// port so the core is testable against a deterministic fake without the whole runtime.
// SessionAdapter (session_adapter.cpp) implements it over the real Session.
#include "native.hpp"

#include <robotics/sensors/types.hpp>
#include <robotics/session/session.hpp>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace robotics::ros_bridge {

struct SensorSample {
    std::string sensor;
    std::int64_t acquired_ns{0};
    std::optional<Value> reading; // nullopt: the sample was unavailable
};

struct StepResult {
    simulation::Snapshot snapshot;
    session::Events task_events;
};

struct PropVisual {
    std::string task, id;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation; // world from mesh
    bool held{false};
};
struct PayloadVisual {
    std::string mechanism_id, mechanism_type;
    int id{0};
    bool loaded{false};
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    double length_m{0}, radius_m{0};
};

class SessionPort {
  public:
    virtual ~SessionPort() = default;
    virtual StepResult advance() = 0; // exactly one physics tick
    virtual simulation::Snapshot observe() const = 0;
    virtual const simulation::Snapshot &lastSnapshot() const = 0;
    virtual std::int64_t timeNs() const = 0;
    virtual std::int64_t timestepNs() const = 0;
    virtual std::vector<SensorSample> drainSensors() = 0; // every executed sensor, id order
    virtual std::map<std::string, sensors::StreamStats> sensorStats() const = 0;
    virtual std::vector<std::string> sensorIds() const = 0;         // executed
    virtual std::vector<std::string> deferredSensorIds() const = 0; // not executed
    virtual std::shared_ptr<const spatial::FixedFrames> frames() const = 0;
    virtual spatial::Pose referencePose(const simulation::BodyState &body) const = 0;

    virtual void commandThrusters(const Eigen::VectorXd &forces_native_order) = 0;
    virtual void setKilled(bool killed) = 0;
    virtual bool killed() const = 0;
    virtual session::CommandResult setArmed(bool armed) = 0;
    virtual session::CommandResult reloadAll() = 0;
    virtual session::CommandResult commandClaw(const std::string &id, bool open) = 0;
    virtual session::CommandResult moveClaw(const std::string &id, double signed_duration_s) = 0;
    // Task events of the release are appended to `events`.
    virtual session::CommandResult fire(const std::string &id, session::Events &events) = 0;
    virtual std::optional<session::MechanismState> mechanismState() const = 0;

    // May throw std::invalid_argument for an invalid placement.
    virtual simulation::Snapshot place(const simulation::BodyState &com_state, bool clear_actuators) = 0;
    virtual const simulation::BodyState &startState() const = 0;
    virtual session::CommandResult resetTasks() = 0;
    virtual simulation::Snapshot fullReset() = 0;
    virtual std::uint64_t seed() const = 0;

    virtual session::CommandResult runStart(const Json &options) = 0;
    virtual session::CommandResult runStop() = 0;
    virtual session::CommandResult runAdjust(double points) = 0;
    virtual std::optional<Json> runSnapshot() const = 0;
    virtual Json takeFeed() = 0;
    virtual Json taskCounters() const = 0;
    virtual Eigen::VectorXd thrusterForces() const = 0;
    virtual std::map<std::string, std::array<double, 2>> clawJaws() const = 0;
    virtual Json indicators() const = 0;
    virtual std::vector<PropVisual> propVisuals() const = 0;
    virtual std::vector<PayloadVisual> payloadVisuals() const = 0;
};
} // namespace robotics::ros_bridge

#pragma once
// One coordinated simulation run. The only
// owner that advances the plant, sensors, mechanisms, payloads, prop worlds and task observers,
// in this fixed order per tick: plant step; mechanism actuation; payload propagation + task
// judging; prop worlds; robot task observation. No wall time, ROS or rendering here.
// Not thread safe: a transport serializes all calls on its stepping thread.
#include <nereus/sensors/runtime.hpp>
#include <nereus/session/mechanisms.hpp>
#include <nereus/session/prop_world.hpp>
#include <nereus/session/tasks.hpp>
#include <nereus/simulation/payload.hpp>
#include <nereus/simulation/plant.hpp>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace nereus::session {
// Plant + sensors built from pack data.
struct PackRuntime {
    std::unique_ptr<sensors::Runtime> runtime;
    simulation::PlantParameters parameters;
    std::shared_ptr<const spatial::FixedFrames> frames;
    simulation::BodyState initial;                            // COM frame
    std::vector<std::string> sensor_ids, deferred_sensor_ids; // executed / not executed
    std::map<std::string, std::string> sensor_types;          // executed sensor id -> pack type
};
// sensor_ids: explicit selection (nullptr = every enabled non-camera sensor).
PackRuntime createRuntime(const ResolvedScenario &scenario, const std::vector<std::string> *sensor_ids = nullptr);

struct Payload {
    int id{0};
    std::string mechanism_id, mechanism_type;
    simulation::PayloadState state;
    std::int64_t released_ns{0};
    bool active{true};
    std::string outcome;
};

struct Step {
    simulation::Snapshot snapshot;
    Events task_events; // everything produced this tick, in order
};

struct SessionOptions {
    const std::vector<std::string> *task_ids{nullptr}; // subset of the scenario's tasks
    bool tasks{true};                                  // false: plain navigation plant
};

class Session {
  public:
    Session(const ResolvedScenario &scenario, PackRuntime pack, const RulesRegistry &rules,
            SessionOptions options = {});
    ~Session();
    Session(const Session &) = delete;
    Session &operator=(const Session &) = delete;

    Step advance(); // exactly one physics tick
    const Step &lastStep() const;
    std::int64_t timeNs() const;
    std::int64_t timestepNs() const;
    sensors::Runtime &runtime();
    const PackRuntime &pack() const;
    spatial::Pose referencePose(const simulation::BodyState &body) const;

    // Commands (serialized by the transport).
    void commandThrusters(const Eigen::VectorXd &forces_native_order);
    void setKilled(bool killed);
    bool killed() const;
    CommandResult setArmed(bool armed);
    CommandResult reloadAll();
    CommandResult commandClaw(const std::string &id, bool open);
    CommandResult moveClaw(const std::string &id, double signed_duration_s);
    CommandResult fire(const std::string &id); // task events land in lastStep().task_events
    std::optional<MechanismState> mechanismState() const;

    // Placement and resets.
    simulation::Snapshot place(const simulation::BodyState &com_state, bool clear_actuators);
    const simulation::BodyState &startState() const;
    CommandResult resetTasks(); // payloads cleared, reload + disarm, props/tasks reset
    simulation::Snapshot fullReset(std::optional<std::uint64_t> seed = std::nullopt);
    std::uint64_t seed() const;

    // Scored run control (run_command protocol) and viewer-facing state.
    bool running() const;
    CommandResult runStart(const Json &options = Json::object());
    CommandResult runStop();
    CommandResult runAdjust(double points);
    void setRunMessage(const std::string &message);                // e.g. "Command rejected: <reason>" from a transport
    std::optional<Json> runSnapshot() const;                       // run_score document (old simulator format)
    Json takeFeed();                                               // task_events records since the last call
    Json taskCounters() const;                                     // task_score document
    Eigen::VectorXd thrusterForces() const;                        // realized, native order
    std::map<std::string, std::array<double, 2>> clawJaws() const; // physical jaws
    const std::vector<Payload> &payloads() const;
    std::map<std::string, std::map<std::string, PropState>> props() const; // task -> props
    Json indicators() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace nereus::session

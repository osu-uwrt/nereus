#pragma once
// SessionPort over the real robotics::session::Session (the production wiring).
#include "session_port.hpp"

#include <robotics/session/session.hpp>

#include <functional>

namespace robotics::ros_bridge {

class SessionAdapter final : public SessionPort {
  public:
    // `sensor_ids`: explicit native sensor selection (nullptr: every enabled non-camera sensor).
    SessionAdapter(const session::ResolvedScenario &scenario, const session::RulesRegistry &rules,
                   const std::vector<std::string> *sensor_ids = nullptr);
    ~SessionAdapter() override;

    StepResult advance() override;
    simulation::Snapshot observe() const override;
    const simulation::Snapshot &lastSnapshot() const override;
    std::int64_t timeNs() const override;
    std::int64_t timestepNs() const override;
    std::vector<SensorSample> drainSensors() override;
    std::map<std::string, sensors::StreamStats> sensorStats() const override;
    std::vector<std::string> sensorIds() const override;
    std::vector<std::string> deferredSensorIds() const override;
    std::shared_ptr<const spatial::FixedFrames> frames() const override;
    spatial::Pose referencePose(const simulation::BodyState &body) const override;
    void commandThrusters(const Eigen::VectorXd &forces_native_order) override;
    void setKilled(bool killed) override;
    bool killed() const override;
    session::CommandResult setArmed(bool armed) override;
    session::CommandResult reloadAll() override;
    session::CommandResult commandClaw(const std::string &id, bool open) override;
    session::CommandResult moveClaw(const std::string &id, double signed_duration_s) override;
    session::CommandResult fire(const std::string &id, session::Events &events) override;
    std::optional<session::MechanismState> mechanismState() const override;
    simulation::Snapshot place(const simulation::BodyState &com_state, bool clear_actuators) override;
    const simulation::BodyState &startState() const override;
    session::CommandResult resetTasks() override;
    simulation::Snapshot fullReset() override;
    std::uint64_t seed() const override;
    session::CommandResult runStart(const Json &options) override;
    session::CommandResult runStop() override;
    session::CommandResult runAdjust(double points) override;
    std::optional<Json> runSnapshot() const override;
    Json takeFeed() override;
    Json taskCounters() const override;
    Eigen::VectorXd thrusterForces() const override;
    std::map<std::string, std::array<double, 2>> clawJaws() const override;
    Json indicators() const override;
    std::vector<PropVisual> propVisuals() const override;
    std::vector<PayloadVisual> payloadVisuals() const override;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace robotics::ros_bridge

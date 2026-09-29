#include "session_adapter.hpp"

#include <robotics/sensors/readings.hpp>

namespace robotics::ros_bridge {
namespace {
Value vec3(const Eigen::Vector3d &v) { return Value::array({v.x(), v.y(), v.z()}); }
Value quat(const Eigen::Quaterniond &q) { return Value::array({q.w(), q.x(), q.y(), q.z()}); }
Value matrix(const Eigen::MatrixXd &m) {
    std::vector<double> flat;
    flat.reserve(static_cast<std::size_t>(m.size()));
    for (Eigen::Index r = 0; r < m.rows(); ++r)
        for (Eigen::Index c = 0; c < m.cols(); ++c)
            flat.push_back(m(r, c));
    return Value::array(std::move(flat));
}
Value vectorX(const Eigen::VectorXd &v) {
    return Value::array(std::vector<double>(v.data(), v.data() + v.size()));
}

Value imuValue(const sensors::ImuReading &r) {
    return Value::map({{"specific_force", vec3(r.specific_force)},
                       {"angular_velocity", vec3(r.angular_velocity)},
                       {"force_covariance", matrix(r.force_covariance)},
                       {"angular_covariance", matrix(r.angular_covariance)}});
}
Value attitudeValue(const sensors::AttitudeReading &r) {
    return Value::map({{"orientation_wxyz", quat(r.sensor_to_world)}, {"covariance", matrix(r.covariance)}});
}
Value toValue(const sensors::ImuReading &r) { return imuValue(r); }
Value toValue(const sensors::AttitudeReading &r) { return attitudeValue(r); }
Value toValue(const sensors::AhrsReading &r) {
    return Value::map({{"inertial", imuValue(r.inertial)}, {"attitude", attitudeValue(r.attitude)}});
}
Value toValue(const sensors::FogReading &r) {
    return Value::map({{"angular_rates", vectorX(r.angular_rates)}, {"covariance", matrix(r.covariance)}});
}
Value toValue(const sensors::DvlReading &r) {
    return Value::map({{"bottom_relative_velocity", vec3(r.bottom_relative_velocity)},
                       {"covariance", matrix(r.covariance)},
                       {"bottom_distance", Value::real(r.bottom_distance)}});
}
Value toValue(const sensors::VelocityReading &r) {
    return Value::map({{"reference_relative_velocity", vec3(r.reference_relative_velocity)},
                       {"covariance", matrix(r.covariance)}});
}
Value toValue(const sensors::AltitudeReading &r) {
    return Value::map({{"mounted_world_z", Value::real(r.mounted_world_z)},
                       {"target_world_z", Value::real(r.target_world_z)},
                       {"variance", Value::real(r.variance)}});
}
Value toValue(const sensors::PressureReading &r) {
    return Value::map({{"absolute_pressure", Value::real(r.absolute_pressure)},
                       {"pressure_variance", Value::real(r.pressure_variance)},
                       {"depth", Value::real(r.depth)},
                       {"depth_variance", Value::real(r.depth_variance)}});
}

struct Drain {
    std::string id;
    std::function<void(std::vector<SensorSample> &)> drain;
    std::function<sensors::StreamStats()> stats;
};

template <class Reading> Drain makeDrain(sensors::Runtime &runtime, const std::string &id) {
    auto stream = runtime.stream<Reading>(id);
    return Drain{id,
                 [stream, id](std::vector<SensorSample> &out) {
                     for (auto &sample : stream->drain()) {
                         SensorSample item;
                         item.sensor = id;
                         item.acquired_ns = sample.header.acquired.count();
                         if (sample.measurement.value)
                             item.reading = toValue(*sample.measurement.value);
                         out.push_back(std::move(item));
                     }
                 },
                 [stream] { return stream->stats(); }};
}

Drain drainFor(sensors::Runtime &runtime, const Json &sensor) {
    const std::string id = sensor.at("id"), kind = sensor.at("type");
    if (kind == "imu") return makeDrain<sensors::ImuReading>(runtime, id);
    if (kind == "attitude") return makeDrain<sensors::AttitudeReading>(runtime, id);
    if (kind == "ahrs") return makeDrain<sensors::AhrsReading>(runtime, id);
    if (kind == "fog") return makeDrain<sensors::FogReading>(runtime, id);
    if (kind == "dvl") return makeDrain<sensors::DvlReading>(runtime, id);
    if (kind == "reference_velocity") return makeDrain<sensors::VelocityReading>(runtime, id);
    if (kind == "reference_altitude") return makeDrain<sensors::AltitudeReading>(runtime, id);
    if (kind == "pressure") return makeDrain<sensors::PressureReading>(runtime, id);
    throw BridgeError("sensor " + repr(id) + ": no native reading for type " + repr(kind));
}
} // namespace

struct SessionAdapter::Impl {
    const session::ResolvedScenario &scenario;
    std::unique_ptr<session::Session> session;
    std::vector<Drain> drains;
    std::optional<session::Mechanisms> mechanisms; // slot geometry for still-loaded payloads
    std::map<std::string, std::pair<double, double>> projectile; // mechanism -> length, radius
    std::map<std::string, std::string> mechanism_type;

    Impl(const session::ResolvedScenario &s, const session::RulesRegistry &rules,
         const std::vector<std::string> *ids)
        : scenario(s) {
        session::PackRuntime pack = session::createRuntime(s, ids);
        session = std::make_unique<session::Session>(s, std::move(pack), rules);
        for (const auto &id : session->pack().sensor_ids)
            for (const auto &sensor : s.robot.at("sensors"))
                if (sensor.at("id") == id)
                    drains.push_back(drainFor(session->runtime(), sensor));
        const Json mechanisms_json = s.robot.value("mechanisms", Json::array());
        if (!mechanisms_json.empty())
            mechanisms.emplace(s.robot);
        for (const auto &item : mechanisms_json) {
            mechanism_type[item.at("id")] = item.at("type").get<std::string>();
            const std::string type = item.at("type");
            if (type == "launcher" || type == "dropper") {
                const auto &p = item.at("parameters").at("projectile");
                projectile[item.at("id")] = {p.at("length_m").get<double>(), p.at("radius_m").get<double>()};
            }
        }
    }
};

SessionAdapter::SessionAdapter(const session::ResolvedScenario &scenario, const session::RulesRegistry &rules,
                               const std::vector<std::string> *sensor_ids)
    : impl_(std::make_unique<Impl>(scenario, rules, sensor_ids)) {}
SessionAdapter::~SessionAdapter() = default;

StepResult SessionAdapter::advance() {
    auto step = impl_->session->advance();
    return StepResult{std::move(step.snapshot), std::move(step.task_events)};
}
simulation::Snapshot SessionAdapter::observe() const { return impl_->session->runtime().observe(); }
const simulation::Snapshot &SessionAdapter::lastSnapshot() const { return impl_->session->lastStep().snapshot; }
std::int64_t SessionAdapter::timeNs() const { return impl_->session->timeNs(); }
std::int64_t SessionAdapter::timestepNs() const { return impl_->session->timestepNs(); }
std::vector<SensorSample> SessionAdapter::drainSensors() {
    std::vector<SensorSample> out;
    for (auto &drain : impl_->drains)
        drain.drain(out);
    return out;
}
std::map<std::string, sensors::StreamStats> SessionAdapter::sensorStats() const {
    std::map<std::string, sensors::StreamStats> out;
    for (const auto &drain : impl_->drains)
        out[drain.id] = drain.stats();
    return out;
}
std::vector<std::string> SessionAdapter::sensorIds() const { return impl_->session->pack().sensor_ids; }
std::vector<std::string> SessionAdapter::deferredSensorIds() const {
    return impl_->session->pack().deferred_sensor_ids;
}
std::shared_ptr<const spatial::FixedFrames> SessionAdapter::frames() const {
    return impl_->session->pack().frames;
}
spatial::Pose SessionAdapter::referencePose(const simulation::BodyState &body) const {
    return impl_->session->referencePose(body);
}
void SessionAdapter::commandThrusters(const Eigen::VectorXd &forces) { impl_->session->commandThrusters(forces); }
void SessionAdapter::setKilled(bool killed) { impl_->session->setKilled(killed); }
bool SessionAdapter::killed() const { return impl_->session->killed(); }
session::CommandResult SessionAdapter::setArmed(bool armed) { return impl_->session->setArmed(armed); }
session::CommandResult SessionAdapter::reloadAll() { return impl_->session->reloadAll(); }
session::CommandResult SessionAdapter::commandClaw(const std::string &id, bool open) {
    return impl_->session->commandClaw(id, open);
}
session::CommandResult SessionAdapter::moveClaw(const std::string &id, double duration) {
    return impl_->session->moveClaw(id, duration);
}
session::CommandResult SessionAdapter::fire(const std::string &id, session::Events &events) {
    const auto result = impl_->session->fire(id);
    for (const auto &event : impl_->session->lastStep().task_events)
        events.push_back(event);
    return result;
}
std::optional<session::MechanismState> SessionAdapter::mechanismState() const {
    return impl_->session->mechanismState();
}
simulation::Snapshot SessionAdapter::place(const simulation::BodyState &state, bool clear_actuators) {
    return impl_->session->place(state, clear_actuators);
}
const simulation::BodyState &SessionAdapter::startState() const { return impl_->session->startState(); }
session::CommandResult SessionAdapter::resetTasks() { return impl_->session->resetTasks(); }
simulation::Snapshot SessionAdapter::fullReset() { return impl_->session->fullReset(); }
std::uint64_t SessionAdapter::seed() const { return impl_->session->seed(); }
session::CommandResult SessionAdapter::runStart(const Json &options) { return impl_->session->runStart(options); }
session::CommandResult SessionAdapter::runStop() { return impl_->session->runStop(); }
session::CommandResult SessionAdapter::runAdjust(double points) { return impl_->session->runAdjust(points); }
std::optional<Json> SessionAdapter::runSnapshot() const { return impl_->session->runSnapshot(); }
void SessionAdapter::setRunMessage(const std::string &message) { impl_->session->setRunMessage(message); }
Json SessionAdapter::takeFeed() { return impl_->session->takeFeed(); }
Json SessionAdapter::taskCounters() const { return impl_->session->taskCounters(); }
Eigen::VectorXd SessionAdapter::thrusterForces() const { return impl_->session->thrusterForces(); }
std::map<std::string, std::array<double, 2>> SessionAdapter::clawJaws() const { return impl_->session->clawJaws(); }
Json SessionAdapter::indicators() const { return impl_->session->indicators(); }

std::vector<PropVisual> SessionAdapter::propVisuals() const {
    std::vector<PropVisual> out;
    for (const auto &[task, props] : impl_->session->props())
        for (const auto &[id, state] : props)
            out.push_back({task, id, state.position, state.orientation, state.attached});
    return out;
}

std::vector<PayloadVisual> SessionAdapter::payloadVisuals() const {
    std::vector<PayloadVisual> out;
    for (const auto &payload : impl_->session->payloads()) {
        const auto found = impl_->projectile.find(payload.mechanism_id);
        if (found == impl_->projectile.end())
            continue;
        out.push_back({payload.mechanism_id, payload.mechanism_type, payload.id, false,
                       payload.state.position, payload.state.orientation, found->second.first,
                       found->second.second});
    }
    const auto state = impl_->session->mechanismState();
    if (state && impl_->mechanisms) {
        const auto &body = impl_->session->lastStep().snapshot.body;
        const spatial::Pose root{body.position, body.orientation};
        for (const auto &[key, release] : state->releases) {
            const auto projectile = impl_->projectile.find(key);
            if (projectile == impl_->projectile.end())
                continue;
            const int count = impl_->mechanisms->slotCount(key);
            for (int index = count - release.available; index < count; ++index) {
                const auto pose = spatial::compose(root, impl_->mechanisms->slotMount(key, index));
                out.push_back({key, impl_->mechanism_type.at(key), index, true, pose.translation,
                               pose.rotation, projectile->second.first, projectile->second.second});
            }
        }
    }
    return out;
}
} // namespace robotics::ros_bridge

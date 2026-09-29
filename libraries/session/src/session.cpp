// Port of python/src/robotics_platform/session.py: the single owner that advances the plant,
// sensors, mechanisms, payloads, prop worlds and task observers.
#include <robotics/session/session.hpp>

#include "json_util.hpp"
#include "run_score.hpp"

#include <cmath>

namespace robotics::session {
namespace {
using namespace detail;
// Several contact worlds: resolved in order within one plant step.
class ResolverChain final : public simulation::ContactResolver {
  public:
    explicit ResolverChain(std::vector<std::shared_ptr<simulation::ContactResolver>> resolvers)
        : resolvers_(std::move(resolvers)) {}
    State resolve(State state, const Matrix6 &inverse_mass) override {
        for (auto &resolver : resolvers_)
            state = resolver->resolve(state, inverse_mass);
        return state;
    }

  private:
    std::vector<std::shared_ptr<simulation::ContactResolver>> resolvers_;
};

simulation::PayloadParameters payloadParameters(const Json &projectile) {
    simulation::PayloadParameters p;
    const std::string model = projectile.at("model").get<std::string>();
    if (model == "finned_rigid_body")
        p.model = simulation::PayloadModel::Finned;
    else if (model == "fixed_axis_body")
        p.model = simulation::PayloadModel::FixedAxis;
    else
        throw std::invalid_argument("unsupported projectile model " + repr(model));
    p.mass = projectile.at("mass_kg").get<double>();
    p.displaced_volume = projectile.at("displaced_volume_m3").get<double>();
    p.neutral_buoyancy = projectile.at("neutral_buoyancy").get<bool>();
    p.added_mass = projectile.at("added_mass_kg").get<double>();
    p.length = projectile.at("length_m").get<double>();
    p.radius = projectile.at("radius_m").get<double>();
    p.drag_axial = projectile.at("drag_axial").get<double>();
    p.drag_lateral = projectile.at("drag_lateral").get<double>();
    const auto optional = [&](const char *key) {
        return projectile.contains(key) ? projectile.at(key).get<double>() : 0.0;
    };
    p.center_of_mass = optional("center_of_mass_m");
    p.center_of_buoyancy = optional("center_of_buoyancy_m");
    p.center_of_drag = optional("center_of_drag_m");
    p.angular_damping = optional("angular_damping_n_m_s");
    return p;
}

Eigen::Vector3d axisOf(const Eigen::Quaterniond &orientation) {
    return rotate(orientation, Eigen::Vector3d::UnitX());
}

struct PayloadExtra {
    std::int64_t max_age_ns{0};
    int slot{0};
};
} // namespace

struct Session::Impl {
    ResolvedScenario scenario;
    PackRuntime pack;
    const Json *task_pack;
    Json robot_safety;
    std::string reference_frame;
    spatial::Pose root_to_reference;
    std::int64_t timestep_ns;
    std::unique_ptr<Mechanisms> mechanisms;
    std::map<std::string, std::string> mechanism_types;
    std::map<std::string, Json> projectiles;
    std::map<std::string, simulation::PayloadDynamics> dynamics;
    std::unique_ptr<TaskRuntime> tasks;
    std::vector<std::unique_ptr<PropWorld>> prop_worlds;
    bool has_environment{false};
    double density{0}, level{0}, frequency{0};
    Eigen::Vector3d current, amplitude;

    simulation::BodyState start_state;
    std::uint64_t seed{0};
    bool killed{false};
    std::vector<Payload> payloads;
    std::vector<PayloadExtra> extras;
    int next_payload{0};
    Step last_step;
    double run_adjustment{0};
    std::string run_message;
    Json counters = Json::object();
    Json feed_queue = Json::array();

    Impl(const ResolvedScenario &s, PackRuntime p) : scenario(s), pack(std::move(p)) {}

    std::int64_t timeNs() const {
        return last_step.snapshot.elapsed.count();
    }
    spatial::Pose referencePose(const simulation::BodyState &body) const {
        return composeChecked({body.position, body.orientation}, root_to_reference);
    }
    simulation::PayloadEnvironment waterAt(double time_s) const {
        simulation::PayloadEnvironment water;
        water.water_density = density;
        water.water_level = level;
        water.water_velocity = current + amplitude * std::sin(2 * kPi * frequency * time_s);
        return water;
    }
    void resetOwnedState() {
        killed = scenario.robot.at("safety").at("initially_killed").get<bool>();
        if (mechanisms)
            mechanisms->reset(killed);
        payloads.clear();
        extras.clear();
        next_payload = 0;
        last_step = Step{pack.runtime->observe(), {}};
        run_adjustment = 0;
        run_message.clear();
        counters = Json::object();
        if (task_pack->contains("outcome_counters"))
            for (const auto &name : task_pack->at("outcome_counters"))
                counters[name.get<std::string>()] = 0;
        feed_queue = Json::array();
    }
    std::string message(const char *key, const char *fallback) const {
        if (task_pack->contains("run_messages") && task_pack->at("run_messages").contains(key)) {
            const Json &value = task_pack->at("run_messages").at(key);
            return value.is_string() ? value.get<std::string>() : value.dump();
        }
        return fallback;
    }
    Json resetEvent() const {
        return {{"kind", "tasks"}, {"result", "reset"}, {"target", ""}, {"time", static_cast<double>(timeNs()) / 1e9}};
    }
    void ingest(const Events &events) {
        if (events.empty() || !tasks)
            return;
        Json context = {{"payloads", Json::object()}};
        for (std::size_t i = 0; i < payloads.size(); ++i)
            context["payloads"][std::to_string(payloads[i].id)] = {{"mechanism_id", payloads[i].mechanism_id},
                                                                     {"mechanism_type", payloads[i].mechanism_type},
                                                                     {"slot", extras[i].slot}};
        for (const auto &item : tasks->feed(events, context)) {
            const std::string result = item.at("result").get<std::string>();
            if (counters.contains(result))
                counters[result] = counters[result].get<int>() + 1;
            feed_queue.push_back(item);
        }
    }
    void record(const Events &events) {
        if (events.empty())
            return;
        last_step.task_events.insert(last_step.task_events.end(), events.begin(), events.end());
        ingest(events);
    }
    bool running() const {
        return tasks && tasks->snapshot().at("run").at("running").get<bool>();
    }

    Events propagate(std::size_t index, std::int64_t now, const simulation::PayloadEnvironment &water) {
        auto &payload = payloads[index];
        const simulation::PayloadState old = payload.state;
        const simulation::PayloadState fresh = dynamics.at(payload.mechanism_id).advance(old, water, static_cast<double>(timestep_ns) / 1e9);
        payload.state = fresh;
        if (now - payload.released_ns > extras[index].max_age_ns) {
            payload.active = false;
            payload.outcome = "timeout";
        }
        if (!tasks)
            return {};
        const auto step = tasks->stepProjectile(now, payload.id, old.position, fresh.position,
                                                axisOf(fresh.orientation), fresh.velocity);
        if (step.position_world)
            payload.state.position = *step.position_world;
        if (step.velocity_world)
            payload.state.velocity = *step.velocity_world;
        if (step.stop) {
            payload.active = false;
            payload.outcome = step.events.empty() ? "stopped" : step.events.back().at("id").get<std::string>();
            payload.state.velocity.setZero();
            payload.state.angular_velocity.setZero();
        }
        return step.events;
    }

    Events stepProps(const simulation::BodyState &body, std::int64_t now) {
        const spatial::Pose root{body.position, body.orientation};
        const Eigen::Vector3d velocity = rotate(root.rotation, body.linear_velocity);
        const Eigen::Vector3d omega = rotate(root.rotation, body.angular_velocity);
        const MechanismState state = mechanisms->snapshot(killed);
        const auto &pool = pack.parameters.pool;
        const Eigen::Vector3d flow =
            pool.current_velocity +
            pool.current_oscillation_amplitude *
                std::sin(2 * kPi * pool.current_oscillation_frequency * static_cast<double>(now) / 1e9);
        Water water;
        water.velocity_world = flow;
        water.density = pool.water_density;
        Events events;
        for (auto &world : prop_worlds) {
            const auto &claw = state.claws.at(world->mechanismId());
            const auto produced = world->step(static_cast<double>(timestep_ns) / 1e9, now, root, velocity, omega,
                                              claw.joint_positions_m, water, state.armed && !killed);
            if (!produced.empty()) {
                const auto recorded = tasks->record(now, produced);
                events.insert(events.end(), recorded.begin(), recorded.end());
            }
        }
        return events;
    }
    // Body-axis COM velocities -> velocity at the reference origin in reference axes.
    std::pair<Eigen::Vector3d, Eigen::Vector3d> referenceVelocities(const simulation::BodyState &body) const {
        const Eigen::Vector3d velocity = body.linear_velocity + body.angular_velocity.cross(root_to_reference.translation);
        const auto rotation = inverseChecked(root_to_reference).rotation;
        return {rotate(rotation, velocity), rotate(rotation, body.angular_velocity)};
    }
    std::string mechanismTypeError(const std::string &id, const char *kind) const {
        const auto found = mechanism_types.find(id);
        return found != mechanism_types.end() && found->second == kind
                   ? std::string()
                   : std::string("unknown ") + kind + " mechanism " + repr(id);
    }
};

Session::Session(const ResolvedScenario &scenario, PackRuntime pack, const RulesRegistry &rules,
                 SessionOptions options)
    : impl_(std::make_unique<Impl>(scenario, std::move(pack))) {
    auto &s = *impl_;
    s.task_pack = &s.scenario.tasks;
    s.timestep_ns = s.pack.parameters.timestep.count();
    s.reference_frame = s.scenario.robot.at("reference_frame").get<std::string>();
    s.root_to_reference = s.pack.frames->fromRoot(s.reference_frame);
    const Json &mechanisms = s.scenario.robot.contains("mechanisms") ? s.scenario.robot.at("mechanisms") : Json::array();
    if (!mechanisms.empty())
        s.mechanisms = std::make_unique<Mechanisms>(s.scenario.robot);
    for (const auto &item : mechanisms) {
        const std::string id = item.at("id").get<std::string>(), kind = item.at("type").get<std::string>();
        s.mechanism_types[id] = kind;
        if (kind == "launcher" || kind == "dropper") {
            s.projectiles[id] = item.at("parameters").at("projectile");
            s.dynamics.emplace(id, simulation::PayloadDynamics(payloadParameters(s.projectiles[id])));
        }
    }
    if (options.tasks && !s.scenario.task_definitions.empty()) {
        s.tasks = std::make_unique<TaskRuntime>(s.scenario, rules, options.task_ids);
        for (const auto &task : s.scenario.task_definitions) {
            const std::string id = task.at("id").get<std::string>();
            bool selected = !options.task_ids;
            if (options.task_ids)
                for (const auto &wanted : *options.task_ids)
                    selected = selected || wanted == id;
            bool contact = false;
            for (const auto &prop : task.at("props"))
                contact = contact || prop.at("type") == "contact_world";
            if (selected && contact)
                s.prop_worlds.push_back(std::make_unique<PropWorld>(s.scenario, id));
        }
        // Robot-side contacts of every contact world (claw pads / held props against task scenery),
        // resolved inside each plant step.
        std::vector<std::shared_ptr<simulation::ContactResolver>> resolvers;
        for (auto &world : s.prop_worlds)
            resolvers.push_back(world->vehicleContacts(s.pack.parameters.contacts.friction));
        if (resolvers.size() == 1)
            s.pack.runtime->setContactResolver(resolvers[0]);
        else if (!resolvers.empty())
            s.pack.runtime->setContactResolver(std::make_shared<ResolverChain>(std::move(resolvers)));
    }
    if (!s.dynamics.empty()) {
        const auto &pool = s.pack.parameters.pool;
        s.has_environment = true;
        s.density = pool.water_density;
        s.level = pool.water_level;
        s.current = pool.current_velocity;
        s.amplitude = pool.current_oscillation_amplitude;
        s.frequency = pool.current_oscillation_frequency;
    }
    s.start_state = s.pack.initial;
    s.seed = s.scenario.scenario.at("seed").get<std::uint64_t>();
    s.resetOwnedState();
}
Session::~Session() = default;

Step Session::advance() {
    auto &s = *impl_;
    auto snapshot = s.pack.runtime->advance(1);
    const std::int64_t now = snapshot.elapsed.count();
    Events events;
    if (s.mechanisms)
        s.mechanisms->advance(s.timestep_ns, s.killed);
    if (s.has_environment && !s.payloads.empty()) {
        const auto water = s.waterAt(static_cast<double>(now) / 1e9);
        for (std::size_t i = 0; i < s.payloads.size(); ++i)
            if (s.payloads[i].active) {
                auto produced = s.propagate(i, now, water);
                events.insert(events.end(), std::make_move_iterator(produced.begin()), std::make_move_iterator(produced.end()));
            }
    }
    if (!s.prop_worlds.empty()) {
        auto produced = s.stepProps(snapshot.body, now);
        events.insert(events.end(), std::make_move_iterator(produced.begin()), std::make_move_iterator(produced.end()));
    }
    if (s.tasks) {
        auto produced = s.tasks->observe(now, s.referencePose(snapshot.body));
        events.insert(events.end(), std::make_move_iterator(produced.begin()), std::make_move_iterator(produced.end()));
    }
    s.last_step = Step{std::move(snapshot), std::move(events)};
    s.ingest(s.last_step.task_events);
    return s.last_step;
}

const Step &Session::lastStep() const {
    return impl_->last_step;
}
std::int64_t Session::timeNs() const {
    return impl_->timeNs();
}
std::int64_t Session::timestepNs() const {
    return impl_->timestep_ns;
}
sensors::Runtime &Session::runtime() {
    return *impl_->pack.runtime;
}
const PackRuntime &Session::pack() const {
    return impl_->pack;
}
spatial::Pose Session::referencePose(const simulation::BodyState &body) const {
    return impl_->referencePose(body);
}

void Session::commandThrusters(const Eigen::VectorXd &forces) {
    impl_->pack.runtime->command(forces);
}
void Session::setKilled(bool killed) {
    auto &s = *impl_;
    s.killed = killed;
    if (killed && s.scenario.robot.at("safety").at("kill_stops_thrusters").get<bool>())
        s.pack.runtime->stopThrusters();
    if (s.mechanisms)
        s.mechanisms->advance(0, killed);
}
bool Session::killed() const {
    return impl_->killed;
}
CommandResult Session::setArmed(bool armed) {
    auto &s = *impl_;
    return s.mechanisms ? s.mechanisms->setArmed(armed, s.killed) : CommandResult{false, "robot has no mechanisms"};
}
CommandResult Session::reloadAll() {
    auto &s = *impl_;
    return s.mechanisms ? s.mechanisms->reloadAll(s.killed) : CommandResult{false, "robot has no mechanisms"};
}
CommandResult Session::commandClaw(const std::string &id, bool open) {
    auto &s = *impl_;
    const auto error = s.mechanismTypeError(id, "claw");
    if (!error.empty() || !s.mechanisms)
        return {false, error.empty() ? "robot has no mechanisms" : error};
    return s.mechanisms->commandClaw(id, open, s.killed);
}
CommandResult Session::moveClaw(const std::string &id, double signed_duration_s) {
    auto &s = *impl_;
    const auto error = s.mechanismTypeError(id, "claw");
    if (!error.empty() || !s.mechanisms)
        return {false, error.empty() ? "robot has no mechanisms" : error};
    return s.mechanisms->moveClaw(id, signed_duration_s, s.killed);
}

CommandResult Session::fire(const std::string &id) {
    auto &s = *impl_;
    if (!s.dynamics.count(id) || !s.mechanisms)
        return {false, "unknown release mechanism " + repr(id)};
    const auto &body = s.last_step.snapshot.body;
    const auto [velocity, omega] = s.referenceVelocities(body);
    const std::int64_t now = s.timeNs();
    PayloadRelease release;
    const auto result = s.mechanisms->fire(id, s.referencePose(body), velocity, omega, s.reference_frame,
                                           s.density, s.killed, release);
    if (!result.accepted)
        return result;
    Payload payload;
    payload.id = s.next_payload++;
    payload.mechanism_id = id;
    payload.mechanism_type = s.mechanism_types.at(id);
    payload.state.position = release.position_world;
    payload.state.orientation = release.orientation_world;
    payload.state.velocity = release.velocity_com_world;
    payload.state.angular_velocity = release.angular_velocity_world;
    payload.released_ns = now;
    const Json &projectile = s.projectiles.at(id);
    const double max_age_s = projectile.contains("max_age_s") ? projectile.at("max_age_s").get<double>() : 30.0;
    s.payloads.push_back(payload);
    s.extras.push_back({static_cast<std::int64_t>(std::nearbyint(max_age_s * 1e9)), release.slot_index});
    if (s.tasks) {
        const double length = projectile.at("length_m").get<double>();
        const Eigen::Vector3d tip = payload.state.position + axisOf(payload.state.orientation) * (length / 2);
        s.record(s.tasks->releaseProjectile(now, payload.id, payload.mechanism_type, tip,
                                            projectile.at("radius_m").get<double>(), length));
    }
    return result;
}

std::optional<MechanismState> Session::mechanismState() const {
    if (!impl_->mechanisms)
        return std::nullopt;
    return impl_->mechanisms->snapshot(impl_->killed);
}

simulation::Snapshot Session::place(const simulation::BodyState &com_state, bool clear_actuators) {
    auto &s = *impl_;
    auto snapshot = s.pack.runtime->place(com_state, clear_actuators);
    s.last_step = Step{snapshot, {}};
    return snapshot;
}
const simulation::BodyState &Session::startState() const {
    return impl_->start_state;
}

CommandResult Session::resetTasks() {
    auto &s = *impl_;
    s.payloads.clear();
    s.extras.clear();
    if (s.mechanisms)
        s.mechanisms->reloadAll(s.killed);
    s.run_adjustment = 0;
    for (auto &[key, value] : s.counters.items()) {
        (void)key;
        value = 0;
    }
    if (s.tasks)
        s.tasks->reset(s.timeNs());
    for (auto &world : s.prop_worlds)
        world->reset();
    s.run_message = s.message("reset", "Tasks and run reset");
    s.feed_queue.push_back(s.resetEvent());
    return {true, "All tasks reset; ammunition reloaded and actuators disarmed"};
}

simulation::Snapshot Session::fullReset(std::optional<std::uint64_t> seed) {
    auto &s = *impl_;
    if (seed)
        s.seed = *seed;
    auto snapshot = s.pack.runtime->reset(s.pack.initial, s.seed);
    s.start_state = s.pack.initial;
    s.resetOwnedState();
    if (s.tasks)
        s.tasks->reset(s.timeNs());
    for (auto &world : s.prop_worlds)
        world->reset();
    s.last_step = Step{snapshot, {}};
    s.feed_queue.push_back(s.resetEvent());
    return snapshot;
}
std::uint64_t Session::seed() const {
    return impl_->seed;
}

bool Session::running() const {
    return impl_->running();
}

CommandResult Session::runStart(const Json &options) {
    auto &s = *impl_;
    if (!s.tasks)
        return {false, "Course scoring is not configured"};
    if (s.running())
        return {false, "Stop the current run first"};
    std::map<std::string, const Json *> declared;
    for (const auto &item : s.task_pack->at("run_options"))
        declared[item.at("key").get<std::string>()] = &item;
    Json chosen = Json::object();
    if (options.is_object())
        for (const auto &[key, value] : options.items()) {
            const auto found = declared.find(key);
            if (found == declared.end())
                return {false, "Unknown run option " + repr(key)};
            const Json &option = *found->second;
            if (option.at("type") == "bool" && !value.is_boolean())
                return {false, "Run option " + repr(key) + " must be true or false"};
            if (option.at("type") == "choice") {
                bool known = false;
                std::string joined;
                for (const auto &choice : option.at("choices")) {
                    known = known || choice == value;
                    joined += (joined.empty() ? "" : " or ") + choice.get<std::string>();
                }
                if (!known)
                    return {false, "Select a known " + key + ": " + joined};
            }
            chosen[key] = value;
        }
    resetTasks();
    s.tasks->start(s.timeNs(), chosen);
    s.run_message = s.message("start", "Run started");
    return {true, s.run_message};
}

CommandResult Session::runStop() {
    auto &s = *impl_;
    if (!s.tasks)
        return {false, "Course scoring is not configured"};
    s.record(s.tasks->stop(s.timeNs()));
    s.run_message = s.message("stop", "Run stopped");
    return {true, s.run_message};
}

CommandResult Session::runAdjust(double points) {
    auto &s = *impl_;
    if (!s.tasks)
        return {false, "Course scoring is not configured"};
    if (!std::isfinite(points))
        return {false, "Adjustment must be finite"};
    s.run_adjustment = points;
    s.run_message = s.message("adjustment", "Manual adjustment updated");
    return {true, s.run_message};
}

std::optional<Json> Session::runSnapshot() const {
    const auto &s = *impl_;
    if (!s.tasks)
        return std::nullopt;
    return buildRunSnapshot(*s.task_pack, s.tasks->snapshot(), s.tasks->describe(), s.timeNs(),
                            s.run_adjustment, s.run_message);
}

Json Session::takeFeed() {
    Json items = std::move(impl_->feed_queue);
    impl_->feed_queue = Json::array();
    return items;
}
Json Session::taskCounters() const {
    return impl_->counters;
}
Eigen::VectorXd Session::thrusterForces() const {
    return impl_->last_step.snapshot.thruster_forces;
}

std::map<std::string, std::array<double, 2>> Session::clawJaws() const {
    std::map<std::string, std::array<double, 2>> jaws;
    if (const auto state = mechanismState())
        for (const auto &[key, claw] : state->claws)
            jaws[key] = claw.joint_positions_m;
    for (const auto &world : impl_->prop_worlds)
        jaws[world->mechanismId()] = {world->jawPosition(), world->jawPosition()};
    return jaws;
}
const std::vector<Payload> &Session::payloads() const {
    return impl_->payloads;
}
std::map<std::string, std::map<std::string, PropState>> Session::props() const {
    std::map<std::string, std::map<std::string, PropState>> result;
    for (const auto &world : impl_->prop_worlds)
        result[world->task()] = world->props();
    return result;
}
Json Session::indicators() const {
    return impl_->tasks ? impl_->tasks->indicators() : Json::array();
}
} // namespace robotics::session

// TaskRuntime (see tasks.hpp for the contract).
#include "tasks/trackers.hpp"

#include <robotics/session/tasks.hpp>

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace robotics::session {
using namespace tasks;

namespace {
constexpr double kMaxPayloadAgeS = 30.0;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

const std::set<std::string> kEventTypes = {
    "pass_through",    "hit",         "payload_landing", "drop_into",       "activate",
    "rotation_judged", "attach",      "detach",          "surface_reached", "surface_lost",
    "facing_reached",  "facing_lost", "breach"};

[[noreturn]] void invalid(const std::string &message) {
    throw std::invalid_argument(message);
}

void checkTimeValue(std::int64_t time_ns) {
    if (time_ns < 0)
        invalid("time_ns must be a nonnegative integer");
}

Event makeEvent(const std::string &id, const std::string &type, const std::string &task, const std::string &region,
                std::int64_t time_ns, Json data) {
    return {{"id", id},         {"type", type},       {"task", task},
            {"region", region}, {"time_ns", time_ns}, {"data", std::move(data)}};
}

Pose placementPose(const Json &config) {
    const double yaw = config.at("yaw_deg").get<double>() * kDegToRad / 2;
    Pose pose;
    pose.translation = vec3(config.at("position_m"), "position_m");
    pose.rotation = Eigen::Quaterniond(std::cos(yaw), 0, 0, std::sin(yaw));
    return ownedPose(pose);
}

spatial::FixedFrames framesOf(const Json &frames) {
    std::vector<spatial::FixedFrame> edges;
    for (const auto &entry : frames.at("transforms"))
        edges.push_back({entry.at("parent").get<std::string>(), entry.at("child").get<std::string>(),
                         poseFrom(entry.at("position_m"), entry.at("orientation_wxyz"))});
    return spatial::FixedFrames(frames.at("root").get<std::string>(), std::move(edges));
}

std::vector<Vec3> envelopeOf(const Json &robot) {
    const auto frames = framesOf(robot.at("frames"));
    const Pose reference_from_root = spatial::inverse(frames.fromRoot(robot.at("reference_frame").get<std::string>()));
    std::map<std::string, const Json *> boxes;
    for (const auto &box : robot.at("collision_boxes"))
        boxes[box.at("id").get<std::string>()] = &box;
    std::vector<Vec3> vertices;
    for (const auto &identifier : robot.at("scoring_envelope").at("collision_boxes")) {
        const Json &box = *boxes.at(identifier.get<std::string>());
        const Pose pose =
            spatial::compose(reference_from_root, poseFrom(box.at("center_m"), box.at("orientation_wxyz")));
        const Vec3 half = vec3(box.at("size_m"), "size_m") / 2;
        for (int sx : {-1, 1})
            for (int sy : {-1, 1})
                for (int sz : {-1, 1})
                    vertices.push_back(spatial::apply(pose, Vec3(half[0] * sx, half[1] * sy, half[2] * sz)));
    }
    for (const auto &point : robot.at("scoring_envelope").at("points")) {
        const Pose pose = spatial::compose(reference_from_root, frames.fromRoot(point.at("frame").get<std::string>()));
        vertices.push_back(spatial::apply(pose, vec3(point.at("position_m"), "position_m")));
    }
    if (vertices.empty())
        invalid("task traversal requires a nonempty robot scoring envelope");
    return vertices;
}

Vec3 probePoint(const Json &robot, const std::string &mechanism_type) {
    std::vector<const Json *> found;
    for (const auto &item : robot.at("mechanisms"))
        if (item.at("type") == mechanism_type)
            found.push_back(&item);
    if (found.size() != 1)
        invalid("task probe needs exactly one '" + mechanism_type + "' mechanism; robot has " +
                std::to_string(found.size()));
    const auto frames = framesOf(robot.at("frames"));
    const Pose pose =
        spatial::compose(spatial::inverse(frames.fromRoot(robot.at("reference_frame").get<std::string>())),
                         frames.fromRoot(found[0]->at("frame").get<std::string>()));
    return spatial::apply(pose, vec3(found[0]->at("parameters").at("tip_position_m"), "tip_position_m"));
}

bool finiteJson(const Json &value) {
    if (value.is_number_float())
        return std::isfinite(value.get<double>());
    if (value.is_array() || value.is_object())
        return std::all_of(value.begin(), value.end(), [](const Json &item) { return finiteJson(item); });
    return true;
}

struct Projectile {
    std::string mechanism;
    double radius{0}, length{0};
    std::int64_t released_ns{0};
    std::set<std::string> tasks;
    std::set<std::pair<std::string, std::string>> entered;
    bool active{true}, scored{false};
};

template <class T> struct Keyed {
    std::string task, region;
    T value;
};
} // namespace

struct TaskRuntime::Impl {
    struct ScoringRules {
        std::string name;
        RulesFactory factory;
        Json parameters;
    };

    std::vector<std::string> order;
    std::map<std::string, Json> tasks;
    Json options;
    bool auto_start{false};
    Json seed;
    double surface_z{0}, floor_z{0}, pool_length{0}, pool_width{0};
    Pose pool_from_world;
    std::vector<Keyed<PortalTracker>> portals;
    std::vector<Keyed<PerforatedPanel>> panels;
    std::vector<Keyed<OpenCrate>> crates;
    std::vector<Keyed<ProximityTarget>> targets;
    std::vector<Keyed<SurfaceTracker>> surfaces;
    std::vector<Keyed<TurnTracker>> turns;
    std::vector<std::pair<std::string, Json>> score_rules;
    std::vector<ScoringRules> scoring;
    std::vector<std::unique_ptr<Rules>> instances;

    Json state; // {run, scores, history, tasks, environment, latched}
    std::map<std::pair<std::string, std::string>, int> award_counts;
    std::map<int, Projectile> projectiles;
    std::int64_t last_time{0};
    bool failed{false};

    const Json &bindings(const std::string &task) const {
        return tasks.at(task).at("events");
    }

    void refreshLatched() {
        Json latched = Json::object();
        for (const auto &t : targets)
            latched[t.task + "/" + t.region] = t.value.latched;
        state["latched"] = std::move(latched);
    }

    void checkTime(std::int64_t time_ns) const {
        checkTimeValue(time_ns);
        if (failed)
            throw std::runtime_error("task observer failed; reset before continuing");
        if (time_ns < last_time)
            invalid("task time cannot go backwards without reset");
    }

    void reset(std::int64_t time_ns) {
        checkTimeValue(time_ns);
        instances.clear();
        for (const auto &entry : scoring) {
            auto rules = entry.factory();
            if (!rules)
                invalid("rules factory '" + entry.name + "' returned nothing");
            instances.push_back(std::move(rules));
        }
        for (auto &t : portals)
            t.value.reset();
        for (auto &t : targets)
            t.value.reset();
        for (auto &t : surfaces)
            t.value.reset();
        for (auto &t : turns)
            t.value.reset();
        state["scores"] = Json::object();
        state["history"] = Json::array();
        award_counts.clear();
        projectiles.clear();
        last_time = time_ns;
        failed = false;
        state["run"] = {{"running", auto_start}, {"ended", false},     {"started_ns", time_ns},
                        {"stopped_ns", nullptr}, {"options", options}, {"seed", seed}};
        refreshLatched();
    }

    // Runs `body`; any failure poisons the observer until reset (as in Python).
    template <class F> auto guarded(F &&body) -> decltype(body()) {
        try {
            return body();
        } catch (...) {
            failed = true;
            throw;
        }
    }

    void validateEvent(const Json &event, const Events &inputs) const {
        static const std::set<std::string> keys = {"id", "type", "task", "region", "time_ns", "data"};
        bool ok = event.is_object() && event.size() == keys.size();
        if (ok)
            for (const auto &key : keys)
                ok = ok && event.contains(key);
        for (const char *key : {"id", "type", "task", "region"})
            ok = ok && event.at(key).is_string();
        ok = ok && !event.at("id").get<std::string>().empty() && !event.at("type").get<std::string>().empty() &&
             tasks.count(event.at("task").get<std::string>()) > 0 && event.at("data").is_object();
        if (!ok)
            invalid("rules emitted a malformed task event");
        const Json &time = event.at("time_ns");
        if (!time.is_number_integer() || time.get<std::int64_t>() < 0)
            invalid("time_ns must be a nonnegative integer");
        std::int64_t lo = inputs.front().at("time_ns").get<std::int64_t>(), hi = lo;
        for (const auto &item : inputs) {
            lo = std::min(lo, item.at("time_ns").get<std::int64_t>());
            hi = std::max(hi, item.at("time_ns").get<std::int64_t>());
        }
        if (!(lo <= time.get<std::int64_t>() && time.get<std::int64_t>() <= hi))
            invalid("rules event time must belong to the input interval");
    }

    Events evaluate(const Events &events) {
        if (events.empty())
            return {};
        for (auto &tracker : turns) {
            bool restart = false;
            for (const auto &event : events)
                for (const auto &entry : tracker.value.restart_events)
                    restart = restart || (event.at("task") == entry.first && event.at("id") == entry.second);
            if (restart)
                tracker.value.restart();
        }
        refreshLatched();
        const Json committed_scores = state.at("scores");
        try {
            Json scores = committed_scores;
            auto counts = award_counts;
            const Json &run = state.at("run");
            if (run.at("running").get<bool>() && !run.at("ended").get<bool>()) {
                for (const auto &entry : score_rules) {
                    const std::string &task = entry.first;
                    const Json &rule = entry.second;
                    const Json &parameters = rule.at("parameters");
                    const std::pair<std::string, std::string> key{task, rule.at("id").get<std::string>()};
                    for (const auto &event : events) {
                        const int limit = parameters.contains("max_awards") ? parameters.at("max_awards").get<int>()
                                                                            : std::numeric_limits<int>::max();
                        if (event.at("task") == task && event.at("id") == parameters.at("event") &&
                            counts[key] < limit) {
                            const std::string row = task + "/" + key.second;
                            scores[row] = (scores.contains(row) ? scores[row].get<long long>() : 0LL) +
                                          parameters.at("points").get<long long>();
                            ++counts[key];
                        }
                    }
                }
            }
            Events emitted;
            for (std::size_t i = 0; i < scoring.size(); ++i) {
                state["scores"] = scores;
                Json result = instances[i]->evaluate(state, events, scoring[i].parameters);
                if (!finiteJson(result))
                    invalid("rules result contains a non-finite number");
                if (!result.is_object() || result.size() != 2 || !result.contains("scores") ||
                    !result.contains("events"))
                    invalid("rules must return scores and events");
                if (!result.at("scores").is_array() || !result.at("events").is_array())
                    invalid("rules scores and events must be lists");
                for (const auto &change : result.at("scores")) {
                    if (!change.is_object() || change.size() != 2 || !change.contains("row") ||
                        !change.contains("points") || !change.at("row").is_string() ||
                        change.at("row").get<std::string>().empty() || !change.at("points").is_number_integer())
                        invalid("rules score changes require a row and integer points");
                    scores[change.at("row").get<std::string>()] = change.at("points");
                }
                for (const auto &event : result.at("events")) {
                    validateEvent(event, events);
                    emitted.push_back(event);
                }
            }
            state["scores"] = scores;
            award_counts = std::move(counts);
            Events all = events;
            all.insert(all.end(), emitted.begin(), emitted.end());
            for (const auto &event : all)
                state["history"].push_back(event);
            return all;
        } catch (...) {
            state["scores"] = committed_scores;
            throw;
        }
    }

    Events portalEvents(const std::string &task, const std::string &region, const PortalEvent &event) const {
        Events out;
        const Json data = event.data();
        if (event.kind == "attempt_finished") {
            out.push_back(makeEvent(region + ":attempt_finished", event.kind, task, region, event.time_ns, data));
            return out;
        }
        for (const auto &binding : bindings(task)) {
            const Json &p = binding.at("parameters");
            if (binding.at("type") == event.kind && p.at("region") == region && p.at("from_side") == event.from_side &&
                p.at("to_side") == event.to_side)
                out.push_back(makeEvent(binding.at("id"), event.kind, task, region, event.time_ns, data));
        }
        return out;
    }

    Events factEvents(const std::string &task, const std::string &region, std::int64_t time_ns,
                      const std::vector<Fact> &facts) const {
        Events out;
        for (const auto &fact : facts)
            for (const auto &binding : bindings(task))
                if (binding.at("type") == fact.kind && binding.at("parameters").at("region") == region)
                    out.push_back(makeEvent(binding.at("id"), fact.kind, task, region, time_ns, fact.data));
        return out;
    }
};

TaskRuntime::TaskRuntime(const ResolvedScenario &scenario, const RulesRegistry &rules,
                         const std::vector<std::string> *task_ids)
    : impl_(std::make_unique<Impl>()) {
    Impl &m = *impl_;
    std::map<std::string, const Json *> definitions;
    std::vector<std::string> defined;
    for (const auto &task : scenario.task_definitions) {
        definitions[task.at("id").get<std::string>()] = &task;
        defined.push_back(task.at("id").get<std::string>());
    }
    m.order = task_ids ? *task_ids : defined;
    {
        const std::set<std::string> unique(m.order.begin(), m.order.end());
        if (unique.size() != m.order.size() || m.order.empty())
            invalid("task_ids must select unique existing tasks");
        for (const auto &id : m.order)
            if (!definitions.count(id))
                invalid("task_ids must select unique existing tasks");
    }
    for (const auto &id : m.order)
        m.tasks[id] = *definitions.at(id); // owned copy
    m.options = scenario.run_options.is_null() ? Json::object() : scenario.run_options;
    m.auto_start = scenario.scenario.at("run").at("auto_start").get<bool>();
    m.seed = scenario.scenario.at("seed");
    const double surface = scenario.pool.at("parameters").at("water_level_m").get<double>() +
                           scenario.scenario.at("pool_placement").at("position_m").at(2).get<double>();
    m.surface_z = surface;
    m.floor_z = surface - scenario.pool.at("parameters").at("depth_m").get<double>();
    m.pool_length = scenario.pool.at("parameters").at("length_m").get<double>();
    m.pool_width = scenario.pool.at("parameters").at("width_m").get<double>();
    m.pool_from_world = spatial::inverse(placementPose(scenario.scenario.at("pool_placement")));
    m.state = Json::object();
    m.state["environment"] = {{"surface_z_m", m.surface_z}, {"floor_z_m", m.floor_z}};
    m.state["tasks"] = Json::object();
    const auto envelope = envelopeOf(scenario.robot);
    std::map<std::string, Pose> placements;
    for (const auto &item : scenario.scenario.at("task_placements"))
        placements[item.at("task").get<std::string>()] = placementPose(item);
    for (const auto &id : m.order) {
        const Json &task = m.tasks.at(id);
        Json frames = Json::object();
        std::map<std::string, const Json *> named;
        for (const auto &frame : task.at("frames")) {
            named[frame.at("id").get<std::string>()] = &frame;
            frames[frame.at("id").get<std::string>()] = frame;
        }
        m.state["tasks"][id] = {{"frames", frames}};
        if (!placements.count(id))
            invalid("task " + id + " has no placement");
        const Pose &base = placements.at(id);
        auto placed = [&](const Json &parameters) -> Pose {
            if (!parameters.contains("frame"))
                return base;
            const Json &frame = *named.at(parameters.at("frame").get<std::string>());
            return spatial::compose(base, poseFrom(frame.at("position_m"), frame.at("orientation_wxyz")));
        };
        for (const auto &region : task.at("regions")) {
            const std::string region_id = region.at("id").get<std::string>();
            const std::string kind = region.at("type").get<std::string>();
            const Json &parameters = region.at("parameters");
            if (kind == "perforated_panel") {
                m.panels.push_back({id, region_id, PerforatedPanel(parameters, base)});
            } else if (kind == "rectangular_portal") {
                m.portals.push_back(
                    {id, region_id, PortalTracker(parameters, placed(parameters), envelope, m.floor_z)});
            } else if (kind == "open_crate") {
                m.crates.push_back({id, region_id, OpenCrate(parameters, placed(parameters))});
            } else if (kind == "proximity_target") {
                ProximityTarget target(parameters, placed(parameters),
                                       probePoint(scenario.robot, parameters.at("probe_mechanism_type")));
                if (parameters.contains("indicator"))
                    for (auto it = parameters.at("indicator").begin(); it != parameters.at("indicator").end(); ++it)
                        target.indicator[it.key()] = it.value().get<std::string>();
                m.targets.push_back({id, region_id, std::move(target)});
            } else if (kind == "surface") {
                std::vector<std::pair<std::string, Vec3>> positions;
                for (const auto &name : parameters.at("facing").at("targets")) {
                    const Json &frame = *named.at(name.get<std::string>());
                    positions.emplace_back(
                        name.get<std::string>(),
                        spatial::compose(base, poseFrom(frame.at("position_m"), frame.at("orientation_wxyz")))
                            .translation);
                }
                m.surfaces.push_back(
                    {id, region_id, SurfaceTracker(parameters, placed(parameters), envelope, surface, positions)});
            } else if (kind == "box") { // containment is judged by the rigid-body prop world
                continue;
            } else if (kind == "turn_zone") {
                m.turns.push_back({id, region_id, TurnTracker(parameters, placed(parameters))});
            } else {
                invalid("task " + id + ": unsupported region '" + kind + "'");
            }
        }
        for (const auto &event : task.at("events")) {
            const std::string type = event.at("type").get<std::string>();
            if (type == "contact" && event.at("parameters").at("with") == "robot")
                continue; // contacts arrive via record()
            if (!kEventTypes.count(type))
                invalid("task " + id + ": unsupported event '" + type + "'");
        }
        for (const auto &rule : task.at("scoring")) {
            if (rule.at("type") != "event_points")
                invalid("task " + id + ": unsupported scoring '" + rule.at("type").get<std::string>() + "'");
            m.score_rules.emplace_back(id, rule);
        }
    }
    for (const auto &entry : scenario.tasks.at("scoring_rules")) {
        const std::string name = entry.at("name").get<std::string>();
        const auto found = rules.find(name);
        if (found == rules.end())
            invalid("unknown rules '" + name + "'");
        m.scoring.push_back({name, found->second, entry.at("parameters")});
    }
    m.reset(0);
}

TaskRuntime::~TaskRuntime() = default;

void TaskRuntime::reset(std::int64_t time_ns) {
    impl_->reset(time_ns);
}

void TaskRuntime::start(std::int64_t time_ns, const Json &options) {
    Impl &m = *impl_;
    m.checkTime(time_ns);
    if (!options.is_null())
        for (auto it = options.begin(); it != options.end(); ++it)
            if (!m.options.contains(it.key()))
                invalid("unknown run options: " + it.key());
    m.reset(time_ns);
    Json &run = m.state["run"];
    if (!options.is_null())
        for (auto it = options.begin(); it != options.end(); ++it)
            run["options"][it.key()] = it.value();
    run["running"] = true;
    run["started_ns"] = time_ns;
}

Events TaskRuntime::stop(std::int64_t time_ns) {
    Impl &m = *impl_;
    m.checkTime(time_ns);
    Events events;
    Events result = m.guarded([&] {
        for (auto &portal : m.portals)
            for (const auto &event : portal.value.finishAttempt(time_ns))
                for (auto &e : m.portalEvents(portal.task, portal.region, event))
                    events.push_back(std::move(e));
        return m.evaluate(events);
    });
    m.last_time = time_ns;
    if (m.state["run"]["running"].get<bool>()) {
        m.state["run"]["running"] = false;
        m.state["run"]["stopped_ns"] = time_ns;
    }
    return result;
}

Json TaskRuntime::snapshot() const {
    impl_->refreshLatched();
    return impl_->state;
}

Json TaskRuntime::describe() const {
    Impl &m = *impl_;
    Json fields = Json::object();
    const Json state = snapshot();
    for (std::size_t i = 0; i < m.scoring.size(); ++i) {
        const Json extra = m.instances[i]->describe(state, m.scoring[i].parameters);
        if (!finiteJson(extra) || !extra.is_object())
            invalid("status function must return a finite object");
        for (auto it = extra.begin(); it != extra.end(); ++it)
            fields[it.key()] = it.value();
    }
    return fields;
}

Json TaskRuntime::feed(const Events &events, const Json &context) const {
    Impl &m = *impl_;
    Json items = Json::array();
    const Json state = snapshot();
    for (std::size_t i = 0; i < m.scoring.size(); ++i) {
        const Json part = m.instances[i]->feed(state, events, context, m.scoring[i].parameters);
        if (!finiteJson(part) || !part.is_array())
            invalid("feed function must return a finite list");
        for (const auto &item : part)
            items.push_back(item);
    }
    return items;
}

Json TaskRuntime::indicators() const {
    Json out = Json::array();
    for (const auto &t : impl_->targets) {
        Json colors = Json::object();
        for (const auto &c : t.value.indicator)
            colors[c.first] = c.second;
        const auto &q = t.value.face_world.rotation;
        out.push_back({{"task", t.task},
                       {"region", t.region},
                       {"position_m", toJson(t.value.face_world.translation)},
                       {"orientation_wxyz", Json::array({q.w(), q.x(), q.y(), q.z()})},
                       {"latched", t.value.latched},
                       {"colors", colors}});
    }
    return out;
}

Events TaskRuntime::record(std::int64_t time_ns, const Events &events) {
    Impl &m = *impl_;
    m.checkTime(time_ns);
    Events inputs;
    for (const auto &event : events) {
        if (!event.is_object())
            invalid("recorded events must be objects");
        const Json region = event.contains("region") && event.at("region").is_string() ? event.at("region") : Json("");
        Event item = makeEvent(event.at("id"), event.at("type"), event.at("task"), region,
                               event.at("time_ns").get<std::int64_t>(), event.at("data"));
        if (!m.tasks.count(item.at("task").get<std::string>()) || item.at("time_ns").get<std::int64_t>() != time_ns)
            invalid("recorded events must belong to a selected task and this time");
        inputs.push_back(std::move(item));
    }
    Events result = m.guarded([&] { return m.evaluate(inputs); });
    m.last_time = time_ns;
    return result;
}

Events TaskRuntime::observe(std::int64_t time_ns, const spatial::Pose &world_reference) {
    Impl &m = *impl_;
    m.checkTime(time_ns);
    Events result = m.guarded([&] {
        const Pose pose = ownedPose(world_reference);
        Events events;
        auto append = [&events](Events more) {
            for (auto &e : more)
                events.push_back(std::move(e));
        };
        for (auto &portal : m.portals)
            for (const auto &event : portal.value.observe(time_ns, pose))
                append(m.portalEvents(portal.task, portal.region, event));
        for (auto &t : m.targets)
            append(m.factEvents(t.task, t.region, time_ns, t.value.observe(time_ns, pose)));
        for (auto &t : m.surfaces)
            append(m.factEvents(t.task, t.region, time_ns, t.value.observe(time_ns, pose)));
        for (auto &t : m.turns)
            append(m.factEvents(t.task, t.region, time_ns, t.value.observe(time_ns, pose)));
        return m.evaluate(events);
    });
    m.last_time = time_ns;
    return result;
}

Events TaskRuntime::releaseProjectile(std::int64_t time_ns, int id, const std::string &mechanism_type,
                                      const Eigen::Vector3d &tip_world, double radius_m, double length_m) {
    Impl &m = *impl_;
    m.checkTime(time_ns);
    if (id < 0 || m.projectiles.count(id))
        invalid("projectile identifiers must be unique nonnegative integers");
    if (mechanism_type != "launcher" && mechanism_type != "dropper")
        invalid("unsupported projectile mechanism type");
    const Vec3 tip = finite(tip_world, "projectile tip");
    if (!std::isfinite(radius_m) || radius_m <= 0)
        invalid("projectile radius must be positive and finite");
    if (!std::isfinite(length_m) || length_m <= 0)
        invalid("projectile length must be positive and finite");
    Events events;
    std::set<std::string> tasks;
    for (const auto &panel : m.panels) {
        for (const auto &b : m.bindings(panel.task)) {
            const Json &p = b.at("parameters");
            if (b.at("type") == "hit" && p.at("region") == panel.region &&
                p.at("projectile_mechanism_type") == mechanism_type) {
                tasks.insert(panel.task);
                events.push_back(makeEvent("payload_released", "payload_released", panel.task, panel.region, time_ns,
                                           {{"projectile_id", id},
                                            {"mechanism_type", mechanism_type},
                                            {"release_distance_m", panel.value.releaseDistance(tip)}}));
                break;
            }
        }
    }
    for (const auto &crate : m.crates) {
        if (tasks.count(crate.task))
            continue;
        bool any = false;
        for (const auto &b : m.bindings(crate.task)) {
            if (b.at("type") != "payload_landing")
                continue;
            const Json &p = b.at("parameters");
            const auto &types = p.at("projectile_mechanism_types");
            const bool listed = std::find(types.begin(), types.end(), Json(mechanism_type)) != types.end();
            if (!listed)
                continue;
            const std::string region = p.at("region").get<std::string>();
            any = std::any_of(m.crates.begin(), m.crates.end(),
                              [&](const auto &c) { return c.task == crate.task && c.region == region; });
            if (any)
                break;
        }
        if (any) {
            tasks.insert(crate.task);
            events.push_back(makeEvent("payload_released", "payload_released", crate.task, crate.region, time_ns,
                                       {{"projectile_id", id}, {"mechanism_type", mechanism_type}}));
        }
    }
    Events result = m.guarded([&] { return m.evaluate(events); });
    Projectile projectile;
    projectile.mechanism = mechanism_type;
    projectile.radius = radius_m;
    projectile.length = length_m;
    projectile.released_ns = time_ns;
    projectile.tasks = std::move(tasks);
    m.projectiles.emplace(id, std::move(projectile));
    m.last_time = time_ns;
    return result;
}

ProjectileStep TaskRuntime::stepProjectile(std::int64_t time_ns, int id, const Eigen::Vector3d &start_center,
                                           const Eigen::Vector3d &end_center, const Eigen::Vector3d &axis_in,
                                           const Eigen::Vector3d &velocity_in) {
    Impl &m = *impl_;
    m.checkTime(time_ns);
    if (id < 0 || !m.projectiles.count(id))
        invalid("projectile must be released before observing its motion");
    const Vec3 start = finite(start_center, "projectile start center");
    const Vec3 end = finite(end_center, "projectile end center");
    const Vec3 axis = finite(axis_in, "projectile axis");
    if (std::abs(axis.norm() - 1) > 1e-6)
        invalid("projectile axis must be unit length");
    const Vec3 velocity = finite(velocity_in, "projectile velocity");
    Projectile &state = m.projectiles.at(id);
    if (!state.active) {
        m.last_time = time_ns;
        ProjectileStep step;
        step.stop = true;
        return step;
    }
    const std::string &mechanism = state.mechanism;
    const double radius = state.radius;
    Events events;
    Vec3 new_position = end, current = velocity;
    bool stop = false;
    for (const auto &panel : m.panels) {
        const auto hit = panel.value.intersect(start, end, axis, radius);
        if (!hit)
            continue;
        state.scored = true;
        if (hit->outcome == "blocked") {
            new_position = panel.value.worldPoint(hit->point_local);
            current = Vec3::Zero();
            stop = true;
        }
        for (const auto &binding : m.bindings(panel.task)) {
            const Json &p = binding.at("parameters");
            if (binding.at("type") == "hit" && p.at("region") == panel.region &&
                p.at("projectile_mechanism_type") == mechanism && p.at("outcome") == hit->outcome)
                events.push_back(makeEvent(binding.at("id"), "hit", panel.task, panel.region, time_ns,
                                           {{"projectile_id", id},
                                            {"mechanism_type", mechanism},
                                            {"outcome", hit->outcome},
                                            {"hole_id", hit->hole_id},
                                            {"hole_class", hit->hole_class},
                                            {"hole_size", hit->hole_size},
                                            {"hit_point_local", toJson(hit->point_local)},
                                            {"stop_projectile", p.at("stop_projectile")}}));
        }
    }
    for (const auto &crate : m.crates) {
        const std::pair<std::string, std::string> key{crate.task, crate.region};
        const CrateStep step =
            crate.value.step(start, new_position, current, axis, radius, state.length, state.entered.count(key) > 0);
        new_position = step.position;
        current = step.velocity;
        if (step.entered)
            state.entered.insert(key);
        if (step.outcome.empty())
            continue;
        stop = true;
        if (state.scored)
            continue;
        state.scored = true;
        for (const auto &binding : m.bindings(crate.task)) {
            const Json &p = binding.at("parameters");
            if (binding.at("type") != "payload_landing" || p.at("region") != crate.region ||
                p.at("outcome") != step.outcome)
                continue;
            const auto &types = p.at("projectile_mechanism_types");
            if (std::find(types.begin(), types.end(), Json(mechanism)) == types.end())
                continue;
            events.push_back(makeEvent(binding.at("id"), "payload_landing", crate.task, crate.region, time_ns,
                                       {{"projectile_id", id},
                                        {"mechanism_type", mechanism},
                                        {"outcome", step.outcome},
                                        {"detail", step.detail},
                                        {"region_class", crate.value.crateClass()}}));
        }
    }
    const double vertical = radius + std::max(0.0, state.length / 2 - radius) * std::abs(axis[2]);
    std::string reason;
    const double floor_limit = m.floor_z + vertical;
    if (new_position[2] < floor_limit) {
        new_position[2] = floor_limit;
        current = Vec3::Zero();
        stop = true;
        reason = "pool_floor";
    }
    const Vec3 local = spatial::apply(m.pool_from_world, new_position);
    if (local[0] < 0 || local[0] > m.pool_length || local[1] < 0 || local[1] > m.pool_width ||
        static_cast<double>(time_ns - state.released_ns) / 1e9 > kMaxPayloadAgeS) {
        current = Vec3::Zero();
        stop = true;
        if (reason.empty())
            reason = "pool_wall_or_timeout";
    }
    if (!reason.empty() && !state.scored) {
        state.scored = true;
        for (const auto &task : state.tasks)
            events.push_back(makeEvent("payload_miss", "miss", task, "", time_ns,
                                       {{"projectile_id", id}, {"mechanism_type", mechanism}, {"reason", reason}}));
    }
    ProjectileStep result;
    result.events = m.guarded([&] { return m.evaluate(events); });
    state.active = !stop;
    m.last_time = time_ns;
    result.stop = stop;
    if (new_position != end)
        result.position_world = new_position;
    if (current != velocity)
        result.velocity_world = current;
    return result;
}
} // namespace robotics::session

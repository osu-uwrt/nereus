#pragma once
// Task observation and scoring (port of python/.../task_runtime.py, task_regions.py,
// task_projectiles.py, task_zones.py). Generic geometric observers are built from task-pack
// regions/events by type name; competition logic lives in a Rules implementation selected by
// name from the task pack. Events are JSON objects with exactly the Python layout:
// {id, type, task, region, time_ns, data}.
#include <robotics/session/scenario.hpp>
#include <robotics/spatial/frames.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace robotics::session {
using Event = Json;
using Events = std::vector<Event>;

// Physical correction a task applies to a payload it judged (port of ProjectileStep).
struct ProjectileStep {
    Events events;
    bool stop{false};
    std::optional<Eigen::Vector3d> position_world, velocity_world;
};

// Competition rules: pure function of read-only run state + ordered events -> score rows and
// derived events (port of the Python hook contract `evaluate(state, events, parameters)`).
// `describe`/`feed` mirror the hook's optional status/feed functions used for run_score and
// task_events. Implementations must not keep state outside `state` except caches they can rebuild.
class Rules {
  public:
    virtual ~Rules() = default;
    // Returns {"scores": [{row, points}], "events": [...]}.
    virtual Json evaluate(const Json &state, const Events &events, const Json &parameters) = 0;
    virtual Json describe(const Json &state, const Json &parameters) {
        (void)state, (void)parameters;
        return Json::object();
    }
    virtual Json feed(const Events &events, const Json &context, const Json &parameters) {
        (void)events, (void)context, (void)parameters;
        return Json::array();
    }
    // The Python feed hook also receives the frozen run snapshot (`TaskRuntime::snapshot()`);
    // TaskRuntime calls this overload. The default forwards to the stateless one.
    virtual Json feed(const Json &state, const Events &events, const Json &context,
                      const Json &parameters) {
        (void)state;
        return feed(events, context, parameters);
    }
};
// Explicit, constructed registry (no global self-registration). Keys are the task pack's
// scoring hook `rules` name (e.g. "robosub_2026"); later a "python" entry can wrap user hooks.
using RulesFactory = std::function<std::unique_ptr<Rules>()>;
using RulesRegistry = std::map<std::string, RulesFactory>;

class TaskRuntime {
  public:
    // task_ids: optional subset; unsupported selected region/event types throw.
    TaskRuntime(const ResolvedScenario &scenario, const RulesRegistry &rules,
                const std::vector<std::string> *task_ids = nullptr);
    ~TaskRuntime();
    TaskRuntime(const TaskRuntime &) = delete;
    TaskRuntime &operator=(const TaskRuntime &) = delete;

    void reset(std::int64_t time_ns = 0);
    void start(std::int64_t time_ns, const Json &options = Json::object());
    Events stop(std::int64_t time_ns);
    Json snapshot() const; // {run, scores, history, tasks, environment, latched, ...} as in Python

    // Per tick: robot reference-frame pose (never a COM pose by implication).
    Events observe(std::int64_t time_ns, const spatial::Pose &world_reference);
    // Events produced by another physical owner (prop world) at this time.
    Events record(std::int64_t time_ns, const Events &events);
    Events releaseProjectile(std::int64_t time_ns, int id, const std::string &mechanism_type,
                             const Eigen::Vector3d &tip_world, double radius_m, double length_m);
    ProjectileStep stepProjectile(std::int64_t time_ns, int id, const Eigen::Vector3d &start_center,
                                  const Eigen::Vector3d &end_center, const Eigen::Vector3d &axis,
                                  const Eigen::Vector3d &velocity);

    Json describe() const;                            // run_score extras from the rules
    Json feed(const Events &events, const Json &context) const; // task_events records
    Json indicators() const;                          // e.g. magnet light states/poses

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace robotics::session

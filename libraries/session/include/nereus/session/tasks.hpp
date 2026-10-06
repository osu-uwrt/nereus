#pragma once
// Task observation and scoring. Generic geometric observers are built from task-pack
// regions/events by type name; competition logic lives in a Rules implementation selected by
// name from the task pack. Events are JSON objects with this layout:
// {id, type, task, region, time_ns, data}.
#include <nereus/session/scenario.hpp>
#include <nereus/spatial/frames.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace nereus::session {
using Event = Json;
using Events = std::vector<Event>; // in production order

// Physical correction a task applies to a payload it judged.
struct ProjectileStep {
    Events events;
    bool stop{false};
    std::optional<Eigen::Vector3d> position_world, velocity_world;
};

// Competition rules: pure function of read-only run state + ordered events -> score rows and
// derived events. `describe` adds run_score fields and `feed` adds task_events records; both
// default to nothing. Implementations must not keep state outside `state` except caches they can rebuild.
class Rules {
  public:
    virtual ~Rules() = default;
    // Returns {"scores": [{row, points}], "events": [...]}.
    virtual Json evaluate(const Json &state, const Events &events, const Json &parameters) = 0;

    // Extra run_score fields for the operator scorecard.
    virtual Json describe(const Json &state, const Json &parameters) {
        (void)state, (void)parameters;
        return Json::object();
    }

    // task_events feed records for `events`; `context` carries per-payload details from the session.
    virtual Json feed(const Events &events, const Json &context, const Json &parameters) {
        (void)events, (void)context, (void)parameters;
        return Json::array();
    }
    // TaskRuntime calls this overload with the frozen run snapshot (`TaskRuntime::snapshot()`). The default forwards to
    // the stateless one.
    virtual Json feed(const Json &state, const Events &events, const Json &context, const Json &parameters) {
        (void)state;
        return feed(events, context, parameters);
    }
};

// Explicit, constructed registry (no global self-registration). Keys are the task pack's
// `scoring_rules[].name` (e.g. "robosub_2026").
using RulesFactory = std::function<std::unique_ptr<Rules>()>;
using RulesRegistry = std::map<std::string, RulesFactory>;

// Every selected task's observers and scoring for one run. Times are nanoseconds that may not go backwards
// until reset(); a failure while observing or scoring poisons the runtime until reset().
class TaskRuntime {
  public:
    // task_ids: optional subset; unsupported selected region/event types throw.
    TaskRuntime(const ResolvedScenario &scenario, const RulesRegistry &rules,
                const std::vector<std::string> *task_ids = nullptr);
    ~TaskRuntime();
    TaskRuntime(const TaskRuntime &) = delete;
    TaskRuntime &operator=(const TaskRuntime &) = delete;

    // Clear scores, history and trackers (the run is open again only if the scenario auto-starts).
    void reset(std::int64_t time_ns = 0);
    // Reset and open a scored run with these run options.
    void start(std::int64_t time_ns, const Json &options = Json::object());
    // Close the run; returns the events from finishing open gate attempts (plus any the rules derive).
    Events stop(std::int64_t time_ns);
    Json snapshot() const; // {run, scores, history, tasks, environment, latched, ...} as in Python

    // Per tick: robot reference-frame pose (never a COM pose by implication).
    Events observe(std::int64_t time_ns, const spatial::Pose &world_reference);
    // Events produced by another physical owner (prop world) at this time.
    Events record(std::int64_t time_ns, const Events &events);

    // Payloads: announce a release (tip = leading end, world), then judge each step's swept centre segment.
    Events releaseProjectile(std::int64_t time_ns, int id, const std::string &mechanism_type,
                             const Eigen::Vector3d &tip_world, double radius_m, double length_m);
    ProjectileStep stepProjectile(std::int64_t time_ns, int id, const Eigen::Vector3d &start_center,
                                  const Eigen::Vector3d &end_center, const Eigen::Vector3d &axis,
                                  const Eigen::Vector3d &velocity);

    Json describe() const;                                      // run_score extras from the rules
    Json feed(const Events &events, const Json &context) const; // task_events records
    Json indicators() const;                                    // e.g. magnet light states/poses

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace nereus::session

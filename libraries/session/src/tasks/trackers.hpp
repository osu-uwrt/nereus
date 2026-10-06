#pragma once
// Generic sampled task-geometry judges (ports of python/.../task_regions.py, task_projectiles.py
// and task_zones.py). Pure geometry: no clock ownership, ledger or competition rule. Times are
// nonnegative integer nanoseconds that never decrease until reset.
#include <nereus/session/scenario.hpp>
#include <nereus/spatial/frames.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace nereus::session::tasks {
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using spatial::Pose;

// A derived fact: (kind, data); the runtime binds it to declared events.
struct Fact {
    std::string kind;
    Json data;
};

// Validation helpers (throw std::invalid_argument, the analog of the Python ValueError).
double number(const Json &value, const std::string &field, bool positive = false);
Vec3 vec3(const Json &value, const std::string &name);
Vec3 finite(const Vec3 &value, const std::string &name);
void requireKeys(const Json &value, std::initializer_list<const char *> expected, const std::string &field,
                 std::initializer_list<const char *> optional = {});
Pose ownedPose(const Pose &pose); // validated, renormalized copy
Pose poseFrom(const Json &position_m, const Json &orientation_wxyz);
Mat3 rotationMatrix(const Pose &pose);
Json toJson(const Vec3 &v);

// A gate passage (`pass_through`) or, when the attempt ends, its summary (`attempt_finished`).
struct PortalEvent {
    std::string kind, from_side, to_side;
    std::int64_t time_ns{0};
    std::optional<Vec3> crossing_point_local; // where the reference origin crossed the plane, task frame
    double envelope_top_world{0};             // highest envelope vertex z
    Vec3 rotation_vector_body{Vec3::Zero()};  // rotation accumulated near the gate during the attempt
    int attempt_id{0};
    std::optional<bool> depth_overlap; // envelope overlaps depth_band_m at the crossing (set only with a band)
    Json data() const;                 // asdict() minus kind and time_ns
};

// Gate passage through a task-frame x plane: tracks which side the robot is on, checks that its envelope fits
// the opening (|y| and z bounds, optional floor clearance) when it changes side, and groups passages into
// attempts that start within approach_radius_m of the gate and end when the robot leaves that radius.
class PortalTracker {
  public:
    PortalTracker(const Json &parameters, const Pose &world_from_task,
                  const std::vector<Vec3> &envelope_reference_vertices, double floor_z);
    void reset();
    // End the current attempt now, reporting its passages.
    std::vector<PortalEvent> finishAttempt(std::int64_t time_ns);
    std::vector<PortalEvent> observe(std::int64_t time_ns, const Pose &world_reference);

  private:
    // One robot pose sample, in the world and in the task frame.
    struct Observation {
        Vec3 position, local_position;
        Mat3 rotation, local_rotation;
        double top;
    };

    // Time spent near the gate; keeps the latest passage per from_side.
    struct Attempt {
        int identifier;
        Vec3 turns;
        std::vector<std::pair<std::string, PortalEvent>> passages; // insertion ordered
    };

    // The state advance() computes for observe() to commit.
    struct Advance {
        std::vector<PortalEvent> events;
        int entry;
        std::optional<Vec3> crossing;
        std::optional<Attempt> attempt;
        int next_id;
    };
    void checkTime(std::int64_t time_ns) const;
    static std::vector<PortalEvent> finished(const std::optional<Attempt> &attempt, std::int64_t time_ns, double top);
    Advance advance(std::int64_t time_ns, const Observation &current, const std::vector<Vec3> &world,
                    const std::vector<Vec3> &local) const;

    // Configuration.
    double floor_, offset_, width_, top_, radius_, max_step_;
    bool floor_check_;
    std::optional<std::pair<double, double>> band_;
    bool full_envelope_;
    bool check_crossing_orientation_, check_completion_, check_origin_;
    std::vector<Vec3> vertices_;
    Pose task_from_world_;
    Mat3 task_rotation_;
    // state
    std::optional<std::int64_t> time_ns_;
    std::optional<Observation> previous_;
    int entry_side_{0}; // 0 = none, else the side (+1 / -1) the robot was last fully on
    std::optional<Vec3> crossing_;
    std::optional<Attempt> attempt_;
    int next_attempt_{1};
};

// Where a projectile met a perforated panel, and through which hole if it passed.
struct PanelHit {
    std::string outcome; // pass | blocked
    Vec3 point_local;
    std::string hole_id, hole_class, hole_size;
};

// A square panel in a task-frame x plane with round holes (positions and radii in uv units of the panel).
// A projectile passes a hole when its radius, widened by 1 / cos of its incidence angle, fits inside it.
class PerforatedPanel {
  public:
    PerforatedPanel(const Json &parameters, const Pose &world_from_task);
    Vec3 worldPoint(const Vec3 &point_local) const;
    // Distance of a point from the panel plane.
    double releaseDistance(const Vec3 &tip_world) const;
    // Judge the segment a projectile swept this step; nullopt when it did not reach the panel square.
    std::optional<PanelHit> intersect(const Vec3 &start_world, const Vec3 &end_world, const Vec3 &axis_world,
                                      double radius_m) const;

  private:
    struct Hole {
        std::string id, category, size;
        Eigen::Vector2d center;
        double radius;
    };

    Pose world_from_task_, task_from_world_;
    double offset_, half_, min_cosine_;
    std::vector<Hole> holes_;
};

// A projectile's state after one OpenCrate::step.
struct CrateStep {
    Vec3 position, velocity;
    bool entered{false};
    std::string outcome; // inside | blocked | "" (still flying)
    std::string detail;  // floor | rim for a blocked landing
};

// An open-topped box: a payload that comes down through the inner opening is `entered`,
// walls deflect it (damped and tangential only), and it ends on the floor (inside / blocked) or on the rim.
class OpenCrate {
  public:
    OpenCrate(const Json &parameters, const Pose &world_from_crate);
    const std::string &crateClass() const {
        return class_;
    }
    CrateStep step(const Vec3 &old_world, const Vec3 &new_world, const Vec3 &velocity_world, const Vec3 &axis_world,
                   double radius_m, double length_m, bool entered) const;

  private:
    std::string class_;
    double outer_, inner_, height_; // outer and inner (liner) half widths, inner depth above the base
    Mat3 rotation_;
    Vec3 origin_;
};

// Touch target: latches `activate` once the robot's probe point (reference frame) stays within
// trigger_distance_m of the face sensor for dwell_s.
class ProximityTarget {
  public:
    ProximityTarget(const Json &parameters, const Pose &world_from_frame, const Vec3 &probe_reference);
    void reset();
    std::vector<Fact> observe(std::int64_t time_ns, const Pose &world_reference);

    bool latched{false};
    Pose face_world;
    std::map<std::string, std::string> indicator; // viewer indicator description, filled by the runtime

  private:
    double distance_, dwell_ns_;
    Vec3 sensor_, probe_; // sensor point in the world, probe point in the reference frame
    std::optional<std::int64_t> time_;
    std::int64_t dwell_{0};
};

// Surfacing inside a regular octagon: after being fully submerged, the envelope must break the surface while
// entirely inside the octagon for dwell_s (`surface_reached`); then facing one of the targets for facing.dwell_s
// gives `facing_reached`. Breaking the surface outside the octagon is a one-time `breach`.
class SurfaceTracker {
  public:
    SurfaceTracker(const Json &parameters, const Pose &world_from_frame,
                   const std::vector<Vec3> &envelope_reference_vertices, double surface_z,
                   const std::vector<std::pair<std::string, Vec3>> &target_positions_world);
    void reset();
    std::vector<Fact> observe(std::int64_t time_ns, const Pose &world_reference);

  private:
    void clear();
    // Heading (body +x in the horizontal plane) against the nearest target bearing.
    std::vector<Fact> facingStep(const Vec3 &position, const Mat3 &rotation, std::int64_t dt);

    double apothem_, margin_, dwell_ns_, max_step_, tolerance_, facing_ns_, surface_;
    std::vector<std::pair<std::string, Vec3>> targets_; // insertion (facing.targets) order
    std::vector<Vec3> vertices_;
    Pose frame_from_world_;
    Mat3 frame_rotation_;
    // state
    std::optional<std::int64_t> time_;
    std::optional<Vec3> previous_;
    bool submerged_{false}, surfaced_{false}, breached_{false};
    std::int64_t dwell_{0};
    double facing_dwell_{0};
    std::optional<std::string> facing_, achieved_;
};

// Yaw rotation judged within radius_m of a frame: unwrapped yaw travel from the start to the peak is reported
// as whole turns (`rotation_judged`) when the robot stops turning (settles for dwell_s), reverses, or leaves.
class TurnTracker {
  public:
    TurnTracker(const Json &parameters, const Pose &world_from_frame);
    void reset();
    // Start counting afresh on the next sample (triggered by one of `restart_events`).
    void restart();
    std::vector<Fact> observe(std::int64_t time_ns, const Pose &world_reference);
    std::vector<std::pair<std::string, std::string>> restart_events; // (task, id)

  private:
    void clear();
    // A rotation_judged fact when the travel since start reaches min_travel_deg.
    std::vector<Fact> judge(const std::string &reason) const;

    double radius_, max_step_, tolerance_, settle_, reversal_, minimum_, dwell_ns_;
    Pose frame_from_world_;
    std::optional<std::int64_t> time_;
    std::optional<Vec3> previous_;
    std::optional<double> previous_yaw_;
    bool tracking_{false}, restart_{false}, judged_{false};
    double yaw_{0}, settle_yaw_{0}, start_{0}, peak_{0};
    std::int64_t dwell_{0};
};
} // namespace nereus::session::tasks

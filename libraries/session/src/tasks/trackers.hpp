#pragma once
// Generic sampled task-geometry judges (ports of python/.../task_regions.py, task_projectiles.py
// and task_zones.py). Pure geometry: no clock ownership, ledger or competition rule. Times are
// nonnegative integer nanoseconds that never decrease until reset.
#include <robotics/session/scenario.hpp>
#include <robotics/spatial/frames.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace robotics::session::tasks {
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using spatial::Pose;

// A derived fact: (kind, data); the runtime binds it to declared events.
struct Fact {
    std::string kind;
    Json data;
};

// Validation helpers (throw std::invalid_argument, the analogue of the Python ValueError).
double number(const Json &value, const std::string &field, bool positive = false);
Vec3 vec3(const Json &value, const std::string &name);
Vec3 finite(const Vec3 &value, const std::string &name);
void requireKeys(const Json &value, std::initializer_list<const char *> expected, const std::string &field,
                 std::initializer_list<const char *> optional = {});
Pose ownedPose(const Pose &pose); // validated, renormalized copy
Pose poseFrom(const Json &position_m, const Json &orientation_wxyz);
Mat3 rotationMatrix(const Pose &pose);
Json toJson(const Vec3 &v);

struct PortalEvent {
    std::string kind, from_side, to_side;
    std::int64_t time_ns{0};
    std::optional<Vec3> crossing_point_local;
    double envelope_top_world{0};
    Vec3 rotation_vector_body{Vec3::Zero()};
    int attempt_id{0};
    std::optional<bool> depth_overlap;
    Json data() const; // asdict() minus kind and time_ns
};

class PortalTracker {
  public:
    PortalTracker(const Json &parameters, const Pose &world_from_task,
                  const std::vector<Vec3> &envelope_reference_vertices, double floor_z);
    void reset();
    std::vector<PortalEvent> finishAttempt(std::int64_t time_ns);
    std::vector<PortalEvent> observe(std::int64_t time_ns, const Pose &world_reference);

  private:
    struct Observation {
        Vec3 position, local_position;
        Mat3 rotation, local_rotation;
        double top;
    };
    struct Attempt {
        int identifier;
        Vec3 turns;
        std::vector<std::pair<std::string, PortalEvent>> passages; // insertion ordered
    };
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
    int entry_side_{0}; // 0 = none
    std::optional<Vec3> crossing_;
    std::optional<Attempt> attempt_;
    int next_attempt_{1};
};

struct PanelHit {
    std::string outcome; // pass | blocked
    Vec3 point_local;
    std::string hole_id, hole_class, hole_size;
};

class PerforatedPanel {
  public:
    PerforatedPanel(const Json &parameters, const Pose &world_from_task);
    Vec3 worldPoint(const Vec3 &point_local) const;
    double releaseDistance(const Vec3 &tip_world) const;
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

struct CrateStep {
    Vec3 position, velocity;
    bool entered{false};
    std::string outcome; // inside | blocked | "" (still flying)
    std::string detail;  // floor | rim for a blocked landing
};

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
    double outer_, inner_, height_;
    Mat3 rotation_;
    Vec3 origin_;
};

class ProximityTarget {
  public:
    ProximityTarget(const Json &parameters, const Pose &world_from_frame, const Vec3 &probe_reference);
    void reset();
    std::vector<Fact> observe(std::int64_t time_ns, const Pose &world_reference);
    bool latched{false};
    Pose face_world;
    std::map<std::string, std::string> indicator;

  private:
    double distance_, dwell_ns_;
    Vec3 sensor_, probe_;
    std::optional<std::int64_t> time_;
    std::int64_t dwell_{0};
};

class SurfaceTracker {
  public:
    SurfaceTracker(const Json &parameters, const Pose &world_from_frame,
                   const std::vector<Vec3> &envelope_reference_vertices, double surface_z,
                   const std::vector<std::pair<std::string, Vec3>> &target_positions_world);
    void reset();
    std::vector<Fact> observe(std::int64_t time_ns, const Pose &world_reference);

  private:
    void clear();
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

class TurnTracker {
  public:
    TurnTracker(const Json &parameters, const Pose &world_from_frame);
    void reset();
    void restart();
    std::vector<Fact> observe(std::int64_t time_ns, const Pose &world_reference);
    std::vector<std::pair<std::string, std::string>> restart_events; // (task, id)

  private:
    void clear();
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
} // namespace robotics::session::tasks

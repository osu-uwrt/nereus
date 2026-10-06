// Geometry judges used by the task runtime: gate passages, perforated panels, open crates, proximity
// targets, surfacing and turns. Numeric details mirror the Python originals so results match exactly.
#include "tasks/trackers.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace nereus::session::tasks {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0; // math.radians(x) == x * (pi / 180)

[[noreturn]] void invalid(const std::string &message) {
    throw std::invalid_argument(message);
}

// Python's float floor division (a // b) for positive b.
double floorDivide(double a, double b) {
    double mod = std::fmod(a, b);
    double div = (a - mod) / b;
    if (mod != 0.0 && ((b < 0) != (mod < 0)))
        div -= 1.0;
    double floored;
    if (div != 0.0) {
        floored = std::floor(div);
        if (div - floored > 0.5)
            floored += 1.0;
    } else {
        floored = std::copysign(0.0, a / b);
    }
    return floored;
}

// Angle wrapped to (-pi, pi].
double wrapAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

// Advance a tracker's sample clock: dt since the previous sample (0 on the first), and reject time going back.
void checkSampleTime(std::optional<std::int64_t> &current, std::int64_t time_ns, std::int64_t &dt, const char *what) {
    dt = current ? time_ns - *current : 0;
    if (dt < 0)
        invalid(std::string(what) + " time must not decrease without reset");
    current = time_ns;
}

// Segment/plane crossing that ignores repeat hits from a start exactly on the plane.
std::optional<Vec3> crossing(const Vec3 &start, const Vec3 &end, int coordinate, double plane) {
    const double a = start[coordinate] - plane, b = end[coordinate] - plane;
    if (a * b > 0 || a == b || a == 0)
        return std::nullopt;
    return Vec3(start + (end - start) * (-a / (b - a)));
}

// Rotation vector (axis * angle) taking `before` to `after`, in the `before` body frame.
Vec3 rotationDelta(const Mat3 &before, const Mat3 &after) {
    const Mat3 r = before.transpose() * after;
    Vec3 v(r(2, 1) - r(1, 2), r(0, 2) - r(2, 0), r(1, 0) - r(0, 1));
    v /= 2;
    const double magnitude = v.norm();
    const double angle = std::atan2(magnitude, std::clamp((r.trace() - 1) / 2, -1.0, 1.0));
    return magnitude > 1e-9 ? Vec3(v * (angle / magnitude)) : v;
}

// Envelope vertices placed at a pose.
std::vector<Vec3> transformed(const std::vector<Vec3> &vertices, const Mat3 &rotation, const Vec3 &position) {
    std::vector<Vec3> out;
    out.reserve(vertices.size());
    for (const auto &v : vertices)
        out.emplace_back(rotation * v + position);
    return out;
}

bool allFinite(const std::vector<Vec3> &values) {
    return std::all_of(values.begin(), values.end(), [](const Vec3 &v) { return v.allFinite(); });
}

std::vector<Vec3> checkedVertices(const std::vector<Vec3> &vertices, const char *what, bool need_finite) {
    if (vertices.empty() || (need_finite && !allFinite(vertices)))
        invalid(std::string(what) + " envelope must be a nonempty finite N-by-3 array");
    return vertices;
}
} // namespace

double number(const Json &value, const std::string &field, bool positive) {
    if (!value.is_number())
        invalid(field + " must be a finite number");
    const double result = value.get<double>();
    if (!std::isfinite(result) || (positive && result <= 0))
        invalid(field + " must be " + (positive ? "positive and " : "") + "finite");
    return result;
}

Vec3 vec3(const Json &value, const std::string &name) {
    if (!value.is_array() || value.size() != 3)
        invalid(name + " must have three finite coordinates");
    Vec3 out;
    for (int i = 0; i < 3; ++i) {
        if (!value[i].is_number())
            invalid(name + " must have three finite coordinates");
        out[i] = value[i].get<double>();
    }
    return finite(out, name);
}

Vec3 finite(const Vec3 &value, const std::string &name) {
    if (!value.allFinite())
        invalid(name + " must have three finite coordinates");
    return value;
}

void requireKeys(const Json &value, std::initializer_list<const char *> expected, const std::string &field,
                 std::initializer_list<const char *> optional) {
    bool ok = value.is_object();
    if (ok) {
        std::set<std::string> known;
        for (const char *key : expected) {
            known.insert(key);
            ok = ok && value.contains(key);
        }
        for (const char *key : optional)
            known.insert(key);
        for (auto it = value.begin(); ok && it != value.end(); ++it)
            ok = known.count(it.key()) > 0;
    }
    if (!ok)
        invalid(field + " has missing or unexpected keys");
}

Pose ownedPose(const Pose &pose) {
    spatial::validate(pose);
    Pose result = pose;
    result.rotation.normalize();
    return result;
}

Pose poseFrom(const Json &position_m, const Json &orientation_wxyz) {
    const Vec3 p = vec3(position_m, "position_m");
    if (!orientation_wxyz.is_array() || orientation_wxyz.size() != 4)
        invalid("orientation_wxyz must have four coordinates");
    Pose pose;
    pose.translation = p;
    pose.rotation = Eigen::Quaterniond(orientation_wxyz[0].get<double>(), orientation_wxyz[1].get<double>(),
                                       orientation_wxyz[2].get<double>(), orientation_wxyz[3].get<double>());
    return ownedPose(pose);
}

Mat3 rotationMatrix(const Pose &pose) {
    Eigen::Quaterniond q = pose.rotation;
    q.normalize();
    return q.toRotationMatrix();
}

Json toJson(const Vec3 &v) {
    return Json::array({v[0], v[1], v[2]});
}

// ---------------------------------------------------------------------------------- portals
Json PortalEvent::data() const {
    return {{"from_side", from_side},
            {"to_side", to_side},
            {"crossing_point_local", crossing_point_local ? toJson(*crossing_point_local) : Json()},
            {"envelope_top_world", envelope_top_world},
            {"rotation_vector_body", toJson(rotation_vector_body)},
            {"attempt_id", attempt_id},
            {"depth_overlap", depth_overlap ? Json(*depth_overlap) : Json()}};
}

PortalTracker::PortalTracker(const Json &parameters, const Pose &world_from_task, const std::vector<Vec3> &envelope,
                             double floor_z) {
    // Validate the parameter shape and the supported options.
    requireKeys(parameters,
                {"plane", "bounds_local", "world_floor_clearance", "crossing_reference", "fit_checks", "traversal",
                 "approach_radius_m", "max_pose_step_m"},
                "portal parameters", {"frame", "depth_band_m"});
    const Json &plane = parameters.at("plane"), &bounds = parameters.at("bounds_local");
    requireKeys(plane, {"axis", "offset_m"}, "plane");
    requireKeys(bounds, {"abs_y_lt_m", "z_lt_m"}, "bounds_local");
    if (plane.at("axis") != "x")
        invalid("portal tracker currently supports only x planes");
    if (parameters.at("crossing_reference") != "robot_reference_origin")
        invalid("unsupported portal crossing_reference");
    const std::string traversal =
        parameters.at("traversal").is_string() ? parameters.at("traversal").get<std::string>() : "";
    if (traversal != "full_envelope" && traversal != "reference_origin")
        invalid("unsupported portal traversal");
    const Json &checks = parameters.at("fit_checks");
    if (!checks.is_array() || checks.empty())
        invalid("unsupported or repeated portal fit_checks");
    std::set<std::string> seen;
    for (const auto &check : checks) {
        if (!check.is_string() || !seen.insert(check.get<std::string>()).second ||
            (check != "envelope_at_crossing_point_with_current_orientation" && check != "envelope_at_completion" &&
             check != "reference_origin_at_crossing"))
            invalid("unsupported or repeated portal fit_checks");
    }
    check_crossing_orientation_ = seen.count("envelope_at_crossing_point_with_current_orientation") > 0;
    check_completion_ = seen.count("envelope_at_completion") > 0;
    check_origin_ = seen.count("reference_origin_at_crossing") > 0;

    // Numbers.
    if (!parameters.at("world_floor_clearance").is_boolean())
        invalid("world_floor_clearance must be boolean");
    floor_ = number(floor_z, "floor_z");
    offset_ = number(plane.at("offset_m"), "plane.offset_m");
    width_ = number(bounds.at("abs_y_lt_m"), "bounds_local.abs_y_lt_m", true);
    top_ = number(bounds.at("z_lt_m"), "bounds_local.z_lt_m");
    radius_ = number(parameters.at("approach_radius_m"), "approach_radius_m", true);
    max_step_ = number(parameters.at("max_pose_step_m"), "max_pose_step_m", true);
    floor_check_ = parameters.at("world_floor_clearance").get<bool>();
    if (parameters.contains("depth_band_m") && !parameters.at("depth_band_m").is_null()) {
        const Json &band = parameters.at("depth_band_m");
        if (!band.is_array() || band.size() != 2)
            invalid("depth_band_m must be [bottom, top]");
        band_ = std::make_pair(number(band[0], "depth_band_m[0]"), number(band[1], "depth_band_m[1]"));
        if (band_->first > band_->second)
            invalid("depth_band_m must be ordered [bottom, top]");
    }
    full_envelope_ = traversal == "full_envelope";
    vertices_ = checkedVertices(envelope, "portal", true);
    task_from_world_ = spatial::inverse(ownedPose(world_from_task));
    task_rotation_ = rotationMatrix(task_from_world_);
    reset();
}

void PortalTracker::reset() {
    time_ns_.reset();
    previous_.reset();
    entry_side_ = 0;
    crossing_.reset();
    attempt_.reset();
    next_attempt_ = 1;
}

void PortalTracker::checkTime(std::int64_t time_ns) const {
    if (time_ns < 0)
        invalid("time_ns must be a nonnegative integer");
    if (time_ns_ && time_ns < *time_ns_)
        invalid("portal time must not decrease without reset");
}

// The attempt's recorded passages re-issued as attempt_finished, with its total rotation.
std::vector<PortalEvent> PortalTracker::finished(const std::optional<Attempt> &attempt, std::int64_t time_ns,
                                                 double top) {
    std::vector<PortalEvent> out;
    if (!attempt)
        return out;
    for (const auto &item : attempt->passages) {
        PortalEvent event = item.second;
        event.kind = "attempt_finished";
        event.time_ns = time_ns;
        event.envelope_top_world = top;
        event.rotation_vector_body = attempt->turns;
        out.push_back(std::move(event));
    }
    return out;
}

std::vector<PortalEvent> PortalTracker::finishAttempt(std::int64_t time_ns) {
    checkTime(time_ns);
    auto events = finished(attempt_, time_ns, previous_ ? previous_->top : 0.0);
    attempt_.reset();
    time_ns_ = time_ns;
    return events;
}

std::vector<PortalEvent> PortalTracker::observe(std::int64_t time_ns, const Pose &world_reference) {
    checkTime(time_ns);

    // Sample the robot and its envelope in the world and in the task frame.
    const Pose pose = ownedPose(world_reference);
    Observation current;
    current.position = pose.translation;
    current.rotation = rotationMatrix(pose);
    current.local_position = spatial::apply(task_from_world_, current.position);
    current.local_rotation = task_rotation_ * current.rotation;
    const auto world = transformed(vertices_, current.rotation, current.position);
    const auto local = transformed(vertices_, current.local_rotation, current.local_position);
    if (!allFinite(world) || !allFinite(local))
        invalid("portal geometry overflow");
    current.top = -std::numeric_limits<double>::infinity();
    for (const auto &v : world)
        current.top = std::max(current.top, v[2]);

    // Compute the next state, then commit it.
    Advance result = advance(time_ns, current, world, local);
    previous_ = current;
    entry_side_ = result.entry;
    crossing_ = result.crossing;
    attempt_ = std::move(result.attempt);
    next_attempt_ = result.next_id;
    time_ns_ = time_ns;
    return std::move(result.events);
}

// Pure step: works on copies of the tracker state and returns them with the events.
PortalTracker::Advance PortalTracker::advance(std::int64_t time_ns, const Observation &current,
                                              const std::vector<Vec3> &world, const std::vector<Vec3> &local) const {
    auto previous = previous_;
    int entry = entry_side_;
    auto cross = crossing_;
    auto attempt = attempt_;
    int next_id = next_attempt_;
    std::vector<PortalEvent> events;
    auto extend = [&events](std::vector<PortalEvent> more) {
        for (auto &e : more)
            events.push_back(std::move(e));
    };

    // A pose jump (teleport) ends the attempt and forgets the side history.
    if (previous && (current.position - previous->position).norm() > max_step_) {
        extend(finished(attempt, time_ns, previous->top));
        previous.reset();
        entry = 0;
        cross.reset();
        attempt.reset();
    }

    // Close to the gate centre: start an attempt if none is open.
    const bool near = (current.local_position - Vec3(offset_, 0, 0)).norm() <= radius_;
    if (near && !attempt) {
        attempt = Attempt{next_id, Vec3::Zero(), {}};
        ++next_id;
    }
    struct Passage {
        int source, target;
        Vec3 point;
        std::optional<bool> depth;
    };
    std::optional<Passage> passage;
    if (previous) {
        // Remember where the reference origin last crossed the plane.
        const double a = previous->local_position[0] - offset_, b = current.local_position[0] - offset_;
        if ((a > 0 && 0 >= b) || (a < 0 && 0 <= b))
            cross =
                Vec3(previous->local_position + (a / (a - b)) * (current.local_position - previous->local_position));

        // Which side the robot is now fully on (whole envelope, or just the origin), 0 while straddling.
        double xmin, xmax;
        if (full_envelope_) {
            xmin = std::numeric_limits<double>::infinity();
            xmax = -xmin;
            for (const auto &v : local) {
                xmin = std::min(xmin, v[0] - offset_);
                xmax = std::max(xmax, v[0] - offset_);
            }
        } else {
            xmin = xmax = current.local_position[0] - offset_;
        }
        const int side = xmin > 0 ? 1 : xmax < 0 ? -1 : 0;

        // Reached the other side: a passage if the fit checks pass at the crossing.
        if (side && entry != 0 && side != entry) {
            if (cross) {
                bool fits = true;
                if (floor_check_) {
                    double zmin = std::numeric_limits<double>::infinity();
                    for (const auto &v : world)
                        zmin = std::min(zmin, v[2]);
                    fits = zmin > floor_;
                }
                if (check_origin_) {
                    const double y = std::abs((*cross)[1]);
                    fits = fits && 0 < y && y < width_;
                }
                auto within = [&](const std::vector<Vec3> &vertices) {
                    double ymax = 0, zmax = -std::numeric_limits<double>::infinity();
                    for (const auto &v : vertices) {
                        ymax = std::max(ymax, std::abs(v[1]));
                        zmax = std::max(zmax, v[2]);
                    }
                    return ymax < width_ && zmax < top_;
                };
                if (check_crossing_orientation_)
                    fits = fits && within(transformed(vertices_, current.local_rotation, *cross));
                if (check_completion_)
                    fits = fits && within(local);
                if (fits) {
                    // Depth band: the envelope's vertical extent (bounding-box approximation) at the crossing.
                    std::optional<bool> depth;
                    if (band_) {
                        Vec3 maxabs = Vec3::Constant(0);
                        for (const auto &v : vertices_)
                            maxabs = maxabs.cwiseMax(v.cwiseAbs());
                        const Vec3 extent = current.local_rotation.cwiseAbs() * maxabs;
                        depth = (*cross)[2] + extent[2] >= band_->first && (*cross)[2] - extent[2] <= band_->second;
                    }
                    passage = Passage{entry, side, *cross, depth};
                }
            }
            cross.reset();
        }
        if (side)
            entry = side;
        if (near && attempt)
            attempt->turns += rotationDelta(previous->rotation, current.rotation);
    } else {
        entry = current.local_position[0] > offset_ ? 1 : -1;
    }

    // Report the passage and keep the latest one per direction in the attempt.
    if (passage) {
        PortalEvent event;
        event.kind = "pass_through";
        event.time_ns = time_ns;
        event.from_side = passage->source > 0 ? "positive" : "negative";
        event.to_side = passage->target > 0 ? "positive" : "negative";
        event.crossing_point_local = passage->point;
        event.envelope_top_world = current.top;
        event.rotation_vector_body = attempt ? attempt->turns : Vec3::Zero();
        event.attempt_id = attempt ? attempt->identifier : 0;
        event.depth_overlap = passage->depth;
        events.push_back(event);
        if (attempt) {
            auto &list = attempt->passages;
            auto found =
                std::find_if(list.begin(), list.end(), [&](const auto &item) { return item.first == event.from_side; });
            if (found != list.end())
                found->second = event;
            else
                list.emplace_back(event.from_side, event);
        }
    }

    // Leaving the approach radius ends the attempt.
    if (!near) {
        extend(finished(attempt, time_ns, current.top));
        attempt.reset();
    }
    return {std::move(events), entry, cross, std::move(attempt), next_id};
}

// ----------------------------------------------------------------------------------- panels
PerforatedPanel::PerforatedPanel(const Json &parameters, const Pose &world_from_task) {
    if (parameters.at("plane").at("axis") != "x")
        invalid("perforated panels currently require an X plane");
    world_from_task_ = ownedPose(world_from_task);
    task_from_world_ = spatial::inverse(world_from_task);
    offset_ = number(parameters.at("plane").at("offset_m"), "offset_m");
    half_ = number(parameters.at("half_size_m"), "half_size_m");
    const Json &clearance = parameters.at("projectile_clearance");
    if (clearance.at("rule") != "radius_over_axis_cosine")
        invalid("unsupported projectile clearance rule");
    min_cosine_ = number(clearance.at("min_cosine"), "min_cosine");
    if (half_ <= 0 || !(0 < min_cosine_ && min_cosine_ <= 1))
        invalid("invalid panel geometry");
    // Holes: uv in [0, 1] across the panel -> panel-centred y, z in metres.
    for (const auto &hole : parameters.at("holes")) {
        const Json &uv = hole.at("uv");
        if (!uv.is_array() || uv.size() != 2)
            invalid("invalid panel hole");
        const double radius = number(hole.at("radius_uv"), "radius_uv") * 2 * half_;
        Hole h{hole.at("id").get<std::string>(), hole.at("class").get<std::string>(),
               hole.at("size").get<std::string>(),
               Eigen::Vector2d((number(uv[0], "uv") - .5) * 2 * half_, (number(uv[1], "uv") - .5) * 2 * half_), radius};
        if (radius <= 0)
            invalid("invalid panel hole");
        holes_.push_back(std::move(h));
    }
}

Vec3 PerforatedPanel::worldPoint(const Vec3 &point_local) const {
    return spatial::apply(world_from_task_, finite(point_local, "point"));
}

double PerforatedPanel::releaseDistance(const Vec3 &tip_world) const {
    return std::abs(spatial::apply(task_from_world_, finite(tip_world, "tip"))[0] - offset_);
}

std::optional<PanelHit> PerforatedPanel::intersect(const Vec3 &start_world, const Vec3 &end_world,
                                                   const Vec3 &axis_world, double radius_m) const {
    const Vec3 start = spatial::apply(task_from_world_, finite(start_world, "start"));
    const Vec3 end = spatial::apply(task_from_world_, finite(end_world, "end"));
    const Vec3 axis = finite(axis_world, "axis");
    if (std::abs(axis.norm() - 1) > 1e-6)
        invalid("projectile axis must be unit length");
    if (!std::isfinite(radius_m) || radius_m <= 0)
        invalid("projectile radius must be positive and finite");

    // Where the segment crosses the panel plane (none when it starts on it).
    const double a = start[0] - offset_, b = end[0] - offset_;
    if (a * b > 0 || a == b || a == 0)
        return std::nullopt;
    const Vec3 hit = start + (end - start) * (-a / (b - a));
    if (std::max(std::abs(hit[1]), std::abs(hit[2])) > half_)
        return std::nullopt;

    // The projectile's footprint on the panel grows as 1 / cos(incidence), capped by min_cosine.
    const Vec3 local_axis = task_from_world_.rotation.normalized() * axis;
    const double clearance = radius_m / std::max(min_cosine_, std::abs(local_axis[0]));
    const Eigen::Vector2d yz(hit[1], hit[2]);
    for (const auto &hole : holes_)
        if ((yz - hole.center).norm() + clearance <= hole.radius)
            return PanelHit{"pass", hit, hole.id, hole.category, hole.size};
    return PanelHit{"blocked", hit, "", "", ""};
}

// ------------------------------------------------------------------------------------ crates
OpenCrate::OpenCrate(const Json &parameters, const Pose &world_from_crate) {
    class_ = parameters.at("class").get<std::string>();
    outer_ = number(parameters.at("outer_width_m"), "outer_width_m", true) / 2;
    const double inner = number(parameters.at("inner_width_m"), "inner_width_m", true);
    const double liner = number(parameters.at("liner_thickness_m"), "liner_thickness_m");
    inner_ = inner / 2 - liner;
    height_ = number(parameters.at("outer_height_m"), "outer_height_m", true) -
              number(parameters.at("base_thickness_m"), "base_thickness_m");
    if (inner_ <= 0 || inner_ > outer_ || height_ <= 0)
        invalid("invalid crate geometry");
    const Pose pose = ownedPose(world_from_crate);
    rotation_ = rotationMatrix(pose);
    origin_ = pose.translation;
}

CrateStep OpenCrate::step(const Vec3 &old_world, const Vec3 &new_world, const Vec3 &velocity_world,
                          const Vec3 &axis_world, double radius_m, double length_m, bool entered) const {
    // Work in the crate frame with the payload as an axis-aligned box: `extent` is its half size (a capsule of
    // radius_m and length_m along its axis).
    Vec3 new_position = new_world, velocity = velocity_world;
    const Vec3 a = rotation_.transpose() * (old_world - origin_);
    Vec3 b = rotation_.transpose() * (new_position - origin_);
    const Vec3 extent = Vec3::Constant(radius_m) +
                        std::max(0.0, length_m / 2 - radius_m) * (rotation_.transpose() * axis_world).cwiseAbs();
    const double inner = inner_, outer = outer_, height = height_;
    auto fitsInner = [&](const Vec3 &p) {
        return std::abs(p[0]) + extent[0] <= inner && std::abs(p[1]) + extent[1] <= inner;
    };

    // Coming down through the top plane within the inner opening enters the crate.
    const auto top = crossing(a, b, 2, height + extent[2]);
    if (top && b[2] < a[2] && fitsInner(*top))
        entered = true;

    // Walls (inner faces once entered, else outer faces): stop at the wall, keep 15 % of the velocity, and drop
    // its normal component.
    for (int axis = 0; axis < 2; ++axis)
        for (int sign : {-1, 1}) {
            const double surface = sign * (entered ? inner - extent[axis] : outer + extent[axis]);
            const auto hit = crossing(a, b, axis, surface);
            if (hit && -extent[2] < (*hit)[2] && (*hit)[2] < height + extent[2] &&
                std::abs((*hit)[1 - axis]) < outer + extent[1 - axis]) {
                b = *hit;
                velocity = velocity * 0.15;
                const Vec3 normal = rotation_.col(axis);
                velocity = velocity - normal * velocity.dot(normal);
                new_position = rotation_ * b + origin_;
            }
        }

    // Landing on the floor ends the flight: inside only if it entered and fits the inner opening.
    const auto floor = crossing(a, b, 2, extent[2]);
    if (floor && b[2] < a[2] && std::max(std::abs((*floor)[0]), std::abs((*floor)[1])) <= outer) {
        const bool inside = fitsInner(*floor);
        return {rotation_ * *floor + origin_, Vec3::Zero(), entered, entered && inside ? "inside" : "blocked", "floor"};
    }

    // Coming down onto the rim (crossing the top plane over the walls, not the opening) also ends it.
    if (top && b[2] < a[2] && !fitsInner(*top) && std::abs((*top)[0]) <= outer + extent[0] &&
        std::abs((*top)[1]) <= outer + extent[1])
        return {rotation_ * *top + origin_, Vec3::Zero(), entered, "blocked", "rim"};
    return {new_position, velocity, entered, "", ""};
}

// ------------------------------------------------------------------------------- proximity
ProximityTarget::ProximityTarget(const Json &parameters, const Pose &world_from_frame, const Vec3 &probe_reference) {
    distance_ = number(parameters.at("trigger_distance_m"), "trigger_distance_m", true);
    dwell_ns_ = number(parameters.at("dwell_s"), "dwell_s", true) * 1e9;
    Pose face;
    const auto &q = parameters.at("face_orientation_wxyz");
    face.rotation =
        Eigen::Quaterniond(q.at(0).get<double>(), q.at(1).get<double>(), q.at(2).get<double>(), q.at(3).get<double>());
    const Vec3 offset = vec3(parameters.at("sensor_offset_m"), "sensor_offset_m");
    face_world = spatial::compose(ownedPose(world_from_frame), ownedPose(face));
    sensor_ = spatial::apply(face_world, offset);
    probe_ = finite(probe_reference, "probe point");
    reset();
}

void ProximityTarget::reset() {
    latched = false;
    time_.reset();
    dwell_ = 0;
}

std::vector<Fact> ProximityTarget::observe(std::int64_t time_ns, const Pose &world_reference) {
    std::int64_t dt;
    checkSampleTime(time_, time_ns, dt, "proximity");
    if (latched)
        return {};
    const Vec3 tip = spatial::apply(ownedPose(world_reference), probe_);
    const double distance = (tip - sensor_).norm();
    if (distance > distance_) {
        dwell_ = 0;
        return {};
    }
    dwell_ += dt;
    if (static_cast<double>(dwell_) + 1e-3 >= dwell_ns_) {
        latched = true;
        return {Fact{"activate", {{"distance_m", (tip - sensor_).norm()}}}};
    }
    return {};
}

// --------------------------------------------------------------------------------- surface
SurfaceTracker::SurfaceTracker(const Json &parameters, const Pose &world_from_frame, const std::vector<Vec3> &envelope,
                               double surface_z, const std::vector<std::pair<std::string, Vec3>> &targets)
    : targets_(targets) {
    if (parameters.at("shape") != "regular_octagon")
        invalid("surface regions currently support only regular_octagon");
    apothem_ =
        number(parameters.at("apothem_m"), "apothem_m", true) - number(parameters.at("pipe_radius_m"), "pipe_radius_m");
    margin_ = number(parameters.at("breach_margin_m"), "breach_margin_m");
    dwell_ns_ = number(parameters.at("dwell_s"), "dwell_s", true) * 1e9;
    max_step_ = number(parameters.at("max_pose_step_m"), "max_pose_step_m", true);
    const Json &facing = parameters.at("facing");
    tolerance_ = number(facing.at("tolerance_deg"), "tolerance_deg") * kDegToRad;
    facing_ns_ = number(facing.at("dwell_s"), "facing dwell_s", true) * 1e9;
    surface_ = number(surface_z, "surface_z");
    vertices_ = checkedVertices(envelope, "surface", false);
    frame_from_world_ = spatial::inverse(ownedPose(world_from_frame));
    frame_rotation_ = rotationMatrix(frame_from_world_);
    reset();
}

void SurfaceTracker::reset() {
    time_.reset();
    previous_.reset();
    clear();
    breached_ = false;
}

void SurfaceTracker::clear() {
    submerged_ = false;
    dwell_ = 0;
    facing_dwell_ = 0;
    facing_.reset();
    surfaced_ = false;
    achieved_.reset();
}

std::vector<Fact> SurfaceTracker::observe(std::int64_t time_ns, const Pose &world_reference) {
    std::int64_t dt;
    checkSampleTime(time_, time_ns, dt, "surface");
    if (breached_)
        return {};
    const Pose pose = ownedPose(world_reference);
    const Vec3 position = pose.translation;
    const Mat3 rotation = rotationMatrix(pose);
    if (previous_ && (position - *previous_).norm() > max_step_)
        clear();
    previous_ = position;
    std::vector<Fact> facts;

    // Envelope top against the surface (with breach_margin_m hysteresis), and whether every envelope vertex is
    // inside the octagon (8 half-planes at the shrunken apothem).
    double top = -std::numeric_limits<double>::infinity();
    for (const auto &v : vertices_)
        top = std::max(top, (rotation * v + position)[2]);
    if (top < surface_ - margin_)
        submerged_ = true;
    const Mat3 local_rotation = frame_rotation_ * rotation;
    const Vec3 local_position = spatial::apply(frame_from_world_, position);
    bool inside = true;
    for (const auto &v : vertices_) {
        const Vec3 p = local_rotation * v + local_position;
        for (int i = 0; i < 8; ++i) {
            const double angle = i * kPi / 4;
            if (!(p[0] * std::cos(angle) + p[1] * std::sin(angle) <= apothem_))
                inside = false;
        }
    }

    if (submerged_ && top > surface_ + margin_ && !inside) {
        breached_ = true;
        return {Fact{"breach", {{"envelope_top_world", top}}}};
    }
    if (submerged_ && inside && top >= surface_) {
        dwell_ += dt;
        if (static_cast<double>(dwell_) >= dwell_ns_) {
            if (!surfaced_) {
                surfaced_ = true;
                facts.push_back(Fact{"surface_reached", {{"envelope_top_world", top}}});
            }
            for (auto &f : facingStep(position, rotation, dt))
                facts.push_back(std::move(f));
        }
        return facts;
    }

    // Not surfaced in the octagon: reset the dwell and report anything lost.
    dwell_ = 0;
    facing_dwell_ = 0;
    facing_.reset();
    if (surfaced_) {
        surfaced_ = false;
        facts.push_back(Fact{"surface_lost", Json::object()});
    }
    if (achieved_) {
        achieved_.reset();
        facts.push_back(Fact{"facing_lost", Json::object()});
    }
    return facts;
}

std::vector<Fact> SurfaceTracker::facingStep(const Vec3 &position, const Mat3 &rotation, std::int64_t dt) {
    const Eigen::Vector2d heading(rotation(0, 0), rotation(1, 0));

    // Nearest target by bearing angle (ties broken by target name).
    std::pair<double, std::string> best{std::numeric_limits<double>::infinity(), ""};
    bool first = true;
    for (const auto &target : targets_) {
        const Eigen::Vector2d direction = target.second.head<2>() - position.head<2>();
        const double denominator = direction.norm() * heading.norm();
        const double angle =
            denominator > 1e-9 ? std::acos(std::clamp(direction.dot(heading) / denominator, -1.0, 1.0)) : kPi;
        const std::pair<double, std::string> candidate{angle, target.first};
        if (first || candidate < best)
            best = candidate;
        first = false;
    }
    const double angle = best.first;
    const std::string &facing = best.second;

    // Dwell on the same target within tolerance.
    std::optional<std::string> achieved;
    if (angle <= tolerance_) {
        facing_dwell_ = (facing_ && *facing_ == facing) ? facing_dwell_ + static_cast<double>(dt) : 0.0;
        facing_ = facing;
        if (facing_dwell_ >= facing_ns_)
            achieved = facing;
    } else {
        facing_dwell_ = 0.0;
        facing_.reset();
    }

    std::vector<Fact> facts;
    if (achieved != achieved_) {
        if (achieved_)
            facts.push_back(Fact{"facing_lost", Json::object()});
        if (achieved)
            facts.push_back(Fact{"facing_reached", {{"target", *achieved}, {"angle_rad", angle}}});
        achieved_ = achieved;
    }
    return facts;
}

// -------------------------------------------------------------------------------------- turn
TurnTracker::TurnTracker(const Json &parameters, const Pose &world_from_frame) {
    radius_ = number(parameters.at("radius_m"), "radius_m", true);
    max_step_ = number(parameters.at("max_pose_step_m"), "max_pose_step_m", true);
    tolerance_ = number(parameters.at("turn_tolerance_deg"), "turn tolerance") * kDegToRad;
    settle_ = number(parameters.at("settle_deg"), "settle_deg") * kDegToRad;
    reversal_ = number(parameters.at("reversal_deg"), "reversal_deg") * kDegToRad;
    minimum_ = number(parameters.at("min_travel_deg"), "min_travel_deg") * kDegToRad;
    dwell_ns_ = number(parameters.at("dwell_s"), "dwell_s", true) * 1e9;
    frame_from_world_ = spatial::inverse(ownedPose(world_from_frame));
    for (const auto &item : parameters.at("restart_events"))
        restart_events.emplace_back(item.at("task").get<std::string>(), item.at("id").get<std::string>());
    reset();
}

// Forget everything (run reset); clear() only restarts the current count.
void TurnTracker::reset() {
    time_.reset();
    previous_.reset();
    previous_yaw_.reset();
    tracking_ = restart_ = false;
    clear();
}

void TurnTracker::clear() {
    yaw_ = settle_yaw_ = start_ = peak_ = 0.0;
    dwell_ = 0;
    judged_ = false;
}

void TurnTracker::restart() {
    restart_ = tracking_;
}

std::vector<Fact> TurnTracker::judge(const std::string &reason) const {
    const double travel = std::abs(peak_ - start_);
    if (travel < minimum_)
        return {};
    // Whole turns, allowing turn_tolerance_deg short of each full turn.
    const int turns = static_cast<int>(floorDivide(travel + tolerance_, 2 * kPi));
    return {Fact{"rotation_judged", {{"reason", reason}, {"travel_rad", travel}, {"turns", turns}}}};
}

std::vector<Fact> TurnTracker::observe(std::int64_t time_ns, const Pose &world_reference) {
    std::int64_t dt;
    checkSampleTime(time_, time_ns, dt, "turn");
    const Pose pose = ownedPose(world_reference);
    const Vec3 position = pose.translation;
    const Mat3 rotation = rotationMatrix(pose);

    // A pose jump (teleport) drops the count.
    if (previous_ && (position - *previous_).norm() > max_step_) {
        clear();
        tracking_ = restart_ = false;
        previous_yaw_.reset();
    }
    previous_ = position;
    const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    std::vector<Fact> facts;
    auto extend = [&facts](std::vector<Fact> more) {
        for (auto &f : more)
            facts.push_back(std::move(f));
    };

    // Near the frame: accumulate unwrapped yaw, track the peak, judge on a reversal, and judge once per settle.
    const bool near = spatial::apply(frame_from_world_, position).norm() <= radius_;
    if (near) {
        if (!tracking_ || restart_) {
            clear();
            restart_ = false;
        } else if (previous_yaw_) {
            yaw_ += wrapAngle(yaw - *previous_yaw_);
            if (std::abs(yaw_ - start_) > std::abs(peak_ - start_)) {
                peak_ = yaw_;
            } else if (std::abs(peak_ - yaw_) > reversal_) {
                extend(judge("reversed"));
                start_ = peak_;
                peak_ = yaw_;
            }
            if (std::abs(yaw_ - settle_yaw_) <= settle_) {
                dwell_ += dt;
            } else {
                settle_yaw_ = yaw_;
                dwell_ = 0;
                judged_ = false;
            }
        }
        tracking_ = true;
        if (static_cast<double>(dwell_) >= dwell_ns_ && !judged_) {
            judged_ = true;
            extend(judge("stopped"));
        }
    } else if (tracking_) {
        extend(judge("left"));
        clear();
        tracking_ = restart_ = false;
    }
    previous_yaw_ = yaw;
    return facts;
}
} // namespace nereus::session::tasks

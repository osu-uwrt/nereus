// Port of python/src/nereus/prop_world.py: same algorithm, constants and event contract,
// on Bullet's C++ API. PyBullet wraps btMultiBodyDynamicsWorld with base-only btMultiBody bodies
// (createMultiBody), so this does the same instead of using btRigidBody: identical integration,
// damping, contact and constraint code paths. Frame/quaternion helpers mirror the Python ones.
#include <robotics/session/prop_world.hpp>

#include "obj_mesh.hpp"

#include <BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h>
#include <BulletCollision/CollisionDispatch/btManifoldResult.h>
#include <BulletDynamics/Featherstone/btMultiBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyFixedConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <btBulletCollisionCommon.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>
#include <set>
#include <stdexcept>

namespace robotics::session {
namespace {
using Matrix4 = Eigen::Matrix4d;
using Vec3 = Eigen::Vector3d;

// Contact heuristics of the original claw_world.py (Bullet tuning priors, not pack data).
constexpr double kPadSpinningFriction = 0.01;
constexpr double kPadMarginM = 0.0005;
constexpr double kUrdfDefaultMarginM = 0.001; // pybullet's importer default collision margin
constexpr double kStallNormalSpeed = -0.2;
constexpr double kStallPenetrationM = -0.0008;
constexpr double kGraspNormalForceN = 0.01;
constexpr double kGraspNormalAlignment = 0.6;
constexpr double kDirectionEpsilonM = 1e-9;

Matrix4 matrixFrom(const Vec3 &position, double w, double x, double y, double z) {
    const double n = std::sqrt(w * w + x * x + y * y + z * z);
    w /= n, x /= n, y /= n, z /= n;
    Matrix4 t = Matrix4::Identity();
    t.block<3, 3>(0, 0) << 1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
        2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 2 * (x * z - y * w),
        2 * (y * z + x * w), 1 - 2 * (x * x + y * y);
    t.block<3, 1>(0, 3) = position;
    return t;
}
Matrix4 matrixFrom(const Vec3 &position, const Eigen::Quaterniond &q) {
    return matrixFrom(position, q.w(), q.x(), q.y(), q.z());
}

// Quaternion (x, y, z, w) stable at 180 degrees: largest component first.
std::array<double, 4> xyzw(const Matrix4 &m) {
    const auto r = m.block<3, 3>(0, 0);
    const double trace = r.trace();
    const double candidates[4] = {1 + trace, 1 + 2 * r(0, 0) - trace, 1 + 2 * r(1, 1) - trace,
                                  1 + 2 * r(2, 2) - trace};
    int i = 0;
    for (int k = 1; k < 4; ++k)
        if (candidates[k] > candidates[i])
            i = k;
    const double s = 2 * std::sqrt(std::max(candidates[i], 0.0));
    std::array<double, 4> q{};
    if (i == 0) {
        q = {(r(2, 1) - r(1, 2)) / s, (r(0, 2) - r(2, 0)) / s, (r(1, 0) - r(0, 1)) / s, s / 4};
    } else {
        const int a = i - 1, b = (a + 1) % 3, c = (a + 2) % 3;
        q[static_cast<std::size_t>(a)] = s / 4;
        q[static_cast<std::size_t>(b)] = (r(b, a) + r(a, b)) / s;
        q[static_cast<std::size_t>(c)] = (r(c, a) + r(a, c)) / s;
        q[3] = (r(c, b) - r(b, c)) / s;
    }
    const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (double &v : q)
        v /= norm;
    return q;
}
Matrix4 fromXyzw(const Vec3 &position, const btQuaternion &q) {
    return matrixFrom(position, q.w(), q.x(), q.y(), q.z());
}
Matrix4 poseMatrix(const spatial::Pose &pose) { return matrixFrom(pose.translation, pose.rotation); }

btVector3 toBt(const Vec3 &v) { return btVector3(v.x(), v.y(), v.z()); }
Vec3 fromBt(const btVector3 &v) { return Vec3(v.x(), v.y(), v.z()); }
btQuaternion quatOf(const Matrix4 &m) {
    const auto q = xyzw(m);
    return btQuaternion(q[0], q[1], q[2], q[3]);
}
btTransform transformOf(const Matrix4 &m) { return btTransform(quatOf(m), toBt(m.block<3, 1>(0, 3))); }

Vec3 vec3(const Json &j, const char *name) {
    if (!j.is_array() || j.size() != 3)
        throw std::invalid_argument(std::string(name) + " must contain 3 values");
    return Vec3(j[0].get<double>(), j[1].get<double>(), j[2].get<double>());
}
Matrix4 poseOf(const Json &position, const Json &wxyz) {
    return matrixFrom(vec3(position, "position_m"), wxyz.at(0).get<double>(), wxyz.at(1).get<double>(),
                      wxyz.at(2).get<double>(), wxyz.at(3).get<double>());
}
Matrix4 yawMatrix(const Json &placement) {
    const double half = placement.at("yaw_deg").get<double>() * M_PI / 180.0 / 2;
    return matrixFrom(vec3(placement.at("position_m"), "position_m"), std::cos(half), 0, 0,
                      std::sin(half));
}
bool allFinite(const Vec3 &v) { return v.allFinite(); }
double sign(double v) { return v > 0 ? 1.0 : (v < 0 ? -1.0 : 0.0); }

struct Contact {
    int other{-1};  // body index of the other object
    Vec3 normal;    // normal on the other body (points towards the queried body)
    double distance{0}, force{0};
};

// Records closest points the way pybullet's getClosestPoints does (normal on B as reported by
// the algorithm), in algorithm order.
struct ClosestPoints : btManifoldResult {
    std::vector<Contact> points;
    ClosestPoints(const btCollisionObjectWrapper *a, const btCollisionObjectWrapper *b)
        : btManifoldResult(a, b) {}
    void addContactPoint(const btVector3 &normalOnB, const btVector3 &point, btScalar depth) override {
        Contact c;
        c.normal = fromBt(normalOnB);
        c.distance = depth;
        points.push_back(c);
        btManifoldResult::addContactPoint(normalOnB, point, depth);
    }
};
// Pair exclusions (pybullet setCollisionFilterPair). btMultiBodyLinkCollider overrides
// checkCollideWithOverride without consulting setIgnoreCollisionCheck, so the ignore list alone never
// stops multibody pairs; this filter keeps excluded pairs out of the broadphase instead.
struct PairFilter : btOverlapFilterCallback {
    std::set<std::pair<const void *, const void *>> excluded;
    static std::pair<const void *, const void *> key(const void *a, const void *b) {
        return a < b ? std::make_pair(a, b) : std::make_pair(b, a);
    }
    bool needBroadphaseCollision(btBroadphaseProxy *a, btBroadphaseProxy *b) const override {
        return (a->m_collisionFilterGroup & b->m_collisionFilterMask) != 0 &&
               (b->m_collisionFilterGroup & a->m_collisionFilterMask) != 0 &&
               !excluded.count(key(a->m_clientObject, b->m_clientObject));
    }
};

// Robot-side collision world (port of c_simulator task_contacts.cpp). Robot shapes (pads, a held
// prop) are posed COM-locally and follow the plant state; scenery is fixed in the world; free props
// sit where the prop world left them. A free prop pushes back only while resting on scenery and
// only when the robot presses down onto it (side pushes stay in the prop world). Resolution: move
// out along the deepest contact, then sequential normal impulses with Coulomb friction (<= 8 passes).
class VehicleContacts final : public simulation::ContactResolver {
  public:
    enum Kind { kRobot = 1, kScenery = 2, kProp = 4 };
    explicit VehicleContacts(double friction) : friction_(friction) {
        if (!std::isfinite(friction) || friction < 0)
            throw std::invalid_argument("contact friction must be finite and nonnegative");
    }
    ~VehicleContacts() override {
        for (auto &e : entries_)
            world_.removeCollisionObject(&e->object);
    }
    int add(std::unique_ptr<btCollisionShape> shape, std::unique_ptr<btTriangleMesh> mesh, Kind kind,
            const Matrix4 &pose) {
        auto e = std::make_unique<Entry>();
        e->kind = kind;
        e->pose = pose;
        e->shape = std::move(shape);
        e->mesh = std::move(mesh);
        e->shape->setMargin(kPadMarginM);
        e->object.setCollisionShape(e->shape.get());
        e->object.setWorldTransform(transformOf(pose));
        e->object.setUserPointer(e.get());
        e->object.setCollisionFlags(kind == kScenery ? btCollisionObject::CF_STATIC_OBJECT
                                                     : btCollisionObject::CF_KINEMATIC_OBJECT);
        // Only robot pairs are detected every pass; prop support is queried when the robot touches a prop.
        world_.addCollisionObject(&e->object, kind, kind == kRobot ? kScenery | kProp : kRobot);
        entries_.push_back(std::move(e));
        return static_cast<int>(entries_.size()) - 1;
    }
    // Robot shape: COM-local pose. Prop: world pose, or COM-local when attached to the robot.
    void pose(int index, const Matrix4 &pose, bool attached = false) {
        auto &e = *entries_.at(static_cast<std::size_t>(index));
        e.pose = pose;
        e.attached = attached;
    }
    State resolve(State state, const Matrix6 &inverse_mass) override {
        Eigen::Quaterniond q(state[3], state[4], state[5], state[6]);
        q.normalize();
        for (int pass = 0; pass < 8; ++pass) {
            const auto contacts = detect(state, q);
            if (contacts.empty())
                break;
            const auto deepest = std::max_element(contacts.begin(), contacts.end(),
                                                  [](const Hit &a, const Hit &b) { return a.depth < b.depth; });
            bool changed = deepest->depth > 1e-5;
            if (changed)
                state.head<3>() += deepest->normal * (deepest->depth + .00005);
            for (const auto &c : contacts) {
                const Vec3 n = q.conjugate() * c.normal, r = q.conjugate() * (c.point - Vec3(state.head<3>()));
                Eigen::Matrix<double, 6, 1> j;
                j << n, r.cross(n);
                const double speed = j.dot(state.segment<6>(7)), den = j.dot(inverse_mass * j);
                if (speed >= -1e-5 || den < 1e-12)
                    continue;
                changed = true;
                const double impulse = -speed / den;
                state.segment<6>(7) += inverse_mass * j * impulse;
                const Vec3 v = Vec3(state.segment<3>(7)) + Vec3(state.segment<3>(10)).cross(r);
                Vec3 tangent = v - n * v.dot(n);
                if (tangent.norm() > 1e-9) {
                    tangent.normalize();
                    Eigen::Matrix<double, 6, 1> jt;
                    jt << tangent, r.cross(tangent);
                    const double d = jt.dot(inverse_mass * jt);
                    if (d > 1e-12)
                        state.segment<6>(7) -=
                            inverse_mass * jt * std::min(friction_ * impulse, jt.dot(state.segment<6>(7)) / d);
                }
            }
            if (!changed)
                break;
        }
        return state;
    }

  private:
    struct Entry {
        Kind kind{kScenery};
        bool attached{false};
        Matrix4 pose{Matrix4::Identity()};
        std::unique_ptr<btCollisionShape> shape;
        std::unique_ptr<btTriangleMesh> mesh;
        btCollisionObject object;
        bool robot() const { return kind == kRobot || attached; }
    };
    struct Hit {
        Vec3 point, normal; // world; normal pushes the robot out
        double depth;
    };
    std::vector<Hit> detect(const State &state, const Eigen::Quaterniond &q) {
        const Matrix4 body = matrixFrom(state.head<3>(), q);
        for (auto &e : entries_) {
            if (e->kind == kScenery)
                continue;
            e->object.setWorldTransform(transformOf(e->robot() ? Matrix4(body * e->pose) : e->pose));
            world_.updateSingleAabb(&e->object);
        }
        world_.performDiscreteCollisionDetection();
        const auto entry = [](const btCollisionObject *o) { return static_cast<Entry *>(o->getUserPointer()); };
        std::vector<Hit> out;
        for (int i = 0; i < dispatcher_.getNumManifolds(); ++i) {
            const auto *m = dispatcher_.getManifoldByIndexInternal(i);
            const auto *a = entry(m->getBody0()), *b = entry(m->getBody1());
            if (a->robot() == b->robot())
                continue;
            const auto *obstacle = a->robot() ? b : a;
            for (int k = 0; k < m->getNumContacts(); ++k) {
                const auto &p = m->getContactPoint(k);
                if (p.getDistance() > .0005)
                    continue;
                const Vec3 normal = fromBt(p.m_normalWorldOnB) * (a->robot() ? 1.0 : -1.0);
                if (obstacle->kind == kProp && (normal.z() < .7 || !supported(*obstacle)))
                    continue;
                out.push_back({fromBt(a->robot() ? p.getPositionWorldOnA() : p.getPositionWorldOnB()), normal,
                               std::max(0.0, -double(p.getDistance()))});
            }
        }
        return out;
    }
    // A free prop resting on scenery (a contact within 3 mm whose normal points up at the prop).
    bool supported(const Entry &prop) {
        struct Support : btCollisionWorld::ContactResultCallback {
            const btCollisionObject *prop{nullptr};
            bool up{false};
            btScalar addSingleResult(btManifoldPoint &p, const btCollisionObjectWrapper *a, int, int,
                                     const btCollisionObjectWrapper *, int, int) override {
                const double z = p.m_normalWorldOnB.z();
                up = up || (p.getDistance() <= .003 && (a->getCollisionObject() == prop ? z > .5 : z < -.5));
                return 0;
            }
        } support;
        support.prop = &prop.object;
        support.m_closestDistanceThreshold = .003;
        for (auto &e : entries_)
            if (e->kind == kScenery && !support.up)
                world_.contactPairTest(const_cast<btCollisionObject *>(&prop.object), &e->object, support);
        return support.up;
    }
    double friction_;
    btDefaultCollisionConfiguration configuration_;
    btCollisionDispatcher dispatcher_{&configuration_};
    btDbvtBroadphase broadphase_;
    btCollisionWorld world_{&dispatcher_, &broadphase_, &configuration_};
    std::vector<std::unique_ptr<Entry>> entries_;
};
} // namespace

struct PropWorld::Impl {
    struct Body {
        std::unique_ptr<btCollisionShape> shape;
        std::unique_ptr<btTriangleMesh> mesh;
        std::unique_ptr<btMultiBody> body;
        std::unique_ptr<btMultiBodyLinkCollider> collider;
        bool fixed{true};
    };
    struct Prop {
        std::string id;
        int body{-1};
        Vec3 center{Vec3::Zero()}, half{Vec3::Zero()};
        const Json *config{nullptr};
        double settled{0};
        bool scored{false}, picked{false};
    };
    struct Basket {
        std::string id;
        Matrix4 region_from_world;
        const Json *region;
    };

    // Owning references into the resolved scenario copy.
    Json definition;
    std::string task, mechanism_id;
    Json settings, claw;
    std::vector<Json> statics, rigid, baskets;
    std::map<std::string, Json> events;
    Matrix4 world_from_task;
    std::map<std::string, Matrix4> frames;
    std::filesystem::path pad_paths[2];
    std::map<std::string, std::filesystem::path> task_assets;
    Matrix4 mount{Matrix4::Identity()};
    double travel{0}, initial_q{0}, surface_z{0};
    struct PoolBox {
        Matrix4 transform;
        Vec3 half;
        bool floor;
    };
    std::vector<PoolBox> pool_boxes;

    // Bullet world (rebuilt by reset(), like the Python client reconnect).
    std::unique_ptr<btDefaultCollisionConfiguration> configuration;
    std::unique_ptr<btCollisionDispatcher> dispatcher;
    std::unique_ptr<btHashedOverlappingPairCache> pair_cache;
    PairFilter pair_filter;
    std::unique_ptr<btDbvtBroadphase> broadphase;
    std::unique_ptr<btMultiBodyConstraintSolver> solver;
    std::unique_ptr<btMultiBodyDynamicsWorld> world;
    std::vector<Body> bodies;
    std::vector<int> scenery, floors, pads;
    std::map<std::string, int> static_meshes;
    std::vector<Prop> props;
    int claw_body{-1};
    std::vector<Basket> basket;

    std::optional<std::size_t> held;
    std::unique_ptr<btMultiBodyFixedConstraint> constraint;
    Matrix4 held_relative{Matrix4::Identity()};
    double q{0};
    int direction{0};
    bool lockout{false};
    double last_command{0};
    std::map<std::string, double> contact_time;
    Events pending;
    std::optional<std::int64_t> time_ns;
    double dt{1.0 / 240};

    // Robot-side contacts (created on demand, survive reset()); entry indices of pads and props.
    std::shared_ptr<VehicleContacts> vehicle;
    std::vector<int> vehicle_pads, vehicle_props;
    void buildVehicle(double friction);
    void syncVehicle();

    Impl(const ResolvedScenario &resolved, const std::string &task_id, const std::string &mechanism);
    ~Impl() { teardown(); }

    void build();
    void teardown();
    void resetState();
    void restore(Prop &prop);

    int addBody(std::unique_ptr<btCollisionShape> shape, std::unique_ptr<btTriangleMesh> mesh,
                double mass, const Matrix4 &pose);
    void resetBase(int index, const Vec3 &position, const btQuaternion &orientation);
    void setVelocity(int index, const Vec3 &linear, const Vec3 &angular);
    Matrix4 basePose(int index) const;
    std::pair<Vec3, Vec3> baseVelocity(int index) const;
    void changeDynamics(int index, double friction, double restitution);

    std::vector<Contact> contactPoints(int a, int b = -1) const;
    std::vector<Contact> closestPoints(int a, int b) const;
    void setCollisionPair(int a, int b, bool enabled);

    Matrix4 propPose(const Prop &prop) const;
    std::optional<std::string> destination(std::size_t key, std::vector<std::size_t> resting = {}) const;
    void placePads(const Matrix4 &mount_pose, double q_value, const Vec3 &velocity,
                   const Vec3 &angular, double dq);
    void driveJaws(double dt_s, const Matrix4 &mount_pose, const Vec3 &velocity, const Vec3 &angular,
                   double target);
    Event event(const std::string &key, std::int64_t t, const Json &region, Json data) const;
    void release(const std::string &reason, std::int64_t t);
    void inferDirection(double command, bool enabled);
    void tryGrasp(double dt_s, const Matrix4 &mount_pose, std::int64_t t);
    void settle(double dt_s, std::int64_t t);
};

PropWorld::Impl::Impl(const ResolvedScenario &resolved, const std::string &task_id,
                      const std::string &mechanism) {
    const Json *found = nullptr;
    for (const auto &item : resolved.task_definitions)
        if (item.at("id") == task_id)
            found = &item;
    if (!found)
        throw std::invalid_argument("unknown task '" + task_id + "'");
    task = task_id;
    definition = *found;
    const Json *placement = nullptr;
    for (const auto &item : resolved.scenario.at("task_placements"))
        if (item.at("task") == task_id)
            placement = &item;
    if (!placement)
        throw std::invalid_argument("no placement for task '" + task_id + "'");
    world_from_task = yawMatrix(*placement);
    frames["task"] = world_from_task;
    for (const auto &frame : definition.at("frames"))
        frames[frame.at("id").get<std::string>()] =
            world_from_task * poseOf(frame.at("position_m"), frame.at("orientation_wxyz"));

    std::vector<Json> worlds;
    for (const auto &prop : definition.at("props")) {
        const auto type = prop.at("type").get<std::string>();
        if (type == "contact_world")
            worlds.push_back(prop);
        else if (type == "static_body")
            statics.push_back(prop);
        else if (type == "rigid_body")
            rigid.push_back(prop);
    }
    if (worlds.size() != 1 || rigid.empty())
        throw std::invalid_argument("task needs one contact_world prop and at least one rigid_body");
    settings = worlds[0].at("parameters");
    for (const auto &region : definition.at("regions"))
        if (region.at("type") == "box")
            baskets.push_back(region);
    std::map<std::string, Json> all_events;
    for (const auto &item : definition.at("events")) {
        std::string key = item.at("type").get<std::string>();
        if (key == "drop_into")
            key = "drop_" + item.at("parameters").at("outcome").get<std::string>();
        all_events.emplace(key, item);
    }
    for (const char *key : {"attach", "detach", "drop_in_region", "drop_elsewhere"}) {
        const auto it = all_events.find(key);
        if (it == all_events.end())
            throw std::invalid_argument(std::string("task lacks an event needed by the prop world: ") +
                                        key);
        events[key] = it->second;
    }

    for (const auto &[id, path] : resolved.asset_paths.count("tasks") ? resolved.asset_paths.at("tasks")
                                                                       : std::map<std::string, std::filesystem::path>{})
        task_assets[id] = path;
    std::vector<const Json *> claws;
    for (const auto &item : resolved.robot.at("mechanisms"))
        if (item.at("type") == settings.at("claw_mechanism_type") &&
            (mechanism.empty() || item.at("id") == mechanism))
            claws.push_back(&item);
    if (claws.size() != 1)
        throw std::invalid_argument("expected exactly one matching claw mechanism");
    mechanism_id = claws[0]->at("id").get<std::string>();
    claw = claws[0]->at("parameters");
    for (const char *key : {"max_gap_m", "min_gap_m", "jaw_speed_m_s", "hold_force_n", "friction",
                            "contact_margin_m", "grasp_dwell_s", "slip_distance_m"}) {
        const double value = claw.at(key).get<double>();
        if (!std::isfinite(value) || value <= 0)
            throw std::invalid_argument(std::string("Invalid claw ") + key);
    }
    if (claw.at("max_gap_m").get<double>() <= claw.at("min_gap_m").get<double>())
        throw std::invalid_argument("Invalid claw travel");
    travel = (claw.at("max_gap_m").get<double>() - claw.at("min_gap_m").get<double>()) / 2;
    initial_q = claw.at("initial_state") == "open" ? travel : 0.0;
    int side = 0;
    for (const char *name : {"left", "right"}) {
        const auto id = claw.at("pads").at(name).get<std::string>();
        try {
            pad_paths[side++] = resolved.asset("robot", id);
        } catch (const std::out_of_range &) {
            throw std::invalid_argument("asset '" + id + "' is not present in its pack");
        }
    }
    std::vector<spatial::FixedFrame> edges;
    for (const auto &entry : resolved.robot.at("frames").at("transforms")) {
        spatial::FixedFrame edge;
        edge.parent = entry.at("parent").get<std::string>();
        edge.child = entry.at("child").get<std::string>();
        edge.pose.translation = vec3(entry.at("position_m"), "position_m");
        const auto &o = entry.at("orientation_wxyz");
        edge.pose.rotation = Eigen::Quaterniond(o[0].get<double>(), o[1].get<double>(),
                                                o[2].get<double>(), o[3].get<double>());
        edges.push_back(edge);
    }
    const spatial::FixedFrames fixed(resolved.robot.at("frames").at("root").get<std::string>(), edges);
    mount = poseMatrix(fixed.fromRoot(claws[0]->at("frame").get<std::string>()));

    const Json &pool = resolved.pool.at("parameters");
    const Json &pool_placement = resolved.scenario.at("pool_placement");
    const double water_level = pool.at("water_level_m").get<double>();
    const double depth = pool.at("depth_m").get<double>();
    surface_z = water_level + pool_placement.at("position_m").at(2).get<double>();
    const Matrix4 pool_from_world = yawMatrix(pool_placement);
    for (const auto &b : resolved.pool.at("collision_boxes")) {
        const Vec3 center = vec3(b.at("center_m"), "center_m");
        const Vec3 size = vec3(b.at("size_m"), "size_m");
        pool_boxes.push_back(
            {pool_from_world * poseOf(b.at("center_m"), b.at("orientation_wxyz")), size / 2,
             std::abs(center.z() + size.z() / 2 - (water_level - depth)) < 1e-3});
    }
    build();
}

void PropWorld::Impl::teardown() {
    if (constraint && world)
        world->removeMultiBodyConstraint(constraint.get());
    constraint.reset();
    if (world) {
        for (auto &b : bodies) {
            if (b.collider)
                world->removeCollisionObject(b.collider.get());
            if (b.body)
                world->removeMultiBody(b.body.get());
        }
    }
    bodies.clear();
    world.reset();
    solver.reset();
    broadphase.reset();
    pair_cache.reset();
    dispatcher.reset();
    configuration.reset();
}

int PropWorld::Impl::addBody(std::unique_ptr<btCollisionShape> shape,
                             std::unique_ptr<btTriangleMesh> mesh, double mass, const Matrix4 &pose) {
    Body b;
    b.fixed = mass <= 0;
    btVector3 inertia(0, 0, 0);
    if (shape && mass > 0)
        shape->calculateLocalInertia(mass, inertia);
    b.body = std::make_unique<btMultiBody>(0, mass, inertia, b.fixed, false);
    b.body->setBaseWorldTransform(transformOf(pose));
    b.body->setHasSelfCollision(false);
    b.body->finalizeMultiDof();
    const int index = static_cast<int>(bodies.size());
    if (shape) {
        b.shape = std::move(shape);
        b.mesh = std::move(mesh);
        b.collider = std::make_unique<btMultiBodyLinkCollider>(b.body.get(), -1);
        b.collider->setCollisionShape(b.shape.get());
        b.collider->setWorldTransform(transformOf(pose));
        b.collider->setFriction(0.5);
        b.collider->setRestitution(0);
        b.collider->setUserIndex(index);
        if (b.fixed)
            b.collider->setCollisionFlags(b.collider->getCollisionFlags() |
                                          btCollisionObject::CF_STATIC_OBJECT);
        b.body->setBaseCollider(b.collider.get());
    }
    world->addMultiBody(b.body.get());
    if (b.collider) {
        const int group = b.fixed ? int(btBroadphaseProxy::StaticFilter) : int(btBroadphaseProxy::DefaultFilter);
        const int mask = b.fixed ? int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter)
                                 : int(btBroadphaseProxy::AllFilter);
        world->addCollisionObject(b.collider.get(), group, mask);
    }
    bodies.push_back(std::move(b));
    return index;
}

void PropWorld::Impl::changeDynamics(int index, double friction, double restitution) {
    auto &c = *bodies[static_cast<std::size_t>(index)].collider;
    c.setFriction(friction);
    c.setRestitution(restitution);
}

void PropWorld::Impl::resetBase(int index, const Vec3 &position, const btQuaternion &orientation) {
    auto &b = bodies[static_cast<std::size_t>(index)];
    const btTransform tr(orientation, toBt(position));
    b.body->setBaseWorldTransform(tr);
    if (b.collider)
        b.collider->setWorldTransform(tr);
}

void PropWorld::Impl::setVelocity(int index, const Vec3 &linear, const Vec3 &angular) {
    auto &b = bodies[static_cast<std::size_t>(index)];
    b.body->setBaseVel(toBt(linear));
    b.body->setBaseOmega(toBt(angular));
}

Matrix4 PropWorld::Impl::basePose(int index) const {
    const btTransform tr = bodies[static_cast<std::size_t>(index)].body->getBaseWorldTransform();
    return fromXyzw(fromBt(tr.getOrigin()), tr.getRotation());
}

std::pair<Vec3, Vec3> PropWorld::Impl::baseVelocity(int index) const {
    const auto &mb = *bodies[static_cast<std::size_t>(index)].body;
    return {fromBt(mb.getBaseVel()), fromBt(mb.getBaseOmega())};
}

void PropWorld::Impl::setCollisionPair(int a, int b, bool enabled) {
    auto *ca = bodies[static_cast<std::size_t>(a)].collider.get();
    auto *cb = bodies[static_cast<std::size_t>(b)].collider.get();
    const auto key = PairFilter::key(ca, cb);
    if (enabled) {
        // The broadphase only re-reports a pair when a proxy leaves its fattened bounds, so an
        // overlapping pair is restored here (pybullet contacts resume on the next step).
        if (pair_filter.excluded.erase(key)) {
            btBroadphaseProxy *pa = ca->getBroadphaseHandle(), *pb = cb->getBroadphaseHandle();
            if (TestAabbAgainstAabb2(pa->m_aabbMin, pa->m_aabbMax, pb->m_aabbMin, pb->m_aabbMax) &&
                !pair_cache->findPair(pa, pb))
                pair_cache->addOverlappingPair(pa, pb);
        }
    } else {
        pair_filter.excluded.insert(key);
        pair_cache->removeOverlappingPair(ca->getBroadphaseHandle(), cb->getBroadphaseHandle(), dispatcher.get());
    }
}

void PropWorld::Impl::build() {
    const double gravity = settings.at("gravity_m_s2").get<double>();
    configuration = std::make_unique<btDefaultCollisionConfiguration>();
    dispatcher = std::make_unique<btCollisionDispatcher>(configuration.get());
    pair_cache = std::make_unique<btHashedOverlappingPairCache>();
    pair_filter.excluded.clear();
    pair_cache->setOverlapFilterCallback(&pair_filter);
    broadphase = std::make_unique<btDbvtBroadphase>(pair_cache.get());
    solver = std::make_unique<btMultiBodyConstraintSolver>();
    world = std::make_unique<btMultiBodyDynamicsWorld>(dispatcher.get(), broadphase.get(),
                                                       solver.get(), configuration.get());
    world->setGravity(btVector3(0, 0, -gravity));
    world->getSolverInfo().m_numIterations = settings.at("solver_iterations").get<int>();
    world->getDispatchInfo().m_deterministicOverlappingPairs = true;

    scenery.clear(), floors.clear(), pads.clear(), props.clear(), static_meshes.clear(), basket.clear();
    const auto boxShape = [](const Vec3 &half) {
        auto shape = std::make_unique<btBoxShape>(toBt(half));
        shape->setMargin(kUrdfDefaultMarginM);
        return shape;
    };
    for (const auto &box : pool_boxes) {
        const int uid = addBody(boxShape(box.half), nullptr, 0, box.transform);
        scenery.push_back(uid);
        if (box.floor)
            floors.push_back(uid);
    }
    for (const auto &prop : statics) {
        const Json &parameters = prop.at("parameters");
        if (parameters.contains("collision_meshes"))
            for (const auto &mesh : parameters.at("collision_meshes")) {
                const auto asset = mesh.at("asset").get<std::string>();
                const auto path = task_assets.find(asset);
                if (path == task_assets.end())
                    throw std::invalid_argument("asset '" + asset + "' is not present in its pack");
                const ObjMesh obj = loadObj(path->second);
                auto triangles = std::make_unique<btTriangleMesh>();
                for (const auto &t : obj.triangles)
                    triangles->addTriangle(toBt(obj.vertices[static_cast<std::size_t>(t[0])]),
                                           toBt(obj.vertices[static_cast<std::size_t>(t[1])]),
                                           toBt(obj.vertices[static_cast<std::size_t>(t[2])]));
                auto shape = std::make_unique<btBvhTriangleMeshShape>(triangles.get(), true, true);
                const Matrix4 &t = frames.at(mesh.at("frame").get<std::string>());
                const int uid = addBody(std::move(shape), std::move(triangles), 0, t);
                changeDynamics(uid, mesh.at("lateral_friction").get<double>(),
                               mesh.at("restitution").get<double>());
                static_meshes[mesh.at("id").get<std::string>()] = uid;
                scenery.push_back(uid);
            }
        for (const auto &box : parameters.at("collision_boxes")) {
            const Matrix4 t = world_from_task * poseOf(box.at("center_m"), box.at("orientation_wxyz"));
            scenery.push_back(addBody(boxShape(vec3(box.at("size_m"), "size_m") / 2), nullptr, 0, t));
        }
    }
    for (const auto &prop : rigid) {
        const Json &c = prop.at("parameters");
        const auto asset = c.at("collision_asset").get<std::string>();
        const auto path = task_assets.find(asset);
        if (path == task_assets.end())
            throw std::invalid_argument("asset '" + asset + "' is not present in its pack");
        const ObjMesh obj = loadObj(path->second);
        Vec3 low = obj.vertices[0], high = obj.vertices[0];
        for (const auto &v : obj.vertices)
            low = low.cwiseMin(v), high = high.cwiseMax(v);
        const Vec3 center = (low + high) / 2;
        std::vector<btVector3> points;
        points.reserve(obj.vertices.size());
        for (const auto &v : obj.vertices)
            points.push_back(toBt(v - center));
        auto shape = std::make_unique<btConvexHullShape>(&points[0].x(), static_cast<int>(points.size()),
                                                         sizeof(btVector3));
        shape->setMargin(kUrdfDefaultMarginM);
        shape->recalcLocalAabb();
        const Matrix4 &t = frames.at(c.at("frame").get<std::string>());
        Matrix4 pose = t;
        pose.block<3, 1>(0, 3) = t.block<3, 1>(0, 3) + t.block<3, 3>(0, 0) * center;
        btCollisionShape *raw = shape.get();
        const int uid = addBody(std::move(shape), nullptr, c.at("mass_kg").get<double>(), pose);
        auto &body = bodies[static_cast<std::size_t>(uid)];
        auto &collider = *body.collider;
        collider.setFriction(c.at("lateral_friction").get<double>());
        collider.setSpinningFriction(c.at("spinning_friction").get<double>());
        collider.setRollingFriction(c.at("rolling_friction").get<double>());
        collider.setRestitution(c.at("restitution").get<double>());
        body.body->setLinearDamping(c.at("linear_damping").get<double>());
        body.body->setAngularDamping(c.at("angular_damping").get<double>());
        raw->setMargin(c.at("collision_margin_m").get<double>()); // no AABB recompute, as pybullet
        collider.setCcdSweptSphereRadius(c.at("ccd_swept_sphere_radius_m").get<double>());
        collider.setContactProcessingThreshold(c.at("contact_processing_threshold_m").get<double>());
        Prop entry;
        entry.id = prop.at("id").get<std::string>();
        entry.body = uid;
        entry.center = center;
        entry.half = (high - low) / 2;
        entry.config = &c;
        props.push_back(entry);
    }
    // Position-held rack drives: kinematic contact surfaces commanded by the mechanism.
    claw_body = addBody(nullptr, nullptr, 0, Matrix4::Identity());
    for (const auto &path : pad_paths) {
        const ObjMesh obj = loadObj(path);
        std::vector<btVector3> points;
        for (const auto &v : obj.referenced())
            points.push_back(toBt(v));
        if (points.empty())
            throw std::invalid_argument("pad mesh has no faces: " + path.string());
        auto shape = std::make_unique<btConvexHullShape>(&points[0].x(), static_cast<int>(points.size()),
                                                         sizeof(btVector3));
        shape->setMargin(kUrdfDefaultMarginM);
        shape->recalcLocalAabb();
        btCollisionShape *raw = shape.get();
        Matrix4 pose = Matrix4::Identity();
        pose(2, 3) = 10;
        const int uid = addBody(std::move(shape), nullptr, 0, pose);
        auto &collider = *bodies[static_cast<std::size_t>(uid)].collider;
        collider.setFriction(claw.at("friction").get<double>());
        collider.setSpinningFriction(kPadSpinningFriction);
        collider.setRestitution(0);
        raw->setMargin(kPadMarginM);
        pads.push_back(uid);
    }
    for (const auto &r : baskets)
        basket.push_back({r.at("id").get<std::string>(),
                          frames.at(r.at("parameters").at("frame").get<std::string>()).inverse(),
                          &r.at("parameters")});
    resetState();
}

void PropWorld::Impl::resetState() {
    held.reset();
    constraint.reset();
    held_relative = Matrix4::Identity();
    q = initial_q;
    direction = 0;
    lockout = false;
    last_command = initial_q;
    contact_time.clear();
    pending.clear();
    time_ns.reset();
    placePads(Matrix4::Identity(), q, Vec3::Zero(), Vec3::Zero(), 0.0);
    for (auto &p : props)
        restore(p);
}

void PropWorld::Impl::restore(Prop &prop) {
    const Matrix4 &t = frames.at(prop.config->at("frame").get<std::string>());
    resetBase(prop.body, t.block<3, 1>(0, 3) + t.block<3, 3>(0, 0) * prop.center, quatOf(t));
    setVelocity(prop.body, Vec3::Zero(), Vec3::Zero());
    prop.settled = 0.0, prop.scored = false, prop.picked = false;
}

// ---------------------------------------------------------------------------------- queries

std::vector<Contact> PropWorld::Impl::contactPoints(int a, int b) const {
    std::vector<Contact> result;
    auto *dispatch = world->getDispatcher();
    for (int i = 0; i < dispatch->getNumManifolds(); ++i) {
        const btPersistentManifold *m = dispatch->getManifoldByIndexInternal(i);
        const int i0 = m->getBody0()->getUserIndex(), i1 = m->getBody1()->getUserIndex();
        const bool direct = i0 == a && (b < 0 || i1 == b);
        const bool swapped = !direct && i1 == a && (b < 0 || i0 == b);
        if (!direct && !swapped)
            continue;
        for (int p = 0; p < m->getNumContacts(); ++p) {
            const btManifoldPoint &pt = m->getContactPoint(p);
            Contact c;
            c.other = direct ? i1 : i0;
            c.normal = fromBt(pt.m_normalWorldOnB) * (swapped ? -1.0 : 1.0);
            c.distance = pt.getDistance();
            c.force = pt.m_appliedImpulse / dt;
            result.push_back(c);
        }
    }
    return result;
}

std::vector<Contact> PropWorld::Impl::closestPoints(int a, int b) const {
    const btCollisionObject *ca = bodies[static_cast<std::size_t>(a)].collider.get();
    const btCollisionObject *cb = bodies[static_cast<std::size_t>(b)].collider.get();
    const btCollisionObjectWrapper wa(nullptr, ca->getCollisionShape(), ca, ca->getWorldTransform(), -1, -1);
    const btCollisionObjectWrapper wb(nullptr, cb->getCollisionShape(), cb, cb->getWorldTransform(), -1, -1);
    auto *dispatch = world->getDispatcher();
    btCollisionAlgorithm *algorithm = dispatch->findAlgorithm(&wa, &wb, nullptr, BT_CLOSEST_POINT_ALGORITHMS);
    if (!algorithm)
        return {};
    btDispatcherInfo info;
    info.m_timeStep = dt;
    info.m_stepCount = 0;
    info.m_dispatchFunc = btDispatcherInfo::DISPATCH_DISCRETE;
    ClosestPoints collector(&wa, &wb);
    collector.m_closestPointDistanceThreshold = 0;
    algorithm->processCollision(&wa, &wb, info, &collector);
    algorithm->~btCollisionAlgorithm();
    dispatch->freeCollisionAlgorithm(algorithm);
    for (auto &c : collector.points)
        c.other = b;
    return collector.points;
}

Matrix4 PropWorld::Impl::propPose(const Prop &prop) const {
    Matrix4 t = basePose(prop.body);
    t.block<3, 1>(0, 3) -= t.block<3, 3>(0, 0) * prop.center;
    return t;
}

std::optional<std::string> PropWorld::Impl::destination(std::size_t key,
                                                        std::vector<std::size_t> resting) const {
    const Prop &p = props[key];
    if (held && *held == key)
        return std::nullopt;
    const Matrix4 pose = basePose(p.body);
    const Vec3 position = pose.block<3, 1>(0, 3);
    const Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);
    for (const auto &b : basket) {
        const Vec3 local = (b.region_from_world * position.homogeneous()).head<3>();
        const Vec3 extent = (b.region_from_world.block<3, 3>(0, 0) * rotation).cwiseAbs() * p.half;
        const auto &region = *b.region;
        const double low = region.at("z_range_m").at(0).get<double>();
        const double high = region.at("z_range_m").at(1).get<double>();
        const double hx = region.at("half_extents_xy_m").at(0).get<double>();
        const double hy = region.at("half_extents_xy_m").at(1).get<double>();
        if (std::abs(local.x()) + extent.x() < hx && std::abs(local.y()) + extent.y() < hy &&
            low < local.z() && local.z() < high) {
            const int support = static_meshes.at(region.at("support_mesh").get<std::string>());
            if (!contactPoints(p.body, support).empty())
                return b.id;
            // Stacked: resting on another prop that is itself in this basket.
            resting.push_back(key);
            for (std::size_t other = 0; other < props.size(); ++other) {
                if (std::find(resting.begin(), resting.end(), other) != resting.end())
                    continue;
                if (!contactPoints(p.body, props[other].body).empty()) {
                    const auto inner = destination(other, resting);
                    if (inner && *inner == b.id)
                        return b.id;
                }
            }
        }
    }
    return std::nullopt;
}

// ---------------------------------------------------------------------------------- dynamics

void PropWorld::Impl::placePads(const Matrix4 &mount_pose, double q_value, const Vec3 &velocity,
                                const Vec3 &angular, double dq) {
    const double signs[2] = {1, -1};
    const Vec3 axis = mount_pose.block<3, 1>(0, 1);
    for (int i = 0; i < 2; ++i) {
        const Vec3 offset = axis * (signs[i] * q_value);
        resetBase(pads[static_cast<std::size_t>(i)], mount_pose.block<3, 1>(0, 3) + offset, quatOf(mount_pose));
        setVelocity(pads[static_cast<std::size_t>(i)],
                    velocity + angular.cross(offset) + axis * (signs[i] * dq), angular);
    }
}

void PropWorld::Impl::driveJaws(double dt_s, const Matrix4 &mount_pose, const Vec3 &velocity,
                                const Vec3 &angular, double target) {
    const double old = q;
    const double step = claw.at("jaw_speed_m_s").get<double>() * dt_s;
    double proposed = std::min(std::max(target, old - step), old + step);
    const double dir = sign(proposed - old);
    placePads(mount_pose, proposed, velocity, angular, 0.0);
    if (dir != 0.0) {
        // Stall commanded travel against solid contacts; scenery never moves the jaws.
        const double signs[2] = {1, -1};
        for (int i = 0; i < 2; ++i) {
            const Vec3 axis = Vec3(mount_pose.block<3, 1>(0, 1)) * signs[i];
            std::vector<int> others = scenery;
            for (const auto &p : props)
                others.push_back(p.body);
            const int pad = pads[static_cast<std::size_t>(i)];
            const btCollisionObject &pad_object = *bodies[static_cast<std::size_t>(pad)].collider;
            btVector3 pad_low, pad_high;
            pad_object.getCollisionShape()->getAabb(pad_object.getWorldTransform(), pad_low, pad_high);
            for (const int other : others) {
                // Only penetration deeper than the stall threshold matters, which needs overlapping
                // bounds; skipping disjoint pairs is exact and avoids most narrow-phase queries.
                const btCollisionObject &object = *bodies[static_cast<std::size_t>(other)].collider;
                btVector3 low, high;
                object.getCollisionShape()->getAabb(object.getWorldTransform(), low, high);
                if (low.x() > pad_high.x() || high.x() < pad_low.x() || low.y() > pad_high.y() ||
                    high.y() < pad_low.y() || low.z() > pad_high.z() || high.z() < pad_low.z())
                    continue;
                for (const Contact &c : closestPoints(pad, other)) {
                    const double normal_speed = c.normal.dot(axis) * dir;
                    if (normal_speed < kStallNormalSpeed && c.distance < kStallPenetrationM)
                        proposed -= dir * std::min(std::abs(proposed - old),
                                                   (kStallPenetrationM - c.distance) / (-normal_speed));
                }
            }
        }
        q = proposed;
    }
    placePads(mount_pose, q, velocity, angular, (q - old) / dt_s);
}

Event PropWorld::Impl::event(const std::string &key, std::int64_t t, const Json &region,
                             Json data) const {
    const Json &source = events.at(key);
    const auto emits = source.at("parameters").find("emits");
    if (emits != source.at("parameters").end()) {
        Json filtered = Json::object();
        for (const auto &[name, value] : data.items())
            if (std::find(emits->begin(), emits->end(), name) != emits->end())
                filtered[name] = value;
        data = filtered;
    }
    return Event{{"task", task}, {"type", source.at("type")}, {"id", source.at("id")},
                 {"region", region}, {"time_ns", t}, {"data", data}};
}

void PropWorld::Impl::release(const std::string &reason, std::int64_t t) {
    if (constraint) {
        world->removeMultiBodyConstraint(constraint.get());
        constraint.reset();
    }
    if (held) {
        for (const int uid : pads)
            setCollisionPair(uid, props[*held].body, true);
        pending.push_back(event("detach", t, Json(),
                                {{"prop_id", props[*held].id},
                                 {"mechanism_id", mechanism_id},
                                 {"reason", reason}}));
    }
    held.reset();
}

void PropWorld::Impl::inferDirection(double command, bool enabled) {
    const double delta = command - last_command;
    last_command = command;
    if (!enabled) {
        direction = 0;
    } else if (delta > kDirectionEpsilonM) {
        direction = 1, lockout = false;
    } else if (delta < -kDirectionEpsilonM) {
        direction = lockout ? 0 : -1;
    } else if (!(direction == -1 && q > command + kDirectionEpsilonM)) {
        // blocked closing continues; anything else is a stop
        direction = 0;
    }
}

void PropWorld::Impl::tryGrasp(double dt_s, const Matrix4 &mount_pose, std::int64_t t) {
    const double signs[2] = {1, -1};
    const Vec3 axis = mount_pose.block<3, 1>(0, 1);
    for (std::size_t key = 0; key < props.size(); ++key) {
        Prop &p = props[key];
        bool touching = true;
        for (int i = 0; i < 2 && touching; ++i) {
            bool any = false;
            for (const Contact &c : contactPoints(pads[static_cast<std::size_t>(i)], p.body))
                if (c.distance < claw.at("contact_margin_m").get<double>() && c.force > kGraspNormalForceN &&
                    c.normal.dot(axis * signs[i]) > kGraspNormalAlignment) {
                    any = true;
                    break;
                }
            touching = any;
        }
        double &time = contact_time[p.id];
        time = touching ? time + dt_s : 0;
        if (time < claw.at("grasp_dwell_s").get<double>())
            continue;
        const Matrix4 relative = mount_pose.inverse() * basePose(p.body);
        // Joint frame in the parent (claw) and in the child (prop, identity), pybullet JOINT_FIXED.
        const btTransform rel = transformOf(relative);
        constraint = std::make_unique<btMultiBodyFixedConstraint>(
            bodies[static_cast<std::size_t>(claw_body)].body.get(), -1,
            bodies[static_cast<std::size_t>(p.body)].body.get(), -1, toBt(relative.block<3, 1>(0, 3)),
            btVector3(0, 0, 0), btMatrix3x3(rel.getRotation()), btMatrix3x3::getIdentity());
        constraint->setMaxAppliedImpulse(claw.at("hold_force_n").get<double>() * dt);
        constraint->finalizeMultiDof();
        world->addMultiBodyConstraint(constraint.get());
        // A grasped object and its pads form one assembly; internal contacts must not fight the
        // grasp constraint.
        for (const int uid : pads)
            setCollisionPair(uid, p.body, false);
        held = key, held_relative = relative;
        p.picked = true, p.scored = false, p.settled = 0.0;
        direction = 0, lockout = true;
        pending.push_back(event("attach", t, Json(), {{"prop_id", p.id}, {"mechanism_id", mechanism_id}}));
        break;
    }
}

void PropWorld::Impl::settle(double dt_s, std::int64_t t) {
    const double rest_speed = settings.at("rest_speed_m_s").get<double>();
    const double rest_time = settings.at("rest_time_s").get<double>();
    for (std::size_t key = 0; key < props.size(); ++key) {
        Prop &p = props[key];
        if ((held && *held == key) || p.scored)
            continue;
        const Vec3 velocity = baseVelocity(p.body).first;
        auto dest = destination(key);
        std::optional<std::string> label = dest;
        bool floor_label = false;
        if (p.picked) {
            bool on_floor = false;
            for (const int f : floors)
                on_floor = on_floor || !contactPoints(p.body, f).empty();
            if (on_floor) {
                label = "floor", floor_label = true, dest.reset();
            } else if (!dest) {
                for (const Contact &c : contactPoints(p.body))
                    if (std::find(pads.begin(), pads.end(), c.other) == pads.end()) {
                        label = "support";
                        break;
                    }
            }
        }
        if (label && velocity.norm() < rest_speed) {
            p.settled += dt_s;
            if (p.settled > rest_time) {
                if (dest) {
                    pending.push_back(event("drop_in_region", t, *dest,
                                            {{"prop_id", p.id},
                                             {"basket", *dest},
                                             {"expected_basket", p.config->at("expected_region")}}));
                } else {
                    const Json &surfaces = events.at("drop_elsewhere").at("parameters").at("surfaces");
                    pending.push_back(event("drop_elsewhere", t,
                                            surfaces.at(floor_label ? "floor" : "support"),
                                            {{"prop_id", p.id}}));
                }
                p.scored = true;
            }
        } else {
            p.settled = 0.0;
        }
    }
}

// ----------------------------------------------------------------------------- robot contacts

void PropWorld::Impl::buildVehicle(double friction) {
    vehicle = std::make_shared<VehicleContacts>(friction);
    const auto meshShape = [](const ObjMesh &obj) {
        auto triangles = std::make_unique<btTriangleMesh>();
        for (const auto &t : obj.triangles)
            triangles->addTriangle(toBt(obj.vertices[static_cast<std::size_t>(t[0])]),
                                   toBt(obj.vertices[static_cast<std::size_t>(t[1])]),
                                   toBt(obj.vertices[static_cast<std::size_t>(t[2])]));
        auto shape = std::make_unique<btBvhTriangleMeshShape>(triangles.get(), true, true);
        return std::make_pair(std::move(shape), std::move(triangles));
    };
    const auto hull = [](const std::vector<Vec3> &vertices, const Vec3 &center) {
        auto shape = std::make_unique<btConvexHullShape>();
        for (const auto &v : vertices)
            shape->addPoint(toBt(v - center), false);
        shape->recalcLocalAabb();
        shape->optimizeConvexHull();
        return shape;
    };
    for (const auto &box : pool_boxes)
        vehicle->add(std::make_unique<btBoxShape>(toBt(box.half)), nullptr, VehicleContacts::kScenery, box.transform);
    for (const auto &prop : statics) {
        const Json &parameters = prop.at("parameters");
        if (parameters.contains("collision_meshes"))
            for (const auto &mesh : parameters.at("collision_meshes")) {
                auto [shape, triangles] = meshShape(loadObj(task_assets.at(mesh.at("asset").get<std::string>())));
                vehicle->add(std::move(shape), std::move(triangles), VehicleContacts::kScenery,
                             frames.at(mesh.at("frame").get<std::string>()));
            }
        for (const auto &box : parameters.at("collision_boxes"))
            vehicle->add(std::make_unique<btBoxShape>(toBt(vec3(box.at("size_m"), "size_m") / 2)), nullptr,
                         VehicleContacts::kScenery,
                         world_from_task * poseOf(box.at("center_m"), box.at("orientation_wxyz")));
    }
    for (const auto &path : pad_paths)
        vehicle_pads.push_back(vehicle->add(hull(loadObj(path).referenced(), Vec3::Zero()), nullptr,
                                            VehicleContacts::kRobot, mount));
    for (const auto &p : props) {
        const auto obj = loadObj(task_assets.at(p.config->at("collision_asset").get<std::string>()));
        vehicle_props.push_back(
            vehicle->add(hull(obj.vertices, p.center), nullptr, VehicleContacts::kProp, basePose(p.body)));
    }
    syncVehicle();
}

void PropWorld::Impl::syncVehicle() {
    if (!vehicle)
        return;
    const double signs[2] = {1, -1};
    for (int i = 0; i < 2; ++i) {
        Matrix4 offset = Matrix4::Identity();
        offset(1, 3) = signs[i] * q;
        vehicle->pose(vehicle_pads[static_cast<std::size_t>(i)], mount * offset);
    }
    for (std::size_t key = 0; key < props.size(); ++key) {
        if (held && *held == key) // the grasp holds it at the grasp pose relative to the claw
            vehicle->pose(vehicle_props[key], mount * held_relative, true);
        else
            vehicle->pose(vehicle_props[key], basePose(props[key].body));
    }
}

// ------------------------------------------------------------------------------------ facade

PropWorld::PropWorld(const ResolvedScenario &scenario, const std::string &task,
                     const std::string &mechanism_id)
    : impl_(std::make_unique<Impl>(scenario, task, mechanism_id)) {}
PropWorld::~PropWorld() = default;

const std::string &PropWorld::task() const { return impl_->task; }
const std::string &PropWorld::mechanismId() const { return impl_->mechanism_id; }
double PropWorld::jawPosition() const { return impl_->q; }

void PropWorld::reset() {
    // Recreate the contact world (clears broadphase caches, constraints and poses).
    impl_->teardown();
    impl_->build();
    impl_->syncVehicle();
}

std::shared_ptr<simulation::ContactResolver> PropWorld::vehicleContacts(double friction) {
    if (!impl_->vehicle)
        impl_->buildVehicle(friction);
    return impl_->vehicle;
}

std::map<std::string, PropState> PropWorld::props() const {
    std::map<std::string, PropState> result;
    for (std::size_t key = 0; key < impl_->props.size(); ++key) {
        const auto &p = impl_->props[key];
        const Matrix4 t = impl_->propPose(p);
        const auto w = xyzw(t);
        PropState state;
        state.position = t.block<3, 1>(0, 3);
        state.orientation = Eigen::Quaterniond(w[3], w[0], w[1], w[2]);
        state.attached = impl_->held && *impl_->held == key;
        if (state.attached)
            state.mechanism_id = impl_->mechanism_id;
        if (const auto basket = impl_->destination(key))
            state.basket = *basket;
        result[p.id] = state;
    }
    return result;
}

std::map<std::string, std::string> PropWorld::basketContents() const {
    std::map<std::string, std::string> result;
    const double rest_time = impl_->settings.at("rest_time_s").get<double>();
    for (std::size_t key = 0; key < impl_->props.size(); ++key) {
        const auto &p = impl_->props[key];
        if (p.settled > rest_time)
            if (const auto basket = impl_->destination(key))
                result[p.id] = *basket;
    }
    return result;
}

Events PropWorld::step(double dt_s, std::int64_t time_ns, const spatial::Pose &robot_root_pose,
                       const Eigen::Vector3d &linear_velocity_world,
                       const Eigen::Vector3d &angular_velocity_world,
                       const std::array<double, 2> &claw_joint_positions, const Water &water,
                       bool enabled) {
    Impl &s = *impl_;
    if (time_ns < 0 || (s.time_ns && time_ns < *s.time_ns))
        throw std::invalid_argument("time_ns must be a nondecreasing nonnegative int");
    if (!std::isfinite(dt_s) || dt_s <= 0)
        throw std::invalid_argument("dt_s must be positive and finite");
    if (!allFinite(linear_velocity_world))
        throw std::invalid_argument("linear_velocity_world must contain 3 finite values");
    if (!allFinite(angular_velocity_world))
        throw std::invalid_argument("angular_velocity_world must contain 3 finite values");
    if (!std::isfinite(claw_joint_positions[0]) || !std::isfinite(claw_joint_positions[1]))
        throw std::invalid_argument("claw_joint_positions must contain 2 finite values");
    if (!allFinite(water.velocity_world))
        throw std::invalid_argument("water velocity must contain 3 finite values");
    if (!std::isfinite(water.density) || water.density <= 0)
        throw std::invalid_argument("water density must be positive and finite");
    s.time_ns = time_ns;
    const Matrix4 body = poseMatrix(robot_root_pose);
    const Matrix4 mount = body * s.mount;
    const Vec3 mount_velocity =
        linear_velocity_world + angular_velocity_world.cross(Vec3(mount.block<3, 1>(0, 3) - body.block<3, 1>(0, 3)));
    const double command =
        std::min(std::max((claw_joint_positions[0] + claw_joint_positions[1]) / 2, 0.0), s.travel);

    s.dt = dt_s;
    s.inferDirection(command, enabled);
    if (s.direction > 0)
        s.release("released", time_ns);
    // Original command semantics: open -> travel, close -> 0, stop -> hold the current gap.
    const double target = s.direction == 1 ? s.travel : (s.direction == -1 ? 0.0 : s.q);
    s.resetBase(s.claw_body, mount.block<3, 1>(0, 3), quatOf(mount));
    s.setVelocity(s.claw_body, mount_velocity, angular_velocity_world);
    s.driveJaws(dt_s, mount, mount_velocity, angular_velocity_world, target);
    const Json &settings = s.settings;
    const double gravity = settings.at("gravity_m_s2").get<double>();
    for (auto &p : s.props) {
        const Matrix4 pose = s.basePose(p.body);
        const Vec3 xyz = pose.block<3, 1>(0, 3);
        const Eigen::Matrix3d r = pose.block<3, 3>(0, 0);
        const auto [vel, omega] = s.baseVelocity(p.body);
        const Vec3 rel = vel - water.velocity_world;
        const double half_z = (r.cwiseAbs() * p.half)[2];
        const double wet = std::min(std::max((half_z - (xyz.z() - s.surface_z)) / (2 * half_z), 0.0), 1.0);
        const Vec3 local = r.transpose() * rel;
        const Vec3 size = 2 * p.half;
        const Vec3 area(size[1] * size[2], size[0] * size[2], size[0] * size[1]);
        Vec3 force = -settings.at("drag_coefficient").get<double>() * water.density * wet *
                     (r * Vec3(area.cwiseProduct(local).cwiseProduct(local.cwiseAbs())));
        force[2] += water.density * gravity * p.config->at("volume_m3").get<double>() * wet;
        auto &mb = *s.bodies[static_cast<std::size_t>(p.body)].body;
        mb.addBaseForce(toBt(force)); // applied at the COM: no torque
        mb.addBaseTorque(toBt(-settings.at("angular_drag_n_m_s_per_rad").get<double>() * wet * omega));
    }
    s.world->stepSimulation(dt_s, 0);
    if (s.held) {
        const Vec3 position = s.basePose(s.props[*s.held].body).block<3, 1>(0, 3);
        const Vec3 expected = (mount * s.held_relative).block<3, 1>(0, 3);
        if ((position - expected).norm() > s.claw.at("slip_distance_m").get<double>())
            s.release("slipped", time_ns);
    }
    if (!s.held && s.direction < 0 && enabled)
        s.tryGrasp(dt_s, mount, time_ns);
    s.settle(dt_s, time_ns);
    s.syncVehicle();
    Events events;
    events.swap(s.pending);
    return events;
}
} // namespace robotics::session

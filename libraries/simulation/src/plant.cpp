#include "nereus/simulation/plant.hpp"
#include "detail/box_contacts.hpp"
#include "detail/marine_dynamics.hpp"
#include "detail/rk4.hpp"
#include "detail/thruster_dynamics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace nereus::simulation {
namespace {
void require(bool condition, const std::string &message) {
    if (!condition) {
        throw std::invalid_argument(message);
    }
}

void validate(const PlantParameters &p) {
    require(p.timestep.count() > 0 && p.timestep <= std::chrono::milliseconds(100), "timestep must be in (0, 100ms]");
    for (double value : {p.pool.length, p.pool.width, p.pool.depth, p.pool.water_density}) {
        require(std::isfinite(value) && value > 0, "pool dimensions and density must be positive");
    }
    require(std::isfinite(p.pool.water_level) && p.pool.current_velocity.allFinite(),
            "pool level and current must be finite");
    require(p.pool.origin_xy_world.allFinite() && std::isfinite(p.pool.yaw_world), "pool placement must be finite");
    const double frequency = p.pool.current_oscillation_frequency;
    const double omega = 2 * std::acos(-1.0) * frequency;
    require(std::isfinite(frequency) && frequency >= 0 && p.pool.current_oscillation_amplitude.allFinite() &&
                (omega * p.pool.current_oscillation_amplitude).allFinite() &&
                std::isfinite(omega * (static_cast<double>(std::numeric_limits<std::int64_t>::max()) / 1e9)),
            "current oscillation must have finite amplitude and nonnegative frequency");
    require(std::isfinite(p.pool.water_level - p.pool.depth), "pool floor must be finite");
    if (!p.pool.floor.empty()) {
        const double extent = p.pool.floor.axis() == FloorProfile::Axis::X ? p.pool.length : p.pool.width;
        require(std::abs(p.pool.floor.extent() - extent) <= 1e-9 * std::max(1.0, extent),
                "pool floor profile must span the pool along its axis");
        require(std::abs(p.pool.floor.maxDepth() - p.pool.depth) <= 1e-9 * std::max(1.0, p.pool.depth),
                "pool depth must be the floor profile's deepest point");
        require(p.contacts.model != ContactModel::SpherePool, "sphere_pool contacts need a flat pool floor");
    }
    std::set<std::string> ids;
    for (const auto &t : p.thrusters) {
        require(!t.id.empty() && ids.insert(t.id).second, "thruster IDs must be nonempty and unique");
        require(t.position.allFinite() && t.direction.allFinite() && std::abs(t.direction.norm() - 1.0) < 1e-9,
                "thruster " + t.id + ": position must be finite and direction must be a unit vector");
        require(!t.propeller_radius || (std::isfinite(*t.propeller_radius) && *t.propeller_radius > 0),
                "propeller radius must be positive and finite when configured");
    }
}

detail::State13d pack(const BodyState &s) {
    require(s.position.allFinite() && s.orientation.coeffs().allFinite() && std::isfinite(s.orientation.norm()) &&
                s.orientation.norm() > 1e-10 && s.linear_velocity.allFinite() && s.angular_velocity.allFinite(),
            "body state must be finite with a nonzero quaternion");
    const auto q = s.orientation.normalized();
    detail::State13d x;
    x << s.position, q.w(), q.x(), q.y(), q.z(), s.linear_velocity, s.angular_velocity;
    return x;
}

BodyState unpack(const detail::State13d &x) {
    BodyState s;
    s.position = x.head<3>();
    s.orientation = Eigen::Quaterniond(x[3], x[4], x[5], x[6]);
    s.linear_velocity = x.segment<3>(7);
    s.angular_velocity = x.tail<3>();
    return s;
}

// Five static planes and a COM-centered sphere. Frictionless, zero restitution.
// No surface ceiling, general mesh contacts, or grasp/prop interaction in this slice.
class PoolContacts {
  public:
    explicit PoolContacts(const PlantParameters &p)
        : lower_(p.body.collision_radius, p.body.collision_radius,
                 p.pool.water_level - p.pool.depth + p.body.collision_radius),
          upper_(p.pool.length - p.body.collision_radius, p.pool.width - p.body.collision_radius,
                 std::numeric_limits<double>::infinity()),
          origin_(p.pool.origin_xy_world.x(), p.pool.origin_xy_world.y(), 0),
          rotation_(Eigen::AngleAxisd(p.pool.yaw_world, Eigen::Vector3d::UnitZ()).toRotationMatrix()) {
        if (!p.pool.origin_xy_world.isZero(0) || p.pool.yaw_world != 0)
            boundary_tolerance_ = 16 * std::numeric_limits<double>::epsilon() *
                                  std::max({1.0, origin_.cwiseAbs().maxCoeff(), p.pool.length, p.pool.width,
                                            std::abs(p.pool.water_level), p.pool.depth});
        require(std::isfinite(p.body.collision_radius) && p.body.collision_radius > 0 &&
                    2 * p.body.collision_radius < std::min({p.pool.length, p.pool.width, p.pool.depth}),
                "positive finite collision sphere must fit in the pool");
    }

    void validateInitial(const detail::State13d &x) const {
        const auto position = local(x.head<3>());
        require((position.array() >= lower_.array()).all() && (position.array() <= upper_.array()).all(),
                "initial collision sphere must be inside the pool walls and above the floor");
    }

    bool touching(const detail::State13d &x) const {
        const auto position = local(x.head<3>());
        return (position.array() <= lower_.array() + 1e-9).any() || (position.array() >= upper_.array() - 1e-9).any();
    }

    void resolve(detail::State13d &x, const Matrix6 &inverse_mass) const {
        const Eigen::Quaterniond q(x[3], x[4], x[5], x[6]);
        const Eigen::Vector3d position = local(x.head<3>()).cwiseMax(lower_).cwiseMin(upper_);
        x.head<3>() = origin_ + rotation_ * position;
        // Coupled added mass can reintroduce another normal velocity at corners.
        for (int pass = 0; pass < 32; ++pass) {
            bool corrected = false;
            for (int axis = 0; axis < 3; ++axis) {
                Eigen::Vector3d normal = Eigen::Vector3d::Zero();
                if (position[axis] <= lower_[axis]) {
                    normal[axis] = 1;
                } else if (axis < 2 && position[axis] >= upper_[axis]) {
                    normal[axis] = -1;
                } else {
                    continue;
                }
                Vector6 jacobian = Vector6::Zero();
                jacobian.head<3>() = q.conjugate() * (rotation_ * normal);
                const double speed = jacobian.dot(x.tail<6>());
                if (speed < -1e-10) {
                    const Vector6 response = inverse_mass * jacobian;
                    x.tail<6>() -= response * (speed / jacobian.dot(response));
                    corrected = true;
                }
            }
            if (!corrected) {
                return;
            }
        }
        throw std::runtime_error("pool contact solve did not converge");
    }

  private:
    Eigen::Vector3d local(const Eigen::Vector3d &position) const {
        Eigen::Vector3d result = rotation_.transpose() * (position - origin_);
        // Preserve exact legal boundaries through finite-precision rigid transforms.
        for (int axis = 0; axis < 3; ++axis) {
            if (std::abs(result[axis] - lower_[axis]) <= boundary_tolerance_)
                result[axis] = lower_[axis];
            else if (std::abs(result[axis] - upper_[axis]) <= boundary_tolerance_)
                result[axis] = upper_[axis];
        }
        return result;
    }
    double boundary_tolerance_{0};
    Eigen::Vector3d lower_, upper_, origin_;
    Eigen::Matrix3d rotation_;
};
} // namespace

struct Plant::Impl {
    explicit Impl(const PlantParameters &p, const BodyState &initial) : parameters(p), state(pack(initial)) {
        validate(p);
        switch (p.contacts.model) {
        case ContactModel::Disabled:
            break;
        case ContactModel::SpherePool:
            pool_contacts = std::make_unique<PoolContacts>(p);
            pool_contacts->validateInitial(state);
            break;
        case ContactModel::BoxScene:
            box_contacts = std::make_unique<detail::BoxContacts>(p.contacts.body_boxes, p.contacts.world_boxes,
                                                                 p.contacts.restitution, p.contacts.friction);
            break;
        default:
            throw std::invalid_argument("unknown contact model");
        }
        const auto &b = p.body;
        dynamics.configure(b.mass, b.inertia, b.added_mass);
        dynamics.configureDamping(b.linear_damping, b.quadratic_damping, b.damping_center);
        dynamics.configureHydrostatics(p.pool.water_density, b.displaced_volume, b.buoyancy_center, b.buoyancy_radii,
                                       9.80665, p.pool.water_level);
        std::vector<detail::ThrusterParameters> actuator_parameters;
        allocation.resize(6, static_cast<Eigen::Index>(p.thrusters.size()));
        for (std::size_t i = 0; i < p.thrusters.size(); ++i) {
            const auto &t = p.thrusters[i];
            detail::ThrusterParameters a;
            a.delay = t.delay;
            a.rise = t.rise_time;
            a.fall = t.fall_time;
            a.slew = t.slew_rate;
            a.forwardLimit = t.forward_limit;
            a.reverseLimit = t.reverse_limit;
            a.deadband = t.deadband;
            a.forwardScale = t.forward_scale;
            a.reverseScale = t.reverse_scale;
            a.efficiency = t.efficiency;
            actuator_parameters.push_back(a);
            allocation.col(static_cast<Eigen::Index>(i)) << t.direction, t.position.cross(t.direction);
        }
        require(allocation.allFinite(), "thruster allocation must be finite");
        actuators.configure(actuator_parameters, p.command_timeout);
        committed_forces = actuators.forces();
    }

    Vector6 propulsion(const detail::State13d &stage, Eigen::VectorXd forces) const {
        const Eigen::Quaterniond q = Eigen::Quaterniond(stage[3], stage[4], stage[5], stage[6]).normalized();
        const double pi = std::acos(-1.0);
        for (std::size_t i = 0; i < parameters.thrusters.size(); ++i) {
            const auto &thruster = parameters.thrusters[i];
            if (!thruster.propeller_radius)
                continue;
            const double z = (stage.head<3>() + q * thruster.position).z();
            const double axis_z = (q * thruster.direction).z();
            const double extent =
                std::max(.001, *thruster.propeller_radius * std::sqrt(std::max(0.0, 1 - axis_z * axis_z)));
            const double c = std::clamp((parameters.pool.water_level - z) / extent, -1.0, 1.0);
            forces[static_cast<Eigen::Index>(i)] *= (std::acos(-c) + c * std::sqrt(std::max(0.0, 1 - c * c))) / pi;
        }
        return allocation * forces;
    }
    detail::State13d derivative(const detail::State13d &stage, const Eigen::VectorXd &forces, double time) const {
        const auto &pool = parameters.pool;
        const double omega = 2 * std::acos(-1.0) * pool.current_oscillation_frequency;
        const double phase = omega * time;
        return dynamics.derivative(stage, propulsion(stage, forces),
                                   pool.current_velocity + pool.current_oscillation_amplitude * std::sin(phase),
                                   pool.current_oscillation_amplitude * (omega * std::cos(phase)));
    }

    PlantParameters parameters;
    detail::MarineDynamics dynamics;
    detail::ThrusterDynamics actuators;
    std::unique_ptr<PoolContacts> pool_contacts;
    std::unique_ptr<detail::BoxContacts> box_contacts;
    std::shared_ptr<ContactResolver> resolver;
    Eigen::Matrix<double, 6, Eigen::Dynamic> allocation;
    detail::State13d state;
    Eigen::VectorXd committed_forces;
    std::uint64_t tick = 0, generation = 0;
    bool faulted = false;
};

Plant::Plant(const PlantParameters &parameters, const BodyState &initial)
    : impl_(std::make_unique<Impl>(parameters, initial)) {}
Plant::~Plant() = default;

void Plant::command(const Eigen::VectorXd &forces) {
    if (impl_->faulted) {
        throw std::logic_error("plant must be reset after a failed advance");
    }
    impl_->actuators.command(forces);
}
void Plant::stopThrusters() {
    if (impl_->faulted)
        throw std::logic_error("plant must be reset after a failed advance");
    impl_->actuators.stop();
}

Snapshot Plant::observe() const {
    const auto &p = *impl_;
    return {p.generation, p.tick,
            std::chrono::nanoseconds(static_cast<std::int64_t>(p.tick) * p.parameters.timestep.count()),
            unpack(p.state), p.committed_forces};
}

MotionSample Plant::motion() const {
    const auto &p = *impl_;
    if (p.faulted) {
        throw std::logic_error("plant must be reset before sampling failed dynamics");
    }
    MotionSample sample;
    sample.state = observe();
    const auto derivative =
        p.derivative(p.state, p.committed_forces, std::chrono::duration<double>(sample.state.elapsed).count());
    const auto &body = sample.state.body;
    sample.acceleration_body = derivative.segment<3>(7) + body.angular_velocity.cross(body.linear_velocity);
    sample.angular_acceleration_body = derivative.tail<3>();
    // Box contacts expose post-impulse free-motion derivatives, excluding the
    // impulse itself, as in the reference. Sphere contacts retain their policy.
    sample.acceleration_valid = !p.pool_contacts || !p.pool_contacts->touching(p.state);
    if (!sample.acceleration_body.allFinite() || !sample.angular_acceleration_body.allFinite()) {
        throw std::runtime_error("nonfinite motion derivative");
    }
    return sample;
}

Snapshot Plant::advance(std::uint64_t ticks) {
    auto &p = *impl_;
    if (p.faulted) {
        throw std::logic_error("plant must be reset after a failed advance");
    }
    const auto max_tick =
        static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max() / p.parameters.timestep.count());
    if (ticks > max_tick - p.tick) {
        throw std::overflow_error("requested advance overflows simulation time");
    }
    const double dt = std::chrono::duration<double>(p.parameters.timestep).count();
    try {
        for (std::uint64_t i = 0; i < ticks; ++i) {
            // Midpoint actuator force held during the RK4 body step (operator splitting).
            // One authoritative state commit per integer tick.
            p.actuators.advance(dt / 2);
            auto start = p.state;
            if (p.box_contacts)
                start = p.box_contacts->resolve(start, p.dynamics.inverseMass());
            if (p.resolver)
                start = p.resolver->resolve(start, p.dynamics.inverseMass());
            const double time = static_cast<double>(p.tick) * dt;
            auto next = detail::integrateBodyRk4(start, dt, [&](const auto &stage, double offset) {
                return p.derivative(stage, p.actuators.forces(), time + offset);
            });
            if (p.box_contacts)
                next = p.box_contacts->resolve(next, p.dynamics.inverseMass());
            if (p.resolver)
                next = p.resolver->resolve(next, p.dynamics.inverseMass());
            if (p.box_contacts || p.resolver) {
                const auto q = Eigen::Quaterniond(next[3], next[4], next[5], next[6]).normalized();
                next[3] = q.w();
                next.segment<3>(4) = q.vec();
            } else {
                next.segment<4>(3).normalize();
            }
            if (p.pool_contacts)
                p.pool_contacts->resolve(next, p.dynamics.inverseMass());
            detail::MarineDynamics::validateState(next);
            p.actuators.advance(dt / 2);
            p.state = next;
            p.committed_forces = p.actuators.forces();
            ++p.tick;
        }
    } catch (...) {
        p.faulted = true;
        throw;
    }
    return observe();
}

void Plant::setContactResolver(std::shared_ptr<ContactResolver> resolver) {
    impl_->resolver = std::move(resolver);
}

Snapshot Plant::place(const BodyState &state, bool clear_actuators) {
    if (impl_->faulted)
        throw std::logic_error("plant must be reset after a failed advance");
    auto next = pack(state);
    if (impl_->pool_contacts)
        impl_->pool_contacts->validateInitial(next);
    if (clear_actuators) {
        impl_->actuators.clear();
        impl_->committed_forces.setZero();
    }
    impl_->state = next;
    return observe();
}

Snapshot Plant::reset(const BodyState &initial) {
    auto next = pack(initial);
    if (impl_->pool_contacts)
        impl_->pool_contacts->validateInitial(next);
    if (impl_->generation == std::numeric_limits<std::uint64_t>::max()) {
        throw std::overflow_error("reset generation overflow");
    }
    impl_->actuators.reset();
    impl_->state = next;
    impl_->committed_forces.setZero();
    impl_->tick = 0;
    ++impl_->generation;
    impl_->faulted = false;
    return observe();
}

FloorProfile floorOf(const Pool &pool) {
    return pool.floor.empty() ? FloorProfile::flat(pool.depth, pool.length) : pool.floor;
}
} // namespace nereus::simulation

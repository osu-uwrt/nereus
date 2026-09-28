#include "robotics/simulation/plant.hpp"
#include "detail/marine_dynamics.hpp"
#include "detail/thruster_dynamics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace robotics::simulation {
namespace {
void require(bool condition, const std::string &message) {
    if (!condition) {
        throw std::invalid_argument(message);
    }
}

void validate(const PlantParameters &p) {
    require(p.timestep.count() > 0 && p.timestep <= std::chrono::milliseconds(100),
            "timestep must be in (0, 100ms]");
    for (double value : {p.pool.length, p.pool.width, p.pool.depth, p.pool.water_density,
                         p.body.collision_radius}) {
        require(std::isfinite(value) && value > 0,
                "pool dimensions, density, radius must be positive");
    }
    require(std::isfinite(p.pool.water_level) && p.pool.current_velocity.allFinite(),
            "pool level and current must be finite");
    require(std::isfinite(p.pool.water_level - p.pool.depth), "pool floor must be finite");
    require(2 * p.body.collision_radius < std::min({p.pool.length, p.pool.width, p.pool.depth}),
            "collision sphere must fit in the pool");
    std::set<std::string> ids;
    for (const auto &t : p.thrusters) {
        require(!t.id.empty() && ids.insert(t.id).second,
                "thruster IDs must be nonempty and unique");
        require(t.position.allFinite() && t.direction.allFinite() &&
                    std::abs(t.direction.norm() - 1.0) < 1e-9,
                "thruster " + t.id +
                    ": position must be finite and direction must be a unit vector");
    }
}

detail::State13d pack(const BodyState &s) {
    require(s.position.allFinite() && s.orientation.coeffs().allFinite() &&
                std::isfinite(s.orientation.norm()) && s.orientation.norm() > 1e-10 &&
                s.linear_velocity.allFinite() && s.angular_velocity.allFinite(),
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
                 std::numeric_limits<double>::infinity()) {}

    void validateInitial(const detail::State13d &x) const {
        require((x.head<3>().array() >= lower_.array()).all() &&
                    (x.head<3>().array() <= upper_.array()).all(),
                "initial collision sphere must be inside the pool walls and above the floor");
    }

    bool touching(const detail::State13d &x) const {
        return (x.head<3>().array() <= lower_.array() + 1e-9).any() ||
               (x.head<3>().array() >= upper_.array() - 1e-9).any();
    }

    void resolve(detail::State13d &x, const Matrix6 &inverse_mass) const {
        const Eigen::Quaterniond q(x[3], x[4], x[5], x[6]);
        x.head<3>() = x.head<3>().cwiseMax(lower_).cwiseMin(upper_);
        // Coupled added mass can reintroduce another normal velocity at corners.
        for (int pass = 0; pass < 32; ++pass) {
            bool corrected = false;
            for (int axis = 0; axis < 3; ++axis) {
                Eigen::Vector3d normal = Eigen::Vector3d::Zero();
                if (x[axis] <= lower_[axis]) {
                    normal[axis] = 1;
                } else if (axis < 2 && x[axis] >= upper_[axis]) {
                    normal[axis] = -1;
                } else {
                    continue;
                }
                Vector6 jacobian = Vector6::Zero();
                jacobian.head<3>() = q.conjugate() * normal;
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
    Eigen::Vector3d lower_, upper_;
};
} // namespace

struct Plant::Impl {
    explicit Impl(const PlantParameters &p, const BodyState &initial)
        : parameters(p), contacts(p), state(pack(initial)) {
        validate(p);
        contacts.validateInitial(state);
        const auto &b = p.body;
        dynamics.configure(b.mass, b.inertia, b.added_mass);
        dynamics.configureDamping(b.linear_damping, b.quadratic_damping, b.damping_center);
        dynamics.configureHydrostatics(p.pool.water_density, b.displaced_volume, b.buoyancy_center,
                                       b.buoyancy_radii, 9.80665, p.pool.water_level);
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
            actuator_parameters.push_back(a);
            allocation.col(static_cast<Eigen::Index>(i)) << t.direction,
                t.position.cross(t.direction);
        }
        require(allocation.allFinite(), "thruster allocation must be finite");
        actuators.configure(actuator_parameters, p.command_timeout);
        committed_forces = actuators.forces();
    }

    PlantParameters parameters;
    detail::MarineDynamics dynamics;
    detail::ThrusterDynamics actuators;
    PoolContacts contacts;
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

Snapshot Plant::observe() const {
    const auto &p = *impl_;
    return {
        p.generation, p.tick,
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
    const Vector6 wrench = p.allocation * p.committed_forces;
    const auto derivative =
        p.dynamics.derivative(p.state, wrench, p.parameters.pool.current_velocity);
    const auto &body = sample.state.body;
    sample.acceleration_body =
        derivative.segment<3>(7) + body.angular_velocity.cross(body.linear_velocity);
    sample.angular_acceleration_body = derivative.tail<3>();
    sample.acceleration_valid = !p.contacts.touching(p.state);
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
    const auto max_tick = static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max() /
                                                     p.parameters.timestep.count());
    if (ticks > max_tick - p.tick) {
        throw std::overflow_error("requested advance overflows simulation time");
    }
    const double dt = std::chrono::duration<double>(p.parameters.timestep).count();
    try {
        for (std::uint64_t i = 0; i < ticks; ++i) {
            // Midpoint actuator force held during the RK4 body step (operator splitting).
            // One authoritative state commit per integer tick.
            p.actuators.advance(dt / 2);
            const Vector6 wrench = p.allocation * p.actuators.forces();
            auto next = p.dynamics.step(p.state, wrench, dt, p.parameters.pool.current_velocity);
            p.contacts.resolve(next, p.dynamics.inverseMass());
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

Snapshot Plant::reset(const BodyState &initial) {
    auto next = pack(initial);
    impl_->contacts.validateInitial(next);
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
} // namespace robotics::simulation

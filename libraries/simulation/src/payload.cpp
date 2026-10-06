// PayloadDynamics: free flight of launched projectiles as a capsule. FixedAxis only translates, with immersion
// frozen per step; Finned also rotates (RK4 substeps on a raw rotation matrix, re-orthonormalized per step).
#include "nereus/simulation/payload.hpp"

#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace nereus::simulation {
namespace {
constexpr double gravity = 9.80665;
using IntegratedState = Eigen::Matrix<double, 18, 1>;
using RowMatrix = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

void positive(double value, const char *name) {
    if (!std::isfinite(value) || value <= 0.0)
        throw std::invalid_argument(name);
}

void nonnegative(double value, const char *name) {
    if (!std::isfinite(value) || value < 0.0)
        throw std::invalid_argument(name);
}

void validate(const PayloadEnvironment &environment) {
    positive(environment.water_density, "payload water density must be finite and positive");
    if (!environment.water_velocity.allFinite() || !std::isfinite(environment.water_level))
        throw std::invalid_argument("payload environment must be finite");
}

// Mass used for gravity: the displaced water mass when neutrally buoyant, else the configured mass.
double effectiveMass(const PayloadParameters &p, const PayloadEnvironment &e) {
    const double mass = p.neutral_buoyancy ? e.water_density * p.displaced_volume : p.mass;
    positive(mass, "payload effective mass must be finite and positive");
    positive(mass + p.added_mass, "payload total mass must be finite and positive");
    positive(e.water_density * p.displaced_volume, "payload displaced water mass must be finite and positive");
    return mass;
}

// Quadratic drag split into components along and across the body axis (world frame).
Eigen::Vector3d drag(const Eigen::Vector3d &relative, const Eigen::Vector3d &axis, const PayloadParameters &p) {
    const Eigen::Vector3d axial = axis * relative.dot(axis);
    const Eigen::Vector3d lateral = relative - axial;
    return -p.drag_axial * axial.norm() * axial - p.drag_lateral * lateral.norm() * lateral;
}
} // namespace

PayloadDynamics::PayloadDynamics(PayloadParameters parameters) : parameters_(parameters) {
    if (parameters.model != PayloadModel::FixedAxis && parameters.model != PayloadModel::Finned)
        throw std::invalid_argument("unknown payload model");
    positive(parameters.mass, "payload mass must be finite and positive");
    positive(parameters.displaced_volume, "payload volume must be finite and positive");
    positive(parameters.radius, "payload radius must be finite and positive");
    positive(parameters.length, "payload length must be finite and positive");
    if (parameters.length / 2.0 < parameters.radius)
        throw std::invalid_argument("payload length must be at least twice its radius");
    nonnegative(parameters.added_mass, "payload added mass must be finite and nonnegative");
    nonnegative(parameters.drag_axial, "payload axial drag must be finite and nonnegative");
    nonnegative(parameters.drag_lateral, "payload lateral drag must be finite and nonnegative");
    nonnegative(parameters.angular_damping, "payload angular damping must be finite and nonnegative");
    nonnegative(parameters.spring_energy, "payload spring energy must be finite and nonnegative");
    if (!std::isfinite(parameters.center_of_mass) || !std::isfinite(parameters.center_of_buoyancy) ||
        !std::isfinite(parameters.center_of_drag))
        throw std::invalid_argument("payload center offsets must be finite");
}

double PayloadDynamics::launch_speed(const PayloadEnvironment &environment) const {
    validate(environment);
    const double result =
        std::sqrt(2.0 * parameters_.spring_energy / (effectiveMass(parameters_, environment) + parameters_.added_mass));
    if (!std::isfinite(result))
        throw std::overflow_error("payload launch speed overflow");
    return result;
}

Eigen::Vector3d PayloadDynamics::support_extent(const Eigen::Vector3d &axis) const {
    if (!axis.allFinite() || std::abs(axis.norm() - 1.0) > 1e-6)
        throw std::invalid_argument("payload support axis must be finite and unit");
    return Eigen::Vector3d::Constant(parameters_.radius) +
           std::max(0.0, parameters_.length / 2.0 - parameters_.radius) * axis.normalized().cwiseAbs();
}

PayloadState PayloadDynamics::advance(const PayloadState &input, const PayloadEnvironment &e, double dt) const {
    validate(e);
    positive(dt, "payload time step must be finite and positive");
    if (!input.position.allFinite() || !input.velocity.allFinite() || !input.angular_velocity.allFinite() ||
        !input.orientation.coeffs().allFinite() || std::abs(input.orientation.norm() - 1.0) > 1e-6)
        throw std::invalid_argument("payload state must be finite with a unit orientation");

    // Shared setup: the body +X axis and immersion at the start of the step.
    const auto &p = parameters_;
    const double mass = effectiveMass(p, e);
    PayloadState output = input;
    output.orientation.normalize();
    const Eigen::Matrix3d orientation = output.orientation.toRotationMatrix();
    const Eigen::Vector3d initial_axis = orientation.col(0);
    const bool initially_wet = input.position.z() < e.water_level;

    if (p.model == PayloadModel::FixedAxis) {
        // This model intentionally freezes immersion for the complete caller step.
        const auto acceleration = [&](const Eigen::Vector3d &velocity) -> Eigen::Vector3d {
            Eigen::Vector3d force = Eigen::Vector3d::Zero();
            if (initially_wet)
                force = drag(velocity - e.water_velocity, initial_axis, p);
            force.z() += gravity * ((initially_wet ? e.water_density * p.displaced_volume : 0.0) - mass);
            return force / (mass + (initially_wet ? p.added_mass : 0.0));
        };

        // RK4 on velocity; position uses the matching RK4 weights (the force depends only on velocity).
        const Eigen::Vector3d a = acceleration(input.velocity);
        const Eigen::Vector3d b = acceleration(input.velocity + dt * a / 2.0);
        const Eigen::Vector3d c = acceleration(input.velocity + dt * b / 2.0);
        const Eigen::Vector3d d = acceleration(input.velocity + dt * c);
        output.position = input.position + dt * input.velocity + dt * dt * (a + b + c) / 6.0;
        output.velocity = input.velocity + dt * (a + 2.0 * b + 2.0 * c + d) / 6.0;
    } else {
        // Solid-cylinder inertia moved to the COM (parallel axis), scaled by mass plus added mass when wet.
        const double inertia_mass = mass + (initially_wet ? p.added_mass : 0.0);
        const double transverse =
            (p.length * p.length + 3.0 * p.radius * p.radius) / 12.0 + p.center_of_mass * p.center_of_mass;
        const Eigen::Vector3d inertia =
            inertia_mass * Eigen::Vector3d(p.radius * p.radius / 2.0, transverse, transverse);
        if (!inertia.allFinite() || (inertia.array() <= 0.0).any())
            throw std::invalid_argument("payload inertia must be finite and positive");

        // Enough substeps that angular damping stays stable (h no larger than transverse inertia / damping).
        const double requested_steps = std::ceil(dt * p.angular_damping / inertia.y());
        if (!std::isfinite(requested_steps) || requested_steps > 100000.0)
            throw std::invalid_argument("payload step exceeds 100000 angular damping substeps");
        const int steps = std::max(1, static_cast<int>(requested_steps));
        const double h = dt / steps;

        // 18-element state: COM position, velocity, row-major world-from-body rotation, world angular velocity.
        IntegratedState state;
        state.head<3>() = input.position + initial_axis * p.center_of_mass;
        state.segment<3>(3) = input.velocity;
        Eigen::Map<RowMatrix>(state.data() + 6) = orientation;
        state.tail<3>() = input.angular_velocity;

        // Immersion follows the mesh center at every stage; gravity acts at the COM, buoyancy and drag at their
        // axial centers.
        const auto derivative = [&](const IntegratedState &s) -> IntegratedState {
            const Eigen::Vector3d center = s.head<3>();
            const Eigen::Vector3d velocity = s.segment<3>(3);
            const Eigen::Map<const RowMatrix> r(s.data() + 6);
            const Eigen::Vector3d omega = s.tail<3>();
            const Eigen::Vector3d axis = r.col(0);
            const bool wet = (center - axis * p.center_of_mass).z() < e.water_level;
            Eigen::Vector3d force(0.0, 0.0, -mass * gravity);
            Eigen::Vector3d torque = Eigen::Vector3d::Zero();
            if (wet) {
                const Eigen::Vector3d buoyancy(0.0, 0.0, e.water_density * p.displaced_volume * gravity);
                const Eigen::Vector3d arm = axis * (p.center_of_drag - p.center_of_mass);
                const Eigen::Vector3d resistance = drag(velocity + omega.cross(arm) - e.water_velocity, axis, p);
                force += buoyancy + resistance;
                torque = (axis * (p.center_of_buoyancy - p.center_of_mass)).cross(buoyancy) + arm.cross(resistance) -
                         p.angular_damping * (r * ((r.transpose() * omega).cwiseProduct(inertia) / inertia.y()));
            }

            // Euler's equations in body axes, rotated back to world.
            const Eigen::Vector3d body_omega = r.transpose() * omega;
            const Eigen::Vector3d angular_acceleration =
                r *
                (r.transpose() * torque - body_omega.cross(inertia.cwiseProduct(body_omega))).cwiseQuotient(inertia);
            // Rotation matrix rate: skew(omega) * R.
            Eigen::Matrix3d skew;
            skew << 0.0, -omega.z(), omega.y(), omega.z(), 0.0, -omega.x(), -omega.y(), omega.x(), 0.0;
            IntegratedState result;
            result.head<3>() = velocity;
            result.segment<3>(3) = force / (mass + (wet ? p.added_mass : 0.0));
            Eigen::Map<RowMatrix>(result.data() + 6) = skew * r;
            result.tail<3>() = angular_acceleration;
            return result;
        };

        for (int i = 0; i < steps; ++i) {
            const IntegratedState a = derivative(state);
            const IntegratedState b = derivative(state + h * a / 2.0);
            const IntegratedState c = derivative(state + h * b / 2.0);
            const IntegratedState d = derivative(state + h * c);
            state += h * (a + 2.0 * b + 2.0 * c + d) / 6.0;
            if (!state.allFinite())
                throw std::overflow_error("payload integration overflow");
        }
        // Project once per caller step, after all raw-matrix RK4 substeps.
        const Eigen::Matrix3d raw_rotation = Eigen::Map<const RowMatrix>(state.data() + 6);
        const Eigen::JacobiSVD<Eigen::Matrix3d> svd(raw_rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Vector3d signs(1.0, 1.0, (svd.matrixU() * svd.matrixV().transpose()).determinant());
        const Eigen::Matrix3d rotation = svd.matrixU() * signs.asDiagonal() * svd.matrixV().transpose();
        output.position = state.head<3>() - rotation.col(0) * p.center_of_mass;
        output.velocity = state.segment<3>(3);
        output.orientation = Eigen::Quaterniond(rotation).normalized();
        output.angular_velocity = state.tail<3>();
    }

    if (!output.position.allFinite() || !output.velocity.allFinite() || !output.orientation.coeffs().allFinite() ||
        !output.angular_velocity.allFinite())
        throw std::overflow_error("payload integration overflow");
    return output;
}
} // namespace nereus::simulation

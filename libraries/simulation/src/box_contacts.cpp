// Static-box SAT contact response.
#include "detail/box_contacts.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace robotics::simulation::detail {
namespace {
void validate(const std::vector<BoxProxy> &proxies) {
    std::set<std::string> ids;
    if (proxies.size() > 4096)
        throw std::invalid_argument("too many collision proxies");
    for (const auto &proxy : proxies) {
        if (proxy.id.empty() || !ids.insert(proxy.id).second || !proxy.size.allFinite() ||
            (proxy.size.array() <= 0).any() || !proxy.center.allFinite() ||
            !proxy.orientation.coeffs().allFinite() || !std::isfinite(proxy.orientation.norm()) ||
            proxy.orientation.norm() < 1e-10)
            throw std::invalid_argument("invalid collision box identity, size or pose");
    }
}
struct Box {
    Eigen::Vector3d size, center;
    Eigen::Quaterniond orientation;
    Eigen::Matrix3d rotation;
    Eigen::Matrix<double, 3, 8> vertices;
    Box(const BoxProxy &proxy, const Eigen::Vector3d &position = Eigen::Vector3d::Zero(),
        const Eigen::Quaterniond &body = Eigen::Quaterniond::Identity())
        : size(proxy.size), center(position + body.normalized() * proxy.center),
          orientation((body.normalized() * proxy.orientation.normalized()).normalized()),
          rotation(orientation.toRotationMatrix()) {
        for (int i = 0; i < 8; ++i)
            vertices.col(i) = Eigen::Vector3d((i & 4) ? size.x() / 2 : -size.x() / 2,
                                              (i & 2) ? size.y() / 2 : -size.y() / 2,
                                              (i & 1) ? size.z() / 2 : -size.z() / 2);
        vertices = (rotation * vertices + center.replicate(1, 8)).eval();
        if (!vertices.allFinite())
            throw std::invalid_argument("collision box transformed vertices overflow");
    }
    Eigen::Vector3d vertex(const Eigen::Vector3d &axis, bool maximum) const {
        Eigen::Index index = 0;
        const Eigen::Matrix<double, 8, 1> projection = vertices.transpose() * axis;
        if (maximum)
            projection.maxCoeff(&index);
        else
            projection.minCoeff(&index);
        return vertices.col(index);
    }
    bool contains(const Eigen::Vector3d &point) const {
        return ((orientation.conjugate() * (point - center)).cwiseAbs().array() <= size.array() / 2)
            .all();
    }
    Eigen::Vector3d clamp(const Eigen::Vector3d &point) const {
        const Eigen::Vector3d local = orientation.conjugate() * (point - center);
        return orientation * local.cwiseMax(-size / 2).cwiseMin(size / 2) + center;
    }
};
struct Contact {
    bool collided{false};
    double depth{0};
    Eigen::Vector3d normal{Eigen::Vector3d::UnitX()}, point{Eigen::Vector3d::Zero()};
};
Contact collide(const Box &body, const Box &world) {
    Eigen::Matrix<double, 3, 15> axes;
    axes.leftCols<3>() = body.rotation;
    axes.middleCols<3>(3) = world.rotation;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            axes.col(6 + 3 * i + j) = body.rotation.col(i).cross(world.rotation.col(j));
    double depth = std::numeric_limits<double>::infinity();
    Eigen::Vector3d minimum = Eigen::Vector3d::UnitX();
    for (int i = 0; i < axes.cols(); ++i) {
        Eigen::Vector3d axis = axes.col(i);
        const double norm = axis.norm();
        if (norm < 1e-9)
            continue;
        axis /= norm;
        if (body.center.dot(axis) > world.center.dot(axis))
            axis = -axis;
        const double overlap = (body.vertices.transpose() * axis).maxCoeff() -
                               (world.vertices.transpose() * axis).minCoeff();
        if (overlap < depth) {
            depth = overlap;
            minimum = axis;
            if (depth <= 0)
                return {};
        }
    }
    Eigen::Matrix<double, 3, 6> box_axes;
    box_axes << body.rotation, world.rotation;
    Eigen::Index index = 0;
    (box_axes.transpose() * minimum).cwiseAbs().maxCoeff(&index);
    Eigen::Vector3d point = index < 3 ? world.vertex(minimum, false) : body.vertex(minimum, true);
    if (!body.contains(point) || !world.contains(point)) {
        point = body.clamp(point);
        point = world.clamp(point);
        point = body.clamp(point);
        point = world.clamp(point);
        point = body.clamp(point);
    }
    return {true, depth, -minimum, point};
}
} // namespace
struct BoxContacts::Impl {
    std::vector<BoxProxy> body;
    std::vector<Box> world; // Static geometry prepared once, outside integration.
    double restitution = .1, friction = .4;
};
BoxContacts::BoxContacts(std::vector<BoxProxy> body, std::vector<BoxProxy> world,
                         double restitution, double friction)
    : impl_(std::make_unique<Impl>()) {
    validate(body);
    validate(world);
    if (!std::isfinite(restitution) || restitution < 0 || restitution > 1 ||
        !std::isfinite(friction) || friction < 0)
        throw std::invalid_argument("invalid contact restitution/friction");
    impl_->body = std::move(body);
    for (const auto &proxy : impl_->body)
        (void)Box(proxy); // Validate local geometry once before accepting the scene.
    impl_->world.reserve(world.size());
    for (const auto &proxy : world)
        impl_->world.emplace_back(proxy);
    impl_->restitution = restitution;
    impl_->friction = friction;
}
BoxContacts::~BoxContacts() = default;
State13d BoxContacts::resolve(State13d state, const Matrix6d &inverse_mass) const {
    const Eigen::Quaterniond raw_orientation(state[3], state[4], state[5], state[6]);
    const Eigen::Quaterniond q = raw_orientation.normalized();
    for (const auto &proxy : impl_->body) {
        for (const auto &obstacle : impl_->world) {
            const auto contact = collide(Box(proxy, state.head<3>(), raw_orientation), obstacle);
            if (!contact.collided)
                continue;
            const Eigen::Vector3d offset = contact.point - state.head<3>();
            const double speed =
                (q * state.segment<3>(7) + (q * state.tail<3>()).cross(offset)).dot(contact.normal);
            state.head<3>() += contact.normal * contact.depth;
            if (speed >= 0)
                continue;
            const Eigen::Vector3d normal = q.conjugate() * contact.normal;
            const Eigen::Vector3d arm = q.conjugate() * offset;
            Vector6d jacobian;
            jacobian << normal, arm.cross(normal);
            const double effective = jacobian.dot(inverse_mass * jacobian);
            if (effective <= 1e-12)
                continue;
            const double impulse = -(1 + impl_->restitution) * speed / effective;
            state.tail<6>() += inverse_mass * jacobian * impulse;
            const Eigen::Vector3d velocity = state.segment<3>(7) + state.tail<3>().cross(arm);
            Eigen::Vector3d tangent = velocity - normal * velocity.dot(normal);
            if (tangent.norm() > 1e-9) {
                tangent.normalize();
                Vector6d jt;
                jt << tangent, arm.cross(tangent);
                const double jt_inverse = jt.dot(inverse_mass * jt);
                const double friction =
                    std::min(impl_->friction * impulse, jt.dot(state.tail<6>()) / jt_inverse);
                state.tail<6>() -= inverse_mass * jt * friction;
            }
        }
    }
    return state;
}
} // namespace robotics::simulation::detail

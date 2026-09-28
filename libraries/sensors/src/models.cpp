#include "robotics/sensors/models.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace robotics::sensors {
namespace {
void normalizeMount(Mount &mount) {
    if (!mount.position_body.allFinite() || !mount.sensor_to_body.coeffs().allFinite() ||
        std::abs(mount.sensor_to_body.norm() - 1) > 1e-9) {
        throw std::invalid_argument("sensor mount requires a finite position and unit quaternion");
    }
    mount.sensor_to_body.normalize();
}
void validateAxis(const Eigen::Vector3d &axis) {
    if (!axis.allFinite() || std::abs(axis.norm() - 1) > 1e-9) {
        throw std::invalid_argument("sensor axis must be finite and unit length");
    }
}
void validateMotion(const simulation::MotionSample &motion) {
    const auto &body = motion.state.body;
    if (!body.position.allFinite() || !body.orientation.coeffs().allFinite() ||
        std::abs(body.orientation.norm() - 1) > 1e-9 || !body.linear_velocity.allFinite() ||
        !body.angular_velocity.allFinite()) {
        throw std::invalid_argument("sensor input requires finite motion and a unit orientation");
    }
}
// Stable, length-framed FNV-1a; does not depend on implementation-defined std::hash.
std::uint64_t noiseSeed(std::uint64_t root, const std::string &id, const std::string &component) {
    std::uint64_t hash = 14695981039346656037ULL;
    const auto byte = [&hash](unsigned char value) { hash = (hash ^ value) * 1099511628211ULL; };
    const auto integer = [&byte](std::uint64_t value) {
        for (int i = 0; i < 8; ++i) {
            byte(static_cast<unsigned char>(value & 255));
            value >>= 8;
        }
    };
    integer(root);
    for (const auto *text : {&id, &component}) {
        integer(text->size());
        for (const unsigned char character : *text) {
            byte(character);
        }
    }
    return hash;
}
} // namespace

Noise3::Noise3(NoiseParameters parameters) : parameters_(std::move(parameters)) {
    if (!parameters_.bias.allFinite() || !parameters_.white_stddev.allFinite() ||
        !parameters_.walk_stddev.allFinite() || (parameters_.white_stddev.array() < 0).any() ||
        (parameters_.walk_stddev.array() < 0).any() ||
        !parameters_.white_stddev.array().square().isFinite().all() ||
        !parameters_.walk_stddev.array().square().isFinite().all()) {
        throw std::invalid_argument("noise requires finite bias and nonnegative finite variances");
    }
}
void Noise3::reset(std::uint64_t seed, const std::string &id, const std::string &component) {
    random_.seed(noiseSeed(seed, id, component));
    normal_.reset();
    drift_.setZero();
    elapsed_ = 0;
}
Eigen::Vector3d Noise3::sample(double elapsed_seconds) {
    if (!std::isfinite(elapsed_seconds) || elapsed_seconds <= 0) {
        throw std::invalid_argument("sensor acquisition interval must be positive and finite");
    }
    elapsed_ += elapsed_seconds;
    Eigen::Vector3d result;
    for (Eigen::Index i = 0; i < 3; ++i) {
        drift_[i] += parameters_.walk_stddev[i] * std::sqrt(elapsed_seconds) * normal_(random_);
        result[i] =
            parameters_.bias[i] + drift_[i] + parameters_.white_stddev[i] * normal_(random_);
    }
    if (!result.allFinite() || !std::isfinite(elapsed_) || !covariance().allFinite()) {
        throw std::overflow_error("sensor noise overflow");
    }
    return result;
}
Eigen::Matrix3d Noise3::covariance() const {
    return (parameters_.white_stddev.array().square() +
            elapsed_ * parameters_.walk_stddev.array().square())
        .matrix()
        .asDiagonal();
}

Imu::Imu(Mount mount, NoiseParameters acceleration, NoiseParameters gyro)
    : mount_(std::move(mount)), acceleration_(std::move(acceleration)), gyro_(std::move(gyro)) {
    normalizeMount(mount_);
}
void Imu::reset(std::uint64_t seed, const std::string &id) {
    acceleration_.reset(seed, id, "imu.acceleration");
    gyro_.reset(seed, id, "imu.gyro");
}
Measurement<Imu::Reading> Imu::sample(const simulation::MotionSample &motion,
                                      double elapsed_seconds) {
    validateMotion(motion);
    const auto acceleration_noise = acceleration_.sample(elapsed_seconds);
    const auto gyro_noise = gyro_.sample(elapsed_seconds);
    if (!motion.acceleration_valid) {
        return {std::nullopt, "instantaneous acceleration unavailable at contact"};
    }
    if (!motion.acceleration_body.allFinite() || !motion.angular_acceleration_body.allFinite() ||
        !motion.gravity_world.allFinite()) {
        throw std::invalid_argument("IMU requires finite acceleration and gravity");
    }
    const auto &body = motion.state.body;
    const Eigen::Vector3d acceleration =
        motion.acceleration_body + motion.angular_acceleration_body.cross(mount_.position_body) +
        body.angular_velocity.cross(body.angular_velocity.cross(mount_.position_body));
    Reading result;
    result.specific_force =
        mount_.sensor_to_body.conjugate() *
            (acceleration - body.orientation.conjugate() * motion.gravity_world) +
        acceleration_noise;
    result.angular_velocity =
        mount_.sensor_to_body.conjugate() * body.angular_velocity + gyro_noise;
    result.force_covariance = acceleration_.covariance();
    result.angular_covariance = gyro_.covariance();
    if (!result.specific_force.allFinite() || !result.angular_velocity.allFinite()) {
        throw std::overflow_error("IMU measurement overflow");
    }
    return {std::move(result), {}};
}

Fog::Fog(Mount mount, std::vector<Eigen::Vector3d> axes, NoiseParameters gyro)
    : mount_(std::move(mount)), gyro_(std::move(gyro)) {
    normalizeMount(mount_);
    if (axes.empty() || axes.size() > 3) {
        throw std::invalid_argument("FOG requires one to three axes");
    }
    axes_.resize(static_cast<Eigen::Index>(axes.size()), 3);
    for (std::size_t i = 0; i < axes.size(); ++i) {
        validateAxis(axes[i]);
        axes_.row(static_cast<Eigen::Index>(i)) = axes[i].normalized().transpose();
    }
}
void Fog::reset(std::uint64_t seed, const std::string &id) {
    gyro_.reset(seed, id, "fog.gyro");
}
Measurement<Fog::Reading> Fog::sample(const simulation::MotionSample &motion,
                                      double elapsed_seconds) {
    validateMotion(motion);
    const Eigen::Vector3d rate =
        mount_.sensor_to_body.conjugate() * motion.state.body.angular_velocity +
        gyro_.sample(elapsed_seconds);
    Reading result{axes_ * rate, axes_ * gyro_.covariance() * axes_.transpose()};
    if (!result.angular_rates.allFinite() || !result.covariance.allFinite()) {
        throw std::overflow_error("FOG measurement overflow");
    }
    return {std::move(result), {}};
}

PoolBottom::PoolBottom(const simulation::Pool &pool)
    : length_(pool.length), width_(pool.width), floor_(pool.water_level - pool.depth),
      surface_(pool.water_level), origin_xy_(pool.origin_xy_world),
      world_to_pool_(Eigen::Rotation2Dd(-pool.yaw_world).toRotationMatrix()) {
    if (!std::isfinite(length_) || length_ <= 0 || !std::isfinite(width_) || width_ <= 0 ||
        !std::isfinite(pool.depth) || pool.depth <= 0 || !std::isfinite(surface_) ||
        !std::isfinite(floor_) || !origin_xy_.allFinite() || !std::isfinite(pool.yaw_world)) {
        throw std::invalid_argument(
            "pool bottom requires positive finite dimensions and finite level");
    }
    if (!origin_xy_.isZero(0) || pool.yaw_world != 0)
        boundary_tolerance_ = 16 * std::numeric_limits<double>::epsilon() *
                              std::max({1.0, origin_xy_.cwiseAbs().maxCoeff(), length_, width_});
}
std::optional<BottomHit> PoolBottom::operator()(const Eigen::Vector3d &origin_world,
                                                const Eigen::Vector3d &direction_world) const {
    validateAxis(direction_world);
    if (!origin_world.allFinite()) {
        throw std::invalid_argument("bottom query requires a finite origin");
    }
    Eigen::Vector3d origin = origin_world, direction = direction_world;
    origin.head<2>() = world_to_pool_ * (origin_world.head<2>() - origin_xy_);
    direction.head<2>() = world_to_pool_ * direction_world.head<2>();
    const auto inside = [this](const Eigen::Vector3d &point) {
        return point.x() >= -boundary_tolerance_ && point.x() <= length_ + boundary_tolerance_ &&
               point.y() >= -boundary_tolerance_ && point.y() <= width_ + boundary_tolerance_;
    };
    if (!inside(origin) || origin.z() <= floor_ || origin.z() > surface_ || direction.z() >= 0) {
        return std::nullopt;
    }
    const double distance = (floor_ - origin.z()) / direction.z();
    const Eigen::Vector3d hit = origin + distance * direction;
    if (!std::isfinite(distance) || !hit.allFinite() || !inside(hit)) {
        return std::nullopt;
    }
    return BottomHit{distance, Eigen::Vector3d::Zero()};
}
Dvl::Dvl(DvlParameters parameters, BottomQuery bottom)
    : parameters_(std::move(parameters)), bottom_(std::move(bottom)),
      velocity_(parameters_.velocity_noise) {
    normalizeMount(parameters_.mount);
    validateAxis(parameters_.bottom_axis);
    parameters_.bottom_axis.normalize();
    if (!bottom_ || !std::isfinite(parameters_.minimum_range) || parameters_.minimum_range < 0 ||
        !std::isfinite(parameters_.maximum_range) ||
        parameters_.maximum_range <= parameters_.minimum_range) {
        throw std::invalid_argument("DVL requires a bottom query and ordered finite range limits");
    }
}
void Dvl::reset(std::uint64_t seed, const std::string &id) {
    velocity_.reset(seed, id, "dvl.velocity");
}
Measurement<Dvl::Reading> Dvl::sample(const simulation::MotionSample &motion,
                                      double elapsed_seconds) {
    validateMotion(motion);
    const auto noise = velocity_.sample(elapsed_seconds); // Drift continues through loss of lock.
    const auto &body = motion.state.body;
    const auto &mount = parameters_.mount;
    const Eigen::Vector3d origin = body.position + body.orientation * mount.position_body;
    const Eigen::Vector3d direction =
        body.orientation * (mount.sensor_to_body * parameters_.bottom_axis);
    if (!origin.allFinite()) {
        throw std::overflow_error("DVL mount position overflow");
    }
    const auto hit = bottom_(origin, direction);
    if (!hit) {
        return {std::nullopt, "no bottom intersection"};
    }
    if (!std::isfinite(hit->distance) || hit->distance < 0 || !hit->velocity_world.allFinite()) {
        throw std::runtime_error("bottom query returned an invalid hit");
    }
    if (hit->distance < parameters_.minimum_range || hit->distance > parameters_.maximum_range) {
        return {std::nullopt, "bottom out of range"};
    }
    const Eigen::Vector3d velocity_body = body.linear_velocity +
                                          body.angular_velocity.cross(mount.position_body) -
                                          body.orientation.conjugate() * hit->velocity_world;
    Reading result{mount.sensor_to_body.conjugate() * velocity_body + noise, velocity_.covariance(),
                   hit->distance};
    if (!result.bottom_relative_velocity.allFinite()) {
        throw std::overflow_error("DVL velocity overflow");
    }
    return {std::move(result), {}};
}
HydrostaticPressure::HydrostaticPressure(double water_level, double density,
                                         double surface_pressure, double gravity)
    : level_(water_level), surface_pressure_(surface_pressure), gradient_(density * gravity) {
    if (!std::isfinite(level_) || !std::isfinite(surface_pressure_) || surface_pressure_ <= 0 ||
        !std::isfinite(density) || density <= 0 || !std::isfinite(gravity) || gravity <= 0 ||
        !std::isfinite(gradient_) || gradient_ <= 0) {
        throw std::invalid_argument(
            "hydrostatic environment requires finite level and positive pressure/density/gravity");
    }
}
std::optional<double> HydrostaticPressure::operator()(const Eigen::Vector3d &position) const {
    if (!position.allFinite()) {
        throw std::invalid_argument("pressure query requires a finite position");
    }
    const double pressure = surface_pressure_ + gradient_ * std::max(0.0, level_ - position.z());
    if (!std::isfinite(pressure)) {
        throw std::overflow_error("hydrostatic pressure overflow");
    }
    return pressure;
}
Pressure::Pressure(PressureParameters parameters, PressureQuery environment)
    : parameters_(std::move(parameters)), environment_(std::move(environment)),
      noise_(NoiseParameters{{parameters_.noise.bias, 0, 0},
                             {parameters_.noise.white_stddev, 0, 0},
                             {parameters_.noise.walk_stddev, 0, 0}}),
      depth_scale_(1.0 / (parameters_.reference_density * parameters_.reference_gravity)) {
    normalizeMount(parameters_.mount);
    if (!environment_ || !std::isfinite(parameters_.reference_pressure) ||
        parameters_.reference_pressure <= 0 || !std::isfinite(parameters_.reference_density) ||
        parameters_.reference_density <= 0 || !std::isfinite(parameters_.reference_gravity) ||
        parameters_.reference_gravity <= 0 || !std::isfinite(depth_scale_) || depth_scale_ <= 0 ||
        !std::isfinite(parameters_.minimum_pressure) || parameters_.minimum_pressure < 0 ||
        !std::isfinite(parameters_.maximum_pressure) ||
        parameters_.maximum_pressure <= parameters_.minimum_pressure) {
        throw std::invalid_argument(
            "pressure sensor requires a provider, positive calibration and ordered finite limits");
    }
}
void Pressure::reset(std::uint64_t seed, const std::string &id) {
    noise_.reset(seed, id, "pressure");
}
Measurement<Pressure::Reading> Pressure::sample(const simulation::MotionSample &motion,
                                                double elapsed_seconds) {
    validateMotion(motion);
    const double noise = noise_.sample(elapsed_seconds).x();
    const Eigen::Vector3d position =
        motion.state.body.position +
        motion.state.body.orientation * parameters_.mount.position_body;
    if (!position.allFinite()) {
        throw std::overflow_error("pressure mount position overflow");
    }
    const auto ideal = environment_(position);
    if (!ideal) {
        return {std::nullopt, "pressure environment unavailable"};
    }
    if (!std::isfinite(*ideal) || *ideal < 0) {
        throw std::runtime_error("pressure provider returned invalid pressure");
    }
    const double measured = *ideal + noise;
    if (!std::isfinite(measured)) {
        throw std::overflow_error("pressure measurement overflow");
    }
    if (*ideal < parameters_.minimum_pressure || *ideal > parameters_.maximum_pressure ||
        measured < parameters_.minimum_pressure || measured > parameters_.maximum_pressure) {
        return {std::nullopt, "pressure out of range"};
    }
    const double variance = noise_.covariance()(0, 0);
    Reading reading{measured, variance, (measured - parameters_.reference_pressure) * depth_scale_,
                    variance * depth_scale_ * depth_scale_};
    if (!std::isfinite(reading.depth) || !std::isfinite(reading.depth_variance)) {
        throw std::overflow_error("pressure-derived depth overflow");
    }
    return {reading, {}};
}
} // namespace robotics::sensors

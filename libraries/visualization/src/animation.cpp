#include <robotics/visualization/animation.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace robotics::visualization {
namespace {
constexpr double pi = 3.14159265358979323846;
void time(double seconds) {
    if (!std::isfinite(seconds) || seconds < 0)
        throw std::invalid_argument("animation time must be finite and nonnegative");
}
} // namespace
RotorAnimator::RotorAnimator(RotorAnimation configuration) : config_(std::move(configuration)) {
    if (!std::isfinite(config_.timeout) || config_.timeout <= 0 ||
        !std::isfinite(config_.speed_scale) || config_.speed_scale <= 0 ||
        !std::isfinite(config_.curve.deadband) || config_.curve.deadband < 0 ||
        config_.input_count == 0 || config_.input_count > 256 || config_.rotors.empty() ||
        config_.rotors.size() > 256)
        throw std::invalid_argument("invalid rotor animation configuration");
    for (const auto &curve : {config_.curve.forward, config_.curve.reverse})
        for (double value : curve)
            if (!std::isfinite(value))
                throw std::invalid_argument("RPM coefficients must be finite");
    for (const auto &rotor : config_.rotors)
        if (rotor.index >= config_.input_count || (rotor.direction != 1 && rotor.direction != -1))
            throw std::invalid_argument("invalid rotor input index or direction");
    forces_.resize(config_.input_count);
    angles_.resize(config_.rotors.size());
}
double RotorAnimator::rpm(double force) const {
    if (!std::isfinite(force))
        throw std::invalid_argument("rotor force must be finite");
    if (std::abs(force) <= config_.curve.deadband)
        return 0;
    const auto &c = force > 0 ? config_.curve.forward : config_.curve.reverse;
    const double value =
        c[0] + c[1] * force + c[2] * std::tanh(force) + c[3] * std::pow(std::abs(force), .25);
    if (!std::isfinite(value))
        throw std::overflow_error("RPM curve overflow");
    return force > 0 ? std::max(0., value) : std::min(0., value);
}
void RotorAnimator::receive(const std::vector<float> &forces, double seconds) {
    time(seconds);
    if (forces.size() != forces_.size() || !std::isfinite(seconds + config_.timeout))
        throw std::invalid_argument("invalid rotor force packet or expiry");
    for (float force : forces) {
        const double speed = rpm(force) * (2. * pi / 60.) * config_.speed_scale;
        if (!std::isfinite(speed) || !std::isfinite(speed * config_.timeout))
            throw std::overflow_error("rotor speed overflow");
    }
    // Integrate the previous sample before replacing it, even between render frames.
    advance(seconds);
    std::copy(forces.begin(), forces.end(), forces_.begin());
    received_at_ = seconds;
}
void RotorAnimator::advance(double seconds) {
    time(seconds);
    if (!last_step_ || seconds < *last_step_) {
        if (last_step_)
            reset();
        last_step_ = seconds;
        return;
    }
    if (received_at_) {
        const double dt = std::max(0., std::min(seconds, *received_at_ + config_.timeout) -
                                           std::max(*last_step_, *received_at_));
        std::array<double, 256> next{};
        for (std::size_t i = 0; i < angles_.size(); ++i) {
            const auto &rotor = config_.rotors[i];
            const double speed = rpm(forces_[rotor.index]) * (2. * pi / 60.) * config_.speed_scale;
            const double angle = angles_[i] + rotor.direction * speed * dt;
            if (!std::isfinite(angle))
                throw std::overflow_error("rotor phase overflow");
            next[i] = std::remainder(angle, 2. * pi);
        }
        std::copy_n(next.begin(), angles_.size(), angles_.begin());
        if (seconds >= *received_at_ + config_.timeout)
            std::fill(forces_.begin(), forces_.end(), 0.f);
    }
    last_step_ = seconds;
}
void RotorAnimator::reset() {
    std::fill(forces_.begin(), forces_.end(), 0.f);
    std::fill(angles_.begin(), angles_.end(), 0.);
    received_at_.reset();
    last_step_.reset();
}
const std::vector<double> &RotorAnimator::angles() const {
    return angles_;
}
spatial::Pose pivotRotation(const Eigen::Vector3d &pivot, const Eigen::Vector3d &axis,
                            double radians) {
    if (!pivot.allFinite() || !axis.allFinite() || !std::isfinite(radians) || axis.norm() < 1e-6 ||
        !std::isfinite(axis.norm()))
        throw std::invalid_argument("invalid rotation pivot, axis or angle");
    const Eigen::Quaterniond rotation(Eigen::AngleAxisd(radians, axis.normalized()));
    spatial::Pose result{pivot - rotation * pivot, rotation};
    spatial::validate(result);
    return result;
}
void Indicator::command(const Eigen::Vector3f &rgb, IndicatorMode mode, double seconds,
                        double pulse_duration) {
    time(seconds);
    if (!rgb.allFinite())
        throw std::invalid_argument("indicator RGB must be finite");
    switch (mode) {
    case IndicatorMode::Solid:
    case IndicatorMode::SlowFlash:
    case IndicatorMode::FastFlash:
    case IndicatorMode::Breath:
    case IndicatorMode::Pulse:
        break;
    default:
        throw std::invalid_argument("unknown indicator mode");
    }
    const Eigen::Vector3f color = rgb.cwiseMax(0.f).cwiseMin(1.f);
    if (mode == IndicatorMode::Pulse) {
        if (!std::isfinite(pulse_duration) || pulse_duration <= 0 ||
            !std::isfinite(seconds + pulse_duration))
            throw std::invalid_argument("invalid indicator pulse duration");
        pulse_ = color;
        pulse_start_ = seconds;
        pulse_end_ = seconds + pulse_duration;
    } else {
        steady_ = color;
        mode_ = mode;
    }
}
Eigen::Vector3f Indicator::color(double seconds) const {
    time(seconds);
    if (seconds >= pulse_start_ && seconds < pulse_end_)
        return pulse_;
    float brightness = 1;
    switch (mode_) {
    case IndicatorMode::SlowFlash:
        brightness = std::fmod(seconds, 2.) < 1. ? 0.f : 1.f;
        break;
    case IndicatorMode::FastFlash:
        brightness = std::fmod(seconds, .5) < .25 ? 0.f : 1.f;
        break;
    case IndicatorMode::Breath: {
        const double phase = seconds * 2. * pi / 3.;
        if (!std::isfinite(phase))
            throw std::overflow_error("indicator phase overflow");
        brightness = static_cast<float>((std::sin(phase) + 1.) * .5);
        break;
    }
    default:
        break;
    }
    return steady_ * brightness;
}
void Indicator::reset() {
    *this = Indicator{};
}
} // namespace robotics::visualization

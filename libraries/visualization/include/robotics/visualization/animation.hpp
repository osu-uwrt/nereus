#pragma once
#include <robotics/spatial/frames.hpp>

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

namespace robotics::visualization {
// Original empirical signed force (N) -> RPM law, independent of robot identity.
struct RpmCurve {
    std::array<double, 4> forward{}, reverse{};
    double deadband{0.01};
};
struct RotorInput {
    std::size_t index{0};
    double direction{1};
};
struct RotorAnimation {
    RpmCurve curve;
    double timeout{0.5}, speed_scale{1};
    std::size_t input_count{1};
    std::vector<RotorInput> rotors{{0, 1}};
};
// An explicit source-clock integrator, not a render-frame clock. One owner supplies
// realized forces and timestamps; dropped display samples must not drive integration.
// Angles are radians, wrapped by remainder to [-pi,pi]. Rewind clears phase/forces.
class RotorAnimator {
  public:
    explicit RotorAnimator(RotorAnimation configuration);
    double rpm(double force) const;
    void receive(const std::vector<float> &forces, double seconds);
    void advance(double seconds);
    void reset();
    const std::vector<double> &angles() const;

  private:
    RotorAnimation config_;
    std::vector<float> forces_;
    std::vector<double> angles_;
    std::optional<double> received_at_, last_step_;
};
// Mesh coordinates and the pivot/axis use the same local frame. The resulting
// rigid transform preserves the shaft and can be composed under a body mount.
spatial::Pose pivotRotation(const Eigen::Vector3d &pivot, const Eigen::Vector3d &axis,
                            double radians);

enum class IndicatorMode { Solid, SlowFlash, FastFlash, Breath, Pulse };
class Indicator {
  public:
    // RGB is clamped to [0,1]. A pulse overlays the underlying mode until its
    // exclusive end time; changing the underlying mode does not cancel a pulse.
    void command(const Eigen::Vector3f &rgb, IndicatorMode mode, double seconds,
                 double pulse_duration = 0.15);
    Eigen::Vector3f color(double seconds) const;
    void reset(); // Source-generation reset; clears underlying state and pending pulse.

  private:
    Eigen::Vector3f steady_{Eigen::Vector3f::Zero()}, pulse_{Eigen::Vector3f::Zero()};
    IndicatorMode mode_{IndicatorMode::Solid};
    double pulse_start_{0}, pulse_end_{0};
};
// All time arguments are finite, nonnegative seconds in the caller's clock.
// Invalid packets/parameters throw before changing state. No clocks, IO, RNG or GL.
} // namespace robotics::visualization

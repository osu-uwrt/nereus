#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace nereus::simulation {

enum class PayloadModel { FixedAxis, Finned };

struct PayloadParameters {
    PayloadModel model{PayloadModel::FixedAxis};
    double mass{1.0};
    double displaced_volume{0.001};
    bool neutral_buoyancy{false};
    double added_mass{0.0};
    double length{1.0};
    double radius{0.1};
    double drag_axial{0.0};
    double drag_lateral{0.0};
    // Signed distances along local +X from the mesh center, in metres.
    double center_of_mass{0.0};
    double center_of_buoyancy{0.0};
    double center_of_drag{0.0};
    double angular_damping{0.0};
    double spring_energy{0.0};
};

struct PayloadState {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};              // Mesh center, world metres.
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};              // COM velocity, world m/s.
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()}; // World from body.
    Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};      // World rad/s.
};

struct PayloadEnvironment {
    Eigen::Vector3d water_velocity{Eigen::Vector3d::Zero()}; // World m/s.
    double water_density{1000.0};                            // kg/m^3.
    double water_level{0.0};                                 // World Z, metres; gravity is -Z, 9.80665 m/s^2.
};

// Stateless, deterministic propagation. Owns a validated parameter copy; no clocks,
// contacts, scene objects or random sources. Immersion uses the mesh center only.
class PayloadDynamics {
  public:
    explicit PayloadDynamics(PayloadParameters parameters);
    const PayloadParameters &parameters() const noexcept {
        return parameters_;
    }
    // dt must be finite and positive. Inputs are unchanged, including on failure.
    // Finned integration resolves angular damping with up to 100000 RK4 substeps;
    // larger requests are rejected. Orientation must be unit within 1e-6.
    PayloadState advance(const PayloadState &state, const PayloadEnvironment &environment, double dt) const;
    double launch_speed(const PayloadEnvironment &environment) const;
    // Axis must be unit within 1e-6. Returns world-axis capsule support distances.
    Eigen::Vector3d support_extent(const Eigen::Vector3d &axis) const;

  private:
    PayloadParameters parameters_;
};

} // namespace nereus::simulation

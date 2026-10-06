#pragma once
// Fossen-style 6-DOF marine body model (rigid-body + added mass, Coriolis, damping, ellipsoid buoyancy),
// integrated by the plant.

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace nereus::simulation::detail {
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using State13d = Eigen::Matrix<double, 13, 1>;

// SI units. Right-handed ROS body FLU; world Z up. State position is COM,
// quaternion [w,x,y,z] maps body to world, velocities [u,v,w,p,q,r] are BODY
// absolute velocities. No ROS, controller configuration, or wall clock here.
class MarineDynamics {
  public:
    MarineDynamics();
    // Rigid-body mass and inertia about the COM plus the 6x6 added mass; caches the inverse total mass.
    void configure(double mass, const Eigen::Matrix3d &rigid_inertia, const Matrix6d &added_mass);
    // Linear and quadratic damping acting at `center` (body frame, offset from the COM).
    void configureDamping(const Matrix6d &linear, const Vector6d &quadratic,
                          const Eigen::Vector3d &center = Eigen::Vector3d::Zero());
    // Buoyancy of an ellipsoid with semi-axes buoyancy_radii centered at cob (body frame); the immersed
    // fraction of it scales the displaced volume.
    void configureHydrostatics(double density, double volume, const Eigen::Vector3d &cob,
                               const Eigen::Vector3d &buoyancy_radii, double gravity = 9.80665,
                               double water_level = 0.);

    // tau includes applied propulsion, damping and restoring wrenches, at COM.
    // dc/dt is the BODY coordinate derivative of inertial water velocity.
    Vector6d acceleration(const Vector6d &velocity, const Vector6d &relative_velocity, const Vector6d &body_wrench,
                          const Vector6d &current_derivative = Vector6d::Zero()) const;
    // Damping wrench at the COM for a through-water (relative) velocity.
    Vector6d dampingWrench(const Vector6d &relative_velocity) const;
    // Weight plus buoyancy as a body-axes wrench at the COM.
    Vector6d restoringWrench(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation) const;
    // Immersed volume fraction of the buoyancy ellipsoid; optionally its wet center (body frame, from COM).
    double submergedFraction(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation,
                             Eigen::Vector3d *wet_center = nullptr) const;
    // State derivative for a body-frame propulsion wrench at the COM, in water moving at world-frame
    // velocity/acceleration.
    State13d derivative(const State13d &state, const Vector6d &propulsion,
                        const Eigen::Vector3d &water_velocity = Eigen::Vector3d::Zero(),
                        const Eigen::Vector3d &water_acceleration = Eigen::Vector3d::Zero()) const;
    // Fixed-input RK4 step for reproducible model rollouts and controller tests.
    State13d step(const State13d &state, const Vector6d &propulsion, double dt,
                  const Eigen::Vector3d &water_velocity = Eigen::Vector3d::Zero(),
                  const Eigen::Vector3d &water_acceleration = Eigen::Vector3d::Zero()) const;

    // Cross-product matrix, momentum-form Coriolis matrix, and the finite/nonzero-quaternion check.
    static Eigen::Matrix3d skew(const Eigen::Vector3d &vector);
    static Matrix6d coriolis(const Matrix6d &mass, const Vector6d &velocity);
    static void validateState(const State13d &state);

    const Matrix6d &rigidBodyMass() const {
        return rigid_body_mass_;
    }
    const Matrix6d &addedMass() const {
        return added_mass_;
    }
    const Matrix6d &mass() const {
        return mass_;
    }
    const Matrix6d &inverseMass() const {
        return inverse_mass_;
    }
    double waterLevel() const {
        return water_level_;
    }

  private:
    Matrix6d rigid_body_mass_, added_mass_, mass_, inverse_mass_;
    Matrix6d linear_damping_ = Matrix6d::Zero();
    Vector6d quadratic_damping_ = Vector6d::Zero();
    Eigen::Vector3d damping_center_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d cob_ = Eigen::Vector3d::Zero(), radii_{.175, .415, .275};
    double vehicle_mass_ = 1., density_ = 998.2, volume_ = 0., gravity_ = 9.80665, water_level_ = 0.;
};
} // namespace nereus::simulation::detail

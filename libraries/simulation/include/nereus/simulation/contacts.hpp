#pragma once
// Contact geometry for the plant's built-in contact models, and the hook for contact responses owned elsewhere.
#include <Eigen/Geometry>
#include <memory>
#include <string>
#include <vector>

namespace nereus::simulation {
// Geometry is data, independent of robots, pools, tasks and rendering.
// Body proxy poses are COM-local; static proxy poses use simulation-world coordinates.
// One box proxy: full edge lengths (m), centre and orientation.
struct BoxProxy {
    std::string id;
    Eigen::Vector3d size{Eigen::Vector3d::Ones()};
    Eigen::Vector3d center{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

// Disabled: no contacts. SpherePool: a COM-centred sphere against a flat pool's walls and floor. BoxScene: the
// robot's box proxies against static world boxes.
enum class ContactModel { Disabled, SpherePool, BoxScene };

struct ContactParameters {
    ContactModel model{ContactModel::SpherePool};
    std::vector<BoxProxy> body_boxes;     // Robot-owned, ordered compound proxies.
    std::vector<BoxProxy> world_boxes;    // World-owned, ordered static geometry.
    double restitution{.1}, friction{.4}; // BoxScene coefficients; sphere model is frictionless/inelastic.
};

// Extra contact response owned outside the plant (e.g. a task's mesh scenery against the robot's
// mechanisms), applied after the built-in contact model before and after every body step.
// State: position (world), quaternion w,x,y,z (body to world), linear and angular velocity (body
// axes) of the COM. inverse_mass is the 6x6 rigid-body + added-mass inverse in body axes.
class ContactResolver {
  public:
    using State = Eigen::Matrix<double, 13, 1>;
    using Matrix6 = Eigen::Matrix<double, 6, 6>;
    virtual ~ContactResolver() = default;
    virtual State resolve(State state, const Matrix6 &inverse_mass) = 0;
};
} // namespace nereus::simulation

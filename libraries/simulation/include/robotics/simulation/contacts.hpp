#pragma once
#include <Eigen/Geometry>
#include <string>
#include <vector>

namespace robotics::simulation {
// Geometry is data, independent of robots, pools, tasks and rendering.
// Body proxy poses are COM-local; static proxy poses use simulation-world coordinates.
struct BoxProxy {
    std::string id;
    Eigen::Vector3d size{Eigen::Vector3d::Ones()};
    Eigen::Vector3d center{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};
enum class ContactModel { Disabled, SpherePool, BoxScene };
struct ContactParameters {
    ContactModel model{ContactModel::SpherePool};
    std::vector<BoxProxy> body_boxes;  // Robot-owned, ordered compound proxies.
    std::vector<BoxProxy> world_boxes; // World-owned, ordered static geometry.
    double restitution{.1},
        friction{.4}; // BoxScene coefficients; sphere model is frictionless/inelastic.
};
} // namespace robotics::simulation

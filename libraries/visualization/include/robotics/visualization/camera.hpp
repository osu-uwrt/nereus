#pragma once
#include <Eigen/Geometry>

namespace robotics::visualization {
struct Camera {
    Eigen::Vector3d target{Eigen::Vector3d::Zero()};
    double yaw{0.8};
    double pitch{0.6};
    double distance{15.0};
};
struct CameraMatrices {
    Eigen::Matrix4f view, projection;
    Eigen::Vector3f eye;
};
CameraMatrices cameraMatrices(const Camera &camera, double aspect);
// Z-up orbit camera, 45-degree vertical perspective. No simulation camera semantics.
Eigen::Matrix4f viewProjection(const Camera &camera, double aspect);
} // namespace robotics::visualization

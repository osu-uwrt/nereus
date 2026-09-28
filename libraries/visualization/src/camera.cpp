#include <robotics/visualization/camera.hpp>

#include <cmath>
#include <stdexcept>

namespace robotics::visualization {
Eigen::Matrix4f viewProjection(const Camera &camera, double aspect) {
    if (!std::isfinite(aspect) || aspect <= 0 || !camera.target.allFinite() ||
        !std::isfinite(camera.yaw) || !std::isfinite(camera.pitch) ||
        std::abs(camera.pitch) > 1.5 || !std::isfinite(camera.distance) || camera.distance < 0.1 ||
        camera.distance > 10000)
        throw std::invalid_argument("invalid view camera");
    const Eigen::Vector3d back(std::cos(camera.pitch) * std::cos(camera.yaw),
                               std::cos(camera.pitch) * std::sin(camera.yaw),
                               std::sin(camera.pitch));
    const Eigen::Vector3d right = Eigen::Vector3d::UnitZ().cross(back).normalized();
    const Eigen::Vector3d up = back.cross(right);
    const Eigen::Vector3d eye = camera.target + camera.distance * back;
    Eigen::Matrix4d view = Eigen::Matrix4d::Identity();
    view.block<1, 3>(0, 0) = right.transpose();
    view.block<1, 3>(1, 0) = up.transpose();
    view.block<1, 3>(2, 0) = back.transpose();
    view(0, 3) = -right.dot(eye);
    view(1, 3) = -up.dot(eye);
    view(2, 3) = -back.dot(eye);
    constexpr double near = 0.01;
    constexpr double far = 100000.0;
    const double scale = 1.0 / std::tan(3.14159265358979323846 / 8);
    Eigen::Matrix4d projection = Eigen::Matrix4d::Zero();
    projection(0, 0) = scale / aspect;
    projection(1, 1) = scale;
    projection(2, 2) = -(far + near) / (far - near);
    projection(2, 3) = -2 * far * near / (far - near);
    projection(3, 2) = -1;
    return (projection * view).cast<float>();
}
} // namespace robotics::visualization

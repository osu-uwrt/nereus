#pragma once
// Private helpers shared by pack_runtime.cpp, mechanisms.cpp and session.cpp: strict JSON
// accessors with the Python reference's failure behaviour (missing key / wrong shape throws).
#include <robotics/session/scenario.hpp>
#include <robotics/spatial/frames.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

namespace robotics::session::detail {
constexpr double kPi = 3.14159265358979323846;
// CPython math.radians: x * (pi / 180).
inline double radians(double degrees) {
    return degrees * (kPi / 180.0);
}

inline double num(const Json &value, const std::string &what) {
    if (!value.is_number())
        throw std::invalid_argument(what + " must be a number");
    return value.get<double>();
}
inline Eigen::VectorXd vec(const Json &value, std::size_t size, const std::string &what) {
    if (!value.is_array() || value.size() != size)
        throw std::invalid_argument(what + " must contain " + std::to_string(size) + " values");
    Eigen::VectorXd out(static_cast<Eigen::Index>(size));
    for (std::size_t i = 0; i < size; ++i)
        out[static_cast<Eigen::Index>(i)] = num(value[i], what);
    return out;
}
inline Eigen::Vector3d vec3(const Json &value, const std::string &what) {
    return vec(value, 3, what);
}
inline Eigen::Quaterniond quat(const Json &value, const std::string &what) {
    const Eigen::VectorXd q = vec(value, 4, what);
    return Eigen::Quaterniond(q[0], q[1], q[2], q[3]);
}
inline Eigen::MatrixXd matrix(const Json &value, std::size_t n, const std::string &what) {
    if (!value.is_array() || value.size() != n)
        throw std::invalid_argument(what + " must be a " + std::to_string(n) + "x" +
                                    std::to_string(n) + " matrix");
    Eigen::MatrixXd out(static_cast<Eigen::Index>(n), static_cast<Eigen::Index>(n));
    for (std::size_t r = 0; r < n; ++r)
        out.row(static_cast<Eigen::Index>(r)) = vec(value[r], n, what).transpose();
    return out;
}
inline spatial::Pose makePose(const Json &position, const Json &orientation,
                              const std::string &what) {
    return {vec3(position, what), quat(orientation, what)};
}
// Pose.compose of the Python binding: both operands and the result must be valid rigid poses.
inline spatial::Pose composeChecked(const spatial::Pose &parent, const spatial::Pose &child) {
    spatial::validate(parent);
    spatial::validate(child);
    auto result = spatial::compose(parent, child);
    spatial::validate(result);
    return result;
}
inline spatial::Pose inverseChecked(const spatial::Pose &pose) {
    spatial::validate(pose);
    return spatial::inverse(pose);
}
inline Eigen::Vector3d rotate(const Eigen::Quaterniond &q, const Eigen::Vector3d &v) {
    spatial::Pose pose;
    pose.rotation = q;
    spatial::validate(pose);
    return spatial::apply(pose, v);
}
// Python repr of a string, as used in the reference's error messages.
inline std::string repr(const std::string &s) {
    return "'" + s + "'";
}
} // namespace robotics::session::detail

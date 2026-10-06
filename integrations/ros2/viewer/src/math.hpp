// Small glm/Eigen pose helpers shared by the host. Poses are column-major 4x4 glm matrices.
#pragma once
#include <Eigen/Core>
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

namespace nereus::ros_viewer::host {
// Translation then roll/pitch/yaw about X, Y, Z (yaw applied last).
inline glm::mat4 pose(const glm::vec3 &p, const glm::vec3 &rpy = {}) {
    return glm::translate(glm::mat4(1), p) * glm::rotate(glm::mat4(1), rpy.z, glm::vec3(0, 0, 1)) *
           glm::rotate(glm::mat4(1), rpy.y, glm::vec3(0, 1, 0)) * glm::rotate(glm::mat4(1), rpy.x, glm::vec3(1, 0, 0));
}

// Translation then rotation by `q` (normalized first).
inline glm::mat4 poseQuat(const glm::vec3 &p, const glm::quat &q) {
    return glm::translate(glm::mat4(1), p) * glm::mat4_cast(glm::normalize(q));
}

// Non-throwing lookup through nested maps: an undefined node when any level is missing, so
// `as<T>(fallback)` returns the fallback and the node tests false. (YAML::Node() is a defined null,
// which as<std::string> turns into "null".)
inline YAML::Node lookup(const YAML::Node &node, std::initializer_list<const char *> keys) {
    YAML::Node current = node;
    for (const char *key : keys) {
        if (!current.IsDefined() || !current.IsMap())
            return YAML::Node(YAML::NodeType::Undefined);
        YAML::Node next;
        for (const auto &entry : current)
            if (entry.first.Scalar() == key) {
                next.reset(entry.second); // rebind the handle; operator= would overwrite the document
                break;
            }
        if (!next.IsDefined() || next.IsNull())
            return YAML::Node(YAML::NodeType::Undefined);
        current.reset(next);
    }
    return current;
}

// A YAML [x, y, z] sequence (extra elements ignored); throws when shorter or not a sequence.
inline glm::vec3 vec3(const YAML::Node &n) {
    if (!n || !n.IsSequence() || n.size() < 3)
        throw std::runtime_error("Expected an xyz vector, got: " + YAML::Dump(n));
    return {n[0].as<float>(), n[1].as<float>(), n[2].as<float>()};
}

// Pack pose fields: position_m + orientation_wxyz (identity when absent).
inline glm::mat4 packPose(const YAML::Node &item) {
    glm::vec3 p(0);
    glm::quat q(1, 0, 0, 0);
    if (item["position_m"])
        p = vec3(item["position_m"]);
    if (item["orientation_wxyz"]) {
        const auto w = item["orientation_wxyz"];
        if (!w.IsSequence() || w.size() != 4)
            throw std::runtime_error("orientation_wxyz needs four numbers");
        q = glm::quat(w[0].as<float>(), w[1].as<float>(), w[2].as<float>(), w[3].as<float>());
    }
    return poseQuat(p, q);
}

// Both libraries are column-major, so the conversions copy the 16 floats as they are.
inline glm::mat4 fromEigen(const Eigen::Matrix4f &m) {
    return glm::make_mat4(m.data());
}
inline Eigen::Matrix4f toEigen(const glm::mat4 &m) {
    return Eigen::Map<const Eigen::Matrix4f>(glm::value_ptr(m));
}

// Yaw (radians) of the matrix's x axis in the world XY plane.
inline float heading(const glm::mat4 &m) {
    return std::atan2(m[0].y, m[0].x);
}

// True when no element is NaN or infinite.
inline bool finite(const glm::mat4 &m) {
    for (int c = 0; c < 4; ++c)
        for (int r = 0; r < 4; ++r)
            if (!std::isfinite(m[c][r]))
                return false;
    return true;
}
} // namespace nereus::ros_viewer::host

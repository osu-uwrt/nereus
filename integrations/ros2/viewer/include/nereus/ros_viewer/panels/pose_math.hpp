// Pose-gizmo math for the viewer: roll/pitch/yaw poses and their rotation axes, screen-to-world rays, and ray
// picking against planes, axes and screen-space segments.
#pragma once

#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

namespace nereus::ros_viewer {
// RPY uses Rz(yaw) * Ry(pitch) * Rx(roll), matching the RViz command fields.
inline glm::mat4 rpyPose(const glm::vec3 &position, const glm::vec3 &angles) {
    return glm::translate(glm::mat4(1), position) * glm::mat4_cast(glm::quat(angles));
}

// World-frame axis that each RPY angle rotates about (0 roll, 1 pitch, 2 yaw): roll turns about the yawed and
// pitched x axis, pitch about the yawed y axis, yaw about world z.
inline glm::vec3 rpyAxis(const glm::vec3 &angles, int axis) {
    const auto yaw = glm::rotate(glm::mat4(1), angles.z, glm::vec3(0, 0, 1));
    if (axis == 0)
        return glm::vec3(yaw * glm::rotate(glm::mat4(1), angles.y, glm::vec3(0, 1, 0)) * glm::vec4(1, 0, 0, 0));
    if (axis == 1)
        return glm::vec3(yaw * glm::vec4(0, 1, 0, 0));
    return {0, 0, 1};
}

// An in-plane reference perpendicular to each RPY rotation axis.
inline glm::vec3 rpyReference(const glm::vec3 &angles, int axis) {
    const auto rotation = glm::mat3_cast(glm::quat(angles));
    if (axis == 0)
        return rotation[1];
    if (axis == 1)
        return rotation[0];
    return {std::cos(angles.z), std::sin(angles.z), 0};
}

// A world-space ray; `direction` is unit length.
struct Ray {
    glm::vec3 origin, direction;
};

// Distance from `point` to the segment ab (2D, e.g. screen pixels); `fraction` gets the closest point's position
// along it (0 at a, 1 at b).
inline float segmentDistance(const glm::vec2 &point, const glm::vec2 &a, const glm::vec2 &b, float &fraction) {
    const auto delta = b - a;
    const auto squared = glm::dot(delta, delta);
    fraction = squared > 1e-6f ? glm::clamp(glm::dot(point - a, delta) / squared, 0.f, 1.f) : 0.f;
    return glm::length(point - (a + fraction * delta));
}

// The ray through a pixel (top-left origin, y down) of a `size`-pixel view, from the near plane outward.
inline Ray screenRay(const glm::mat4 &viewProjection, const glm::vec2 &pixel, const glm::vec2 &size) {
    const auto inverse = glm::inverse(viewProjection);
    const glm::vec2 ndc(pixel.x / size.x * 2 - 1, 1 - pixel.y / size.y * 2);
    auto near = inverse * glm::vec4(ndc, -1, 1), far = inverse * glm::vec4(ndc, 1, 1);
    return {glm::vec3(near) / near.w, glm::normalize(glm::vec3(far) / far.w - glm::vec3(near) / near.w)};
}

// Where the ray meets the plane through `origin` with `normal`; false when nearly parallel or behind the ray.
inline bool planeHit(const Ray &ray, const glm::vec3 &origin, const glm::vec3 &normal, glm::vec3 &point) {
    const float denominator = glm::dot(ray.direction, normal);
    if (std::abs(denominator) < 1e-4f)
        return false;
    const float distance = glm::dot(origin - ray.origin, normal) / denominator;
    if (distance < 0 || !std::isfinite(distance))
        return false;
    point = ray.origin + ray.direction * distance;
    return true;
}

// The point on the line origin + s * axis (unit axis) closest to the ray, as `position` = s; false when the ray
// is nearly parallel to the axis.
inline bool axisHit(const Ray &ray, const glm::vec3 &origin, const glm::vec3 &axis, float &position) {
    const float dot = glm::dot(ray.direction, axis), denominator = 1 - dot * dot;
    if (denominator < 1e-4f)
        return false;
    const auto offset = ray.origin - origin;
    position = (glm::dot(offset, axis) - dot * glm::dot(offset, ray.direction)) / denominator;
    return std::isfinite(position);
}
} // namespace nereus::ros_viewer

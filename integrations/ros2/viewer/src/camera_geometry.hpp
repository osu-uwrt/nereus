// Pinhole projection of a rectified sensor and the render view for its optical frame.
#pragma once
#include "math.hpp"

namespace robotics::ros_viewer::host {
struct Intrinsics {
    int width = 1920, height = 1200;
    double fx = 0, fy = 0, cx = 0, cy = 0;
    double nearPlane = 0.05, farPlane = 100;
    // Sensor optical frame: x right, y down, z forward. OpenGL looks down -Z with +Y up.
    glm::mat4 projection() const {
        glm::mat4 p(0);
        p[0][0] = float(2 * fx / width);
        p[1][1] = float(2 * fy / height);
        // ROS pixel coordinates index pixel centres; GL viewport coordinates index edges.
        p[2][0] = float(1 - 2 * (cx + .5) / width);
        p[2][1] = float(2 * (cy + .5) / height - 1);
        p[2][2] = float(-(farPlane + nearPlane) / (farPlane - nearPlane));
        p[2][3] = -1;
        p[3][2] = float(-2 * farPlane * nearPlane / (farPlane - nearPlane));
        return p;
    }
    void validate() const {
        if (width < 16 || height < 16 || width > 8192 || height > 8192 || !std::isfinite(fx) || !std::isfinite(fy) ||
            fx <= 0 || fy <= 0 || !std::isfinite(cx) || !std::isfinite(cy) || farPlane <= nearPlane)
            throw std::runtime_error("Invalid camera resolution or intrinsics");
    }
};
struct SensorView {
    glm::vec3 eye{};
    glm::mat4 view{1}, projection{1};
};
// `opticalWorld`: pose of the optical frame in the world (map) frame.
inline SensorView sensorView(const glm::mat4 &opticalWorld, const Intrinsics &k) {
    // GL camera = optical frame with y and z flipped (down -> up, forward -> -Z).
    const glm::mat4 flip = glm::scale(glm::mat4(1), glm::vec3(1, -1, -1));
    return {glm::vec3(opticalWorld[3]), glm::inverse(opticalWorld * flip), k.projection()};
}
inline float linearDepth(float z, float nearPlane = 0.05f, float farPlane = 100.f) {
    return 2 * nearPlane * farPlane / (farPlane + nearPlane - (2 * z - 1) * (farPlane - nearPlane));
}
} // namespace robotics::ros_viewer::host

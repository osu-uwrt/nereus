#pragma once

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <rclcpp/time.hpp>
#include <stdexcept>
#include <string>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/msg/marker.hpp>

namespace nereus::ros_viewer::host {
inline glm::mat4 opticalToLink() {
    glm::mat4 m(0);
    m[0] = {0, -1, 0, 0};
    m[1] = {0, 0, -1, 0};
    m[2] = {1, 0, 0, 0};
    m[3] = {0, 0, 0, 1};
    return m;
}

// Detections describe observations in the world, even if the incoming marker
// requests frame locking. Always use acquisition time, never the current robot
// pose. A zero stamp selects the latest transform only for initial placement.
// For simulated camera observations the caller supplies the saved optical
// render pose, or the simulator TF frame if that acquisition has aged out.
inline bool resolveDetectionPose(const visualization_msgs::msg::Marker &marker, const std::string &fixedFrame,
                                 tf2_ros::Buffer &tf, glm::mat4 &worldPose, const std::string &sourceFrame = "",
                                 const glm::mat4 *acquisitionPose = nullptr) {
    glm::mat4 frame(1);
    const rclcpp::Time stamp(marker.header.stamp);
    if (acquisitionPose && stamp.nanoseconds() != 0) {
        frame = *acquisitionPose;
    } else {
        try {
            const auto t =
                tf.lookupTransform(fixedFrame, sourceFrame.empty() ? marker.header.frame_id : sourceFrame, stamp)
                    .transform;
            const auto &q = t.rotation;
            frame = glm::translate(glm::mat4(1), glm::vec3(t.translation.x, t.translation.y, t.translation.z)) *
                    glm::mat4_cast(glm::normalize(glm::quat(q.w, q.x, q.y, q.z)));
        } catch (const tf2::TransformException &) {
            return false;
        }
    }
    const auto &p = marker.pose.position;
    const auto &q = marker.pose.orientation;
    worldPose = frame * glm::translate(glm::mat4(1), glm::vec3(p.x, p.y, p.z)) *
                glm::mat4_cast(glm::normalize(glm::quat(q.w, q.x, q.y, q.z)));
    return true;
}

// Owned by one observation, not by the camera. Once resolved, its world pose
// is immutable until that observation is replaced or deleted. In particular,
// frame_locked and zero-stamped detections must not follow subsequent TF.
class DetectionPose {
  public:
    bool place(const visualization_msgs::msg::Marker &marker, const std::string &fixedFrame, tf2_ros::Buffer &tf,
               const std::string &sourceFrame = "", const glm::mat4 *acquisitionPose = nullptr) {
        if (!placed_)
            placed_ = resolveDetectionPose(marker, fixedFrame, tf, world_, sourceFrame, acquisitionPose);
        return placed_;
    }

    // RViz-like placement through TF: the marker frame at the marker stamp. While TF has not reached the
    // stamp, keep retrying for `retryWindow` seconds (`age` = time since the observation arrived), then use the
    // latest transform once and flag the result approximate. Zero stamps mean "latest" and are exact.
    bool placeViaTf(const visualization_msgs::msg::Marker &marker, const std::string &fixedFrame, tf2_ros::Buffer &tf,
                    double age, double retryWindow = 0.5) {
        if (placed_)
            return true;
        if (resolveDetectionPose(marker, fixedFrame, tf, world_))
            return placed_ = true;
        if (rclcpp::Time(marker.header.stamp).nanoseconds() == 0 || age < retryWindow)
            return false;
        auto latest = marker;
        latest.header.stamp = builtin_interfaces::msg::Time();
        placed_ = resolveDetectionPose(latest, fixedFrame, tf, world_);
        approximate_ = placed_;
        return placed_;
    }

    const glm::mat4 &world() const {
        return world_;
    }
    bool placed() const {
        return placed_;
    }
    bool approximate() const {
        return approximate_;
    }

  private:
    bool placed_ = false, approximate_ = false;
    glm::mat4 world_{1};
};

// Which placements of a detection to show. PoseSource follows the viewer's pose source.
enum class DetectionMode { PoseSource, Truth, Estimate, Both };
inline DetectionMode parseDetectionMode(const std::string &text) {
    if (text == "pose_source" || text.empty())
        return DetectionMode::PoseSource;
    if (text == "truth")
        return DetectionMode::Truth;
    if (text == "estimate")
        return DetectionMode::Estimate;
    if (text == "both")
        return DetectionMode::Both;
    throw std::invalid_argument("detections placement must be pose_source|truth|estimate|both, got '" + text + "'");
}
struct DetectionShow {
    bool truth = false, estimate = false;
    bool downgraded = false; // truth/both was requested but no simulator truth exists: estimate only
};
// Truth placement needs simulator truth; without it (real robot) truth/both collapse to the estimate.
inline DetectionShow resolveDetectionMode(DetectionMode requested, bool truthAvailable, bool truthActive) {
    DetectionShow show;
    if (!truthAvailable) {
        show.estimate = true;
        show.downgraded = requested == DetectionMode::Truth || requested == DetectionMode::Both;
        return show;
    }
    switch (requested) {
    case DetectionMode::PoseSource:
        (truthActive ? show.truth : show.estimate) = true;
        break;
    case DetectionMode::Truth:
        show.truth = true;
        break;
    case DetectionMode::Estimate:
        show.estimate = true;
        break;
    case DetectionMode::Both:
        show.truth = show.estimate = true;
        break;
    }
    return show;
}

// The simulator publishes only the truth base link, so the optical pose at which a detection was
// acquired is that truth pose at the marker stamp composed with the fixed base-to-camera transform
// from the robot pack. Returns false while TF has not caught up (retry next frame).
inline bool truthAcquisitionPose(tf2_ros::Buffer &tf, const std::string &fixedFrame, const std::string &truthBase,
                                 const rclcpp::Time &stamp, const glm::mat4 &baseToCamera, glm::mat4 &world) {
    try {
        const auto t = tf.lookupTransform(fixedFrame, truthBase, stamp).transform;
        const auto &q = t.rotation;
        world = glm::translate(glm::mat4(1), glm::vec3(t.translation.x, t.translation.y, t.translation.z)) *
                glm::mat4_cast(glm::normalize(glm::quat(q.w, q.x, q.y, q.z))) * baseToCamera;
        return true;
    } catch (const tf2::TransformException &) {
        return false;
    }
}
} // namespace nereus::ros_viewer::host

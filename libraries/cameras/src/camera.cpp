// Camera intrinsics/view matrices and the framebuffer-to-Frame processor.
#include "nereus/cameras/camera.hpp"
#include "depth_noise.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace nereus::cameras {

// Also checks the clipping planes survive the float cast used by the GL matrices.
void Intrinsics::validate() const {
    if (width < 1 || height < 1 || width > 4096 || height > 4096 || !std::isfinite(fx) || !std::isfinite(fy) ||
        fx <= 0 || fy <= 0 || !std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(near_plane) ||
        !std::isfinite(far_plane) || near_plane <= 0 || far_plane <= near_plane ||
        !std::isfinite(static_cast<float>(far_plane)) || static_cast<float>(near_plane) <= 0 ||
        static_cast<float>(far_plane) <= static_cast<float>(near_plane))
        throw std::invalid_argument("invalid rectified camera calibration or clipping planes");
}

Eigen::Matrix4f Intrinsics::projection() const {
    validate();
    // Original camera.hpp projection, including the half-pixel edge/centre correction.
    Eigen::Matrix4f result = Eigen::Matrix4f::Zero();
    result(0, 0) = 2 * fx / width;
    result(1, 1) = 2 * fy / height;
    result(0, 2) = 1 - 2 * (cx + .5) / width;
    result(1, 2) = 2 * (cy + .5) / height - 1;
    result(2, 2) = -(far_plane + near_plane) / (far_plane - near_plane);
    result(2, 3) = -2 * far_plane * near_plane / (far_plane - near_plane);
    result(3, 2) = -1;
    if (!result.allFinite())
        throw std::invalid_argument("camera projection exceeds float precision");
    return result;
}

Eigen::Matrix4f opticalView(const spatial::Pose &world_from_optical) {
    spatial::validate(world_from_optical);
    const auto pose = spatial::inverse(world_from_optical);
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    result.topLeftCorner<3, 3>() = pose.rotation.toRotationMatrix();
    result.topRightCorner<3, 1>() = pose.translation;

    // Optical (+Y down, +Z forward) -> OpenGL eye (+Y up, -Z forward): flip Y and Z.
    result.row(1) *= -1;
    result.row(2) *= -1;
    if (!result.cast<float>().allFinite())
        throw std::invalid_argument("camera view exceeds float precision");
    return result.cast<float>();
}

Processor::Processor(std::uint32_t seed) : random_(seed) {}

void Processor::reset(std::uint32_t seed) {
    random_.seed(seed);
}

Frame Processor::process(const Intrinsics &calibration, const DepthNoise &noise, const std::vector<std::uint8_t> &rgb,
                         const std::vector<float> &depth, bool jpeg, int jpeg_quality) {
    // Validate everything before touching any state.
    calibration.validate();
    noise.validate();
    const auto pixels = static_cast<std::size_t>(calibration.width) * calibration.height;
    if ((!rgb.empty() && rgb.size() != pixels * 3) || (!depth.empty() && depth.size() != pixels) ||
        (jpeg && rgb.empty()) || jpeg_quality < 0 || jpeg_quality > 100)
        throw std::invalid_argument("camera buffers do not match calibration or requested products");

    Frame result;
    result.width = calibration.width;
    result.height = calibration.height;
    result.rgb.resize(rgb.size());
    result.depth.resize(depth.size());
    auto candidate = random_; // Failed processing must not consume the camera's random state.

    // Flip GL's bottom-up rows to top-down and convert depth-buffer values to metres.
    for (int y = 0; y < result.height; ++y) {
        const auto source = static_cast<std::size_t>(result.height - 1 - y) * result.width;
        const auto target = static_cast<std::size_t>(y) * result.width;
        if (!rgb.empty())
            std::copy_n(rgb.begin() + source * 3, result.width * 3, result.rgb.begin() + target * 3);
        if (!depth.empty())
            for (int x = 0; x < result.width; ++x) {
                const float z = depth[source + x];
                const float near = static_cast<float>(calibration.near_plane);
                const float far = static_cast<float>(calibration.far_plane);
                // Depth at (or numerically next to) the far plane is cleared background -> NaN.
                result.depth[target + x] = !std::isfinite(z) || z < 0 || z >= .999999f
                                               ? std::numeric_limits<float>::quiet_NaN()
                                               : 2 * near * far / (far + near - (2 * z - 1) * (far - near));
            }
    }

    if (!depth.empty())
        detail::applyNoise(noise, result.depth, result.width, result.height, candidate);

    // OpenCV encodes BGR, so swap channels before compressing.
    if (jpeg) {
        const cv::Mat pixels(result.height, result.width, CV_8UC3, result.rgb.data());
        cv::Mat bgr;
        cv::cvtColor(pixels, bgr, cv::COLOR_RGB2BGR);
        if (!cv::imencode(".jpg", bgr, result.jpeg, {cv::IMWRITE_JPEG_QUALITY, jpeg_quality}))
            throw std::runtime_error("camera JPEG encoding failed");
    }

    // Commit the random stream only once the whole frame succeeded.
    random_ = candidate;
    return result;
}

} // namespace nereus::cameras

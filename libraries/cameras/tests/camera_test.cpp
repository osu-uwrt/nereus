// Tests for camera projection/view geometry, frame products and the depth noise model.
#include "nereus/cameras/camera.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <opencv2/imgcodecs.hpp>

using namespace nereus::cameras;
namespace spatial = nereus::spatial;
namespace {

// Inverse of the processor's linearization: metric depth -> OpenGL [0, 1] depth-buffer value.
float bufferDepth(double metres, const Intrinsics &k) {
    return static_cast<float>((k.far_plane + k.near_plane - 2 * k.near_plane * k.far_plane / metres) /
                                  (k.far_plane - k.near_plane) * .5 +
                              .5);
}

// Small odd-calibrated camera so tests are fast and catch principal-point mistakes.
Intrinsics small(int width = 64, int height = 48) {
    Intrinsics k;
    k.width = width;
    k.height = height;
    k.fx = 57.3;
    k.fy = 56.2;
    k.cx = 30.1;
    k.cy = 22.7;
    return k;
}

// Projects a world point through the GL matrices and returns its pixel coordinates (integer centres).
Eigen::Vector2d pixel(const Intrinsics &k, const spatial::Pose &pose, Eigen::Vector3d world) {
    const Eigen::Vector4f point(world.x(), world.y(), world.z(), 1);
    const Eigen::Vector4f clip = k.projection() * opticalView(pose) * point;
    return {(clip.x() / clip.w() + 1) * k.width / 2 - .5, (1 - clip.y() / clip.w()) * k.height / 2 - .5};
}

// Element-wise equality that treats NaN == NaN.
void same(const std::vector<float> &a, const std::vector<float> &b) {
    ASSERT_EQ(a.size(), b.size());
    for (std::size_t i = 0; i < a.size(); ++i)
        if (std::isnan(a[i]))
            EXPECT_TRUE(std::isnan(b[i]));
        else
            EXPECT_EQ(a[i], b[i]);
}

} // namespace

TEST(CameraGeometry, PixelCentresMatchPinholeWithArbitraryPrincipalPointAndPose) {
    const auto k = small();
    spatial::Pose pose;
    pose.translation = {3, -2, -1};
    pose.rotation = Eigen::AngleAxisd(.6, Eigen::Vector3d::UnitY());
    for (const Eigen::Vector3d &optical : {Eigen::Vector3d(0, 0, 2), {-.3, .4, 3}, {.4, -.2, 1}}) {
        const auto measured = pixel(k, pose, spatial::apply(pose, optical));
        EXPECT_NEAR(measured.x(), k.fx * optical.x() / optical.z() + k.cx, 2e-5);
        EXPECT_NEAR(measured.y(), k.fy * optical.y() / optical.z() + k.cy, 2e-5);
    }
}

TEST(CameraGeometry, RectifiedStereoDisparityUsesPhysicalBaseline) {
    const auto k = small();
    spatial::Pose left;
    left.translation = {1, 2, -1};
    left.rotation = Eigen::AngleAxisd(-.7, Eigen::Vector3d::UnitZ());
    spatial::Pose offset;
    offset.translation.x() = .05;
    const auto right = spatial::compose(left, offset);
    const auto target = spatial::apply(left, {0.1, .2, 2.5});
    const auto a = pixel(k, left, target), b = pixel(k, right, target);
    EXPECT_NEAR(a.x() - b.x(), k.fx * .05 / 2.5, 2e-5);
    EXPECT_NEAR(a.y(), b.y(), 2e-5);
}

TEST(CameraGeometry, RejectsInvalidCalibrationAndNonrigidPose) {
    auto k = small();
    k.fx = 0;
    EXPECT_THROW(k.projection(), std::invalid_argument);
    k = small();
    k.height = 4097;
    EXPECT_THROW(k.validate(), std::invalid_argument);
    k = small();
    k.far_plane = k.near_plane;
    EXPECT_THROW(k.validate(), std::invalid_argument);
    spatial::Pose pose;
    pose.rotation.coeffs().setZero();
    EXPECT_THROW(opticalView(pose), std::invalid_argument);
}

TEST(CameraProducts, TopDownColorAndOpticalDepthStayRegistered) {
    const auto k = small(2, 2);
    const std::vector<std::uint8_t> bottom_rgb{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    const std::vector<float> bottom_depth{bufferDepth(1, k), bufferDepth(2, k), bufferDepth(3, k), 1};
    DepthNoise noise;
    noise.enabled = false;
    Processor processor;
    const auto frame = processor.process(k, noise, bottom_rgb, bottom_depth);
    EXPECT_EQ(frame.rgb, (std::vector<std::uint8_t>{7, 8, 9, 10, 11, 12, 1, 2, 3, 4, 5, 6}));
    ASSERT_EQ(frame.depth.size(), 4U);
    EXPECT_NEAR(frame.depth[0], 3, 2e-5);
    EXPECT_TRUE(std::isnan(frame.depth[1]));
    EXPECT_NEAR(frame.depth[2], 1, 2e-5);
    EXPECT_NEAR(frame.depth[3], 2, 2e-5);
    EXPECT_EQ(bottom_rgb[0], 1); // Caller inputs remain owned and unchanged.
    EXPECT_EQ(bottom_depth[3], 1);
}

TEST(CameraProducts, RangeAndBackgroundRemainInvalidWithNoiseDisabled) {
    const auto k = small(5, 1);
    DepthNoise noise;
    noise.enabled = false;
    noise.max_range = 4;
    const auto frame =
        Processor().process(k, noise, {}, {bufferDepth(.1, k), bufferDepth(5, k), 1, NAN, bufferDepth(2, k)});
    for (int i = 0; i < 4; ++i)
        EXPECT_TRUE(std::isnan(frame.depth[i]));
    EXPECT_NEAR(frame.depth[4], 2, 2e-5);
    EXPECT_TRUE(frame.rgb.empty());
    EXPECT_TRUE(frame.jpeg.empty());
}

TEST(CameraProducts, JpegKeepsRgbChannelMeaning) {
    const auto k = small(32, 32);
    std::vector<std::uint8_t> rgb(k.width * k.height * 3);
    for (std::size_t i = 0; i < rgb.size(); i += 3) {
        rgb[i] = 200;
        rgb[i + 1] = 50;
        rgb[i + 2] = 20;
    }
    const auto frame = Processor().process(k, {}, rgb, {}, true);
    const auto image = cv::imdecode(frame.jpeg, cv::IMREAD_COLOR);
    ASSERT_EQ(image.cols, k.width);
    ASSERT_EQ(image.rows, k.height);
    const auto color = image.at<cv::Vec3b>(16, 16);
    EXPECT_NEAR(color[0], 20, 3);
    EXPECT_NEAR(color[1], 50, 3);
    EXPECT_NEAR(color[2], 200, 3);
    EXPECT_TRUE(frame.depth.empty());
}

TEST(CameraNoise, ReplayOtherCameraAndRejectedFrameDoNotChangeRandomStream) {
    const auto k = small();
    const std::vector<float> depth(k.width * k.height, bufferDepth(2, k));
    Processor camera(42), reference(42), other(99);
    const auto first = camera.process(k, {}, {}, depth);
    same(first.depth, reference.process(k, {}, {}, depth).depth);
    EXPECT_THROW(camera.process(k, {}, {}, depth, true), std::invalid_argument);
    other.process(k, {}, {}, depth);
    same(camera.process(k, {}, {}, depth).depth, reference.process(k, {}, {}, depth).depth);
    camera.reset(42);
    same(camera.process(k, {}, {}, depth).depth, first.depth);
}

TEST(CameraNoise, MarginalVarianceAndSpatialCorrelationMatchSettings) {
    const auto k = small(400, 400);
    const std::vector<float> depth(k.width * k.height, bufferDepth(2, k));
    DepthNoise noise;
    noise.base_sigma = .01;
    noise.range_sigma = 0;
    noise.dropout = noise.range_dropout = noise.edge_dropout = noise.outliers = 0;
    noise.correlation = .5;
    const auto frame = Processor(7).process(k, noise, {}, depth);
    double sum = 0, squares = 0, adjacent = 0;
    std::size_t pairs = 0;
    for (std::size_t i = 0; i < frame.depth.size(); ++i) {
        const double error = frame.depth[i] - 2;
        ASSERT_TRUE(std::isfinite(error));
        sum += error;
        squares += error * error;
        if (i % k.width) {
            adjacent += error * (frame.depth[i - 1] - 2);
            ++pairs;
        }
    }
    EXPECT_NEAR(sum / frame.depth.size(), 0, .001);
    EXPECT_NEAR(std::sqrt(squares / frame.depth.size()), .01, .001);
    EXPECT_GT(adjacent / pairs, .000025);
    EXPECT_LT(adjacent / pairs, .000075);
}

TEST(CameraNoise, CompleteDropoutAndInvalidParametersAreExplicit) {
    const auto k = small();
    DepthNoise noise;
    noise.dropout = 1;
    const auto frame = Processor().process(k, noise, {}, std::vector<float>(k.width * k.height, bufferDepth(2, k)));
    for (const auto value : frame.depth)
        EXPECT_TRUE(std::isnan(value));
    noise.patch_size = 0;
    EXPECT_THROW(noise.validate(), std::invalid_argument);
    noise.patch_size = 8;
    noise.correlation = 1.1;
    EXPECT_THROW(noise.validate(), std::invalid_argument);
}

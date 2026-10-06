// Camera geometry (pinhole intrinsics, OpenGL projection/view) and the CPU-side processor that
// turns rendered framebuffers into owned RGB, JPEG and noisy metric depth products.
#pragma once
#include <cstdint>
#include <nereus/spatial/frames.hpp>
#include <random>
#include <vector>

namespace nereus::cameras {

// Rectified pinhole camera: +X right, +Y down, +Z forward; pixel centres are integers.
struct Intrinsics {
    // Image size in pixels and focal length / principal point in pixels.
    int width = 1920, height = 1200;
    double fx = 1700, fy = 1700, cx = 959.5, cy = 599.5;

    // OpenGL clipping planes in metres along the optical axis.
    double near_plane = .05, far_plane = 100;

    // Throws std::invalid_argument on non-finite/non-positive values or sizes above 4096.
    void validate() const;

    // OpenGL clip-from-camera matrix whose pixel grid matches fx/fy/cx/cy exactly.
    Eigen::Matrix4f projection() const;
};

// Optical-to-world pose -> OpenGL world-to-camera view. No middleware frame names.
Eigen::Matrix4f opticalView(const spatial::Pose &world_from_optical);

// Original empirical stereo failure model; does not claim to reproduce a learned estimator.
struct DepthNoise {
    bool enabled = true;

    // Gaussian sigma in metres: base_sigma + range_sigma * depth^exponent, plus a constant bias.
    double base_sigma = .002, range_sigma = .0015, exponent = 2;

    // Valid depth window in metres; anything outside becomes NaN.
    double min_range = .15, max_range = 8, bias = 0;

    // Per-pixel dropout probabilities: constant, growing with (depth / max_range)^2, and at depth edges.
    double dropout = .005, range_dropout = .10, edge_dropout = .20;

    // Outlier probability, and the fraction of noise variance shared across patch_size-pixel patches.
    double outliers = .002, correlation = .5;
    int patch_size = 8;

    void validate() const;
};

// One processed camera frame.
struct Frame {
    int width = 0, height = 0;

    // Owned, top-down tightly packed RGB8 and optical-axis depth in metres (NaN invalid).
    // Empty arrays mean the product was not requested, not a valid all-zero image.
    std::vector<std::uint8_t> rgb, jpeg;
    std::vector<float> depth;
};

// One processor per camera, used by one owner; reset restores its random stream.
// Inputs are immutable bottom-up framebuffer RGB8/nonlinear depth; either may be empty.
class Processor {
  public:
    explicit Processor(std::uint32_t seed = 7);
    void reset(std::uint32_t seed);

    // Flips rows to top-down, linearizes depth, applies noise and optionally encodes JPEG.
    // Throws on mismatched buffers; a throwing call leaves the random stream untouched.
    Frame process(const Intrinsics &, const DepthNoise &, const std::vector<std::uint8_t> &rgb,
                  const std::vector<float> &depth, bool jpeg = false, int jpeg_quality = 93);

  private:
    std::mt19937 random_;
};

} // namespace nereus::cameras

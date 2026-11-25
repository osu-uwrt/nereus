// Camera geometry (pinhole intrinsics, OpenGL projection/view) and the CPU-side processor that
// turns rendered framebuffers into owned RGB, JPEG and noisy metric depth products.
#pragma once
#include <cstdint>
#include <nereus/spatial/frames.hpp>
#include <random>
#include <vector>

namespace nereus::cameras {

// Rectified pinhole camera: +X right, +Y down, +Z forward; pixel centers are integers.
struct Intrinsics {
    // Image size in pixels and focal length / principal point in pixels.
    int width = 1920, height = 1200;
    double fx = 1700, fy = 1700, cx = 959.5, cy = 599.5;

    // OpenGL clipping planes in meters along the optical axis.
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

    // Gaussian sigma in meters: base_sigma + range_sigma * depth^exponent, plus a constant bias.
    double base_sigma = .002, range_sigma = .0015, exponent = 2;

    // Valid depth window in meters; anything outside becomes NaN.
    double min_range = .15, max_range = 8, bias = 0;

    // Per-pixel dropout probabilities: constant, growing with (depth / max_range)^2, and at depth edges.
    double dropout = .005, range_dropout = .10, edge_dropout = .20;

    // Outlier probability, and the fraction of noise variance shared across patch_size-pixel patches.
    double outliers = .002, correlation = .5;
    int patch_size = 8;

    // Structured error of a learned stereo matcher (ZED NEURAL*), which is smooth: the depth of a flat surface
    // undulates in waves wave_patch pixels across, with sigma wave_sigma + wave_range_sigma * depth, and each frame
    // keeps wave_correlation of the previous frame's waves. Off (0) by default.
    double wave_sigma = 0, wave_range_sigma = 0, wave_correlation = 0;
    int wave_patch = 100;
    // Per-frame plane error: depth *= 1 + scale + a * x/z + b * y/z with a, b ~ N(0, tilt_sigma) (radians of
    // surface tilt) and scale ~ N(0, scale_sigma), each keeping tilt_correlation of the previous frame's value.
    double tilt_sigma = 0, scale_sigma = 0, tilt_correlation = 0;
    // Foreground fattening: within edge_fatten pixels of a depth edge the nearer surface spreads over the farther.
    int edge_fatten = 0;
    // Edge smearing: within 2 * edge_smooth pixels of a depth jump larger than edge_jump, depth is the valid-weighted
    // Gaussian (sigma edge_smooth px) average, so the matcher's gradual transition replaces a sharp step; each pixel of
    // that band is also dropped with probability edge_band_dropout. Off (0) by default.
    double edge_smooth = 0, edge_band_dropout = 0, edge_jump = .1;

    void validate() const;
};

// One processed camera frame.
struct Frame {
    int width = 0, height = 0;

    // Owned, top-down tightly packed RGB8 and optical-axis depth in meters (NaN invalid).
    // Empty arrays mean the product was not requested, not a valid all-zero image.
    std::vector<std::uint8_t> rgb, jpeg;
    std::vector<float> depth;
};

// Layout of the raw buffers given to Processor::process. The default is the framebuffer's: rows bottom-up and
// nonlinear [0,1] depth. A GPU sensor capture (rendering::SensorFormat) delivers rows top-down and depth already
// in meters (NaN invalid), so the processor skips those steps.
struct RawLayout {
    bool top_down = false, metric_depth = false;
};

// One processor per camera, used by one owner; reset restores its random stream.
// Inputs are RGB8/depth buffers in `layout` (moved from when already top-down); either may be empty.
// Noise carried from frame to frame (structured terms).
struct NoiseState {
    std::vector<float> waves; // coarse wave grid
    int rows = 0, cols = 0;
    double tilt_x = 0, tilt_y = 0, scale = 0;
};

class Processor {
  public:
    explicit Processor(std::uint32_t seed = 7);
    void reset(std::uint32_t seed);

    // Flips rows to top-down and linearizes depth unless `layout` says that is done, applies noise and
    // optionally encodes JPEG. Throws on mismatched buffers; a throwing call leaves the random stream untouched.
    Frame process(const Intrinsics &, const DepthNoise &, std::vector<std::uint8_t> rgb, std::vector<float> depth,
                  bool jpeg = false, int jpeg_quality = 93, RawLayout layout = {});

  private:
    std::mt19937 random_;
    NoiseState state_;
};

} // namespace nereus::cameras

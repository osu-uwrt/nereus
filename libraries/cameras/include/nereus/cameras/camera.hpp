#pragma once
#include <cstdint>
#include <nereus/spatial/frames.hpp>
#include <random>
#include <vector>

namespace nereus::cameras {
// Rectified pinhole camera: +X right, +Y down, +Z forward; pixel centres are integers.
struct Intrinsics {
    int width = 1920, height = 1200;
    double fx = 1700, fy = 1700, cx = 959.5, cy = 599.5;
    double near_plane = .05, far_plane = 100;
    void validate() const;
    Eigen::Matrix4f projection() const;
};
// Optical-to-world pose -> OpenGL world-to-camera view. No middleware frame names.
Eigen::Matrix4f opticalView(const spatial::Pose &world_from_optical);

// Original empirical stereo failure model; does not claim to reproduce a learned estimator.
struct DepthNoise {
    bool enabled = true;
    double base_sigma = .002, range_sigma = .0015, exponent = 2;
    double min_range = .15, max_range = 8, bias = 0;
    double dropout = .005, range_dropout = .10, edge_dropout = .20;
    double outliers = .002, correlation = .5;
    int patch_size = 8;
    void validate() const;
};
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
    Frame process(const Intrinsics &, const DepthNoise &, const std::vector<std::uint8_t> &rgb,
                  const std::vector<float> &depth, bool jpeg = false, int jpeg_quality = 93);

  private:
    std::mt19937 random_;
};
} // namespace nereus::cameras

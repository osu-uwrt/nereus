// Internal entry point for the depth noise model (depth_noise.cpp), used by Processor.
#pragma once
#include "nereus/cameras/camera.hpp"
namespace nereus::cameras::detail {

// Applies `DepthNoise` in place to a top-down metric depth image of width x height pixels.
void applyNoise(const DepthNoise &, std::vector<float> &, int width, int height, std::mt19937 &);
} // namespace nereus::cameras::detail

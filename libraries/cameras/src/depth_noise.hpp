// Internal entry point for the depth noise model (depth_noise.cpp), used by Processor.
#pragma once
#include "nereus/cameras/camera.hpp"
namespace nereus::cameras::detail {

// Applies `DepthNoise` in place to a top-down metric depth image of width x height pixels.
void applyNoise(const DepthNoise &, std::vector<float> &, int width, int height, std::mt19937 &);
// The structured terms (waves, per-frame plane error, edge fattening); `state` carries them between frames.
void applyStructuredNoise(const DepthNoise &, std::vector<float> &, int width, int height, double fx, double fy,
                          double cx, double cy, std::mt19937 &, NoiseState &state);
} // namespace nereus::cameras::detail

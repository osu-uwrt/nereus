#pragma once
#include "nereus/cameras/camera.hpp"
namespace nereus::cameras::detail {
void applyNoise(const DepthNoise &, std::vector<float> &, int width, int height, std::mt19937 &);
}

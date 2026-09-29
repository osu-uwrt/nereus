#pragma once
#include "robotics/cameras/camera.hpp"
namespace robotics::cameras::detail {
void applyNoise(const DepthNoise &, std::vector<float> &, int width, int height, std::mt19937 &);
}

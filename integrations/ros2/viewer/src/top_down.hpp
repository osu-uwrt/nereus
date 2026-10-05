// Top-down images of meshes for the course map: each mesh projected straight down (the highest surface wins),
// coloured by its material (diffuse texture through its UVs, else the base colour) and shaded so tops read
// lighter than slopes. Baked once on the CPU when a scene loads; the course map draws them as plain images.
#pragma once
#include "nereus/rendering/assets.hpp"
#include <cstdint>
#include <filesystem>
#include <functional>
#include <glm/glm.hpp>
#include <memory>
#include <vector>

namespace nereus::ros_viewer::host {
struct TopDownPart {
    std::shared_ptr<const rendering::MeshAsset> mesh;
    glm::mat4 planeFromAsset{1}; // into the image's plane: x right, y up, z towards the viewer
};
struct TopDownImage {
    std::vector<std::uint8_t> rgba; // straight alpha; row 0 is the top (largest y)
    int width = 0, height = 0;
    glm::vec2 low{0}, high{0}; // the plane rectangle the image covers
};
// Reads a diffuse texture (PNG) as top-down RGBA rows; false when it cannot (the part takes its base colour).
using ImageReader = std::function<bool(const std::filesystem::path &, int &width, int &height,
                                       std::vector<unsigned char> &rgba)>;
// The parts' top-down image over [low, high] at `pixelsPerMetre` (capped at `maximumSide` pixels a side).
TopDownImage bakeTopDown(const std::vector<TopDownPart> &parts, glm::vec2 low, glm::vec2 high, float pixelsPerMetre,
                         const ImageReader &readImage = {}, int maximumSide = 4096);
// The image's silhouette grown by `radius` pixels (a square max filter), as a white mask in the alpha channel: the
// halo that keeps thin or pale props visible on a small map.
TopDownImage haloOf(const TopDownImage &image, int radius);
// The parts' extent in the plane (x, y), or an empty box (low > high) when there are none.
std::pair<glm::vec2, glm::vec2> topDownBounds(const std::vector<TopDownPart> &parts);
} // namespace nereus::ros_viewer::host

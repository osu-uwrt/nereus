// Minimal libjpeg decoder for bridge camera previews (sensor_msgs/CompressedImage, JPEG).
#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

namespace robotics::ros_viewer::host {
struct DecodedImage {
    int width = 0, height = 0;
    std::vector<std::uint8_t> rgb; // tightly packed RGB8, top row first
};
// Decodes with the largest DCT scaling (1/1, 1/2, 1/4, 1/8) whose width stays at least `minWidth`,
// which makes card previews of 1920 px frames cheap. Returns false on malformed data.
bool decodeJpeg(const std::uint8_t *data, std::size_t size, int minWidth, DecodedImage &out);
} // namespace robotics::ros_viewer::host

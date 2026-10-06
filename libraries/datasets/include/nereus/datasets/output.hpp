#pragma once
// Sample outputs: label-capture statistics, id maps, image post-processing and encoding, atomic file writes.
#include <nereus/rendering/renderer.hpp>

#include <cstdint>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

namespace nereus::datasets {
// Writes through `<path>.tmp.<pid>` (fsynced) and a rename, so a killed process never leaves a partial file;
// `sync_directory` also fsyncs the parent directory so the rename itself is durable.
void writeAtomic(const std::filesystem::path &, const void *data, std::size_t size, bool sync_directory = false);
inline void writeAtomic(const std::filesystem::path &path, const std::vector<std::uint8_t> &data,
                        bool sync_directory = false) {
    writeAtomic(path, data.data(), data.size(), sync_directory);
}
inline void writeAtomic(const std::filesystem::path &path, const std::string &data, bool sync_directory = false) {
    writeAtomic(path, data.data(), data.size(), sync_directory);
}

// A regular file with at least one byte (the resume check's notion of "already written").
bool nonEmptyFile(const std::filesystem::path &);

// Nonlinear [0, 1] depth -> optical-axis meters (+inf at the far plane / background).
float linearDepth(float nonlinear, float near, float far);

// One label key (id << 8 | part, part != 0) as seen in a label capture. Pixel coordinates are top-down.
struct KeyStats {
    std::int64_t pixels = 0;
    int min_x = 0, min_y = 0, max_x = -1, max_y = -1;
    std::vector<float> depth_m;           // linear depth of every pixel (unsorted)
    bool truncated = false;               // touches the image border
    std::vector<std::int64_t> components; // 8-connected component sizes, largest first (after measureComponents)
    double medianDepth() const;           // reorders a copy
    // §10.1: >= min_visible_px in total and >= 2 components of >= min_fragment_px.
    bool fragmented(std::int64_t min_visible_px, std::int64_t min_fragment_px) const;
};

// Per-key statistics of one label capture plus the near-geometry pixel counts.
struct LabelStats {
    std::map<std::uint32_t, KeyStats> keys;
    std::int64_t near_pixels = 0, counted_pixels = 0; // pixels nearer than near_m / pixels considered
};

// `exclude` (optional, bottom-up like the capture, 1 = skip) masks the robot's own pixels out of the near count.
LabelStats analyzeLabels(const rendering::LabelCapture &, float near_plane, float far_plane, float near_m,
                         const std::vector<std::uint8_t> *exclude = nullptr);

// Fills KeyStats::components of every key (8-connectivity, within its bounding box).
void measureComponents(const rendering::LabelCapture &, LabelStats &);

// Top-down 16-bit id map: pixel key -> table index + 1 (keys absent from the table -> 0).
std::vector<std::uint16_t> idMap(const rendering::LabelCapture &, const std::map<std::uint32_t, std::uint16_t> &table);

// Gaussian blur (sigma px, 0 = off) then Gaussian RGB noise (sigma in 8-bit units, 0 = off) on a packed RGB8
// image, in place. Noise is drawn from `noise_seed` (OpenCV RNG), deterministic for a seed.
void postProcess(std::vector<std::uint8_t> &rgb, int width, int height, double blur_sigma, double noise_sigma,
                 std::uint64_t noise_seed);

// Encoders for top-down packed images.
std::vector<std::uint8_t> encodePngRgb(const std::vector<std::uint8_t> &rgb, int width, int height);
std::vector<std::uint8_t> encodePng16(const std::vector<std::uint16_t> &gray, int width, int height);
std::vector<std::uint16_t> decodePng16(const std::filesystem::path &, int &width, int &height);
} // namespace nereus::datasets

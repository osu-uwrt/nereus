#pragma once
// The planner's job document (format nereus.dataset_job.v1, written by `nereus-dataset plan`): what to render,
// from which scenarios, through which camera, which parts are labelled, how views are accepted and how each
// sample's appearance is randomized. Parsing validates the fields the renderer reads; paths stay absolute.
#include <nereus/session/scenario.hpp>

#include <Eigen/Core>

#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace nereus::datasets {
using Json = session::Json;

// Closed interval [lo, hi]; a JSON number n reads as [n, n].
struct Range {
    double lo = 0, hi = 0;
};

struct Sampler {
    std::string type;                        // approach | overhead | free
    std::vector<std::string> frames{"task"}; // one picked uniformly per attempt
    Eigen::Vector3d offset = Eigen::Vector3d::Zero(), facing = Eigen::Vector3d::UnitX();
    Range range_m{1, 3}, elevation_deg{0, 0}, altitude_m{1, 2}, depth_m{.5, 3};
    double bearing_deg = 0, aim_jitter_deg = 0, roll_deg = 0, pitch_deg = 0, max_aim_pitch_deg = 20, radius_m = 0;
    bool both_sides = false;
    std::optional<double> yaw_deg; // overhead: fixed robot yaw instead of U(0, 360)
};

struct SampleBlock {
    std::optional<std::string> task; // empty: background
    std::int64_t count = 0;
    Sampler sampler;
};

struct CameraSpec {
    std::string sensor;
    int width = 0, height = 0; // 0: the sensor's native resolution
    std::string crop = "center", format = "jpg";
    int jpeg_quality = 92;
    bool robot_visuals = true;
};

struct TexturePart {
    std::filesystem::path texture, mask; // canonical
    std::optional<std::string> task;
    std::map<int, std::string> values; // mask value -> part
};
struct VisualPart {
    std::string task, asset;
    std::optional<std::string> prop, frame, part, indicator;
    std::map<std::string, std::string> materials; // importer material name -> part
    bool split = false;                           // split: connected
};

struct Acceptance {
    double max_range_m = 5, near_m = .2, max_near_fraction = .02;
    std::int64_t min_target_px = 150, max_attempts = 200, background_max_labelled_px = 0;
};

// Every key optional: an absent key leaves the pool's value (no randomization).
struct Randomize {
    std::optional<Range> tint_scale, absorption_scale, scattering;
    std::optional<Range> caustics, exposure, direct_light_scale, ambient_light_scale, sun_azimuth_deg,
        sun_elevation_deg;
    std::optional<Range> time_s;
    double task_yaw_deg = 0, task_offset_m = 0; // rigid jitter of each task about its origin (±, disc radius)
    // Tasks that move together: one drawn yaw/offset per group, about the group's first task's origin (e.g. the
    // table under the octagon). Tasks in no group move alone.
    std::vector<std::vector<std::string>> placement_groups;
    double latched_probability = 0;
    std::optional<Range> noise_sigma, blur_px;
};

struct Job {
    std::string dataset;
    std::uint64_t seed = 0;
    std::filesystem::path output;
    std::vector<std::pair<std::string, std::filesystem::path>> scenarios; // id, resolved scenario JSON
    CameraSpec camera;
    std::vector<TexturePart> textures;
    std::vector<VisualPart> visuals;
    std::set<std::pair<std::string, std::string>> labelled; // (task, part)
    Acceptance acceptance;
    std::vector<SampleBlock> samples;
    Randomize randomize;

    std::int64_t sampleCount() const;
    // Block of global sample k (blocks are contiguous in document order).
    const SampleBlock &block(std::int64_t k) const;
    // `<task or "background">_<k zero-padded to 6>`.
    std::string name(std::int64_t k) const;
};

Job parseJob(const Json &document);
Job loadJob(const std::filesystem::path &path);
} // namespace nereus::datasets

#pragma once
// The planner's job document (format nereus.dataset_job.v1, written by `nereus-dataset plan`): what to render,
// from which scenarios, through which camera, which parts are labeled, how views are accepted and how each
// sample's appearance is randomized. Parsing validates the fields the renderer reads; paths stay absolute.
#include <nereus/session/scenario.hpp>
#include <nereus/spatial/frames.hpp>

#include <Eigen/Core>

#include <array>

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

// How robot poses are drawn for a sample block (geometry in sampling.cpp). Angles in degrees, lengths in meters.
struct Sampler {
    std::string type;                        // approach | overhead | free | fixed
    std::vector<std::string> frames{"task"}; // one picked uniformly per attempt
    Eigen::Vector3d offset = Eigen::Vector3d::Zero(), facing = Eigen::Vector3d::UnitX();
    Range range_m{1, 3}, elevation_deg{0, 0}, altitude_m{1, 2}, depth_m{.5, 3};
    double bearing_deg = 0, aim_jitter_deg = 0, roll_deg = 0, pitch_deg = 0, max_aim_pitch_deg = 20, radius_m = 0;
    bool both_sides = false;
    std::optional<double> yaw_deg; // overhead: fixed robot yaw instead of U(0, 360)
    // fixed (§11.1): this exact robot pose, one acceptance attempt; target_frame is copied to the record.
    spatial::Pose world_from_root;
    std::optional<std::string> target_frame;
};

// `count` consecutive samples of one task (or background) drawn with one sampler.
struct SampleBlock {
    std::optional<std::string> task; // empty: background
    std::int64_t count = 0;
    Sampler sampler;
    std::optional<std::size_t> environment; // forces randomize.environments[i] (no draw consumed)
};

// Output camera: a stereo_camera sensor of the robot pack (its left eye) and the image encoding.
struct CameraSpec {
    std::string sensor;        // sensor id in the resolved robot
    int width = 0, height = 0; // 0: the sensor's native resolution
    std::string crop = "center", format = "jpg";
    int jpeg_quality = 92;
    bool robot_visuals = true;
    int supersample = 1; // RGB anti-aliasing (rendering::Appearance::supersample), 1..4; labels never
};

// parts.textures rule: a part-map PNG aligned with a diffuse texture; mask pixel values name parts.
struct TexturePart {
    std::filesystem::path texture, mask; // canonical
    std::optional<std::string> task;
    std::map<int, std::string> values; // mask value -> part
};

// parts.visuals rule: a task/prop asset whose submeshes (by material, or all of them) carry fixed parts.
struct VisualPart {
    std::string task, asset;
    std::optional<std::string> prop, frame, part, indicator;
    std::map<std::string, std::string> materials; // importer material name -> part
    bool split = false;                           // split: connected
};

// Label-pass acceptance thresholds for a drawn view (§3.2). Pixel counts are at the output resolution.
struct Acceptance {
    double max_range_m = 5, near_m = .2, max_near_fraction = .02;
    std::int64_t min_target_px = 150, max_attempts = 200, background_max_labeled_px = 0;
    // §10.1: reject views where a labeled instance (>= min_visible_px) has >= 2 8-connected components of
    // >= min_fragment_px each ("allow" keeps them).
    bool reject_fragments = true;
    std::int64_t min_fragment_px = 25, min_visible_px = 25;
};

// One appearance environment (§10.2), fully merged by the planner. Every value: fixed or uniform in [lo, hi].
// Water is relative to the pool pack's calibrated values (scales) unless an absolute override is given; an
// absent key keeps the pool's value.
struct Environment {
    std::string id = "default";
    double weight = 1;
    std::optional<Range> tint_scale, absorption_scale, scattering_scale, distance_scale_scale; // tint: per channel
    std::optional<std::array<Range, 3>> tint_rgb, absorption_per_m_rgb;                        // absolute
    std::optional<Range> scattering;                                                           // absolute
    std::optional<Range> caustics, exposure, sun_azimuth_deg, sun_elevation_deg, glare;        // absolute
    std::optional<Range> direct_light_scale, ambient_light_scale;
    std::optional<std::string> profile; // indoor | outdoor (overrides the pool's)
    std::optional<Range> time_s, noise_sigma, blur_px;
};

// Per-sample randomization: appearance environments, task placement jitter, indicator states.
struct Randomize {
    std::vector<Environment> environments{Environment{}}; // >= 1
    bool sweep = false; // environment_mode: weighted (first draw of the sample's stream) | sweep (cycle per block)
    double task_yaw_deg = 0, task_offset_m = 0; // rigid jitter of each task about its origin (±, disc radius)
    // Tasks that move together: one drawn yaw/offset per group, about the group's first task's origin (e.g. the
    // table under the octagon). Tasks in no group move alone.
    std::vector<std::vector<std::string>> placement_groups;
    double latched_probability = 0; // per indicator region, chance of drawing its latched state
};

// The parsed job. Global sample k uses scenario k mod scenarios.size() and the stream Stream(seed, k).
struct Job {
    std::string dataset;
    std::uint64_t seed = 0;
    std::filesystem::path output;
    std::vector<std::pair<std::string, std::filesystem::path>> scenarios; // id, resolved scenario JSON
    CameraSpec camera;
    std::vector<TexturePart> textures;
    std::vector<VisualPart> visuals;
    std::set<std::pair<std::string, std::string>> labeled; // (task, part)
    Acceptance acceptance;
    std::vector<SampleBlock> samples;
    Randomize randomize;

    std::int64_t sampleCount() const; // sum of the block counts
    // Block of global sample k (blocks are contiguous in document order).
    const SampleBlock &block(std::int64_t k) const;
    std::int64_t blockStart(std::int64_t k) const; // global index of the first sample of k's block
    // `<task or "background">_<k zero-padded to 6>`.
    std::string name(std::int64_t k) const;
};

// Throws std::runtime_error naming the offending JSON path on any invalid field.
Job parseJob(const Json &document);
Job loadJob(const std::filesystem::path &path);
} // namespace nereus::datasets

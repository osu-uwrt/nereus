#pragma once
// Scheduled, on-demand acquisition of the robot pack's stereo cameras for a running session:
// capture, processing, scheduling, bounded queues, stale discard and seed reset. No ROS: products are plain buffers and
// numbers for a bridge to publish.
//
// Threading: request()/setDemand()/invalidate() are called by the session owner and never wait for
// rendering or delivery (invalidate/reset wait only for an in-flight delivery callback). One worker per
// camera; GL capture is serialized on one EGL host, CPU processing (depth noise, JPEG) overlaps.
// The delivery callback runs on a worker thread, serialized across cameras.
//
// On-demand: an output (rgb_left, depth_left, rgb_right) is rendered/processed only while some
// consumer declared interest, or every pack output in `always` mode. Depth noise consumes its random
// stream only for frames whose depth was produced, so reproducible runs use `always`.
#include <nereus/cameras/camera.hpp>
#include <nereus/pack_scene/pack_scene.hpp>
#include <nereus/rendering/offscreen.hpp>

#include <array>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace nereus::session_cameras {

// The per-camera products a consumer can ask for.
enum class Output { RgbLeft, DepthLeft, RgbRight };
const char *outputName(Output); // "rgb_left" | "depth_left" | "rgb_right" (pack output ids)
std::optional<Output> outputFromName(const std::string &);

// Stable 32-bit processor seed: first 4 bytes, big-endian, of
// sha256(b"nereus.camera.v1" NUL decimal(seed) NUL sensor id NUL eye).
std::uint32_t deriveSeed(std::uint64_t seed, const std::string &sensor_id, const std::string &eye);
// Plain SHA-256 digest of the string's bytes (used by deriveSeed).
std::array<std::uint8_t, 32> sha256(const std::string &);

// Rectified calibration: K and P row-major; right P carries Tx = -fx * baseline.
struct CameraInfo {
    int width = 0, height = 0;
    std::array<double, 9> k{};
    std::array<double, 12> p{};
};

// Construction options; per-sensor maps are keyed by robot sensor id.
struct Options {
    std::filesystem::path shader_directory;  // default: $NEREUS_SHADER_DIR, then the source-tree shaders
    std::vector<std::string> sensor_ids;     // empty: every enabled stereo_camera
    bool always = false;                     // render every pack output regardless of demand
    std::map<std::string, int> jpeg_quality; // sensor id -> JPEG quality for colour outputs (absent: none)
    bool sensor_noise = true;                // overridden by the scenario's sensor_noise when present
    int supersample = 1;                     // colour/depth anti-aliasing (rendering::Appearance::supersample), 1..4
};

// One capture of one camera. snapshot_time_ns is the session clock of the request, ros_stamp_ns the stamp
// to publish, and revision the invalidate() generation the request belonged to.
struct Products {
    std::string camera;
    std::int64_t snapshot_time_ns = 0, ros_stamp_ns = 0;
    std::uint64_t revision = 0;
    // Top-down frames; an empty rgb/jpeg/depth array means that output was not produced.
    std::optional<cameras::Frame> left, right;
    CameraInfo left_info, right_info;
    double render_ms = 0, process_ms = 0; // GL capture wall time (incl. waiting for the GL lock), processing
};

// Per-camera counters. requested counts schedule slots reached; dropped_pending counts queue overflow;
// discarded_stale counts work dropped by invalidate()/close(). *_ns fields are accumulated wall times.
struct CameraStats {
    std::uint64_t requested = 0, skipped_no_demand = 0, captured = 0, delivered = 0, dropped_pending = 0,
                  discarded_stale = 0;
    std::uint64_t render_ns = 0, process_ns = 0, capture_wall_ns = 0;
};

// Owns the selected cameras, one offscreen GL host shared by all of them, and one worker thread per camera.
class SessionCameras {
  public:
    using Callback = std::function<void(Products &&)>;
    // Builds (strict) its own pack scene unless one is supplied. Throws on unusable pack content.
    SessionCameras(const session::ResolvedScenario &, Options = {},
                   std::shared_ptr<const pack_scene::PackScene> scene = nullptr);
    ~SessionCameras();
    SessionCameras(const SessionCameras &) = delete;
    SessionCameras &operator=(const SessionCameras &) = delete;

    // Starts one worker per camera; products go to `deliver` (worker thread, serialized).
    void start(Callback deliver);

    // Physics-owner call after a tick: queues a capture for every camera whose period elapsed. Never
    // waits. world_from_root is the robot frame root pose; dynamic/overrides are copied.
    void request(std::int64_t snapshot_time_ns, std::int64_t ros_stamp_ns, const spatial::Pose &world_from_root,
                 const std::vector<rendering::Instance> &dynamic = {},
                 const std::vector<pack_scene::RobotOverride> &overrides = {},
                 const std::map<std::string, bool> &latched = {});

    // Interest of one consumer in one camera output (any consumer true enables it).
    void setDemand(const std::string &camera, Output, bool wanted, const std::string &consumer = "default");
    void setAlways(bool);
    void setJpegQuality(const std::string &camera, std::optional<int> quality);

    // Discards pre-placement work (in-flight results are dropped at delivery). With a seed this is a
    // full reset: schedules restart and every eye's noise stream is reseeded once no capture is active.
    void invalidate(std::optional<std::uint64_t> seed = std::nullopt);
    void close(); // joins workers; idempotent; safe before destruction

    std::vector<std::string> cameraIds() const;
    bool hasOutput(const std::string &camera, Output) const;
    double periodSeconds(const std::string &camera) const;
    // Rectified calibration of one eye ("left" or "right").
    CameraInfo info(const std::string &camera, const std::string &eye = "left") const;
    // Snapshot of the per-camera counters.
    std::map<std::string, CameraStats> stats() const;
    const pack_scene::PackScene &scene() const {
        return *scene_;
    }
    // JSON summary of the configuration (cameras, depth noise, seeds, scene) for diagnostics.
    session::Json describe() const;

  private:
    // One queued capture: the request's snapshot plus the outputs wanted when it was queued.
    struct Job {
        std::int64_t native_ns = 0, ros_ns = 0;
        std::uint64_t revision = 0;
        spatial::Pose root;
        std::shared_ptr<const std::vector<rendering::Instance>> dynamic;
        std::shared_ptr<const std::vector<pack_scene::RobotOverride>> overrides;
        std::map<std::string, bool> latched; // indicator region -> latch state
        bool rgb_left = false, depth_left = false, rgb_right = false;
        std::optional<int> jpeg_quality;
    };

    // Pack configuration of one camera plus its runtime state.
    struct Camera {
        // Sensor id and the pack frame ids of the eyes; right_frame is empty when the pack gives none (the
        // right eye then reuses the left eye's pose).
        std::string id, frame, right_frame;
        // Capture schedule and the bounded pending queue's policy.
        std::int64_t period_ns = 0;
        std::size_t capacity = 1;
        bool fail_on_overflow = false;

        double baseline_m = 0;
        std::set<Output> outputs;
        std::array<cameras::Intrinsics, 2> intrinsics; // left, right
        cameras::DepthNoise noise;
        spatial::Pose left_eye, right_eye; // root_from_eye
        // Per-eye processors (they own the depth-noise random state) and the seeds they were reset with.
        std::array<cameras::Processor, 2> processors;
        std::array<std::uint32_t, 2> seeds{};

        // Runtime state: consumer demand, queued jobs, next scheduled capture time and counters.
        std::map<Output, std::set<std::string>> demand;
        std::optional<int> jpeg_quality;
        std::deque<Job> pending;
        std::int64_t next_ns = 0;
        CameraStats stats;
    };

    std::shared_ptr<const pack_scene::PackScene> scene_;
    Options options_;
    std::uint64_t seed_ = 0;
    bool sensor_noise_ = true;
    std::map<std::string, std::unique_ptr<Camera>> cameras_;

    // One GL host shared by all workers: gl_mutex_ serializes captures on it; publication_mutex_
    // serializes delivery callbacks with invalidate().
    std::unique_ptr<rendering::OffscreenRenderer> host_;
    std::mutex gl_mutex_, publication_mutex_;

    mutable std::mutex mutex_; // guards everything below and the mutable Camera fields
    std::condition_variable condition_;
    // Incremented by invalidate()/close(); work from an older revision is discarded at delivery.
    std::uint64_t revision_ = 0;
    // Pending reseed: applied by a worker once no capture is in flight (resetting_ while it runs).
    std::optional<std::uint64_t> reset_seed_;
    bool resetting_ = false, stopping_ = false, always_ = false;
    // Number of captures currently in flight.
    int active_ = 0;
    // First worker exception; rethrown from request() and close().
    std::exception_ptr failure_;
    std::vector<std::thread> threads_;
    Callback deliver_;

    // Worker loop for one camera: waits for jobs or a reseed, captures, then delivers if still current.
    void run(Camera &);
    void reseedAll(std::uint64_t seed);
    // Renders and processes one job for one camera (GL work under gl_mutex_).
    Products capture(Camera &, const Job &);
    // Counts and drops every queued job (caller holds mutex_).
    void discardPending();
    void raiseFailure() const;
    Camera &camera(const std::string &) const;
};
} // namespace nereus::session_cameras

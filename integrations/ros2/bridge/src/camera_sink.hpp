#pragma once
// Hook for camera acquisition (the cpp_cameras runtime plugs in here; until then the bridge runs
// with --no-cameras). The bridge never renders: it asks the sink to acquire at each tick and
// publishes whatever encoded images the sink hands back, from any thread.
#include "ros_types.hpp"

#include <nereus/session/scenario.hpp>
#include <nereus/simulation/plant.hpp>

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace nereus::ros_bridge {

// One finished camera output ready to publish.
struct EncodedImage {
    std::string stream;               // bridge stream id (image or camera_info)
    std::shared_ptr<Message> message; // already filled; the node serializes and publishes it
};

// Camera acquisition runtime the node drives (session_camera_sink.cpp implements it).
class CameraSink {
  public:
    virtual ~CameraSink() = default;
    // Bridge stream ids this sink produces; the core skips compiling them and the node creates
    // their publishers.
    virtual std::vector<std::string> streamIds() const = 0;
    // Called once before stepping; `publish` is thread-safe.
    virtual void start(std::function<void(EncodedImage)> publish) = 0;
    // Non-blocking request to acquire for the state at ROS stamp `ros_ns`.
    virtual void acquire(const simulation::Snapshot &snapshot, std::int64_t ros_ns) = 0;
    // Consumer interest in one of streamIds() (the node derives it from subscriber counts and
    // calls this when it changes); output-less streams such as camera_info ignore it.
    virtual void setDemand(const std::string &stream, bool wanted) = 0;
    // Discard pending work after a placement or reset; `seed` set for a full reset.
    virtual void invalidate(std::optional<std::uint64_t> seed) = 0;
    virtual void close() = 0;
    virtual Json describe() const = 0; // execution.json "cameras"
    virtual Json stats() const = 0;    // summary.json "camera_stats"
};

struct CameraSinkOptions {
    bool always{false}; // render every pack output regardless of subscribers (reproducible runs)
    int supersample{1}; // anti-aliasing factor, 1..4 (session_cameras::Options::supersample)
};

class SessionPort;
// Builds the camera runtime for the given camera sensor ids of a scenario; nullptr when this
// build has no camera acquisition (the bridge then requires --no-cameras). Throws
// MappingError/BridgeError for unusable camera stream declarations.
std::unique_ptr<CameraSink> createCameraSink(const session::ResolvedScenario &scenario,
                                             const std::vector<std::string> &camera_ids, SessionPort &session,
                                             const CameraSinkOptions &options);

} // namespace nereus::ros_bridge

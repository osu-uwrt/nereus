#pragma once
// Viewer-facing state endpoints and the named encoders that turn them into messages. Streams declare `native: state:<name>` and `format: json | marker_array` plus
// `options`; everything is driven by pack data (frames, assets, mechanism types, indicator
// colors), never by robot, task or year names.
#include "ros_types.hpp"
#include "session_port.hpp"

#include <robotics/session/scenario.hpp>

#include <functional>
#include <optional>

namespace robotics::ros_bridge {

struct VisualContext {
    SessionPort &session;
    const session::ResolvedScenario &resolved;
    std::string world_frame;
    std::function<std::int64_t()> clock_ns;
    std::function<spatial::Pose(const simulation::BodyState &)> reference_pose;
    std::function<std::string()> scenario_json;
};

struct FormatStream {
    bool timed{false};
    // json event streams: message from text. Timed streams: `state` builds the current message.
    std::function<std::shared_ptr<Message>(const std::string &text)> encode_json;
    std::function<std::shared_ptr<Message>()> state;
};

// Endpoint -> (format it must use, "timed" | "event"); nullopt for an unknown endpoint.
struct EndpointKind {
    std::string format, mode;
};
std::optional<EndpointKind> formatEndpoint(const std::string &endpoint);

// Validates one `format` stream and returns its encoders. Throws BridgeError.
FormatStream compileFormat(VisualContext &context, const Json &stream,
                           const std::shared_ptr<const MessageType> &type, const std::string &where);

} // namespace robotics::ros_bridge

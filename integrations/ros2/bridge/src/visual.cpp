// Format streams (json and marker_array) for the viewer: option validation and the encoders.

#include "visual.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cmath>
#include <set>

namespace nereus::ros_bridge {
namespace {
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

// "['a', 'b']" for error messages.
std::string listRepr(const std::set<std::string> &items) {
    std::string out = "[";
    bool first = true;
    for (const auto &item : items) {
        out += (first ? "" : ", ") + repr(item);
        first = false;
    }
    return out + "]";
}

// Throws unless `options` has every required key and nothing beyond required + optional.
void checkOptions(const Json &options, const std::string &where, const std::set<std::string> &required,
                  const std::set<std::string> &optional = {}) {
    std::set<std::string> unknown, missing;
    for (const auto &item : options.items())
        if (!required.count(item.key()) && !optional.count(item.key()))
            unknown.insert(item.key());
    for (const auto &key : required)
        if (!options.contains(key))
            missing.insert(key);
    if (!unknown.empty() || !missing.empty())
        throw BridgeError(where + "/options: keys must be " + listRepr(required) + " plus optional " +
                          listRepr(optional) + " (unknown " + listRepr(unknown) + ", missing " + listRepr(missing) +
                          ")");
}

// Exactly `count` finite numbers (booleans rejected).
std::vector<double> numbers(const Json &value, std::size_t count, const std::string &where) {
    std::vector<double> out;
    bool ok = value.is_array() && value.size() == count;
    if (ok)
        for (const auto &item : value) {
            if (!item.is_number() || item.is_boolean() || !std::isfinite(item.get<double>())) {
                ok = false;
                break;
            }
            out.push_back(item.get<double>());
        }
    if (!ok)
        throw BridgeError(where + ": must be " + std::to_string(count) + " finite numbers");
    return out;
}

// One marker to emit: a sphere or a mesh, or a DELETEALL when `delete_all` is set. Orientation is
// w, x, y, z; color r, g, b, a.
struct Item {
    bool delete_all{false};
    std::string ns, frame_id, mesh;
    int id{0};
    bool mesh_type{false}, embedded_materials{false};
    std::array<double, 3> position{}, scale{};
    std::array<double, 4> orientation{1, 0, 0, 0}, color{};
};

// MarkerArray of `items`, all stamped `stamp` (ROS ns).
std::shared_ptr<Message> encodeMarkers(const std::shared_ptr<const MessageType> &type, std::int64_t stamp,
                                       const std::vector<Item> &items) {
    auto message = std::make_shared<Message>(type);
    auto &array = *static_cast<MarkerArray *>(message->data());
    for (const auto &item : items) {
        Marker marker;
        if (item.delete_all) {
            marker.action = Marker::DELETEALL;
            array.markers.push_back(marker);
            continue;
        }

        marker.header.frame_id = item.frame_id;
        // Split ns into sec/nanosec with nanosec in [0, 1e9).
        std::int64_t sec = stamp / 1000000000, rest = stamp % 1000000000;
        if (rest < 0) {
            rest += 1000000000;
            --sec;
        }

        marker.header.stamp.sec = static_cast<std::int32_t>(sec);
        marker.header.stamp.nanosec = static_cast<std::uint32_t>(rest);
        marker.ns = item.ns;
        marker.id = item.id;
        marker.type = item.mesh_type ? Marker::MESH_RESOURCE : Marker::SPHERE;
        marker.action = Marker::ADD;
        marker.pose.position.x = item.position[0];
        marker.pose.position.y = item.position[1];
        marker.pose.position.z = item.position[2];
        marker.pose.orientation.w = item.orientation[0];
        marker.pose.orientation.x = item.orientation[1];
        marker.pose.orientation.y = item.orientation[2];
        marker.pose.orientation.z = item.orientation[3];
        marker.scale.x = item.scale[0];
        marker.scale.y = item.scale[1];
        marker.scale.z = item.scale[2];
        marker.color.r = static_cast<float>(item.color[0]);
        marker.color.g = static_cast<float>(item.color[1]);
        marker.color.b = static_cast<float>(item.color[2]);
        marker.color.a = static_cast<float>(item.color[3]);

        if (item.mesh_type) {
            marker.mesh_resource = item.mesh;
            marker.mesh_use_embedded_materials = item.embedded_materials;
        }
        array.markers.push_back(marker);
    }
    return message;
}

// Fixed-size arrays from validated vectors/JSON; quaternions are w, x, y, z.
std::array<double, 3> triple(const std::vector<double> &v) {
    return {v[0], v[1], v[2]};
}

std::array<double, 4> quad(const std::vector<double> &v) {
    return {v[0], v[1], v[2], v[3]};
}

std::array<double, 4> wxyz(const Eigen::Quaterniond &q) {
    return {q.w(), q.x(), q.y(), q.z()};
}

std::array<double, 4> jsonQuad(const Json &j) {
    return {j.at(0).get<double>(), j.at(1).get<double>(), j.at(2).get<double>(), j.at(3).get<double>()};
}

std::array<double, 3> jsonTriple(const Json &j) {
    return {j.at(0).get<double>(), j.at(1).get<double>(), j.at(2).get<double>()};
}

// Produces the current marker items when a timed stream fires.
using ItemSource = std::function<std::vector<Item>()>;

// file:// URI of an asset of the given pack role ("robot", "tasks").
std::string assetUri(VisualContext &context, const std::string &role, const std::string &asset,
                     const std::string &where) {
    const auto pack = context.resolved.asset_paths.find(role);
    if (pack != context.resolved.asset_paths.end()) {
        const auto found = pack->second.find(asset);
        if (found != pack->second.end())
            return "file://" + found->second.string();
    }
    throw BridgeError(where + ": asset " + repr(asset) + " is not present in the " + role + " pack");
}

// Task indicator spheres, colored by their latched or initial state from options.colors (every color an
// indicator can take is checked up front).
ItemSource indicatorItems(VisualContext &context, const Json &options, const std::string &frame_id,
                          const std::string &where) {
    checkOptions(options, where, {"shape", "scale_m", "colors"});
    if (options["shape"] != "sphere")
        throw BridgeError(where + "/options/shape: only 'sphere' is supported");
    const auto scale = triple(numbers(options["scale_m"], 3, where + "/options/scale_m"));
    std::map<std::string, std::array<double, 4>> colors;
    for (const auto &[name, value] : options["colors"].items())
        colors[name] = quad(numbers(value, 4, where + "/options/colors/" + name));
    const Json initial = context.session.indicators();
    for (const auto &item : initial) {
        std::set<std::string> missing;
        for (const auto &color : item.at("colors"))
            if (!colors.count(color.get<std::string>()))
                missing.insert(color.get<std::string>());
        if (!missing.empty())
            throw BridgeError(where + "/options/colors: no color for " + listRepr(missing) + " of indicator " +
                              repr(item.at("region").get<std::string>()));
    }

    SessionPort *session = &context.session;
    return [session, scale, colors, frame_id]() {
        std::vector<Item> result;
        int index = 0;
        for (const auto &item : session->indicators()) {
            const auto &state = item.at("colors").at(item.at("latched").get<bool>() ? "latched" : "initial");
            Item out;
            out.ns = item.at("region").get<std::string>();
            out.id = index++;
            out.frame_id = frame_id;
            out.position = jsonTriple(item.at("position_m"));
            out.orientation = jsonQuad(item.at("orientation_wxyz"));
            out.scale = scale;
            out.color = colors.at(state.get<std::string>());
            result.push_back(out);
        }
        return result;
    };
}

// Task prop meshes in the world frame; held props are drawn relative to the robot reference pose in
// held_frame_id so they follow the robot.
ItemSource propItems(VisualContext &context, const Json &options, const std::string &frame_id,
                     const std::string &where) {
    checkOptions(options, where, {"held_frame_id"}, {"scale_m", "color"});
    if (!options["held_frame_id"].is_string() || options["held_frame_id"].get<std::string>().empty())
        throw BridgeError(where + "/options/held_frame_id: must be a ROS frame name");
    const std::string held_frame = options["held_frame_id"];
    const auto scale = triple(numbers(options.value("scale_m", Json::array({1, 1, 1})), 3, where + "/options/scale_m"));
    const auto color = quad(numbers(options.value("color", Json::array({0, 0, 0, 1})), 4, where + "/options/color"));

    // Mesh URI per (task, prop) for rigid-body props with a visual asset; other props are not drawn.
    auto uris = std::make_shared<std::map<std::pair<std::string, std::string>, std::string>>();
    for (const auto &task : context.resolved.task_definitions)
        for (const auto &prop : task.at("props")) {
            if (prop.at("type") != "rigid_body")
                continue;
            const auto &parameters = prop.at("parameters");
            if (!parameters.contains("visual_asset") || parameters["visual_asset"].is_null())
                continue;
            (*uris)[{task.at("id").get<std::string>(), prop.at("id").get<std::string>()}] =
                assetUri(context, "tasks", parameters["visual_asset"].get<std::string>(),
                         where + ": prop " + repr(prop.at("id").get<std::string>()));
        }

    VisualContext *ctx = &context;
    return [ctx, uris, scale, color, frame_id, held_frame]() {
        std::vector<Item> result;
        std::optional<spatial::Pose> reference;
        int index = 0;
        for (const auto &prop : ctx->session.propVisuals()) {
            const int this_index = index++;
            const auto uri = uris->find({prop.task, prop.id});
            if (uri == uris->end())
                continue;
            Item item;
            item.ns = prop.id;
            item.id = this_index;
            item.mesh_type = true;
            item.frame_id = frame_id;
            item.scale = scale;
            item.color = color;
            item.mesh = uri->second;
            item.embedded_materials = true;
            spatial::Pose pose{prop.position, prop.orientation};
            if (prop.held) {
                if (!reference)
                    reference = ctx->reference_pose(ctx->session.lastSnapshot().body);
                pose = spatial::compose(spatial::inverse(*reference), pose);
                item.frame_id = held_frame;
            }
            item.position = {pose.translation.x(), pose.translation.y(), pose.translation.z()};
            item.orientation = wxyz(pose.rotation);
            result.push_back(item);
        }
        return result;
    };
}

// Payload meshes scaled to length x diameter x diameter, with `loaded_suffix` on the namespace of
// payloads still in their slot; with delete_all a DELETEALL first clears stale payload markers.
ItemSource payloadItems(VisualContext &context, const Json &options, const std::string &frame_id,
                        const std::string &where) {
    checkOptions(options, where, {"namespaces", "loaded_suffix", "mesh_asset", "color"}, {"delete_all"});
    // Every launcher/dropper mechanism type needs a marker namespace.
    std::map<std::string, std::string> namespaces;
    for (const auto &[kind, name] : options["namespaces"].items())
        namespaces[kind] = name.get<std::string>();
    std::set<std::string> missing;
    for (const auto &mechanism : context.resolved.robot.value("mechanisms", Json::array())) {
        const std::string type = mechanism.at("type");
        if ((type == "launcher" || type == "dropper") && !namespaces.count(type))
            missing.insert(type);
    }
    if (!missing.empty())
        throw BridgeError(where + "/options/namespaces: no namespace for mechanism types " + listRepr(missing));

    if (!options["loaded_suffix"].is_string())
        throw BridgeError(where + "/options/loaded_suffix: must be a string");
    const std::string suffix = options["loaded_suffix"];
    const auto color = quad(numbers(options["color"], 4, where + "/options/color"));
    const std::string uri =
        assetUri(context, "robot", options["mesh_asset"].get<std::string>(), where + "/options/mesh_asset");
    const bool delete_all = options.value("delete_all", false);

    SessionPort *session = &context.session;
    return [=]() {
        std::vector<Item> result;
        if (delete_all) {
            Item wipe;
            wipe.delete_all = true;
            result.push_back(wipe);
        }
        for (const auto &payload : session->payloadVisuals()) {
            Item item;
            item.ns = namespaces.at(payload.mechanism_type) + (payload.loaded ? suffix : "");
            item.id = payload.id;
            item.mesh_type = true;
            item.frame_id = frame_id;
            item.position = {payload.position.x(), payload.position.y(), payload.position.z()};
            item.orientation = wxyz(payload.orientation);
            item.scale = {payload.length_m, 2 * payload.radius_m, 2 * payload.radius_m};
            item.color = color;
            item.mesh = uri;
            item.embedded_materials = false;
            result.push_back(item);
        }
        return result;
    };
}

// A json-format message: `text` in its string `data` field.
std::shared_ptr<Message> encodeJson(const std::shared_ptr<const MessageType> &type, const std::string &text) {
    auto message = std::make_shared<Message>(type);
    const auto path = resolveField(type->members(), "data");
    *static_cast<std::string *>(locate(message->data(), path)) = text;
    return message;
}
} // namespace

std::optional<EndpointKind> formatEndpoint(const std::string &endpoint) {
    static const std::map<std::string, EndpointKind> table = {
        {"state:run", {"json", "timed"}},
        {"state:task_score", {"json", "timed"}},
        {"state:props", {"marker_array", "timed"}},
        {"state:indicators", {"marker_array", "timed"}},
        {"state:payloads", {"marker_array", "timed"}},
        {"event:tasks.feed", {"json", "event"}},
        {"event:scenario.description", {"json", "event"}},
    };
    const auto found = table.find(endpoint);
    if (found == table.end())
        return std::nullopt;
    return found->second;
}

FormatStream compileFormat(VisualContext &context, const Json &stream, const std::shared_ptr<const MessageType> &type,
                           const std::string &where) {
    const std::string endpoint = stream.at("native"), encoding = stream.at("format");
    const auto known = formatEndpoint(endpoint);
    if (!known || known->format != encoding)
        throw BridgeError(where + ": native " + repr(endpoint) + " cannot use format " + repr(encoding));
    if (!stream.at("fields").empty())
        throw BridgeError(where + ": format " + repr(encoding) + " streams take no field map");
    const Json options = stream.value("options", Json::object());
    if (stream.at("direction") != "publish")
        throw BridgeError(where + ": format streams publish");

    FormatStream result;
    result.timed = known->mode == "timed";
    const std::string frame_id = stream.value("frame_id", "");
    // json: unstamped text in a string `data` field.
    if (encoding == "json") {
        checkOptions(options, where, {});
        if (!frame_id.empty())
            throw BridgeError(where + ": json streams are unstamped (frame_id '')");
        try {
            if (resolveField(type->members(), "data").type.base != "string")
                throw BridgeError(where + ": json needs a message with a string 'data' field");
        } catch (const MappingError &) {
            throw;
        }
        result.encode_json = [type](const std::string &text) { return encodeJson(type, text); };
        if (endpoint == "state:run") {
            SessionPort *session = &context.session;
            result.state = [type, session]() {
                const auto snapshot = session->runSnapshot();
                return encodeJson(type, snapshot ? snapshot->dump() : "null");
            };
        } else if (endpoint == "state:task_score") {
            SessionPort *session = &context.session;
            result.state = [type, session]() { return encodeJson(type, session->taskCounters().dump()); };
        }
        return result;
    }

    // marker_array: stamped in the world frame, items from the endpoint's source.
    if (frame_id != context.world_frame)
        throw BridgeError(where + ": marker frame_id must be the world frame " + repr(context.world_frame));
    const auto *members = type->members();
    if (resolveField(type->members(), "markers").type.base != "visualization_msgs/Marker" ||
        std::string(members->message_name_) != "MarkerArray")
        throw BridgeError(where + ": marker_array needs a message with Marker[] 'markers'");
    ItemSource source = endpoint == "state:props"        ? propItems(context, options, frame_id, where)
                        : endpoint == "state:indicators" ? indicatorItems(context, options, frame_id, where)
                                                         : payloadItems(context, options, frame_id, where);
    VisualContext *ctx = &context;
    result.state = [type, ctx, source]() { return encodeMarkers(type, ctx->clock_ns(), source()); };
    return result;
}

} // namespace nereus::ros_bridge

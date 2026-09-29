#include "core.hpp"

#include "visual.hpp"

#include <fnmatch.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <regex>
#include <set>

namespace robotics::ros_bridge {
namespace {
using session::CommandResult;

const std::set<std::string> kSimpleActions = {"command:mechanisms.reload_all", "command:tasks.reset",
                                              "command:scenario.reset", "command:robot.reset_to_start"};
const std::set<std::string> kAlignmentTriggers = {"startup", "placement", "reset_to_start", "full_reset"};

SpecTree tree(std::map<std::string, SpecTree> nodes) { return SpecTree(std::move(nodes)); }
SpecTree simTime() { return tree({{"time", timeSpec()}}); }

const std::map<std::string, Spec> &poseArguments() {
    static const std::map<std::string, Spec> arguments = {
        {"frame", stringSpec()},          {"position_m", vector3Spec()},   {"orientation_w", scalarSpec()},
        {"orientation_x", scalarSpec()},  {"orientation_y", scalarSpec()}, {"orientation_z", scalarSpec()}};
    return arguments;
}

SpecTree resultSources() {
    return tree({{"sim", simTime()}, {"accepted", booleanSpec()}, {"message", stringSpec()}});
}
Value resultValue(std::int64_t ros_ns, const CommandResult &result) {
    return Value::map({{"sim", Value::map({{"time", Value::time(ros_ns)}})},
                       {"accepted", Value::boolean(result.accepted)},
                       {"message", Value::text(result.message)}});
}

std::string numberText(const Json &value) {
    return value.is_string() ? value.get<std::string>() : value.dump();
}

Spec fixedFloat(int n) { return floatArray({n}); }

Eigen::Vector3d rotateBy(const spatial::Pose &pose, const Eigen::Vector3d &v) {
    return pose.rotation * v;
}
Value vec3(const Eigen::Vector3d &v) { return Value::array({v.x(), v.y(), v.z()}); }
Value quat4(const Eigen::Quaterniond &q) { return Value::array({q.w(), q.x(), q.y(), q.z()}); }

std::vector<std::string> keysOf(const Json &array, const char *field) {
    std::vector<std::string> out;
    for (const auto &item : array)
        out.push_back(item.at(field).get<std::string>());
    return out;
}
} // namespace

const std::vector<std::string> &supportedServiceTypes() {
    static const std::vector<std::string> types = {"std_srvs/srv/Trigger", "std_srvs/srv/SetBool",
                                                   "robot_localization/srv/SetPose"};
    return types;
}

Json Counters::toJson() const {
    Json out = Json::object();
    const auto put = [&](const char *name, const CounterTable &table) {
        Json entry = Json::object();
        for (const auto &[key, count] : table)
            entry[key] = count;
        out[name] = entry;
    };
    put("published", published);
    put("unavailable_samples", unavailable_samples);
    put("rejected_commands", rejected_commands);
    put("filtered_messages", filtered_messages);
    put("service_calls", service_calls);
    put("alignments", alignments);
    put("alignments_superseded", alignments_superseded);
    put("alignments_acknowledged", alignments_acknowledged);
    put("alignments_failed", alignments_failed);
    return out;
}

SpecTree readingSpec(const Json &sensor) {
    const auto imu = [] {
        return tree({{"specific_force", vector3Spec()}, {"angular_velocity", vector3Spec()},
                     {"force_covariance", matrix3Spec()}, {"angular_covariance", matrix3Spec()}});
    };
    const auto attitude = [] {
        return tree({{"orientation_wxyz", quaternionSpec()}, {"covariance", matrix3Spec()}});
    };
    const std::string kind = sensor.at("type");
    if (kind == "imu")
        return imu();
    if (kind == "attitude")
        return attitude();
    if (kind == "ahrs")
        return tree({{"inertial", imu()}, {"attitude", attitude()}});
    if (kind == "fog") {
        const int axes = static_cast<int>(sensor.at("parameters").at("axes").size());
        return tree({{"angular_rates", fixedFloat(axes)}, {"covariance", floatArray({axes, axes})}});
    }
    if (kind == "reference_velocity")
        return tree({{"reference_relative_velocity", vector3Spec()}, {"covariance", matrix3Spec()}});
    if (kind == "dvl")
        return tree({{"bottom_relative_velocity", vector3Spec()}, {"covariance", matrix3Spec()},
                     {"bottom_distance", scalarSpec()}});
    if (kind == "reference_altitude")
        return tree({{"mounted_world_z", scalarSpec()}, {"target_world_z", scalarSpec()},
                     {"variance", scalarSpec()}});
    if (kind == "pressure")
        return tree({{"absolute_pressure", scalarSpec()}, {"pressure_variance", scalarSpec()},
                     {"depth", scalarSpec()}, {"depth_variance", scalarSpec()}});
    throw MappingError("sensor " + repr(sensor.at("id").get<std::string>()) + ": no native reading for type " +
                       repr(kind));
}

// ------------------------------------------------------------------ construction

std::optional<std::string> BridgeCore::checkRealTimeFactor(double value) {
    if (!std::isfinite(value) || value < 0)
        return std::string("real_time_factor must be finite and >= 0");
    return std::nullopt;
}

std::optional<std::string> BridgeCore::setRealTimeFactor(double value) {
    if (auto problem = checkRealTimeFactor(value))
        return problem;
    real_time_factor_.store(value);
    return std::nullopt;
}

std::int64_t BridgeCore::steppedPeriod(double rate_hz, const std::string &where) const {
    if (!(rate_hz > 0))
        throw BridgeError(where + ": rate_hz " + std::to_string(rate_hz) + " is too high");
    const auto period = static_cast<std::int64_t>(std::nearbyint(1e9 / rate_hz));
    if (period <= 0)
        throw BridgeError(where + ": rate_hz " + std::to_string(rate_hz) + " is too high");
    if (period < timestep_ns_)
        throw BridgeError(where + ": rate_hz " + std::to_string(rate_hz) + " exceeds the physics step rate");
    return period;
}

BridgeCore::BridgeCore(const session::ResolvedScenario &resolved, SessionPort &session,
                       std::int64_t epoch_ns, Lookup lookup, CameraSink *cameras)
    : resolved_(resolved), session_(session), cameras_(cameras), lookup_(std::move(lookup)) {
    if (resolved.bridge.is_null())
        throw BridgeError("scenario selects no bridge pack");
    config_ = resolved.bridge;
    timestep_ns_ = session_.timestepNs();
    const Json &clock = config_.at("clock");
    real_time_factor_ = clock.at("real_time_factor").get<double>();
    epoch_ns_ = clock.at("epoch") == "system_time_at_start" ? epoch_ns : 0;
    reset_policy_ = clock.at("reset_policy");
    clock_ = Timed{steppedPeriod(clock.at("rate_hz").get<double>(), "clock"), 0};
    const Json names = config_.value("frame_names", Json::object());
    world_frame_ = names.value("world", resolved.scenario.at("world_frame").get<std::string>());
    reference_frame_ = resolved.robot.at("reference_frame");
    root_to_reference_ = session_.frames()->fromRoot(reference_frame_);
    const auto snapshot = session_.observe();
    generation_ = snapshot.generation;
    last_ros_ns_ = rosNs(snapshot.elapsed.count());
    start_state_ = session_.startState();
    for (const auto &item : resolved.robot.value("mechanisms", Json::array()))
        mechanism_types_[item.at("id")] = item.at("type").get<std::string>();
    for (const auto &item : resolved.robot.at("sensors"))
        sensor_types_[item.at("id")] = item.at("type").get<std::string>();

    const Json &safety = resolved.robot.at("safety");
    kill_stops_thrusters_ = safety.at("kill_stops_thrusters");
    commands_while_killed_ = safety.at("commands_while_killed");
    thruster_index_ = std::nullopt;
    if (auto permutation = thrusterPermutation(); !permutation.empty())
        thruster_index_ = std::move(permutation);

    for (const auto &stream : config_.at("streams"))
        stream_config_[stream.at("id")] = stream;
    visual_ = std::make_unique<VisualContext>(VisualContext{
        session_, resolved_, world_frame_, [this] { return clockNs(); },
        [this](const simulation::BodyState &body) { return referencePose(body); },
        [this] { return scenarioJson(); }});
    compileStreams();
    compileServices();
    compileTf();
    compileStaticTf();
    compileAlignment();
    checkBindings();
}

BridgeCore::~BridgeCore() = default;

spatial::Pose BridgeCore::referencePose(const simulation::BodyState &body) const {
    return session_.referencePose(body);
}

std::vector<std::size_t> BridgeCore::thrusterPermutation() {
    if (!config_.contains("thrusters"))
        return {};
    const Json &block = config_["thrusters"];
    std::vector<std::string> native_ids = keysOf(resolved_.robot.at("thrusters"), "id");
    std::vector<std::string> order = block.at("order").get<std::vector<std::string>>();
    auto sorted_order = order, sorted_native = native_ids;
    std::sort(sorted_order.begin(), sorted_order.end());
    std::sort(sorted_native.begin(), sorted_native.end());
    if (sorted_order != sorted_native)
        throw BridgeError("thrusters.order must be a permutation of the robot thrusters");
    if (block.at("input_scales").size() != order.size())
        throw BridgeError("thrusters.input_scales must match thrusters.order");
    std::set<std::string> reject = block.at("reject").get<std::set<std::string>>();
    if (reject != std::set<std::string>{"wrong_length", "nonfinite"})
        throw BridgeError("thrusters.reject must list wrong_length and nonfinite: the native plant "
                          "accepts neither");
    std::vector<std::size_t> index;
    for (const auto &name : order)
        index.push_back(static_cast<std::size_t>(
            std::find(native_ids.begin(), native_ids.end(), name) - native_ids.begin()));
    return index;
}

std::pair<std::string, std::string> BridgeCore::mechanism(const std::string &endpoint,
                                                          const std::string &where) const {
    static const std::regex pattern("^command:mechanisms\\.([A-Za-z0-9_]+)\\.(timed_move|fire|command)$");
    std::smatch match;
    if (!std::regex_match(endpoint, match, pattern))
        throw BridgeError(where + ": unsupported native endpoint " + repr(endpoint));
    const std::string identifier = match[1], operation = match[2];
    const auto kind = mechanism_types_.find(identifier);
    const bool claw_op = operation == "timed_move" || operation == "command";
    const bool ok = kind != mechanism_types_.end() &&
                    (claw_op ? kind->second == "claw" : kind->second == "launcher" || kind->second == "dropper");
    if (!ok)
        throw BridgeError(where + ": " + repr(identifier) + " is not a robot " +
                          (claw_op ? "claw" : "launcher/dropper") + " mechanism");
    return {identifier, operation};
}

SpecTree BridgeCore::mechanismSpec() const {
    std::map<std::string, SpecTree> spec = {{"sim", simTime()}, {"armed", booleanSpec()},
                                            {"any_busy", booleanSpec()}};
    for (const auto &[identifier, kind] : mechanism_types_) {
        if (kind == "launcher" || kind == "dropper")
            spec[identifier] = tree({{"state", stringSpec()}, {"available", integerSpec()}});
        else if (kind == "claw")
            spec[identifier] = tree({{"state", stringSpec()}, {"gap_m", scalarSpec()},
                                     {"target_gap_m", scalarSpec()}});
    }
    return tree(spec);
}

SpecTree BridgeCore::clawSpec() const {
    std::map<std::string, SpecTree> spec = {{"sim", simTime()}};
    for (const auto &[identifier, kind] : mechanism_types_)
        if (kind == "claw")
            spec[identifier] = tree({{"jaws_m", fixedFloat(2)}});
    return tree(spec);
}

Json BridgeCore::sensorJson(const std::string &name, const std::string &where) const {
    for (const auto &item : resolved_.robot.at("sensors"))
        if (item.at("id") == name) {
            const auto ids = session_.sensorIds();
            if (std::find(ids.begin(), ids.end(), name) == ids.end())
                throw BridgeError(where + ": robot sensor " + repr(name) + " is not selected for execution");
            return item;
        }
    throw BridgeError(where + ": unknown robot sensor " + repr(name));
}

void BridgeCore::compileFormatStream(const Json &stream, const std::shared_ptr<const MessageType> &type,
                                     const std::string &where) {
    FormatStream format = compileFormat(*visual_, stream, type, where);
    const std::string id = stream.at("id"), endpoint = stream.at("native");
    PublisherEntry entry;
    entry.type = type;
    if (format.encode_json) {
        auto encode = format.encode_json;
        entry.encode = [encode](const Value &values) { return encode(values.at("json").s); };
    }
    entry.state = format.state;
    publishers_[id] = entry;
    if (format.timed) {
        const auto period = steppedPeriod(stream.at("rate_hz").get<double>(), where);
        timed_.push_back({id, Timed{period, period}});
    } else if (stream.at("rate_hz").get<double>() != 0) {
        throw BridgeError(where + ": event streams have rate_hz 0");
    }
    if (endpoint == "event:tasks.feed")
        feed_streams_.push_back(id);
    else if (endpoint == "event:scenario.description")
        startup_streams_.push_back(id);
}

void BridgeCore::compileStreams() {
    for (const auto &stream : config_.at("streams")) {
        const std::string id = stream.at("id"), where = "streams/" + id, endpoint = stream.at("native");
        try {
            const bool camera_owned =
                cameras_ != nullptr && [&] {
                    const auto ids = cameras_->streamIds();
                    return std::find(ids.begin(), ids.end(), id) != ids.end();
                }();
            const auto type = MessageType::get(stream.at("message_type"));
            stream_types_[id] = type;
            if (camera_owned)
                continue; // the camera sink compiled these mappings
            const std::string direction = stream.at("direction");
            if (direction == "subscribe") {
                if (stream.contains("format") || stream.contains("options"))
                    throw BridgeError(where + ": format/options apply to publish streams");
                std::map<std::string, Spec> arguments;
                if (endpoint == "command:thrusters.set_forces")
                    arguments = {{"forces_n", floatArray({-1})}};
                else if (endpoint == "command:robot.set_killed")
                    arguments = {{"killed", booleanSpec()}};
                else if (endpoint == "command:runs.command")
                    arguments = {{"command_json", stringSpec()}};
                else if (endpoint == "estimate:latest")
                    arguments = poseArguments();
                else if (kSimpleActions.count(endpoint) && endpoint != "command:robot.reset_to_start")
                    arguments = {};
                else if (endpoint == "command:mechanisms.set_armed")
                    arguments = {{"armed", booleanSpec()}};
                else {
                    const auto operation = mechanism(endpoint, where).second;
                    if (operation == "timed_move")
                        arguments = {{"signed_duration_s", scalarSpec()}};
                    else if (operation == "command")
                        arguments = {{"open", booleanSpec()}};
                }
                if (endpoint == "command:thrusters.set_forces" && !thruster_index_)
                    throw BridgeError(where + ": thruster commands need a thrusters block");
                readers_.emplace(id, compileReader(type->members(), stream.at("fields"), arguments,
                                                   stream.value("accept_if", Json()), where));
                continue;
            }
            if (stream.contains("image"))
                throw BridgeError(where + ": image streams are not executed by this bridge");
            if (stream.contains("format") || stream.contains("options")) {
                compileFormatStream(stream, type, where);
                continue;
            }
            SpecTree sources;
            if (endpoint.rfind("sensor:", 0) == 0) {
                const std::string rest = endpoint.substr(7);
                const auto dot = rest.find('.');
                const std::string name = rest.substr(0, dot);
                const std::string output = dot == std::string::npos ? "" : rest.substr(dot + 1);
                const Json sensor = sensorJson(name, where);
                if (!output.empty())
                    throw BridgeError(where + ": sensor output " + repr(output) + " is not executed");
                const double expected = 1e9 / sensor.at("period_ns").get<double>();
                const double rate = stream.at("rate_hz").get<double>();
                if (!(std::fabs(rate - expected) <= 1e-6 * std::max(std::fabs(rate), std::fabs(expected)))) {
                    char buffer[64];
                    std::snprintf(buffer, sizeof buffer, "%.9g", expected);
                    throw BridgeError(where + ": rate_hz " + numberText(stream.at("rate_hz")) +
                                      " differs from sensor " + repr(name) + " rate " + buffer + " Hz");
                }
                sources = tree({{"sample", simTime()}, {"reading", readingSpec(sensor)}});
                sensor_streams_[name].push_back(id);
            } else if (endpoint == "timer" || endpoint == "state:robot" || endpoint == "state:mechanisms" ||
                       endpoint == "state:thrusters" || endpoint == "state:claws") {
                if (endpoint == "state:thrusters" && !thruster_index_)
                    throw BridgeError(where + ": thruster state needs a thrusters block");
                if (endpoint == "state:robot")
                    sources = tree({{"sim", simTime()},
                                    {"reference_pose", tree({{"position", vector3Spec()},
                                                             {"orientation_wxyz", quaternionSpec()}})},
                                    {"reference_body_velocity", vector3Spec()},
                                    {"body_angular_velocity", vector3Spec()}});
                else if (endpoint == "state:mechanisms")
                    sources = mechanismSpec();
                else if (endpoint == "state:claws")
                    sources = clawSpec();
                else if (endpoint == "state:thrusters")
                    sources = tree({{"sim", simTime()}, {"forces_n", floatArray({-1})}});
                else
                    sources = tree({{"sim", simTime()}});
                const auto period = steppedPeriod(stream.at("rate_hz").get<double>(), where);
                timed_.push_back({id, Timed{period, period}});
            } else if (endpoint == "event:robot.kill_changed") {
                sources = tree({{"sim", simTime()}, {"killed", booleanSpec()}});
                events_[endpoint].push_back(id);
            } else if (endpoint == "event:mechanisms.command_result") {
                sources = resultSources();
                events_[endpoint].push_back(id);
            } else {
                throw BridgeError(where + ": unsupported native endpoint " + repr(endpoint));
            }
            const std::string frame_id = stream.value("frame_id", "");
            Writer writer = compileWriter(type->members(), stream.at("fields"), sources, frame_id, where);
            PublisherEntry entry;
            entry.type = type;
            entry.encode = [type, writer](const Value &values) { return writer.make(type, values); };
            publishers_[id] = entry;
        } catch (const MappingError &error) {
            throw BridgeError(error.what());
        }
    }
    for (const auto &stream : config_.at("streams")) {
        if (!stream.contains("reply_stream") || stream["reply_stream"].is_null())
            continue;
        const std::string reply = stream["reply_stream"];
        const auto found = stream_config_.find(reply);
        if (found == stream_config_.end() || found->second.at("native") != "event:mechanisms.command_result")
            throw BridgeError("streams/" + stream.at("id").get<std::string>() +
                              ": reply_stream must name an event:mechanisms.command_result stream");
    }
}

bool BridgeCore::hasEstimateStream() const {
    for (const auto &stream : config_.at("streams"))
        if (stream.at("native") == "estimate:latest")
            return true;
    return false;
}

void BridgeCore::compileServices() {
    for (const auto &service : config_.value("services", Json::array())) {
        const std::string id = service.at("id"), where = "services/" + id, action = service.at("action");
        Json options = service.value("placement", Json::object());
        std::map<std::string, Spec> arguments;
        if (action == "command:robot.place") {
            if (options.empty())
                throw BridgeError(where + ": robot.place needs placement options");
            if (options.at("pose_source") == "request")
                arguments = poseArguments();
        } else if (kSimpleActions.count(action)) {
            arguments = {};
        } else if (action == "command:mechanisms.set_armed") {
            arguments = {{"armed", booleanSpec()}};
        } else {
            const auto operation = mechanism(action, where).second;
            if (operation == "timed_move")
                throw BridgeError(where + ": timed_move is a topic command");
            if (operation == "command")
                arguments = {{"open", booleanSpec()}};
        }
        if (options.value("pose_source", "") == "estimate" && !hasEstimateStream())
            throw BridgeError(where + ": pose_source estimate needs an estimate:latest stream");
        try {
            const std::string type_name = service.at("service_type");
            const auto &supported = supportedServiceTypes();
            if (std::find(supported.begin(), supported.end(), type_name) == supported.end()) {
                std::string list;
                for (const auto &name : supported)
                    list += (list.empty() ? "" : ", ") + name;
                throw BridgeError(where + ": service type " + repr(type_name) +
                                  " is not supported by this bridge (supported: " + list + ")");
            }
            const auto type = ServiceType::get(type_name);
            ServiceEntry entry{type,
                               compileReader(type->request, service.at("request"), arguments, Json(),
                                             where + "/request"),
                               compileWriter(type->response, service.at("response"), resultSources(),
                                             std::nullopt, where + "/response"),
                               options};
            options["action"] = action;
            entry.options = options;
            services_.emplace(id, std::move(entry));
        } catch (const MappingError &error) {
            throw BridgeError(error.what());
        }
    }
}

void BridgeCore::compileTf() {
    for (const auto &entry : config_.value("tf", Json::object()).value("publish", Json::array())) {
        if (entry.at("native") != "state:robot.reference_pose")
            throw BridgeError("tf: unsupported native endpoint " + repr(entry.at("native").get<std::string>()));
        if (entry.at("parent") != world_frame_)
            throw BridgeError("tf: truth transforms must have parent " + repr(world_frame_));
        const auto period = steppedPeriod(entry.at("rate_hz").get<double>(), "tf");
        tf_publish_.push_back({entry, period});
        tf_timed_.push_back(Timed{period, period});
    }
}

void BridgeCore::compileStaticTf() {
    const Json tf = config_.value("tf", Json::object());
    std::map<std::string, std::string> parents;
    for (const char *group : {"publish", "static", "lookup"})
        for (const auto &edge : tf.value(group, Json::array())) {
            const std::string child = edge.at("child");
            if (parents.count(child))
                throw BridgeError("tf: duplicate child/owner " + repr(child));
            parents[child] = edge.at("parent").get<std::string>();
        }
    for (const auto &[child, parent] : parents) {
        std::set<std::string> seen;
        std::string current = child;
        while (parents.count(current) && !seen.count(current)) {
            seen.insert(current);
            current = parents.at(current);
        }
        if (seen.count(current))
            throw BridgeError("tf: cycle at " + repr(current));
    }
    const Json names = config_.value("frame_names", Json::object());
    for (const auto &edge : tf.value("static", Json::array())) {
        for (const auto &[native_name, ros_name] :
             {std::pair<const char *, const char *>{"from_frame", "parent"}, {"to_frame", "child"}}) {
            const std::string frame = edge.at(native_name);
            if (names.contains(frame) && names[frame] != edge.at(ros_name))
                throw BridgeError(std::string("tf: ") + ros_name + " differs from frame_names[" + repr(frame) + "]");
        }
        try {
            const auto pose = session_.frames()->lookup(edge.at("from_frame"), edge.at("to_frame"));
            static_transforms_.push_back({edge.at("parent"), edge.at("child"), last_ros_ns_,
                                          pose.translation, pose.rotation});
        } catch (const std::exception &error) {
            throw BridgeError(std::string("tf: invalid static robot frames: ") + error.what());
        }
    }
}

void BridgeCore::compileAlignment() {
    const Json placement = config_.value("placement", Json::object());
    if (!placement.contains("estimator_alignment") || placement["estimator_alignment"].is_null())
        return;
    alignment_ = placement["estimator_alignment"];
    for (const auto &trigger : alignment_->at("triggers"))
        if (!kAlignmentTriggers.count(trigger.get<std::string>()))
            throw BridgeError("estimator_alignment: unknown trigger");
    const auto stream = stream_config_.find(alignment_->at("estimate_stream"));
    if (stream == stream_config_.end() || stream->second.at("native") != "estimate:latest")
        throw BridgeError("estimator_alignment: estimate_stream must be an estimate:latest subscription");
    try {
        const std::string type_name = alignment_->at("service_type");
        if (type_name != "robot_localization/srv/SetPose")
            throw BridgeError("estimator_alignment: service type " + repr(type_name) +
                              " is not supported by this bridge (supported: robot_localization/srv/SetPose)");
        const auto type = ServiceType::get(type_name);
        if (resolveField(type->request, "pose").type.base != "geometry_msgs/PoseWithCovarianceStamped")
            throw BridgeError("estimator_alignment: service request must be pose: "
                              "geometry_msgs/PoseWithCovarianceStamped");
    } catch (const MappingError &error) {
        throw BridgeError(error.what());
    }
    for (const auto &trigger : alignment_->at("triggers"))
        if (trigger == "startup")
            alignment_pending_ = "startup";
}

void BridgeCore::checkBindings() {
    std::size_t converters = config_.value("converters", Json::array()).size();
    for (const auto &stream : config_.at("streams"))
        converters += stream.contains("converter");
    for (const auto &service : config_.value("services", Json::array()))
        converters += service.contains("converter");
    if (converters)
        throw BridgeError("compiled converters are declared but none is implemented here");

    std::map<std::string, Json> services;
    for (const auto &item : config_.value("services", Json::array()))
        services[item.at("id")] = item;
    const auto require = [](const std::string &kind, const std::string &name, bool check,
                            const std::string &expected) {
        if (!check)
            throw BridgeError(kind + " " + repr(name) + " must be " + expected);
    };
    if (config_.contains("kill") && !config_["kill"].is_null()) {
        const Json &kill = config_["kill"];
        const Json &command = stream_config_.at(kill.at("command_stream"));
        const Json &state = stream_config_.at(kill.at("state_stream"));
        require("kill.command_stream", command.at("id"), command.at("native") == "command:robot.set_killed",
                "command:robot.set_killed");
        require("kill.state_stream", state.at("id"), state.at("native") == "event:robot.kill_changed",
                "event:robot.kill_changed");
    }
    static const std::map<std::string, std::string> actions = {
        {"robot_service", "command:robot.reset_to_start"},
        {"tasks_service", "command:tasks.reset"},
        {"full_service", "command:scenario.reset"}};
    for (const auto &[key, value] : config_.value("reset", Json::object()).items()) {
        const auto action = actions.find(key);
        const std::string service = value;
        const bool ok = action != actions.end() && services.count(service) &&
                        services.at(service).at("action") == action->second;
        require("reset." + key, service, ok,
                "a service with action " + (action == actions.end() ? "(unknown reset key)" : action->second));
    }
    const Json placement = config_.value("placement", Json::object());
    for (const auto &[key, source] : {std::pair<const char *, const char *>{"set_service", "request"},
                                      {"sync_service", "estimate"}}) {
        if (!placement.contains(key))
            continue;
        const std::string name = placement[key];
        const Json &service = services.at(name);
        require(std::string("placement.") + key, service.at("id"),
                service.at("action") == "command:robot.place" &&
                    service.value("placement", Json::object()).value("pose_source", "") == source,
                std::string("a command:robot.place service with pose_source ") + source);
    }
    const Json names = config_.value("frame_names", Json::object());
    for (const auto &stream : config_.at("streams")) {
        const std::string id = stream.at("id");
        if (cameras_ != nullptr) {
            const auto ids = cameras_->streamIds();
            if (std::find(ids.begin(), ids.end(), id) != ids.end())
                continue; // the camera sink checks the appropriate optical frame
        }
        const std::string native = stream.at("native");
        if (native.rfind("sensor:", 0) != 0)
            continue;
        const std::string name = native.substr(7, native.find('.') == std::string::npos
                                                      ? std::string::npos
                                                      : native.find('.') - 7);
        const std::string frame_id = stream.value("frame_id", "");
        for (const auto &sensor : resolved_.robot.at("sensors"))
            if (sensor.at("id") == name && !frame_id.empty() && names.contains(sensor.at("frame").get<std::string>())) {
                const std::string expected = names[sensor.at("frame").get<std::string>()];
                require("streams/" + id + ".frame_id", frame_id, frame_id == expected,
                        repr(expected) + " (frame_names of sensor frame " +
                            repr(sensor.at("frame").get<std::string>()) + ")");
            }
    }
    std::vector<std::string> children;
    for (const auto &[entry, period] : tf_publish_)
        children.push_back(entry.at("child"));
    for (const auto &transform : static_transforms_)
        children.push_back(transform.child);
    for (const auto &child : children)
        for (const auto &pattern : config_.value("tf", Json::object()).value("never_publish", Json::array()))
            if (fnmatch(pattern.get<std::string>().c_str(), child.c_str(), 0) == 0)
                throw BridgeError("tf: " + repr(child) + " is listed in never_publish");
}

// ------------------------------------------------------------------ time and stepping

void BridgeCore::observeGeneration(const simulation::Snapshot &snapshot, bool coordinated) {
    if (snapshot.generation == generation_)
        return;
    if (cameras_ != nullptr && !coordinated) {
        cameras_->invalidate(std::nullopt);
        throw BridgeError("native reset with cameras needs a coordinated full reset");
    }
    generation_ = snapshot.generation;
    if (reset_policy_ == "preserve_ros_epoch_and_time")
        offset_ns_ = last_ros_ns_ - epoch_ns_ + timestep_ns_;
    clock_.next_ns = 0;
    for (auto &[id, timed] : timed_)
        timed.next_ns = timed.period_ns;
    for (auto &timed : tf_timed_)
        timed.next_ns = timed.period_ns;
}

Publication BridgeCore::publish(const std::string &stream, const Value &values) {
    Counters::bump(counters_.published, stream);
    const auto &entry = publishers_.at(stream);
    return Publication{stream, entry.encode(values)};
}

Publication BridgeCore::publishMessage(const std::string &stream, std::shared_ptr<Message> message) {
    Counters::bump(counters_.published, stream);
    return Publication{stream, std::move(message)};
}

Value BridgeCore::robotState(const simulation::Snapshot &snapshot) const {
    const auto &body = snapshot.body;
    const spatial::Pose world_reference = referencePose(body);
    const Eigen::Vector3d &offset = root_to_reference_.translation;
    const Eigen::Vector3d omega = body.angular_velocity;
    const Eigen::Vector3d velocity = body.linear_velocity + omega.cross(offset);
    const spatial::Pose to_reference = spatial::inverse(root_to_reference_);
    return Value::map(
        {{"sim", Value::map({{"time", Value::time(last_ros_ns_)}})},
         {"reference_pose", Value::map({{"position", vec3(world_reference.translation)},
                                        {"orientation_wxyz", quat4(world_reference.rotation)}})},
         {"reference_body_velocity", vec3(rotateBy(to_reference, velocity))},
         {"body_angular_velocity", vec3(rotateBy(to_reference, omega))}});
}

Value BridgeCore::thrusterValues() const {
    const auto &scales = config_.at("thrusters").at("input_scales");
    const Eigen::VectorXd native = session_.thrusterForces();
    std::vector<double> forces;
    for (std::size_t k = 0; k < thruster_index_->size(); ++k) {
        const double scale = scales.at(k).get<double>();
        forces.push_back(scale != 0 ? native[static_cast<Eigen::Index>((*thruster_index_)[k])] / scale : 0.0);
    }
    return Value::map({{"sim", Value::map({{"time", Value::time(last_ros_ns_)}})},
                       {"forces_n", Value::array(std::move(forces))}});
}

Value BridgeCore::clawValues() const {
    const auto jaws = session_.clawJaws();
    std::map<std::string, Value> values = {{"sim", Value::map({{"time", Value::time(last_ros_ns_)}})}};
    for (const auto &[identifier, kind] : mechanism_types_)
        if (kind == "claw") {
            const auto found = jaws.find(identifier);
            const std::array<double, 2> pair = found == jaws.end() ? std::array<double, 2>{0, 0} : found->second;
            values[identifier] = Value::map({{"jaws_m", Value::array({pair[0], pair[1]})}});
        }
    return Value::map(std::move(values));
}

Value BridgeCore::mechanismValues() const {
    const auto state = session_.mechanismState();
    std::map<std::string, Value> values = {
        {"sim", Value::map({{"time", Value::time(last_ros_ns_)}})},
        {"armed", Value::boolean(state && state->armed)},
        {"any_busy", Value::boolean(state && state->any_busy)}};
    if (state) {
        for (const auto &[key, release] : state->releases)
            values[key] = Value::map({{"state", Value::text(release.state)},
                                      {"available", Value::integer(release.available)}});
        for (const auto &[key, claw] : state->claws)
            values[key] = Value::map({{"state", Value::text(claw.state)}, {"gap_m", Value::real(claw.gap_m)},
                                      {"target_gap_m", Value::real(claw.target_gap_m)}});
    }
    return Value::map(std::move(values));
}

StepOutput BridgeCore::step() {
    StepResult result = session_.advance();
    const auto &snapshot = result.snapshot;
    for (auto &event : result.task_events)
        task_events_.push_back(event);
    observeGeneration(snapshot);
    const std::int64_t now = snapshot.elapsed.count();
    last_ros_ns_ = rosNs(now);
    StepOutput out;
    if (now >= clock_.next_ns) {
        out.clocks.push_back(last_ros_ns_);
        clock_.next_ns = now - now % clock_.period_ns + clock_.period_ns;
    }
    for (auto &sample : session_.drainSensors()) {
        const auto streams = sensor_streams_.find(sample.sensor); // unbridged sensors still drain
        if (!sample.reading) {
            Counters::bump(counters_.unavailable_samples, sample.sensor);
            continue;
        }
        if (streams == sensor_streams_.end())
            continue;
        const Value values = Value::map({{"sample", Value::map({{"time", Value::time(rosNs(sample.acquired_ns))}})},
                                         {"reading", std::move(*sample.reading)}});
        for (const auto &stream : streams->second)
            out.publications.push_back(publish(stream, values));
    }
    std::optional<Value> state;
    const auto robot = [&]() -> const Value & {
        if (!state)
            state = robotState(snapshot);
        return *state;
    };
    for (auto &[stream, timed] : timed_) {
        if (now < timed.next_ns)
            continue;
        const std::string native = stream_config_.at(stream).at("native");
        if (publishers_.at(stream).state)
            out.publications.push_back(publishMessage(stream, publishers_.at(stream).state()));
        else if (native == "state:mechanisms")
            out.publications.push_back(publish(stream, mechanismValues()));
        else if (native == "state:thrusters")
            out.publications.push_back(publish(stream, thrusterValues()));
        else if (native == "state:claws")
            out.publications.push_back(publish(stream, clawValues()));
        else
            out.publications.push_back(publish(stream, robot()));
        timed.next_ns += timed.period_ns;
    }
    for (std::size_t k = 0; k < tf_publish_.size(); ++k) {
        auto &timed = tf_timed_[k];
        if (now < timed.next_ns)
            continue;
        const Value &pose = robot().at("reference_pose");
        const auto &p = pose.at("position").a;
        const auto &q = pose.at("orientation_wxyz").a;
        out.transforms.push_back({tf_publish_[k].first.at("parent"), tf_publish_[k].first.at("child"),
                                  last_ros_ns_, Eigen::Vector3d(p[0], p[1], p[2]),
                                  Eigen::Quaterniond(q[0], q[1], q[2], q[3])});
        timed.next_ns += timed.period_ns;
    }
    for (auto &publication : flush())
        out.publications.push_back(std::move(publication));
    return out;
}

std::vector<Publication> BridgeCore::flush() {
    std::vector<Publication> publications;
    const Json feed = session_.takeFeed();
    for (const auto &item : feed)
        for (const auto &stream : feed_streams_)
            publications.push_back(publish(stream, Value::map({{"sim", Value::map({{"time", Value::time(last_ros_ns_)}})},
                                                               {"json", Value::text(item.dump())}})));
    return publications;
}

std::vector<Publication> BridgeCore::refresh() {
    std::vector<Publication> publications;
    for (const auto &[stream, timed] : timed_) {
        const std::string native = stream_config_.at(stream).at("native");
        if (native.rfind("state:", 0) != 0 || native == "state:robot")
            continue;
        if (publishers_.at(stream).state)
            publications.push_back(publishMessage(stream, publishers_.at(stream).state()));
        else if (native == "state:mechanisms")
            publications.push_back(publish(stream, mechanismValues()));
        else if (native == "state:thrusters")
            publications.push_back(publish(stream, thrusterValues()));
        else
            publications.push_back(publish(stream, clawValues()));
    }
    return publications;
}

std::vector<Publication> BridgeCore::startupPublications() {
    std::vector<Publication> publications;
    for (const auto &stream : startup_streams_)
        publications.push_back(publish(stream, Value::map({{"sim", Value::map({{"time", Value::time(last_ros_ns_)}})},
                                                           {"json", Value::text(scenarioJson())}})));
    return publications;
}

std::string BridgeCore::scenarioJson() const {
    return resolved_.document.dump();
}

// ------------------------------------------------------------------ inbound

std::vector<Publication> BridgeCore::receive(const std::string &stream, const void *message) {
    const Reader &reader = readers_.at(stream);
    Value arguments;
    try {
        if (!reader.accepts(message)) {
            Counters::bump(counters_.filtered_messages, stream);
            return {};
        }
        arguments = reader(message);
    } catch (const MappingError &) {
        Counters::bump(counters_.rejected_commands, stream + ":malformed");
        return {};
    }
    const std::string endpoint = stream_config_.at(stream).at("native");
    if (endpoint == "command:thrusters.set_forces") {
        command(stream, arguments.at("forces_n").a);
        return {};
    }
    if (endpoint == "command:robot.set_killed")
        return setKilled(arguments.at("killed").b);
    if (endpoint == "estimate:latest") {
        latest_estimate_ = arguments;
        return {};
    }
    if (endpoint == "command:runs.command") {
        runCommand(arguments.at("command_json").s);
        return {};
    }
    CommandResult result;
    if (endpoint == "command:scenario.reset") {
        const auto [accepted, text] = fullReset();
        requestAlignment("full_reset");
        result = CommandResult{accepted, text};
    } else {
        result = mechanismAction(endpoint, arguments);
    }
    const auto &config = stream_config_.at(stream);
    if (!config.contains("reply_stream") || config["reply_stream"].is_null())
        return {};
    return {publish(config["reply_stream"], resultValue(last_ros_ns_, result))};
}

CommandResult BridgeCore::runCommand(const std::string &text) {
    CommandResult result;
    try {
        const Json command = Json::parse(text);
        if (!command.is_object())
            throw std::invalid_argument("run command must be a JSON object");
        if (!command.contains("action"))
            throw std::out_of_range("missing field 'action'");
        const Json &action = command["action"];
        if (action == "start") {
            Json options = command;
            options.erase("action");
            result = session_.runStart(options);
        } else if (action == "stop") {
            result = session_.runStop();
        } else if (action == "adjustment") {
            if (!command.contains("points"))
                throw std::out_of_range("missing field 'points'");
            if (!command["points"].is_number())
                throw std::invalid_argument("points must be a number");
            result = session_.runAdjust(command["points"].get<double>());
        } else {
            result = CommandResult{false, "Unknown run command"};
        }
    } catch (const std::out_of_range &error) {
        result = CommandResult{false, error.what()};
    } catch (const std::invalid_argument &error) {
        result = CommandResult{false, error.what()};
    } catch (const Json::exception &error) {
        result = CommandResult{false, error.what()};
    }
    if (!result.accepted)
        Counters::bump(counters_.rejected_commands, "run_command");
    return result;
}

void BridgeCore::command(const std::string &stream, const std::vector<double> &forces_in) {
    const Json &block = config_.at("thrusters");
    if (forces_in.size() != thruster_index_->size()) {
        Counters::bump(counters_.rejected_commands, stream + ":wrong_length");
        return;
    }
    if (!std::all_of(forces_in.begin(), forces_in.end(), [](double v) { return std::isfinite(v); })) {
        Counters::bump(counters_.rejected_commands, stream + ":nonfinite");
        return;
    }
    std::vector<double> forces = forces_in;
    if (session_.killed()) {
        if (commands_while_killed_ == "rejected") {
            Counters::bump(counters_.rejected_commands, stream + ":killed");
            return;
        }
        std::fill(forces.begin(), forces.end(), 0.0);
    }
    Eigen::VectorXd native = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(thruster_index_->size()));
    for (std::size_t position = 0; position < thruster_index_->size(); ++position)
        native[static_cast<Eigen::Index>((*thruster_index_)[position])] =
            forces[position] * block.at("input_scales").at(position).get<double>();
    if (!native.allFinite()) { // finite input can overflow when scaled
        Counters::bump(counters_.rejected_commands, stream + ":nonfinite_scaled");
        return;
    }
    session_.commandThrusters(native);
}

std::vector<Publication> BridgeCore::setKilled(bool killed) {
    session_.setKilled(killed);
    std::vector<Publication> out;
    const auto found = events_.find("event:robot.kill_changed");
    if (found == events_.end())
        return out;
    const Value values = Value::map({{"sim", Value::map({{"time", Value::time(last_ros_ns_)}})},
                                     {"killed", Value::boolean(session_.killed())}});
    for (const auto &stream : found->second)
        out.push_back(publish(stream, values));
    return out;
}

// ------------------------------------------------------------------ services

void BridgeCore::call(const std::string &service, const void *request, void *response) {
    ServiceEntry &entry = services_.at(service);
    Counters::bump(counters_.service_calls, service);
    Value arguments;
    try {
        arguments = entry.reader(request);
    } catch (const MappingError &error) {
        Counters::bump(counters_.rejected_commands, service + ":malformed");
        entry.writer.apply(response, resultValue(last_ros_ns_, CommandResult{false, error.what()}));
        return;
    }
    const std::string action = entry.options.at("action");
    std::optional<std::string> trigger;
    bool accepted = false;
    std::string message;
    if (action == "command:robot.reset_to_start") {
        std::tie(accepted, message) = placeState(start_state_, false, false);
        trigger = "reset_to_start";
    } else if (action == "command:scenario.reset") {
        std::tie(accepted, message) = fullReset();
        trigger = "full_reset";
    } else if (action != "command:robot.place") {
        const auto result = mechanismAction(action, arguments);
        accepted = result.accepted;
        message = result.message;
    } else if (entry.options.at("pose_source") == "estimate") {
        if (!latest_estimate_)
            std::tie(accepted, message) = std::pair<bool, std::string>{false, "no estimate received yet"};
        else
            std::tie(accepted, message) = placeReference(*latest_estimate_, entry.options);
        trigger = "placement";
    } else {
        std::tie(accepted, message) = placeReference(arguments, entry.options);
        trigger = "placement";
    }
    if (accepted && trigger &&
        (*trigger == "reset_to_start" || *trigger == "full_reset" || entry.options.value("align_estimator", false)))
        requestAlignment(*trigger);
    entry.writer.apply(response, resultValue(last_ros_ns_, CommandResult{accepted, message}));
}

CommandResult BridgeCore::mechanismAction(const std::string &action, const Value &arguments) {
    if (action == "command:mechanisms.set_armed")
        return session_.setArmed(arguments.at("armed").b);
    if (action == "command:mechanisms.reload_all")
        return session_.reloadAll();
    if (action == "command:tasks.reset")
        return session_.resetTasks();
    const auto [identifier, operation] = mechanism(action, "command");
    if (operation == "timed_move")
        return session_.moveClaw(identifier, arguments.at("signed_duration_s").asDouble());
    if (operation == "fire") {
        session::Events events;
        const auto result = session_.fire(identifier, events);
        for (auto &event : events)
            task_events_.push_back(event);
        return result;
    }
    return session_.commandClaw(identifier, arguments.at("open").b);
}

std::pair<bool, std::string> BridgeCore::fullReset() {
    if (cameras_ != nullptr)
        cameras_->invalidate(session_.seed());
    const auto snapshot = session_.fullReset();
    start_state_ = session_.startState();
    observeGeneration(snapshot, true);
    return {true, "Scenario reset: plant, sensors, mechanisms, payloads, tasks and scores"};
}

std::optional<spatial::Pose> BridgeCore::transform(const std::string &target, const std::string &source) const {
    if (target == source)
        return spatial::Pose{};
    return lookup_ ? lookup_(target, source) : std::nullopt;
}

std::pair<bool, std::string> BridgeCore::placeReference(const Value &pose, const Json &options) {
    Eigen::Quaterniond raw(pose.at("orientation_w").asDouble(), pose.at("orientation_x").asDouble(),
                           pose.at("orientation_y").asDouble(), pose.at("orientation_z").asDouble());
    const auto &p = pose.at("position_m").a;
    const Eigen::Vector3d position(p[0], p[1], p[2]);
    const double norm = raw.norm();
    if (!(position.allFinite() && std::isfinite(norm) && norm > 1e-10))
        return {false, "pose must be finite with a nonzero quaternion"};
    const std::string frame = pose.at("frame").s.empty() ? world_frame_ : pose.at("frame").s;
    const auto world_to_frame = transform(world_frame_, frame);
    if (!world_to_frame)
        return {false, "no transform from " + repr(frame) + " to " + repr(world_frame_)};
    raw.coeffs() /= norm;
    const spatial::Pose world_reference = spatial::compose(*world_to_frame, spatial::Pose{position, raw});
    const spatial::Pose com = spatial::compose(world_reference, spatial::inverse(root_to_reference_));
    simulation::BodyState state;
    state.position = com.translation;
    state.orientation = com.rotation;
    const bool keep = options.at("keep_velocity");
    if (keep) {
        const auto body = session_.observe().body;
        state.linear_velocity = body.linear_velocity;
        state.angular_velocity = body.angular_velocity;
    }
    return placeState(state, keep, options.at("becomes_start_pose"));
}

std::pair<bool, std::string> BridgeCore::placeState(const simulation::BodyState &state, bool keep_velocity,
                                                    bool becomes_start) {
    simulation::BodyState target;
    target.position = state.position;
    target.orientation = state.orientation;
    if (keep_velocity) {
        target.linear_velocity = state.linear_velocity;
        target.angular_velocity = state.angular_velocity;
    }
    simulation::Snapshot snapshot;
    try {
        if (cameras_ != nullptr)
            cameras_->invalidate(std::nullopt);
        snapshot = session_.place(target, !keep_velocity);
    } catch (const std::invalid_argument &error) {
        return {false, error.what()};
    }
    if (becomes_start)
        start_state_ = target;
    const auto reference = referencePose(snapshot.body);
    char buffer[160];
    std::snprintf(buffer, sizeof buffer, "placed %s at (%.3f, %.3f, %.3f) m", reference_frame_.c_str(),
                  reference.translation.x(), reference.translation.y(), reference.translation.z());
    return {true, buffer};
}

// ------------------------------------------------------------------ estimator alignment

void BridgeCore::requestAlignment(const std::string &trigger) {
    if (!alignment_)
        return;
    for (const auto &item : alignment_->at("triggers"))
        if (item == trigger) {
            if (alignment_pending_)
                Counters::bump(counters_.alignments_superseded, *alignment_pending_);
            alignment_pending_ = trigger;
            return;
        }
}

std::optional<Alignment> BridgeCore::pendingAlignment() {
    if (!alignment_pending_ || !latest_estimate_)
        return std::nullopt;
    const std::string frame = latest_estimate_->at("frame").s;
    const auto estimate_to_world = transform(frame, world_frame_);
    if (!estimate_to_world)
        return std::nullopt;
    const auto pose = spatial::compose(*estimate_to_world, referencePose(session_.observe().body));
    Alignment alignment;
    alignment.trigger = *alignment_pending_;
    alignment.frame = frame;
    alignment.stamp_ns = last_ros_ns_;
    alignment.position = pose.translation;
    alignment.orientation = pose.rotation;
    alignment.covariance_diagonal = alignment_->at("covariance_diagonal").get<double>();
    Counters::bump(counters_.alignments, alignment.trigger);
    alignment_pending_.reset();
    return alignment;
}

} // namespace robotics::ros_bridge

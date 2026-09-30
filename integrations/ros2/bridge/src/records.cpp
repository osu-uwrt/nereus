#include "records.hpp"

#include <fstream>

namespace robotics::ros_bridge {
namespace {
const char *kGapNote =
    "Mechanisms, payloads and selected task packs run in the session; pending task entries are not "
    "executed. Selected cameras load their referenced scene assets.";

Json pick(const Json &object, const std::vector<std::string> &keys) {
    Json out = Json::object();
    for (const auto &key : keys)
        out[key] = object.contains(key) ? object[key] : Json();
    return out;
}
} // namespace

void writeJson(const std::filesystem::path &path, const Json &document) {
    std::ofstream out(path);
    if (!out)
        throw std::runtime_error("cannot write " + path.string());
    out << document.dump(2) << "\n";
}

Json executionRecord(const session::ResolvedScenario &resolved, const BridgeCore &core,
                     const std::vector<std::string> &sensors, const std::vector<std::string> &deferred,
                     std::optional<std::int64_t> duration_ns, const CameraSink *cameras) {
    const Json &config = core.config();
    Json without_stream = Json::array();
    for (const auto &name : sensors) {
        bool bridged = false;
        for (const auto &stream : config.at("streams")) {
            const std::string native = stream.at("native");
            if (native.rfind("sensor:", 0) == 0 && native.substr(7, native.find('.') == std::string::npos
                                                                        ? std::string::npos
                                                                        : native.find('.') - 7) == name)
                bridged = true;
        }
        if (!bridged)
            without_stream.push_back(name);
    }
    Json streams = Json::array();
    for (const auto &stream : config.at("streams"))
        streams.push_back(pick(stream, {"id", "direction", "topic", "message_type", "native", "frame_id",
                                        "rate_hz", "format"}));
    Json services = Json::array();
    for (const auto &service : config.value("services", Json::array()))
        services.push_back(pick(service, {"id", "service", "service_type", "action"}));
    return Json{
        {"format", "nereus_ros.execution"},
        {"version", 1},
        {"resolved_content_sha256", resolved.document.value("content_sha256", "")},
        {"timestep_ns", core.timestepNs()},
        {"duration_ns", duration_ns ? Json(*duration_ns) : Json()},
        {"clock", {{"epoch_ns", core.epochNs()}, {"reset_policy", core.resetPolicy()},
                   {"real_time_factor", core.realTimeFactor()}, {"topic", config.at("clock").at("topic")}}},
        {"namespace", config.at("namespace")},
        {"node_name", config.value("node_name", "nereus_bridge")},
        {"parameters", {{"real_time_factor", "double on the bridge node; 0 pauses stepping and /clock, "
                                             "negative or non-finite values are rejected"}}},
        {"world_frame", core.worldFrame()},
        {"sensors", {{"selected", sensors}, {"not_executed", deferred}, {"selected_without_stream", without_stream}}},
        {"streams", streams},
        {"services", services},
        {"tf", config.value("tf", Json::object())},
        {"cameras", cameras == nullptr ? Json() : cameras->describe()},
        {"estimator_alignment", config.value("placement", Json::object()).value("estimator_alignment", Json())},
        {"unresolved", {{"note", kGapNote}, {"items", resolved.document.value("unresolved", Json::array())}}},
        {"not_executed_config",
         {{"scenario.run.options", "defaults passed to task hooks; simulator/run_command start overrides "
                                   "them per run"},
          {"tf.lookup", "external owners; uses latest live TF, not stamp-matched transforms"}}},
    };
}

Json tasksRecord(BridgeCore &core) {
    SessionPort &session = core.session();
    const auto run = session.runSnapshot();
    Json scores = Json::object();
    if (run && run->contains("rows"))
        for (const auto &row : run->at("rows"))
            scores[row.at("key").get<std::string>()] = row.at("points");
    return Json{{"format", "nereus_ros.tasks"},
                {"version", 1},
                {"scores", scores},
                {"run", run ? *run : Json()},
                {"counters", session.taskCounters()},
                {"events", core.taskEvents()}};
}

Json summaryRecord(BridgeCore &core, const std::string &reason, const CameraSink *cameras) {
    const auto snapshot = core.session().observe();
    Json stats = Json::object();
    for (const auto &[name, item] : core.session().sensorStats())
        stats[name] = {{"acquired", item.acquired}, {"delivered", item.delivered},
                       {"unavailable", item.unavailable}, {"dropped_pending", item.dropped_pending},
                       {"dropped_delivered", item.dropped_delivered}};
    return Json{{"format", "nereus_ros.summary"},
                {"version", 1},
                {"stop", reason},
                {"ticks", snapshot.tick},
                {"elapsed_ns", snapshot.elapsed.count()},
                {"generation", snapshot.generation},
                {"killed", core.killed()},
                {"counters", core.counters().toJson()},
                {"camera_stats", cameras == nullptr ? Json::object() : cameras->stats()},
                {"stream_stats", stats}};
}
} // namespace robotics::ros_bridge

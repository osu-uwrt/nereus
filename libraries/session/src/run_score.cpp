#include "run_score.hpp"

#include <algorithm>
#include <set>

namespace robotics::session {
Json buildRunSnapshot(const Json &task_pack, const Json &snapshot, Json extra, std::int64_t now_ns, double adjustment,
                      const std::string &message) {
    const Json &run = snapshot.at("run");
    const bool running = run.at("running").get<bool>();
    const std::int64_t started = run.at("started_ns").get<std::int64_t>();
    const Json &stopped = run.at("stopped_ns");
    const std::int64_t end = running ? now_ns : (stopped.is_null() ? started : stopped.get<std::int64_t>());
    std::string ended;
    if (extra.is_object() && extra.contains("ended_reason")) {
        const Json &reason = extra.at("ended_reason");
        ended = reason.is_string() ? reason.get<std::string>() : reason.dump();
        extra.erase("ended_reason");
    }
    const Json &scores = snapshot.at("scores");
    Json declared = Json::array();
    if (task_pack.contains("score_rows") && task_pack.at("score_rows").is_array() &&
        !task_pack.at("score_rows").empty())
        declared = task_pack.at("score_rows");
    else
        for (const auto &[key, value] : scores.items()) {
            (void)value;
            declared.push_back({{"key", key}, {"label", key}});
        }
    Json rows = Json::array();
    std::set<std::string> listed;
    double total = 0;
    const auto add = [&](const std::string &key, const Json &label, const Json &points) {
        rows.push_back({{"key", key}, {"label", label}, {"points", points}});
        total += points.get<double>();
    };
    for (const auto &row : declared) {
        const std::string key = row.at("key").get<std::string>();
        listed.insert(key);
        add(key, row.at("label"), scores.contains(key) ? scores.at(key) : Json(0));
    }
    for (const auto &[key, value] : scores.items())
        if (!listed.count(key))
            add(key, key, value);
    Json out = {{"running", running},
                {"elapsed", static_cast<double>(std::max<std::int64_t>(0, end - started)) / 1e9},
                {"scoring_open", running && ended.empty()},
                {"ended_reason", ended}};
    if (extra.is_object())
        for (const auto &[key, value] : extra.items())
            out[key] = value;
    out["total"] = total + adjustment;
    out["adjustment"] = adjustment;
    out["rows"] = rows;
    out["message"] = message;
    out["ui"] = task_pack.contains("ui") ? task_pack.at("ui") : Json::object();
    out["config_id"] = task_pack.at("id");
    return out;
}
} // namespace robotics::session

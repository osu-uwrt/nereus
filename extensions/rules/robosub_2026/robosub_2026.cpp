// RoboSub 2026 scoring. Required JSON keys use .at() so a missing key throws; malformed launcher
// events throw std::invalid_argument.
#include "rules/robosub_2026/robosub_2026.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace robotics::rules {
namespace {
using session::Event;
using session::Events;
using session::Json;
using Award = std::function<void(const std::string &, std::int64_t)>;

bool truthy(const Json &v) {
    if (v.is_null())
        return false;
    if (v.is_boolean())
        return v.get<bool>();
    if (v.is_number())
        return v.get<double>() != 0.0;
    return !v.empty(); // string, array, object
}
std::int64_t integer(const Json &v) { // integer arithmetic (bool counts as 0/1)
    if (v.is_boolean())
        return v.get<bool>() ? 1 : 0;
    if (v.is_number_integer())
        return v.get<std::int64_t>();
    throw std::invalid_argument("expected an integer value");
}
double real(const Json &v) {
    if (!v.is_number())
        throw std::invalid_argument("expected a number");
    return v.get<double>();
}
bool listed(const Json &value, const Json &list) {
    return std::find(list.begin(), list.end(), value) != list.end();
}
Json strOf(const Json &v) {
    return v.is_string() ? v : Json(v.is_null() ? "None" : v.dump());
}

// Insertion-ordered map with Json keys.
template <class V> struct Ordered {
    std::vector<std::pair<Json, V>> items;
    V *find(const Json &key) {
        for (auto &item : items)
            if (item.first == key)
                return &item.second;
        return nullptr;
    }
    V &insert(const Json &key, V value) {
        items.emplace_back(key, std::move(value));
        return items.back().second;
    }
    std::size_t size() const {
        return items.size();
    }
};

bool forward(const Event &event) {
    if (!(event.at("task") == "gate" && event.at("region") == "gate_opening" && event.at("id") == "forward_pass" &&
          event.at("type") == "pass_through"))
        return false;
    const Json &data = event.at("data");
    return data.at("attempt_id") > 0 && data.at("from_side") == "positive" && data.at("to_side") == "negative";
}

Json projectileId(const Json &data) {
    auto it = data.find("projectile_id");
    if (it == data.end() || !it->is_number_integer() || it->get<std::int64_t>() < 0)
        throw std::invalid_argument("launcher event requires a nonnegative integer projectile_id");
    return *it;
}
double distance(const Json &data) {
    auto it = data.find("release_distance_m");
    if (it == data.end() || !it->is_number() || !std::isfinite(it->get<double>()) || it->get<double>() < 0)
        throw std::invalid_argument("launcher release requires a finite nonnegative release_distance_m");
    return it->get<double>();
}

std::pair<std::string, int> selectRole(const Event &event, const Json &state, const Json &parameters) {
    double y = real(event.at("data").at("crossing_point_local").at(1));
    const auto &reference = parameters.at("gate").at("role_reference_frame").get_ref<const std::string &>();
    double repair_y = real(state.at("tasks").at("gate").at("frames").at(reference).at("position_m").at(1));
    return {y * repair_y > 0 ? "repair" : "rescue", y > 0 ? 1 : -1};
}

Json valueOr(const Json &object, const char *key, Json fallback) {
    auto it = object.find(key);
    return it == object.end() ? std::move(fallback) : *it;
}

class Ledger {
  public:
    Ledger(const Json &state, const Json &parameters, Award award)
        : state_(state), parameters_(parameters), points_(parameters.at("points")), award_(std::move(award)) {}

    std::optional<std::string> role;
    int side{0};
    bool ended{false};
    Events *emit{nullptr};
    std::map<Json, Json> contents;

    void feed(const Event &event) {
        if (ended)
            return;
        const Json &task = event.at("task"), &kind = event.at("type"), &data = event.at("data");
        if (forward(event) && !role) {
            auto [r, s] = selectRole(event, state_, parameters_);
            role = r, side = s;
        } else if (kind == "breach" && task == "surface") {
            ended = true;
            if (emit)
                emit->push_back({{"id", "surface:scoring_ended"},
                                 {"type", "scoring_ended"},
                                 {"task", task},
                                 {"region", event.at("region")},
                                 {"time_ns", event.at("time_ns")},
                                 {"data", {{"reason", "breach outside octagon"}}}});
            return;
        } else if (task == "slalom" && kind == "pass_through") {
            slalom(event);
        } else if (task == "bins") {
            bins(event);
        } else if (task == "surface") {
            if (kind == "surface_reached") {
                surface_active_ = true;
            } else if (kind == "surface_lost") {
                surface_active_ = false, facing_ = Json();
            } else if (kind == "facing_reached") {
                facing_ = data.at("target");
            } else if (kind == "facing_lost") {
                facing_ = Json();
            } else if (kind == "rotation_judged") {
                basketTurns(real(data.at("turns")));
            }
        } else if (task == "table") {
            table(event);
        }
        surfaceRows();
    }

  private:
    const Json &state_, &parameters_, &points_;
    Award award_;
    Ordered<Json> shots_; // {eligible, result, target, correct}
    std::set<Json> lights_, grasped_, dropped_, surfaced_;
    std::map<Json, std::int64_t> basket_awards_;
    Json held_, facing_; // null = None
    bool surface_active_{false};

    std::int64_t pt(const char *key) const {
        return integer(points_.at(key));
    }
    Json targetClass() const {
        return strOf(parameters_.at("roles").at(*role).at("target_class"));
    }

    void slalom(const Event &event) {
        const Json &data = event.at("data");
        const Json &region = event.at("region");
        if (!role || !listed(region, parameters_.at("slalom").at("rows")) || data.at("from_side") != "positive" ||
            data.at("to_side") != "negative")
            return;
        int s = real(data.at("crossing_point_local").at(1)) > 0 ? 1 : -1;
        award_(region.get<std::string>(), (s == side ? pt("slalom_same") : pt("slalom_other")) +
                                              pt("slalom_depth") * (truthy(data.at("depth_overlap")) ? 1 : 0));
    }

    void bins(const Event &event) {
        const Json &data = event.at("data");
        const Json &type = event.at("type");
        if (type == "activate") {
            if (role) {
                lights_.insert(event.at("region"));
                award_("lights", pt("light") * std::min<std::int64_t>(pt("max_lights"), lights_.size()));
            }
            return;
        }
        if (valueOr(data, "mechanism_type", Json()) != parameters_.at("bins").at("mechanism_type"))
            return;
        Json identifier = valueOr(data, "projectile_id", Json());
        if (type == "payload_released") {
            if (!shots_.find(identifier) && static_cast<std::int64_t>(shots_.size()) < pt("max_shots"))
                shots_.insert(
                    identifier,
                    {{"eligible", role.has_value()}, {"result", nullptr}, {"target", ""}, {"correct", false}});
        } else if (type == "payload_landing" && data.at("outcome") == "inside") {
            Json *shot = shots_.find(identifier);
            if (!role || !shot || !truthy(shot->at("eligible")) || truthy(shot->at("result")))
                return;
            bool correct = data.at("region_class") == targetClass();
            (*shot)["result"] = correct ? "success" : "wrong_target";
            (*shot)["correct"] = correct;
            (*shot)["target"] = event.at("region");
            std::int64_t good = 0;
            std::set<Json> unique;
            for (auto &item : shots_.items) {
                const Json &r = item.second.at("result");
                if (r == "success" || r == "wrong_target") {
                    ++good;
                    if (item.second.at("correct").get<bool>())
                        unique.insert(item.second.at("target"));
                }
            }
            award_("bins", pt("bin") * good + pt("bin_class") * static_cast<std::int64_t>(unique.size()));
        }
    }

    void table(const Event &event) {
        const Json &data = event.at("data");
        const Json &type = event.at("type");
        Json prop = valueOr(data, "prop_id", Json());
        if (type == "attach") {
            held_ = prop;
            if (role) {
                grasped_.insert(prop);
                contents.erase(prop);
            }
        } else if (type == "detach") {
            if (held_ == prop)
                held_ = Json();
            if (role && grasped_.count(prop) && valueOr(data, "reason", "released") == "released") {
                dropped_.insert(prop);
                award_("objects_drop", pt("object_drop") * static_cast<std::int64_t>(dropped_.size()));
            }
        } else if (type == "drop_into" && role &&
                   listed(valueOr(data, "basket", Json()), parameters_.at("table").at("baskets"))) {
            contents[prop] = data.at("basket");
            std::int64_t value =
                data.at("basket") == data.at("expected_basket") ? pt("basket_correct") : pt("basket_other");
            auto [it, inserted] = basket_awards_.emplace(prop, value);
            if (!inserted)
                it->second = std::max(value, it->second);
            std::int64_t sum = 0;
            for (auto &entry : basket_awards_)
                sum += entry.second;
            award_("baskets", sum);
        }
    }

    void basketTurns(double turns) {
        double count = static_cast<double>(contents.size());
        if (role && count > 0 && turns > 0)
            award_("basket_count", turns == count                  ? pt("basket_count")
                                   : std::fabs(turns - count) == 1 ? pt("basket_count_near")
                                                                   : 0);
    }

    void surfaceRows() {
        if (!role || !surface_active_)
            return;
        award_("surface", pt("surface"));
        if (!held_.is_null() && grasped_.count(held_)) {
            surfaced_.insert(held_);
            award_("objects_surface", pt("object_surface") * static_cast<std::int64_t>(surfaced_.size()));
        }
        if (!facing_.is_null()) {
            const Json &icons = parameters_.at("roles").at(*role).at("facing_icons");
            std::int64_t value = listed(facing_, icons) ? pt("facing_correct") : pt("facing_other");
            std::size_t count = contents.size();
            if (count > 0 && facing_ == icons.at(std::min<std::size_t>(count, 2) - 1))
                value = pt("facing_count");
            award_("facing", value);
        }
    }
};

const char *const kEndedReason = "Breach outside octagon; scoring ended (timer remains manual)";

Ledger replayed(const Json &state, const Json &parameters) {
    Ledger ledger(state, parameters, [](const std::string &, std::int64_t) {});
    const Json &started = state.at("run").at("started_ns");
    for (const Json &event : state.at("history"))
        if (event.at("time_ns") >= started && !ledger.ended)
            ledger.feed(event);
    return ledger;
}

class Robosub2026 final : public session::Rules {
  public:
    using session::Rules::feed;

    Json evaluate(const Json &state, const Events &events, const Json &parameters) override {
        Json scores_out = Json::array();
        Events emitted;
        const Json &run = state.at("run");
        if (!truthy(run.at("running")) || truthy(run.at("ended")))
            return {{"scores", scores_out}, {"events", Json::array()}};
        const Json &options = run.at("options"), &points = parameters.at("points");
        const Json &started = run.at("started_ns");
        std::vector<const Json *> history;
        for (const Json &event : state.at("history"))
            if (event.at("time_ns") >= started)
                history.push_back(&event);
        const Json *selected = nullptr;
        for (const Json *event : history)
            if (event->at("type") == "role_selected" && event->at("task") == "gate") {
                selected = event;
                break;
            }
        std::optional<std::string> role;
        if (selected)
            role = selected->at("data").at("role").get<std::string>();

        Ordered<Json> shots;
        for (const Json *event : history) {
            if (event->at("task") != "torpedo")
                continue;
            const Json &data = event->at("data");
            if (event->at("type") == "shot_registered") {
                if (!shots.find(data.at("projectile_id"))) {
                    Json shot = data;
                    shot["result"] = nullptr;
                    shots.insert(data.at("projectile_id"), shot);
                }
            } else if (event->at("type") == "shot_result") {
                Json *shot = shots.find(data.at("projectile_id"));
                if (shot && shot->at("result").is_null())
                    shot->update(data);
            }
        }
        std::set<Json> passed;
        if (selected)
            for (const Json *event : history)
                if (event->at("time_ns") >= selected->at("time_ns") && forward(*event))
                    passed.insert(event->at("data").at("attempt_id"));

        Json scores = state.at("scores");
        std::vector<std::pair<std::string, std::int64_t>> updated;
        Award award = [&](const std::string &row, std::int64_t value) {
            auto it = scores.find(row);
            std::int64_t current = it == scores.end() ? 0 : integer(*it);
            if (value <= current)
                return;
            scores[row] = value;
            for (auto &entry : updated)
                if (entry.first == row) {
                    entry.second = value;
                    return;
                }
            updated.emplace_back(row, value);
        };
        auto gatePoints = [&](std::int64_t style) {
            std::int64_t bonus = truthy(options.at("role_coin")) && role && Json(*role) == options.at("role")
                                     ? integer(points.at("role"))
                                     : 0;
            return integer(points.at("gate")) + integer(points.at("heading")) * integer(options.at("heading_coin")) +
                   bonus + style;
        };
        auto emitShot = [&](const Event &event, const char *kind, Json data) {
            emitted.push_back({{"id", std::string("torpedo:") + kind},
                               {"type", kind},
                               {"task", "torpedo"},
                               {"region", event.at("region")},
                               {"time_ns", event.at("time_ns")},
                               {"data", std::move(data)}});
        };
        auto awardTorpedoes = [&]() {
            const Json &rules = parameters.at("torpedo");
            const Json &goodResults = rules.at("good_results");
            std::vector<const Json *> good;
            for (auto &item : shots.items)
                if (listed(item.second.at("result"), goodResults))
                    good.push_back(&item.second);
            award("torpedoes", integer(points.at("torpedo")) * static_cast<std::int64_t>(good.size()));
            std::int64_t bonus = 0;
            for (const Json *shot : good) {
                double d = real(shot->at("release_distance_m"));
                bonus += d >= real(points.at("distance_far_m"))    ? integer(points.at("distance_far"))
                         : d >= real(points.at("distance_near_m")) ? integer(points.at("distance_near"))
                                                                   : 0;
            }
            award("distance", bonus);
            const Json &order = rules.at("sequence_order");
            if (shots.size() != order.size())
                return;
            for (auto &item : shots.items)
                if (!listed(item.second.at("result"), goodResults) || !truthy(item.second.at("correct")))
                    return;
            Json sizes = Json::array();
            for (auto &item : shots.items)
                sizes.push_back(item.second.at("hole_size"));
            if (sizes == order)
                award("sequence", integer(points.at("sequence")));
        };

        auto handle = [&](const Event &event) {
            const Json &data = event.at("data");
            if (event.at("task") == "torpedo" && valueOr(data, "mechanism_type", Json()) == "launcher") {
                if (event.at("type") == "payload_released" && event.at("id") == "payload_released") {
                    Json identifier = projectileId(data);
                    double d = distance(data);
                    if (!shots.find(identifier) &&
                        static_cast<std::int64_t>(shots.size()) < integer(points.at("max_shots"))) {
                        Json registered = {{"projectile_id", identifier},
                                           {"mechanism_type", "launcher"},
                                           {"release_distance_m", d},
                                           {"eligible", role.has_value()}};
                        Json shot = registered;
                        shot["result"] = nullptr;
                        shots.insert(identifier, shot);
                        emitShot(event, "shot_registered", registered);
                    }
                } else if (event.at("type") == "hit" && event.at("region") == "panel" &&
                           (event.at("id") == "hole_pass" || event.at("id") == "panel_blocked")) {
                    Json identifier = projectileId(data);
                    Json outcome = valueOr(data, "outcome", Json());
                    if (outcome != "pass" && outcome != "blocked" && outcome != "timeout")
                        throw std::invalid_argument("launcher hit requires pass, blocked or timeout outcome");
                    if ((outcome == "pass") != (event.at("id") == "hole_pass"))
                        throw std::invalid_argument("launcher hit id differs from its outcome");
                    if (outcome == "pass")
                        for (const char *key : {"hole_id", "hole_class", "hole_size"}) {
                            Json v = valueOr(data, key, Json());
                            if (!v.is_string() || v.get_ref<const std::string &>().empty())
                                throw std::invalid_argument(
                                    "launcher passage requires hole_id, hole_class and hole_size");
                        }
                    Json *shot = shots.find(identifier);
                    if (role && shot && truthy(shot->at("eligible")) && shot->at("result").is_null()) {
                        bool correct =
                            valueOr(data, "hole_class", Json()) == parameters.at("roles").at(*role).at("target_class");
                        Json accepted = {
                            {"projectile_id", identifier},
                            {"mechanism_type", "launcher"},
                            {"result", outcome == "pass" ? Json(correct ? "success" : "wrong_target") : outcome},
                            {"correct", correct},
                            {"hole_id", valueOr(data, "hole_id", "")},
                            {"hole_size", valueOr(data, "hole_size", "")}};
                        shot->update(accepted);
                        emitShot(event, "shot_result", accepted);
                        awardTorpedoes();
                    }
                }
                return;
            }
            if (event.at("task") != "gate" || event.at("region") != "gate_opening")
                return;
            if (forward(event)) {
                if (!role) {
                    double y = real(data.at("crossing_point_local").at(1));
                    const auto &reference =
                        parameters.at("gate").at("role_reference_frame").get_ref<const std::string &>();
                    double repair_y =
                        real(state.at("tasks").at("gate").at("frames").at(reference).at("position_m").at(1));
                    role = y * repair_y > 0 ? "repair" : "rescue";
                    emitted.push_back(
                        {{"id", "gate:role_selected"},
                         {"type", "role_selected"},
                         {"task", "gate"},
                         {"region", "gate_opening"},
                         {"time_ns", event.at("time_ns")},
                         {"data", {{"role", *role}, {"side", y > 0 ? 1 : -1}, {"attempt_id", data.at("attempt_id")}}}});
                }
                passed.insert(data.at("attempt_id"));
                award("gate", gatePoints(0));
            } else if (event.at("id") == "gate_opening:attempt_finished" && event.at("type") == "attempt_finished" &&
                       role && passed.count(data.at("attempt_id"))) {
                const Json &style = parameters.at("gate").at("style");
                std::vector<std::int64_t> quarters;
                for (const Json &turn : data.at("rotation_vector_body"))
                    quarters.push_back(static_cast<std::int64_t>(std::floor(
                        (std::fabs(real(turn)) + real(style.at("epsilon_rad"))) / real(style.at("quarter_rad")))));
                std::int64_t limit = integer(style.at("max_quarters"));
                std::int64_t q0 = quarters.at(0), q1 = quarters.at(1), q2 = quarters.at(2), rp, yaw;
                if (truthy(style.at("roll_pitch_first"))) {
                    rp = std::min(limit, q0 + q1);
                    yaw = std::min(limit - rp, q2);
                } else {
                    yaw = std::min(limit, q2);
                    rp = std::min(limit - yaw, q0 + q1);
                }
                award("gate", gatePoints(integer(points.at("style_rp")) * rp + integer(points.at("style_yaw")) * yaw));
            } else if (event.at("id") == "reverse_pass" && event.at("type") == "pass_through" &&
                       data.at("from_side") == "negative" && data.at("to_side") == "positive" && role &&
                       data.at("envelope_top_world") < state.at("environment").at("surface_z_m")) {
                award("home", integer(points.at("home")));
            }
        };

        Ledger ledger(state, parameters, award);
        for (const Json *event : history)
            ledger.feed(*event);
        ledger.emit = &emitted;
        for (const Event &event : events) {
            if (event.at("time_ns") < started || ledger.ended)
                continue;
            handle(event);
            ledger.feed(event);
        }
        for (auto &entry : updated)
            scores_out.push_back({{"row", entry.first}, {"points", entry.second}});
        return {{"scores", scores_out}, {"events", Json(emitted)}};
    }

    Json describe(const Json &state, const Json &parameters) override {
        Json intended = state.at("run").at("options").at("role");
        Ledger ledger = replayed(state, parameters);
        const Json &scores = state.at("scores");
        auto score = [&](const std::string &row) {
            auto it = scores.find(row);
            return it != scores.end() && truthy(*it);
        };
        bool anySlalom = false;
        for (const Json &row : parameters.at("slalom").at("rows"))
            anySlalom = anySlalom || score(row.get<std::string>());
        Json role = ledger.role ? Json(*ledger.role) : Json();
        const Json &key = ledger.role ? role : intended;
        // Coin-flip bonuses earned at the gate; null until the gate is passed.
        const Json &options = state.at("run").at("options"), &points = parameters.at("points");
        Json heading_bonus, role_bonus;
        if (ledger.role) {
            heading_bonus = integer(points.at("heading")) * (truthy(options.at("heading_coin")) ? 1 : 0);
            role_bonus = truthy(options.at("role_coin")) && role == intended ? integer(points.at("role")) : 0;
        }
        return {{"intended_role", intended},
                {"role", role},
                {"heading_bonus", heading_bonus},
                {"role_bonus", role_bonus},
                {"target_class", parameters.at("roles").at(key.get<std::string>()).at("target_class")},
                {"gate_passed", ledger.role.has_value()},
                {"ended_reason", ledger.ended ? kEndedReason : ""},
                {"basket_count", ledger.contents.size()},
                {"pinger", {{"mode", "disabled"}, {"first", nullptr}, {"active", nullptr}, {"stage", 0}}},
                {"time_bonus_eligible", score("surface") && anySlalom && (score("bins") || score("torpedoes"))}};
    }

    Json feed(const Events &, const Json &, const Json &) override {
        throw std::logic_error("robosub_2026 feed needs the run state: call feed(state, ...)");
    }

    Json feed(const Json &state, const Events &events, const Json &context, const Json &parameters) override {
        const Json &spec = parameters.at("feed");
        const Json &kinds = spec.at("kinds"), &baskets = spec.at("basket_names");
        Json targetClass = describe(state, parameters).at("target_class");
        const Json &released = context.at("payloads");
        std::set<std::tuple<std::string, Json, Json>> seen;
        Json items = Json::array();

        auto add = [&](const Json &identifier, const std::string &kind, const Json &result, const Json &target,
                       const Json &time_ns, const Json &slot = Json()) {
            if (!seen.emplace(kind, identifier, result).second)
                return;
            Json item = {{"id", identifier},
                         {"kind", kind},
                         {"result", result},
                         {"target", target},
                         {"time", real(time_ns) / 1e9}};
            if (!slot.is_null())
                item["slot"] = slot;
            items.push_back(item);
        };
        auto isKind = [&](const Json &mechanism) {
            return mechanism.is_string() && kinds.contains(mechanism.get<std::string>());
        };
        for (const Event &event : events) {
            const Json &kind = event.at("type"), &data = event.at("data"), &time_ns = event.at("time_ns");
            Json mechanism = valueOr(data, "mechanism_type", Json());
            Json identifier = valueOr(data, "projectile_id", Json());
            Json slot;
            if (identifier.is_number_integer()) {
                auto it = released.find(std::to_string(identifier.get<std::int64_t>()));
                if (it != released.end())
                    slot = valueOr(*it, "slot", Json());
            }
            auto kindOf = [&] { return kinds.at(mechanism.get<std::string>()).get<std::string>(); };
            auto holeId = [&] {
                Json v = valueOr(data, "hole_id", Json());
                return truthy(v) ? v : Json("");
            };
            if (kind == "payload_released" && isKind(mechanism)) {
                add(identifier, kindOf(), "released", "", time_ns, slot);
            } else if (kind == "hit" && isKind(mechanism)) {
                if (data.at("outcome") == "pass")
                    add(identifier, kindOf(),
                        valueOr(data, "hole_class", Json()) == targetClass ? "success" : "wrong_target", holeId(),
                        time_ns, slot);
                else if (data.at("outcome") == "blocked")
                    add(identifier, kindOf(), "blocked", holeId(), time_ns, slot);
            } else if (kind == "payload_landing" && isKind(mechanism)) {
                Json result = "blocked";
                if (data.at("outcome") == "inside" && mechanism == parameters.at("bins").at("mechanism_type"))
                    result = data.at("region_class") == targetClass ? "success" : "wrong_target";
                add(identifier, kindOf(), result, event.at("region"), time_ns, slot);
            } else if (kind == "miss" && isKind(mechanism)) {
                add(identifier, kindOf(), "miss", strOf(valueOr(data, "reason", "")), time_ns, slot);
            } else if (kind == "activate") {
                add("", spec.at("magnet_kind").get<std::string>(), "activated", event.at("region"), time_ns);
            } else if (kind == "attach" && event.at("task") == "table") {
                add(data.at("prop_id"), kinds.at("claw").get<std::string>(), "grasped", "", time_ns);
            } else if (kind == "detach" && event.at("task") == "table") {
                add(data.at("prop_id"), kinds.at("claw").get<std::string>(), valueOr(data, "reason", "released"), "",
                    time_ns);
            } else if (kind == "drop_into" && event.at("id") == "basket_drop") {
                Json result = data.at("basket") == data.at("expected_basket") ? "success" : "wrong_target";
                Json target = data.at("basket");
                if (target.is_string() && baskets.contains(target.get<std::string>()))
                    target = baskets.at(target.get<std::string>());
                add(data.at("prop_id"), kinds.at("claw").get<std::string>(), result, target, time_ns);
            }
        }
        return items;
    }
};
} // namespace

std::unique_ptr<session::Rules> makeRobosub2026() {
    return std::make_unique<Robosub2026>();
}
} // namespace robotics::rules

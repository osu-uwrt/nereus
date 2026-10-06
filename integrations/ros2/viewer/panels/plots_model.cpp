// Saved plots and layout plots as YAML: the spec in model.hpp, strict on read (unknown keys refused).
#include "nereus/ros_viewer/plots/model.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>
#include <stdexcept>

namespace nereus::ros_viewer::plots {
namespace {

// Throws unless `node` is a map whose keys are all in `allowed`.
void only(const YAML::Node &node, std::initializer_list<const char *> allowed, const std::string &where) {
    if (!node.IsMap())
        throw std::invalid_argument(where + " must be a map");
    for (const auto &item : node) {
        const auto key = item.first.as<std::string>();
        if (std::none_of(allowed.begin(), allowed.end(), [&](const char *k) { return key == k; }))
            throw std::invalid_argument(where + ": unknown key '" + key + "'");
    }
}

std::string text(const YAML::Node &node, const char *key, const std::string &where, bool required = false) {
    const auto value = node[key];
    if (!value) {
        if (required)
            throw std::invalid_argument(where + ": needs '" + key + "'");
        return {};
    }
    if (!value.IsScalar())
        throw std::invalid_argument(where + "." + key + " must be text");
    return value.as<std::string>();
}

YAML::Node sourceYaml(const Source &s) {
    YAML::Node node(YAML::NodeType::Map);
    if (s.kind == Source::Kind::Figure)
        node["figure"] = s.figure;
    else {
        node["topic"] = s.topic;
        node["field"] = s.field;
    }
    return node;
}

Source sourceFrom(const YAML::Node &node, const std::string &where) {
    only(node, {"topic", "field", "figure"}, where);
    Source s;
    if (node["figure"]) {
        if (node["topic"] || node["field"])
            throw std::invalid_argument(where + ": a figure has no topic or field");
        s.kind = Source::Kind::Figure;
        s.figure = text(node, "figure", where, true);
        if (s.figure.empty())
            throw std::invalid_argument(where + ": empty figure");
        return s;
    }
    s.topic = text(node, "topic", where, true);
    s.field = text(node, "field", where, true);
    if (s.topic.empty() || s.field.empty())
        throw std::invalid_argument(where + ": needs a topic and a field");
    return s;
}

std::string hex(uint32_t rgb) {
    char out[8];
    std::snprintf(out, sizeof(out), "#%06x", rgb & 0xffffffu);
    return out;
}

uint32_t parseHex(const std::string &value, const std::string &where) {
    if (value.size() != 7 || value[0] != '#')
        throw std::invalid_argument(where + ": \"" + value + "\" is not a #rrggbb color");
    uint32_t rgb = 0;
    for (std::size_t i = 1; i < 7; ++i) {
        const char c = value[i];
        const int digit = c >= '0' && c <= '9'   ? c - '0'
                          : c >= 'a' && c <= 'f' ? c - 'a' + 10
                          : c >= 'A' && c <= 'F' ? c - 'A' + 10
                                                 : -1;
        if (digit < 0)
            throw std::invalid_argument(where + ": \"" + value + "\" is not a #rrggbb color");
        rgb = rgb * 16 + uint32_t(digit);
    }
    return rgb;
}

double number(const YAML::Node &node, const std::string &where) {
    try {
        const double v = node.as<double>();
        if (!std::isfinite(v))
            throw std::invalid_argument("");
        return v;
    } catch (const std::exception &) {
        throw std::invalid_argument(where + " must be a number");
    }
}

} // namespace

std::string Source::key() const {
    return kind == Kind::Figure ? "figure:" + figure : "topic:" + topic + "#" + field;
}

YAML::Node toYaml(const PlotSpec &spec) {
    YAML::Node root(YAML::NodeType::Map);
    root["title"] = spec.title;
    root["span_s"] = spec.span;
    root["time"] = spec.time == TimeBase::Header ? "header" : "receipt";
    if (spec.ownTime)
        root["own_time"] = true;
    YAML::Node lanes(YAML::NodeType::Sequence);
    for (const auto &lane : spec.lanes) {
        YAML::Node l(YAML::NodeType::Map);
        l["label"] = lane.label;
        if (!lane.unit.empty())
            l["unit"] = lane.unit;
        if (lane.limits) {
            YAML::Node limits(YAML::NodeType::Sequence);
            limits.push_back(lane.limits->first);
            limits.push_back(lane.limits->second);
            limits.SetStyle(YAML::EmitterStyle::Flow);
            l["limits"] = limits;
        }
        YAML::Node series(YAML::NodeType::Sequence);
        for (const auto &s : lane.series) {
            YAML::Node n(YAML::NodeType::Map);
            n["label"] = s.label;
            if (s.minus) {
                YAML::Node derive(YAML::NodeType::Map);
                derive["a"] = sourceYaml(s.source);
                derive["b"] = sourceYaml(*s.minus);
                n["derive"] = derive;
            } else
                n["source"] = sourceYaml(s.source);
            n["slot"] = s.slot;
            if (s.color)
                n["color"] = hex(*s.color);
            if (s.steps)
                n["style"] = "steps";
            if (s.hidden)
                n["hidden"] = true;
            series.push_back(n);
        }
        l["series"] = series;
        lanes.push_back(l);
    }
    root["lanes"] = lanes;
    return root;
}

PlotSpec fromYaml(const YAML::Node &root, const std::string &where) {
    only(root, {"title", "span_s", "time", "own_time", "lanes"}, where);
    PlotSpec spec;
    spec.title = text(root, "title", where);
    if (spec.title.empty())
        spec.title = "Plot";
    if (root["span_s"]) {
        spec.span = number(root["span_s"], where + ".span_s");
        if (spec.span <= 0 || spec.span > 3600)
            throw std::invalid_argument(where + ".span_s must be in (0, 3600]");
    }
    if (const auto time = text(root, "time", where); !time.empty()) {
        if (time != "header" && time != "receipt")
            throw std::invalid_argument(where + ".time must be header or receipt");
        spec.time = time == "header" ? TimeBase::Header : TimeBase::Receipt;
    }
    if (root["own_time"])
        spec.ownTime = root["own_time"].as<bool>();
    const auto lanes = root["lanes"];
    if (lanes && !lanes.IsSequence())
        throw std::invalid_argument(where + ".lanes must be a list");
    for (std::size_t i = 0; lanes && i < lanes.size(); ++i) {
        const std::string at = where + ".lanes[" + std::to_string(i) + "]";
        only(lanes[i], {"label", "unit", "limits", "series"}, at);
        Lane lane;
        lane.label = text(lanes[i], "label", at);
        lane.unit = text(lanes[i], "unit", at);
        if (const auto limits = lanes[i]["limits"]) {
            if (!limits.IsSequence() || limits.size() != 2)
                throw std::invalid_argument(at + ".limits must be [low, high]");
            const double low = number(limits[0], at + ".limits"), high = number(limits[1], at + ".limits");
            if (!(high > low))
                throw std::invalid_argument(at + ".limits must rise");
            lane.limits = std::make_pair(low, high);
        }
        const auto series = lanes[i]["series"];
        if (series && !series.IsSequence())
            throw std::invalid_argument(at + ".series must be a list");
        for (std::size_t j = 0; series && j < series.size(); ++j) {
            const std::string sat = at + ".series[" + std::to_string(j) + "]";
            only(series[j], {"label", "source", "derive", "slot", "color", "style", "hidden"}, sat);
            Series s;
            s.label = text(series[j], "label", sat);
            if (const auto derive = series[j]["derive"]) {
                if (series[j]["source"])
                    throw std::invalid_argument(sat + ": a source or a derive, not both");
                only(derive, {"a", "b"}, sat + ".derive");
                if (!derive["a"] || !derive["b"])
                    throw std::invalid_argument(sat + ".derive needs a and b");
                s.source = sourceFrom(derive["a"], sat + ".derive.a");
                s.minus = sourceFrom(derive["b"], sat + ".derive.b");
            } else if (series[j]["source"])
                s.source = sourceFrom(series[j]["source"], sat + ".source");
            else
                throw std::invalid_argument(sat + ": needs a source");
            if (series[j]["slot"]) {
                s.slot = series[j]["slot"].as<int>(0);
                if (s.slot < 0 || s.slot >= 4 * kSlots)
                    throw std::invalid_argument(sat + ".slot out of range");
            } else
                s.slot = freeSlot(lane);
            if (const auto color = text(series[j], "color", sat); !color.empty())
                s.color = parseHex(color, sat + ".color");
            if (const auto style = text(series[j], "style", sat); !style.empty()) {
                if (style != "steps" && style != "lines")
                    throw std::invalid_argument(sat + ".style must be steps or lines");
                s.steps = style == "steps";
            }
            if (series[j]["hidden"])
                s.hidden = series[j]["hidden"].as<bool>();
            if (s.label.empty())
                s.label = s.source.kind == Source::Kind::Figure ? s.source.figure : fieldLabel(s.source.field);
            lane.series.push_back(std::move(s));
        }
        spec.lanes.push_back(std::move(lane));
    }
    return spec;
}

std::string toLine(const PlotSpec &spec) {
    YAML::Emitter out;
    out.SetMapFormat(YAML::Flow);
    out.SetSeqFormat(YAML::Flow);
    out << toYaml(spec);
    std::string line = out.c_str();
    std::replace(line.begin(), line.end(), '\n', ' ');
    return line;
}

PlotSpec fromLine(const std::string &line, const std::string &where) {
    YAML::Node node;
    try {
        node = YAML::Load(line);
    } catch (const YAML::Exception &error) {
        throw std::invalid_argument(where + ": " + error.what());
    }
    return fromYaml(node, where);
}

int freeSlot(const Lane &lane) {
    std::set<int> used;
    for (const auto &s : lane.series)
        used.insert(s.slot);
    for (int slot = 0;; ++slot)
        if (!used.count(slot))
            return slot;
}

bool moveSeries(PlotSpec &from, std::size_t lane, std::size_t index, PlotSpec &to, int toLane) {
    if (lane >= from.lanes.size() || index >= from.lanes[lane].series.size())
        return false;
    if (&from == &to && toLane == int(lane))
        return false;
    Series moved = from.lanes[lane].series[index];
    const std::string unit = from.lanes[lane].unit;
    from.lanes[lane].series.erase(from.lanes[lane].series.begin() + long(index));
    if (toLane < 0 || std::size_t(toLane) >= to.lanes.size()) {
        Lane fresh;
        fresh.label = moved.label;
        fresh.unit = unit;
        if (!moved.color)
            moved.slot = 0;
        fresh.series.push_back(moved);
        to.lanes.push_back(fresh);
    } else {
        auto &target = to.lanes[std::size_t(toLane)];
        if (!moved.color)
            for (const auto &s : target.series)
                if (!s.color && s.slot == moved.slot) {
                    moved.slot = freeSlot(target);
                    break;
                }
        if (target.unit.empty())
            target.unit = unit;
        target.series.push_back(moved);
    }
    // The emptied lane goes last: every index above still pointed at the lanes as they were.
    if (from.lanes[lane].series.empty())
        from.lanes.erase(from.lanes.begin() + long(lane));
    return true;
}

void setSlot(Lane &lane, std::size_t index, int slot) {
    if (index >= lane.series.size())
        return;
    auto &target = lane.series[index];
    for (std::size_t i = 0; i < lane.series.size(); ++i)
        if (i != index && lane.series[i].slot == slot && !lane.series[i].color) {
            lane.series[i].slot = target.slot;
            break;
        }
    target.slot = slot;
    target.color.reset();
}

std::string fieldLabel(const std::string &field) {
    const auto dot = field.rfind('.');
    if (dot == std::string::npos)
        return field;
    const std::string last = field.substr(dot + 1);
    const bool axis = last == "x" || last == "y" || last == "z" || last == "w";
    if (!axis)
        return last;
    const auto before = field.rfind('.', dot - 1);
    return field.substr(before == std::string::npos ? 0 : before + 1);
}

std::string fieldUnit(const std::string &field) {
    const auto dot = field.rfind('.');
    const std::string last = dot == std::string::npos ? field : field.substr(dot + 1);
    if (last == "roll" || last == "pitch" || last == "yaw")
        return "°";
    return {};
}

} // namespace nereus::ros_viewer::plots

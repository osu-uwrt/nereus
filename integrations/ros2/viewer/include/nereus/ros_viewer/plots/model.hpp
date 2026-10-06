// What a plot is, as saved: lanes of series and where each series' numbers come from. No ROS, no ImGui, so saved
// plots and the layout's plot windows round-trip in tests. A saved plot is a definition, never data.
#pragma once
#include "nereus/ros_viewer/plots/series.hpp"
#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace nereus::ros_viewer::plots {

// Where a series' numbers come from: a numeric field of a ROS topic, or a figure a panel shows (Motion's Z actual,
// a telemetry reading), sampled from the panel's provider.
struct Source {
    enum class Kind { Topic, Figure };
    Kind kind = Kind::Topic;
    std::string topic, field; // Topic: the topic (relative to the robot namespace, or absolute) and field path
    std::string figure;       // Figure: the figure's id ("motion.z.actual", "telemetry.fog")

    // A stable identity, shared by every series reading the same numbers.
    std::string key() const;
    bool operator==(const Source &o) const {
        return key() == o.key();
    }
};

// Series past this many in one lane reuse the theme's colors, drawn dashed.
constexpr int kSlots = 8;

// One line: its source (or the difference of two sources), and how it is drawn.
struct Series {
    std::string label;
    Source source;
    std::optional<Source> minus;   // derived: source minus this (Z error = Commanded - Actual)
    int slot = 0;                  // theme color slot; slots >= kSlots repeat the colors dashed
    std::optional<uint32_t> color; // a custom color, 0xRRGGBB (overrides the slot)
    bool steps = false;            // drawn as steps (setpoints) rather than lines
    bool hidden = false;           // still recorded, not drawn
};

// A plot lane: its own value axis, shared time axis. Limits, when set, become the lane's outer ticks.
struct Lane {
    std::string label, unit;
    std::vector<Series> series;
    std::optional<std::pair<double, double>> limits;
};

struct PlotSpec {
    std::string title = "Plot";
    double span = 30; // seconds shown
    TimeBase time = TimeBase::Header;
    bool ownTime = false; // off the shared timeline (its own span, pause and scrub)
    std::vector<Lane> lanes;
};

// YAML for saved plots and the layout: fromYaml throws std::invalid_argument (naming `where`) on unknown keys,
// a missing source or a bad value.
YAML::Node toYaml(const PlotSpec &);
PlotSpec fromYaml(const YAML::Node &, const std::string &where);
// One line of flow YAML, and back (the layout ini keeps each plot on one line).
std::string toLine(const PlotSpec &);
PlotSpec fromLine(const std::string &, const std::string &where);

// The color slot a new series in this lane takes: the first slot not in use, then the dashed repeats.
int freeSlot(const Lane &);
// Moves series `index` of lane `lane` in `from` to lane `toLane` of `to` (the same plot or another; -1 or past the
// last lane: a new lane at the end, named for the series). It keeps its color unless its new lane already shows
// that one, and a lane the move empties goes, so dragging a lane's only series onto another merges the two.
// Returns false when nothing moved (an index out of range, or its own lane).
bool moveSeries(PlotSpec &from, std::size_t lane, std::size_t index, PlotSpec &to, int toLane);
// Gives series `index` slot `slot` (clearing a custom color); a series already on that slot takes the old one,
// so a lane never shows two identical lines.
void setSlot(Lane &, std::size_t index, int slot);

// A short label for a field path: its last segment, with the one before when that is an axis letter
// ("position.z", "data[3]", "linear.x", "yaw").
std::string fieldLabel(const std::string &field);
// The unit a field path implies, when it does (angles derived from quaternions are degrees).
std::string fieldUnit(const std::string &field);

} // namespace nereus::ros_viewer::plots

// Plots without ROS traffic: the sample ring and its min / max decimation, axis ticks, saved-plot YAML (round
// trip, strictness), color slots, and the plot windows headless (the [NereusPlots] ini round trip, drawing).
#include "nereus/ros_viewer/plots/model.hpp"
#include "nereus/ros_viewer/plots/plots.hpp"
#include "nereus/ros_viewer/plots/series.hpp"
#include "nereus/ros_viewer/theme.hpp"
#include <cassert>
#include <cmath>
#include <imgui.h>
#include <imgui_internal.h>
#include <iostream>
#include <string>

using namespace nereus::ros_viewer;
using namespace nereus::ros_viewer::plots;

namespace {

bool near(double a, double b, double tolerance = 1e-6) {
    return std::abs(a - b) <= tolerance;
}

template <class F> bool throws(F f) {
    try {
        f();
    } catch (const std::invalid_argument &) {
        return true;
    }
    return false;
}

void ringAndLookup() {
    SeriesBuffer b(5, 100);
    for (int i = 0; i < 8; ++i)
        b.push({double(i), double(i), float(i * 10)});
    assert(b.size() == 5); // capacity keeps the newest
    assert(b.at(0).value == 30 && b.back().value == 70);
    assert(b.lowerBound(5, TimeBase::Header) == 2);
    assert(b.valueAt(5.5, TimeBase::Header)->value == 50); // zero-order hold
    assert(b.valueAt(2, TimeBase::Header) == nullptr);     // before the oldest kept

    SeriesBuffer aged(1000, 10); // ten seconds of history by receipt time
    for (int i = 0; i <= 30; ++i)
        aged.push({double(i), double(i), 1});
    assert(aged.size() == 11 && aged.at(0).receipt == 20);
}

void decimation() {
    // 6000 samples of a sawtooth over 30 s into 300 columns: at most one entry per column, full envelope kept.
    SeriesBuffer b(10000);
    for (int i = 0; i < 6000; ++i) {
        const double t = i * .005;
        b.push({t, t, float(i % 20)});
    }
    const auto d = decimate(b, 0, 30, 300, TimeBase::Receipt, 1);
    assert(d.columns.size() <= 300 && d.count == 6000);
    assert(d.low == 0 && d.high == 19);
    assert(d.columns[0].low == 0 && d.columns[0].high == 19);
    assert(!d.hasLead);

    // A lead sample before the window, and a gap longer than `gap` breaks the line.
    SeriesBuffer g;
    for (const double t : {0.0, 1.0, 2.0, 10.0, 11.0})
        g.push({t, t, float(t)});
    const auto e = decimate(g, .5, 11, 100, TimeBase::Receipt, 3);
    assert(e.hasLead && e.lead.value == 0);
    std::size_t breaks = 0;
    for (const bool b : e.breakBefore)
        breaks += b;
    assert(breaks == 1);

    // Scrolling by a fraction of a column keeps every column's samples: the columns are anchored to absolute
    // time, so a dense line keeps its shape from frame to frame instead of shimmering.
    SeriesBuffer dense(20000);
    for (int i = 0; i < 12000; ++i) {
        const double t = 1000 + i * .0025;
        dense.push({t, t, float(std::sin(t * 7) + (i % 3) * .1)});
    }
    const auto before = decimate(dense, 1005, 1025, 250, TimeBase::Receipt, 1);
    const auto after = decimate(dense, 1005.037, 1025.037, 250, TimeBase::Receipt, 1);
    std::size_t same = 0;
    for (std::size_t k = 0; k + 1 < before.columns.size(); ++k) // a's last column ends at its edge: fewer samples
        for (const auto &cb : after.columns)
            if (const auto &ca = before.columns[k]; ca.t0 == cb.t0) {
                assert(ca.t1 == cb.t1 && ca.low == cb.low && ca.high == cb.high && ca.first == cb.first &&
                       ca.last == cb.last);
                ++same;
            }
    assert(same + 2 >= std::min(before.columns.size(), after.columns.size()));

    // A NaN sample is an explicit gap.
    SeriesBuffer n;
    n.push({0, 0, 1});
    n.push({1, 1, std::numeric_limits<float>::quiet_NaN()});
    n.push({2, 2, 3});
    const auto f = decimate(n, -1, 3, 40, TimeBase::Receipt);
    assert(f.count == 2 && f.breakBefore.back());
}

void ticks() {
    const auto t = niceTicks(-1.65, -.6, 4);
    assert(near(t.step, .2));
    assert(t.values.size() == 6 && near(t.values.front(), -1.6) && near(t.values.back(), -.6));
    assert(near(niceStep(30, 5), 5) && near(niceStep(68, 4), 20));
    double low = 2, high = 2;
    fitRange(low, high); // a flat signal gets a span around it
    assert(low < 2 && high > 2);
}

void yamlRoundTrip() {
    PlotSpec spec;
    spec.title = "Depth";
    spec.span = 60;
    spec.time = TimeBase::Receipt;
    spec.ownTime = true;
    Lane z;
    z.label = "Z";
    z.unit = "m";
    Series actual;
    actual.label = "Actual";
    actual.source.topic = "odometry/filtered";
    actual.source.field = "pose.pose.position.z";
    z.series.push_back(actual);
    Series commanded;
    commanded.label = "Commanded";
    commanded.source.kind = Source::Kind::Figure;
    commanded.source.figure = "motion.z.commanded";
    commanded.slot = 1;
    commanded.steps = true;
    commanded.color = 0x7a5230;
    z.series.push_back(commanded);
    Lane error;
    error.label = "Error";
    error.limits = std::make_pair(-24.0, 24.0);
    Series difference;
    difference.label = "Z error";
    difference.source = commanded.source;
    difference.minus = actual.source;
    difference.hidden = true;
    error.series.push_back(difference);
    spec.lanes = {z, error};

    for (const auto &back : {fromYaml(toYaml(spec), "test"), fromLine(toLine(spec), "test")}) {
        assert(back.title == "Depth" && back.span == 60 && back.time == TimeBase::Receipt && back.ownTime);
        assert(back.lanes.size() == 2 && back.lanes[0].unit == "m");
        assert(back.lanes[0].series[0].source == actual.source);
        assert(back.lanes[0].series[1].source.figure == "motion.z.commanded");
        assert(back.lanes[0].series[1].steps && back.lanes[0].series[1].color == 0x7a5230u);
        assert(back.lanes[1].limits && back.lanes[1].limits->second == 24);
        assert(back.lanes[1].series[0].minus && *back.lanes[1].series[0].minus == actual.source);
        assert(back.lanes[1].series[0].hidden);
    }
    assert(toLine(spec).find('\n') == std::string::npos);

    // Strict: unknown keys, a series without a source, bad colors and spans are refused.
    assert(throws([] { fromYaml(YAML::Load("{title: x, colour: red}"), "t"); }));
    assert(throws([] { fromYaml(YAML::Load("{lanes: [{series: [{label: a}]}]}"), "t"); }));
    assert(throws([] { fromYaml(YAML::Load("{lanes: [{series: [{source: {topic: a}}]}]}"), "t"); }));
    assert(throws([] { fromYaml(YAML::Load("{lanes: [{series: [{source: {figure: f}, color: '#12345'}]}]}"), "t"); }));
    assert(throws([] { fromYaml(YAML::Load("{span_s: -1}"), "t"); }));
    assert(throws([] { fromYaml(YAML::Load("{lanes: [{limits: [2, 1]}]}"), "t"); }));
    // A series without a slot or label takes the lane's next free slot and its field's label.
    const auto filled = fromYaml(YAML::Load("{lanes: [{series: [{source: {topic: t, field: a.b.position.z}}, {source: "
                                            "{topic: t, field: 'data[3]'}}]}]}"),
                                 "t");
    assert(filled.lanes[0].series[0].label == "position.z" && filled.lanes[0].series[1].label == "data[3]");
    assert(filled.lanes[0].series[0].slot == 0 && filled.lanes[0].series[1].slot == 1);
}

void slots() {
    Lane lane;
    for (int i = 0; i < 3; ++i) {
        Series s;
        s.slot = freeSlot(lane);
        lane.series.push_back(s);
    }
    assert(lane.series[2].slot == 2 && freeSlot(lane) == 3);
    setSlot(lane, 0, 2); // taking a slot in use swaps the two
    assert(lane.series[0].slot == 2 && lane.series[2].slot == 0);
    lane.series[1].color = 0x123456;
    setSlot(lane, 1, 5); // a theme slot clears the custom color
    assert(lane.series[1].slot == 5 && !lane.series[1].color);
    for (int i = 0; i < 6; ++i) {
        Series s;
        s.slot = freeSlot(lane);
        lane.series.push_back(s);
    }
    assert(lane.series.back().slot >= kSlots); // past eight: the dashed repeats
}

// Moving a series between lanes and plots: colors stay distinct, an emptied lane goes, its own lane is no move.
void moves() {
    const auto series = [](const char *label, int slot) {
        Series s;
        s.label = label;
        s.source.topic = "t";
        s.source.field = label;
        s.slot = slot;
        return s;
    };
    PlotSpec plot;
    Lane z, rate;
    z.label = "Z";
    z.unit = "m";
    z.series = {series("actual", 0), series("commanded", 1)};
    rate.label = "Heave rate";
    rate.unit = "m/s";
    rate.series = {series("vz", 0)};
    plot.lanes = {z, rate};

    assert(!moveSeries(plot, 0, 0, plot, 0)); // its own lane
    assert(!moveSeries(plot, 0, 5, plot, 1)); // no such series
    // The only series of lane 1 onto lane 0: it takes a free color there, and the emptied lane goes.
    assert(moveSeries(plot, 1, 0, plot, 0));
    assert(plot.lanes.size() == 1 && plot.lanes[0].series.size() == 3);
    assert(plot.lanes[0].series[2].label == "vz" && plot.lanes[0].series[2].slot == 2);

    // Out to a new lane (named for it, keeping the old lane's unit), then into another plot.
    assert(moveSeries(plot, 0, 1, plot, -1));
    assert(plot.lanes.size() == 2 && plot.lanes[1].label == "commanded" && plot.lanes[1].unit == "m");
    assert(plot.lanes[1].series[0].slot == 0);
    PlotSpec other;
    Lane thrust;
    thrust.label = "Thrust";
    thrust.series = {series("VUS", 0)};
    other.lanes = {thrust};
    plot.lanes[0].series[0].color = 0x7a5230; // a custom color goes with it
    assert(moveSeries(plot, 0, 0, other, 0));
    assert(other.lanes[0].series.size() == 2 && other.lanes[0].series[1].color == 0x7a5230u);
    assert(other.lanes[0].unit == "m"); // an empty unit takes the series' old one
    assert(plot.lanes[0].series.size() == 1 && plot.lanes[0].series[0].label == "vz");

    // A lane before the target emptied in the same plot: the series still lands in the lane it was dropped on.
    PlotSpec three;
    Lane a, b, c;
    a.label = "A";
    a.series = {series("a", 0)};
    b.label = "B";
    b.series = {series("b", 0)};
    c.label = "C";
    c.series = {series("c", 0)};
    three.lanes = {a, b, c};
    assert(moveSeries(three, 0, 0, three, 2));
    assert(three.lanes.size() == 2 && three.lanes[1].label == "C" && three.lanes[1].series.size() == 2);
}

void labels() {
    assert(fieldLabel("pose.pose.position.z") == "position.z");
    assert(fieldLabel("pose.pose.orientation.yaw") == "yaw");
    assert(fieldLabel("data[3]") == "data[3]");
    assert(fieldLabel("esc_telemetry[0].rpm") == "rpm");
    assert(fieldUnit("pose.pose.orientation.roll") == "°" && fieldUnit("data[0]").empty());
}

// Headless ImGui: the plot windows come back from the ini, draw without ROS, and write the same ini.
void windowsHeadless() {
    ImGui::CreateContext();
    auto &io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.DisplaySize = {1400, 900};
    io.DeltaTime = 1.f / 30;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    unsigned char *pixels;
    int w, h;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &w, &h);
    loadThemes(NEREUS_VIEWER_THEMES);
    assert(applyTheme("heat-sheet"));
    assert(palette().series[0].w > 0); // the theme's own plot colors
    install();

    PlotSpec spec;
    spec.title = "Depth";
    Lane lane;
    lane.label = "Z";
    Series s;
    s.label = "Actual";
    s.source.topic = "odometry/filtered";
    s.source.field = "pose.pose.position.z";
    lane.series.push_back(s);
    spec.lanes.push_back(lane);
    const std::string ini = "[NereusPlots][Plots]\ntimeline\t60\nplot\t7\t1\t" + toLine(spec) + "\nplot\t9\t0\t" +
                            toLine(PlotSpec{}) + "\n\n";
    ImGui::LoadIniSettingsFromMemory(ini.c_str(), ini.size());

    auto list = windows();
    assert(list.size() == 3); // two plots and the Topics browser
    assert(list[0].key == "plot.7" && list[0].label == "Depth" && *list[0].open);
    assert(list[1].key == "plot.9" && !*list[1].open);
    assert(list[2].key == "topics" && !list[2].plot);

    // A few frames, wide then narrow, with no ROS behind it (empty lanes, "waiting").
    for (const float width : {1400.f, 420.f}) {
        io.DisplaySize = {width, 900};
        for (int i = 0; i < 3; ++i) {
            ImGui::NewFrame();
            frame();
            drawWindows();
            ImGui::Render();
        }
    }
    figureMenuItems("telemetry.none"); // unknown groups draw nothing

    std::size_t size = 0;
    const std::string saved = ImGui::SaveIniSettingsToMemory(&size);
    assert(saved.find("[NereusPlots][Plots]") != std::string::npos);
    assert(saved.find("timeline\t60") != std::string::npos);
    assert(saved.find("plot\t7\t1\t") != std::string::npos && saved.find("plot\t9\t0\t") != std::string::npos);
    assert(saved.find("pose.pose.position.z") != std::string::npos);

    // An ini without the section (a layout from before plots) keeps the plots there are.
    const std::string old = "[Window][Debug##Default]\nPos=60,60\n\n";
    ImGui::LoadIniSettingsFromMemory(old.c_str(), old.size());
    assert(windows().size() == 3);
    ImGui::DestroyContext();
}

} // namespace

int main() {
    ringAndLookup();
    decimation();
    ticks();
    yamlRoundTrip();
    slots();
    moves();
    labels();
    windowsHeadless();
    std::cout << "plots ok\n";
    return 0;
}

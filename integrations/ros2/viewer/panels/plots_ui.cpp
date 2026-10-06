// Plot windows, the Topics browser, saved plots and the panels' hooks, drawn with ImGui over the Hub's channels.
// A plot window stacks lanes on one time axis; each lane has its own value axis and a results table (series, value
// now or under the cursor, min, max). Every window shares one timeline unless it takes its own time span.
#include "nereus/ros_viewer/panel_layout.hpp"
#include "nereus/ros_viewer/pins.hpp"
#include "nereus/ros_viewer/plots/plots.hpp"
#include "nereus/ros_viewer/theme.hpp"
#include "plots_hub.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <glm/gtc/quaternion.hpp>
#include <imgui.h>
#include <imgui_internal.h>
#include <iterator>
#include <list>
#include <set>
#include <sstream>

namespace nereus::ros_viewer::plots {
namespace {
namespace fs = std::filesystem;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr const char *kPayload = "NEREUS_PLOT_SOURCE";       // a field dragged from Topics
constexpr const char *kSeriesPayload = "NEREUS_PLOT_SERIES"; // a series dragged out of a lane
constexpr double kTrendSeconds = 30;
constexpr double kFigurePeriod = .05; // figures sampled at 20 Hz
const double kSpans[] = {10, 30, 60, 300, 600};
const char *const kSpanLabels[] = {"Last 10 s", "Last 30 s", "Last 1 min", "Last 5 min", "Last 10 min"};

// The validated series colors (adjacent pairs colorblind-safe, 3:1 or better) for light and dark windows.
constexpr uint32_t kLight[kSlots] = {0x2a78d6, 0xe0591f, 0x12936a, 0xb07800, 0xcc5a8a, 0x008300, 0x4a3aa7, 0xd93f3e};
constexpr uint32_t kDark[kSlots] = {0x3987e5, 0xd95926, 0x199e70, 0xc98500, 0xd55181, 0x008300, 0x9085e9, 0xe66767};

const char *kAxisIds[6] = {"x", "y", "z", "roll", "pitch", "yaw"};
const char *kAxisNames[6] = {"X", "Y", "Z", "Roll", "Pitch", "Yaw"};

double steadyNow() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

ImVec4 fromRgb(uint32_t c) {
    return {float((c >> 16) & 255) / 255.f, float((c >> 8) & 255) / 255.f, float(c & 255) / 255.f, 1};
}
uint32_t toRgb(const float c[3]) {
    const auto byte = [](float v) { return uint32_t(std::clamp(v, 0.f, 1.f) * 255.f + .5f); };
    return byte(c[0]) << 16 | byte(c[1]) << 8 | byte(c[2]);
}
ImU32 u32(ImVec4 c) {
    return ImGui::ColorConvertFloat4ToU32(c);
}
ImVec4 alpha(ImVec4 c, float a) {
    return {c.x, c.y, c.z, c.w * a};
}
float luminance(ImVec4 c) {
    return .2126f * c.x + .7152f * c.y + .0722f * c.z;
}
bool darkWindow() {
    return luminance(ImGui::GetStyle().Colors[ImGuiCol_WindowBg]) < .4f;
}
ImVec4 slotColor(int slot) {
    const int i = ((slot % kSlots) + kSlots) % kSlots;
    const auto given = palette().series[std::size_t(i)];
    return given.w > 0 ? given : fromRgb((darkWindow() ? kDark : kLight)[i]);
}
ImVec4 seriesColor(const Series &s) {
    return s.color ? fromRgb(*s.color) : slotColor(s.slot);
}
bool dashed(const Series &s) {
    return !s.color && s.slot >= kSlots;
}

ImFont *fontOr(ImFont *f) {
    return f ? f : ImGui::GetFont();
}
float textWidth(ImFont *font, const std::string &text) {
    return font->CalcTextSizeA(font->FontSize, 1e9f, 0, text.c_str()).x;
}
void text(ImDrawList *draw, ImFont *font, ImVec2 at, ImU32 color, const std::string &value) {
    draw->AddText(font, font->FontSize, at, color, value.c_str());
}
// Text clipped to `width`, an ellipsis replacing what does not fit.
std::string fit(ImFont *font, const std::string &value, float width) {
    if (textWidth(font, value) <= width)
        return value;
    std::string cut = value;
    while (!cut.empty() && textWidth(font, cut + "…") > width)
        cut.pop_back();
    return cut + "…";
}

std::string format(const char *pattern, double value) {
    char out[64];
    std::snprintf(out, sizeof(out), pattern, value);
    return out;
}
std::string number(double value, int decimals) {
    if (!std::isfinite(value))
        return "--";
    char out[64];
    std::snprintf(out, sizeof(out), "%.*f", decimals, value);
    std::string s = out;
    if (s.find_first_not_of("-0.") == std::string::npos && s[0] == '-')
        s.erase(0, 1); // no "-0.000"
    return s;
}
// Decimals that resolve about a thousandth of a lane's range.
int decimalsFor(double range) {
    if (!(range > 0))
        return 2;
    return std::clamp(int(std::ceil(-std::log10(range))) + 2, 0, 4);
}
// Decimals a tick step needs (0.25 -> 2, 5 -> 0).
int stepDecimals(double step) {
    int d = 0;
    while (d < 6 && std::abs(step * std::pow(10, d) - std::round(step * std::pow(10, d))) > 1e-6)
        ++d;
    return d;
}

// --- State ---

struct TimeView {
    bool paused = false;
    double end = 0, span = 30;
};

struct Removed {
    std::size_t lane = 0, index = 0;
    bool wholeLane = false;
    Series series;
    Lane laneSpec;
};

struct Live {
    std::shared_ptr<Channel> a, b; // b: the subtracted source of a derived series
};

struct PlotWindow {
    int id = 0;
    PlotSpec spec;
    bool open = true, remove = false, focus = false, place = false;
    TimeView own;
    double hover = kNaN, hoverNext = kNaN; // own-time windows' cursor
    std::optional<Removed> undo;
    int addLane = -1;     // the lane the search adds to (-1: a new lane)
    bool openAdd = false; // open the search next frame (asked from inside a lane's table)
    std::vector<std::vector<Live>> live;
    std::string liveSignature;
    std::string message;
    double messageUntil = 0;
    std::string key() const {
        return "plot." + std::to_string(id);
    }
    std::string name() const {
        return spec.title + "###plot." + std::to_string(id);
    }
};

struct FigureInfo {
    std::string label, unit, from;
};

struct IndexEntry {
    Source source;
    std::string text;
};

struct State {
    std::unique_ptr<Hub> hub;
    Options options;
    std::shared_ptr<panels::Motion> motion;
    std::shared_ptr<panels::Telemetry> telemetry;
    std::list<PlotWindow> plots;
    int nextId = 1;
    TimeView shared;
    double hover = kNaN, hoverNext = kNaN; // the shared timeline's cursor (last frame / this frame)
    bool topicsOpen = false, manageOpen = false;
    std::map<std::string, FigureInfo> figures;
    std::vector<std::pair<std::string, std::string>> readings; // telemetry id, label
    double lastSample = -1e9;
    std::vector<std::string> saved;
    double savedAt = -1e9;
    char filter[128] = {};
    char search[128] = {};
    std::map<std::string, std::unique_ptr<Message>> decoded; // the Topics browser's latest message per open topic
    std::map<std::string, double> decodedAt;
    std::map<std::string, std::unique_ptr<FieldReader>> readers; // "topic#path"
    std::vector<IndexEntry> index;
    double indexAt = -1e9;
    std::map<std::string, std::vector<std::string>> typePaths; // numeric paths per type, without a message
    std::uint64_t jumpsSeen = 0;
    double jumpUntil = 0, jumpSeconds = 0;
    int focused = 0; // the plot window focused last
    std::vector<std::function<void()>> deferred;
    // ini
    bool iniSeen = false;
    struct IniPlot {
        int id = 0;
        bool open = true;
        std::string line;
    };
    std::vector<IniPlot> iniPlots;
    double iniSpan = 0;
};
State &state() {
    static State s;
    return s;
}

PlotWindow *plotById(int id) {
    for (auto &w : state().plots)
        if (w.id == id && !w.remove)
            return &w;
    return nullptr;
}

double now() {
    return state().hub ? state().hub->now() : 0;
}

TimeView &viewOf(PlotWindow &w) {
    return w.spec.ownTime ? w.own : state().shared;
}

// --- Sources, labels and groups ---

std::string relativeTopic(const std::string &topic) {
    auto &s = state();
    return s.hub ? s.hub->relative(s.hub->resolve(topic)) : topic;
}

std::string sourceText(const Source &source) {
    if (source.kind == Source::Kind::Figure) {
        const auto found = state().figures.find(source.figure);
        return found == state().figures.end() ? source.figure : found->second.from;
    }
    return relativeTopic(source.topic) + " › " + source.field;
}

std::string sourceLabel(const Source &source) {
    if (source.kind == Source::Kind::Figure) {
        const auto found = state().figures.find(source.figure);
        return found == state().figures.end() ? source.figure : found->second.label;
    }
    return fieldLabel(source.field);
}

std::string sourceUnit(const Source &source) {
    if (source.kind == Source::Kind::Figure) {
        const auto found = state().figures.find(source.figure);
        return found == state().figures.end() ? std::string() : found->second.unit;
    }
    return fieldUnit(source.field);
}

Series seriesOf(const Source &source, const Lane &lane) {
    Series s;
    s.source = source;
    s.label = sourceLabel(source);
    s.slot = freeSlot(lane);
    return s;
}

Lane laneOf(const Source &source) {
    Lane lane;
    lane.label = sourceLabel(source);
    lane.unit = sourceUnit(source);
    lane.series.push_back(seriesOf(source, lane));
    return lane;
}

Source figureSource(const std::string &id) {
    Source s;
    s.kind = Source::Kind::Figure;
    s.figure = id;
    return s;
}

// A Motion axis as plotted: actual and commanded (steps) over the axis, then its error.
std::vector<Lane> axisLanes(int axis, bool withError) {
    const std::string id = std::string("motion.") + kAxisIds[axis], unit = axis < 3 ? "m" : "°";
    Lane values;
    values.label = kAxisNames[axis];
    values.unit = unit;
    Series actual = seriesOf(figureSource(id + ".actual"), values);
    actual.label = "Actual";
    values.series.push_back(actual);
    Series commanded = seriesOf(figureSource(id + ".commanded"), values);
    commanded.label = "Commanded";
    commanded.steps = true;
    values.series.push_back(commanded);
    std::vector<Lane> lanes{values};
    if (withError) {
        Lane error;
        error.label = "Error";
        error.unit = unit;
        Series e = seriesOf(figureSource(id + ".error"), error);
        e.label = std::string(kAxisNames[axis]) + " error";
        e.slot = 2;
        error.series.push_back(e);
        lanes.push_back(error);
    }
    return lanes;
}

// A "Plot this" group: its title, the menu's hint, and the lanes it adds.
struct Group {
    std::string title, hint;
    std::vector<Lane> lanes;
};

std::optional<Group> figureGroup(const std::string &group) {
    auto &s = state();
    if (group == "motion.all") {
        Group g{"Pose", "all six axes", {}};
        for (int i = 0; i < 6; ++i)
            for (auto &lane : axisLanes(i, false))
                g.lanes.push_back(lane);
        return g;
    }
    for (int i = 0; i < 6; ++i)
        if (group == std::string("motion.") + kAxisIds[i])
            return Group{kAxisNames[i], "actual, commanded, error", axisLanes(i, true)};
    if (group.rfind("telemetry.", 0) == 0) {
        const auto found = s.figures.find(group);
        if (found == s.figures.end())
            return std::nullopt;
        return Group{found->second.label, found->second.from, {laneOf(figureSource(group))}};
    }
    return std::nullopt;
}

// --- Plot windows ---

PlotWindow &addPlot(PlotSpec spec) {
    auto &s = state();
    PlotWindow w;
    w.id = s.nextId++;
    if (!spec.ownTime)
        spec.span = s.shared.span;
    w.own.span = spec.span;
    w.spec = std::move(spec);
    w.focus = w.place = true;
    s.plots.push_back(std::move(w));
    s.focused = s.plots.back().id;
    return s.plots.back();
}

void addLanes(PlotWindow &w, const std::vector<Lane> &lanes) {
    for (const auto &lane : lanes)
        w.spec.lanes.push_back(lane);
    w.undo.reset();
    w.open = w.focus = true;
}

// A source into a lane (-1: a new lane).
void addSource(PlotWindow &w, const Source &source, int lane) {
    w.undo.reset();
    if (lane < 0 || std::size_t(lane) >= w.spec.lanes.size()) {
        w.spec.lanes.push_back(laneOf(source));
        return;
    }
    auto &l = w.spec.lanes[std::size_t(lane)];
    l.series.push_back(seriesOf(source, l));
    if (l.unit.empty())
        l.unit = sourceUnit(source);
}

void newPlotWith(const Source &source) {
    PlotSpec spec;
    spec.title = sourceLabel(source);
    spec.lanes.push_back(laneOf(source));
    addPlot(std::move(spec));
}

// The focused plot window's last lane gets `source` when Shift is held, else a new plot opens with it.
void plotSource(const Source &source) {
    if (ImGui::GetIO().KeyShift)
        if (auto *w = plotById(state().focused)) {
            addSource(*w, source, w->spec.lanes.empty() ? -1 : int(w->spec.lanes.size()) - 1);
            w->open = w->focus = true;
            return;
        }
    newPlotWith(source);
}

void removeSeries(PlotWindow &w, std::size_t lane, std::size_t index) {
    if (lane >= w.spec.lanes.size() || index >= w.spec.lanes[lane].series.size())
        return;
    Removed r;
    r.lane = lane;
    r.index = index;
    r.series = w.spec.lanes[lane].series[index];
    w.spec.lanes[lane].series.erase(w.spec.lanes[lane].series.begin() + long(index));
    w.undo = r;
}

// A series into another lane of this plot or another (-1: a new lane); see plots::moveSeries.
void moveTo(PlotWindow &from, std::size_t lane, std::size_t index, PlotWindow &to, int toLane) {
    if (!moveSeries(from.spec, lane, index, to.spec, toLane))
        return;
    from.undo.reset();
    to.undo.reset();
    to.open = true;
}

void removeLane(PlotWindow &w, std::size_t lane) {
    if (lane >= w.spec.lanes.size())
        return;
    Removed r;
    r.lane = lane;
    r.wholeLane = true;
    r.laneSpec = w.spec.lanes[lane];
    w.spec.lanes.erase(w.spec.lanes.begin() + long(lane));
    w.undo = r;
}

void undo(PlotWindow &w) {
    if (!w.undo)
        return;
    auto r = *w.undo;
    w.undo.reset();
    if (r.wholeLane) {
        w.spec.lanes.insert(w.spec.lanes.begin() + long(std::min(r.lane, w.spec.lanes.size())), r.laneSpec);
        return;
    }
    if (r.lane >= w.spec.lanes.size())
        w.spec.lanes.push_back(Lane{});
    auto &series = w.spec.lanes[std::min(r.lane, w.spec.lanes.size() - 1)].series;
    series.insert(series.begin() + long(std::min(r.index, series.size())), r.series);
}

// Channels for every series, rebuilt when the sources change. Plots hold them, so a hidden window keeps recording.
void bindLive(PlotWindow &w) {
    auto &s = state();
    if (!s.hub)
        return;
    std::string signature;
    for (const auto &lane : w.spec.lanes) {
        for (const auto &series : lane.series)
            signature += series.source.key() + (series.minus ? "-" + series.minus->key() : "") + "|";
        signature += "#";
    }
    if (signature == w.liveSignature)
        return;
    w.live.clear();
    for (const auto &lane : w.spec.lanes) {
        std::vector<Live> row;
        for (const auto &series : lane.series)
            row.push_back({s.hub->channel(series.source), series.minus ? s.hub->channel(*series.minus) : nullptr});
        w.live.push_back(std::move(row));
    }
    w.liveSignature = signature;
}

// --- Data for drawing ---

// What a lane needs of one series this frame, copied out under the channel locks.
struct View {
    Decimated drawn;
    double cursor = kNaN, latest = kNaN; // value under the cursor (or the newest), as drawn
    std::string status;                  // why it is not live ("stale 4.2 s", "no publisher", an error)
    bool error = false, stale = false;
};

// Seconds between samples above which a line breaks: several typical intervals, at least a second.
double gapOf(const SeriesBuffer &b) {
    if (b.size() < 2)
        return 1;
    const std::size_t k = std::min<std::size_t>(50, b.size() - 1);
    const double dt = (b.back().receipt - b.at(b.size() - 1 - k).receipt) / double(k);
    return std::max(1.0, 5 * dt);
}

// a - b over [t0, t1] (b held at its last value), as a buffer the decimator can read.
SeriesBuffer difference(const SeriesBuffer &a, const SeriesBuffer &b, double t0, double t1, TimeBase base) {
    SeriesBuffer out(a.size() + 2, 1e12);
    std::size_t i = a.lowerBound(t0, base);
    if (i > 0)
        --i;
    for (; i < a.size(); ++i) {
        const auto &sa = a.at(i);
        if (timeOf(sa, base) > t1)
            break;
        const auto *sb = b.valueAt(timeOf(sa, base), base);
        if (sb)
            out.push({sa.stamp, sa.receipt, sa.value - sb->value});
    }
    return out;
}

View seriesView(const Series &series, const Live &live, double t0, double t1, int columns, TimeBase base,
                double cursor) {
    View v;
    auto &s = state();
    if (!live.a)
        return v;
    const auto status = [&](const Channel &c, const Source &source) {
        if (!c.error.empty()) {
            v.error = true;
            v.status = "Can't read this topic: " + c.error;
            return;
        }
        const double receiptNow = now();
        if (c.samples.empty()) {
            if (source.kind == Source::Kind::Topic && s.hub && s.hub->publishers(s.hub->resolve(source.topic)) == 0)
                v.status = "no publisher";
            else
                v.status = "waiting";
            v.stale = true;
            return;
        }
        const double age = receiptNow - c.samples.back().receipt;
        if (age > std::max(2.0, gapOf(c.samples))) {
            v.stale = true;
            v.status =
                source.kind == Source::Kind::Topic && s.hub && s.hub->publishers(s.hub->resolve(source.topic)) == 0
                    ? "no publisher"
                    : "stale " + format("%.1f s", age);
        } else if (c.stamped && c.hasOffset && std::abs(c.offset) > .5)
            v.status = "clock offset " + format("%.1f s", c.offset);
    };
    if (live.b) {
        if (live.a == live.b) { // a - a: zero wherever a is
            std::lock_guard<std::mutex> lock(live.a->mutex);
            status(*live.a, series.source);
            SeriesBuffer zero(live.a->samples.size() + 1, 1e12);
            for (std::size_t i = 0; i < live.a->samples.size(); ++i) {
                auto sample = live.a->samples.at(i);
                sample.value = 0;
                zero.push(sample);
            }
            v.drawn = decimate(zero, t0, t1, columns, base, gapOf(live.a->samples));
            v.latest = zero.empty() ? kNaN : 0;
            v.cursor = std::isfinite(cursor) && zero.valueAt(cursor, base) ? 0 : kNaN;
            return v;
        }
        std::scoped_lock lock(live.a->mutex, live.b->mutex);
        status(*live.a, series.source);
        if (!v.error && !live.b->error.empty()) {
            v.error = true;
            v.status = "Can't read this topic: " + live.b->error;
        }
        const double lead = (t1 - t0) * .02;
        const auto diff = difference(live.a->samples, live.b->samples, t0 - lead, t1, base);
        v.drawn = decimate(diff, t0, t1, columns, base, std::max(gapOf(live.a->samples), gapOf(live.b->samples)));
        if (!diff.empty())
            v.latest = diff.back().value;
        if (std::isfinite(cursor)) {
            const auto *a = live.a->samples.valueAt(cursor, base), *b = live.b->samples.valueAt(cursor, base);
            if (a && b)
                v.cursor = a->value - b->value;
        }
        if (!live.a->samples.empty() && !live.b->samples.empty())
            v.latest = live.a->samples.back().value - live.b->samples.back().value;
        return v;
    }
    std::lock_guard<std::mutex> lock(live.a->mutex);
    status(*live.a, series.source);
    const auto &samples = live.a->samples;
    v.drawn = decimate(samples, t0, t1, columns, base, gapOf(samples));
    if (!samples.empty())
        v.latest = samples.back().value;
    if (std::isfinite(cursor))
        if (const auto *at = samples.valueAt(cursor, base))
            v.cursor = at->value;
    return v;
}

// --- Drawing a lane ---

float timeX(double t, double t0, double t1, const ImRect &r) {
    return float(r.Min.x + (t - t0) / (t1 - t0) * r.GetWidth());
}

// A polyline, dashed when asked (6 on, 4 off, along its length).
void polyline(ImDrawList *draw, const std::vector<ImVec2> &points, ImU32 color, float thickness, bool dash) {
    if (points.size() < 2)
        return;
    if (!dash) {
        draw->AddPolyline(points.data(), int(points.size()), color, ImDrawFlags_None, thickness);
        return;
    }
    const float on = ui(6), off = ui(4);
    float phase = 0;
    for (std::size_t i = 1; i < points.size(); ++i) {
        ImVec2 a = points[i - 1];
        const ImVec2 b = points[i];
        float length = std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
        const ImVec2 dir = length > 0 ? ImVec2((b.x - a.x) / length, (b.y - a.y) / length) : ImVec2(0, 0);
        while (length > 0) {
            const bool drawing = phase < on;
            const float run = std::min(length, drawing ? on - phase : on + off - phase);
            const ImVec2 c(a.x + dir.x * run, a.y + dir.y * run);
            if (drawing)
                draw->AddLine(a, c, color, thickness);
            phase = std::fmod(phase + run, on + off);
            length -= run;
            a = c;
        }
    }
}

// A line of constant width. Points join into polylines, which are split wherever the line turns back by more than
// about 100 degrees: there a polyline's miter join spikes out to several times its width, so a noisy line would
// look thicker in places. Dashed lines keep their dash phase across the splits.
class Stroke {
  public:
    Stroke(ImDrawList *draw, ImU32 color, float thickness, bool dash)
        : draw_(draw), color_(color), thickness_(thickness), dash_(dash) {}
    ~Stroke() {
        lift();
    }
    Stroke(const Stroke &) = delete;
    Stroke &operator=(const Stroke &) = delete;

    // Continues the line to `p` (starts one after lift()).
    void to(ImVec2 p) {
        if (!run_.empty()) {
            const ImVec2 last = run_.back();
            if (std::abs(p.x - last.x) < .05f && std::abs(p.y - last.y) < .05f)
                return;
            if (run_.size() >= 2 && sharp(run_[run_.size() - 2], last, p)) {
                flush();
                run_.assign(1, last);
            }
        }
        run_.push_back(p);
    }
    // Ends the line here; the next to() starts a new one.
    void lift() {
        flush();
        run_.clear();
    }
    // A separate straight piece (a dense column's range).
    void segment(ImVec2 a, ImVec2 b) {
        lift();
        to(a);
        to(b);
        lift();
    }

  private:
    static bool sharp(ImVec2 a, ImVec2 b, ImVec2 c) {
        const float ux = b.x - a.x, uy = b.y - a.y, vx = c.x - b.x, vy = c.y - b.y;
        const float lengths = std::sqrt((ux * ux + uy * uy) * (vx * vx + vy * vy));
        return lengths > 0 && (ux * vx + uy * vy) / lengths < -.17f; // turning by more than ~100 degrees
    }
    void flush() {
        if (run_.size() < 2)
            return;
        if (!dash_) {
            draw_->AddPolyline(run_.data(), int(run_.size()), color_, ImDrawFlags_None, thickness_);
            return;
        }
        const float on = ui(6), off = ui(4);
        for (std::size_t i = 1; i < run_.size(); ++i) {
            ImVec2 a = run_[i - 1];
            const ImVec2 b = run_[i];
            float length = std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
            const ImVec2 dir = length > 0 ? ImVec2((b.x - a.x) / length, (b.y - a.y) / length) : ImVec2(0, 0);
            while (length > 0) {
                const bool drawing = phase_ < on;
                const float run = std::min(length, drawing ? on - phase_ : on + off - phase_);
                const ImVec2 c(a.x + dir.x * run, a.y + dir.y * run);
                if (drawing)
                    draw_->AddLine(a, c, color_, thickness_);
                phase_ = std::fmod(phase_ + run, on + off);
                length -= run;
                a = c;
            }
        }
    }

    ImDrawList *draw_;
    ImU32 color_;
    float thickness_;
    bool dash_;
    float phase_ = 0;
    std::vector<ImVec2> run_;
};

// The series from its decimated columns: a column of one sample is a point on the line; a column of several draws
// their range as one vertical stroke, the line arriving at its first sample and leaving from its last.
void drawSeries(ImDrawList *draw, const View &v, const Series &series, double t0, double t1, const ImRect &r,
                double low, double high) {
    const auto y = [&](double value) { return float(r.Max.y - (value - low) / (high - low) * r.GetHeight()); };
    Stroke stroke(draw, u32(seriesColor(series)), std::max(1.f, ui(2)), dashed(series));
    bool have = false;
    ImVec2 previous;
    const auto arrive = [&](ImVec2 p) {
        if (series.steps && have)
            stroke.to({p.x, previous.y});
        stroke.to(p);
    };
    if (v.drawn.hasLead && std::isfinite(v.drawn.lead.value) && !v.drawn.columns.empty() && !v.drawn.breakBefore[0]) {
        previous = {r.Min.x - 1, y(v.drawn.lead.value)};
        stroke.to(previous);
        have = true;
    }
    for (std::size_t i = 0; i < v.drawn.columns.size(); ++i) {
        const auto &c = v.drawn.columns[i];
        if (v.drawn.breakBefore[i]) {
            stroke.lift();
            have = false;
        }
        const float x = .5f * (timeX(c.t0, t0, t1, r) + timeX(c.t1, t0, t1, r));
        arrive({x, y(c.first)});
        if (c.low != c.high) {
            stroke.segment({x, y(c.high)}, {x, y(c.low)});
            stroke.to({x, y(c.last)});
        }
        previous = {x, y(c.last)};
        have = true;
    }
    // A held setpoint runs on to the right edge.
    if (series.steps && have)
        stroke.to({std::min(r.Max.x, timeX(t1, t0, t1, r)), previous.y});
}

// The lane's value range: the visible samples of shown series, padded, widened to its limits.
std::pair<double, double> laneRange(const Lane &lane, const std::vector<View> &views) {
    double low = std::numeric_limits<double>::infinity(), high = -low;
    for (std::size_t i = 0; i < views.size() && i < lane.series.size(); ++i) {
        if (lane.series[i].hidden)
            continue;
        const auto &d = views[i].drawn;
        if (d.count) {
            low = std::min(low, double(d.low));
            high = std::max(high, double(d.high));
        }
        if (d.hasLead && std::isfinite(d.lead.value)) {
            low = std::min(low, double(d.lead.value));
            high = std::max(high, double(d.lead.value));
        }
    }
    if (lane.limits) {
        low = std::isfinite(low) ? std::min(low, lane.limits->first) : lane.limits->first;
        high = std::isfinite(high) ? std::max(high, lane.limits->second) : lane.limits->second;
    }
    fitRange(low, high);
    return {low, high};
}

// Time label: relative to the live edge ("-25", "now"), else the clock (sim seconds, or the time of day).
std::string timeLabel(double t, double end, bool live, double step, bool first) {
    const int d = stepDecimals(step);
    if (live) {
        const double rel = t - end;
        if (std::abs(rel) < step * 1e-3)
            return "now";
        return number(rel, d) + (first ? " s" : "");
    }
    if (state().options.useSimTime)
        return number(t, d) + (first ? " s" : "");
    const std::time_t whole = std::time_t(std::floor(t));
    std::tm local{};
    localtime_r(&whole, &local);
    char out[32];
    std::strftime(out, sizeof(out), "%H:%M:%S", &local);
    return out;
}

std::string cursorLabel(double t, double end, bool live) {
    if (live)
        return number(t - end, 1) + " s";
    if (state().options.useSimTime)
        return number(t, 1) + " s";
    return timeLabel(t, end, false, 1, false);
}

// --- Results table ---

// Row heights: a row is its label over its path, or (compact, when the lanes are short) the label alone.
struct TableGeometry {
    float header, row;
};
TableGeometry tableGeometry(bool compact) {
    const float line = ImGui::GetTextLineHeight();
    ImFont *small = fontOr(typeRamp().small);
    return {line + ui(9), compact ? line + ui(9) : line + small->FontSize + ui(11)};
}

float tableHeight(const PlotWindow &w, std::size_t lane, bool compact) {
    const auto g = tableGeometry(compact);
    return g.header + float(w.spec.lanes[lane].series.size()) * g.row;
}

void seriesMenu(PlotWindow &w, std::size_t laneIndex, std::size_t index);
void dropTarget(PlotWindow &w, const ImRect &rect, ImGuiID id, int lane);
void laneMenu(PlotWindow &w, std::size_t laneIndex);

// One lane's table at `at`, `width` wide: the head (lane, unit, cursor time or Now, Min, Max) and a row per series.
void drawTable(PlotWindow &w, std::size_t laneIndex, const std::vector<View> &views, ImVec2 at, float width,
               bool cursorShown, const std::string &cursorText, double low, double high, bool compact) {
    auto &lane = w.spec.lanes[laneIndex];
    auto *draw = ImGui::GetWindowDrawList();
    const auto &p = palette();
    const auto g = tableGeometry(compact);
    ImFont *body = ImGui::GetFont(), *strong = fontOr(typeRamp().strong), *small = fontOr(typeRamp().small);
    const bool full = width >= ui(330);
    const float valueW = ui(66), extremeW = full ? ui(58) : 0, removeW = ui(22);
    const float right = at.x + width;
    const float maxX = right - removeW, minX = maxX - extremeW, valueX = minX - extremeW;
    const int decimals = decimalsFor(high - low);

    // Head: right-click for the lane's menu.
    ImGui::SetCursorScreenPos(at);
    ImGui::SetNextItemAllowOverlap();
    ImGui::InvisibleButton("##lanehead", {width, g.header});
    if (ImGui::BeginPopupContextItem("lane menu")) {
        laneMenu(w, laneIndex);
        ImGui::EndPopup();
    }
    // The head's + (where the rows show their remove box): a series into this lane.
    {
        const ImVec2 size(ui(18), ui(18));
        const ImVec2 boxMin(right - removeW + ui(2), at.y + (g.header - size.y) * .5f - ui(1));
        ImGui::SetCursorScreenPos(boxMin);
        if (ImGui::InvisibleButton("##add", size)) {
            w.addLane = int(laneIndex);
            w.openAdd = true;
        }
        const bool hot = ImGui::IsItemHovered();
        if (hot) {
            draw->AddRectFilled(boxMin, {boxMin.x + size.x, boxMin.y + size.y},
                                ImGui::GetColorU32(ImGuiCol_ButtonHovered), ui(2));
            ImGui::SetTooltip("Add a series to this lane");
        }
        const float c = ui(5);
        const ImVec2 m(std::round(boxMin.x + size.x * .5f) + .5f, std::round(boxMin.y + size.y * .5f) + .5f);
        const ImU32 ink = u32(hot ? p.text : p.muted);
        draw->AddLine({m.x - c, m.y}, {m.x + c + 1, m.y}, ink, std::max(1.f, ui(1.5f)));
        draw->AddLine({m.x, m.y - c}, {m.x, m.y + c + 1}, ink, std::max(1.f, ui(1.5f)));
    }
    const float textY = at.y + (g.header - ImGui::GetTextLineHeight()) * .5f - ui(1);
    const std::string title = lane.label.empty() ? "Lane " + std::to_string(laneIndex + 1) : lane.label;
    text(draw, strong, {at.x, textY}, u32(p.text), title);
    if (!lane.unit.empty())
        text(draw, body, {at.x + textWidth(strong, title) + ui(5), textY}, u32(p.muted), lane.unit);
    const std::string valueHead = cursorShown ? cursorText : "Now";
    text(draw, strong, {valueX - textWidth(strong, valueHead), textY}, u32(p.text), valueHead);
    if (full) {
        text(draw, strong, {minX - textWidth(strong, "Min"), textY}, u32(p.text), "Min");
        text(draw, strong, {maxX - textWidth(strong, "Max"), textY}, u32(p.text), "Max");
    }
    const float ruleY = at.y + g.header - ui(1);
    if (ruledTheme())
        draw->AddRectFilled({at.x, ruleY - ui(1)}, {right, ruleY + ui(1)}, u32(p.text));
    else
        draw->AddLine({at.x, ruleY}, {right, ruleY}, ImGui::GetColorU32(ImGuiCol_Border));

    float y = at.y + g.header;
    std::optional<std::size_t> remove;
    for (std::size_t j = 0; j < lane.series.size(); ++j) {
        auto &series = lane.series[j];
        static const View empty{};
        const View &v = j < views.size() ? views[j] : empty;
        ImGui::PushID(int(j));
        const ImVec2 rowMin(at.x, y), rowMax(right, y + g.row);
        ImGui::SetCursorScreenPos(rowMin);
        ImGui::SetNextItemAllowOverlap();
        ImGui::InvisibleButton("##row", {width, g.row});
        if (ImGui::BeginPopupContextItem("series menu")) {
            seriesMenu(w, laneIndex, j);
            ImGui::EndPopup();
        }
        // Drag the row onto another lane, the new-lane strip, or another plot to move the series there.
        if (ImGui::BeginDragDropSource()) {
            const std::string data = std::to_string(w.id) + "\t" + std::to_string(laneIndex) + "\t" +
                                     std::to_string(j) + "\t" + series.source.key();
            ImGui::SetDragDropPayload(kSeriesPayload, data.data(), data.size());
            const ImVec2 at = ImGui::GetCursorScreenPos();
            const float keyY = at.y + ImGui::GetTextLineHeight() * .5f;
            polyline(ImGui::GetWindowDrawList(), {{at.x, keyY}, {at.x + ui(16), keyY}}, u32(seriesColor(series)),
                     std::max(1.f, ui(2)), dashed(series));
            ImGui::SetCursorScreenPos({at.x + ui(24), at.y});
            ImGui::TextUnformatted(series.label.c_str());
            ImGui::TextDisabled("Drop on a lane to move it there");
            ImGui::EndDragDropSource();
        }
        const bool inRow = ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem) &&
                           ImGui::IsMouseHoveringRect(rowMin, rowMax);
        if (inRow)
            draw->AddRectFilled(rowMin, rowMax, ImGui::GetColorU32(ImGuiCol_FrameBgHovered));
        if (inRow && ImGui::IsWindowFocused() && ImGui::IsKeyPressed(ImGuiKey_Delete, false))
            remove = j;

        // The line key: click to hide or show.
        const float lineY = y + ui(6) + ImGui::GetTextLineHeight() * .5f;
        ImGui::SetCursorScreenPos({at.x, y + ui(4)});
        if (ImGui::InvisibleButton("##key", {ui(20), ImGui::GetTextLineHeight()}))
            series.hidden = !series.hidden;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(series.hidden ? "Show" : "Hide");
        const ImVec4 color = series.hidden ? alpha(seriesColor(series), .35f) : seriesColor(series);
        polyline(draw, {{at.x, lineY}, {at.x + ui(16), lineY}}, u32(color), std::max(1.f, ui(2)), dashed(series));

        // Label, values, and the path (or why it is not live) beneath.
        const ImU32 ink = u32(series.hidden ? p.muted : p.text);
        const float ty = y + ui(5);
        text(draw, body, {at.x + ui(24), ty}, ink, fit(body, series.label, valueX - valueW - at.x - ui(24)));
        const double shown = cursorShown ? v.cursor : v.latest;
        const ImU32 valueInk = u32(v.stale || series.hidden ? p.muted : p.text);
        const std::string value = number(shown, decimals);
        text(draw, body, {valueX - textWidth(body, value), ty}, valueInk, value);
        if (full) {
            const std::string lo = v.drawn.count ? number(v.drawn.low, decimals) : "--";
            const std::string hi = v.drawn.count ? number(v.drawn.high, decimals) : "--";
            text(draw, body, {minX - textWidth(body, lo), ty}, valueInk, lo);
            text(draw, body, {maxX - textWidth(body, hi), ty}, valueInk, hi);
        }
        std::string path =
            series.minus ? sourceText(series.source) + " - " + sourceText(*series.minus) : sourceText(series.source);
        const float pathY = ty + ImGui::GetTextLineHeight() + ui(1);
        if (compact) { // the path in the label's tooltip; a status after the label
            if (!v.status.empty()) {
                const float x = at.x + ui(24) + textWidth(body, series.label) + ui(8);
                text(draw, small, {x, ty + (ImGui::GetTextLineHeight() - small->FontSize) * .5f},
                     u32(v.error ? p.error : p.muted), fit(small, v.status, std::max(0.f, valueX - valueW - x)));
            }
            if (ImGui::IsMouseHoveringRect(rowMin, rowMax) && !ImGui::IsPopupOpen("", ImGuiPopupFlags_AnyPopupId))
                ImGui::SetTooltip("%s%s%s", path.c_str(), v.status.empty() ? "" : "\n", v.status.c_str());
        } else if (!v.status.empty()) {
            const std::string note = v.error ? v.status : path + "  ·  " + v.status;
            text(draw, small, {at.x + ui(24), pathY}, u32(v.error ? p.error : p.muted),
                 fit(small, note, right - at.x - ui(24)));
            if (v.error && ImGui::IsMouseHoveringRect(rowMin, rowMax))
                ImGui::SetTooltip("%s", v.status.c_str());
        } else
            text(draw, small, {at.x + ui(24), pathY}, u32(p.muted), fit(small, path, right - at.x - ui(24)));

        // Remove box, while the row is pointed at (like a tab's close box).
        if (inRow) {
            const ImVec2 boxMin(right - removeW + ui(2), y + ui(4)), size(ui(18), ui(18));
            ImGui::SetCursorScreenPos(boxMin);
            if (ImGui::InvisibleButton("##remove", size))
                remove = j;
            const bool hot = ImGui::IsItemHovered();
            if (hot) {
                draw->AddRectFilled(boxMin, {boxMin.x + size.x, boxMin.y + size.y},
                                    ImGui::GetColorU32(ImGuiCol_ButtonHovered), ui(2));
                ImGui::SetTooltip("Remove from plot (Del)\nCtrl+Z brings it back");
            }
            const float c = ui(4.5f);
            const ImVec2 m(boxMin.x + size.x * .5f, boxMin.y + size.y * .5f);
            const ImU32 xInk = u32(hot ? p.text : p.muted);
            draw->AddLine({m.x - c, m.y - c}, {m.x + c, m.y + c}, xInk, std::max(1.f, ui(1.5f)));
            draw->AddLine({m.x + c, m.y - c}, {m.x - c, m.y + c}, xInk, std::max(1.f, ui(1.5f)));
        }
        draw->AddLine({at.x, rowMax.y - .5f}, {right, rowMax.y - .5f}, ImGui::GetColorU32(ImGuiCol_Border, .6f));
        ImGui::PopID();
        y += g.row;
    }
    if (remove)
        state().deferred.push_back([&w, laneIndex, j = *remove] { removeSeries(w, laneIndex, j); });
    dropTarget(w, ImRect(at, {right, y}), ImGui::GetID("##table drop"), int(laneIndex));
}

// Eight theme swatches and a custom one; the current slot is ringed.
void colorRow(PlotWindow &w, Series &series, std::size_t laneIndex, std::size_t index) {
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Color");
    ImGui::SameLine(ui(70));
    const float size = ui(15);
    auto *draw = ImGui::GetWindowDrawList();
    for (int slot = 0; slot < kSlots; ++slot) {
        ImGui::PushID(slot);
        if (slot)
            ImGui::SameLine(0, ui(4));
        const ImVec2 at = ImGui::GetCursorScreenPos();
        if (ImGui::InvisibleButton("##swatch", {size, size})) {
            setSlot(w.spec.lanes[laneIndex], index, slot);
            ImGui::CloseCurrentPopup();
        }
        draw->AddRectFilled(at, {at.x + size, at.y + size}, u32(slotColor(slot)));
        if (!series.color && series.slot % kSlots == slot)
            draw->AddRect({at.x - ui(2.5f), at.y - ui(2.5f)}, {at.x + size + ui(2.5f), at.y + size + ui(2.5f)},
                          u32(palette().text), 0, 0, std::max(1.f, ui(1.5f)));
        ImGui::PopID();
    }
    ImGui::SameLine(0, ui(4));
    const ImVec2 at = ImGui::GetCursorScreenPos();
    if (ImGui::InvisibleButton("##custom", {size, size}))
        ImGui::OpenPopup("custom color");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Custom color");
    draw->AddRect(at, {at.x + size, at.y + size}, ImGui::GetColorU32(ImGuiCol_Border));
    const ImU32 ink = u32(palette().text);
    draw->AddLine({at.x + size * .5f, at.y + ui(3)}, {at.x + size * .5f, at.y + size - ui(3)}, ink);
    draw->AddLine({at.x + ui(3), at.y + size * .5f}, {at.x + size - ui(3), at.y + size * .5f}, ink);
    if (series.color)
        draw->AddRect({at.x - ui(2.5f), at.y - ui(2.5f)}, {at.x + size + ui(2.5f), at.y + size + ui(2.5f)}, ink, 0, 0,
                      std::max(1.f, ui(1.5f)));

    if (ImGui::BeginPopup("custom color")) {
        ImGui::TextUnformatted(("Color for " + series.label).c_str());
        const ImVec4 current = seriesColor(series);
        float rgb[3] = {current.x, current.y, current.z};
        ImGui::SetNextItemWidth(ui(240));
        if (ImGui::ColorPicker3("##picker", rgb,
                                ImGuiColorEditFlags_NoSidePreview | ImGuiColorEditFlags_NoSmallPreview |
                                    ImGuiColorEditFlags_DisplayHex | ImGuiColorEditFlags_PickerHueBar))
            series.color = toRgb(rgb);
        // A custom color is one value for every theme: how it reads on this one and on the opposite kind.
        const ImVec4 here = ImGui::GetStyle().Colors[ImGuiCol_WindowBg];
        const ImVec4 opposite = darkWindow() ? fromRgb(0xffffff) : fromRgb(0x15181c);
        const ImVec4 picked{rgb[0], rgb[1], rgb[2], 1};
        const float onHere = contrastRatio(picked, here), onOpposite = contrastRatio(picked, opposite);
        const auto note = [](const char *where, float ratio) {
            if (ratio >= 3)
                ImGui::TextDisabled("%s  %.1f : 1, clear", where, double(ratio));
            else {
                ImGui::PushStyleColor(ImGuiCol_Text, palette().warn);
                ImGui::Text("%s  %.1f : 1, hard to see", where, double(ratio));
                ImGui::PopStyleColor();
            }
        };
        note("This theme", onHere);
        note(darkWindow() ? "Light themes" : "Dark themes", onOpposite);
        if (ImGui::Button("Done"))
            ImGui::CloseCurrentPopup();
        ImGui::SameLine();
        if (ImGui::Button(series.slot >= kSlots ? "Back to dashed" : "Back to theme color")) {
            series.color.reset();
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

// A text field committed with Enter (or the button): returns the text once committed.
std::optional<std::string> nameField(const char *id, const std::string &initial, const char *action) {
    static char buffer[128];
    if (ImGui::IsWindowAppearing()) {
        std::snprintf(buffer, sizeof(buffer), "%s", initial.c_str());
        ImGui::SetKeyboardFocusHere();
    }
    ImGui::SetNextItemWidth(ui(200));
    const bool enter = ImGui::InputText(id, buffer, sizeof(buffer), ImGuiInputTextFlags_EnterReturnsTrue);
    ImGui::SameLine();
    if (enter || ImGui::Button(action)) {
        ImGui::CloseCurrentPopup();
        return std::string(buffer);
    }
    return std::nullopt;
}

void seriesMenu(PlotWindow &w, std::size_t laneIndex, std::size_t index) {
    auto &lane = w.spec.lanes[laneIndex];
    if (index >= lane.series.size())
        return;
    auto &series = lane.series[index];
    if (ImGui::MenuItem(series.hidden ? "Show" : "Hide", "click the key"))
        series.hidden = !series.hidden;
    colorRow(w, series, laneIndex, index);
    if (ImGui::BeginMenu("Move to lane")) {
        for (std::size_t l = 0; l < w.spec.lanes.size(); ++l) {
            if (l == laneIndex)
                continue;
            ImGui::PushID(int(l));
            const auto &other = w.spec.lanes[l];
            if (ImGui::MenuItem(other.label.empty() ? ("Lane " + std::to_string(l + 1)).c_str() : other.label.c_str()))
                state().deferred.push_back([&w, laneIndex, index, l] { moveTo(w, laneIndex, index, w, int(l)); });
            ImGui::PopID();
        }
        ImGui::Separator();
        ImGui::BeginDisabled(lane.series.size() == 1); // its own lane already
        if (ImGui::MenuItem("New lane"))
            state().deferred.push_back([&w, laneIndex, index] { moveTo(w, laneIndex, index, w, -1); });
        ImGui::EndDisabled();
        ImGui::EndMenu();
    }
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Draw as");
    ImGui::SameLine(ui(70));
    int style = series.steps ? 0 : 1;
    if (pins::Switch("##style", &style, {"Steps", "Lines"}))
        series.steps = style == 0;
    if (ImGui::BeginMenu("Rename")) {
        if (const auto name = nameField("##rename", series.label, "Rename"); name && !name->empty())
            series.label = *name;
        ImGui::EndMenu();
    }
    if (!series.minus && ImGui::BeginMenu("Difference with")) {
        bool any = false;
        for (std::size_t l = 0; l < w.spec.lanes.size(); ++l)
            for (std::size_t k = 0; k < w.spec.lanes[l].series.size(); ++k) {
                const auto &other = w.spec.lanes[l].series[k];
                if ((l == laneIndex && k == index) || other.minus)
                    continue;
                any = true;
                ImGui::PushID(int(l * 1000 + k));
                if (ImGui::MenuItem(other.label.c_str()))
                    state().deferred.push_back([&w, laneIndex, index, l, k] {
                        const auto &a = w.spec.lanes[laneIndex].series[index];
                        const auto &b = w.spec.lanes[l].series[k];
                        Lane lane;
                        lane.label = a.label + " - " + b.label;
                        lane.unit = w.spec.lanes[laneIndex].unit;
                        Series d;
                        d.label = lane.label;
                        d.source = a.source;
                        d.minus = b.source;
                        d.slot = 2;
                        lane.series.push_back(d);
                        w.spec.lanes.push_back(lane);
                        w.undo.reset();
                    });
                ImGui::PopID();
            }
        if (!any)
            ImGui::TextDisabled("Add another series first");
        ImGui::EndMenu();
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Remove from plot", "Del"))
        state().deferred.push_back([&w, laneIndex, index] { removeSeries(w, laneIndex, index); });
}

void laneMenu(PlotWindow &w, std::size_t laneIndex) {
    auto &lane = w.spec.lanes[laneIndex];
    if (ImGui::MenuItem("Add series\u2026")) {
        w.addLane = int(laneIndex);
        w.openAdd = true;
    }
    ImGui::Separator();
    if (ImGui::BeginMenu("Rename lane")) {
        if (const auto name = nameField("##lane", lane.label, "Rename"))
            lane.label = *name;
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Unit")) {
        if (const auto unit = nameField("##unit", lane.unit, "Set"))
            lane.unit = *unit;
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Limits")) {
        static double low = 0, high = 0;
        if (ImGui::IsWindowAppearing()) {
            low = lane.limits ? lane.limits->first : -1;
            high = lane.limits ? lane.limits->second : 1;
        }
        ImGui::TextDisabled("Drawn as the lane's outer ticks");
        ImGui::SetNextItemWidth(ui(90));
        ImGui::InputDouble("Low", &low, 0, 0, "%.3g");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(ui(90));
        ImGui::InputDouble("High", &high, 0, 0, "%.3g");
        ImGui::BeginDisabled(!(high > low));
        if (ImGui::Button("Set")) {
            lane.limits = std::make_pair(low, high);
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(!lane.limits);
        if (ImGui::Button("Clear")) {
            lane.limits.reset();
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndDisabled();
        ImGui::EndMenu();
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Remove lane"))
        state().deferred.push_back([&w, laneIndex] { removeLane(w, laneIndex); });
}

// --- Saved plots ---

std::string fileStem(const std::string &name) {
    std::string out;
    for (const char c : name)
        out += std::isalnum(static_cast<unsigned char>(c)) || c == ' ' || c == '-' || c == '_' ? c : '_';
    while (!out.empty() && out.back() == ' ')
        out.pop_back();
    return out;
}

const std::vector<std::string> &savedPlots() {
    auto &s = state();
    if (steadyNow() - s.savedAt < 2)
        return s.saved;
    s.savedAt = steadyNow();
    s.saved.clear();
    std::error_code error;
    for (const auto &entry : fs::directory_iterator(s.options.savedDir, error))
        if (entry.path().extension() == ".yaml")
            s.saved.push_back(entry.path().stem().string());
    std::sort(s.saved.begin(), s.saved.end());
    return s.saved;
}

void savePlot(PlotWindow &w, const std::string &name) {
    auto &s = state();
    const auto stem = fileStem(name);
    if (stem.empty() || s.options.savedDir.empty())
        return;
    try {
        fs::create_directories(s.options.savedDir);
        PlotSpec spec = w.spec;
        spec.span = viewOf(w).span;
        std::ofstream out(s.options.savedDir / (stem + ".yaml"));
        out << "# A Nereus saved plot: lanes and series, not data. Open it from Windows > Plots or Ctrl+P.\n";
        YAML::Emitter emitter;
        emitter << toYaml(spec);
        out << emitter.c_str() << "\n";
        if (!out)
            throw std::runtime_error("write failed");
        w.message = "Saved as " + stem;
    } catch (const std::exception &error) {
        w.message = std::string("Could not save: ") + error.what();
    }
    w.messageUntil = steadyNow() + 4;
    s.savedAt = -1e9;
}

void openSaved(const std::string &stem) {
    auto &s = state();
    try {
        const auto file = s.options.savedDir / (stem + ".yaml");
        auto spec = fromYaml(YAML::LoadFile(file.string()), file.filename().string());
        addPlot(std::move(spec));
    } catch (const std::exception &error) {
        std::fprintf(stderr, "nereus-viewer: saved plot %s: %s\n", stem.c_str(), error.what());
    }
}

void openFolder(const fs::path &folder) {
    std::error_code error;
    fs::create_directories(folder, error);
    std::string quoted = "'";
    for (const char c : folder.string())
        quoted += c == '\'' ? std::string("'\\''") : std::string(1, c);
    quoted += "'";
    const int result = std::system(("xdg-open " + quoted + " >/dev/null 2>&1 &").c_str());
    (void)result;
}

void drawManage() {
    auto &s = state();
    if (!s.manageOpen)
        return;
    ImGui::SetNextWindowSize({ui(420), ui(320)}, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Saved plots###plots_saved", &s.manageOpen)) {
        const auto saved = savedPlots();
        if (saved.empty())
            emptyState("No saved plots yet. In a plot window, choose ··· > Save as preset.");
        for (const auto &name : saved) {
            ImGui::PushID(name.c_str());
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(name.c_str());
            ImGui::SameLine(ImGui::GetContentRegionMax().x - ui(200));
            if (ImGui::Button("Open"))
                openSaved(name);
            ImGui::SameLine();
            if (ImGui::Button("Rename"))
                ImGui::OpenPopup("rename saved");
            ImGui::SameLine();
            if (ImGui::Button("Delete")) {
                std::error_code error;
                fs::remove(s.options.savedDir / (name + ".yaml"), error);
                s.savedAt = -1e9;
            }
            if (ImGui::BeginPopup("rename saved")) {
                if (const auto to = nameField("##to", name, "Rename"); to && !fileStem(*to).empty()) {
                    std::error_code error;
                    fs::rename(s.options.savedDir / (name + ".yaml"), s.options.savedDir / (fileStem(*to) + ".yaml"),
                               error);
                    s.savedAt = -1e9;
                }
                ImGui::EndPopup();
            }
            ImGui::PopID();
        }
        ImGui::Spacing();
        if (ImGui::Button("Open folder"))
            openFolder(s.options.savedDir);
        ImGui::SameLine();
        ImGui::TextDisabled("%s", s.options.savedDir.string().c_str());
    }
    ImGui::End();
}

// --- CSV export ---

void exportCsv(PlotWindow &w) {
    auto &s = state();
    const auto &view = viewOf(w);
    const double t1 = view.end, t0 = t1 - view.span;
    try {
        fs::create_directories(s.options.exportDir);
        char stamp[32];
        const std::time_t wall = std::time(nullptr);
        std::tm local{};
        localtime_r(&wall, &local);
        std::strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", &local);
        const auto file = s.options.exportDir / (fileStem(w.spec.title) + "-" + stamp + ".csv");
        std::ofstream out(file);
        out << "lane,series,source,time_s,value\n";
        bindLive(w);
        for (std::size_t l = 0; l < w.spec.lanes.size() && l < w.live.size(); ++l)
            for (std::size_t k = 0; k < w.spec.lanes[l].series.size() && k < w.live[l].size(); ++k) {
                const auto &series = w.spec.lanes[l].series[k];
                const auto &live = w.live[l][k];
                if (!live.a)
                    continue;
                SeriesBuffer copy;
                {
                    std::lock_guard<std::mutex> lock(live.a->mutex);
                    copy = live.a->samples;
                }
                if (live.b) {
                    std::lock_guard<std::mutex> lock(live.b->mutex);
                    copy = difference(copy, live.b->samples, t0, t1, w.spec.time);
                }
                const std::string source = series.minus ? sourceText(series.source) + " - " + sourceText(*series.minus)
                                                        : sourceText(series.source);
                for (std::size_t i = copy.lowerBound(t0, w.spec.time); i < copy.size(); ++i) {
                    const auto &sample = copy.at(i);
                    const double t = timeOf(sample, w.spec.time);
                    if (t > t1)
                        break;
                    out << '"' << w.spec.lanes[l].label << "\",\"" << series.label << "\",\"" << source << "\","
                        << format("%.6f", t) << ',' << format("%.9g", sample.value) << '\n';
                }
            }
        if (!out)
            throw std::runtime_error("write failed");
        w.message = "Saved " + file.string();
    } catch (const std::exception &error) {
        w.message = std::string("Could not export: ") + error.what();
    }
    w.messageUntil = steadyNow() + 6;
}

// --- Search (+ Series and Ctrl+P) ---

const std::vector<IndexEntry> &searchIndex() {
    auto &s = state();
    if (!s.hub || steadyNow() - s.indexAt < 3)
        return s.index;
    s.indexAt = steadyNow();
    s.index.clear();
    for (const auto &[id, info] : s.figures)
        s.index.push_back({figureSource(id), info.from + " · " + info.label});
    auto topics = s.hub->topics();
    std::sort(topics.begin(), topics.end(), [](const auto &a, const auto &b) { return a.name < b.name; });
    for (const auto &topic : topics) {
        std::shared_ptr<const MessageType> type;
        try {
            type = MessageType::get(topic.type);
        } catch (const std::exception &) {
            continue;
        }
        std::vector<std::string> paths;
        // A topic with a message at hand lists its sequences' elements; otherwise the type's fixed fields.
        if (const auto latest = s.hub->latest(topic.name)) {
            Message message(type);
            if (message.deserialize(*latest))
                paths = numericPaths(*type, message.data(), 8);
        }
        if (paths.empty()) {
            auto &cached = s.typePaths[topic.type];
            if (cached.empty())
                cached = numericPaths(*type, nullptr, 8);
            paths = cached;
        }
        const std::string relative = s.hub->relative(topic.name);
        for (const auto &path : paths) {
            Source source;
            source.topic = relative;
            source.field = path;
            s.index.push_back({source, relative + " › " + path});
        }
    }
    return s.index;
}

// Every whitespace-separated word of `query` appears in `text` (case-insensitive).
bool matches(const std::string &text, const std::string &query) {
    std::string lower = text;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) { return char(std::tolower(c)); });
    std::istringstream words(query);
    std::string word;
    while (words >> word) {
        std::transform(word.begin(), word.end(), word.begin(), [](unsigned char c) { return char(std::tolower(c)); });
        if (lower.find(word) == std::string::npos)
            return false;
    }
    return true;
}

void addSeriesPopup(PlotWindow &w) {
    auto &s = state();
    if (!ImGui::BeginPopup("add series"))
        return;
    if (ImGui::IsWindowAppearing()) {
        s.search[0] = 0;
        ImGui::SetKeyboardFocusHere();
    }
    ImGui::SetNextItemWidth(ui(420));
    ImGui::InputTextWithHint("##find", "Topic or field, e.g. odom z", s.search, sizeof(s.search));
    const bool toLane = w.addLane >= 0 && std::size_t(w.addLane) < w.spec.lanes.size();
    const std::string into = toLane ? w.spec.lanes[std::size_t(w.addLane)].label : std::string();
    ImGui::TextDisabled("%s  \u00B7  Shift-click adds several",
                        toLane ? ("Adds to " + (into.empty() ? "lane " + std::to_string(w.addLane + 1) : into)).c_str()
                               : "Starts a new lane");
    ImGui::Separator();
    int shown = 0;
    if (ImGui::BeginChild("##results", {ui(420), ui(260)})) {
        for (const auto &entry : searchIndex()) {
            if (!matches(entry.text, s.search))
                continue;
            if (++shown > 60)
                break;
            ImGui::PushID(shown);
            if (ImGui::Selectable(entry.text.c_str(), false, ImGuiSelectableFlags_NoAutoClosePopups)) {
                addSource(w, entry.source, toLane ? w.addLane : -1);
                if (!toLane) // the next Shift-click joins the lane this one started
                    w.addLane = int(w.spec.lanes.size()) - 1;
                if (!ImGui::GetIO().KeyShift)
                    ImGui::CloseCurrentPopup();
            }
            ImGui::PopID();
        }
        if (!shown)
            ImGui::TextDisabled(s.hub ? "Nothing matches" : "Not connected to ROS");
    }
    ImGui::EndChild();
    ImGui::EndPopup();
}

// --- The plot window ---

void plotMenu(PlotWindow &w) {
    auto &s = state();
    if (!ImGui::BeginPopup("plot menu"))
        return;
    if (ImGui::BeginMenu("Rename")) {
        if (const auto name = nameField("##title", w.spec.title, "Rename"); name && !name->empty())
            w.spec.title = *name;
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Save as preset")) {
        static char name[96];
        if (ImGui::IsWindowAppearing()) {
            std::snprintf(name, sizeof(name), "%s", w.spec.title.c_str());
            ImGui::SetKeyboardFocusHere();
        }
        ImGui::TextDisabled("Name");
        ImGui::SetNextItemWidth(ui(230));
        const bool enter = ImGui::InputText("##preset", name, sizeof(name), ImGuiInputTextFlags_EnterReturnsTrue);
        const auto stem = fileStem(name);
        const auto &saved = savedPlots();
        const bool exists = std::find(saved.begin(), saved.end(), stem) != saved.end();
        ImGui::BeginDisabled(stem.empty());
        pushActiveColors(true);
        const bool clicked = ImGui::Button(exists ? "Replace" : "Save", {ui(110), 0});
        popActiveColors();
        ImGui::EndDisabled();
        if ((enter || clicked) && !stem.empty()) {
            savePlot(w, name);
            ImGui::CloseCurrentPopup();
        }
        if (exists)
            ImGui::TextDisabled("A saved plot has this name");
        ImGui::PushTextWrapPos(ui(250));
        ImGui::TextDisabled("Saved plots keep the lanes and series, not the data. Open them from Windows > Plots or "
                            "Ctrl+P.");
        ImGui::PopTextWrapPos();
        ImGui::EndMenu();
    }
    ImGui::Separator();
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Time from");
    ImGui::SameLine(ui(110));
    int base = w.spec.time == TimeBase::Header ? 0 : 1;
    if (pins::Switch("##timebase", &base, {"Header", "Receipt"}))
        w.spec.time = base == 0 ? TimeBase::Header : TimeBase::Receipt;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Header: when the data was measured (its stamp). Receipt: when it reached this computer.\n"
                          "Messages without a header always use receipt time.");
    bool own = w.spec.ownTime;
    if (ImGui::MenuItem("Own time span", "not shared", &own)) {
        if (own) {
            w.own = s.shared; // starts where the shared timeline is
        } else
            w.spec.span = s.shared.span;
        w.spec.ownTime = own;
    }
    if (ImGui::MenuItem("Export visible span as CSV"))
        exportCsv(w);
    ImGui::Separator();
    if (ImGui::MenuItem("Delete plot"))
        w.remove = true;
    ImGui::EndPopup();
}

// The clock at the toolbar's right: sim seconds, or the time of day on a real robot.
std::string clockText(const TimeView &view) {
    auto &s = state();
    const double t = view.end;
    std::string value;
    if (s.options.useSimTime)
        value = number(t, 1) + " s";
    else
        value = timeLabel(t, t, false, 1, false);
    return (view.paused ? "held at " : s.options.useSimTime ? "sim time " : "") + value;
}

// A dragged series: its plot, lane, index, and source key (checked on drop, in case the plot changed meanwhile).
struct SeriesRef {
    int plot = 0;
    std::size_t lane = 0, index = 0;
    std::string key;
};

std::optional<SeriesRef> seriesRef(const ImGuiPayload *payload) {
    if (!payload || !payload->IsDataType(kSeriesPayload))
        return std::nullopt;
    std::istringstream in(std::string(static_cast<const char *>(payload->Data), std::size_t(payload->DataSize)));
    SeriesRef ref;
    std::string plot, lane, index;
    if (!std::getline(in, plot, '\t') || !std::getline(in, lane, '\t') || !std::getline(in, index, '\t'))
        return std::nullopt;
    std::getline(in, ref.key);
    ref.plot = std::atoi(plot.c_str());
    ref.lane = std::size_t(std::atol(lane.c_str()));
    ref.index = std::size_t(std::atol(index.c_str()));
    return ref;
}

// Accepts a dragged field or series over `rect`: into `lane` (-1: a new lane). A series over its own lane is not
// a drop.
void dropTarget(PlotWindow &w, const ImRect &rect, ImGuiID id, int lane) {
    if (const auto ref = seriesRef(ImGui::GetDragDropPayload()); ref && ref->plot == w.id && int(ref->lane) == lane)
        return;
    ImGui::PushStyleColor(ImGuiCol_DragDropTarget, palette().active);
    if (ImGui::BeginDragDropTargetCustom(rect, id)) {
        if (const auto ref = seriesRef(ImGui::AcceptDragDropPayload(kSeriesPayload)))
            state().deferred.push_back([&w, ref = *ref, lane] {
                auto *from = plotById(ref.plot);
                if (from && ref.lane < from->spec.lanes.size() &&
                    ref.index < from->spec.lanes[ref.lane].series.size() &&
                    from->spec.lanes[ref.lane].series[ref.index].source.key() == ref.key)
                    moveTo(*from, ref.lane, ref.index, w, lane);
            });
        if (const auto *payload = ImGui::AcceptDragDropPayload(kPayload)) {
            const std::string data(static_cast<const char *>(payload->Data), std::size_t(payload->DataSize));
            const auto split = data.find('\n');
            Source source;
            if (data.rfind("figure:", 0) == 0) {
                source = figureSource(data.substr(7));
            } else if (split != std::string::npos) {
                source.topic = data.substr(0, split);
                source.field = data.substr(split + 1);
            }
            state().deferred.push_back([&w, source, lane] { addSource(w, source, lane); });
        }
        ImGui::EndDragDropTarget();
    }
    ImGui::PopStyleColor();
}

// A field or a series is being dragged (plots show the new-lane strip).
bool draggingIntoPlots() {
    const auto *payload = ImGui::GetDragDropPayload();
    return payload && (payload->IsDataType(kPayload) || payload->IsDataType(kSeriesPayload));
}

void drawPlot(PlotWindow &w) {
    auto &s = state();
    const auto &p = palette();
    TimeView &view = viewOf(w);
    const double current = now();
    if (!view.paused)
        view.end = current;
    if (!w.spec.ownTime)
        w.spec.span = view.span;
    const double t1 = view.end, t0 = t1 - view.span;
    double &hoverNext = w.spec.ownTime ? w.hoverNext : s.hoverNext;
    const double hover = w.spec.ownTime ? w.hover : s.hover;
    auto *draw = ImGui::GetWindowDrawList();

    // Toolbar: Live / Paused, span, + Lane, the menu, and the clock (or a passing message) at the right. Series are
    // added per lane, from the + in each lane's table head.
    int mode = view.paused ? 1 : 0;
    if (pins::Switch("##timeline", &mode, {"Live", "Paused"})) {
        view.paused = mode == 1;
        if (view.paused)
            view.end = current;
    }
    ImGui::SameLine();
    std::string spanLabel = "Last " + number(view.span, 0) + " s";
    for (std::size_t i = 0; i < std::size(kSpans); ++i)
        if (std::abs(kSpans[i] - view.span) < 1e-6)
            spanLabel = kSpanLabels[i];
    ImGui::SetNextItemWidth(ui(118));
    if (ImGui::BeginCombo("##span", spanLabel.c_str())) {
        for (std::size_t i = 0; i < std::size(kSpans); ++i)
            if (ImGui::Selectable(kSpanLabels[i], std::abs(kSpans[i] - view.span) < 1e-6))
                view.span = kSpans[i];
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    if (ImGui::Button("+ Lane")) {
        w.addLane = -1;
        ImGui::OpenPopup("add series");
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("A new lane: pick its first series. The + in a lane's head adds to that lane.");
    if (w.openAdd) {
        ImGui::OpenPopup("add series");
        w.openAdd = false;
    }
    ImGui::SameLine();
    if (ImGui::Button("···"))
        ImGui::OpenPopup("plot menu");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Rename, save as preset, time source, own time span, export, delete");
    addSeriesPopup(w);
    plotMenu(w);
    {
        std::string right = clockText(view);
        ImVec4 ink = p.muted;
        if (steadyNow() < w.messageUntil)
            right = w.message;
        else if (steadyNow() < s.jumpUntil) {
            right = "Time went back " + number(s.jumpSeconds, 1) + " s, so the plots started over";
            ink = p.warn;
        }
        if (w.spec.ownTime)
            right = "own time  ·  " + right;
        const float width = ImGui::CalcTextSize(right.c_str()).x;
        const float x = ImGui::GetWindowContentRegionMax().x - width;
        ImGui::SameLine();
        if (x > ImGui::GetCursorPosX())
            ImGui::SetCursorPosX(x);
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(ink, "%s", right.c_str());
    }

    // Ctrl+Z brings back the last series or lane removed (nothing stays on screen to say so).
    if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) &&
        ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_Z))
        s.deferred.push_back([&w] { undo(w); });

    ImGui::Dummy({0, ui(2)});
    const ImVec2 origin = ImGui::GetCursorScreenPos(), avail = ImGui::GetContentRegionAvail();
    const bool dragging = draggingIntoPlots();

    // Nothing plotted yet: say how to start, and take a dropped field.
    if (w.spec.lanes.empty()) {
        const ImRect box(origin, {origin.x + avail.x, origin.y + std::max(ui(140), std::min(avail.y, ui(200)))});
        draw->AddRect(box.Min, box.Max, ImGui::GetColorU32(ImGuiCol_Border));
        ImGui::SetCursorScreenPos({box.Min.x + ui(20), box.Min.y + ui(26)});
        ImGui::PushTextWrapPos(box.Max.x - ui(20));
        if (typeRamp().strong)
            ImGui::PushFont(typeRamp().strong);
        ImGui::TextUnformatted("Nothing plotted yet");
        if (typeRamp().strong)
            ImGui::PopFont();
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ui(20));
        emptyState("Drag a numeric field here from Topics, choose + Lane and type its name, or right-click a figure "
                   "in Motion and choose Plot.");
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ui(20));
        emptyState("Plots you saved are under Windows > Plots.");
        ImGui::PopTextWrapPos();
        ImGui::SetCursorScreenPos(box.Min);
        ImGui::Dummy(box.GetSize());
        dropTarget(w, box, ImGui::GetID("##empty drop"), -1);
        return;
    }

    // Layout: lanes on the left with their value labels, tables at the right (wide) or under the plot (narrow).
    const float labelW = ui(48), gapY = ui(12), axisH = ui(22);
    const bool wide = avail.x >= ui(640);
    const float tableW = wide ? std::clamp(avail.x * .38f, ui(300), ui(440)) : avail.x;
    const ImVec2 plotX(origin.x + labelW, origin.x + (wide ? avail.x - tableW - ui(18) : avail.x) - ui(6));
    const std::size_t lanes = w.spec.lanes.size();
    const float dropH = dragging ? ui(46) : 0;
    const float fillY = std::floor(avail.y) - 1; // a pixel of slack: rounding never tips the lanes into a scrollbar
    // Rows drop their path line (into a tooltip) when the full tables would crowd the lanes.
    const auto layoutFor = [&](bool compact, float &tablesH, float &laneH) {
        tablesH = 0;
        if (!wide)
            for (std::size_t l = 0; l < lanes; ++l)
                tablesH += tableHeight(w, l, compact) + ui(10);
        const float lanesH = fillY - axisH - dropH - (wide ? 0 : tablesH + ui(6));
        laneH = (lanesH - gapY * float(lanes - 1)) / float(lanes);
        float need = ui(58);
        if (wide)
            for (std::size_t l = 0; l < lanes; ++l)
                need = std::max(need, tableHeight(w, l, compact) - ui(4));
        const bool fits = laneH >= need;
        laneH = std::max(laneH, need);
        return fits;
    };
    float tablesH = 0, laneH = 0;
    const bool compact = !layoutFor(false, tablesH, laneH);
    if (compact)
        layoutFor(true, tablesH, laneH);
    const int columns = std::max(1, int((plotX.y - plotX.x) / std::max(1.f, ui(1))));

    // Mouse over the lanes: the cursor time; Ctrl+wheel zooms around it (the wheel alone scrolls the window, as
    // everywhere else), drag scrubs (and pauses), double-click goes live.
    const ImRect lanesRect({plotX.x, origin.y}, {plotX.y, origin.y + laneH * float(lanes) + gapY * float(lanes - 1)});
    ImGui::SetCursorScreenPos(lanesRect.Min);
    ImGui::InvisibleButton("##lanes", {lanesRect.GetWidth(), lanesRect.GetHeight() + axisH});
    const auto &io = ImGui::GetIO();
    const auto toTime = [&](float x) { return t0 + (x - plotX.x) / (plotX.y - plotX.x) * (t1 - t0); };
    if (ImGui::IsItemHovered()) {
        hoverNext = std::clamp(toTime(io.MousePos.x), t0, t1);
        if (io.KeyCtrl && io.MouseWheel != 0) { // ImGui never scrolls a window while Ctrl is held
            const double scale = io.MouseWheel > 0 ? .8 : 1.25;
            const double span = std::clamp(view.span * scale, .5, 3600.0);
            if (view.paused) {
                const double pivot = hoverNext;
                view.end = std::min(current, pivot + (view.end - pivot) * span / view.span);
            }
            view.span = span;
        }
        if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
            view.paused = false;
            view.span = 30;
        }
    }
    if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left, ui(2))) {
        if (!view.paused) {
            view.paused = true;
            view.end = current;
        }
        view.end = std::min(current, view.end - io.MouseDelta.x / (plotX.y - plotX.x) * view.span);
    }

    const bool cursorShown = std::isfinite(hover) && hover >= t0 && hover <= t1;
    const std::string cursorText = cursorShown ? cursorLabel(hover, t1, !view.paused) : "";
    ImFont *small = fontOr(typeRamp().small);
    const ImU32 grid = ImGui::GetColorU32(ImGuiCol_Border, .55f), timeGrid = ImGui::GetColorU32(ImGuiCol_Border, .3f);
    const Ticks timeTicks = view.paused ? niceTicks(t0, t1, std::max(2, int((plotX.y - plotX.x) / ui(90))))
                                        : niceTicks(-view.span, 0, std::max(2, int((plotX.y - plotX.x) / ui(90))));

    std::vector<std::vector<View>> laneViews(lanes);
    std::vector<std::pair<double, double>> ranges(lanes);
    float y = origin.y;
    for (std::size_t l = 0; l < lanes; ++l) {
        auto &lane = w.spec.lanes[l];
        const ImRect r({plotX.x, y}, {plotX.y, y + laneH});
        for (std::size_t k = 0; k < lane.series.size(); ++k)
            laneViews[l].push_back(
                l < w.live.size() && k < w.live[l].size()
                    ? seriesView(lane.series[k], w.live[l][k], t0, t1, columns, w.spec.time, cursorShown ? hover : kNaN)
                    : View{});
        const auto [low, high] = laneRange(lane, laneViews[l]);
        ranges[l] = {low, high};

        // Field: a sunken white field in a bevelled theme; ruled grid elsewhere.
        if (bevelledTheme()) {
            draw->AddRectFilled(r.Min, r.Max, ImGui::GetColorU32(ImGuiCol_FrameBg));
            bevel(draw, {r.Min.x - 1, r.Min.y - 1}, {r.Max.x + 1, r.Max.y + 1}, false);
        }
        for (const double t : timeTicks.values) {
            const double at = view.paused ? t : t1 + t;
            const float x = timeX(at, t0, t1, r);
            if (x > r.Min.x + 1 && x < r.Max.x - 1)
                draw->AddLine({x, r.Min.y}, {x, r.Max.y}, timeGrid);
        }
        const auto toY = [&](double v) { return float(r.Max.y - (v - low) / (high - low) * r.GetHeight()); };
        const Ticks ticks = niceTicks(low, high, std::max(2, int(laneH / ui(30))));
        const int tickDecimals = stepDecimals(ticks.step);
        for (const double v : ticks.values) {
            const float ty = std::round(toY(v)) + .5f;
            const bool zero = std::abs(v) < ticks.step * 1e-6 && low < 0 && high > 0;
            draw->AddLine({r.Min.x, ty}, {r.Max.x, ty}, zero ? ImGui::GetColorU32(ImGuiCol_Border) : grid);
            const std::string label = number(v, tickDecimals);
            text(draw, small, {r.Min.x - ui(7) - textWidth(small, label), ty - small->FontSize * .5f}, u32(p.muted),
                 label);
        }
        if (lane.limits)
            for (const double v : {lane.limits->first, lane.limits->second}) {
                const float ty = std::round(toY(v)) + .5f;
                draw->AddLine({r.Min.x, ty}, {r.Max.x, ty}, u32(alpha(p.muted, .7f)));
                const std::string label = number(v, stepDecimals(ticks.step));
                if (std::none_of(ticks.values.begin(), ticks.values.end(),
                                 [&](double t) { return std::abs(toY(t) - toY(v)) < small->FontSize; }))
                    text(draw, small, {r.Min.x - ui(7) - textWidth(small, label), ty - small->FontSize * .5f},
                         u32(p.muted), label);
            }
        draw->AddLine({r.Min.x, r.Max.y - .5f}, {r.Max.x, r.Max.y - .5f}, ImGui::GetColorU32(ImGuiCol_Border));

        draw->PushClipRect(r.Min, {r.Max.x + 1, r.Max.y}, true);
        for (std::size_t k = 0; k < lane.series.size(); ++k)
            if (!lane.series[k].hidden)
                drawSeries(draw, laneViews[l][k], lane.series[k], t0, t1, r, low, high);
        draw->PopClipRect();

        // The cursor: one rule through every lane, a dot where it meets each line.
        if (cursorShown) {
            const float x = std::round(timeX(hover, t0, t1, r)) + .5f;
            draw->AddLine({x, r.Min.y - (l ? gapY : 0)}, {x, r.Max.y}, u32(p.text));
            for (std::size_t k = 0; k < lane.series.size(); ++k)
                if (!lane.series[k].hidden && std::isfinite(laneViews[l][k].cursor)) {
                    const ImVec2 dot(x, toY(laneViews[l][k].cursor));
                    if (dot.y >= r.Min.y && dot.y <= r.Max.y) {
                        draw->AddCircleFilled(dot, ui(5.5f), ImGui::GetColorU32(ImGuiCol_WindowBg));
                        draw->AddCircleFilled(dot, ui(3.5f), u32(seriesColor(lane.series[k])));
                    }
                }
        }
        dropTarget(w, r, ImGui::GetID(("##lane drop" + std::to_string(l)).c_str()), int(l));
        y += laneH + gapY;
    }
    y -= gapY;

    // Time axis under the last lane, the cursor's time on it.
    const ImRect last({plotX.x, y - laneH}, {plotX.y, y});
    for (std::size_t i = 0; i < timeTicks.values.size(); ++i) {
        const double t = timeTicks.values[i];
        const double at = view.paused ? t : t1 + t;
        const std::string label = timeLabel(at, t1, !view.paused, timeTicks.step, i == 0);
        float x = timeX(at, t0, t1, last) - textWidth(small, label) * .5f;
        x = std::clamp(x, plotX.x - ui(4), plotX.y - textWidth(small, label));
        text(draw, small, {x, y + ui(5)}, u32(p.muted), label);
    }
    if (cursorShown) {
        const float x = timeX(hover, t0, t1, last), width = textWidth(small, cursorText) + ui(10);
        const ImVec2 min(std::clamp(x - width * .5f, plotX.x - ui(6), plotX.y - width), y + ui(3));
        draw->AddRectFilled(min, {min.x + width, min.y + small->FontSize + ui(5)}, u32(p.text));
        text(draw, small, {min.x + ui(5), min.y + ui(2)}, ImGui::GetColorU32(ImGuiCol_WindowBg), cursorText);
    }
    y += axisH;

    // A strip for a new lane while a field is being dragged.
    if (dragging) {
        const ImRect strip({origin.x, y + ui(4)}, {origin.x + avail.x, y + dropH - ui(4)});
        draw->AddRect(strip.Min, strip.Max, ImGui::GetColorU32(ImGuiCol_Border));
        const char *hint = "Drop here for a new lane";
        const ImVec2 size = ImGui::CalcTextSize(hint);
        draw->AddText({strip.GetCenter().x - size.x * .5f, strip.GetCenter().y - size.y * .5f}, u32(p.muted), hint);
        dropTarget(w, strip, ImGui::GetID("##new lane drop"), -1);
        y += dropH;
    }

    // Tables: beside each lane, or stacked under the plot.
    float bottom = y;
    if (wide) {
        float ty = origin.y;
        for (std::size_t l = 0; l < lanes; ++l) {
            ImGui::PushID(int(l));
            drawTable(w, l, laneViews[l], {origin.x + avail.x - tableW, ty - ui(4)}, tableW, cursorShown, cursorText,
                      ranges[l].first, ranges[l].second, compact);
            ImGui::PopID();
            ty += laneH + gapY;
        }
    } else {
        float ty = y + ui(6);
        for (std::size_t l = 0; l < lanes; ++l) {
            ImGui::PushID(int(l));
            drawTable(w, l, laneViews[l], {origin.x, ty}, avail.x, cursorShown, cursorText, ranges[l].first,
                      ranges[l].second, compact);
            ImGui::PopID();
            ty += tableHeight(w, l, compact) + ui(10);
        }
        bottom = ty;
    }
    // The height used, exactly: lanes sized to the window leave no overflow, so no scrollbar until they truly do not
    // fit (more lanes or rows than the window holds at their smallest), and then the wheel scrolls.
    ImGui::SetCursorScreenPos(origin);
    ImGui::Dummy({1, std::max(1.f, bottom - origin.y)});
}

// --- The Topics browser ---

const Message *decodedLatest(const std::string &topic, const std::shared_ptr<const MessageType> &type) {
    auto &s = state();
    auto &slot = s.decoded[topic];
    if (!slot || &slot->type() != type.get())
        slot = std::make_unique<Message>(type);
    if (steadyNow() - s.decodedAt[topic] > .2) {
        s.decodedAt[topic] = steadyNow();
        if (const auto latest = s.hub->latest(topic))
            if (!slot->deserialize(*latest))
                return nullptr;
    }
    return s.hub->latest(topic) ? slot.get() : nullptr;
}

// Right-click on a leaf: plot it alone, or add it to a plot.
void fieldMenu(const Source &source) {
    if (!ImGui::BeginPopupContextItem("field menu"))
        return;
    if (ImGui::MenuItem("Plot in a new window"))
        newPlotWith(source);
    if (ImGui::BeginMenu("Add to", !state().plots.empty())) {
        for (auto &w : state().plots) {
            if (w.remove)
                continue;
            ImGui::PushID(w.id);
            if (ImGui::BeginMenu(w.spec.title.c_str())) {
                for (std::size_t l = 0; l < w.spec.lanes.size(); ++l) {
                    ImGui::PushID(int(l));
                    const auto &lane = w.spec.lanes[l];
                    if (ImGui::MenuItem(lane.label.empty() ? ("Lane " + std::to_string(l + 1)).c_str()
                                                           : lane.label.c_str())) {
                        addSource(w, source, int(l));
                        w.open = w.focus = true;
                    }
                    ImGui::PopID();
                }
                ImGui::Separator();
                if (ImGui::MenuItem("New lane")) {
                    addSource(w, source, -1);
                    w.open = w.focus = true;
                }
                ImGui::EndMenu();
            }
            ImGui::PopID();
        }
        ImGui::EndMenu();
    }
    ImGui::EndPopup();
}

// Right-click on an array of numbers: every element at once, a lane each (thrusters 1 to 8) or in one lane.
void arrayMenu(const std::string &topic, const MessageType &type, const Message *message, const Field &array) {
    if (!ImGui::BeginPopupContextItem("array menu"))
        return;
    std::vector<Source> elements;
    try {
        for (const auto &f : fieldsAt(type, array.path, message ? message->data() : nullptr, 64))
            if (f.numeric && f.name.front() == '[') {
                Source source;
                source.topic = state().hub->relative(topic);
                source.field = f.path;
                elements.push_back(source);
            }
    } catch (const std::exception &) {
    }
    if (elements.empty())
        ImGui::TextDisabled(message ? "Not an array of numbers" : "Waiting for a message to count its elements");
    else {
        const std::string title = fieldLabel(array.path);
        if (ImGui::MenuItem(("Plot each of the " + std::to_string(elements.size()) + " in its own lane").c_str())) {
            PlotSpec spec;
            spec.title = title;
            for (const auto &source : elements) {
                Lane lane = laneOf(source);
                lane.series[0].slot = 0;
                spec.lanes.push_back(lane);
            }
            addPlot(std::move(spec));
        }
        if (ImGui::MenuItem(("Plot all " + std::to_string(elements.size()) + " in one lane").c_str())) {
            PlotSpec spec;
            spec.title = title;
            Lane lane;
            lane.label = title;
            for (const auto &source : elements)
                lane.series.push_back(seriesOf(source, lane));
            spec.lanes.push_back(lane);
            addPlot(std::move(spec));
        }
    }
    ImGui::EndPopup();
}

void drawFields(const std::string &topic, const MessageType &type, const Message *message, const std::string &path,
                int depth) {
    auto &s = state();
    std::vector<Field> fields;
    try {
        fields = fieldsAt(type, path, message ? message->data() : nullptr, 64);
    } catch (const std::exception &) {
        return;
    }
    if (depth > 12)
        return;
    for (const auto &f : fields) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::PushID(f.path.c_str());
        if (f.expandable) {
            const bool open = ImGui::TreeNodeEx("##node", ImGuiTreeNodeFlags_SpanAvailWidth, "%s", f.name.c_str());
            arrayMenu(topic, type, message, f);
            ImGui::SameLine();
            ImGui::TextDisabled("%s", f.angle ? "°" : f.type.c_str());
            if (open) {
                drawFields(topic, type, message, f.path, depth + 1);
                ImGui::TreePop();
            }
        } else {
            ImGui::TreeNodeEx("##leaf",
                              ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen |
                                  ImGuiTreeNodeFlags_SpanAvailWidth,
                              "%s", f.name.c_str());
            const bool numeric = f.numeric;
            Source source;
            source.topic = s.hub->relative(topic);
            source.field = f.path;
            if (numeric) {
                if (ImGui::BeginDragDropSource()) {
                    const std::string data = source.topic + "\n" + source.field;
                    ImGui::SetDragDropPayload(kPayload, data.data(), data.size());
                    if (typeRamp().strong)
                        ImGui::PushFont(typeRamp().strong);
                    ImGui::TextUnformatted(f.path.c_str());
                    if (typeRamp().strong)
                        ImGui::PopFont();
                    ImGui::TextDisabled("%s", source.topic.c_str());
                    ImGui::EndDragDropSource();
                }
                if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    if (auto *w = plotById(s.focused)) {
                        addSource(*w, source, w->spec.lanes.empty() ? -1 : int(w->spec.lanes.size()) - 1);
                        w->open = w->focus = true;
                    } else
                        newPlotWith(source);
                }
                fieldMenu(source);
            }
            ImGui::SameLine();
            ImGui::TextDisabled("%s", f.angle ? "°" : numeric ? "" : f.type.c_str());
            ImGui::TableNextColumn();
            if (numeric && message) {
                auto &reader = s.readers[topic + "#" + f.path];
                if (!reader)
                    try {
                        reader = std::make_unique<FieldReader>(type, f.path);
                    } catch (const std::exception &) {
                    }
                if (reader) {
                    const double value = reader->read(message->data());
                    const std::string textValue = number(value, std::abs(value) >= 1000 ? 1 : 3);
                    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x -
                                         ImGui::CalcTextSize(textValue.c_str()).x);
                    ImGui::TextUnformatted(textValue.c_str());
                }
            }
            ImGui::TableNextColumn();
        }
        ImGui::PopID();
    }
}

void drawTopics() {
    auto &s = state();
    if (!s.topicsOpen)
        return;
    ImGui::SetNextWindowSize({ui(470), ui(600)}, ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Topics###topics", &s.topicsOpen)) {
        ImGui::End();
        return;
    }
    if (!s.hub) {
        emptyState("Connected, this lists the ROS topics and their fields to plot.");
        ImGui::End();
        return;
    }
    ImGui::SetNextItemWidth(-1);
    ImGui::InputTextWithHint("##filter", "Filter topics", s.filter, sizeof(s.filter));
    emptyState("Drag a field onto a plot; double-click adds it to the focused one. Values and rates show while a "
               "topic is open or plotted.");
    const ImGuiTableFlags flags = ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingStretchProp |
                                  (ruledTheme() ? ImGuiTableFlags_BordersInnerH : ImGuiTableFlags_RowBg);
    if (ImGui::BeginTable("topics", 3, flags)) {
        ImGui::TableSetupColumn("Topic / field", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthFixed, ui(82));
        ImGui::TableSetupColumn("Hz", ImGuiTableColumnFlags_WidthFixed, ui(48));
        ImGui::TableSetupScrollFreeze(0, 1);
        tableHeaders();
        auto topics = s.hub->topics();
        std::sort(topics.begin(), topics.end(),
                  [&](const auto &a, const auto &b) { return s.hub->relative(a.name) < s.hub->relative(b.name); });
        for (const auto &topic : topics) {
            const std::string relative = s.hub->relative(topic.name);
            if (!matches(relative + " " + topic.type, s.filter))
                continue;
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::PushID(topic.name.c_str());
            const bool open = ImGui::TreeNodeEx("##topic", ImGuiTreeNodeFlags_SpanAvailWidth, "%s", relative.c_str());
            ImGui::SameLine();
            ImGui::TextDisabled("%s", shortType(topic.type).c_str());
            ImGui::TableNextColumn();
            ImGui::TableNextColumn();
            const double rate = s.hub->rate(topic.name);
            const std::string hz = rate > 0 ? number(rate, 1) : "--";
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x -
                                 ImGui::CalcTextSize(hz.c_str()).x);
            if (rate > 0)
                ImGui::TextUnformatted(hz.c_str());
            else
                ImGui::TextDisabled("%s", hz.c_str());
            if (open) {
                s.hub->watch(topic.name);
                try {
                    const auto type = MessageType::get(topic.type);
                    drawFields(topic.name, *type, decodedLatest(topic.name, type), "", 0);
                } catch (const std::exception &error) {
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    ImGui::PushStyleColor(ImGuiCol_Text, palette().error);
                    ImGui::TextWrapped("Can't read this topic: %s", error.what());
                    ImGui::PopStyleColor();
                }
                ImGui::TreePop();
            }
            ImGui::PopID();
        }
        if (topics.empty()) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextDisabled("No topics yet");
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

// --- Figures (the panels' numbers) ---

void registerFigures() {
    auto &s = state();
    s.figures.clear();
    if (s.motion)
        for (int i = 0; i < 6; ++i) {
            const std::string id = std::string("motion.") + kAxisIds[i], unit = i < 3 ? "m" : "°";
            s.figures[id + ".actual"] = {std::string(kAxisNames[i]) + " actual", unit, "Motion"};
            s.figures[id + ".commanded"] = {std::string(kAxisNames[i]) + " commanded", unit, "Motion"};
            s.figures[id + ".error"] = {std::string(kAxisNames[i]) + " error", unit, "Motion: commanded - actual"};
        }
    s.readings.clear();
    if (s.telemetry)
        for (const auto &reading : s.telemetry->state().readings) {
            s.figures["telemetry." + reading.id] = {reading.label, reading.unit, "Telemetry"};
            s.readings.emplace_back(reading.id, reading.label);
        }
}

void sampleFigures() {
    auto &s = state();
    if (!s.hub || steadyNow() - s.lastSample < kFigurePeriod)
        return;
    s.lastSample = steadyNow();
    if (s.motion) {
        const auto m = s.motion->state();
        const auto actual = glm::degrees(glm::eulerAngles(glm::quat_cast(m.actual)));
        const auto sent = glm::degrees(glm::eulerAngles(glm::quat_cast(m.commanded)));
        for (int i = 0; i < 6; ++i) {
            const std::string id = std::string("motion.") + kAxisIds[i];
            const double a = i < 3 ? m.actual[3][i] : actual[i - 3], c = i < 3 ? m.commanded[3][i] : sent[i - 3];
            if (m.fresh)
                s.hub->pushFigure(id + ".actual", a);
            if (m.hasCommand)
                s.hub->pushFigure(id + ".commanded", c);
            if (m.fresh && m.hasCommand)
                s.hub->pushFigure(id + ".error", i < 3 ? c - a : std::remainder(c - a, 360.0));
        }
    }
    if (s.telemetry)
        for (const auto &reading : s.telemetry->state().readings) {
            auto &info = s.figures["telemetry." + reading.id];
            if (info.label.empty()) {
                info = {reading.label, reading.unit, "Telemetry"};
                s.readings.emplace_back(reading.id, reading.label);
            }
            if (!reading.unit.empty())
                info.unit = reading.unit;
            if (reading.level != panels::Level::Stale && std::isfinite(reading.number))
                s.hub->pushFigure("telemetry." + reading.id, reading.number);
        }
}

// --- ini ---

void readIniLine(const char *line) {
    auto &s = state();
    std::vector<std::string> fields;
    for (const char *at = line;;) {
        const char *tab = std::strchr(at, '\t');
        fields.emplace_back(at, tab ? tab : at + std::strlen(at));
        if (!tab)
            break;
        at = tab + 1;
    }
    if (fields.size() >= 2 && fields[0] == "timeline")
        s.iniSpan = std::atof(fields[1].c_str());
    else if (fields.size() >= 4 && fields[0] == "plot")
        s.iniPlots.push_back({std::atoi(fields[1].c_str()), fields[2] == "1", fields[3]});
}

void applyIni() {
    auto &s = state();
    if (!s.iniSeen)
        return; // a layout from before plots: keep the plots there are
    s.iniSeen = false;
    if (s.iniSpan > 0)
        s.shared.span = std::clamp(s.iniSpan, .5, 3600.0);
    s.plots.clear();
    for (const auto &entry : s.iniPlots) {
        try {
            PlotWindow w;
            w.id = entry.id;
            w.open = entry.open;
            w.spec = fromLine(entry.line, "plot " + std::to_string(entry.id));
            w.own.span = w.spec.span;
            s.plots.push_back(std::move(w));
            s.nextId = std::max(s.nextId, entry.id + 1);
        } catch (const std::exception &error) {
            std::fprintf(stderr, "nereus-viewer: layout plot %d: %s\n", entry.id, error.what());
        }
    }
    s.iniPlots.clear();
}

} // namespace

// --- Public API ---

void figureTooltip(const std::string &figure, const std::string &title, const std::string &detail) {
    auto &s = state();
    if (!s.hub) {
        if (!title.empty() || !detail.empty())
            ImGui::SetTooltip("%s%s%s", title.c_str(), detail.empty() || title.empty() ? "" : "\n", detail.c_str());
        return;
    }
    const auto channel = s.hub->channel(figureSource(figure));
    const auto &p = palette();
    ImGui::BeginTooltip();
    if (typeRamp().strong)
        ImGui::PushFont(typeRamp().strong);
    ImGui::TextUnformatted(title.c_str());
    if (typeRamp().strong)
        ImGui::PopFont();
    ImGui::SameLine();
    ImGui::TextDisabled("last 30 s");
    const ImVec2 size(ui(210), ui(52));
    const ImVec2 at = ImGui::GetCursorScreenPos();
    ImGui::Dummy(size);
    auto *draw = ImGui::GetWindowDrawList();
    const ImRect r({at.x, at.y + ui(2)}, {at.x + size.x, at.y + size.y - ui(2)});
    draw->AddLine({r.Min.x, r.Min.y - .5f}, {r.Max.x, r.Min.y - .5f}, ImGui::GetColorU32(ImGuiCol_Border, .6f));
    draw->AddLine({r.Min.x, r.Max.y + .5f}, {r.Max.x, r.Max.y + .5f}, ImGui::GetColorU32(ImGuiCol_Border, .6f));
    const double t1 = s.hub->now(), t0 = t1 - kTrendSeconds;
    View v;
    {
        std::lock_guard<std::mutex> lock(channel->mutex);
        v.drawn = decimate(channel->samples, t0, t1, int(size.x / 2), TimeBase::Receipt, 1.0);
    }
    const auto info = s.figures.find(figure);
    const std::string unit = info == s.figures.end() ? std::string() : info->second.unit;
    if (v.drawn.count) {
        double low = v.drawn.low, high = v.drawn.high;
        fitRange(low, high);
        Series line;
        line.slot = 0;
        drawSeries(draw, v, line, t0, t1, r, low, high);
        const int d = decimalsFor(high - low);
        ImGui::TextDisabled("Min");
        ImGui::SameLine(0, ui(4));
        ImGui::TextUnformatted(number(v.drawn.low, d).c_str());
        ImGui::SameLine(0, ui(14));
        ImGui::TextDisabled("Max");
        ImGui::SameLine(0, ui(4));
        ImGui::Text("%s %s", number(v.drawn.high, d).c_str(), unit.c_str());
    } else {
        const char *none = "No data in the last 30 s";
        const ImVec2 ts = ImGui::CalcTextSize(none);
        draw->AddText({r.GetCenter().x - ts.x * .5f, r.GetCenter().y - ts.y * .5f}, u32(p.muted), none);
    }
    if (!detail.empty()) {
        ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + ui(280));
        ImGui::PushStyleColor(ImGuiCol_Text, p.muted);
        ImGui::TextUnformatted(detail.c_str());
        ImGui::PopStyleColor();
        ImGui::PopTextWrapPos();
    }
    ImGui::TextDisabled("Right-click to plot");
    ImGui::EndTooltip();
}

void figureMenuItems(const std::string &group) {
    auto &s = state();
    const auto g = figureGroup(group);
    if (!g)
        return;
    beginSurface(Surface::Sheet);
    if (ImGui::MenuItem(("Plot " + g->title).c_str(), g->hint.c_str())) {
        PlotSpec spec;
        spec.title = g->title;
        spec.lanes = g->lanes;
        addPlot(std::move(spec));
    }
    if (ImGui::BeginMenu(("Add " + g->title + " to").c_str())) {
        for (auto &w : s.plots) {
            if (w.remove)
                continue;
            ImGui::PushID(w.id);
            if (ImGui::MenuItem(w.spec.title.c_str()))
                addLanes(w, g->lanes);
            ImGui::PopID();
        }
        if (!s.plots.empty())
            ImGui::Separator();
        if (ImGui::MenuItem("New plot")) {
            PlotSpec spec;
            spec.title = g->title;
            spec.lanes = g->lanes;
            addPlot(std::move(spec));
        }
        ImGui::EndMenu();
    }
    if (group.rfind("motion.", 0) == 0 && group != "motion.all") {
        ImGui::Separator();
        if (ImGui::MenuItem("Plot all six axes")) {
            const auto all = figureGroup("motion.all");
            PlotSpec spec;
            spec.title = all->title;
            spec.lanes = all->lanes;
            addPlot(std::move(spec));
        }
    }
    endSurface();
}

void install() {
    ImGuiSettingsHandler handler;
    handler.TypeName = "NereusPlots";
    handler.TypeHash = ImHashStr("NereusPlots");
    handler.ClearAllFn = [](ImGuiContext *, ImGuiSettingsHandler *) {
        auto &s = state();
        s.iniSeen = false;
        s.iniPlots.clear();
        s.iniSpan = 0;
    };
    handler.ReadOpenFn = [](ImGuiContext *, ImGuiSettingsHandler *, const char *) -> void * {
        state().iniSeen = true;
        return &state();
    };
    handler.ReadLineFn = [](ImGuiContext *, ImGuiSettingsHandler *, void *, const char *line) { readIniLine(line); };
    handler.ApplyAllFn = [](ImGuiContext *, ImGuiSettingsHandler *) { applyIni(); };
    handler.WriteAllFn = [](ImGuiContext *, ImGuiSettingsHandler *, ImGuiTextBuffer *out) {
        auto &s = state();
        out->append("[NereusPlots][Plots]\n");
        out->appendf("timeline\t%g\n", s.shared.span);
        for (const auto &w : s.plots) {
            if (w.remove)
                continue;
            PlotSpec spec = w.spec;
            spec.span = w.spec.ownTime ? w.own.span : s.shared.span;
            out->appendf("plot\t%d\t%d\t%s\n", w.id, w.open ? 1 : 0, toLine(spec).c_str());
        }
        out->append("\n");
    };
    ImGui::AddSettingsHandler(&handler);
}

void configure(const Options &options, const panels::Providers &providers) {
    auto &s = state();
    s.hub.reset();
    for (auto &w : s.plots) {
        w.live.clear();
        w.liveSignature.clear();
    }
    s.options = options;
    s.motion.reset();
    s.telemetry.reset();
    for (const auto &[id, provider] : providers) {
        if (!s.motion)
            s.motion = std::dynamic_pointer_cast<panels::Motion>(provider);
        if (!s.telemetry)
            s.telemetry = std::dynamic_pointer_cast<panels::Telemetry>(provider);
    }
    registerFigures();
    s.decoded.clear();
    s.readers.clear();
    s.index.clear();
    s.indexAt = -1e9;
    s.hub = std::make_unique<Hub>(options.robotNamespace, options.useSimTime);
}

void shutdown() {
    auto &s = state();
    for (auto &w : s.plots) {
        w.live.clear();
        w.liveSignature.clear();
    }
    s.decoded.clear();
    s.hub.reset();
}

void frame() {
    auto &s = state();
    s.hover = s.hoverNext;
    s.hoverNext = kNaN;
    for (auto &w : s.plots) {
        w.hover = w.hoverNext;
        w.hoverNext = kNaN;
    }
    if (!s.hub)
        return;
    sampleFigures();
    for (auto &w : s.plots)
        bindLive(w);
    s.hub->maintain();
    if (s.hub->jumps() != s.jumpsSeen) {
        s.jumpsSeen = s.hub->jumps();
        s.jumpSeconds = s.hub->lastJump();
        s.jumpUntil = steadyNow() + 6;
    }
}

void drawWindows() {
    auto &s = state();
    s.deferred.clear();
    // A plot opened just now docks beside an existing plot, else floats mid-window.
    ImGuiID dock = 0;
    for (const auto &w : s.plots)
        if (const auto *window = ImGui::FindWindowByName(w.name().c_str()); window && window->DockId && !w.place) {
            dock = window->DockId;
            break;
        }
    for (auto &w : s.plots) {
        if (!w.open || w.remove)
            continue;
        if (w.place) {
            if (dock)
                ImGui::SetNextWindowDockID(dock, ImGuiCond_Once);
            else {
                const auto *viewport = ImGui::GetMainViewport();
                ImGui::SetNextWindowPos(viewport->GetCenter(), ImGuiCond_Once, {.5f, .5f});
                ImGui::SetNextWindowSize({ui(780), ui(380)}, ImGuiCond_Once);
            }
            w.place = false;
        }
        if (w.focus) {
            ImGui::SetNextWindowFocus();
            w.focus = false;
        }
        if (ImGui::Begin(w.name().c_str(), &w.open)) {
            if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows))
                s.focused = w.id;
            drawPlot(w);
        }
        ImGui::End();
    }
    for (auto &action : s.deferred)
        action();
    s.deferred.clear();
    s.plots.remove_if([&](const PlotWindow &w) {
        if (w.remove && s.focused == w.id)
            s.focused = 0;
        return w.remove;
    });
    drawTopics();
    drawManage();
}

std::vector<Window> windows() {
    auto &s = state();
    std::vector<Window> out;
    for (auto &w : s.plots)
        if (!w.remove)
            out.push_back({w.key(), w.name(), w.spec.title, &w.open, true});
    out.push_back({"topics", "Topics###topics", "Topics", &s.topicsOpen, false});
    return out;
}

void drawMenu() {
    auto &s = state();
    for (auto &w : s.plots) {
        if (w.remove)
            continue;
        ImGui::PushID(w.id);
        if (ImGui::MenuItem(w.spec.title.c_str(), nullptr, w.open)) {
            if (w.open && ImGui::FindWindowByName(w.name().c_str()) &&
                ImGui::FindWindowByName(w.name().c_str())->WasActive)
                w.open = false;
            else
                w.open = w.focus = true;
        }
        ImGui::PopID();
    }
    const auto &saved = savedPlots();
    if (!saved.empty()) {
        if (!s.plots.empty())
            ImGui::Spacing();
        ImGui::TextDisabled("Saved");
        for (const auto &name : saved) {
            ImGui::PushID(name.c_str());
            if (ImGui::MenuItem(name.c_str()))
                openSaved(name);
            ImGui::PopID();
        }
    }
    ImGui::Separator();
    if (ImGui::MenuItem("New plot")) {
        PlotSpec spec;
        spec.title = "Plot " + std::to_string(s.nextId);
        addPlot(std::move(spec));
    }
    if (ImGui::MenuItem("Manage saved plots…"))
        s.manageOpen = true;
}

std::vector<Command> commands() {
    auto &s = state();
    std::vector<Command> out;
    if (!s.hub)
        return out;
    out.push_back({"Plot", "New plot", [] {
                       PlotSpec spec;
                       spec.title = "Plot " + std::to_string(state().nextId);
                       addPlot(std::move(spec));
                   }});
    if (s.motion) {
        for (int i = 0; i < 6; ++i)
            out.push_back({"Plot", std::string("Motion · ") + kAxisNames[i] + ": actual, commanded, error", [i] {
                               const auto g = figureGroup(std::string("motion.") + kAxisIds[i]);
                               if (auto *w = ImGui::GetIO().KeyShift ? plotById(state().focused) : nullptr)
                                   addLanes(*w, g->lanes);
                               else {
                                   PlotSpec spec;
                                   spec.title = g->title;
                                   spec.lanes = g->lanes;
                                   addPlot(std::move(spec));
                               }
                           }});
        out.push_back({"Plot", "Motion · all six axes", [] {
                           const auto g = figureGroup("motion.all");
                           PlotSpec spec;
                           spec.title = g->title;
                           spec.lanes = g->lanes;
                           addPlot(std::move(spec));
                       }});
    }
    for (const auto &entry : searchIndex()) {
        if (entry.source.kind == Source::Kind::Figure && entry.source.figure.rfind("motion.", 0) == 0)
            continue; // the axes above cover Motion
        const Source source = entry.source;
        out.push_back({"Plot", entry.text, [source] { plotSource(source); }});
    }
    for (const auto &name : savedPlots())
        out.push_back({"Open plot", name, [name] { openSaved(name); }});
    return out;
}

} // namespace nereus::ros_viewer::plots

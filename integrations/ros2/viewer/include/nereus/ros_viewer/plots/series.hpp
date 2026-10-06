// Plot data without ROS or ImGui: a time-ordered ring buffer of samples, the min / max decimation that draws any
// number of samples as at most two points per pixel column, and "nice" axis ticks.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <vector>

namespace nereus::ros_viewer::plots {

// One sample: the message's own time (header stamp; the receipt time when it has none), the time it arrived on this
// computer, and the value. Times are seconds on the ROS clock the viewer runs on.
struct Sample {
    double stamp = 0, receipt = 0;
    float value = 0;
};

// Which of a sample's two times a plot uses.
enum class TimeBase { Header, Receipt };

inline double timeOf(const Sample &s, TimeBase base) {
    return base == TimeBase::Header ? s.stamp : s.receipt;
}

// Samples in arrival order, oldest dropped once `capacity` is reached or they are older than `history` seconds
// (by receipt time) behind the newest. Indexing is logical: 0 is the oldest kept sample.
class SeriesBuffer {
  public:
    explicit SeriesBuffer(std::size_t capacity = 120000, double history = 600)
        : capacity_(std::max<std::size_t>(capacity, 2)), history_(history) {}

    void push(const Sample &s) {
        data_.push_back(s);
        if (data_.size() > capacity_)
            data_.pop_front();
        // Age out by receipt time (the clock this computer controls), keeping at least the newest sample.
        while (data_.size() > 1 && s.receipt - data_.front().receipt > history_)
            data_.pop_front();
    }

    void clear() {
        data_.clear();
    }
    void setHistory(double seconds) {
        history_ = seconds;
    }

    std::size_t size() const {
        return data_.size();
    }
    bool empty() const {
        return data_.empty();
    }
    const Sample &at(std::size_t i) const {
        return data_[i];
    }
    const Sample &back() const {
        return data_.back();
    }

    // First index whose time (in `base`) is >= t; size() when none. Assumes times rise with the index, which
    // receipt times always do and header stamps nearly always do (a stray older stamp only blurs one column).
    std::size_t lowerBound(double t, TimeBase base) const {
        std::size_t lo = 0, hi = size();
        while (lo < hi) {
            const std::size_t mid = (lo + hi) / 2;
            if (timeOf(at(mid), base) < t)
                lo = mid + 1;
            else
                hi = mid;
        }
        return lo;
    }

    // The last sample at or before t (zero-order hold), or nullptr.
    const Sample *valueAt(double t, TimeBase base) const {
        const std::size_t i = lowerBound(t, base);
        if (i < size() && timeOf(at(i), base) == t)
            return &at(i);
        return i == 0 ? nullptr : &at(i - 1);
    }

  private:
    std::deque<Sample> data_;
    std::size_t capacity_;
    double history_;
};

// A sample run reduced for drawing: one entry per pixel column holding the first, lowest, highest and last value
// of the samples that fall in it, so a dense run draws its full envelope and a sparse one draws its true points.
struct Column {
    double t0 = 0, t1 = 0; // first and last sample time in the column
    float first = 0, low = 0, high = 0, last = 0;
};

// Decimated view of [t0, t1] into `columns` columns, plus the last sample before t0 (so a line enters from the
// left edge) and min / max of the visible samples. Columns are anchored to absolute time (multiples of the column
// width), not to t0, so a sample stays in the same column while the view scrolls and a dense line keeps its shape
// from frame to frame. Gaps: a column with no samples is skipped; a jump in time
// longer than `gap` seconds between consecutive samples breaks the line (breakBefore marks the column after it).
struct Decimated {
    std::vector<Column> columns;
    std::vector<int> index;        // pixel column of each entry in `columns`
    std::vector<bool> breakBefore; // a gap precedes this entry
    bool hasLead = false;
    Sample lead; // the last sample before t0
    float low = std::numeric_limits<float>::infinity(), high = -std::numeric_limits<float>::infinity();
    std::size_t count = 0; // samples in [t0, t1]
};

inline Decimated decimate(const SeriesBuffer &buffer, double t0, double t1, int columns, TimeBase base,
                          double gap = std::numeric_limits<double>::infinity()) {
    Decimated out;
    if (columns <= 0 || !(t1 > t0) || buffer.empty())
        return out;
    const double width = (t1 - t0) / columns;
    std::size_t i = buffer.lowerBound(t0, base);
    if (i > 0) {
        out.hasLead = true;
        out.lead = buffer.at(i - 1);
    }
    double previous = out.hasLead ? timeOf(out.lead, base) : -std::numeric_limits<double>::infinity();
    const long long firstBin = (long long)std::floor(t0 / width);
    long long current = std::numeric_limits<long long>::min();
    bool pendingBreak = false;
    for (; i < buffer.size(); ++i) {
        const Sample &s = buffer.at(i);
        const double t = timeOf(s, base);
        if (t > t1)
            break;
        if (!std::isfinite(s.value)) { // a NaN sample is an explicit gap
            pendingBreak = true;
            previous = t;
            continue;
        }
        if (t - previous > gap)
            pendingBreak = true;
        previous = t;
        const long long bin = (long long)std::floor(t / width);
        const int column = int(std::clamp<long long>(bin - firstBin, 0, columns - 1));
        if (bin != current || pendingBreak) {
            out.columns.push_back({t, t, s.value, s.value, s.value, s.value});
            out.index.push_back(column);
            out.breakBefore.push_back(pendingBreak);
            pendingBreak = false;
            current = bin;
        } else {
            auto &c = out.columns.back();
            c.t1 = t;
            c.low = std::min(c.low, s.value);
            c.high = std::max(c.high, s.value);
            c.last = s.value;
        }
        out.low = std::min(out.low, s.value);
        out.high = std::max(out.high, s.value);
        ++out.count;
    }
    return out;
}

// Round tick positions covering [low, high] with about `target` steps of 1, 2 or 5 x 10^n.
struct Ticks {
    double step = 1;
    std::vector<double> values;
};

inline double niceStep(double span, int target) {
    if (!(span > 0) || target < 1)
        return 1;
    const double raw = span / target;
    const double magnitude = std::pow(10.0, std::floor(std::log10(raw)));
    const double r = raw / magnitude;
    return (r < 1.5 ? 1 : r < 3.5 ? 2 : r < 7.5 ? 5 : 10) * magnitude;
}

inline Ticks niceTicks(double low, double high, int target) {
    Ticks ticks;
    if (!(high > low))
        return ticks;
    ticks.step = niceStep(high - low, target);
    for (double v = std::ceil(low / ticks.step - 1e-9) * ticks.step; v <= high + ticks.step * 1e-6; v += ticks.step)
        ticks.values.push_back(std::abs(v) < ticks.step * 1e-9 ? 0 : v);
    return ticks;
}

// A value axis for samples in [low, high]: padded by 8 % and widened to include `include` (limits, zero lines);
// a flat signal gets a span around its value so it draws mid-lane rather than on an edge.
inline void fitRange(double &low, double &high) {
    if (!std::isfinite(low) || !std::isfinite(high)) {
        low = -1;
        high = 1;
        return;
    }
    if (high - low < 1e-9) {
        const double pad = std::max(std::abs(low) * .1, 1e-3);
        low -= pad;
        high += pad;
        return;
    }
    const double pad = (high - low) * .08;
    low -= pad;
    high += pad;
}

} // namespace nereus::ros_viewer::plots

// Frame-time statistics for robotics-pool-viewer (--profile log and the F3 readout). Pure data, no GL/ROS.
#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>

namespace robotics::ros_viewer::host {
enum class Phase { Spin, Scene, Main, Cards, Ui, Swap, Sleep, Count };
inline const char *phaseName(Phase p) {
    static const char *names[] = {"spin", "scene", "main", "cards", "ui", "swap", "sleep"};
    return names[int(p)];
}
constexpr int kPhaseCount = int(Phase::Count);

struct FrameSample {
    double frame = 0; // seconds between frame starts (what the eye sees)
    std::array<double, kPhaseCount> phase{};
    double speed = -1; // displayed robot speed over this frame (m/s); <0 when unknown
    // CPU/GL work only: everything except the swap (vsync wait) and the frame-cap sleep.
    double work() const {
        return frame - phase[int(Phase::Swap)] - phase[int(Phase::Sleep)];
    }
};
struct Distribution {
    double mean = 0, p50 = 0, p95 = 0, p99 = 0, max = 0;
};
inline double percentile(std::vector<double> &sorted, double q) {
    if (sorted.empty())
        return 0;
    const double at = q * double(sorted.size() - 1);
    const std::size_t i = std::size_t(at);
    const double f = at - double(i);
    return i + 1 < sorted.size() ? sorted[i] * (1 - f) + sorted[i + 1] * f : sorted[i];
}
inline Distribution distribution(std::vector<double> values) {
    Distribution d;
    if (values.empty())
        return d;
    std::sort(values.begin(), values.end());
    double sum = 0;
    for (double v : values)
        sum += v;
    d.mean = sum / double(values.size());
    d.p50 = percentile(values, .5);
    d.p95 = percentile(values, .95);
    d.p99 = percentile(values, .99);
    d.max = values.back();
    return d;
}

class FrameProfiler {
  public:
    explicit FrameProfiler(std::size_t rolling = 240) : rolling_(rolling) {}
    void add(Phase p, double seconds) {
        current_.phase[std::size_t(p)] += seconds;
    }
    // Displayed robot position this frame (for the motion-smoothness statistic).
    void setPosition(double x, double y, double z) {
        pos_ = {x, y, z};
        havePos_ = true;
    }
    // Close the frame that started at the previous end (or begin()).
    void endFrame(std::chrono::steady_clock::time_point now) {
        if (started_) {
            current_.frame = std::chrono::duration<double>(now - last_).count();
            if (havePos_ && hadPos_ && current_.frame > 0) {
                const double dx = pos_[0] - prevPos_[0], dy = pos_[1] - prevPos_[1], dz = pos_[2] - prevPos_[2];
                current_.speed = std::sqrt(dx * dx + dy * dy + dz * dz) / current_.frame;
            }
            recent_.push_back(current_);
            if (recent_.size() > rolling_)
                recent_.pop_front();
            interval_.push_back(current_);
        }
        started_ = true;
        hadPos_ = havePos_;
        prevPos_ = pos_;
        last_ = now;
        current_ = {};
    }
    const std::array<double, kPhaseCount> &pendingPhases() const {
        return current_.phase;
    }
    const std::deque<FrameSample> &recent() const {
        return recent_;
    }
    // Samples since the last takeInterval().
    std::vector<FrameSample> takeInterval() {
        std::vector<FrameSample> out;
        out.swap(interval_);
        return out;
    }
    std::size_t pending() const {
        return interval_.size();
    }

  private:
    std::size_t rolling_;
    bool started_ = false, havePos_ = false, hadPos_ = false;
    std::array<double, 3> pos_{}, prevPos_{};
    std::chrono::steady_clock::time_point last_{};
    FrameSample current_;
    std::deque<FrameSample> recent_;
    std::vector<FrameSample> interval_;
};

inline Distribution frameDistribution(const std::vector<FrameSample> &s, bool work = false) {
    std::vector<double> v;
    v.reserve(s.size());
    for (const auto &x : s)
        v.push_back((work ? x.work() : x.frame) * 1000);
    return distribution(std::move(v));
}
inline Distribution phaseDistribution(const std::vector<FrameSample> &s, Phase p) {
    std::vector<double> v;
    v.reserve(s.size());
    for (const auto &x : s)
        v.push_back(x.phase[std::size_t(p)] * 1000);
    return distribution(std::move(v));
}
// Two-line report: frame interval distribution, then per-phase mean/p99/max (milliseconds).
inline std::string profileReport(const std::vector<FrameSample> &s, double seconds) {
    if (s.empty())
        return "profile: no frames";
    char line[512];
    const auto f = frameDistribution(s), w = frameDistribution(s, true);
    std::snprintf(line, sizeof(line),
                  "profile: %zu frames %.1f fps | frame ms mean %.2f p50 %.2f p95 %.2f p99 %.2f max %.2f"
                  " | work ms mean %.2f p99 %.2f max %.2f",
                  s.size(), seconds > 0 ? double(s.size()) / seconds : 0., f.mean, f.p50, f.p95, f.p99, f.max, w.mean,
                  w.p99, w.max);
    std::string out = line;
    out += "\n  phases ms mean/p99/max:";
    for (int i = 0; i < kPhaseCount; ++i) {
        const auto d = phaseDistribution(s, Phase(i));
        std::snprintf(line, sizeof(line), "  %s %.2f/%.2f/%.2f", phaseName(Phase(i)), d.mean, d.p99, d.max);
        out += line;
    }
    // Motion smoothness: spread of the displayed speed of a moving robot (steady motion = small spread).
    std::vector<double> speeds;
    for (const auto &x : s)
        if (x.speed >= 0)
            speeds.push_back(x.speed);
    if (!speeds.empty()) {
        double mean = 0, var = 0;
        for (double v : speeds)
            mean += v;
        mean /= double(speeds.size());
        for (double v : speeds)
            var += (v - mean) * (v - mean);
        const double sd = std::sqrt(var / double(speeds.size()));
        if (mean > 1e-3) {
            std::snprintf(line, sizeof(line), "\n  displayed speed m/s mean %.3f sd %.3f (cv %.1f%%) min %.3f max %.3f",
                          mean, sd, 100 * sd / mean, *std::min_element(speeds.begin(), speeds.end()),
                          *std::max_element(speeds.begin(), speeds.end()));
            out += line;
        }
    }
    return out;
}
} // namespace robotics::ros_viewer::host

// Smooth display time for TF sampling. Poses arrive in bursts (a 100 Hz truth stream polled at 60 Hz sees
// 1-2 new stamps per frame), so sampling at the latest stamp moves the robot in uneven steps. The display
// clock maps wall time onto the stamp domain with a low-pass estimate of (stamp - wall), so the sampled time
// advances evenly every frame, `delay` seconds behind the newest data, and never past the newest stamp.
#pragma once
#include <algorithm>
#include <cmath>
#include <limits>

namespace robotics::ros_viewer::host {
class DisplayClock {
  public:
    // A new newest stamp (seconds) was seen at `wall` (monotonic seconds).
    void observe(double stamp, double wall) {
        const double sample = stamp - wall;
        if (!valid_ || std::abs(sample - offset_) > jump || stamp < latest_ - jump) {
            offset_ = sample; // first sample, /clock jump or simulator reset: re-anchor
            last_ = -std::numeric_limits<double>::infinity();
            valid_ = true;
        } else
            offset_ += alpha * (sample - offset_);
        latest_ = stamp;
    }
    bool valid() const {
        return valid_;
    }
    double latest() const {
        return latest_;
    }
    // Stamp to sample at `wall`, `delay` seconds behind the estimated newest data, clamped to the newest stamp
    // so a lookup never has to extrapolate. Non-decreasing between re-anchors.
    double at(double wall, double delay) {
        if (!valid_)
            return 0;
        last_ = std::max(last_, wall + offset_);
        return std::min(last_ - delay, latest_);
    }
    double alpha = .05, jump = .5;

  private:
    bool valid_ = false;
    double offset_ = 0, latest_ = 0, last_ = -std::numeric_limits<double>::infinity();
};
} // namespace robotics::ros_viewer::host

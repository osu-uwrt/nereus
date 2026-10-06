// Smooth display time for TF sampling (one clock per pose stream).
//
// Poses arrive late by a variable amount (DDS delivery, a loaded machine, the receiver's scheduling).
// Sampling "latest" moves the robot in uneven steps; sampling a fixed small delay behind the newest stamp
// freezes and jumps whenever a sample is later than the delay. This clock instead:
//  - takes observations (stamp, arrival wall time) from a receive thread, so arrival times are exact;
//  - models the stream as stamp = anchor + rate * (wall - anchor wall) + floor, where the floor is the
//    arrival-latency floor (earliest arrivals) and the jitter above it is tracked as a decaying envelope;
//  - shows data `delay` behind the floor, where delay covers how far the newest stamp has recently lagged
//    that timeline just before a new one arrived (sample period + late or overtaken samples; decaying
//    envelope, >= the caller's minimum), so interpolation almost always has a newer sample to reach;
//  - advances the displayed time at the stream rate and corrects towards the target proportionally
//    (time constant `settle`, at most +-`slew` of the rate), so it never pauses or jumps while data flows;
//  - re-anchors on stream resets, pauses/resumes and real-time-factor changes (large errors).
// Thread safe: observe() and at() may be called from different threads.
#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <utility>

namespace nereus::ros_viewer::host {
// See the file header. Times are in seconds: stamps in the stream's clock, wall in monotonic time.
class DisplayClock {
  public:
    // A stamp (seconds) arrived at `wall` (monotonic seconds).
    void observe(double stamp, double wall) {
        std::lock_guard<std::mutex> lock(mutex_);

        // First sample, a stamp jump backwards or a much later arrival than usual: start over.
        if (!valid_ || stamp < latest_ - reanchor || offset(stamp, wall) < floor_ - reanchor) {
            anchor(stamp, wall);
            return;
        }
        if (stamp <= latest_)
            return; // duplicate or reordered sample: no new timing information

        const double dt = std::max(0., wall - lastWall_);
        lastWall_ = wall;
        // How far the newest data lagged the display timeline just before this arrival.
        const double lag = model(wall) + floor_ - latest_;
        lag_ = std::max(lag, lag_ * std::exp(-dt / jitterDecay));
        latest_ = stamp;

        // Stream rate (simulated seconds per wall second) over spans of `rateSpan` seconds.
        if (wall - rateWall_ >= rateSpan) {
            const double measured = (stamp - rateStamp_) / (wall - rateWall_);
            rateStamp_ = stamp;
            rateWall_ = wall;
            if (measured > 0 && std::isfinite(measured)) {
                if (std::abs(measured - rate_) > .2 * rate_) { // real-time factor changed
                    rate_ = measured;
                    anchor(stamp, wall);
                    return;
                }
                setRate(rate_ + .2 * (measured - rate_), wall);
            }
        }

        // Latency floor: the earliest arrival seen, leaking slowly towards later ones.
        const double x = offset(stamp, wall); // larger = earlier arrival
        floor_ = std::max(x, floor_ - floorLeak * dt);
    }

    bool valid() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return valid_;
    }
    double latest() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return latest_;
    }

    // Delay currently applied behind the latency floor (seconds), for diagnostics.
    double delay(double minimum) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return targetDelay(minimum);
    }

    // Stamp to sample at `wall`: at least `minimum` behind the newest data (more while arrivals are
    // jittery), never past the newest stamp, non-decreasing between re-anchors.
    double at(double wall, double minimum) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!valid_)
            return 0;

        // Where the display should be now, and where it gets by just advancing at the stream rate.
        const double target = model(wall) + floor_ - targetDelay(minimum);
        const double advance = rate_ * std::max(0., wall - outWall_);

        // Far off (first call, pause, reset): jump. Otherwise steer towards the target within the slew limit,
        // never past the newest stamp and never backwards.
        if (!haveOut_ || std::abs(target - (out_ + advance)) > reanchor) {
            reanchors_ += haveOut_;
            out_ = std::min(target, latest_);
            haveOut_ = true;
        } else if (advance > 0) {
            const double error = target - (out_ + advance);
            const double gain = std::min(1., (wall - outWall_) / settle);
            const double correction = std::clamp(error * gain, -slew * advance, slew * advance);
            const double wanted = out_ + advance + correction;
            holds_ += wanted > latest_;
            out_ = std::max(out_, std::min(wanted, latest_));
        }
        outWall_ = wall;
        return out_;
    }

    // Diagnostics since the last take: display re-anchors (visible jumps) and frames held at the newest stamp.
    std::pair<int, int> takeCounts() {
        std::lock_guard<std::mutex> lock(mutex_);
        const std::pair<int, int> counts{reanchors_, holds_};
        reanchors_ = holds_ = 0;
        return counts;
    }

    // Tuning (public so tests and the host can adjust them).
    double reanchor = .3;     // seconds of error treated as a reset/pause, not jitter
    double slew = .1;         // max relative rate correction
    double settle = 1.;       // seconds to converge on the target
    double floorLeak = .005;  // seconds per second the latency floor may drift towards later arrivals
    double jitterDecay = 10.; // seconds for the lag envelope to decay (longer than typical burst spacing)
    double margin = .008;     // added to the lag envelope
    double maxDelay = .25;    // upper bound on the adaptive delay (unless the caller's minimum is larger)
    double rateSpan = 2.;     // seconds per rate measurement

  private:
    // Stream stamp predicted at `wall` (without the latency floor).
    double model(double wall) const {
        return anchorStamp_ + rate_ * (wall - anchorWall_);
    }

    // How far a stamp is ahead of the model at its arrival time.
    double offset(double stamp, double wall) const {
        return stamp - model(wall);
    }

    void setRate(double rate, double wall) { // keep the model continuous at `wall`
        anchorStamp_ = model(wall);
        anchorWall_ = wall;
        rate_ = rate;
    }

    // Lag envelope plus margin, clamped to [minimum, max(minimum, maxDelay)].
    double targetDelay(double minimum) const {
        return std::clamp(lag_ + margin, minimum, std::max(minimum, maxDelay));
    }

    // Restart the model at this sample, forgetting the floor and lag envelope.
    void anchor(double stamp, double wall) {
        valid_ = true;
        haveOut_ = false;
        anchorStamp_ = rateStamp_ = latest_ = stamp;
        anchorWall_ = rateWall_ = lastWall_ = wall;
        floor_ = lag_ = 0;
    }

    mutable std::mutex mutex_;
    int reanchors_ = 0, holds_ = 0;
    bool valid_ = false, haveOut_ = false;

    // Timing model and last observation.
    double rate_ = 1, anchorStamp_ = 0, anchorWall_ = 0, floor_ = 0, lag_ = 0;
    double latest_ = -std::numeric_limits<double>::infinity(), lastWall_ = 0, rateWall_ = 0, rateStamp_ = 0;

    // Last displayed stamp and the wall time it was returned at.
    double out_ = 0, outWall_ = 0;
};
} // namespace nereus::ros_viewer::host

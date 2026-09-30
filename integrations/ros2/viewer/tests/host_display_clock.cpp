#include "display_clock.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <random>
using namespace nereus::ros_viewer::host;

namespace {
// Poses stamped every 10 ms arrive with wall-clock jitter; a 60 Hz renderer samples the display clock.
struct Run {
    std::vector<double> latestSamples, clockSamples;
    bool ahead = false;
};
Run simulate(double delay) {
    DisplayClock clock;
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> jitter(0, .004);
    Run run;
    std::vector<std::pair<double, double>> arrivals; // wall, stamp
    for (int i = 0; i < 1000; ++i) {
        const double stamp = 100 + i * .01;
        arrivals.emplace_back(i * .01 + jitter(rng), stamp);
    }
    std::size_t next = 0;
    double latest = 0;
    for (double wall = 0; wall < 9.5; wall += 1. / 60) {
        while (next < arrivals.size() && arrivals[next].first <= wall) {
            clock.observe(arrivals[next].second, arrivals[next].first);
            latest = arrivals[next].second;
            ++next;
        }
        if (!clock.valid())
            continue;
        const double t = clock.at(wall, delay);
        if (t > latest + 1e-9)
            run.ahead = true;
        run.clockSamples.push_back(t);
        run.latestSamples.push_back(latest - delay);
    }
    return run;
}
double stepSpread(const std::vector<double> &v) {
    double mean = 0, var = 0;
    std::vector<double> steps;
    for (std::size_t i = 1; i < v.size(); ++i)
        steps.push_back(v[i] - v[i - 1]);
    for (double s : steps)
        mean += s;
    mean /= double(steps.size());
    for (double s : steps)
        var += (s - mean) * (s - mean);
    return std::sqrt(var / double(steps.size())) / mean;
}
} // namespace

TEST(HostDisplayClock, SmootherThanLatestStampAndNeverAhead) {
    const auto run = simulate(.02);
    EXPECT_FALSE(run.ahead); // never samples beyond the newest stamp (no extrapolation)
    EXPECT_GT(stepSpread(run.latestSamples), .2);
    EXPECT_LT(stepSpread(run.clockSamples), stepSpread(run.latestSamples) / 2);
    EXPECT_LT(stepSpread(run.clockSamples), .1);
    for (std::size_t i = 1; i < run.clockSamples.size(); ++i)
        ASSERT_GE(run.clockSamples[i], run.clockSamples[i - 1]);
}

TEST(HostDisplayClock, ReanchorsOnClockJump) {
    DisplayClock clock;
    for (int i = 0; i < 50; ++i)
        clock.observe(50 + i * .01, i * .01);
    EXPECT_NEAR(clock.at(.5, .02), 50.5 - .02, .01);
    clock.observe(5, .51); // simulator reset: stamps jump back
    EXPECT_LE(clock.at(.52, .02), 5.0);
    EXPECT_NEAR(clock.at(.52, .02), 5 + .01 - .02, .02);
}

TEST(HostDisplayClock, DelayLargerThanPeriodKeepsInterpolationInsideBuffer) {
    // A 30 Hz stream: with delay > period the sampled time always has a newer sample after it.
    DisplayClock clock;
    for (int i = 0; i < 100; ++i)
        clock.observe(10 + i / 30., i / 30.);
    const double newest = clock.latest();
    for (double wall = 3.0; wall < 3.3; wall += 1. / 60)
        EXPECT_LE(clock.at(wall, .06), newest);
}

namespace {
// Stamps every `period` s (at `rate` simulated s per wall s) arriving with loaded-machine delivery jitter
// (uniform 0-15 ms plus a 40 ms burst every ~2 s); a 60 Hz renderer samples the clock.
struct Loaded {
    double cv = 0, maxStepRatio = 0;
    int held = 0, frames = 0;
    bool ahead = false, backwards = false;
};
Loaded loaded(double period, double rate = 1., double seconds = 20) {
    DisplayClock clock;
    std::mt19937 rng(11);
    std::uniform_real_distribution<double> jitter(0, .015);
    std::vector<std::pair<double, double>> arrivals;
    for (int i = 0; i * period < seconds * rate; ++i) {
        const double wall = i * period / rate;
        const double late = jitter(rng) + (std::fmod(wall, 2.) < period / rate ? .04 : 0.);
        arrivals.emplace_back(wall + late, 500 + i * period);
    }
    std::sort(arrivals.begin(), arrivals.end());
    Loaded result;
    std::size_t next = 0;
    double latest = 0, previous = -1;
    std::vector<double> steps;
    for (double wall = 0; wall < seconds - .1; wall += 1. / 60) {
        while (next < arrivals.size() && arrivals[next].first <= wall) {
            clock.observe(arrivals[next].second, arrivals[next].first);
            latest = std::max(latest, arrivals[next].second);
            ++next;
        }
        if (!clock.valid())
            continue;
        const double t = clock.at(wall, .02);
        result.ahead |= t > latest + 1e-9;
        if (wall > 3) { // after the jitter envelope has formed
            ++result.frames;
            result.held += std::abs(t - latest) < 1e-9;
            if (previous >= 0) {
                result.backwards |= t < previous;
                steps.push_back(t - previous);
            }
        }
        previous = t;
    }
    double mean = 0, var = 0;
    for (double s : steps)
        mean += s;
    mean /= double(steps.size());
    for (double s : steps) {
        var += (s - mean) * (s - mean);
        result.maxStepRatio = std::max(result.maxStepRatio, s / mean);
    }
    result.cv = std::sqrt(var / double(steps.size())) / mean;
    return result;
}
} // namespace

TEST(HostDisplayClock, LoadedDeliveryStaysEvenWithoutHolding) {
    const auto truth = loaded(.01); // 100 Hz truth
    EXPECT_FALSE(truth.ahead);
    EXPECT_FALSE(truth.backwards);
    EXPECT_LT(truth.cv, .05) << "displayed time must advance evenly frame to frame";
    EXPECT_LT(truth.maxStepRatio, 1.2);
    EXPECT_LT(truth.held, truth.frames / 100) << "display must almost never wait at the newest stamp";
    const auto estimate = loaded(.034); // 30 Hz EKF
    EXPECT_FALSE(estimate.ahead);
    EXPECT_LT(estimate.cv, .05) << "held " << estimate.held << "/" << estimate.frames << " max step ratio "
                                << estimate.maxStepRatio;
    EXPECT_LT(estimate.held, estimate.frames / 100);
}

TEST(HostDisplayClock, FollowsFasterSimulation) {
    const auto fast = loaded(.01, 2.); // real_time_factor 2
    EXPECT_FALSE(fast.ahead);
    EXPECT_LT(fast.cv, .05);
    EXPECT_LT(fast.held, fast.frames / 50);
}

TEST(HostDisplayClock, HoldsWhilePausedAndReanchorsOnResume) {
    DisplayClock clock;
    for (int i = 0; i <= 300; ++i)
        clock.observe(10 + i * .01, i * .01); // 3 s at 100 Hz
    const double paused = clock.at(3.0, .02);
    for (double wall = 3.0; wall < 5.0; wall += 1. / 60) // no data: simulation paused
        EXPECT_LE(clock.at(wall, .02), 13.0);
    clock.observe(13.01, 5.0); // resumes 2 s (wall) later
    for (int i = 2; i < 30; ++i)
        clock.observe(13 + i * .01, 5.0 + (i - 1) * .01);
    const double resumed = clock.at(5.3, .02);
    EXPECT_GE(resumed, paused);
    EXPECT_NEAR(resumed, 13.29 - .02, .04);
}

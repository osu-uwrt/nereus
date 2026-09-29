#include <gtest/gtest.h>
#include "display_clock.hpp"
#include <cmath>
#include <random>
using namespace robotics::ros_viewer::host;

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

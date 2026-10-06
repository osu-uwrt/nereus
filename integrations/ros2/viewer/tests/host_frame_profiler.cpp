// The viewer's frame profiler: per-phase frame timings, percentile summaries and the text report.
#include "frame_profiler.hpp"
#include <gtest/gtest.h>
using namespace nereus::ros_viewer::host;
using Clock = std::chrono::steady_clock;

// distribution() gives the mean, interpolated percentiles and max of a sample set (0 for an empty one).
TEST(HostFrameProfiler, PercentilesAndSpikes) {
    std::vector<double> v;
    for (int i = 1; i <= 100; ++i)
        v.push_back(i);
    const auto d = distribution(v);
    EXPECT_NEAR(d.mean, 50.5, 1e-9);
    EXPECT_NEAR(d.p50, 50.5, 1e-9);
    EXPECT_NEAR(d.p95, 95.05, 1e-9);
    EXPECT_NEAR(d.p99, 99.01, 1e-9);
    EXPECT_EQ(d.max, 100);
    EXPECT_EQ(distribution({}).max, 0);
}

// Each endFrame closes a sample with its phase times, frame period and camera speed; recent() keeps a rolling
// window while takeInterval() drains everything since the last report.
TEST(HostFrameProfiler, RecordsFramePhasesAndSpeed) {
    FrameProfiler profiler(4);
    const auto t0 = Clock::time_point{} + std::chrono::seconds(10);
    profiler.setPosition(0, 0, 0);
    profiler.endFrame(t0); // starts the first frame

    // Six 10 ms frames, each 4 ms main + 1 ms swap, moving 5 mm per frame.
    for (int i = 1; i <= 6; ++i) {
        profiler.add(Phase::Main, .004);
        profiler.add(Phase::Swap, .001);
        profiler.setPosition(.005 * i, 0, 0);
        profiler.endFrame(t0 + std::chrono::microseconds(10000 * i)); // 10 ms frames, 0.5 m/s
    }

    ASSERT_EQ(profiler.recent().size(), 4u); // rolling window
    const auto &s = profiler.recent().back();
    EXPECT_NEAR(s.frame, .01, 1e-6);
    EXPECT_NEAR(s.phase[int(Phase::Main)], .004, 1e-9);
    EXPECT_NEAR(s.work(), .009, 1e-6); // frame minus the swap (vsync wait)
    EXPECT_NEAR(s.speed, .5, 1e-3);

    // The interval keeps all six samples and the report names the percentiles, phases and speed.
    const auto samples = profiler.takeInterval();
    EXPECT_EQ(samples.size(), 6u);
    const auto report = profileReport(samples, .06);
    EXPECT_NE(report.find("p99"), std::string::npos);
    EXPECT_NE(report.find("main 4.00"), std::string::npos);
    EXPECT_NE(report.find("displayed speed"), std::string::npos);
    EXPECT_EQ(profiler.pending(), 0u);
}

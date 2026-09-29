#include <gtest/gtest.h>
#include "frame_profiler.hpp"
using namespace robotics::ros_viewer::host;
using Clock = std::chrono::steady_clock;

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

TEST(HostFrameProfiler, RecordsFramePhasesAndSpeed) {
    FrameProfiler profiler(4);
    const auto t0 = Clock::time_point{} + std::chrono::seconds(10);
    profiler.setPosition(0, 0, 0);
    profiler.endFrame(t0); // starts the first frame
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
    EXPECT_NEAR(s.work(), .009, 1e-6);
    EXPECT_NEAR(s.speed, .5, 1e-3);
    const auto samples = profiler.takeInterval();
    EXPECT_EQ(samples.size(), 6u);
    const auto report = profileReport(samples, .06);
    EXPECT_NE(report.find("p99"), std::string::npos);
    EXPECT_NE(report.find("main 4.00"), std::string::npos);
    EXPECT_NE(report.find("displayed speed"), std::string::npos);
    EXPECT_EQ(profiler.pending(), 0u);
}

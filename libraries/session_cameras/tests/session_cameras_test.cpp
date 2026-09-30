#include <robotics/session_cameras/session_cameras.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>

namespace sc = robotics::session_cameras;
namespace rs = robotics::session;
namespace sp = robotics::spatial;
using namespace std::chrono_literals;

namespace {
const rs::ResolvedScenario &talos() {
    static const auto resolved = rs::loadResolvedScenario(NEREUS_RESOLVED_TALOS);
    return resolved;
}

// Products collected from the delivery callback.
struct Sink {
    std::mutex mutex;
    std::condition_variable condition;
    std::vector<sc::Products> products;
    void operator()(sc::Products &&p) {
        std::lock_guard<std::mutex> lock(mutex);
        products.push_back(std::move(p));
        condition.notify_all();
    }
    bool waitFor(std::size_t count, std::chrono::seconds timeout = 60s) {
        std::unique_lock<std::mutex> lock(mutex);
        return condition.wait_for(lock, timeout, [&] { return products.size() >= count; });
    }
    std::size_t size() {
        std::lock_guard<std::mutex> lock(mutex);
        return products.size();
    }
};

sp::Pose poolPose() {
    // In front of the gate at mid-depth, looking down the pool (+X).
    sp::Pose pose;
    pose.translation = {2.0, 0.0, -1.0};
    return pose;
}

std::unique_ptr<sc::SessionCameras> make(sc::Options options = {}) {
    try {
        return std::make_unique<sc::SessionCameras>(talos(), options);
    } catch (const std::exception &error) {
        if (std::string(error.what()).find("EGL") != std::string::npos ||
            std::string(error.what()).find("GL") != std::string::npos)
            return nullptr;
        throw;
    }
}
} // namespace

TEST(SessionCamerasSeeds, Sha256AndDerivation) {
    const auto digest = sc::sha256("abc");
    EXPECT_EQ(digest[0], 0xba);
    EXPECT_EQ(digest[1], 0x78);
    EXPECT_EQ(digest[31], 0xad);
    // Two-block message (56 bytes forces a second padding block).
    const auto two = sc::sha256("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq");
    EXPECT_EQ(two[0], 0x24);
    EXPECT_EQ(two[31], 0xc1); // 248d6a61...19db06c1
    // Recorded reference seeds.
    EXPECT_EQ(sc::deriveSeed(0, "front", "left"), 3562221999u);
    EXPECT_EQ(sc::deriveSeed(12345, "front", "right"), 737816300u);
    EXPECT_EQ(sc::deriveSeed(18446744073709551615ull, "a", "left"), 1862611008u);
}

TEST(SessionCameras, ConfigurationFromPack) {
    auto cameras = make();
    if (!cameras)
        GTEST_SKIP() << "no EGL";
    const auto ids = cameras->cameraIds();
    ASSERT_EQ(ids.size(), 2u);
    for (const auto &id : ids) {
        EXPECT_TRUE(cameras->hasOutput(id, sc::Output::RgbLeft));
        EXPECT_TRUE(cameras->hasOutput(id, sc::Output::DepthLeft));
        EXPECT_FALSE(cameras->hasOutput(id, sc::Output::RgbRight));
        EXPECT_NEAR(cameras->periodSeconds(id), 1. / 15, 1e-6);
        const auto left = cameras->info(id, "left"), right = cameras->info(id, "right");
        EXPECT_EQ(left.width, 1920);
        EXPECT_EQ(left.p[3], 0.0);
        EXPECT_LT(right.p[3], 0.0); // Tx = -fx * baseline
        EXPECT_DOUBLE_EQ(left.k[0], left.p[0]);
    }
    EXPECT_THROW(cameras->setDemand(ids[0], sc::Output::RgbRight, true), std::invalid_argument);
    EXPECT_THROW(cameras->setDemand("nope", sc::Output::RgbLeft, true), std::invalid_argument);
    EXPECT_TRUE(cameras->describe().at("scene").contains("cutouts"));
}

TEST(SessionCameras, OnlyDemandedOutputsAreProduced) {
    auto cameras = make();
    if (!cameras)
        GTEST_SKIP() << "no EGL";
    const auto id = cameras->cameraIds()[0];
    Sink sink;
    cameras->start([&](sc::Products &&p) { sink(std::move(p)); });

    // No consumer: the schedule advances but nothing renders.
    cameras->request(0, 100, poolPose());
    cameras->close();
    auto stats = cameras->stats().at(id);
    EXPECT_EQ(stats.requested, 1u);
    EXPECT_EQ(stats.skipped_no_demand, 1u);
    EXPECT_EQ(stats.captured, 0u);
    EXPECT_EQ(sink.size(), 0u);

    auto again = make();
    Sink second;
    again->setDemand(id, sc::Output::RgbLeft, true, "viewer");
    again->start([&](sc::Products &&p) { second(std::move(p)); });
    again->request(0, 111, poolPose());
    ASSERT_TRUE(second.waitFor(1));
    {
        const auto &p = second.products[0];
        EXPECT_EQ(p.camera, id);
        EXPECT_EQ(p.ros_stamp_ns, 111);
        ASSERT_TRUE(p.left.has_value());
        EXPECT_FALSE(p.left->rgb.empty());
        EXPECT_TRUE(p.left->depth.empty()); // depth was not requested
        EXPECT_TRUE(p.left->jpeg.empty());  // no JPEG quality configured
        EXPECT_FALSE(p.right.has_value());
        EXPECT_EQ(p.left->rgb.size(), 1920u * 1200u * 3u);
        EXPECT_GT(p.render_ms, 0);
    }
    // A second consumer adds depth; dropping the first consumer leaves the second's interest.
    again->setDemand(id, sc::Output::DepthLeft, true, "planner");
    again->setDemand(id, sc::Output::RgbLeft, false, "viewer");
    again->request(200'000'000, 222, poolPose());
    ASSERT_TRUE(second.waitFor(2));
    const auto &p = second.products[1];
    EXPECT_TRUE(p.left->rgb.empty());
    ASSERT_EQ(p.left->depth.size(), 1920u * 1200u);
    std::size_t valid = 0;
    for (const float z : p.left->depth)
        valid += std::isfinite(z) && z > 0.05f;
    EXPECT_GT(valid, p.left->depth.size() / 10) << "the pool should fill much of the depth image";
    again->close();
    EXPECT_EQ(again->stats().at(id).delivered, 2u);
}

TEST(SessionCameras, ScheduleFollowsSensorPeriod) {
    sc::Options options;
    options.always = true;
    auto cameras = make(options);
    if (!cameras)
        GTEST_SKIP() << "no EGL";
    Sink sink;
    // Not started: jobs only queue, so the schedule is observable through stats.
    for (std::int64_t t = 0; t <= 210'000'000; t += 10'000'000)
        cameras->request(t, t, poolPose());
    for (const auto &id : cameras->cameraIds()) {
        const auto s = cameras->stats().at(id);
        EXPECT_EQ(s.requested,
                  4u); // due at 0, 66.7, 133.3 and 200 ms; the 200 ms slot is the first request >= 200000001 ns
        EXPECT_EQ(s.dropped_pending, 3u); // capacity 1, drop_oldest
    }
}

TEST(SessionCameras, InvalidateDiscardsPendingAndStaleResults) {
    sc::Options options;
    options.always = true;
    auto cameras = make(options);
    if (!cameras)
        GTEST_SKIP() << "no EGL";
    Sink sink;
    cameras->request(0, 1, poolPose()); // queued; workers not started yet
    cameras->invalidate();
    for (const auto &id : cameras->cameraIds())
        EXPECT_EQ(cameras->stats().at(id).discarded_stale, 1u);
    cameras->start([&](sc::Products &&p) { sink(std::move(p)); });
    cameras->request(100'000'000, 2, poolPose());
    ASSERT_TRUE(sink.waitFor(2));
    for (const auto &p : sink.products) {
        EXPECT_EQ(p.ros_stamp_ns, 2); // only post-invalidation work is delivered
        EXPECT_EQ(p.revision, 1u);
    }
    // In-flight work at invalidation is dropped at delivery (or delivered before it, never after).
    const auto before = sink.size();
    cameras->request(300'000'000, 3, poolPose());
    cameras->invalidate();
    cameras->close();
    std::size_t stale = 0;
    for (const auto &[id, s] : cameras->stats())
        stale += s.discarded_stale;
    EXPECT_GE(stale + sink.size() - before, 2u);
    for (std::size_t i = before; i < sink.products.size(); ++i)
        EXPECT_EQ(sink.products[i].revision, 1u);
}

TEST(SessionCameras, FullResetRestoresNoiseStreams) {
    sc::Options options;
    options.always = true;
    auto cameras = make(options);
    if (!cameras)
        GTEST_SKIP() << "no EGL";
    const auto id = cameras->cameraIds()[0];
    Sink sink;
    cameras->start([&](sc::Products &&p) { sink(std::move(p)); });
    const auto grab = [&](std::int64_t t) {
        const auto count = sink.size();
        cameras->request(t, t, poolPose());
        EXPECT_TRUE(sink.waitFor(count + cameras->cameraIds().size()));
        for (const auto &p : sink.products)
            if (p.camera == id && p.snapshot_time_ns == t && &p >= &sink.products[count])
                return p;
        ADD_FAILURE() << "no product for " << id;
        return sc::Products{};
    };
    const auto first = grab(0);
    const auto second = grab(100'000'000); // advances the stream
    cameras->invalidate(7);                // scenario seed: restores every eye's stream and the schedule
    const auto third = grab(0);
    ASSERT_EQ(first.left->depth.size(), third.left->depth.size());
    std::size_t different = 0, same = 0;
    for (std::size_t i = 0; i < first.left->depth.size(); ++i) {
        const float a = first.left->depth[i], c = third.left->depth[i];
        if ((std::isnan(a) && std::isnan(c)) || a == c)
            ++same;
        else
            ++different;
    }
    EXPECT_EQ(different, 0u) << "same seed, same pose, same time must reproduce the frame";
    (void)second;
    (void)id;
    cameras->close();
}

TEST(SessionCameras, JpegAndCostReport) {
    sc::Options options;
    options.always = true;
    auto probe = make(options);
    if (!probe)
        GTEST_SKIP() << "no EGL";
    const auto id = probe->cameraIds()[0];
    probe.reset();
    options.jpeg_quality[id] = 90;
    auto cameras = make(options);
    Sink sink;
    cameras->start([&](sc::Products &&p) { sink(std::move(p)); });
    constexpr int frames = 6;
    for (int i = 0; i < frames; ++i) {
        cameras->request(std::int64_t(i) * 100'000'000, i, poolPose());
        ASSERT_TRUE(sink.waitFor(std::size_t(2 * (i + 1)))); // one product per camera
    }
    cameras->close();
    ASSERT_GE(sink.size(), std::size_t(frames));
    const auto &p = sink.products.back();
    ASSERT_GT(p.left->jpeg.size(), 1000u);
    EXPECT_EQ(p.left->jpeg[0], 0xff);
    EXPECT_EQ(p.left->jpeg[1], 0xd8);
    double render = 0, process = 0;
    for (const auto &q : sink.products) {
        render += q.render_ms;
        process += q.process_ms;
    }
    std::cout << "[cost] " << sink.size() << " captures (rgb+depth+jpeg, 1920x1200): render "
              << render / double(sink.size()) << " ms, process " << process / double(sink.size()) << " ms each\n";
}

TEST(SessionCameras, RequestNeverWaitsForRendering) {
    sc::Options options;
    options.always = true;
    auto cameras = make(options);
    if (!cameras)
        GTEST_SKIP() << "no EGL";
    Sink sink;
    cameras->start([&](sc::Products &&p) { sink(std::move(p)); });
    double worst = 0;
    for (int i = 0; i < 40; ++i) {
        const auto start = std::chrono::steady_clock::now();
        cameras->request(std::int64_t(i) * 70'000'000, i, poolPose());
        worst = std::max(worst,
                         std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
        std::this_thread::sleep_for(5ms);
    }
    std::cout << "[cost] worst request() call: " << worst << " ms\n";
    EXPECT_LT(worst, 20.0);
    cameras->close();
}

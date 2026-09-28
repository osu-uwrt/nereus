#include <atomic>
#include <gtest/gtest.h>
#include <robotics/viewer/session.hpp>
#include <robotics/visualization/live_source.hpp>
#include <thread>

using namespace robotics::visualization;

TEST(LiveSource, BoundsQueuesAndHistoryWithoutInvalidatingSnapshots) {
    LivePoseSource source({"live", "clock", "world", "body", "pose", 2, 3});
    EXPECT_FALSE(source.snapshot().data);
    for (Time t = 0; t < 5; ++t)
        source.publish({0, t, {}});
    const auto retained = source.snapshot();
    ASSERT_TRUE(retained.data);
    EXPECT_EQ(retained.delivery.dropped_queue, 3U);
    EXPECT_EQ(retained.data->streams.at("pose").front().time_ns, 3);
    source.publish({0, 5, {}});
    source.publish({0, 6, {}});
    const auto next = source.snapshot();
    EXPECT_EQ(next.delivery.trimmed_history, 1U);
    EXPECT_EQ(next.data->streams.at("pose").size(), 3U);
    EXPECT_EQ(next.data->streams.at("pose").front().time_ns, 4);
    EXPECT_EQ(retained.data->streams.at("pose").size(), 2U);
    EXPECT_EQ(retained.time_ns, 4);
}

TEST(LiveSource, EpochsRejectStaleDataAndReconnectAtLatestPose) {
    LivePoseSource source({"live", "clock"});
    source.publish({0, 100, {}});
    const auto first = source.snapshot();
    source.publish({1, 0, {{1, 2, 3}, Eigen::Quaterniond::Identity()}});
    source.publish({0, 101, {}});
    source.publish({1, 0, {}});
    const auto reset = source.snapshot();
    EXPECT_GT(reset.generation, first.generation);
    EXPECT_EQ(reset.delivery.rejected_stale, 2U);
    ASSERT_EQ(reset.data->streams.at("pose").size(), 1U);
    EXPECT_EQ(reset.time_ns, 0);
    source.disconnect();
    source.publish({1, 10, {}});
    EXPECT_FALSE(source.snapshot().data);
    source.reconnect();
    const auto reconnected = source.snapshot();
    EXPECT_GT(reconnected.generation, reset.generation);
    ASSERT_TRUE(reconnected.data);
    EXPECT_EQ(reconnected.time_ns, 10);
    EXPECT_EQ(reconnected.data->streams.at("pose").size(), 1U);
    EXPECT_TRUE(reconnected.data->frames->lookup("world", "body", 10).pose);
    EXPECT_FALSE(reconnected.data->frames->lookup("world", "body", 0).pose);
}

TEST(LiveSource, InvalidUpdatesLeaveAcceptedStateIntact) {
    EXPECT_THROW(LivePoseSource({"", "clock"}), std::invalid_argument);
    LivePoseSource source({"live", "clock"});
    source.publish({0, 1, {}});
    EXPECT_THROW(source.publish({0, -1, {}}), std::invalid_argument);
    Pose bad;
    bad.rotation.coeffs().setZero();
    EXPECT_THROW(source.publish({0, 2, bad}), std::invalid_argument);
    EXPECT_EQ(source.snapshot().time_ns, 1);
}

TEST(LiveSource, PendingOldEpochIsDiscardedAndSessionCannotAdvanceLiveClock) {
    auto source = std::make_shared<LivePoseSource>(LivePoseOptions{"live", "clock"});
    source->publish({0, 100, {}});
    source->publish({0, 200, {}});
    source->publish({1, 0, {}});
    robotics::viewer::Session session({{"live",
                                        [source](const auto &, const auto &) {
                                            return robotics::viewer::Connection{
                                                source, nullptr, 0,
                                                [source] { source->reconnect(); }};
                                        }}},
                                      standardDisplays());
    auto workspace = robotics::viewer::emptyWorkspace();
    workspace.sources.push_back({"live", "live", {}});
    workspace.selected_source = "live";
    session.open(std::move(workspace));
    ASSERT_TRUE(session.snapshot()->data);
    EXPECT_EQ(session.snapshot()->data->streams.at("pose").size(), 1U);
    EXPECT_FALSE(session.canSeek());
    EXPECT_THROW(session.seek(100), std::logic_error);
    session.advance(100);
    EXPECT_EQ(session.snapshot()->time_ns, 0);
    session.open(session.workspace());
    EXPECT_TRUE(session.snapshot()->data); // Reopening a retained endpoint does not disconnect it.
    session.open(robotics::viewer::emptyWorkspace());
    EXPECT_FALSE(source->snapshot().data); // Removing it does disconnect presentation.
}

TEST(LiveSource, ConcurrentProducerAndConsumerRetainNewestValue) {
    LivePoseSource source({"live", "clock", "world", "body", "pose", 8, 16});
    std::atomic<bool> done{false};
    std::thread producer([&] {
        for (Time t = 0; t < 10000; ++t)
            source.publish({0, t, {}});
        done.store(true);
    });
    Time previous = 0;
    do {
        const auto snapshot = source.snapshot();
        EXPECT_GE(snapshot.time_ns, previous);
        previous = snapshot.time_ns;
        if (snapshot.data) {
            EXPECT_LE(snapshot.data->streams.at("pose").size(), 16U);
        }
    } while (!done.load());
    producer.join();
    EXPECT_EQ(source.snapshot().time_ns, 9999);
}

TEST(LiveSource, FixedMountsFollowTheSameBodyHistoryAndSurviveReconnect) {
    LivePoseOptions options{"live", "clock"};
    options.fixed_frames = {{"body", "sensor", {{.2, -.1, .3}, Eigen::Quaterniond::Identity()}}};
    LivePoseSource source(options);
    options.fixed_frames.front().pose.translation.setZero(); // Construction owns its copy.
    const Pose body{{4, 5, -2},
                    Eigen::Quaterniond(Eigen::AngleAxisd(.8, Eigen::Vector3d::UnitZ()))};
    source.publish({0, 10, body});
    const auto before = source.snapshot();
    const auto world_sensor = before.data->frames->lookup("world", "sensor", 10);
    ASSERT_TRUE(world_sensor.pose);
    EXPECT_TRUE(world_sensor.pose->translation.isApprox(apply(body, {.2, -.1, .3}), 1e-14));
    source.disconnect();
    source.publish({1, 0, {}});
    source.reconnect();
    const auto after = source.snapshot();
    EXPECT_TRUE(after.data->frames->lookup("world", "sensor", 0)
                    .pose->translation.isApprox(Eigen::Vector3d(.2, -.1, .3)));
    EXPECT_TRUE(before.data->frames->lookup("world", "sensor", 10)
                    .pose->translation.isApprox(world_sensor.pose->translation));
    options.fixed_frames.front().child = "world";
    EXPECT_THROW((LivePoseSource(options)), std::invalid_argument);
    options.fixed_frames.front().child = "sensor";
    options.fixed_frames.front().parent = "missing";
    EXPECT_THROW((LivePoseSource(options)), std::invalid_argument);
}

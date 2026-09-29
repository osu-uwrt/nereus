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

TEST(LiveSource, MovingFramesAreAtomicAcrossLossReconnectAndProducerReset) {
    LivePoseOptions options{"live", "clock", "world", "body", "pose", 2, 3};
    options.fixed_frames = {{"body", "mount", {{0, 2, 0}, Eigen::Quaterniond::Identity()}}};
    options.moving_frames = {{"mount", "rotor"}};
    LivePoseSource source(options);
    const auto packet = [](std::uint64_t generation, Time time) {
        const double x = static_cast<double>(time);
        return PoseUpdate{generation,
                          time,
                          {{x, 0, 0}, Eigen::Quaterniond::Identity()},
                          {{{0, 0, x}, Eigen::Quaterniond::Identity()}}};
    };
    source.publish(packet(0, 1));
    auto invalid = packet(1, 0);
    invalid.moving_poses.front().rotation.coeffs().setZero();
    EXPECT_THROW(source.publish(invalid), std::invalid_argument);
    EXPECT_THROW(source.publish({0, 2, {}}), std::invalid_argument);
    const auto retained = source.snapshot();
    ASSERT_TRUE(retained.data);
    EXPECT_EQ(retained.time_ns, 1);
    EXPECT_EQ(retained.generation, 0U);
    for (Time t = 2; t <= 5; ++t)
        source.publish(packet(0, t));
    const auto newest = source.snapshot();
    EXPECT_EQ(newest.delivery.dropped_queue, 2U);
    const auto pose = newest.data->frames->lookup("world", "rotor", newest.time_ns).pose;
    ASSERT_TRUE(pose);
    EXPECT_EQ(pose->translation, Eigen::Vector3d(5, 2, 5));
    source.disconnect();
    source.publish(packet(0, 6));
    EXPECT_FALSE(source.snapshot().data);
    source.reconnect();
    const auto reconnected = source.snapshot();
    EXPECT_EQ(reconnected.time_ns, 6);
    EXPECT_FALSE(reconnected.data->frames->lookup("world", "rotor", 5).pose);
    EXPECT_EQ(reconnected.data->frames->lookup("world", "rotor", 6).pose->translation,
              Eigen::Vector3d(6, 2, 6));
    source.publish(packet(1, 0));
    source.publish(packet(0, 7));
    const auto reset = source.snapshot();
    EXPECT_EQ(reset.time_ns, 0);
    EXPECT_EQ(reset.delivery.rejected_stale, 1U);
    EXPECT_EQ(reset.data->frames->lookup("world", "rotor", 0).pose->translation,
              Eigen::Vector3d(0, 2, 0));
    EXPECT_FALSE(reset.data->frames->lookup("world", "rotor", 6).pose);
    EXPECT_EQ(retained.data->frames->lookup("world", "rotor", 1).pose->translation,
              Eigen::Vector3d(1, 2, 1));
}

TEST(LiveSource, MovingTopologyAndCombinedHistoryAreValidatedAtConstruction) {
    LivePoseOptions options{"live", "clock"};
    options.moving_frames = {{"missing", "rotor"}};
    EXPECT_THROW((LivePoseSource(options)), std::invalid_argument);
    options.moving_frames = {{"body", "body"}};
    EXPECT_THROW((LivePoseSource(options)), std::invalid_argument);
    options.fixed_frames = {{"body", "rotor", {}}};
    options.moving_frames = {{"body", "rotor"}};
    EXPECT_THROW((LivePoseSource(options)), std::invalid_argument);
    options.fixed_frames.clear();
    options.history_capacity = 10000;
    options.moving_frames.clear();
    for (int i = 0; i < 10; ++i)
        options.moving_frames.push_back({"body", "rotor" + std::to_string(i)});
    EXPECT_THROW((LivePoseSource(options)), std::invalid_argument);
    options.moving_frames.pop_back();
    EXPECT_NO_THROW((LivePoseSource(options)));
}

TEST(LiveSource, ColorBatchSharesPoseCutoffAndRejectsPartialOrInvalidUpdates) {
    LivePoseOptions options{"live", "clock", "world", "body", "pose", 1, 2};
    options.color_channels = {"status"};
    LivePoseSource source(options);
    source.publish({0, 1, {}, {}, {{1, 0, 0}}});
    const auto retained = source.snapshot();
    EXPECT_THROW(source.publish({1, 0, {}}), std::invalid_argument);
    EXPECT_THROW(source.publish({1, 0, {}, {}, {{0, -1, 0}}}), std::invalid_argument);
    EXPECT_EQ(source.snapshot().generation, retained.generation);
    source.publish({0, 2, {}, {}, {{0, 1, 0}}});
    source.publish({0, 3, {}, {}, {{0, 0, 1}}});
    const auto latest = source.snapshot();
    EXPECT_EQ(latest.time_ns, 3);
    EXPECT_EQ(latest.delivery.dropped_queue, 1U);
    EXPECT_EQ(colorAt(latest.data->colors.at("status"), 3).value(), Eigen::Vector3f(0, 0, 1));
    source.disconnect();
    source.publish({1, 0, {}, {}, {{0, 0, 0}}});
    source.reconnect();
    const auto reset = source.snapshot();
    EXPECT_EQ(reset.time_ns, 0);
    EXPECT_EQ(reset.data->colors.at("status").size(), 1U);
    EXPECT_EQ(colorAt(reset.data->colors.at("status"), 0).value(), Eigen::Vector3f::Zero());
    EXPECT_EQ(colorAt(retained.data->colors.at("status"), 1).value(), Eigen::Vector3f(1, 0, 0));
    options.color_channels = {"duplicate", "duplicate"};
    EXPECT_THROW((LivePoseSource(options)), std::invalid_argument);
    options.color_channels = {""};
    EXPECT_THROW((LivePoseSource(options)), std::invalid_argument);
}

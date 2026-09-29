#pragma once
#include <robotics/visualization/source.hpp>

namespace robotics::visualization {
struct PoseUpdate {
    std::uint64_t generation{0}; // Producer epoch; increase before rewinding its clock.
    Time time_ns{0};
    Pose pose;                        // Body frame into the configured world frame.
    std::vector<Pose> moving_poses{}; // Complete configured moving-frame batch, same timestamp.
};
struct MovingFrame {
    std::string parent, child;
};
struct LivePoseOptions {
    std::string id;
    std::string clock;
    std::string world_frame{"world"};
    std::string body_frame{"body"};
    std::string stream{"pose"};
    std::size_t queue_capacity{64};
    std::size_t history_capacity{1000};
    std::vector<spatial::FixedFrame> fixed_frames{}; // Immutable source-owned mount/world frames.
    std::vector<MovingFrame> moving_frames{}; // Ordered per-update poses, without topology changes.
};
// One producer calls publish; one presentation thread calls snapshot/disconnect/reconnect.
// Join both callers before destruction.
// Copy/drain uses a bounded mutex critical section; no graphics/IO runs under it.
// Overflow keeps newest samples and reports drops without waiting for consumption.
class LivePoseSource final : public Source {
  public:
    explicit LivePoseSource(LivePoseOptions options);
    ~LivePoseSource() override;
    LivePoseSource(const LivePoseSource &) = delete;
    LivePoseSource &operator=(const LivePoseSource &) = delete;
    const std::vector<MovingFrame> &
    movingFrames() const; // Immutable configuration, source lifetime.
    void publish(const PoseUpdate &update);
    SourceSnapshot snapshot() const override;
    void disconnect() override;
    void reconnect(); // Seeds a new display epoch from the latest producer value.

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace robotics::visualization

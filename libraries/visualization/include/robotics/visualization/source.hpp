#pragma once
#include <robotics/visualization/frames.hpp>

#include <memory>

namespace robotics::visualization {
struct PoseSample {
    Time time_ns{0};
    std::string frame;
    Pose pose;
};
using PoseHistory = std::vector<PoseSample>;
struct SourceData {
    std::string clock;
    std::shared_ptr<const FrameGraph> frames;
    std::map<std::string, PoseHistory> streams;
};
struct Recording {
    SourceData data;
    Time duration_ns{0};
};
void validate(const Recording &recording);
struct SourceSnapshot {
    std::string id;
    std::uint64_t generation{0};
    Time time_ns{0};
    // Null means disconnected. Retained snapshots own immutable data.
    std::shared_ptr<const SourceData> data;
};
class Source {
  public:
    virtual ~Source() = default;
    virtual SourceSnapshot snapshot() const = 0;
    virtual void disconnect() = 0;
};
// Separate optional capability: a live Source need not support seeking.
class Playback {
  public:
    virtual ~Playback() = default;
    virtual void seek(Time time_ns) = 0;
};
class LocalSource final : public Source, public Playback {
  public:
    LocalSource(std::string id, Recording recording);
    SourceSnapshot snapshot() const override;
    void disconnect() override;
    void reconnect();
    void seek(Time time_ns) override;
    Time duration() const;

  private:
    std::string id_;
    std::shared_ptr<const SourceData> data_;
    Time duration_ns_{0};
    Time time_ns_{0};
    std::uint64_t generation_{0};
    bool connected_{true};
};
} // namespace robotics::visualization

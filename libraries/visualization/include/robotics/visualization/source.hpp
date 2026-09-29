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
struct ColorSample {
    Time time_ns{0};
    Eigen::Vector3f rgb{Eigen::Vector3f::Zero()}; // Linear normalized RGB, no radiance.
};
using ColorHistory = std::vector<ColorSample>;
void validateColor(const Eigen::Vector3f &rgb);
// Validated, strictly increasing history. Hold preceding value within the observed
// range; no interpolation across transitions and no extrapolation past the last sample.
std::optional<Eigen::Vector3f> colorAt(const ColorHistory &, Time time_ns);
struct SourceData {
    std::string clock;
    std::shared_ptr<const FrameGraph> frames;
    std::map<std::string, PoseHistory> streams;
    std::map<std::string, ColorHistory> colors;
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
    // Null means no active data (disconnected or awaiting the first update).
    // Retained snapshots own immutable data.
    std::shared_ptr<const SourceData> data;
    struct Delivery {
        std::uint64_t dropped_queue{0};
        std::uint64_t trimmed_history{0};
        std::uint64_t rejected_stale{0};
    } delivery{};
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

#pragma once
#include <robotics/simulation/plant.hpp>
#include <robotics/visualization/animation.hpp>
#include <robotics/visualization/live_source.hpp>

namespace robotics::integrations {
// Observation-only conversion. Snapshot pose is COM, not a robot's base_link or CAD origin.
// Source options name that frame; a content-owned transform can mount a future robot model.
void publishSimulationPose(visualization::LivePoseSource &destination,
                           const simulation::Snapshot &snapshot);
// Single simulation-side owner; processes every observation before lossy delivery.
// Requires consecutive ticks and increasing times within each producer generation when
// animating. This publisher must be the sole producer for its destination. New generations
// clear the animation. Force channels are mapped by configured IDs, not profile order.
class SimulationPosePublisher {
  public:
    SimulationPosePublisher(visualization::LivePoseSource &destination,
                            std::vector<std::string> plant_inputs,
                            std::optional<visualization::RotorRig> rig = std::nullopt);
    void publish(const simulation::Snapshot &snapshot);

  private:
    visualization::LivePoseSource &destination_;
    std::size_t plant_input_count_;
    std::optional<visualization::RotorRig> rig_;
    std::optional<visualization::RotorAnimator> animator_, staged_animator_;
    std::vector<std::size_t> inputs_;
    std::vector<float> forces_;
    visualization::PoseUpdate packet_;
    std::uint64_t previous_tick_{0};
    bool have_packet_{false};
};
} // namespace robotics::integrations

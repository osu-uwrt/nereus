#pragma once
#include <robotics/simulation/plant.hpp>
#include <robotics/visualization/live_source.hpp>

namespace robotics::integrations {
// Observation-only conversion. Snapshot pose is COM, not a robot's base_link or CAD origin.
// Source options name that frame; a content-owned transform can mount a future robot model.
void publishSimulationPose(visualization::LivePoseSource &destination,
                           const simulation::Snapshot &snapshot);
} // namespace robotics::integrations

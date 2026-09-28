#include <robotics/integrations/simulation_view.hpp>

namespace robotics::integrations {
void publishSimulationPose(visualization::LivePoseSource &destination,
                           const simulation::Snapshot &snapshot) {
    destination.publish({snapshot.generation,
                         snapshot.elapsed.count(),
                         {snapshot.body.position, snapshot.body.orientation}});
}
} // namespace robotics::integrations

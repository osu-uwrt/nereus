#include <robotics/integrations/simulation_view.hpp>
#include <stdexcept>

int main() {
    robotics::simulation::BodyState initial;
    initial.position = {5, 5, -2};
    robotics::simulation::Plant plant({}, initial);
    robotics::visualization::LivePoseSource source({"simulation", "simulation_clock"});
    robotics::integrations::publishSimulationPose(source, plant.advance());
    const auto view = source.snapshot();
    if (!view.data || view.time_ns != plant.observe().elapsed.count())
        throw std::runtime_error("installed simulation adapter did not deliver state");
}

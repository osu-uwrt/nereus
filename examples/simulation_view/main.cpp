#include <robotics/integrations/simulation_view.hpp>
#include <stdexcept>

int main() {
    namespace v = robotics::visualization;
    robotics::simulation::BodyState initial;
    initial.position = {5, 5, -2};
    robotics::simulation::PlantParameters parameters;
    parameters.thrusters.push_back({"motor"});
    robotics::simulation::Plant plant(parameters, initial);
    v::RotorRig rig;
    rig.inputs = {"motor"};
    rig.animation.curve.forward = rig.animation.curve.reverse = {0, 60, 0, 0};
    rig.mounts = {{"body", "rotor", {0, 0, 0}, Eigen::Vector3d::UnitX()}};
    v::LivePoseOptions options{"simulation", "simulation_clock"};
    options.moving_frames = {{"body", "rotor"}};
    v::LivePoseSource source(options);
    robotics::integrations::SimulationPosePublisher publisher(source, {"motor"}, rig);
    publisher.publish(plant.observe());
    plant.command(Eigen::VectorXd::Constant(1, 4));
    for (int tick = 0; tick < 100; ++tick)
        publisher.publish(plant.advance());
    const auto view = source.snapshot();
    if (!view.data || view.time_ns != plant.observe().elapsed.count())
        throw std::runtime_error("installed simulation adapter did not deliver state");
    const auto rotor = view.data->frames->lookup("body", "rotor", view.time_ns).pose;
    if (!rotor || rotor->rotation.isApprox(Eigen::Quaterniond::Identity()))
        throw std::runtime_error("installed simulation adapter did not animate moving frame");
}

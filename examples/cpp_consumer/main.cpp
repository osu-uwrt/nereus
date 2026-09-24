#include <iostream>
#include <robotics/simulation/plant.hpp>

int main() {
    robotics::simulation::PlantParameters parameters;
    robotics::simulation::BodyState initial;
    initial.position = {5, 5, -2};
    robotics::simulation::Plant plant(parameters, initial);
    const auto state = plant.advance(500);
    if (state.tick != 500 || state.elapsed.count() != 1'000'000'000 ||
        (state.body.position - initial.position).norm() > 1e-12) {
        return 1;
    }
    std::cout << "Installed plant advanced one simulated second without ROS.\n";
}

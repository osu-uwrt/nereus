#include "robotics/config/scenario.hpp"
#include <iomanip>
#include <iostream>
#include <string>

namespace {
void print(const robotics::simulation::Snapshot &s) {
    const auto &b = s.body;
    std::cout << s.generation << ',' << s.tick << ',' << s.elapsed.count() << ',' << b.position.x()
              << ',' << b.position.y() << ',' << b.position.z() << ',' << b.orientation.w() << ','
              << b.orientation.x() << ',' << b.orientation.y() << ',' << b.orientation.z();
    for (int i = 0; i < 3; ++i) {
        std::cout << ',' << b.linear_velocity[i];
    }
    for (int i = 0; i < 3; ++i) {
        std::cout << ',' << b.angular_velocity[i];
    }
    for (Eigen::Index i = 0; i < s.thruster_forces.size(); ++i) {
        std::cout << ',' << s.thruster_forces[i];
    }
    std::cout << '\n';
}
} // namespace

int main(int argc, char **argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "Usage: robotics-sim SCENARIO.yaml\n"
                     "Run fixed simulation ticks without ROS or graphics; write CSV to stdout.\n";
        return 0;
    }
    if (argc != 2) {
        std::cerr << "Usage: robotics-sim SCENARIO.yaml\n";
        return 2;
    }
    try {
        const auto scenario = robotics::config::loadScenario(argv[1]);
        robotics::simulation::Plant plant(scenario.plant, scenario.initial);
        std::cout << std::setprecision(17)
                  << "generation,tick,time_ns,x_m,y_m,z_m,qw,qx,qy,qz,u_m_s,v_m_s,w_m_s,p_rad_s,q_"
                     "rad_s,r_rad_s";
        for (std::size_t i = 0; i < scenario.plant.thrusters.size(); ++i) {
            std::cout << ",thrust_" << i << "_n";
        }
        std::cout << '\n';
        print(plant.observe());
        std::size_t command = 0;
        for (std::uint64_t tick = 0; tick < scenario.ticks; ++tick) {
            if (command < scenario.commands.size() && scenario.commands[command].tick == tick) {
                plant.command(scenario.commands[command++].forces);
            }
            print(plant.advance());
            if (!std::cout) {
                throw std::runtime_error("failed to write trajectory");
            }
        }
        std::cout.flush();
        return std::cout ? 0 : 1;
    } catch (const std::exception &error) {
        std::cerr << "robotics-sim: " << error.what() << '\n';
        return 1;
    }
}

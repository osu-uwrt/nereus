#include "robotics/config/scenario.hpp"
#include "telemetry.hpp"
#include <fstream>
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
        std::cout << "Usage: nereus-run SCENARIO.yaml [--sensors OUTPUT.csv]\n"
                     "Run fixed simulation ticks without ROS or graphics; write CSV to stdout.\n";
        return 0;
    }
    if (argc != 2 && (argc != 4 || std::string(argv[2]) != "--sensors")) {
        std::cerr << "Usage: nereus-run SCENARIO.yaml [--sensors OUTPUT.csv]\n";
        return 2;
    }
    try {
        const auto scenario = robotics::config::loadScenario(argv[1]);
        auto runtime = robotics::config::makeRuntime(scenario);
        std::ofstream sensor_output;
        const auto observers = robotics::runner::telemetry(*runtime, scenario.sensors,
                                                           argc == 4 ? &sensor_output : nullptr);
        if (argc == 4) {
            const auto destination = std::filesystem::weakly_canonical(argv[3]);
            for (const auto &source : scenario.sources) {
                if (destination == std::filesystem::weakly_canonical(source) ||
                    (std::filesystem::exists(destination) &&
                     std::filesystem::equivalent(destination, source))) {
                    throw std::invalid_argument(
                        "sensor output must not overwrite an input profile");
                }
            }
            sensor_output.open(argv[3]);
            if (!sensor_output) {
                throw std::runtime_error("cannot open sensor output file");
            }
            robotics::runner::sensorHeader(sensor_output);
        }
        std::cout << std::setprecision(17)
                  << "generation,tick,time_ns,x_m,y_m,z_m,qw,qx,qy,qz,u_m_s,v_m_s,w_m_s,p_rad_s,q_"
                     "rad_s,r_rad_s";
        for (std::size_t i = 0; i < scenario.plant.thrusters.size(); ++i) {
            std::cout << ",thrust_" << i << "_n";
        }
        std::cout << '\n';
        print(runtime->observe());
        std::size_t command = 0;
        for (std::uint64_t tick = 0; tick < scenario.ticks; ++tick) {
            if (command < scenario.commands.size() && scenario.commands[command].tick == tick) {
                runtime->command(scenario.commands[command++].forces);
            }
            print(runtime->advance());
            for (const auto &observe : observers) {
                observe();
            }
            if (!std::cout) {
                throw std::runtime_error("failed to write trajectory");
            }
        }
        if (argc == 4) {
            sensor_output.flush();
            if (!sensor_output) {
                throw std::runtime_error("failed to flush sensor observations");
            }
        }
        std::cout.flush();
        return std::cout ? 0 : 1;
    } catch (const std::exception &error) {
        std::cerr << "nereus-run: " << error.what() << '\n';
        return 1;
    }
}

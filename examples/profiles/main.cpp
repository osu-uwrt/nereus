#include "robotics/config/scenario.hpp"
#include "robotics/sensors/readings.hpp"
#include <iostream>

int main(int argc, char **argv) {
    if (argc != 2) {
        return 2;
    }
    const auto scenario = robotics::config::loadScenario(argv[1]);
    auto runtime = robotics::config::makeRuntime(scenario);
    auto pressure = runtime->stream<robotics::sensors::PressureReading>("pressure");
    runtime->advance(25);
    const auto first = pressure->latest();
    if (!first || !first->measurement.value) {
        return 1;
    }
    std::cout << "Pressure-derived depth: " << first->measurement.value->depth << " m\n";
    runtime->reset(scenario.initial, scenario.seed);
    if (pressure->latest()) {
        return 1;
    }
    runtime->advance(25);
    const auto replay = pressure->latest();
    return !replay || !replay->measurement.value ||
           replay->measurement.value->absolute_pressure !=
               first->measurement.value->absolute_pressure;
}

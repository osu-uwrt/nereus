#include "telemetry.hpp"
#include <algorithm>
#include <charconv>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <robotics/config/scenario.hpp>

namespace {
using Clock = std::chrono::steady_clock;
using Scenario = robotics::config::Scenario;
void measure(const Scenario &scenario, int iterations, bool sensors) {
    std::vector<double> timings;
    timings.reserve(static_cast<std::size_t>(scenario.ticks) *
                    static_cast<std::size_t>(iterations));
    double wall_seconds = 0, checksum = 0;
    for (int run = -3; run < iterations; ++run) {
        auto execute = [&](auto &runtime, const std::vector<std::function<void()>> &observers) {
            std::size_t command = 0;
            const auto started = Clock::now();
            for (std::uint64_t tick = 0; tick < scenario.ticks; ++tick) {
                const auto before = Clock::now();
                if (command < scenario.commands.size() && scenario.commands[command].tick == tick)
                    runtime.command(scenario.commands[command++].forces);
                runtime.advance();
                for (const auto &observe : observers)
                    observe();
                const double elapsed =
                    std::chrono::duration<double, std::micro>(Clock::now() - before).count();
                if (run >= 0)
                    timings.push_back(elapsed);
            }
            if (run >= 0) {
                wall_seconds += std::chrono::duration<double>(Clock::now() - started).count();
                checksum += runtime.observe().body.position.sum();
            }
        };
        if (sensors) {
            auto runtime = robotics::config::makeRuntime(scenario);
            execute(*runtime, robotics::runner::telemetry(*runtime, scenario.sensors, nullptr));
        } else {
            robotics::simulation::Plant plant(scenario.plant, scenario.initial);
            execute(plant, {});
        }
    }
    std::sort(timings.begin(), timings.end());
    const auto quantile = [&](double fraction) {
        return timings[static_cast<std::size_t>(fraction *
                                                static_cast<double>(timings.size() - 1))];
    };
    const double simulated_seconds = static_cast<double>(iterations) *
                                     static_cast<double>(scenario.ticks) *
                                     std::chrono::duration<double>(scenario.plant.timestep).count();
    std::cout << "{\"mode\":\"" << (sensors ? "plant_and_scheduled_sensors" : "plant")
              << "\",\"measured_ticks\":" << timings.size() << ",\"mean_us\":"
              << std::accumulate(timings.begin(), timings.end(), 0.0) /
                     static_cast<double>(timings.size())
              << ",\"p50_us\":" << quantile(.5) << ",\"p95_us\":" << quantile(.95)
              << ",\"p99_us\":" << quantile(.99) << ",\"max_us\":" << timings.back()
              << ",\"wall_seconds\":" << wall_seconds
              << ",\"simulated_seconds\":" << simulated_seconds
              << ",\"real_time_factor\":" << simulated_seconds / wall_seconds
              << ",\"final_position_checksum\":" << checksum << "}\n";
}
} // namespace
int main(int argc, char **argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--help") {
            std::cout << "nereus-benchmark SCENARIO.yaml [ITERATIONS=20]\n"
                         "Three warmup runs; plant then plant+sensors; no IO/graphics in measured "
                         "steps.\n";
            return 0;
        }
        if (argc < 2 || argc > 3)
            throw std::invalid_argument("expected SCENARIO.yaml [ITERATIONS]; see --help");
        int iterations = 20;
        if (argc == 3) {
            const std::string text(argv[2]);
            const auto result = std::from_chars(text.data(), text.data() + text.size(), iterations);
            if (result.ec != std::errc{} || result.ptr != text.data() + text.size() ||
                iterations < 1 || iterations > 1000)
                throw std::invalid_argument("iterations must be in [1,1000]");
        }
        const auto scenario = robotics::config::loadScenario(argv[1]);
        if (scenario.ticks == 0 || scenario.ticks > 10000000U / static_cast<unsigned>(iterations))
            throw std::invalid_argument(
                "benchmark requires between 1 and 10 million measured ticks");
        std::cout << std::setprecision(10);
        measure(scenario, iterations, false);
        measure(scenario, iterations, true);
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "nereus-benchmark: " << error.what() << '\n';
        return 1;
    }
}

#include "running_scenario.hpp"
#include "telemetry.hpp"
#include <limits>
#include <robotics/integrations/simulation_view.hpp>
#include <stdexcept>

namespace robotics::runner {
RunningScenario::RunningScenario(config::Scenario scenario,
                                 std::shared_ptr<visualization::LivePoseSource> destination,
                                 bool paced, std::optional<visualization::RotorRig> rotors) {
    if (!destination)
        throw std::invalid_argument("simulation presentation source is required");
    worker_ =
        std::thread([this, scenario = std::move(scenario), destination = std::move(destination),
                     paced, rotors = std::move(rotors)] {
            try {
                run(scenario, destination, paced, std::move(rotors));
            } catch (...) {
                failure_ = std::current_exception();
            }
            finished_.store(true);
        });
}
RunningScenario::~RunningScenario() {
    stop();
    if (worker_.joinable())
        worker_.join();
}
bool RunningScenario::finished() const {
    return finished_.load();
}
void RunningScenario::stop() {
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        stopped_ = true;
    }
    wake_.notify_all();
}
void RunningScenario::join() {
    if (worker_.joinable())
        worker_.join();
    if (failure_)
        std::rethrow_exception(failure_);
}
void RunningScenario::run(config::Scenario scenario,
                          const std::shared_ptr<visualization::LivePoseSource> &destination,
                          bool paced, std::optional<visualization::RotorRig> rotors) {
    auto runtime = config::makeRuntime(scenario);
    if (scenario.ticks > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max() /
                                                    scenario.plant.timestep.count()))
        throw std::overflow_error("scenario duration overflows simulation clock");
    const auto observers = telemetry(*runtime, scenario.sensors, nullptr);
    std::vector<std::string> force_ids;
    for (const auto &thruster : scenario.plant.thrusters)
        force_ids.push_back(thruster.id);
    integrations::SimulationPosePublisher publisher(*destination, std::move(force_ids),
                                                    std::move(rotors));
    publisher.publish(runtime->observe());
    using Clock = std::chrono::steady_clock;
    const auto started = Clock::now();
    std::size_t command = 0;
    for (std::uint64_t tick = 0; tick < scenario.ticks; ++tick) {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            // Advance exactly one configured step. Wall time only paces execution;
            // no skipped steps or graphics-driven acquisition, including when behind.
            if (paced) {
                const auto desired = runtime->observe().elapsed + scenario.plant.timestep;
                const auto elapsed =
                    std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - started);
                if (desired > elapsed)
                    wake_.wait_for(lock, desired - elapsed, [this] { return stopped_; });
            }
            if (stopped_)
                return;
        }
        if (command < scenario.commands.size() && scenario.commands[command].tick == tick)
            runtime->command(scenario.commands[command++].forces);
        publisher.publish(runtime->advance());
        for (const auto &observe : observers)
            observe();
    }
}
} // namespace robotics::runner

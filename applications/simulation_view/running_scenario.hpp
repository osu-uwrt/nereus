#pragma once
#include <atomic>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <robotics/config/scenario.hpp>
#include <robotics/visualization/animation.hpp>
#include <robotics/visualization/live_source.hpp>
#include <thread>

namespace robotics::runner {
// Application-owned execution. The runtime and sensor consumers live on its worker;
// graphics only read the supplied source. Destruction cancels and joins the worker.
class RunningScenario {
  public:
    RunningScenario(config::Scenario scenario,
                    std::shared_ptr<visualization::LivePoseSource> destination, bool paced = true,
                    std::optional<visualization::RotorRig> rotors = std::nullopt);
    ~RunningScenario();
    RunningScenario(const RunningScenario &) = delete;
    RunningScenario &operator=(const RunningScenario &) = delete;
    bool finished() const;
    void stop();
    void join(); // Rethrows worker failure after joining. Call only from the owner thread.

  private:
    void run(config::Scenario scenario,
             const std::shared_ptr<visualization::LivePoseSource> &destination, bool paced,
             std::optional<visualization::RotorRig> rotors);
    std::mutex mutex_;
    std::condition_variable wake_;
    bool stopped_{false};
    std::atomic<bool> finished_{false};
    std::exception_ptr failure_;
    std::thread worker_;
};
} // namespace robotics::runner

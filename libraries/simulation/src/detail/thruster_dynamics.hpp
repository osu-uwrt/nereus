#pragma once
#include <Eigen/Dense>
#include <deque>
#include <vector>
namespace nereus::simulation::detail {

// One thruster's actuator model: delay, rise and fall time constants (s), slew limit (N/s), deadband and
// force limits (N), direction-dependent command scale and an efficiency in [0, 1].
struct ThrusterParameters {
    double delay = .1, rise = .08, fall = .06, slew = 300, deadband = 0;
    double forwardLimit = 28, reverseLimit = 28, forwardScale = 1, reverseScale = 1, efficiency = 1;
};

// Simulation-time actuator dynamics, independent of ROS scheduling and wall
// time.
class ThrusterDynamics {
  public:
    // One parameter set per thruster; `timeout` (simulated s) zeroes targets after the last command
    // (0 disables the watchdog).
    void configure(const std::vector<ThrusterParameters> &parameters, double timeout = .5);
    // Queues desired forces (N), each due after its thruster's delay.
    void command(const Eigen::VectorXd &force);
    // Advances the actuator clock by dt seconds: applies due commands, the watchdog, and the force response.
    void advance(double dt);
    void stop(); // Cancel queued commands immediately; propellers coast down.
    void reset();
    void clear(); // Clears force/history while preserving the actuator clock.
    double time() const {
        return time_;
    }
    const Eigen::VectorXd &forces() const {
        return forces_;
    }

  private:
    // A queued target force and the actuator time it takes effect.
    struct Command {
        double time;
        double force;
    };

    std::vector<ThrusterParameters> parameters_;
    std::vector<std::deque<Command>> queues_;
    Eigen::VectorXd targets_, forces_;
    // lastCommand_ starts far in the past so the watchdog doesn't fire before the first command.
    double time_ = 0, timeout_ = .5, lastCommand_ = -1e9;

    // First-order response of thruster i toward its target over dt, slew-limited.
    void evolve(Eigen::Index i, double dt);
};
} // namespace nereus::simulation::detail

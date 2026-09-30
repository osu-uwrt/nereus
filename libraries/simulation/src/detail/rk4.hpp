#pragma once
#include "marine_dynamics.hpp"
#include <cmath>
#include <stdexcept>

namespace nereus::simulation::detail {
// Evaluate all state/time-dependent forces at each stage. Actuator midpoint
// values may be held by the caller, while geometry/environment remain stage-local.
template <class Derivative> State13d integrateBodyRk4(const State13d &state, double dt, Derivative derivative) {
    if (!std::isfinite(dt) || dt <= 0 || dt > .1)
        throw std::invalid_argument("RK4 step must be in (0,0.1] seconds");
    const State13d k1 = derivative(state, 0.0);
    const State13d k2 = derivative(state + dt * .5 * k1, dt * .5);
    const State13d k3 = derivative(state + dt * .5 * k2, dt * .5);
    const State13d k4 = derivative(state + dt * k3, dt);
    State13d next = state + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
    MarineDynamics::validateState(next);
    // Caller normalizes after its selected contact phase. The original box
    // contact solver consumes the raw RK4 endpoint before state commit.
    return next;
}
} // namespace nereus::simulation::detail

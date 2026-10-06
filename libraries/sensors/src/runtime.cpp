// Sensor runtime implementation: plant stepping, device fan-out, and fault/reset handling.
#include "nereus/sensors/runtime.hpp"

namespace nereus::sensors {
Runtime::Runtime(const simulation::PlantParameters &parameters, const simulation::BodyState &initial,
                 std::uint64_t seed)
    : plant_(parameters, initial), timestep_(parameters.timestep), seed_(seed) {}
Runtime::~Runtime() = default;

void Runtime::requireHealthy() const {
    if (faulted_) {
        throw std::logic_error("sensor runtime must be reset after a failed advance/reset");
    }
}

// Rejects late registration, empty/NUL-containing ids or frames, duplicates, periods shorter
// than a plant tick, negative latency, zero capacity, and out-of-range overflow policies.
void Runtime::validateDevice(const Device &device) const {
    requireHealthy();
    if (sealed_) {
        throw std::logic_error("sensor registration is closed after first advancement");
    }
    if (device.id.empty() || device.frame.empty() || device.id.find('\0') != std::string::npos ||
        device.frame.find('\0') != std::string::npos || streams_.count(device.id) || device.period < timestep_ ||
        device.latency.count() < 0 || device.capacity == 0 ||
        (device.overflow != OverflowPolicy::Fail && device.overflow != OverflowPolicy::DropOldest)) {
        throw std::invalid_argument("invalid or duplicate sensor identity, period, latency, or capacity");
    }
}

void Runtime::command(const Eigen::VectorXd &forces) {
    requireHealthy();
    plant_.command(forces);
}

void Runtime::stopThrusters() {
    requireHealthy();
    plant_.stopThrusters();
}

simulation::Snapshot Runtime::observe() const {
    return plant_.observe();
}

// Steps the plant tick by tick, offering each post-step motion sample to every device.
// Any exception faults the runtime and invalidates all streams.
simulation::Snapshot Runtime::advance(std::uint64_t ticks) {
    requireHealthy();
    // Elapsed time is tick * timestep in int64 nanoseconds; refuse ticks that would overflow it.
    const auto max_tick = static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max() / timestep_.count());
    if (ticks > max_tick - plant_.observe().tick) {
        throw std::overflow_error("requested advance overflows simulation time");
    }
    if (ticks != 0) {
        sealed_ = true;
    }

    try {
        for (std::uint64_t i = 0; i < ticks; ++i) {
            plant_.advance();
            if (!devices_.empty()) {
                const auto motion = plant_.motion();
                for (auto &device : devices_) {
                    device->advance(motion);
                }
            }
        }
    } catch (...) {
        faulted_ = true;
        for (auto &device : devices_) {
            device->invalidate();
        }
        throw;
    }
    return observe();
}

simulation::Snapshot Runtime::place(const simulation::BodyState &state, bool clear_actuators) {
    requireHealthy();
    const auto snapshot = plant_.place(state, clear_actuators);
    for (auto &device : devices_) {
        device->discardBuffered();
    }
    return snapshot;
}

// Resets the plant and reseeds every device; also the only way to clear a fault.
simulation::Snapshot Runtime::reset(const simulation::BodyState &initial, std::uint64_t seed) {
    // Invalid plant initial conditions are rejected before changing any sensor state.
    const auto snapshot = plant_.reset(initial);
    try {
        for (auto &device : devices_) {
            device->reset(seed);
        }
        seed_ = seed;
        faulted_ = false;
    } catch (...) {
        faulted_ = true;
        for (auto &device : devices_) {
            device->invalidate();
        }
        throw;
    }
    return snapshot;
}
} // namespace nereus::sensors

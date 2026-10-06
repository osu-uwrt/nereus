// Core sensor scheduling types: device configuration, per-sample metadata, and stream counters.
#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace nereus::sensors {
using Nanoseconds = std::chrono::nanoseconds;

// What a full pending/delivered queue does: throw, or drop its oldest sample and count it.
enum class OverflowPolicy { Fail, DropOldest };

// Scheduling configuration for one registered sensor (period/latency in simulation time).
struct Device {
    std::string id;
    std::string frame;
    Nanoseconds period{10'000'000};
    Nanoseconds latency{0};
    std::size_t capacity = 64; // Bound each of pending and delivered queues.
    OverflowPolicy overflow = OverflowPolicy::Fail;
};

// Metadata stamped on every sample. scheduled = due time, acquired = sim time it was taken,
// delivered = sim time it left the latency queue; sequence counts acquisitions since reset.
struct SampleHeader {
    std::string device_id;
    std::string frame;
    std::uint64_t generation = 0, sequence = 0, tick = 0;
    Nanoseconds scheduled{0}, acquired{0}, delivered{0};
};

// Exactly one of value / unavailable_reason is set (the scheduler enforces this).
template <class T> struct Measurement {
    std::optional<T> value;
    std::string unavailable_reason;
};

template <class T> struct Sample {
    SampleHeader header;
    Measurement<T> measurement;
};

// Per-stream counters since the last reset.
struct StreamStats {
    std::uint64_t acquired = 0, delivered = 0, unavailable = 0;
    std::uint64_t dropped_pending = 0, dropped_delivered = 0;
};

} // namespace nereus::sensors

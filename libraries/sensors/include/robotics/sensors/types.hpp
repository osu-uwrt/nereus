#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace robotics::sensors {
using Nanoseconds = std::chrono::nanoseconds;

enum class OverflowPolicy { Fail, DropOldest };
struct Device {
    std::string id;
    std::string frame;
    Nanoseconds period{10'000'000};
    Nanoseconds latency{0};
    std::size_t capacity = 64; // Bound each of pending and delivered queues.
    OverflowPolicy overflow = OverflowPolicy::Fail;
};

struct SampleHeader {
    std::string device_id;
    std::string frame;
    std::uint64_t generation = 0, sequence = 0, tick = 0;
    Nanoseconds scheduled{0}, acquired{0}, delivered{0};
};

template <class T> struct Measurement {
    std::optional<T> value;
    std::string unavailable_reason;
};
template <class T> struct Sample {
    SampleHeader header;
    Measurement<T> measurement;
};
struct StreamStats {
    std::uint64_t acquired = 0, delivered = 0, unavailable = 0;
    std::uint64_t dropped_pending = 0, dropped_delivered = 0;
};

} // namespace robotics::sensors

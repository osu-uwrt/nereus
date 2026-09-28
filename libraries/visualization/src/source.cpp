#include <robotics/visualization/source.hpp>

#include <stdexcept>
#include <utility>

namespace robotics::visualization {
void validate(const Recording &recording) {
    if (recording.data.clock.empty() || !recording.data.frames || recording.duration_ns < 0 ||
        recording.data.streams.size() > 64)
        throw std::invalid_argument(
            "invalid source identity, clock, frames, duration, or stream count");
    std::size_t total = 0;
    for (const auto &[stream, samples] : recording.data.streams) {
        if (stream.empty() || samples.size() > 10000)
            throw std::invalid_argument("stream requires a name and at most 10000 samples");
        Time previous = -1;
        for (const auto &sample : samples) {
            validate(sample.pose);
            if (sample.frame.empty() || sample.time_ns <= previous ||
                sample.time_ns > recording.duration_ns)
                throw std::invalid_argument("pose times must increase within recording duration");
            previous = sample.time_ns;
        }
        total += samples.size();
    }
    if (total > 100000)
        throw std::invalid_argument("pose histories exceed 100000 samples");
}
LocalSource::LocalSource(std::string id, Recording recording) : id_(std::move(id)) {
    if (id_.empty())
        throw std::invalid_argument("source ID must not be empty");
    validate(recording);
    duration_ns_ = recording.duration_ns;
    data_ = std::make_shared<const SourceData>(std::move(recording.data));
}
SourceSnapshot LocalSource::snapshot() const {
    return {id_, generation_, time_ns_, connected_ ? data_ : nullptr};
}
void LocalSource::disconnect() {
    if (connected_) {
        connected_ = false;
        ++generation_;
    }
}
void LocalSource::reconnect() {
    if (!connected_) {
        connected_ = true;
        time_ns_ = 0;
        ++generation_;
    }
}
Time LocalSource::duration() const {
    return duration_ns_;
}
void LocalSource::seek(Time time_ns) {
    if (!connected_)
        throw std::logic_error("source is disconnected");
    if (time_ns < 0 || time_ns > duration_ns_)
        throw std::out_of_range("seek outside recording duration");
    if (time_ns < time_ns_)
        ++generation_;
    time_ns_ = time_ns;
}
} // namespace robotics::visualization

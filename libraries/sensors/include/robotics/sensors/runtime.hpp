#pragma once

#include "robotics/sensors/types.hpp"
#include "robotics/simulation/plant.hpp"
#include <any>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace robotics::sensors {
namespace detail {
template <class Model> class ScheduledSensor;
}

// A value-observation/explicit-consumption handle. It never drives acquisition.
// Handles may outlive the runtime; active() becomes false on failure/destruction.
// Access is single-threaded, like Runtime. No blocking or middleware lives here.
template <class T> class SensorStream {
  public:
    SensorStream() = default;
    SensorStream(const SensorStream &) = delete;
    SensorStream &operator=(const SensorStream &) = delete;
    std::optional<Sample<T>> latest() const {
        return latest_;
    }
    StreamStats stats() const {
        return stats_;
    }
    bool active() const {
        return active_;
    }
    std::vector<Sample<T>> drain() {
        std::vector<Sample<T>> samples;
        samples.reserve(ready_.size());
        while (!ready_.empty()) {
            samples.push_back(std::move(ready_.front()));
            ready_.pop_front();
        }
        return samples;
    }

  private:
    template <class Model> friend class detail::ScheduledSensor;
    std::deque<Sample<T>> ready_;
    std::optional<Sample<T>> latest_;
    StreamStats stats_;
    bool active_ = false;
};

namespace detail {
// Type erasure only for scheduling/lifecycle; measurement types remain model-owned.
class ScheduledDevice {
  public:
    virtual ~ScheduledDevice() = default;
    virtual void advance(const simulation::MotionSample &) = 0;
    virtual void reset(std::uint64_t seed) = 0;
    virtual void invalidate() noexcept = 0;
    virtual void discardBuffered() noexcept = 0;
};

template <class Model> class ScheduledSensor final : public ScheduledDevice {
  public:
    using Reading = typename Model::Reading;
    ScheduledSensor(Device config, Model model, std::uint64_t seed)
        : config_(std::move(config)), model_(std::move(model)), stream_(std::make_shared<SensorStream<Reading>>()) {
        reset(seed);
    }
    ~ScheduledSensor() override {
        invalidate();
    }
    std::shared_ptr<SensorStream<Reading>> stream() const {
        return stream_;
    }

    void reset(std::uint64_t seed) override {
        invalidate();
        model_.reset(seed, config_.id);
        next_due_ = config_.period;
        last_acquired_ = Nanoseconds{0};
        stream_->stats_ = {};
        stream_->active_ = true;
    }
    void invalidate() noexcept override {
        pending_.clear();
        stream_->ready_.clear();
        stream_->latest_.reset();
        stream_->active_ = false;
    }
    void discardBuffered() noexcept override {
        stream_->stats_.dropped_pending += pending_.size();
        stream_->stats_.dropped_delivered += stream_->ready_.size();
        pending_.clear();
        stream_->ready_.clear();
        stream_->latest_.reset();
    }
    void advance(const simulation::MotionSample &motion) override {
        const auto now = motion.state.elapsed;
        deliver(now);
        if (!next_due_ || now < *next_due_) {
            return;
        }
        if (config_.latency.count() > std::numeric_limits<std::int64_t>::max() - now.count()) {
            throw std::overflow_error("sensor delivery time overflow: " + config_.id);
        }
        const auto elapsed = std::chrono::duration<double>(now - last_acquired_).count();
        auto measurement = model_.sample(motion, elapsed);
        if (measurement.value.has_value() == !measurement.unavailable_reason.empty()) {
            throw std::logic_error("sensor must return either a value or an unavailable reason: " + config_.id);
        }
        Sample<Reading> sample{{config_.id, config_.frame, motion.state.generation, stream_->stats_.acquired,
                                motion.state.tick, *next_due_, now, Nanoseconds{0}},
                               std::move(measurement)};
        ++stream_->stats_.acquired;
        if (!sample.measurement.value) {
            ++stream_->stats_.unavailable;
        }
        last_acquired_ = now;
        if (config_.period.count() > std::numeric_limits<std::int64_t>::max() - next_due_->count()) {
            next_due_.reset();
        } else {
            *next_due_ += config_.period;
        }
        makeRoom(pending_, stream_->stats_.dropped_pending);
        pending_.push_back({now + config_.latency, std::move(sample)});
        deliver(now);
    }

  private:
    struct Pending {
        Nanoseconds available;
        Sample<Reading> sample;
    };
    template <class Queue> void makeRoom(Queue &queue, std::uint64_t &dropped) {
        if (queue.size() == config_.capacity) {
            if (config_.overflow == OverflowPolicy::Fail) {
                throw std::runtime_error("sensor queue capacity exceeded: " + config_.id);
            }
            queue.pop_front();
            ++dropped;
        }
    }
    void deliver(Nanoseconds now) {
        while (!pending_.empty() && pending_.front().available <= now) {
            makeRoom(stream_->ready_, stream_->stats_.dropped_delivered);
            auto sample = std::move(pending_.front().sample);
            pending_.pop_front();
            sample.header.delivered = now;
            stream_->latest_ = sample;
            stream_->ready_.push_back(std::move(sample));
            ++stream_->stats_.delivered;
        }
    }
    Device config_;
    Model model_;
    std::shared_ptr<SensorStream<Reading>> stream_;
    std::deque<Pending> pending_;
    std::optional<Nanoseconds> next_due_;
    Nanoseconds last_acquired_{0};
};
} // namespace detail

// Composes the plant and scheduled devices. Register before first advancement.
// Model requirements: Reading type; reset(seed, id); sample(motion, elapsed_seconds).
class Runtime {
  public:
    Runtime(const simulation::PlantParameters &parameters, const simulation::BodyState &initial,
            std::uint64_t seed = 0);
    ~Runtime();
    Runtime(const Runtime &) = delete;
    Runtime &operator=(const Runtime &) = delete;

    template <class Model> std::shared_ptr<SensorStream<typename Model::Reading>> add(Device device, Model model) {
        validateDevice(device);
        auto entry = std::make_unique<detail::ScheduledSensor<Model>>(device, std::move(model), seed_);
        auto stream = entry->stream();
        // Roll back ID reservation if registration allocation fails.
        auto inserted = streams_.emplace(device.id, stream);
        try {
            devices_.push_back(std::move(entry));
        } catch (...) {
            streams_.erase(inserted.first);
            throw;
        }
        return stream;
    }
    template <class Reading> std::shared_ptr<SensorStream<Reading>> stream(const std::string &id) const {
        const auto found = streams_.find(id);
        if (found == streams_.end()) {
            throw std::invalid_argument("unknown sensor: " + id);
        }
        const auto *typed = std::any_cast<std::shared_ptr<SensorStream<Reading>>>(&found->second);
        if (!typed) {
            throw std::invalid_argument("sensor reading type mismatch: " + id);
        }
        return *typed;
    }
    void command(const Eigen::VectorXd &forces);
    void stopThrusters(); // Clears propulsion targets/queues without restarting sensors/time.
    simulation::Snapshot advance(std::uint64_t ticks = 1);
    simulation::Snapshot observe() const;
    // Keeps sensor phase/noise and time; discards readings from before placement.
    simulation::Snapshot place(const simulation::BodyState &state, bool clear_actuators = true);
    simulation::Snapshot reset(const simulation::BodyState &initial, std::uint64_t seed);
    void setContactResolver(std::shared_ptr<simulation::ContactResolver> resolver) {
        plant_.setContactResolver(std::move(resolver));
    }
    bool faulted() const {
        return faulted_;
    }

  private:
    void validateDevice(const Device &) const;
    void requireHealthy() const;
    simulation::Plant plant_;
    Nanoseconds timestep_;
    std::uint64_t seed_;
    std::map<std::string, std::any> streams_;
    std::vector<std::unique_ptr<detail::ScheduledDevice>> devices_;
    bool sealed_ = false, faulted_ = false;
};
} // namespace robotics::sensors

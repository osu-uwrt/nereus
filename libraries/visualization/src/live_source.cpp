#include <robotics/visualization/live_source.hpp>

#include <deque>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace robotics::visualization {
struct LivePoseSource::Impl {
    explicit Impl(LivePoseOptions configuration) : options(std::move(configuration)) {
        if (options.id.empty() || options.clock.empty() || options.world_frame.empty() ||
            options.body_frame.empty() || options.world_frame == options.body_frame ||
            options.stream.empty() || options.queue_capacity < 1 || options.queue_capacity > 4096 ||
            options.history_capacity < 1 || options.history_capacity > 10000)
            throw std::invalid_argument("invalid live source identity, frames, or capacities");
        pending.resize(options.queue_capacity);
        drained.reserve(options.queue_capacity);
    }
    LivePoseOptions options;
    std::mutex mutex;
    std::vector<PoseUpdate> pending; // Allocated once; producer never grows the queue.
    std::size_t head{0}, count{0};
    std::optional<PoseUpdate> latest;
    std::uint64_t generation{0};
    bool connected{true};
    SourceSnapshot::Delivery delivery;
    // Consumer-owned cache, never accessed by publish().
    std::optional<std::uint64_t> cached_generation;
    std::vector<PoseUpdate> drained;
    std::deque<PoseUpdate> history;
    std::shared_ptr<const SourceData> data;
    Time time_ns{0};

    void enqueue(const PoseUpdate &update) {
        if (count == pending.size()) {
            head = (head + 1) % pending.size();
            --count;
            ++delivery.dropped_queue;
        }
        pending[(head + count) % pending.size()] = update;
        ++count;
    }
};
LivePoseSource::LivePoseSource(LivePoseOptions options)
    : impl_(std::make_unique<Impl>(std::move(options))) {}
LivePoseSource::~LivePoseSource() = default;
void LivePoseSource::publish(const PoseUpdate &update) {
    validate(update.pose);
    if (update.time_ns < 0)
        throw std::invalid_argument("live pose time must be nonnegative");
    auto &state = *impl_;
    const std::lock_guard<std::mutex> lock(state.mutex);
    if (state.latest) {
        if (update.generation < state.latest->generation ||
            (update.generation == state.latest->generation &&
             update.time_ns <= state.latest->time_ns)) {
            ++state.delivery.rejected_stale;
            return;
        }
        if (update.generation != state.latest->generation) {
            ++state.generation;
            state.count = 0;
            state.head = 0;
        }
    }
    state.latest = update;
    if (state.connected)
        state.enqueue(update);
}
SourceSnapshot LivePoseSource::snapshot() const {
    auto &state = *impl_;
    auto &updates = state.drained;
    updates.clear();
    std::uint64_t generation = 0;
    SourceSnapshot::Delivery delivery;
    bool connected = false;
    {
        const std::lock_guard<std::mutex> lock(state.mutex);
        generation = state.generation;
        delivery = state.delivery;
        connected = state.connected;
        for (std::size_t i = 0; i < state.count; ++i)
            updates.push_back(state.pending[(state.head + i) % state.pending.size()]);
        state.count = 0;
        state.head = 0;
    }
    if (!state.cached_generation || *state.cached_generation != generation) {
        state.history.clear();
        state.data.reset();
        state.time_ns = 0;
        state.cached_generation = generation;
    }
    if (!connected)
        return {state.options.id, generation, state.time_ns, nullptr, delivery};
    if (!updates.empty()) {
        for (const auto &update : updates) {
            state.history.push_back(update);
            if (state.history.size() > state.options.history_capacity) {
                state.history.pop_front();
                ++delivery.trimmed_history;
            }
        }
        SourceData data;
        data.clock = state.options.clock;
        auto &poses = data.streams[state.options.stream];
        FrameEdge body{state.options.world_frame, state.options.body_frame, false, {}};
        poses.reserve(state.history.size());
        body.samples.reserve(state.history.size());
        for (const auto &update : state.history) {
            poses.push_back({update.time_ns, state.options.world_frame, update.pose});
            body.samples.push_back({update.time_ns, update.pose});
        }
        data.frames = std::make_shared<const FrameGraph>(state.options.world_frame,
                                                         std::vector<FrameEdge>{std::move(body)});
        state.data = std::make_shared<const SourceData>(std::move(data));
        state.time_ns = updates.back().time_ns;
        const std::lock_guard<std::mutex> lock(state.mutex);
        state.delivery.trimmed_history = delivery.trimmed_history;
    }
    return {state.options.id, generation, state.time_ns, state.data, delivery};
}
void LivePoseSource::disconnect() {
    auto &state = *impl_;
    const std::lock_guard<std::mutex> lock(state.mutex);
    if (state.connected) {
        state.connected = false;
        ++state.generation;
        state.count = 0;
    }
}
void LivePoseSource::reconnect() {
    auto &state = *impl_;
    const std::lock_guard<std::mutex> lock(state.mutex);
    if (!state.connected) {
        state.connected = true;
        ++state.generation;
        if (state.latest)
            state.enqueue(*state.latest);
    }
}
} // namespace robotics::visualization

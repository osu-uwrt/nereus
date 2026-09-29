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
            options.history_capacity < 1 || options.history_capacity > 10000 ||
            options.moving_frames.size() > 127 || options.fixed_frames.size() > 127 ||
            (options.moving_frames.size() + 1) * options.history_capacity +
                    options.fixed_frames.size() >
                100000)
            throw std::invalid_argument("invalid live source identity, frames, or capacities");
        std::vector<FrameEdge> edges{{options.world_frame, options.body_frame, false, {{0, {}}}}};
        for (const auto &frame : options.fixed_frames)
            edges.push_back({frame.parent, frame.child, true, {{0, frame.pose}}});
        for (const auto &frame : options.moving_frames)
            edges.push_back({frame.parent, frame.child, false, {{0, {}}}});
        (void)FrameGraph(options.world_frame,
                         std::move(edges)); // Validate before accepting updates.
        pending.resize(options.queue_capacity);
        drained.resize(options.queue_capacity);
        for (auto *buffer : {&pending, &drained})
            for (auto &packet : *buffer)
                packet.moving_poses.resize(options.moving_frames.size());
        latest.moving_poses.resize(options.moving_frames.size());
    }
    LivePoseOptions options;
    std::mutex mutex;
    std::vector<PoseUpdate> pending; // Allocated once; producer never grows the queue.
    std::size_t head{0}, count{0};
    PoseUpdate latest;
    bool has_latest{false};
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
const std::vector<MovingFrame> &LivePoseSource::movingFrames() const {
    return impl_->options.moving_frames;
}
void LivePoseSource::publish(const PoseUpdate &update) {
    validate(update.pose);
    if (update.time_ns < 0)
        throw std::invalid_argument("live pose time must be nonnegative");
    auto &state = *impl_;
    if (update.moving_poses.size() != state.options.moving_frames.size())
        throw std::invalid_argument("moving-frame batch does not match source topology");
    for (const auto &pose : update.moving_poses)
        validate(pose);
    const std::lock_guard<std::mutex> lock(state.mutex);
    if (state.has_latest) {
        if (update.generation < state.latest.generation ||
            (update.generation == state.latest.generation &&
             update.time_ns <= state.latest.time_ns)) {
            ++state.delivery.rejected_stale;
            return;
        }
        if (update.generation != state.latest.generation) {
            ++state.generation;
            state.count = 0;
            state.head = 0;
        }
    }
    state.latest = update;
    state.has_latest = true;
    if (state.connected)
        state.enqueue(update);
}
SourceSnapshot LivePoseSource::snapshot() const {
    auto &state = *impl_;
    auto &updates = state.drained;
    std::size_t update_count = 0;
    std::uint64_t generation = 0;
    SourceSnapshot::Delivery delivery;
    bool connected = false;
    {
        const std::lock_guard<std::mutex> lock(state.mutex);
        generation = state.generation;
        delivery = state.delivery;
        connected = state.connected;
        update_count = state.count;
        for (std::size_t i = 0; i < update_count; ++i)
            updates[i] = state.pending[(state.head + i) % state.pending.size()];
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
    if (update_count) {
        for (std::size_t i = 0; i < update_count; ++i) {
            state.history.push_back(updates[i]);
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
        std::vector<FrameEdge> edges{std::move(body)};
        for (const auto &frame : state.options.fixed_frames)
            edges.push_back({frame.parent, frame.child, true, {{0, frame.pose}}});
        for (std::size_t i = 0; i < state.options.moving_frames.size(); ++i) {
            const auto &frame = state.options.moving_frames[i];
            FrameEdge edge{frame.parent, frame.child, false, {}};
            edge.samples.reserve(state.history.size());
            for (const auto &update : state.history)
                edge.samples.push_back({update.time_ns, update.moving_poses[i]});
            edges.push_back(std::move(edge));
        }
        data.frames =
            std::make_shared<const FrameGraph>(state.options.world_frame, std::move(edges));
        state.data = std::make_shared<const SourceData>(std::move(data));
        state.time_ns = updates[update_count - 1].time_ns;
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
        if (state.has_latest)
            state.enqueue(state.latest);
    }
}
} // namespace robotics::visualization

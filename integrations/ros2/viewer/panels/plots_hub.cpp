// The plots' node and thread: generic subscriptions shared by every series of a topic, decoded once per message.
#include "plots_hub.hpp"
#include <chrono>
#include <cmath>
#include <deque>

namespace nereus::ros_viewer::plots {
namespace {

double steadySeconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Figures are sampled at up to 20 Hz; ten minutes of them.
constexpr std::size_t kFigureCapacity = 12000;
// How long the hub keeps a source nobody holds (a layout reload in between keeps its history).
constexpr double kKeepUnused = 10;
// How often the graph's topic list is refreshed.
constexpr double kGraphPeriod = 2;

} // namespace

struct Hub::Topic {
    std::string name, type;
    std::shared_ptr<const MessageType> messageType;
    std::string error;
    rclcpp::GenericSubscription::SharedPtr subscription;

    std::mutex mutex; // guards everything below (the ROS thread decodes, the UI adds readers)
    struct Reader {
        std::string field;
        std::unique_ptr<FieldReader> reader;
        std::weak_ptr<Channel> channel;
    };
    std::vector<Reader> readers;
    std::unique_ptr<Message> scratch;
    std::shared_ptr<rclcpp::SerializedMessage> latest;
    std::deque<double> arrivals; // steady seconds of the messages of the last 2 s
    double watchedUntil = 0;     // steady seconds
};

Hub::Hub(const std::string &robotNamespace, bool useSimTime)
    : namespace_(robotNamespace),
      node_(std::make_shared<rclcpp::Node>("viewer_plots", "/" + robotNamespace,
                                           rclcpp::NodeOptions().use_global_arguments(false).parameter_overrides(
                                               {rclcpp::Parameter("use_sim_time", useSimTime)}))) {
    executor_.add_node(node_);
    worker_ = std::thread([this] { executor_.spin(); });
}

Hub::~Hub() {
    executor_.cancel();
    if (worker_.joinable())
        worker_.join();
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto &[name, topic] : topics_)
        topic->subscription.reset();
}

double Hub::now() const {
    return node_->get_clock()->now().seconds();
}

std::string Hub::resolve(const std::string &topic) const {
    if (topic.empty() || topic[0] == '/')
        return topic;
    return namespace_.empty() ? "/" + topic : "/" + namespace_ + "/" + topic;
}

std::string Hub::relative(const std::string &absolute) const {
    const std::string prefix = "/" + namespace_ + "/";
    if (!namespace_.empty() && absolute.compare(0, prefix.size(), prefix) == 0)
        return absolute.substr(prefix.size());
    return absolute;
}

std::shared_ptr<Hub::Topic> Hub::topic(const std::string &absolute, bool create) {
    // mutex_ held by the caller
    if (const auto found = topics_.find(absolute); found != topics_.end())
        return found->second;
    if (!create)
        return nullptr;
    auto t = std::make_shared<Topic>();
    t->name = absolute;
    for (const auto &info : graph_)
        if (info.name == absolute)
            t->type = info.type;
    if (t->type.empty()) // not in the last listing: ask the graph now
        for (const auto &[name, types] : node_->get_topic_names_and_types())
            if (name == absolute && !types.empty())
                t->type = types.front();
    topics_[absolute] = t;
    subscribe(t);
    return t;
}

// Subscribes once the type is known and installed, and compiles the readers waiting for it.
void Hub::subscribe(const std::shared_ptr<Topic> &t) {
    if (t->subscription || t->type.empty())
        return;
    try {
        t->messageType = MessageType::get(t->type);
    } catch (const std::exception &error) {
        t->error = error.what();
        std::lock_guard<std::mutex> lock(t->mutex);
        for (auto &r : t->readers)
            if (auto channel = r.channel.lock()) {
                std::lock_guard<std::mutex> clock(channel->mutex);
                channel->error = t->error;
            }
        return;
    }
    {
        std::lock_guard<std::mutex> lock(t->mutex);
        t->scratch = std::make_unique<Message>(t->messageType);
        for (auto &r : t->readers)
            if (!r.reader)
                if (auto channel = r.channel.lock()) {
                    try {
                        r.reader = std::make_unique<FieldReader>(*t->messageType, r.field);
                    } catch (const std::exception &error) {
                        std::lock_guard<std::mutex> clock(channel->mutex);
                        channel->error = error.what();
                    }
                }
    }
    // Best effort with a modest queue: it hears reliable and best-effort publishers alike, and a slow UI never
    // backs up the publisher.
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(50)).best_effort().durability_volatile();
    std::weak_ptr<Topic> weak = t;
    try {
        t->subscription = node_->create_generic_subscription(
            t->name, t->type, qos, [this, weak](std::shared_ptr<rclcpp::SerializedMessage> message) {
                if (auto topic = weak.lock())
                    receive(topic, std::move(message));
            });
    } catch (const std::exception &error) {
        t->error = error.what();
    }
}

void Hub::receive(const std::shared_ptr<Topic> &t, std::shared_ptr<rclcpp::SerializedMessage> message) {
    const double receipt = now(), steady = steadySeconds();
    std::lock_guard<std::mutex> lock(t->mutex);
    t->arrivals.push_back(steady);
    while (!t->arrivals.empty() && steady - t->arrivals.front() > 2)
        t->arrivals.pop_front();
    t->latest = message;
    if (t->readers.empty() || !t->scratch || !t->scratch->deserialize(*message))
        return;
    const auto stamp = headerStamp(*t->messageType, t->scratch->data());
    for (auto &r : t->readers) {
        if (!r.reader)
            continue;
        auto channel = r.channel.lock();
        if (!channel)
            continue;
        const double value = r.reader->read(t->scratch->data());
        std::lock_guard<std::mutex> clock(channel->mutex);
        channel->samples.push({stamp.value_or(receipt), receipt, float(value)});
        channel->stamped = stamp.has_value();
        if (stamp) {
            const double offset = receipt - *stamp;
            channel->offset = channel->hasOffset ? channel->offset * .95 + offset * .05 : offset;
            channel->hasOffset = true;
        }
    }
}

std::shared_ptr<Channel> Hub::channel(const Source &source) {
    const std::string key = source.key();
    std::lock_guard<std::mutex> lock(mutex_);
    if (const auto found = channels_.find(key); found != channels_.end()) {
        found->second.lastUsed = steadySeconds();
        return found->second.channel;
    }
    auto channel = std::make_shared<Channel>();
    channels_[key] = {channel, steadySeconds()};
    if (source.kind == Source::Kind::Figure) {
        channel->samples = SeriesBuffer(kFigureCapacity);
        return channel;
    }
    const std::string absolute = resolve(source.topic);
    auto t = topic(absolute, true);
    {
        std::lock_guard<std::mutex> tlock(t->mutex);
        Topic::Reader reader{source.field, nullptr, channel};
        if (t->messageType) {
            try {
                reader.reader = std::make_unique<FieldReader>(*t->messageType, source.field);
            } catch (const std::exception &error) {
                channel->error = error.what();
            }
        } else if (!t->error.empty())
            channel->error = t->error;
        t->readers.push_back(std::move(reader));
    }
    return channel;
}

void Hub::pushFigure(const std::string &figure, double value) {
    Source source;
    source.kind = Source::Kind::Figure;
    source.figure = figure;
    const auto channel = this->channel(source);
    const double t = now();
    std::lock_guard<std::mutex> lock(channel->mutex);
    channel->samples.push({t, t, float(value)});
}

std::vector<TopicInfo> Hub::topics() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return graph_;
}

void Hub::watch(const std::string &absolute) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto t = topic(absolute, true);
    std::lock_guard<std::mutex> tlock(t->mutex);
    t->watchedUntil = steadySeconds() + 1;
}

std::shared_ptr<const rclcpp::SerializedMessage> Hub::latest(const std::string &absolute) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = topics_.find(absolute);
    if (found == topics_.end())
        return nullptr;
    std::lock_guard<std::mutex> tlock(found->second->mutex);
    return found->second->latest;
}

double Hub::rate(const std::string &absolute) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = topics_.find(absolute);
    if (found == topics_.end())
        return 0;
    std::lock_guard<std::mutex> tlock(found->second->mutex);
    const auto &a = found->second->arrivals;
    if (a.size() < 2 || steadySeconds() - a.back() > 2)
        return 0;
    return double(a.size() - 1) / std::max(a.back() - a.front(), 1e-6);
}

std::string Hub::topicError(const std::string &absolute) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = topics_.find(absolute);
    return found == topics_.end() ? std::string() : found->second->error;
}

std::string Hub::topicType(const std::string &absolute) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (const auto found = topics_.find(absolute); found != topics_.end() && !found->second->type.empty())
        return found->second->type;
    for (const auto &info : graph_)
        if (info.name == absolute)
            return info.type;
    return {};
}

std::size_t Hub::publishers(const std::string &absolute) const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto &info : graph_)
        if (info.name == absolute)
            return info.publishers;
    return 0;
}

void Hub::maintain() {
    const double steady = steadySeconds(), current = now();

    // Time going back more than a second (a looped bag, a restarted clock) starts every plot over.
    if (current + 1 < lastNow_) {
        lastJump_ = lastNow_ - current;
        ++jumps_;
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto &[key, held] : channels_) {
            std::lock_guard<std::mutex> clock(held.channel->mutex);
            held.channel->samples.clear();
            held.channel->hasOffset = false;
        }
    }
    lastNow_ = current;

    std::lock_guard<std::mutex> lock(mutex_);
    if (steady - graphAt_ > kGraphPeriod) {
        graphAt_ = steady;
        graph_.clear();
        for (const auto &[name, types] : node_->get_topic_names_and_types())
            if (!types.empty())
                graph_.push_back({name, types.front(), node_->count_publishers(name)});
        for (auto &[name, t] : topics_)
            if (t->type.empty())
                for (const auto &info : graph_)
                    if (info.name == name) {
                        t->type = info.type;
                        subscribe(t);
                    }
    }

    // Sources nobody has held for a while go; figures stay (the panels' hover trends read them).
    for (auto it = channels_.begin(); it != channels_.end();) {
        if (it->second.channel.use_count() > 1)
            it->second.lastUsed = steady;
        const bool figure = it->first.rfind("figure:", 0) == 0;
        if (!figure && steady - it->second.lastUsed > kKeepUnused)
            it = channels_.erase(it);
        else
            ++it;
    }
    // Topics with no live reader and no browser watching are unsubscribed.
    for (auto it = topics_.begin(); it != topics_.end();) {
        auto &t = *it->second;
        bool keep;
        {
            std::lock_guard<std::mutex> tlock(t.mutex);
            t.readers.erase(std::remove_if(t.readers.begin(), t.readers.end(),
                                           [](const Topic::Reader &r) { return r.channel.expired(); }),
                            t.readers.end());
            keep = !t.readers.empty() || t.watchedUntil > steady;
        }
        if (keep)
            ++it;
        else
            it = topics_.erase(it);
    }
}

} // namespace nereus::ros_viewer::plots

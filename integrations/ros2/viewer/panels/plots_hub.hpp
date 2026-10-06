// The plots' ROS side: one node ("viewer_plots") spun on its own thread, so decoding a busy topic never delays the
// panels' node (the motion heartbeat and kill watchdogs). Topics are subscribed generically while a plot or the
// Topics browser uses them; each requested field fills a Channel (a ring buffer of samples) that the UI reads.
#pragma once
#include "nereus/ros_viewer/plots/model.hpp"
#include "nereus/ros_viewer/plots/series.hpp"
#include "plots_fields.hpp"
#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <vector>

namespace nereus::ros_viewer::plots {

// One source's samples and state, written by the ROS thread (or the UI, for figures) and read by the UI. Lock
// `mutex` around every access.
struct Channel {
    mutable std::mutex mutex;
    SeriesBuffer samples;
    std::string error;    // why the source cannot be read (type missing, bad field); empty when fine
    bool stamped = false; // samples carry header stamps
    double offset = 0;    // smoothed receipt minus stamp (seconds), for header-stamped topics
    bool hasOffset = false;
};

// A topic of the ROS graph as the browser lists it.
struct TopicInfo {
    std::string name, type; // absolute name, "pkg/msg/Name"
    std::size_t publishers = 0;
};

class Hub {
  public:
    // The node lives in the robot namespace and follows use_sim_time; the thread starts at once.
    Hub(const std::string &robotNamespace, bool useSimTime);
    ~Hub();
    Hub(const Hub &) = delete;
    Hub &operator=(const Hub &) = delete;

    // Seconds on the node's clock (sim time under the simulator).
    double now() const;
    // A topic as written in a plot (relative to the robot namespace, or absolute) to its absolute name, and back.
    std::string resolve(const std::string &topic) const;
    std::string relative(const std::string &absolute) const;

    // The channel of a source, subscribing to its topic as needed. The hub keeps it while anyone else holds it, and
    // for a while after (so reloading a layout keeps the history).
    std::shared_ptr<Channel> channel(const Source &);
    // A figure sample from the UI thread, stamped now(); creates the figure's channel on first use. NaN: a gap.
    void pushFigure(const std::string &figure, double value);

    // The graph's topics, refreshed by maintain() every few seconds.
    std::vector<TopicInfo> topics() const;
    // Keeps a topic subscribed while the browser shows it open (call every frame it is; lapses after a second).
    void watch(const std::string &absolute);
    // The newest message of a watched or plotted topic, its rate (Hz over the last 2 s) and any error.
    std::shared_ptr<const rclcpp::SerializedMessage> latest(const std::string &absolute) const;
    double rate(const std::string &absolute) const;
    std::string topicError(const std::string &absolute) const;
    std::string topicType(const std::string &absolute) const;
    std::size_t publishers(const std::string &absolute) const;

    // Once per UI frame: refreshes the graph, drops sources nobody used for a while, and notices time jumping
    // back (a looped bag, a clock reset), which clears every channel.
    void maintain();
    // How far time last jumped back (seconds) and a counter bumped each time, for the plots' notice.
    double lastJump() const {
        return lastJump_;
    }
    std::uint64_t jumps() const {
        return jumps_;
    }

  private:
    struct Topic;
    std::shared_ptr<Topic> topic(const std::string &absolute, bool create);
    void subscribe(const std::shared_ptr<Topic> &);
    void receive(const std::shared_ptr<Topic> &, std::shared_ptr<rclcpp::SerializedMessage>);

    std::string namespace_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread worker_;

    mutable std::mutex mutex_; // guards the maps and the graph list below
    std::map<std::string, std::shared_ptr<Topic>> topics_;
    struct Held {
        std::shared_ptr<Channel> channel;
        double lastUsed = 0; // steady seconds
    };
    std::map<std::string, Held> channels_; // by Source::key()
    std::vector<TopicInfo> graph_;
    double graphAt_ = -1e9, lastNow_ = 0;
    double lastJump_ = 0;
    std::uint64_t jumps_ = 0;
};

} // namespace nereus::ros_viewer::plots

#pragma once
// Bridge semantics without middleware: one owner steps the session and maps data both ways
// (port of core.py). Everything here is deterministic in integer simulation ticks and fully
// validated against the ROS types and native endpoints before the first step. Unknown
// endpoints and unsupported actions fail at construction. Not thread safe except for
// realTimeFactor(): the node serializes every other call on its stepping thread.
#include "camera_sink.hpp"
#include "mapping.hpp"
#include "session_port.hpp"

#include <robotics/session/scenario.hpp>

#include <atomic>
#include <functional>
#include <memory>
#include <optional>

namespace robotics::ros_bridge {

struct VisualContext;

struct Publication {
    std::string stream;
    std::shared_ptr<Message> message;
};

struct Transform {
    std::string parent, child;
    std::int64_t stamp_ns{0};
    Eigen::Vector3d translation;
    Eigen::Quaterniond orientation; // wxyz semantics of Eigen
};

// A request the node sends to the external estimator's pose service.
struct Alignment {
    std::string trigger, frame;
    std::int64_t stamp_ns{0};
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    double covariance_diagonal{0};
};

using CounterTable = std::map<std::string, std::uint64_t>;
struct Counters {
    CounterTable published, unavailable_samples, rejected_commands, filtered_messages,
        service_calls, alignments, alignments_superseded, alignments_acknowledged,
        alignments_failed;
    static void bump(CounterTable &table, const std::string &key) { ++table[key]; }
    Json toJson() const;
};

struct StepOutput {
    std::vector<std::int64_t> clocks;
    std::vector<Publication> publications;
    std::vector<Transform> transforms;
};

// (target frame, source frame) -> target_T_source, nullopt when unknown.
using Lookup = std::function<std::optional<spatial::Pose>(const std::string &, const std::string &)>;

// Service types the node can serve (no generic service server exists in Humble).
const std::vector<std::string> &supportedServiceTypes();

class BridgeCore {
  public:
    // `resolved` and `session` must outlive the core. Throws BridgeError.
    BridgeCore(const session::ResolvedScenario &resolved, SessionPort &session,
               std::int64_t epoch_ns, Lookup lookup = {}, CameraSink *cameras = nullptr);

    const Json &config() const { return config_; }
    const session::ResolvedScenario &resolved() const { return resolved_; }
    std::int64_t timestepNs() const { return timestep_ns_; }
    std::int64_t epochNs() const { return epoch_ns_; }
    const std::string &resetPolicy() const { return reset_policy_; }
    const std::string &worldFrame() const { return world_frame_; }
    double realTimeFactor() const { return real_time_factor_.load(); }
    // Returns the rejection reason or nullopt (0 pauses).
    std::optional<std::string> setRealTimeFactor(double value);
    static std::optional<std::string> checkRealTimeFactor(double value);
    void setLookup(Lookup lookup) { lookup_ = std::move(lookup); }
    bool killed() const { return session_.killed(); }
    const Counters &counters() const { return counters_; }
    Counters &counters() { return counters_; }
    const Json &taskEvents() const { return task_events_; }
    SessionPort &session() { return session_; }

    // Streams with their message types (publish and subscribe) for the node.
    const std::map<std::string, std::shared_ptr<const MessageType>> &streamTypes() const {
        return stream_types_;
    }
    struct ServiceEntry {
        std::shared_ptr<const ServiceType> type;
        Reader reader;
        Writer writer;
        Json options;
    };
    const std::map<std::string, ServiceEntry> &services() const { return services_; }
    const std::vector<Transform> &staticTransforms() const { return static_transforms_; }

    // ---- time
    std::int64_t rosNs(std::int64_t native_ns) const { return epoch_ns_ + offset_ns_ + native_ns; }
    std::int64_t clockNs() const { return last_ros_ns_; }

    // ---- stepping: exactly one tick; clock stamps first, then data, in send order
    StepOutput step();
    std::vector<Publication> flush();    // events produced outside stepping
    std::vector<Publication> refresh();  // viewer state while paused
    std::vector<Publication> startupPublications();
    std::string scenarioJson() const;

    // ---- inbound (`message` is a deserialized message of the stream's type)
    std::vector<Publication> receive(const std::string &stream, const void *message);
    session::CommandResult runCommand(const std::string &text);
    // Serves one request; `response` is an initialised response message.
    void call(const std::string &service, const void *request, void *response);
    std::optional<Alignment> pendingAlignment();
    bool hasAlignment() const { return alignment_.has_value(); }
    const Json &alignmentConfig() const { return *alignment_; }
    std::pair<bool, std::string> fullReset();

    spatial::Pose referencePose(const simulation::BodyState &body) const;

    ~BridgeCore();

  private:
    struct Timed {
        std::int64_t period_ns, next_ns;
    };
    struct PublisherEntry {
        std::shared_ptr<const MessageType> type;
        std::function<std::shared_ptr<Message>(const Value &)> encode;
        std::function<std::shared_ptr<Message>()> state; // timed format streams
    };

    std::pair<std::string, std::string> mechanism(const std::string &endpoint,
                                                  const std::string &where) const;
    SpecTree mechanismSpec() const;
    SpecTree clawSpec() const;
    void compileFormatStream(const Json &stream, const std::shared_ptr<const MessageType> &type,
                             const std::string &where);
    std::int64_t steppedPeriod(double rate_hz, const std::string &where) const;
    std::vector<std::size_t> thrusterPermutation();
    Json sensorJson(const std::string &name, const std::string &where) const;
    void compileStreams();
    void compileServices();
    void compileTf();
    void compileStaticTf();
    void compileAlignment();
    void checkBindings();
    void observeGeneration(const simulation::Snapshot &snapshot, bool coordinated = false);
    Publication publish(const std::string &stream, const Value &values);
    Publication publishMessage(const std::string &stream, std::shared_ptr<Message> message);
    Value robotState(const simulation::Snapshot &snapshot) const;
    Value thrusterValues() const;
    Value clawValues() const;
    Value mechanismValues() const;
    Publication timedState(const std::string &stream);
    void command(const std::string &stream, const std::vector<double> &forces);
    std::vector<Publication> setKilled(bool killed);
    session::CommandResult mechanismAction(const std::string &action, const Value &arguments);
    std::pair<bool, std::string> placeReference(const Value &pose, const Json &options);
    std::pair<bool, std::string> placeState(const simulation::BodyState &state, bool keep_velocity,
                                            bool becomes_start);
    std::optional<spatial::Pose> transform(const std::string &target, const std::string &source) const;
    void requestAlignment(const std::string &trigger);
    bool hasEstimateStream() const;

    const session::ResolvedScenario &resolved_;
    SessionPort &session_;
    Json config_;
    CameraSink *cameras_;
    Lookup lookup_;
    Counters counters_;
    std::int64_t timestep_ns_;
    std::atomic<double> real_time_factor_;
    std::int64_t epoch_ns_;
    std::string reset_policy_, world_frame_, reference_frame_;
    Timed clock_{0, 0};
    spatial::Pose root_to_reference_;
    std::uint64_t generation_{0};
    std::int64_t offset_ns_{0}, last_ros_ns_{0};
    simulation::BodyState start_state_;
    bool kill_stops_thrusters_{false};
    std::string commands_while_killed_;
    std::optional<std::vector<std::size_t>> thruster_index_;
    std::map<std::string, std::string> mechanism_types_;
    Json task_events_ = Json::array();

    std::map<std::string, PublisherEntry> publishers_;
    std::map<std::string, Reader> readers_;
    std::map<std::string, Json> stream_config_;
    std::map<std::string, std::shared_ptr<const MessageType>> stream_types_;
    std::map<std::string, std::vector<std::string>> sensor_streams_;
    std::vector<std::pair<std::string, Timed>> timed_;
    std::map<std::string, std::vector<std::string>> events_;
    std::vector<std::string> feed_streams_, startup_streams_;
    std::map<std::string, ServiceEntry> services_;
    std::vector<std::pair<Json, std::int64_t>> tf_publish_; // entry, period
    std::vector<Timed> tf_timed_;
    std::vector<Transform> static_transforms_;
    std::optional<Json> alignment_;
    std::optional<std::string> alignment_pending_;
    std::optional<Value> latest_estimate_;
    std::map<std::string, std::string> sensor_types_;
    std::unique_ptr<VisualContext> visual_;
};

// Typed native reading tree of one robot sensor (mapping of core.reading_spec).
SpecTree readingSpec(const Json &sensor);

} // namespace robotics::ros_bridge

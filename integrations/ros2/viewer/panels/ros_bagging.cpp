#include "ros_runtime.hpp"
#include <algorithm>
#include <condition_variable>
#include <deque>
#include <set>
#include <sstream>

namespace nereus::ros_viewer::panels {
namespace {
// ros2 bag record on each target through bag_recorder's scripts: one worker thread per target runs them (bash
// locally, ssh to the robot), polling the target's status between commands. The topic list is the viewer node's
// own view of the graph.
class RosBagging final : public Bagging {
    enum class Op { Status, Start, Stop, Kill };
    struct Command {
        Op op = Op::Status;
        BagRequest request;
    };
    struct Target {
        BagHost host;
        bool stopOnExit = false;
        BagTargetState value;
        std::deque<Command> commands;
        Steady::time_point nextPoll{}, polledAt{};
        uint64_t generation = 0; // bumped by setHost: replies from the previous host are dropped
        std::thread worker;
    };

  public:
    RosBagging(std::shared_ptr<RosRuntime> runtime, const YAML::Node &cfg, const Context &ctx)
        : runtime(runtime), stopWait(cfg["stop_timeout"].as<double>(10)),
          requestTimeout(cfg["request_timeout"].as<double>(30)) {
        for (const auto &entry : cfg["targets"]) {
            auto target = std::make_unique<Target>();
            target->value.id = entry["id"].as<std::string>();
            target->value.label = entry["label"].as<std::string>(target->value.id);
            target->host.host = entry["host"].as<std::string>("");
            target->host.directory = entry["directory"].as<std::string>("~/bags");
            target->host.setup = entry["setup"].as<std::string>("");
            if (entry["ssh"])
                target->host.ssh = entry["ssh"].as<std::vector<std::string>>();
            if (entry["record_args"])
                target->host.recordArgs = entry["record_args"].as<std::vector<std::string>>();
            target->stopOnExit = entry["stop_on_exit"].as<bool>(target->host.host.empty());
            target->value.host = target->host.host;
            target->value.directory = target->host.directory;
            target->value.message = target->host.host.empty() ? "Checking..." : "Connecting to " + target->host.host;
            targets.push_back(std::move(target));
        }
        // Preset topics as RViz's rosbag presets: relative names under the robot namespace.
        for (const auto &entry : cfg["presets"]) {
            BagPreset preset{entry.first.as<std::string>(), {}};
            for (const auto &topic : entry.second) {
                const auto name = expand(topic.as<std::string>(), ctx);
                preset.topics.push_back(!name.empty() && name[0] == '/' ? name : "/" + ctx.robotNamespace + "/" + name);
            }
            presets.push_back(std::move(preset));
        }
        for (auto &target : targets)
            target->worker = std::thread([this, t = target.get()] { run(*t); });
        timer = runtime->node->create_wall_timer(std::chrono::seconds(2), [this] { listTopics(); });
        listTopics();
    }
    ~RosBagging() override {
        timer->cancel();
        {
            std::lock_guard<std::mutex> lock(mutex);
            for (auto &target : targets) { // a bag on this computer ends with the viewer unless configured otherwise
                target->commands.clear();
                if (target->stopOnExit && (target->value.recording || target->value.stopping))
                    target->commands.push_back({Op::Stop, {}});
            }
            closing = true;
        }
        wake.notify_all();
        for (auto &target : targets)
            if (target->worker.joinable())
                target->worker.join();
    }
    BaggingState state() override {
        std::lock_guard<std::mutex> lock(mutex);
        BaggingState value;
        const auto now = Steady::now();
        for (const auto &target : targets) {
            value.targets.push_back(target->value);
            if (target->value.recording || target->value.stopping)
                value.targets.back().elapsed += std::chrono::duration<double>(now - target->polledAt).count();
        }
        value.topics = topics;
        value.presets = presets;
        return value;
    }
    void start(const std::string &id, const BagRequest &request) override {
        std::lock_guard<std::mutex> lock(mutex);
        auto *target = find(id);
        if (!target || target->value.pending || target->value.recording || target->value.stopping ||
            !validBagName(request.name) || (!request.all && request.topics.empty()))
            return;
        queue(*target, {Op::Start, request}, "Starting " + request.name + "...");
    }
    void stop(const std::string &id) override {
        std::lock_guard<std::mutex> lock(mutex);
        auto *target = find(id);
        if (target && !target->value.pending && (target->value.recording || target->value.stopping))
            queue(*target, {Op::Stop, {}}, "Stopping (closing the bag)...");
    }
    void kill(const std::string &id) override {
        std::lock_guard<std::mutex> lock(mutex);
        auto *target = find(id);
        if (target && target->value.stopping && !target->value.pending)
            queue(*target, {Op::Kill, {}}, "Killing the recorder...");
    }

    void setHost(const std::string &id, const std::string &host) override {
        std::lock_guard<std::mutex> lock(mutex);
        auto *target = find(id);
        if (!target || target->host.host.empty() || host.empty() || host == target->host.host ||
            target->value.pending || target->value.recording || target->value.stopping)
            return;
        ++target->generation;
        target->host.host = host;
        auto &v = target->value;
        v.host = host;
        v.known = v.reachable = false;
        v.bag.clear();
        v.lastBag.clear();
        v.lastNote.clear();
        v.bytes = v.lastBytes = v.elapsed = 0;
        v.freeBytes = -1;
        v.lastComplete = true;
        v.message = "Connecting to " + host;
        v.failed = false;
        target->nextPoll = Steady::now();
        wake.notify_all();
    }

  private:
    Target *find(const std::string &id) {
        for (auto &target : targets)
            if (target->value.id == id)
                return target.get();
        return nullptr;
    }
    void queue(Target &target, Command command, const std::string &message) {
        target.value.pending = true;
        target.value.message = message;
        target.value.failed = false;
        target.commands.push_back(std::move(command));
        wake.notify_all();
    }
    void listTopics() {
        std::vector<std::pair<std::string, std::string>> found;
        for (const auto &[name, types] : runtime->node->get_topic_names_and_types())
            found.emplace_back(name, types.empty() ? "" : types.front());
        std::sort(found.begin(), found.end());
        std::lock_guard<std::mutex> lock(mutex);
        topics = std::move(found);
    }
    void run(Target &target) {
        std::unique_lock<std::mutex> lock(mutex);
        for (;;) {
            wake.wait_until(lock, target.nextPoll, [&] { return closing || !target.commands.empty(); });
            if (closing && target.commands.empty())
                return;
            Command command;
            if (!target.commands.empty()) {
                command = std::move(target.commands.front());
                target.commands.pop_front();
            }
            BagHost host = target.host;
            const auto generation = target.generation;
            if (!command.request.directory.empty())
                host.directory = command.request.directory;
            lock.unlock();
            std::string script;
            double timeout = requestTimeout;
            switch (command.op) {
            case Op::Status:
                script = bagStatusScript(host);
                timeout = std::min(timeout, 15.);
                break;
            case Op::Start:
                script = bagStartScript(host, command.request);
                break;
            case Op::Stop:
                script = bagStopScript(host, stopWait);
                timeout += stopWait;
                break;
            case Op::Kill:
                script = bagStopScript(host, stopWait, true);
                break;
            }
            // Closing cuts a command short, except the closing stop of this computer's bag, queued for after it.
            const auto result = runProcess(bagCommand(host, script), timeout, closing ? nullptr : &closing);
            lock.lock();
            if (generation != target.generation)
                continue; // the target moved to another host meanwhile; poll that one
            apply(target, command.op, result);
            const bool busy = target.value.recording || target.value.stopping;
            target.nextPoll = Steady::now() + std::chrono::seconds(busy ? 2 : target.value.reachable ? 5 : 10);
        }
    }
    void apply(Target &target, Op op, const ProcessResult &result) {
        auto &v = target.value;
        const auto report = parseBagReport(result.output);
        const auto state = report.get("state");
        v.pending = !target.commands.empty();
        v.known = true;
        if (state.empty()) { // never got to run the script: ssh could not connect, or no answer in time
            v.reachable = false;
            std::string reason = result.timedOut ? "no answer in time" : lastLine(result.output);
            if (reason.empty())
                reason = "exit status " + std::to_string(result.status);
            v.message =
                (target.host.host.empty() ? "Could not run bash: " : "Can't reach " + target.host.host + ": ") + reason;
            v.failed = true;
            return;
        }
        const bool wasRecording = v.recording || v.stopping;
        const bool wasUnreachable = !v.reachable;
        v.reachable = true;
        v.recording = state == "recording";
        v.stopping = state == "stopping";
        if (v.recording || v.stopping) {
            v.bag = report.get("bag");
            v.elapsed = report.number("now") - report.number("start");
            v.bytes = report.number("bytes");
            target.polledAt = Steady::now();
        }
        v.freeBytes = report.number("free", -1);
        v.lastBag = report.get("last_bag");
        v.lastNote = report.get("last_note");
        v.lastBytes = report.number("last_bytes");
        v.lastComplete = report.has("last_complete");
        const auto saved = [&] {
            return "Saved " + v.lastBag + " (" + byteSize(v.lastBytes) + ")" +
                   (v.lastComplete ? "" : ", without metadata.yaml: ros2 bag reindex can rebuild it");
        };
        if (state == "error") {
            v.message = report.get("error");
            v.failed = true;
        } else if (state == "died") {
            v.message = "The recorder exited on its own: " + v.lastNote;
            v.failed = true;
        } else if (state == "stopped") {
            v.message = v.lastNote.empty() ? saved() : "Stopped: " + v.lastNote;
            v.failed = !v.lastNote.empty();
        } else if (op == Op::Start && v.recording) {
            v.message = "Recording";
            v.failed = false;
        } else if (op == Op::Stop && v.stopping) {
            v.message = "Still closing the bag...";
            v.failed = false;
        } else if (state == "idle" && wasRecording) { // ended between polls: stopped elsewhere
            v.message = v.lastBag.empty() ? "Stopped" : saved();
            v.failed = false;
        } else if (op == Op::Status &&
                   (wasUnreachable || v.message.rfind("Checking", 0) == 0 || v.message.rfind("Connecting", 0) == 0)) {
            v.message.clear();
            v.failed = false;
        }
    }
    static std::string lastLine(const std::string &text) {
        std::string line, last;
        std::istringstream lines(text);
        while (std::getline(lines, line))
            if (line.find_first_not_of(" \t\r") != std::string::npos)
                last = line;
        return last;
    }
    std::shared_ptr<RosRuntime> runtime;
    std::mutex mutex;
    std::condition_variable wake;
    std::atomic<bool> closing{false};
    double stopWait, requestTimeout;
    std::vector<std::unique_ptr<Target>> targets;
    std::vector<BagPreset> presets;
    std::vector<std::pair<std::string, std::string>> topics;
    rclcpp::TimerBase::SharedPtr timer;
};
} // namespace
void registerRosBagging(Registry &registry, const RuntimeFactory &runtime) {
    registry.providers.emplace(
        "ros.bagging",
        ProviderFactory{
            Kind::Bagging,
            [](const YAML::Node &cfg) {
                keys(cfg, {"targets", "presets", "stop_timeout", "request_timeout"}, "ros.bagging");
                positive(cfg, "stop_timeout", 10, 300);
                positive(cfg, "request_timeout", 30, 300);
                if (!cfg["targets"].IsSequence() || cfg["targets"].size() == 0)
                    throw std::invalid_argument("targets must be a non-empty sequence");
                std::set<std::string> ids;
                for (const auto &entry : cfg["targets"]) {
                    keys(entry, {"id", "label", "host", "directory", "setup", "ssh", "record_args", "stop_on_exit"},
                         "bag target");
                    required(entry, {"id"});
                    if (!ids.insert(entry["id"].as<std::string>()).second)
                        throw std::invalid_argument("duplicate bag target ID");
                    (void)entry["label"].as<std::string>("");
                    (void)entry["host"].as<std::string>("");
                    (void)entry["directory"].as<std::string>("");
                    (void)entry["setup"].as<std::string>("");
                    (void)entry["stop_on_exit"].as<bool>(false);
                    for (const char *list : {"ssh", "record_args"})
                        if (entry[list] && !entry[list].IsSequence())
                            throw std::invalid_argument(std::string(list) + " must be a sequence");
                        else if (entry[list])
                            (void)entry[list].as<std::vector<std::string>>();
                    if (entry["ssh"] && entry["ssh"].size() == 0)
                        throw std::invalid_argument("ssh must name the ssh program");
                }
                if (cfg["presets"]) {
                    if (!cfg["presets"].IsMap())
                        throw std::invalid_argument("presets must map names to topic lists");
                    for (const auto &preset : cfg["presets"])
                        (void)preset.second.as<std::vector<std::string>>();
                }
            },
            [runtime](const YAML::Node &cfg, const Context &ctx) {
                return std::make_shared<RosBagging>(runtime(ctx), cfg, ctx);
            }});
}
} // namespace nereus::ros_viewer::panels

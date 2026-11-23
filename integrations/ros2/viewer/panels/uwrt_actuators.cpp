// "uwrt.actuators" provider: riptide ActuatorStatus (armed, claw / torpedo link, torpedo / marker counts) and
// configurable actuator command buttons, each publishing a Bool, Float32 or Empty message.
#include "ros_runtime.hpp"
#include <riptide_msgs2/msg/actuator_status.hpp>
#include <set>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

namespace nereus::ros_viewer::panels {
namespace {
using ActuatorStatus = riptide_msgs2::msg::ActuatorStatus;

// ActuatorStatus state names (indexed by the message constants).
const std::vector<std::string> clawStates{"Error",           "Disarmed",       "Unknown", "Opened",
                                          "Closed (bottle)", "Closed (spoon)", "Opening", "Closing"};
const std::vector<std::string> torpedoStates{"Error", "Disarmed", "Charging", "Charged", "Firing", "Fired"};

// A mechanism's link from its ActuatorStatus state: the firmware reports ERROR (0) while the servo does not
// answer its pings (`error` says what else it can mean); any other state means the board hears the servo.
ActuatorLink link(const std::string &label, bool fresh, uint8_t state, const std::vector<std::string> &names,
                  const std::string &error) {
    if (!fresh)
        return {label, "No status", "No recent actuator status from the board", Level::Stale};
    if (state == 0)
        return {label, "Not connected", error, Level::Error};
    return {label, "Connected", "State: " + (state < names.size() ? names[state] : "invalid " + std::to_string(state)),
            Level::Ok};
}

class UwrtActuators final : public Actuators {
    // One configured button; publish() is passed the current armed state.
    struct Command {
        std::string id, label, armedLabel;
        bool requiresArmed = false, disabled = false;
        int durationSign = 0;
        std::function<void(bool, float)> publish;
    };

  public:
    UwrtActuators(std::shared_ptr<RosRuntime> runtime, const YAML::Node &cfg, const Context &ctx)
        : runtime(runtime), timeout(cfg["status_timeout"].as<double>(1)) {
        // Build a publisher per command. A bool command sends its fixed value, or with toggle_armed the
        // opposite of the current armed state (an arm / disarm toggle).
        auto node = runtime->node;
        for (const auto &entry : cfg["commands"]) {
            Command command;
            command.id = entry["id"].as<std::string>();
            command.label = entry["label"].as<std::string>();
            command.armedLabel = entry["armed_label"].as<std::string>(command.label);
            command.requiresArmed = entry["requires_armed"].as<bool>(false);
            command.disabled = entry["disabled"].as<bool>(false);
            command.durationSign = entry["duration_sign"].as<int>(0);
            auto topic = expand(entry["topic"].as<std::string>(), ctx);
            auto type = entry["type"].as<std::string>();
            if (type == "bool") {
                auto pub = node->create_publisher<std_msgs::msg::Bool>(topic, 10);
                bool toggle = entry["toggle_armed"].as<bool>(false), value = entry["value"].as<bool>(false);
                command.publish = [pub, toggle, value](bool armed, float) {
                    std_msgs::msg::Bool msg;
                    msg.data = toggle ? !armed : value;
                    pub->publish(msg);
                };
            } else if (type == "float32") {
                auto pub = node->create_publisher<std_msgs::msg::Float32>(topic, 10);
                float value = entry["value"].as<float>(0);
                int sign = command.durationSign;
                command.publish = [pub, value, sign](bool, float duration) {
                    std_msgs::msg::Float32 msg;
                    msg.data = sign ? sign * duration : value;
                    pub->publish(msg);
                };
            } else {
                auto pub = node->create_publisher<std_msgs::msg::Empty>(topic, 10);
                command.publish = [pub](bool, float) { pub->publish(std_msgs::msg::Empty{}); };
            }
            commands.push_back(std::move(command));
        }

        hasCommandFeedback = bool(cfg["command_status_topic"]) || bool(cfg["command_feedback_topic"]);
        status = node->create_subscription<ActuatorStatus>(expand(cfg["status_topic"].as<std::string>(), ctx),
                                                           rclcpp::SensorDataQoS(), [this](const ActuatorStatus &msg) {
                                                               std::lock_guard<std::mutex> lock(mutex);
                                                               armed = msg.actuators_armed;
                                                               clawState = msg.claw_state;
                                                               torpedoState = msg.torpedo_state;
                                                               lastStatus = Steady::now();
                                                               readings["Torpedoes remaining"] =
                                                                   std::to_string(msg.torpedo_available_count);
                                                               readings["Markers remaining"] =
                                                                   std::to_string(msg.dropper_available_count);
                                                           });
        if (cfg["command_status_topic"])
            commandStatus = node->create_subscription<std_msgs::msg::Bool>(
                expand(cfg["command_status_topic"].as<std::string>(), ctx), 10,
                [this](const std_msgs::msg::Bool &msg) {
                    std::lock_guard<std::mutex> lock(mutex);
                    commandResultReceived = true;
                    commandSucceeded = msg.data;
                });
        if (cfg["command_feedback_topic"])
            commandFeedback = node->create_subscription<std_msgs::msg::String>(
                expand(cfg["command_feedback_topic"].as<std::string>(), ctx), 10,
                [this](const std_msgs::msg::String &msg) {
                    std::lock_guard<std::mutex> lock(mutex);
                    commandFeedbackReceived = true;
                    commandFeedbackText = msg.data;
                });
    }

    // Buttons are enabled only while status is fresh and, for requires_armed commands, the actuators are armed.
    ActuatorState state() override {
        std::lock_guard<std::mutex> lock(mutex);
        ActuatorState value;
        value.fresh = fresh();
        value.armed = armed;
        value.message =
            value.fresh ? (armed ? "Actuators armed" : "Actuators disarmed") : "Actuator status unavailable / stale";
        for (const auto &command : commands)
            value.actions.push_back({command.id, armed ? command.armedLabel : command.label,
                                     value.fresh && !command.disabled && (!command.requiresArmed || armed), command.durationSign != 0});
        value.links = {
            link("Claw", value.fresh, clawState, clawStates, "Status reports Error: the servo is not answering"),
            link("Torpedo", value.fresh, torpedoState, torpedoStates,
                 "Status reports Error: the servo is not answering, or has not been homed")};
        value.readings.assign(readings.begin(), readings.end());
        value.hasCommandFeedback = hasCommandFeedback;
        value.commandResultReceived = commandResultReceived;
        value.commandSucceeded = commandSucceeded;
        value.commandFeedbackReceived = commandFeedbackReceived;
        value.commandFeedback = commandFeedbackText;
        return value;
    }

    void command(const std::string &id, float duration = 0) override {
        std::lock_guard<std::mutex> lock(mutex);
        if (!fresh())
            return;
        for (const auto &command : commands)
            if (command.id == id && !command.disabled && (!command.requiresArmed || armed) &&
                (!command.durationSign || (std::isfinite(duration) && duration > 0))) {
                command.publish(armed, duration);
                return;
            }
    }

  private:
    // An ActuatorStatus arrived within status_timeout seconds.
    bool fresh() const {
        return std::chrono::duration<double>(Steady::now() - lastStatus).count() < timeout;
    }

    std::shared_ptr<RosRuntime> runtime;
    std::mutex mutex;
    double timeout;
    bool armed = false, hasCommandFeedback = false, commandResultReceived = false, commandSucceeded = false;
    bool commandFeedbackReceived = false;
    std::string commandFeedbackText;
    uint8_t clawState = ActuatorStatus::CLAW_ERROR, torpedoState = ActuatorStatus::TORPEDO_ERROR;
    Steady::time_point lastStatus{};
    std::vector<Command> commands;
    std::map<std::string, std::string> readings;
    rclcpp::Subscription<ActuatorStatus>::SharedPtr status;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr commandStatus;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr commandFeedback;
};
} // namespace

// Registers "uwrt.actuators"; validates each command entry (unique id, known type, finite float value).
void registerUwrtActuators(Registry &registry, const RuntimeFactory &runtime) {
    registry.providers.emplace(
        "uwrt.actuators",
        ProviderFactory{Kind::Actuators,
                        [](const YAML::Node &cfg) {
                            keys(cfg, {"status_topic", "commands", "status_timeout", "command_status_topic", "command_feedback_topic"}, "uwrt.actuators");
                            required(cfg, {"status_topic"});
                            positive(cfg, "status_timeout", 1);
                            if (!cfg["commands"].IsSequence())
                                throw std::invalid_argument("commands must be a sequence");
                            std::set<std::string> ids;
                            for (const auto &entry : cfg["commands"]) {
                                keys(entry,
                                     {"id", "label", "armed_label", "topic", "type", "value", "requires_armed",
                                      "toggle_armed", "duration_sign", "disabled"},
                                     "actuator command");
                                required(entry, {"id", "label", "topic", "type"});
                                if (!ids.insert(entry["id"].as<std::string>()).second)
                                    throw std::invalid_argument("duplicate actuator command ID");
                                auto type = entry["type"].as<std::string>();
                                if (type != "bool" && type != "float32" && type != "empty")
                                    throw std::invalid_argument("command type must be bool, float32 or empty");
                                int sign = entry["duration_sign"].as<int>(0);
                                if (sign < -1 || sign > 1 || (sign && type != "float32"))
                                    throw std::invalid_argument("duration_sign must be -1 or 1 on float32 commands");
                                if (type == "float32" && !sign && !std::isfinite(entry["value"].as<float>()))
                                    throw std::invalid_argument("nonfinite actuator command");
                                if (type == "bool")
                                    (void)entry["value"].as<bool>(false);
                                (void)entry["requires_armed"].as<bool>(false);
                                (void)entry["disabled"].as<bool>(false);
                                (void)entry["toggle_armed"].as<bool>(false);
                            }
                        },
                        [runtime](const YAML::Node &cfg, const Context &ctx) {
                            return std::make_shared<UwrtActuators>(runtime(ctx), cfg, ctx);
                        }});
}
} // namespace nereus::ros_viewer::panels

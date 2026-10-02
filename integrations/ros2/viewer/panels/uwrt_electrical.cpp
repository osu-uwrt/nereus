#include "ros_runtime.hpp"
#include <cmath>
#include <ctime>
#include <deque>
#include <rclcpp_action/rclcpp_action.hpp>
#include <riptide_msgs2/action/mag_cal.hpp>
#include <riptide_msgs2/action/tare_gyro.hpp>
#include <riptide_msgs2/msg/electrical_command.hpp>
#include <riptide_msgs2/msg/u_int8_stamped.hpp>
#include <riptide_msgs2/srv/query_imu_serial.hpp>
#include <set>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace nereus::ros_viewer::panels {
namespace {
using MagCal = riptide_msgs2::action::MagCal;
using MagGoal = rclcpp_action::ClientGoalHandle<MagCal>;
using TareGyro = riptide_msgs2::action::TareGyro;
using TareGoal = rclcpp_action::ClientGoalHandle<TareGyro>;
using ImuSerial = riptide_msgs2::srv::QueryImuSerial;
using CommandMsg = riptide_msgs2::msg::ElectricalCommand;

std::string clockStamp() {
    char text[16];
    const std::time_t now = std::time(nullptr);
    std::tm local{};
    localtime_r(&now, &local);
    std::strftime(text, sizeof(text), "%H:%M:%S", &local);
    return text;
}
// The register fields of a VectorNav reply, "$VNRRG,05,115200*58" -> "115200" (as the RViz panel shows them).
std::string registerFields(const std::string &reply) {
    auto fields = reply;
    for (int i = 0; i < 2; ++i)
        if (const auto comma = fields.find(','); comma != std::string::npos)
            fields = fields.substr(comma + 1);
    return fields.substr(0, fields.find('*'));
}
bool digits(const std::string &text) {
    return !text.empty() && text.size() <= 3 && text.find_first_not_of("0123456789") == std::string::npos;
}

// The RViz electrical panel's endpoints: command/electrical, the VectorNav driver's mag cal action and register
// service, the FOG driver's tare action, the pinger broker and the IVC link. Sections absent from the options
// are not shown. The pinger enable state is re-sent every `heartbeat_s` (as RViz) so a rebooted board recovers it.
class UwrtElectrical final : public Electrical {
    struct Command {
        ElectricalCommandItem item;
        uint8_t value = 0;
    };

  public:
    UwrtElectrical(std::shared_ptr<RosRuntime> runtime, const YAML::Node &cfg, const Context &ctx)
        : runtime(runtime), timeout(cfg["request_timeout"].as<double>(3)),
          saveTimeout(cfg["save_timeout"].as<double>(10)) {
        auto node = runtime->node;
        const auto name = [&](const YAML::Node &n) { return expand(n.as<std::string>(), ctx); };
        if (cfg["command_topic"]) {
            commandPub = node->create_publisher<CommandMsg>(name(cfg["command_topic"]), 10);
            for (const auto &entry : cfg["commands"])
                commands.push_back({{entry["id"].as<std::string>(), entry["label"].as<std::string>(),
                                     entry["confirm"].as<bool>(false)},
                                    uint8_t(entry["value"].as<int>())});
        }
        if (const auto imu = cfg["imu"]) {
            value.hasImu = true;
            magCal = rclcpp_action::create_client<MagCal>(node, name(imu["mag_cal_action"]));
            registers = node->create_client<ImuSerial>(name(imu["config_service"]));
        }
        if (cfg["tare_action"]) {
            value.hasTare = true;
            tare = rclcpp_action::create_client<TareGyro>(node, name(cfg["tare_action"]));
        }
        if (const auto pinger = cfg["pinger"]) {
            value.hasPinger = true;
            value.pingerEnabled = pinger["enabled"].as<bool>(true);
            value.pingerFrequencies = pinger["frequencies_khz"].as<std::vector<int>>();
            heartbeat = pinger["heartbeat_s"].as<double>(1);
            pingerEnable = node->create_publisher<std_msgs::msg::Bool>(name(pinger["enable_topic"]), 10);
            pingerFrequency = node->create_publisher<std_msgs::msg::Int32>(name(pinger["frequency_topic"]), 10);
            pingerSelected = node->create_subscription<std_msgs::msg::Int32>(
                name(pinger["selected_topic"]), 10, [this](const std_msgs::msg::Int32 &msg) {
                    std::lock_guard<std::mutex> lock(mutex);
                    value.pingerSelected = msg.data;
                });
            pingerAmplitude = node->create_subscription<std_msgs::msg::Float32>(
                name(pinger["amplitude_topic"]), 10, [this](const std_msgs::msg::Float32 &msg) {
                    std::lock_guard<std::mutex> lock(mutex);
                    value.pingerAmplitude = msg.data;
                });
        }
        if (const auto ivc = cfg["ivc"]) {
            value.hasIvc = true;
            value.ivcHeaders = ivc["headers"].as<std::vector<std::string>>();
            value.ivcStatuses = ivc["statuses"].as<std::vector<std::string>>(std::vector<std::string>{});
            value.ivcStatusHeaders = ivc["status_headers"].as<int>(0);
            ivcTx = node->create_publisher<std_msgs::msg::UInt8>(name(ivc["tx_topic"]), 10);
            // Our own sends come back on tx too, like everyone else's: the log shows what went out.
            const auto logger = [this](const char *direction) {
                return [this, direction](const std_msgs::msg::UInt8 &msg) {
                    std::lock_guard<std::mutex> lock(mutex);
                    logIvc(direction, msg.data);
                };
            };
            ivcTxSub = node->create_subscription<std_msgs::msg::UInt8>(name(ivc["tx_topic"]), 10, logger("SEND"));
            ivcRx = node->create_subscription<std_msgs::msg::UInt8>(name(ivc["rx_topic"]), 10, logger("RECV"));
            ivcConfirm = node->create_subscription<riptide_msgs2::msg::UInt8Stamped>(
                name(ivc["confirm_topic"]), 10, [this](const riptide_msgs2::msg::UInt8Stamped &msg) {
                    std::lock_guard<std::mutex> lock(mutex);
                    logIvc("ACK ", msg.data);
                });
        }
        for (const auto &command : commands)
            value.commands.push_back(command.item);
        timer = node->create_wall_timer(std::chrono::milliseconds(100), [this] { tick(); });
    }
    ~UwrtElectrical() override {
        // The host stops the executor first; cancel accepted goals without callbacks that could outlive us.
        if (magGoal && rclcpp::ok())
            magCal->async_cancel_goal(magGoal);
        if (tareGoal && rclcpp::ok())
            tare->async_cancel_goal(tareGoal);
    }
    ElectricalState state() override {
        std::lock_guard<std::mutex> lock(mutex);
        auto copy = value;
        copy.ivcLog.assign(ivcLines.begin(), ivcLines.end());
        return copy;
    }
    void command(const std::string &id) override {
        std::lock_guard<std::mutex> lock(mutex);
        for (const auto &command : commands)
            if (command.item.id == id) {
                CommandMsg msg;
                msg.command = command.value;
                commandPub->publish(msg);
                value.commandMessage = clockStamp() + "  Sent: " + command.item.label;
                return;
            }
    }
    void startMagCal() override {
        std::lock_guard<std::mutex> lock(mutex);
        if (!magCal || value.magCalRunning || !magCal->action_server_is_ready())
            return;
        value.magCalRunning = true;
        value.magCalProgress = 0;
        value.magCalMessage = "Starting mag cal...";
        magCancel = false;
        magAwaiting = true;
        magSince = Steady::now();
        maxDeviation = 1e-10;
        auto options = rclcpp_action::Client<MagCal>::SendGoalOptions();
        options.goal_response_callback = [this](MagGoal::SharedPtr accepted) {
            std::lock_guard<std::mutex> lock(mutex);
            magAwaiting = false;
            magGoal = accepted;
            if (!magGoal) {
                value.magCalRunning = false;
                value.magCalMessage = "Mag cal rejected (already running, or the IMU is not connected)";
            } else if (magCancel)
                magCal->async_cancel_goal(magGoal);
            else
                value.magCalMessage = "Calibrating: turn the robot slowly through every orientation";
        };
        options.feedback_callback = [this](MagGoal::SharedPtr, MagCal::Feedback::ConstSharedPtr feedback) {
            std::lock_guard<std::mutex> lock(mutex);
            double sum = 0;
            for (const double deviation : feedback->curr_avg_dev)
                sum += deviation * deviation;
            const double total = std::sqrt(sum);
            maxDeviation = std::max(maxDeviation, total);
            value.magCalProgress = float(std::clamp(1 - total / maxDeviation, 0., 1.));
        };
        options.result_callback = [this](const MagGoal::WrappedResult &result) {
            std::lock_guard<std::mutex> lock(mutex);
            magGoal.reset();
            value.magCalRunning = false;
            switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                value.magCalProgress = 1;
                value.magCalMessage = "Mag cal complete";
                break;
            case rclcpp_action::ResultCode::CANCELED:
                value.magCalMessage = "Mag cal canceled";
                break;
            default:
                value.magCalMessage = "Mag cal aborted; see the IMU driver log";
            }
        };
        magCal->async_send_goal(MagCal::Goal{}, options);
    }
    void cancelMagCal() override {
        std::lock_guard<std::mutex> lock(mutex);
        if (!value.magCalRunning)
            return;
        magCancel = true;
        value.magCalMessage = "Canceling mag cal...";
        if (magGoal)
            magCal->async_cancel_goal(magGoal);
    }
    void readRegister(const std::string &reg) override {
        if (digits(reg))
            request("$VNRRG," + reg, "Read register " + reg, timeout);
    }
    void writeRegister(const std::string &reg, const std::string &data) override {
        if (digits(reg) && !data.empty() && data.find_first_of("$*\r\n") == std::string::npos)
            request("$VNWRG," + reg + "," + data, "Wrote register " + reg, timeout);
    }
    void saveImuSettings() override {
        request("$VNWNV", "Saved the IMU settings to flash", saveTimeout);
    }
    void startTare(int samples, double seconds) override {
        std::lock_guard<std::mutex> lock(mutex);
        if (!tare || value.tareRunning || !tare->action_server_is_ready() || samples < 1 || !std::isfinite(seconds) ||
            seconds <= 0)
            return;
        value.tareRunning = true;
        value.tareMessage = "Starting tare...";
        tareCancel = false;
        tareAwaiting = true;
        tareSince = Steady::now();
        TareGyro::Goal goal;
        goal.num_samples = samples;
        goal.timeout_seconds = seconds;
        auto options = rclcpp_action::Client<TareGyro>::SendGoalOptions();
        options.goal_response_callback = [this, samples](TareGoal::SharedPtr accepted) {
            std::lock_guard<std::mutex> lock(mutex);
            tareAwaiting = false;
            tareGoal = accepted;
            if (!tareGoal) {
                value.tareRunning = false;
                value.tareMessage = "Tare rejected";
            } else if (tareCancel)
                tare->async_cancel_goal(tareGoal);
            else
                value.tareMessage = "Taring: hold the robot still (" + std::to_string(samples) + " samples)";
        };
        options.result_callback = [this](const TareGoal::WrappedResult &result) {
            std::lock_guard<std::mutex> lock(mutex);
            tareGoal.reset();
            value.tareRunning = false;
            const std::string detail = result.result && !result.result->result.empty() ? result.result->result : "";
            switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                value.tareMessage = "Tare complete" + (detail.empty() ? "" : ": " + detail);
                break;
            case rclcpp_action::ResultCode::CANCELED:
                value.tareMessage = "Tare canceled";
                break;
            default:
                value.tareMessage = "Tare aborted" + (detail.empty() ? "" : ": " + detail);
            }
        };
        tare->async_send_goal(goal, options);
    }
    void cancelTare() override {
        std::lock_guard<std::mutex> lock(mutex);
        if (!value.tareRunning)
            return;
        tareCancel = true;
        value.tareMessage = "Canceling tare...";
        if (tareGoal)
            tare->async_cancel_goal(tareGoal);
    }
    void setPingerEnabled(bool enabled) override {
        std::lock_guard<std::mutex> lock(mutex);
        if (!pingerEnable)
            return;
        value.pingerEnabled = enabled;
        publishPingerEnabled();
    }
    void setPingerFrequency(int khz) override {
        std::lock_guard<std::mutex> lock(mutex);
        if (!pingerFrequency || khz <= 0)
            return;
        std_msgs::msg::Int32 msg;
        msg.data = khz;
        pingerFrequency->publish(msg);
    }
    void sendIvc(int header, int command) override {
        std::lock_guard<std::mutex> lock(mutex);
        if (!ivcTx || header < 0 || header >= int(value.ivcHeaders.size()) || header > 7 || command < 0 ||
            (header < value.ivcStatusHeaders ? command >= int(value.ivcStatuses.size()) : command > 31))
            return;
        std_msgs::msg::UInt8 msg;
        msg.data = uint8_t((header & 0x7) << 5 | (command & 0x1F));
        ivcTx->publish(msg);
    }

  private:
    void request(const std::string &text, const std::string &done, double limit) {
        std::lock_guard<std::mutex> lock(mutex);
        if (!registers || value.registerPending || !registers->service_is_ready())
            return;
        value.registerPending = true;
        value.registerMessage = "Waiting for the IMU...";
        registerSince = Steady::now();
        registerLimit = limit;
        auto request = std::make_shared<ImuSerial::Request>();
        request->request = text;
        const auto epoch = ++registerEpoch;
        const bool save = text == "$VNWNV";
        registerId = registers
                         ->async_send_request(request,
                                              [this, epoch, done, save](rclcpp::Client<ImuSerial>::SharedFuture reply) {
                                                  std::lock_guard<std::mutex> lock(mutex);
                                                  if (epoch != registerEpoch)
                                                      return;
                                                  value.registerPending = false;
                                                  const auto response = reply.get()->response;
                                                  if (response.rfind("$VNERR", 0) == 0)
                                                      value.registerMessage = "IMU error reply: " + response;
                                                  else if (response.empty())
                                                      value.registerMessage = "No reply from the IMU";
                                                  else {
                                                      if (!save)
                                                          value.registerValue = registerFields(response);
                                                      value.registerMessage = done;
                                                  }
                                              })
                         .request_id;
    }
    void publishPingerEnabled() {
        std_msgs::msg::Bool msg;
        msg.data = value.pingerEnabled;
        pingerEnable->publish(msg);
        lastHeartbeat = Steady::now();
    }
    std::string ivcName(uint8_t byte) const {
        const int header = byte >> 5, data = byte & 0x1F;
        std::string text = header < int(value.ivcHeaders.size()) ? value.ivcHeaders[header] : "header";
        text += ": ";
        text += header < value.ivcStatusHeaders && data < int(value.ivcStatuses.size()) ? value.ivcStatuses[data]
                                                                                        : std::to_string(data);
        return text + "  (" + std::to_string(header) + "-" + std::to_string(data) + ")";
    }
    void logIvc(const char *direction, uint8_t byte) {
        ivcLines.push_back(clockStamp() + "  " + direction + "  " + ivcName(byte));
        while (ivcLines.size() > 200)
            ivcLines.pop_front();
    }
    void tick() {
        std::lock_guard<std::mutex> lock(mutex);
        const auto now = Steady::now();
        auto elapsed = [&](auto since) { return std::chrono::duration<double>(now - since).count(); };
        if (magCal) {
            value.magCalReady = magCal->action_server_is_ready();
            value.registerReady = registers->service_is_ready();
            if (magAwaiting && elapsed(magSince) > timeout) {
                magAwaiting = false;
                value.magCalRunning = false;
                value.magCalMessage = "Mag cal request timed out";
            }
        }
        if (tare) {
            value.tareReady = tare->action_server_is_ready();
            if (tareAwaiting && elapsed(tareSince) > timeout) {
                tareAwaiting = false;
                value.tareRunning = false;
                value.tareMessage = "Tare request timed out";
            }
        }
        if (value.registerPending && elapsed(registerSince) > registerLimit) {
            registers->remove_pending_request(registerId);
            ++registerEpoch;
            value.registerPending = false;
            value.registerMessage = "The IMU config service never replied";
        }
        if (pingerEnable && heartbeat > 0 && elapsed(lastHeartbeat) >= heartbeat)
            publishPingerEnabled();
    }
    std::shared_ptr<RosRuntime> runtime;
    std::mutex mutex;
    ElectricalState value;
    std::deque<std::string> ivcLines;
    double timeout, saveTimeout, heartbeat = 0, registerLimit = 3, maxDeviation = 1e-10;
    std::vector<Command> commands;
    rclcpp::Publisher<CommandMsg>::SharedPtr commandPub;
    rclcpp_action::Client<MagCal>::SharedPtr magCal;
    MagGoal::SharedPtr magGoal;
    bool magAwaiting = false, magCancel = false;
    rclcpp_action::Client<TareGyro>::SharedPtr tare;
    TareGoal::SharedPtr tareGoal;
    bool tareAwaiting = false, tareCancel = false;
    rclcpp::Client<ImuSerial>::SharedPtr registers;
    uint64_t registerEpoch = 0;
    int64_t registerId = 0;
    Steady::time_point magSince{}, tareSince{}, registerSince{}, lastHeartbeat{};
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pingerEnable;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pingerFrequency;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pingerSelected;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pingerAmplitude;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr ivcTx;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr ivcTxSub, ivcRx;
    rclcpp::Subscription<riptide_msgs2::msg::UInt8Stamped>::SharedPtr ivcConfirm;
    rclcpp::TimerBase::SharedPtr timer;
};
void requireTopics(const YAML::Node &node, std::initializer_list<const char *> names, const char *where) {
    try {
        required(node, names);
    } catch (const std::exception &e) {
        throw std::invalid_argument(std::string(where) + ": " + e.what());
    }
}
} // namespace
void registerUwrtElectrical(Registry &registry, const RuntimeFactory &runtime) {
    registry.providers.emplace(
        "uwrt.electrical",
        ProviderFactory{
            Kind::Electrical,
            [](const YAML::Node &cfg) {
                keys(cfg,
                     {"command_topic", "commands", "imu", "tare_action", "pinger", "ivc", "request_timeout",
                      "save_timeout"},
                     "uwrt.electrical");
                positive(cfg, "request_timeout", 3);
                positive(cfg, "save_timeout", 10);
                if (cfg["commands"] && !cfg["command_topic"])
                    throw std::invalid_argument("commands need command_topic");
                std::set<std::string> ids;
                for (const auto &entry : cfg["commands"]) {
                    keys(entry, {"id", "label", "value", "confirm"}, "electrical command");
                    required(entry, {"id", "label", "value"});
                    const int value = entry["value"].as<int>();
                    if (value < 0 || value > 255 || !ids.insert(entry["id"].as<std::string>()).second)
                        throw std::invalid_argument("electrical commands need unique IDs and values 0..255");
                    (void)entry["confirm"].as<bool>(false);
                }
                if (const auto imu = cfg["imu"]) {
                    keys(imu, {"mag_cal_action", "config_service"}, "electrical imu");
                    requireTopics(imu, {"mag_cal_action", "config_service"}, "electrical imu");
                }
                if (const auto pinger = cfg["pinger"]) {
                    keys(pinger,
                         {"enable_topic", "frequency_topic", "selected_topic", "amplitude_topic", "frequencies_khz",
                          "enabled", "heartbeat_s"},
                         "electrical pinger");
                    requireTopics(pinger, {"enable_topic", "frequency_topic", "selected_topic", "amplitude_topic"},
                                  "electrical pinger");
                    for (const int khz : pinger["frequencies_khz"].as<std::vector<int>>())
                        if (khz <= 0)
                            throw std::invalid_argument("pinger frequencies must be positive kHz");
                    (void)pinger["enabled"].as<bool>(true);
                    const double heartbeat = pinger["heartbeat_s"].as<double>(1);
                    if (!std::isfinite(heartbeat) || heartbeat < 0 || heartbeat > 60)
                        throw std::invalid_argument("pinger heartbeat_s must be 0 (off) to 60");
                }
                if (const auto ivc = cfg["ivc"]) {
                    keys(ivc, {"tx_topic", "rx_topic", "confirm_topic", "headers", "statuses", "status_headers"},
                         "electrical ivc");
                    requireTopics(ivc, {"tx_topic", "rx_topic", "confirm_topic"}, "electrical ivc");
                    const auto headers = ivc["headers"].as<std::vector<std::string>>();
                    const auto statuses = ivc["statuses"].as<std::vector<std::string>>(std::vector<std::string>{});
                    const int statusHeaders = ivc["status_headers"].as<int>(0);
                    if (headers.empty() || headers.size() > 8 || statuses.size() > 32 || statusHeaders < 0 ||
                        statusHeaders > int(headers.size()))
                        throw std::invalid_argument(
                            "ivc: 1..8 headers, at most 32 statuses, status_headers <= headers");
                }
            },
            [runtime](const YAML::Node &cfg, const Context &ctx) {
                return std::make_shared<UwrtElectrical>(runtime(ctx), cfg, ctx);
            }});
}
} // namespace nereus::ros_viewer::panels

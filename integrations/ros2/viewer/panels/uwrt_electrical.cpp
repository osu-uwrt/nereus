// "uwrt.electrical" provider: the RViz electrical panel's tools for the viewer (board commands, IMU mag cal
// and registers, FOG tare, pinger, IVC link and LEDs).
#include "ros_runtime.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <deque>
#include <optional>
#include <rclcpp_action/rclcpp_action.hpp>
#include <riptide_msgs2/action/mag_cal.hpp>
#include <riptide_msgs2/action/tare_gyro.hpp>
#include <riptide_msgs2/msg/electrical_command.hpp>
#include <riptide_msgs2/msg/led_command.hpp>
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
using LedMsg = riptide_msgs2::msg::LedCommand;

const char *const kLedModes[] = {"solid", "slow flash", "fast flash", "breath", "single flash"};
const char *const kLedTargets[] = {"no LEDs", "CCB", "ALU", "all"};

// Cop: (CCB, ALU) colours and how long each step holds (s). Side A double-flashes while B is dark, then swap.
struct CopStep {
    std::array<float, 3> ccb, alu;
    double seconds;
};
constexpr std::array<float, 3> kRed{1, 0, 0}, kBlue{0, 0, 1}, kDark{0, 0, 0};
const CopStep kCop[] = {{kRed, kBlue, .08}, {kDark, kDark, .05}, {kRed, kBlue, .08}, {kDark, kDark, .07},
                        {kBlue, kRed, .08}, {kDark, kDark, .05}, {kBlue, kRed, .08}, {kDark, kDark, .07}};
constexpr double kRainbowCycle = 3;  // s per turn of the colour wheel
constexpr double kRainbowPeriod = .05; // s between rainbow updates (20 per second, per target)

// Full-saturation, full-value hue (turns) as RGB 0..1.
std::array<float, 3> hue(double turns) {
    const double h = 6 * (turns - std::floor(turns));
    const float x = float(1 - std::abs(std::fmod(h, 2) - 1));
    switch (int(h)) {
    case 0: return {1, x, 0};
    case 1: return {x, 1, 0};
    case 2: return {0, 1, x};
    case 3: return {0, x, 1};
    case 4: return {x, 0, 1};
    default: return {1, 0, x};
    }
}

// An LedCommand as a setting: the brightest channel is the brightness, the colour normalized to it.
LedSetting ledSetting(const LedMsg &m) {
    LedSetting led;
    const int peak = std::max({m.red, m.green, m.blue});
    led.brightness = float(peak) / 255;
    if (peak > 0) {
        led.red = float(m.red) / float(peak);
        led.green = float(m.green) / float(peak);
        led.blue = float(m.blue) / float(peak);
    }
    led.mode = m.mode;
    led.target = m.target;
    return led;
}

// Local wall-clock time as HH:MM:SS, for message and log prefixes.
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

// A VectorNav register number: one to three decimal digits.
bool digits(const std::string &text) {
    return !text.empty() && text.size() <= 3 && text.find_first_not_of("0123456789") == std::string::npos;
}

// The RViz electrical panel's endpoints: command/electrical, the VectorNav driver's mag cal action and register
// service, the FOG driver's tare action, the pinger broker and the IVC link. Sections absent from the options
// are not shown. The pinger enable state is re-sent every `heartbeat_s` (as RViz) so a rebooted board recovers it.
class UwrtElectrical final : public Electrical {
    // A configured board command: the UI item plus the ElectricalCommand byte it sends.
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

        // Each optional section creates its endpoints and sets its has* flag so the panel shows it.
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

        if (const auto leds = cfg["leds"]) {
            value.hasLeds = true;
            const auto topic = name(leds["topic"]);
            ledPub = node->create_publisher<LedMsg>(topic, 10);
            // Single flashes (riptide_mapping's detection pulses) are momentary and leave the LEDs as they were.
            ledSub = node->create_subscription<LedMsg>(topic, 10, [this](const LedMsg &msg) {
                if (msg.mode == LedMsg::SINGLETON_FLASH)
                    return;
                std::lock_guard<std::mutex> lock(mutex);
                value.ledsLast = ledSetting(msg);
            });
        }

        if (ledPub)
            effectTimer = node->create_wall_timer(std::chrono::milliseconds(10), [this] { animate(); });

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

    // Starts the VectorNav mag cal action; a Cancel pressed before acceptance is applied on acceptance.
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
        // Progress: how far the deviation norm has fallen from the largest value seen in this run.
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

    // VectorNav ASCII commands: $VNRRG read register, $VNWRG write register, $VNWNV save to flash.
    void readRegister(const std::string &reg) override {
        if (digits(reg))
            request("$VNRRG," + reg, "Read register " + reg, timeout);
    }

    // Data may not contain the sentence's own framing characters.
    void writeRegister(const std::string &reg, const std::string &data) override {
        if (digits(reg) && !data.empty() && data.find_first_of("$*\r\n") == std::string::npos)
            request("$VNWRG," + reg + "," + data, "Wrote register " + reg, timeout);
    }

    void saveImuSettings() override {
        request("$VNWNV", "Saved the IMU settings to flash", saveTimeout);
    }

    // Starts the FOG driver's gyro tare action (sample count and timeout in seconds).
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

    // The colour scaled by the brightness, each channel rounded to 0..255.
    void setLeds(const LedSetting &led) override {
        std::lock_guard<std::mutex> lock(mutex);
        if (ledPub)
            value.ledEffect = LedEffect::None;
        if (!ledPub || led.mode < LedMsg::MODE_SOLID || led.mode > LedMsg::SINGLETON_FLASH ||
            led.target < LedMsg::TARGET_CCB || led.target > LedMsg::TARGET_ALL)
            return;
        const auto byte = [&](float channel) {
            return uint8_t(std::lround(255 * std::clamp(channel, 0.f, 1.f) * std::clamp(led.brightness, 0.f, 1.f)));
        };
        LedMsg msg;
        msg.red = byte(led.red);
        msg.green = byte(led.green);
        msg.blue = byte(led.blue);
        msg.mode = uint8_t(led.mode);
        msg.target = uint8_t(led.target);
        ledPub->publish(msg);
        char rgb[16];
        std::snprintf(rgb, sizeof(rgb), "#%02X%02X%02X", msg.red, msg.green, msg.blue);
        value.ledsMessage = clockStamp() + "  Sent " + rgb + " " + kLedModes[msg.mode] + " to " +
                            kLedTargets[msg.target] + " LEDs";
    }

    void setLedEffect(LedEffect effect, float brightness) override {
        std::lock_guard<std::mutex> lock(mutex);
        if (!ledPub)
            return;
        effectBrightness = std::clamp(brightness, 0.f, 1.f);
        if (effect == value.ledEffect)
            return; // a new brightness only
        value.ledEffect = effect;
        effectStart = Steady::now();
        effectSent.reset();
        value.ledsMessage = clockStamp() + (effect == LedEffect::Cop       ? "  Cop lights on"
                                            : effect == LedEffect::Rainbow ? "  Rainbow on"
                                                                           : "  Effect stopped");
    }

    // One IVC byte: 3-bit header, 5-bit command. Status headers only take a listed status index.
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
    // Sends one sentence through the IMU serial query service; `done` is shown on success and `limit` is the
    // reply timeout in seconds. One request at a time.
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

    // Publishes the pinger enable state and restarts the heartbeat interval (mutex held).
    void publishPingerEnabled() {
        std_msgs::msg::Bool msg;
        msg.data = value.pingerEnabled;
        pingerEnable->publish(msg);
        lastHeartbeat = Steady::now();
    }

    // "header: status-or-number  (header-data)" for an IVC log line.
    std::string ivcName(uint8_t byte) const {
        const int header = byte >> 5, data = byte & 0x1F;
        std::string text = header < int(value.ivcHeaders.size()) ? value.ivcHeaders[header] : "header";
        text += ": ";
        text += header < value.ivcStatusHeaders && data < int(value.ivcStatuses.size()) ? value.ivcStatuses[data]
                                                                                        : std::to_string(data);
        return text + "  (" + std::to_string(header) + "-" + std::to_string(data) + ")";
    }

    // Appends a timestamped IVC log line, keeping the last 200 (mutex held).
    void logIvc(const char *direction, uint8_t byte) {
        ivcLines.push_back(clockStamp() + "  " + direction + "  " + ivcName(byte));
        while (ivcLines.size() > 200)
            ivcLines.pop_front();
    }

    // 100 Hz while an effect runs: each target's colour from the time since it started, sent when it changes
    // (Cop) or every kRainbowPeriod (Rainbow).
    void animate() {
        std::lock_guard<std::mutex> lock(mutex);
        if (value.ledEffect == LedEffect::None)
            return;
        const auto now = Steady::now();
        const double t = std::chrono::duration<double>(now - effectStart).count();
        std::array<float, 3> ccb, alu;
        if (value.ledEffect == LedEffect::Cop) {
            double cycle = 0;
            for (const auto &step : kCop)
                cycle += step.seconds;
            double at = std::fmod(t, cycle);
            const CopStep *step = &kCop[0];
            for (const auto &s : kCop) {
                step = &s;
                if (at < s.seconds)
                    break;
                at -= s.seconds;
            }
            ccb = step->ccb;
            alu = step->alu;
        } else {
            if (effectSent && std::chrono::duration<double>(now - effectSentAt).count() < kRainbowPeriod)
                return;
            ccb = hue(t / kRainbowCycle);
            alu = hue(t / kRainbowCycle + .5);
        }
        const auto bytes = [&](const std::array<float, 3> &c) {
            std::array<uint8_t, 3> out;
            for (int i = 0; i < 3; ++i)
                out[i] = uint8_t(std::lround(255 * c[i] * effectBrightness));
            return out;
        };
        const std::array<std::array<uint8_t, 3>, 2> frame{bytes(ccb), bytes(alu)};
        if (effectSent && *effectSent == frame)
            return;
        for (int i = 0; i < 2; ++i) {
            LedMsg msg;
            msg.red = frame[i][0];
            msg.green = frame[i][1];
            msg.blue = frame[i][2];
            msg.mode = LedMsg::MODE_SOLID;
            msg.target = i == 0 ? LedMsg::TARGET_CCB : LedMsg::TARGET_ALU;
            ledPub->publish(msg);
        }
        effectSent = frame;
        effectSentAt = now;
    }

    // 10 Hz: readiness flags, request timeouts and the pinger heartbeat.
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

        // heartbeat_s = 0 disables the periodic resend.
        if (pingerEnable && heartbeat > 0 && elapsed(lastHeartbeat) >= heartbeat)
            publishPingerEnabled();
    }

    std::shared_ptr<RosRuntime> runtime;
    std::mutex mutex;
    ElectricalState value;
    std::deque<std::string> ivcLines;
    // Seconds; maxDeviation is the largest mag cal deviation seen (progress baseline).
    double timeout, saveTimeout, heartbeat = 0, registerLimit = 3, maxDeviation = 1e-10;

    std::vector<Command> commands;
    rclcpp::Publisher<CommandMsg>::SharedPtr commandPub;

    // Action goals and their "sent but not yet accepted" / "cancel on acceptance" flags.
    rclcpp_action::Client<MagCal>::SharedPtr magCal;
    MagGoal::SharedPtr magGoal;
    bool magAwaiting = false, magCancel = false;
    rclcpp_action::Client<TareGyro>::SharedPtr tare;
    TareGoal::SharedPtr tareGoal;
    bool tareAwaiting = false, tareCancel = false;

    // IMU register service request bookkeeping.
    rclcpp::Client<ImuSerial>::SharedPtr registers;
    uint64_t registerEpoch = 0;
    int64_t registerId = 0;
    Steady::time_point magSince{}, tareSince{}, registerSince{}, lastHeartbeat{};

    // Pinger and IVC endpoints.
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pingerEnable;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pingerFrequency;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pingerSelected;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pingerAmplitude;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr ivcTx;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr ivcTxSub, ivcRx;
    rclcpp::Subscription<riptide_msgs2::msg::UInt8Stamped>::SharedPtr ivcConfirm;
    rclcpp::Publisher<LedMsg>::SharedPtr ledPub;
    rclcpp::Subscription<LedMsg>::SharedPtr ledSub;
    // The running LED effect: its brightness, start, and the last (CCB, ALU) colours sent and when.
    float effectBrightness = 1;
    Steady::time_point effectStart{}, effectSentAt{};
    std::optional<std::array<std::array<uint8_t, 3>, 2>> effectSent;
    rclcpp::TimerBase::SharedPtr effectTimer;
    rclcpp::TimerBase::SharedPtr timer;
};

// required(), with the section name prefixed to the error.
void requireTopics(const YAML::Node &node, std::initializer_list<const char *> names, const char *where) {
    try {
        required(node, names);
    } catch (const std::exception &e) {
        throw std::invalid_argument(std::string(where) + ": " + e.what());
    }
}
} // namespace

// Registers "uwrt.electrical"; every section is optional and validated on its own.
void registerUwrtElectrical(Registry &registry, const RuntimeFactory &runtime) {
    registry.providers.emplace(
        "uwrt.electrical",
        ProviderFactory{
            Kind::Electrical,
            [](const YAML::Node &cfg) {
                keys(cfg,
                     {"command_topic", "commands", "imu", "tare_action", "pinger", "ivc", "leds", "request_timeout",
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
                if (const auto leds = cfg["leds"]) {
                    keys(leds, {"topic"}, "electrical leds");
                    requireTopics(leds, {"topic"}, "electrical leds");
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

// "uwrt.telemetry" provider: header readouts for the FOG, computer diagnostics and battery packs.
#include "ros_runtime.hpp"
#include <cstdio>
#include <cstdlib>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <riptide_msgs2/msg/battery_status.hpp>
#include <riptide_msgs2/msg/gyro_status.hpp>
#include <set>

namespace nereus::ros_viewer::panels {
namespace {
using Diagnostics = diagnostic_msgs::msg::DiagnosticArray;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using GyroStatus = riptide_msgs2::msg::GyroStatus;
using BatteryStatus = riptide_msgs2::msg::BatteryStatus;

// Value formatters for readouts and tooltips.
std::string celsius(double value) {
    char text[32];
    std::snprintf(text, sizeof(text), "%.1f\u00B0C", value);
    return text;
}

std::string number(double value) {
    char text[32];
    std::snprintf(text, sizeof(text), "%.3g", value);
    return text;
}

std::string fixed(double value, int digits) {
    char text[32];
    std::snprintf(text, sizeof(text), "%.*f", digits, value);
    return text;
}

// Robot health readings at a glance. Sources:
//   gyro_status  riptide_msgs2/GyroStatus (the FOG): temperature plus the driver's health flags.
//   diagnostic   one status (by name) of a diagnostic_msgs/DiagnosticArray: the largest numeric value among its
//                key/values (the computer monitor's per-core "58.41 C"), combined with the status level.
//   battery      riptide_msgs2/BatteryStatus of one side (port / stbd; both share the topic): state of charge,
//                low is bad.
class UwrtTelemetry final : public Telemetry {
    // One configured reading: its latest Reading, thresholds (°C or %) and subscription. Held by pointer so
    // the subscription callbacks can keep `s` across vector growth.
    struct Source {
        Reading reading;
        std::string topic;
        double warn = 0, error = 0, timeout = 2;
        bool lowIsBad = false;
        bool seen = false;
        Steady::time_point received{};
        rclcpp::SubscriptionBase::SharedPtr subscription;
    };

  public:
    UwrtTelemetry(std::shared_ptr<RosRuntime> runtime, const YAML::Node &cfg, const Context &ctx) : runtime(runtime) {
        auto node = runtime->node;
        // Best effort matches any publisher. Keep 10, not the system default's single sample: both battery packs
        // publish back to back on one topic, and a one-deep history drops the first before its callback runs.
        const auto qos = rclcpp::QoS(10).best_effort();
        for (const auto &entry : cfg["readings"]) {
            auto source = std::make_unique<Source>();
            auto *s = source.get();
            s->reading.id = entry["id"].as<std::string>();
            s->reading.label = entry["label"].as<std::string>(s->reading.id);
            s->topic = expand(entry["topic"].as<std::string>(), ctx);
            s->timeout = entry["timeout"].as<double>(2);

            // Thresholds are percent for batteries (low is bad), degrees C for the rest (high is bad).
            const auto type = entry["source"].as<std::string>();
            if (type == "battery") {
                s->warn = entry["warn_pct"].as<double>();
                s->error = entry["error_pct"].as<double>();
                s->lowIsBad = true;
                const auto detect =
                    entry["side"].as<std::string>() == "port" ? BatteryStatus::DETECT_PORT : BatteryStatus::DETECT_STBD;
                s->subscription = node->create_subscription<BatteryStatus>(s->topic, qos,
                                                                           [this, s, detect](const BatteryStatus &msg) {
                                                                               if (msg.detect != detect)
                                                                                   return;
                                                                               std::lock_guard<std::mutex> lock(mutex);
                                                                               battery(*s, msg);
                                                                           });
            } else {
                s->warn = entry["warn_c"].as<double>();
                s->error = entry["error_c"].as<double>();
            }
            if (type == "gyro_status")
                s->subscription =
                    node->create_subscription<GyroStatus>(s->topic, qos, [this, s](const GyroStatus &msg) {
                        std::lock_guard<std::mutex> lock(mutex);
                        gyro(*s, msg);
                    });
            else if (type == "diagnostic")
                s->subscription = node->create_subscription<Diagnostics>(
                    s->topic, qos, [this, s, name = entry["status"].as<std::string>()](const Diagnostics &msg) {
                        for (const auto &status : msg.status)
                            if (status.name == name) {
                                std::lock_guard<std::mutex> lock(mutex);
                                diagnostic(*s, status);
                                return;
                            }
                    });
            sources.push_back(std::move(source));
        }
    }

    // Copies the readings, marking those never received or older than their timeout as stale.
    TelemetryState state() override {
        std::lock_guard<std::mutex> lock(mutex);
        TelemetryState value;
        const auto now = Steady::now();
        for (const auto &s : sources) {
            auto reading = s->reading;
            const double age = std::chrono::duration<double>(now - s->received).count();
            if (!s->seen) {
                reading.value = "--";
                reading.level = Level::Stale;
                reading.detail = "Waiting for " + s->topic;
            } else if (age > s->timeout) {
                reading.level = Level::Stale; // keep the last value, shown muted
                reading.detail = "Stale: last update " + number(age) + " s ago on " + s->topic + "\n" + reading.detail;
            } else
                reading.detail += "\n" + s->topic;
            value.readings.push_back(std::move(reading));
        }
        return value;
    }

  private:
    // Level for a value against the source's warn / error thresholds.
    static Level threshold(const Source &s, double value) {
        if (s.lowIsBad)
            return value < s.error ? Level::Error : value < s.warn ? Level::Warn : Level::Ok;
        return value >= s.error ? Level::Error : value >= s.warn ? Level::Warn : Level::Ok;
    }

    static void mark(Source &s) {
        s.seen = true;
        s.received = Steady::now();
    }

    // FOG temperature against the thresholds, overridden by the driver's connection and health flags.
    void gyro(Source &s, const GyroStatus &m) {
        mark(s);
        auto &r = s.reading;
        r.value = celsius(m.temperature);
        r.level = threshold(s, m.temperature);
        // The driver's flags, in the order the sensor monitor reports them.
        std::string state = "Connected";
        if (!m.connected) {
            r.value = "--";
            r.level = Level::Error;
            state = "Not connected";
        } else if (!m.temp_good || !m.vsupply_good || !m.sldcurrent_good || !m.diagsignal_good) {
            r.level = Level::Error;
            state = !m.temp_good         ? "Overheating"
                    : !m.vsupply_good    ? "Bad supply voltage"
                    : !m.sldcurrent_good ? "Bad SLD current"
                                         : "Bad diagnostic signal";
        } else if (!m.temp_within_cal) {
            r.level = std::max(r.level, Level::Warn);
            state = "Temperature outside calibration";
        }
        r.detail = state + "\nTemperature " + celsius(m.temperature) + " (warn " + number(s.warn) + ", error " +
                   number(s.error) + ")\nSupply " + number(m.vsupply) + ", SLD current " + number(m.sldcurrent) +
                   ", diag signal " + number(m.diagsignal);
    }

    // The fields the voltage monitor reports; pack current is negative while discharging.
    void battery(Source &s, const BatteryStatus &m) {
        mark(s);
        auto &r = s.reading;
        r.value = std::to_string(m.soc) + "%";
        r.level = threshold(s, m.soc);
        r.detail = "State of charge " + r.value + " (warn below " + number(s.warn) + "%, error below " +
                   number(s.error) + "%)\n" + fixed(m.pack_voltage, 2) + " V, " + fixed(m.pack_current, 2) +
                   " A (average " + fixed(m.average_current, 2) + " A)\nTime to discharge " +
                   std::to_string(m.time_to_dischg) + " min\nCell " + m.cell_name + ", serial " +
                   std::to_string(m.serial);
    }

    // Shows the highest numeric key/value of the matched status as °C; every key/value goes in the detail.
    void diagnostic(Source &s, const DiagnosticStatus &status) {
        mark(s);
        auto &r = s.reading;
        bool found = false;
        double highest = 0;
        r.detail = status.message;
        for (const auto &pair : status.values) {
            r.detail += "\n" + pair.key + ": " + pair.value;
            char *end = nullptr;
            const double parsed = std::strtod(pair.value.c_str(), &end);
            if (end != pair.value.c_str() && std::isfinite(parsed) && (!found || parsed > highest)) {
                highest = parsed;
                found = true;
            }
        }
        r.value = found ? celsius(highest) : "--";
        // The worse of our thresholds and the publisher's own level (its thresholds may differ).
        const Level reported = status.level == DiagnosticStatus::OK      ? Level::Ok
                               : status.level == DiagnosticStatus::WARN  ? Level::Warn
                               : status.level == DiagnosticStatus::ERROR ? Level::Error
                                                                         : Level::Stale;
        r.level =
            reported == Level::Stale ? Level::Stale : std::max(found ? threshold(s, highest) : Level::Ok, reported);
    }

    std::shared_ptr<RosRuntime> runtime;
    std::mutex mutex;
    std::vector<std::unique_ptr<Source>> sources;
};
} // namespace

// Registers "uwrt.telemetry"; battery readings and temperature readings take different keys.
void registerUwrtTelemetry(Registry &registry, const RuntimeFactory &runtime) {
    registry.providers.emplace(
        "uwrt.telemetry",
        ProviderFactory{
            Kind::Telemetry,
            [](const YAML::Node &cfg) {
                keys(cfg, {"readings"}, "uwrt.telemetry");
                if (!cfg["readings"].IsSequence() || cfg["readings"].size() == 0)
                    throw std::invalid_argument("readings must be a nonempty sequence");
                std::set<std::string> ids;
                for (const auto &entry : cfg["readings"]) {
                    if (!entry.IsMap())
                        throw std::invalid_argument("telemetry reading: expected a mapping");
                    const auto source = entry["source"].as<std::string>("");
                    if (source == "battery") {
                        keys(entry, {"id", "label", "source", "topic", "side", "warn_pct", "error_pct", "timeout"},
                             "battery reading");
                        required(entry, {"id", "topic", "side", "warn_pct", "error_pct"});
                        const auto side = entry["side"].as<std::string>();
                        if (side != "port" && side != "stbd")
                            throw std::invalid_argument("battery side must be port or stbd");
                        const double warn = entry["warn_pct"].as<double>(), error = entry["error_pct"].as<double>();
                        if (!(error >= 0 && error <= warn && warn <= 100))
                            throw std::invalid_argument("battery thresholds: 0 <= error_pct <= warn_pct <= 100");
                        positive(entry, "timeout", 2);
                        if (!ids.insert(entry["id"].as<std::string>()).second)
                            throw std::invalid_argument("duplicate telemetry reading ID");
                        continue;
                    }
                    keys(entry, {"id", "label", "source", "topic", "status", "warn_c", "error_c", "timeout"},
                         "telemetry reading");
                    required(entry, {"id", "source", "topic", "warn_c", "error_c"});
                    if (!ids.insert(entry["id"].as<std::string>()).second)
                        throw std::invalid_argument("duplicate telemetry reading ID");
                    if (source != "gyro_status" && source != "diagnostic")
                        throw std::invalid_argument("reading source must be gyro_status, diagnostic or battery");
                    if (source == "diagnostic")
                        required(entry, {"status"});
                    else if (entry["status"])
                        throw std::invalid_argument("'status' applies to diagnostic readings only");
                    const double warn = entry["warn_c"].as<double>(), error = entry["error_c"].as<double>();
                    if (!std::isfinite(warn) || !std::isfinite(error) || warn > error)
                        throw std::invalid_argument("warn_c and error_c must be finite, warn_c <= error_c");
                    positive(entry, "timeout", 2);
                }
            },
            [runtime](const YAML::Node &cfg, const Context &ctx) {
                return std::make_shared<UwrtTelemetry>(runtime(ctx), cfg, ctx);
            }});
}
} // namespace nereus::ros_viewer::panels

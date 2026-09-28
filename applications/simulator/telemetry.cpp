#include "telemetry.hpp"
#include "robotics/sensors/readings.hpp"
#include <iomanip>
#include <map>
#include <ostream>

namespace robotics::runner {
namespace {
std::string quote(const std::string &text) {
    std::string result = "\"";
    for (const char c : text) {
        result += c;
        if (c == '"') {
            result += c;
        }
    }
    return result + '"';
}
class Row {
  public:
    Row(std::ostream &output, const sensors::SampleHeader &header, const std::string &reason)
        : output_(output), header_(header), reason_(reason) {}
    void field(const std::string &name, const char *unit, std::optional<double> value) const {
        output_ << header_.generation << ',' << quote(header_.device_id) << ','
                << quote(header_.frame) << ',' << header_.sequence << ',' << header_.tick << ','
                << header_.scheduled.count() << ',' << header_.acquired.count() << ','
                << header_.delivered.count() << ',' << (reason_.empty() ? 1 : 0) << ','
                << quote(reason_) << ',' << quote(name) << ',' << quote(unit) << ',';
        if (value) {
            output_ << *value;
        }
        output_ << '\n';
    }
    void vector(const std::string &name, const char *unit, const Eigen::Vector3d &value) const {
        const char *axes[] = {"x", "y", "z"};
        for (Eigen::Index i = 0; i < 3; ++i) {
            field(name + "." + axes[i], unit, value[i]);
        }
    }
    void matrix(const std::string &name, const char *unit, const Eigen::MatrixXd &value) const {
        for (Eigen::Index row = 0; row < value.rows(); ++row) {
            for (Eigen::Index col = 0; col < value.cols(); ++col) {
                field(name + "." + std::to_string(row) + "." + std::to_string(col), unit,
                      value(row, col));
            }
        }
    }

  private:
    std::ostream &output_;
    const sensors::SampleHeader &header_;
    const std::string &reason_;
};
void fields(const Row &row, const sensors::ImuReading &reading) {
    row.vector("specific_force", "m/s^2", reading.specific_force);
    row.vector("angular_velocity", "rad/s", reading.angular_velocity);
    row.matrix("force_covariance", "m^2/s^4", reading.force_covariance);
    row.matrix("angular_covariance", "rad^2/s^2", reading.angular_covariance);
}
void fields(const Row &row, const sensors::FogReading &reading) {
    for (Eigen::Index i = 0; i < reading.angular_rates.size(); ++i) {
        row.field("angular_rate." + std::to_string(i), "rad/s", reading.angular_rates[i]);
    }
    row.matrix("angular_covariance", "rad^2/s^2", reading.covariance);
}
void fields(const Row &row, const sensors::DvlReading &reading) {
    row.vector("bottom_relative_velocity", "m/s", reading.bottom_relative_velocity);
    row.matrix("velocity_covariance", "m^2/s^2", reading.covariance);
    row.field("bottom_distance", "m", reading.bottom_distance);
}
void fields(const Row &row, const sensors::PressureReading &reading) {
    row.field("absolute_pressure", "Pa", reading.absolute_pressure);
    row.field("pressure_variance", "Pa^2", reading.pressure_variance);
    row.field("depth", "m", reading.depth);
    row.field("depth_variance", "m^2", reading.depth_variance);
}
template <class Reading>
std::function<void()> watch(sensors::Runtime &runtime, const std::string &id,
                            std::ostream *output) {
    auto stream = runtime.stream<Reading>(id);
    return [stream, output] {
        for (const auto &sample : stream->drain()) {
            if (!output) {
                continue;
            }
            const Row row(*output, sample.header, sample.measurement.unavailable_reason);
            if (sample.measurement.value) {
                fields(row, *sample.measurement.value);
            } else {
                row.field("", "", std::nullopt);
            }
        }
        if (output && !*output) {
            throw std::runtime_error("failed to write sensor observations");
        }
    };
}
} // namespace
void sensorHeader(std::ostream &output) {
    output << std::setprecision(17)
           << "generation,device,frame,sequence,tick,scheduled_ns,acquired_ns,delivered_ns,valid,"
              "reason,field,unit,value\n";
}
std::vector<std::function<void()>> telemetry(sensors::Runtime &runtime,
                                             const std::vector<config::SensorPlan> &plans,
                                             std::ostream *output) {
    using Factory = std::function<std::function<void()>(sensors::Runtime &, const std::string &,
                                                        std::ostream *)>;
    const std::map<std::string, Factory> factories{{"imu", watch<sensors::ImuReading>},
                                                   {"fog", watch<sensors::FogReading>},
                                                   {"dvl", watch<sensors::DvlReading>},
                                                   {"pressure", watch<sensors::PressureReading>}};
    std::vector<std::function<void()>> observers;
    for (const auto &plan : plans) {
        const auto factory = factories.find(plan.model);
        if (factory == factories.end()) {
            throw std::invalid_argument("no CSV exporter for model " + plan.model);
        }
        observers.push_back(factory->second(runtime, plan.device.id, output));
    }
    return observers;
}
} // namespace robotics::runner

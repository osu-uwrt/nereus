#pragma once
// Shared helpers for the equivalence tests: fixtures written by fixtures/capture_session.py from
// the Python reference are compared with the C++ port through the same JSON layout.
#include <robotics/sensors/models.hpp>
#include <robotics/session/session.hpp>

#include <cmath>
#include <fstream>
#include <sstream>

namespace robotics::session::testing {
inline Json loadFixture(const std::string &name) {
    std::ifstream stream(std::string(RP_SESSION_FIXTURES) + "/" + name);
    if (!stream)
        throw std::runtime_error("missing fixture " + name);
    return Json::parse(stream);
}

template <class Matrix> Json flat(const Matrix &m) { // row-major, like numpy ravel()
    Json out = Json::array();
    for (Eigen::Index r = 0; r < m.rows(); ++r)
        for (Eigen::Index c = 0; c < m.cols(); ++c)
            out.push_back(m(r, c));
    return out;
}
inline Json quat(const Eigen::Quaterniond &q) {
    return Json::array({q.w(), q.x(), q.y(), q.z()});
}
inline Json bodyJson(const simulation::BodyState &b) {
    return {{"position", flat(b.position)},
            {"orientation", quat(b.orientation)},
            {"linear_velocity", flat(b.linear_velocity)},
            {"angular_velocity", flat(b.angular_velocity)}};
}

// Long-horizon trajectories: Release builds reproduce the Python binding to ~1e-12; Debug builds
// differ in floating-point contraction and chaotic amplification (observed up to ~5e-4 absolute after 1-2 s of contact-rich motion; only Release is a tight check).
#ifdef NDEBUG
constexpr double kTrajectoryTolerance = 1e-9;
#else
constexpr double kTrajectoryTolerance = 5e-3;
#endif

// Compare with tolerance for floating values, exactly for everything else. Returns "" when equal.
inline std::string diff(const Json &expected, const Json &actual, const std::string &path = "$",
                        double tolerance = kTrajectoryTolerance) {
    if (expected.is_number() && actual.is_number()) {
        if (expected.is_number_integer() && actual.is_number_integer())
            return expected == actual ? "" : path + ": " + expected.dump() + " != " + actual.dump();
        const double a = expected.get<double>(), b = actual.get<double>();
        return std::abs(a - b) <= tolerance * (1 + std::abs(a)) ? ""
                                                                 : path + ": " + expected.dump() + " != " + actual.dump();
    }
    if (expected.type() != actual.type())
        return path + ": type " + expected.dump().substr(0, 60) + " vs " + actual.dump().substr(0, 60);
    if (expected.is_object()) {
        for (const auto &[key, value] : expected.items()) {
            if (!actual.contains(key))
                return path + "." + key + ": missing";
            if (auto d = diff(value, actual.at(key), path + "." + key, tolerance); !d.empty())
                return d;
        }
        for (const auto &[key, value] : actual.items()) {
            (void)value;
            if (!expected.contains(key))
                return path + "." + key + ": unexpected";
        }
        return "";
    }
    if (expected.is_array()) {
        if (expected.size() != actual.size())
            return path + ": size " + std::to_string(expected.size()) + " != " + std::to_string(actual.size());
        for (std::size_t i = 0; i < expected.size(); ++i)
            if (auto d = diff(expected[i], actual[i], path + "[" + std::to_string(i) + "]", tolerance); !d.empty())
                return d;
        return "";
    }
    return expected == actual ? "" : path + ": " + expected.dump() + " != " + actual.dump();
}

inline Json sensorsJson(sensors::Runtime &runtime, const PackRuntime &pack) {
    using namespace sensors;
    Json out = Json::object();
    for (const auto &id : pack.sensor_ids) {
        const std::string &type = pack.sensor_types.at(id);
        const auto emit = [&](const auto &stream, auto values) {
            const auto sample = stream->latest();
            if (!sample || !sample->measurement.value) {
                out[id] = nullptr;
                return;
            }
            Json entry = {{"sequence", sample->header.sequence},
                          {"acquired_ns", sample->header.acquired.count()},
                          {"stats", stream->stats().acquired}};
            entry.update(values(*sample->measurement.value));
            out[id] = entry;
        };
        if (type == "ahrs")
            emit(runtime.stream<AhrsReading>(id), [](const AhrsReading &r) {
                return Json{{"specific_force", flat(r.inertial.specific_force)},
                            {"angular_velocity", flat(r.inertial.angular_velocity)},
                            {"force_covariance", flat(r.inertial.force_covariance)},
                            {"angular_covariance", flat(r.inertial.angular_covariance)},
                            {"attitude", quat(r.attitude.sensor_to_world)},
                            {"attitude_covariance", flat(r.attitude.covariance)}};
            });
        else if (type == "fog")
            emit(runtime.stream<FogReading>(id), [](const FogReading &r) {
                return Json{{"angular_rates", flat(r.angular_rates)}, {"covariance", flat(r.covariance)}};
            });
        else if (type == "reference_velocity")
            emit(runtime.stream<VelocityReading>(id), [](const VelocityReading &r) {
                return Json{{"velocity", flat(r.reference_relative_velocity)}, {"covariance", flat(r.covariance)}};
            });
        else if (type == "reference_altitude")
            emit(runtime.stream<AltitudeReading>(id), [](const AltitudeReading &r) {
                return Json{{"mounted_world_z", r.mounted_world_z},
                            {"target_world_z", r.target_world_z},
                            {"variance", r.variance}};
            });
        else
            throw std::runtime_error("test has no reader for sensor type " + type);
    }
    return out;
}
inline void drain(sensors::Runtime &runtime, const PackRuntime &pack) {
    using namespace sensors;
    for (const auto &id : pack.sensor_ids) {
        const std::string &type = pack.sensor_types.at(id);
        if (type == "ahrs") runtime.stream<AhrsReading>(id)->drain();
        else if (type == "fog") runtime.stream<FogReading>(id)->drain();
        else if (type == "reference_velocity") runtime.stream<VelocityReading>(id)->drain();
        else if (type == "reference_altitude") runtime.stream<AltitudeReading>(id)->drain();
    }
}
inline Json resultJson(const CommandResult &r) {
    return {{"accepted", r.accepted}, {"message", r.message}};
}
inline Json mechJson(const MechanismState &s) {
    Json releases = Json::object(), claws = Json::object();
    for (const auto &[k, v] : s.releases)
        releases[k] = {{"state", v.state}, {"available", v.available}};
    for (const auto &[k, v] : s.claws)
        claws[k] = {{"state", v.state},
                    {"gap_m", v.gap_m},
                    {"target_gap_m", v.target_gap_m},
                    {"joints", Json::array({v.joint_positions_m[0], v.joint_positions_m[1]})}};
    return {{"time_ns", s.time_ns}, {"armed", s.armed}, {"any_busy", s.any_busy},
            {"releases", releases}, {"claws", claws}};
}
inline std::vector<std::string> sensorNames(const ResolvedScenario &scenario) {
    std::vector<std::string> ids;
    for (const auto &s : scenario.robot.at("sensors"))
        if (s.at("type") != "stereo_camera")
            ids.push_back(s.at("id"));
    return ids;
}
} // namespace robotics::session::testing

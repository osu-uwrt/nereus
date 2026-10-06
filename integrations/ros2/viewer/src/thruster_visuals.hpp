// Rotor animation driven by realized thruster forces. Rotor pivots/axes and the propeller thrust law come
// from a viewer YAML document; the force-array order comes from the bridge document (thrusters.order).
#pragma once
#include "math.hpp"
#include <algorithm>
#include <array>
#include <limits>
#include <set>
#include <string>
#include <vector>

namespace nereus::ros_viewer::host {
struct ThrusterRotor {
    std::string id, asset, frame;
    size_t inputIndex = 0;
    glm::vec3 pivot{0}, axis{1, 0, 0};
    double direction = 1, angle = 0;

    // Spin about the pivot; mesh vertices and pivot share the visual's authoring frame.
    glm::mat4 transform() const {
        return glm::translate(glm::mat4(1), pivot) * glm::rotate(glm::mat4(1), float(angle), axis) *
               glm::translate(glm::mat4(1), -pivot);
    }
};

struct ThrusterVisuals {
    std::string topic;
    std::vector<ThrusterRotor> rotors;
    double timeout = .5, deadband = .01, speedScale = 1;
    // Propeller law |F| = K_T rho D^4 (rpm/60)^2 with K_T = a + b rpm per direction ([a > 0, b >= 0]).
    std::array<double, 2> forwardKt{}, reverseKt{};
    double ktScale = 0; // rho D^4 / 3600: newtons per (K_T rpm^2)

    ThrusterVisuals() = default;
    // thrusterOrder: thruster ids in force-array order (bridge thrusters.order).
    ThrusterVisuals(const YAML::Node &config, const std::vector<std::string> &thrusterOrder) {
        if (!config || config.IsNull())
            return;
        topic = config["topic"].as<std::string>();
        if (topic.empty() || thrusterOrder.empty())
            throw std::invalid_argument("Thruster visuals need a topic and vehicle thrusters");
        timeout = config["timeout"].as<double>(.5);
        deadband = config["force_deadband"].as<double>(.01);
        speedScale = config["speed_scale"].as<double>(1.);
        for (double value : {timeout, speedScale})
            if (!std::isfinite(value) || value <= 0)
                throw std::invalid_argument("Thruster visual timing/speed settings must be positive and finite");
        if (!std::isfinite(deadband) || deadband < 0)
            throw std::invalid_argument("Thruster visual deadband must be finite and nonnegative");
        const auto propeller = config["propeller"];
        const double diameter = propeller["diameter_m"].as<double>(0.), rho = propeller["water_density"].as<double>(0.);
        if (!std::isfinite(diameter) || diameter <= 0 || !std::isfinite(rho) || rho <= 0)
            throw std::invalid_argument("Thruster propeller needs a positive diameter_m and water_density");
        ktScale = rho * std::pow(diameter, 4) / 3600.;
        for (const auto &key : {"forward", "reverse"}) {
            const auto coefficients = propeller[std::string("thrust_coefficient_") + key];
            if (!coefficients.IsSequence() || coefficients.size() != 2)
                throw std::invalid_argument("Thruster thrust coefficients need [a, b] per direction");
            auto &kt = std::string(key) == "forward" ? forwardKt : reverseKt;
            kt = {coefficients[0].as<double>(), coefficients[1].as<double>()};
            if (!std::isfinite(kt[0]) || !std::isfinite(kt[1]) || kt[0] <= 0 || kt[1] < 0)
                throw std::invalid_argument("Thruster thrust coefficients need a > 0 and b >= 0");
        }
        const auto entries = config["rotors"];
        if (!entries.IsSequence() || entries.size() == 0 || entries.size() > 256)
            throw std::invalid_argument("Thruster visuals must contain 1 to 256 rotors");
        std::set<std::string> ids;
        for (const auto &entry : entries) {
            ThrusterRotor rotor;
            rotor.id = entry["id"].as<std::string>();
            if (rotor.id.empty() || !ids.insert(rotor.id).second)
                throw std::invalid_argument("Thruster rotor IDs must be nonempty and unique");
            const auto thruster = entry["thruster"].as<std::string>(rotor.id);
            const auto found = std::find(thrusterOrder.begin(), thrusterOrder.end(), thruster);
            if (found == thrusterOrder.end())
                throw std::invalid_argument("Thruster rotor '" + rotor.id + "' names unknown thruster " + thruster);
            rotor.inputIndex = size_t(found - thrusterOrder.begin());
            for (const auto &key : {"pivot", "axis"}) {
                const auto vector = entry[key];
                if (!vector.IsSequence() || vector.size() != 3)
                    throw std::invalid_argument("Thruster rotor pivot/axis must contain three numbers");
                for (const auto &value : vector)
                    if (!std::isfinite(value.as<float>()))
                        throw std::invalid_argument("Thruster rotor geometry must be finite");
            }
            rotor.pivot = vec3(entry["pivot"]);
            rotor.axis = vec3(entry["axis"]);
            if (glm::length(rotor.axis) < 1e-6f)
                throw std::invalid_argument("Thruster rotor axis must be nonzero");
            rotor.axis = glm::normalize(rotor.axis);
            rotor.direction = entry["direction"].as<double>(1.);
            if (rotor.direction != 1 && rotor.direction != -1)
                throw std::invalid_argument("Thruster rotor direction must be 1 or -1");
            rotor.asset = entry["asset"].as<std::string>("");
            if (rotor.asset.empty())
                throw std::invalid_argument("Thruster rotor '" + rotor.id + "' needs a robot-pack asset id");
            rotor.frame = entry["frame"].as<std::string>("");
            rotors.push_back(rotor);
        }
        forces.assign(thrusterOrder.size(), 0.f);
    }

    // Inverse of the propeller law: the rotor speed that makes `force`, signed like it.
    double rpm(double force) const {
        if (!std::isfinite(force) || std::abs(force) <= deadband)
            return 0;
        const auto &[a, b] = force > 0 ? forwardKt : reverseKt;
        const double target = std::abs(force) / ktScale;
        // (a + b R) R^2 is convex and increasing for R > 0 and the b = 0 root is at or right of the true one,
        // so Newton descends monotonically onto it.
        double r = std::sqrt(target / a);
        for (int i = 0; i < 50; ++i) {
            const double step = ((a + b * r) * r * r - target) / ((2 * a + 3 * b * r) * r);
            r -= step;
            if (std::abs(step) <= 1e-12 * r)
                break;
        }
        return std::copysign(r, force);
    }

    bool receive(const std::vector<float> &values, double now) {
        if (values.size() != forces.size() || !std::isfinite(now))
            return false;
        for (float force : values)
            if (!std::isfinite(force))
                return false;
        // Integrate the previous sample only up to its replacement time.
        advance(now);
        forces = values;
        receivedAt = now;
        return true;
    }

    void advance(double now) {
        if (!std::isfinite(now))
            return;
        if (!std::isfinite(lastStep) || now < lastStep) {
            if (std::isfinite(lastStep)) {
                for (auto &rotor : rotors)
                    rotor.angle = 0;
                std::fill(forces.begin(), forces.end(), 0.f);
                receivedAt = std::numeric_limits<double>::quiet_NaN();
            }
            lastStep = now;
            return;
        }
        if (std::isfinite(receivedAt)) {
            const double dt = std::max(0., std::min(now, receivedAt + timeout) - std::max(lastStep, receivedAt));
            for (auto &rotor : rotors) {
                const double force = forces[rotor.inputIndex];
                const double speed = rpm(force) * (2. * glm::pi<double>() / 60.) * speedScale;
                rotor.angle = std::remainder(rotor.angle + rotor.direction * speed * dt, 2. * glm::pi<double>());
            }
            if (now >= receivedAt + timeout)
                std::fill(forces.begin(), forces.end(), 0.f);
        }
        lastStep = now;
    }

  private:
    std::vector<float> forces;
    double receivedAt = std::numeric_limits<double>::quiet_NaN();
    double lastStep = std::numeric_limits<double>::quiet_NaN();
};
} // namespace nereus::ros_viewer::host

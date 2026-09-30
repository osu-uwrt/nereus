#include <robotics/session/mechanisms.hpp>

#include "json_util.hpp"

#include <cmath>
#include <limits>
#include <set>

namespace robotics::session {
namespace {
using namespace detail;
constexpr std::int64_t kMaxNs = std::numeric_limits<std::int64_t>::max();

double number(const Json &value, const std::string &name, double minimum = 0) {
    if (!value.is_number())
        throw std::invalid_argument(name + " must be a finite number");
    const double result = value.get<double>();
    if (!std::isfinite(result) || result < minimum) {
        std::string bound = std::isinf(minimum) ? "-inf" : (minimum == 0 ? "0" : std::to_string(minimum));
        throw std::invalid_argument(name + " must be finite and >= " + bound);
    }
    return result;
}
bool boolean(const Json &value, const std::string &name) {
    if (!value.is_boolean())
        throw std::invalid_argument(name + " must be a bool");
    return value.get<bool>();
}
std::int64_t nanoseconds(double seconds) {
    const double value = seconds * 1e9;
    if (!std::isfinite(value) || value > 9223372036854775807.0)
        throw std::invalid_argument("duration exceeds signed 64-bit nanoseconds");
    return static_cast<std::int64_t>(std::nearbyint(value)); // round half to even, as Python round()
}
const Json &at(const Json &object, const char *key) {
    if (!object.is_object() || !object.contains(key))
        throw std::invalid_argument(std::string("missing field '") + key + "'");
    return object.at(key);
}

struct Claw {
    double minimum, maximum, speed, tolerance, initial, q, target;
    int direction{0};
    std::optional<std::int64_t> remaining_ns;
    double travel() const {
        return (maximum - minimum) / 2;
    }
    void stop() {
        target = q;
        direction = 0;
        remaining_ns.reset();
    }
};
} // namespace

struct Mechanisms::Impl {
    std::unique_ptr<spatial::FixedFrames> frames;
    std::map<std::string, Json> release_configs; // launcher/dropper parameters (stable addresses)
    std::map<std::string, std::vector<Pose>> mounts;
    std::map<std::string, std::int64_t> cooldown_ns;
    std::map<std::string, Claw> claws;
    std::map<std::string, std::string> types;
    std::vector<std::string> order;
    bool initial_armed, kill_disarms, reject_arm_killed;
    std::set<std::string> guarded;

    std::int64_t time_ns{0};
    bool armed{false};
    std::map<std::string, int> available;
    std::map<std::string, std::pair<std::int64_t, std::string>> cooldowns; // group -> deadline, owner

    int capacity(const std::string &id) const {
        return release_configs.at(id).at("capacity").get<int>();
    }
    void reset(bool killed) {
        time_ns = 0;
        armed = initial_armed && !(killed && kill_disarms);
        available.clear();
        for (const auto &[key, p] : release_configs)
            available[key] = p.at("capacity").get<int>();
        cooldowns.clear();
        for (auto &[key, claw] : claws) {
            (void)key;
            claw.q = claw.initial;
            claw.stop();
        }
    }
    void kill(bool killed) {
        if (killed && kill_disarms)
            armed = false;
        for (auto &[key, claw] : claws)
            if (killed || (guarded.count(key) && !armed))
                claw.stop();
    }
    bool blocked(const std::string &id, bool killed) const {
        return killed || (guarded.count(id) && !armed);
    }
    CommandResult commandClaw(const std::string &id, bool opened, std::optional<std::int64_t> duration_ns,
                              bool killed) {
        auto &claw = claws.at(id);
        kill(killed);
        if (blocked(id, killed))
            return {false, "Mechanism is disarmed or vehicle is killed"};
        if (duration_ns && *duration_ns == 0) {
            claw.stop();
        } else {
            claw.target = opened ? claw.travel() : 0.;
            claw.direction = opened ? 1 : -1;
            claw.remaining_ns = duration_ns;
        }
        return {true, "Claw command accepted; grasp physics unavailable"};
    }
};

Mechanisms::Mechanisms(const Json &robot) : impl_(std::make_unique<Impl>()) {
    auto &m = *impl_;
    const Json &config = at(robot, "frames");
    std::vector<spatial::FixedFrame> edges;
    for (const auto &entry : at(config, "transforms")) {
        spatial::FixedFrame edge;
        edge.parent = at(entry, "parent").get<std::string>();
        edge.child = at(entry, "child").get<std::string>();
        edge.pose = composeChecked({}, makePose(at(entry, "position_m"), at(entry, "orientation_wxyz"), "transform"));
        edges.push_back(std::move(edge));
    }
    m.frames = std::make_unique<spatial::FixedFrames>(at(config, "root").get<std::string>(), std::move(edges));
    const Json &safety = at(robot, "safety");
    const Json &arming = at(safety, "arming");
    m.initial_armed = boolean(at(arming, "initially_armed"), "initially_armed");
    m.kill_disarms = boolean(at(safety, "kill_disarms_mechanisms"), "kill_disarms_mechanisms");
    m.reject_arm_killed = boolean(at(arming, "arm_rejected_while_killed"), "arm_rejected_while_killed");
    for (const auto &id : at(arming, "applies_to"))
        m.guarded.insert(id.get<std::string>());
    std::set<std::string> ids;
    const Json none = Json::array();
    const Json &list = robot.contains("mechanisms") ? robot.at("mechanisms") : none;
    for (const auto &entry : list) {
        const std::string id = at(entry, "id").get<std::string>();
        const std::string kind = at(entry, "type").get<std::string>();
        const Json &p = at(entry, "parameters");
        if (!ids.insert(id).second)
            throw std::invalid_argument("duplicate mechanism " + repr(id));
        m.types[id] = kind;
        m.order.push_back(id);
        const Pose mount = m.frames->fromRoot(at(entry, "frame").get<std::string>());
        if (kind == "launcher" || kind == "dropper") {
            const Json &slots = at(p, "slots");
            const Json &capacity = at(p, "capacity");
            std::set<std::string> slot_ids;
            for (const auto &s : slots)
                slot_ids.insert(at(s, "id").get<std::string>());
            if (!capacity.is_number_integer() || capacity.get<std::int64_t>() < 1 ||
                capacity.get<std::size_t>() != slots.size() || slot_ids.size() != slots.size())
                throw std::invalid_argument("capacity must match nonempty unique slots");
            m.cooldown_ns[id] = nanoseconds(number(at(p, "cooldown_s"), "cooldown_s"));
            number(at(at(p, "launch"), "spring_energy_j"), "spring_energy_j");
            const Json &projectile = at(p, "projectile");
            for (const char *key : {"mass_kg", "added_mass_kg", "displaced_volume_m3"})
                number(at(projectile, key), key);
            if (projectile.at("mass_kg").get<double>() + projectile.at("added_mass_kg").get<double>() <= 0)
                throw std::invalid_argument("projectile inertial mass must be positive");
            boolean(at(projectile, "neutral_buoyancy"), "neutral_buoyancy");
            if (projectile.contains("center_of_mass_m"))
                number(projectile.at("center_of_mass_m"), "center_of_mass_m", -INFINITY);
            std::vector<Pose> mounts;
            for (const auto &s : slots)
                mounts.push_back(composeChecked(mount, composeChecked({}, makePose(at(s, "position_m"), at(s, "orientation_wxyz"), "slot"))));
            m.mounts[id] = std::move(mounts);
            m.release_configs[id] = p;
        } else if (kind == "claw") {
            Claw claw{};
            claw.minimum = number(at(p, "min_gap_m"), "min_gap_m");
            claw.maximum = number(at(p, "max_gap_m"), "max_gap_m");
            claw.speed = number(at(p, "jaw_speed_m_s"), "jaw_speed_m_s");
            claw.tolerance = number(at(p, "completion_tolerance_m"), "completion_tolerance_m");
            if (claw.maximum <= claw.minimum || claw.speed <= 0 ||
                claw.tolerance >= (claw.maximum - claw.minimum) / 2)
                throw std::invalid_argument("invalid claw travel, speed or completion tolerance");
            const Json &initial = at(p, "initial_state");
            if (!initial.is_string() || (initial != "open" && initial != "closed"))
                throw std::invalid_argument("unsupported initial claw state");
            if (at(p, "timed_command") != "signed_duration")
                throw std::invalid_argument("unsupported timed claw command");
            claw.initial = initial == "open" ? (claw.maximum - claw.minimum) / 2 : 0.;
            claw.q = claw.target = claw.initial;
            m.claws[id] = claw;
        } else if (kind != "magnet" || !(p.contains("actuated") && p.at("actuated") == false)) {
            throw std::invalid_argument("unsupported mechanism type " + repr(kind));
        }
    }
    for (const auto &g : m.guarded)
        if (!ids.count(g))
            throw std::invalid_argument("arming references unknown mechanisms");
    m.reset(boolean(at(safety, "initially_killed"), "initially_killed"));
}
Mechanisms::~Mechanisms() = default;
Mechanisms::Mechanisms(Mechanisms &&) noexcept = default;
Mechanisms &Mechanisms::operator=(Mechanisms &&) noexcept = default;

void Mechanisms::reset(bool killed) {
    impl_->reset(killed);
}

CommandResult Mechanisms::setArmed(bool armed, bool killed) {
    auto &m = *impl_;
    m.kill(killed);
    if (armed && killed && m.reject_arm_killed)
        return {false, "Cannot arm while killed"};
    m.armed = armed;
    m.kill(killed);
    return {true, armed ? "Armed" : "Disarmed"};
}

CommandResult Mechanisms::reloadAll(bool killed) {
    auto &m = *impl_;
    m.kill(killed);
    m.armed = false;
    for (const auto &[key, p] : m.release_configs)
        m.available[key] = p.at("capacity").get<int>();
    m.cooldowns.clear();
    m.kill(killed);
    return {true, "Reloaded and disarmed"};
}

CommandResult Mechanisms::fire(const std::string &id, const Pose &world_from_reference,
                               const Eigen::Vector3d &linear_velocity_reference,
                               const Eigen::Vector3d &angular_velocity_reference,
                               const std::string &reference_frame, double water_density, bool killed,
                               PayloadRelease &release) {
    auto &m = *impl_;
    const auto found = m.release_configs.find(id);
    if (found == m.release_configs.end())
        throw std::out_of_range("unknown release mechanism " + repr(id));
    const Json &p = found->second;
    const Pose world = composeChecked({}, world_from_reference);
    if (!linear_velocity_reference.allFinite())
        throw std::invalid_argument("linear velocity must contain three finite values");
    if (!angular_velocity_reference.allFinite())
        throw std::invalid_argument("angular velocity must contain three finite values");
    if (!std::isfinite(water_density) || water_density < 0)
        throw std::invalid_argument("water_density_kg_m3 must be finite and >= 0");
    if (water_density <= 0)
        throw std::invalid_argument("water density must be positive");
    const Pose root_from_reference = m.frames->fromRoot(reference_frame);
    m.kill(killed);
    if (m.blocked(id, killed))
        return {false, "Mechanism is disarmed or vehicle is killed"};
    const std::string group = p.at("cooldown_group").get<std::string>();
    const auto cooldown = m.cooldowns.find(group);
    if (m.time_ns < (cooldown == m.cooldowns.end() ? 0 : cooldown->second.first))
        return {false, "Release group is busy"};
    if (m.available.at(id) == 0)
        return {false, "No ammunition; reload first"};
    const int capacity = m.capacity(id);
    const int index = capacity - m.available.at(id);
    const Pose mount = composeChecked(inverseChecked(root_from_reference), m.mounts.at(id)[static_cast<std::size_t>(index)]);
    const Pose pose = composeChecked(world, mount);
    const Eigen::Vector3d axis = rotate(pose.rotation, Eigen::Vector3d::UnitX());
    const Json &projectile = p.at("projectile");
    const double mass = projectile.at("neutral_buoyancy").get<bool>()
                            ? water_density * projectile.at("displaced_volume_m3").get<double>()
                            : projectile.at("mass_kg").get<double>();
    const double inertia = mass + projectile.at("added_mass_kg").get<double>();
    if (inertia <= 0)
        throw std::invalid_argument("effective projectile inertial mass must be positive");
    const double speed = std::sqrt(2 * p.at("launch").at("spring_energy_j").get<double>() / inertia);
    const Eigen::Vector3d world_omega = rotate(world.rotation, angular_velocity_reference);
    Eigen::Vector3d world_velocity = rotate(
        world.rotation, linear_velocity_reference + angular_velocity_reference.cross(mount.translation));
    const double com = projectile.contains("center_of_mass_m") ? projectile.at("center_of_mass_m").get<double>() : 0.;
    world_velocity += axis * speed + world_omega.cross(axis * com);
    if (!world_velocity.allFinite())
        throw std::invalid_argument("release velocity is not finite");
    const std::int64_t cooldown_ns = m.cooldown_ns.at(id);
    if (m.time_ns > kMaxNs - cooldown_ns)
        throw std::invalid_argument("cooldown exceeds simulation time range");
    const std::int64_t deadline = m.time_ns + cooldown_ns;
    release.mechanism_id = id;
    release.mechanism_type = m.types.at(id);
    release.slot_id = p.at("slots")[static_cast<std::size_t>(index)].at("id").get<std::string>();
    release.slot_index = index;
    release.time_ns = m.time_ns;
    release.position_world = pose.translation;
    release.orientation_world = pose.rotation;
    release.velocity_com_world = world_velocity;
    release.angular_velocity_world = world_omega;
    release.projectile = &projectile;
    m.available[id] -= 1;
    m.cooldowns[group] = {deadline, id};
    return {true, "Payload released"};
}

CommandResult Mechanisms::commandClaw(const std::string &id, bool open, bool killed) {
    return impl_->commandClaw(id, open, std::nullopt, killed);
}

CommandResult Mechanisms::moveClaw(const std::string &id, double signed_duration_s, bool killed) {
    const double value = number(Json(signed_duration_s), "signed_duration_s", -INFINITY);
    return impl_->commandClaw(id, value > 0, nanoseconds(std::fabs(value)), killed);
}

void Mechanisms::advance(std::int64_t dt_ns, bool killed) {
    auto &m = *impl_;
    if (dt_ns < 0 || m.time_ns > kMaxNs - dt_ns)
        throw std::invalid_argument("dt_ns must be nonnegative and fit the simulation time range");
    m.kill(killed);
    for (auto &[key, claw] : m.claws) {
        (void)key;
        // Timer-before-drive: expiry stops this entire step.
        if (claw.remaining_ns) {
            *claw.remaining_ns -= dt_ns;
            if (*claw.remaining_ns <= 0)
                claw.stop();
        }
        const double distance = claw.speed * static_cast<double>(dt_ns) / 1e9;
        claw.q += std::max(-distance, std::min(distance, claw.target - claw.q));
    }
    m.time_ns += dt_ns;
}

MechanismState Mechanisms::snapshot(bool killed) const {
    auto &m = *impl_; // Python's snapshot also applies the kill state, so state is mutated here
    m.kill(killed);
    MechanismState state;
    state.time_ns = m.time_ns;
    state.armed = m.armed;
    bool busy = false;
    for (const auto &[group, entry] : m.cooldowns) {
        (void)group;
        busy = busy || m.time_ns < entry.first;
    }
    for (const auto &[key, p] : m.release_configs) {
        const auto found = m.cooldowns.find(p.at("cooldown_group").get<std::string>());
        const std::int64_t deadline = found == m.cooldowns.end() ? 0 : found->second.first;
        const std::string owner = found == m.cooldowns.end() ? "" : found->second.second;
        const int available = m.available.at(key);
        const char *name = m.blocked(key, killed) ? "disarmed"
                           : (m.time_ns < deadline && owner == key) ? "busy"
                           : available ? "loaded" : "empty";
        state.releases[key] = {name, available};
    }
    for (const auto &[key, claw] : m.claws) {
        const bool moving = !m.blocked(key, killed) && std::fabs(claw.q - claw.target) > claw.tolerance;
        busy = busy || moving;
        const char *name = m.blocked(key, killed) ? "disarmed"
                           : moving ? (claw.direction > 0 ? "opening" : "closing")
                           : claw.q > claw.travel() - claw.tolerance ? "opened" : "closed";
        state.claws[key] = {name, claw.minimum + 2 * claw.q, claw.minimum + 2 * claw.target, {claw.q, claw.q}};
    }
    state.any_busy = busy;
    return state;
}

int Mechanisms::slotCount(const std::string &id) const {
    return static_cast<int>(impl_->mounts.at(id).size());
}
Pose Mechanisms::slotMount(const std::string &id, int index) const {
    return impl_->mounts.at(id).at(static_cast<std::size_t>(index));
}
const std::string &Mechanisms::type(const std::string &id) const {
    return impl_->types.at(id);
}
std::vector<std::string> Mechanisms::ids() const {
    return impl_->order;
}
} // namespace robotics::session

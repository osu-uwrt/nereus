// createRuntime on the Talos scenario against a recorded reference:
// every derived plant parameter, then 1500 scripted ticks of body and sensor output.
#include "session_test_util.hpp"

#include <gtest/gtest.h>

using namespace robotics::session;
using namespace robotics::session::testing;

namespace {
Json boxJson(const robotics::simulation::BoxProxy &b) {
    return {{"id", b.id}, {"size", flat(b.size)}, {"center", flat(b.center)}, {"orientation", quat(b.orientation)}};
}
} // namespace

TEST(PackRuntime, MatchesReferencePlantParametersFramesAndDynamics) {
    const auto fixture = loadFixture("pack_runtime_reference.json");
    const auto scenario = loadResolvedScenario(NEREUS_RESOLVED_TALOS);
    auto pack = createRuntime(scenario);
    const auto &p = pack.parameters;

    Json thrusters = Json::array(), body_boxes = Json::array(), world_boxes = Json::array();
    for (const auto &t : p.thrusters)
        thrusters.push_back({{"id", t.id},
                             {"position", flat(t.position)},
                             {"direction", flat(t.direction)},
                             {"delay", t.delay},
                             {"rise_time", t.rise_time},
                             {"fall_time", t.fall_time},
                             {"slew_rate", t.slew_rate},
                             {"forward_limit", t.forward_limit},
                             {"reverse_limit", t.reverse_limit},
                             {"deadband", t.deadband},
                             {"forward_scale", t.forward_scale},
                             {"reverse_scale", t.reverse_scale},
                             {"efficiency", t.efficiency},
                             {"propeller_radius", t.propeller_radius ? Json(*t.propeller_radius) : Json()}});
    for (const auto &b : p.contacts.body_boxes)
        body_boxes.push_back(boxJson(b));
    for (const auto &b : p.contacts.world_boxes)
        world_boxes.push_back(boxJson(b));
    const char *model[] = {"ContactModel.DISABLED", "ContactModel.SPHERE_POOL", "ContactModel.BOX_SCENE"};
    Json plant = {{"mass", p.body.mass},
                  {"inertia", flat(p.body.inertia)},
                  {"added_mass", flat(p.body.added_mass)},
                  {"linear_damping", flat(p.body.linear_damping)},
                  {"quadratic_damping", flat(p.body.quadratic_damping)},
                  {"damping_center", flat(p.body.damping_center)},
                  {"displaced_volume", p.body.displaced_volume},
                  {"buoyancy_center", flat(p.body.buoyancy_center)},
                  {"buoyancy_radii", flat(p.body.buoyancy_radii)},
                  {"command_timeout", p.command_timeout},
                  {"timestep_ns", p.timestep.count()},
                  {"pool",
                   {{"origin_xy", flat(p.pool.origin_xy_world)},
                    {"yaw", p.pool.yaw_world},
                    {"length", p.pool.length},
                    {"width", p.pool.width},
                    {"depth", p.pool.depth},
                    {"water_level", p.pool.water_level},
                    {"water_density", p.pool.water_density},
                    {"current", flat(p.pool.current_velocity)},
                    {"amplitude", flat(p.pool.current_oscillation_amplitude)},
                    {"frequency", p.pool.current_oscillation_frequency}}},
                  {"thrusters", thrusters},
                  {"contacts",
                   {{"model", model[static_cast<int>(p.contacts.model)]},
                    {"restitution", p.contacts.restitution},
                    {"friction", p.contacts.friction},
                    {"body_boxes", body_boxes},
                    {"world_boxes", world_boxes}}}};
    // Plant parameters, box geometry and thrusters: exact up to float printing (1e-12).
    EXPECT_EQ(diff(fixture.at("plant"), plant, "$.plant", 1e-12), "");

    Json frames = Json::object();
    for (const auto &[name, expected] : fixture.at("frames").items()) {
        const auto &pose = pack.frames->fromRoot(name);
        frames[name] = {{"translation", flat(pose.translation)}, {"orientation", quat(pose.rotation)}};
    }
    EXPECT_EQ(diff(fixture.at("frames"), frames, "$.frames", 1e-12), "");
    EXPECT_EQ(diff(fixture.at("initial"), bodyJson(pack.initial), "$.initial", 1e-12), "");
    EXPECT_EQ(Json(pack.sensor_ids), fixture.at("sensor_ids"));
    EXPECT_EQ(Json(pack.deferred_sensor_ids), fixture.at("deferred"));

    // Dynamics + sensors incl. noise seeds: kTrajectoryTolerance (in practice bit-exact in Release).
    const auto &forces = fixture.at("forces");
    std::size_t next = 0;
    const auto &checkpoints = fixture.at("checkpoints");
    for (int tick = 1; tick <= 1500; ++tick) {
        if (tick == 1 || tick == 501 || tick == 1001) {
            Eigen::VectorXd f(8);
            for (int i = 0; i < 8; ++i)
                f[i] = forces[(tick - 1) / 500][i].get<double>();
            pack.runtime->command(f);
        }
        const auto snapshot = pack.runtime->advance(1);
        if (tick % 100 == 0) {
            const auto &expected = checkpoints[next++];
            Json actual = {{"tick", tick},
                           {"elapsed_ns", snapshot.elapsed.count()},
                           {"body", bodyJson(snapshot.body)},
                           {"forces", flat(snapshot.thruster_forces)},
                           {"sensors", sensorsJson(*pack.runtime, pack)}};
            ASSERT_EQ(diff(expected, actual, "$.checkpoint[" + std::to_string(next - 1) + "]"), "");
        }
        drain(*pack.runtime, pack);
    }
    EXPECT_EQ(next, checkpoints.size());
}

TEST(PackRuntime, RejectsBadSensorSelections) {
    const auto scenario = loadResolvedScenario(NEREUS_RESOLVED_TALOS);
    std::vector<std::string> duplicate{"imu", "imu"}, unknown{"nope"}, camera{"ffc"};
    EXPECT_THROW(createRuntime(scenario, &duplicate), std::invalid_argument);
    EXPECT_THROW(createRuntime(scenario, &unknown), std::invalid_argument);
    EXPECT_THROW(createRuntime(scenario, &camera), std::invalid_argument); // no native physics
    std::vector<std::string> imu_only{"imu"};
    const auto pack = createRuntime(scenario, &imu_only);
    EXPECT_EQ(pack.sensor_ids, imu_only);
    EXPECT_EQ(pack.deferred_sensor_ids, (std::vector<std::string>{"fog", "dvl", "depth", "ffc", "dfc"}));
}

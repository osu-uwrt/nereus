// Plant behaviour: analytic motion, placement and reset semantics, determinism, validation, sphere-pool and box
// contacts, motion samples and actuator calibration.
#include "nereus/simulation/plant.hpp"
#include <gtest/gtest.h>
#include <limits>

using namespace nereus::simulation;
namespace {
// 2 m deep inside the default 20 x 10 x 5 m pool, at rest.
BodyState initial() {
    BodyState s;
    s.position = {5, 5, -2};
    return s;
}

// Default body (10 kg, neutrally buoyant, undamped) with one undelayed, instantaneous surge thruster at the COM and
// no command watchdog, so the commanded thrust is the applied force.
PlantParameters ideal() {
    PlantParameters p;
    p.command_timeout = 0;
    Thruster t;
    t.id = "surge";
    t.delay = t.rise_time = t.fall_time = t.slew_rate = 0;
    p.thrusters = {t};
    return p;
}

Eigen::VectorXd force(double value) {
    return Eigen::VectorXd::Constant(1, value);
}

// Bit-identical snapshots (generation excluded).
void same(const Snapshot &a, const Snapshot &b) {
    EXPECT_EQ(a.tick, b.tick);
    EXPECT_EQ(a.elapsed, b.elapsed);
    EXPECT_EQ(a.body.position, b.body.position);
    EXPECT_EQ(a.body.orientation.coeffs(), b.body.orientation.coeffs());
    EXPECT_EQ(a.body.linear_velocity, b.body.linear_velocity);
    EXPECT_EQ(a.body.angular_velocity, b.body.angular_velocity);
    EXPECT_EQ(a.thruster_forces, b.thruster_forces);
}
} // namespace

// 10 N on 10 kg for 1 s: v = 1 m/s, x advances 0.5 m.
TEST(Plant, ConstantForceMatchesAnalyticalTranslation) {
    Plant plant(ideal(), initial());
    plant.command(force(10));
    const auto end = plant.advance(500);
    EXPECT_EQ(end.tick, 500U);
    EXPECT_EQ(end.elapsed.count(), 1'000'000'000);
    EXPECT_NEAR(end.body.position.x(), 5.5, 1e-12);
    EXPECT_NEAR(end.body.linear_velocity.x(), 1, 1e-12);
    EXPECT_NEAR(end.body.position.z(), -2, 1e-12);
    EXPECT_NEAR(end.body.angular_velocity.norm(), 0, 1e-12);
}

TEST(Plant, PlacementPreservesTimeAndClearsActiveAndDelayedForces) {
    auto parameters = ideal();
    parameters.thrusters[0].delay = 0.01;
    Plant plant(parameters, initial());
    plant.command(force(10));
    const auto before = plant.advance(20);
    ASSERT_GT(before.thruster_forces[0], 0);

    // A delayed command is still pending at placement; placing drops it and the active force.
    plant.command(force(20));
    auto position = initial();
    position.position.x() = 7;
    const auto placed = plant.place(position);
    EXPECT_EQ(placed.tick, before.tick);
    EXPECT_EQ(placed.elapsed, before.elapsed);
    EXPECT_EQ(placed.generation, before.generation);
    EXPECT_EQ(placed.body.position, position.position);
    EXPECT_EQ(placed.thruster_forces[0], 0);
    EXPECT_EQ(plant.advance(20).thruster_forces[0], 0);

    plant.command(force(3));
    EXPECT_EQ(plant.advance(20).thruster_forces[0], 3);
}

TEST(Plant, PlacementCanPreservePropulsionAndRejectsInvalidStateAtomically) {
    auto parameters = ideal();
    parameters.thrusters[0].delay = 0.01;
    parameters.thrusters[0].rise_time = 0.1;
    Plant plant(parameters, initial()), reference(parameters, initial());
    plant.command(force(10));
    reference.command(force(10));
    plant.advance(20);
    reference.advance(20);
    plant.command(force(20));
    reference.command(force(20));

    // Placing without clearing actuators keeps propulsion in step with the unplaced reference.
    auto placed = plant.observe().body;
    placed.position.x() += 1;
    plant.place(placed, false);
    for (int i = 0; i < 20; ++i)
        EXPECT_EQ(plant.advance().thruster_forces, reference.advance().thruster_forces);

    // A rejected placement leaves the plant untouched.
    const auto before = plant.observe();
    placed.position.x() = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(plant.place(placed), std::invalid_argument);
    same(before, plant.observe());
    EXPECT_EQ(plant.advance().thruster_forces, reference.advance().thruster_forces);
}

TEST(Plant, OffCenterThrusterProducesExpectedTorque) {
    auto p = ideal();
    p.thrusters[0].position.y() = 0.2;
    Plant plant(p, initial());
    // 10 N at y = 0.2 m: -2 N m of yaw torque on unit inertia for one 2 ms tick.
    plant.command(force(10));
    const auto end = plant.advance();
    EXPECT_NEAR(end.body.angular_velocity.z(), -2 * 0.002, 1e-12);
}

TEST(Plant, AddedMassReducesAcceleration) {
    auto p = ideal();
    p.body.added_mass(0, 0) = 10;
    Plant plant(p, initial());
    // 10 kg plus 10 kg added mass halves the acceleration.
    plant.command(force(10));
    EXPECT_NEAR(plant.advance(500).body.linear_velocity.x(), 0.5, 1e-12);
}

TEST(Plant, BatchPartitionDoesNotChangeStateOrTime) {
    auto p = ideal();
    p.thrusters[0].delay = 0.031;
    p.thrusters[0].rise_time = 0.13;
    Plant a(p, initial()), b(p, initial());
    a.command(force(10));
    b.command(force(10));
    a.advance(500);
    for (int i = 0; i < 500; ++i) {
        b.advance();
    }
    same(a.observe(), b.observe());
}

TEST(Plant, ResetClearsPendingCommandsAndRestartsTime) {
    auto p = ideal();
    p.thrusters[0].delay = 0.1;
    Plant a(p, initial()), fresh(p, initial());
    a.command(force(20));
    a.advance(10);
    const auto reset = a.reset(initial());
    EXPECT_EQ(reset.generation, 1U);
    EXPECT_EQ(reset.tick, 0U);
    EXPECT_EQ(reset.thruster_forces.norm(), 0);
    same(a.advance(200), fresh.advance(200));

    // Reset replays deterministically.
    a.reset(initial());
    a.command(force(12));
    const auto first = a.advance(300);
    a.reset(initial());
    a.command(force(12));
    same(first, a.advance(300));
}

TEST(Plant, InstancesAndReturnedSnapshotsAreIsolated) {
    Plant a(ideal(), initial()), b(ideal(), initial());
    auto copy = b.observe();
    copy.body.position.x() = 100;
    a.command(force(10));
    a.advance(100);
    EXPECT_EQ(b.observe().tick, 0U);
    EXPECT_EQ(b.observe().body.position.x(), 5);
    b.advance(100);
    EXPECT_EQ(b.observe().body.position.x(), 5);
    a.reset(initial());
    EXPECT_EQ(b.observe().tick, 100U);
}

TEST(Plant, LastCommandAtBoundaryWins) {
    Plant a(ideal(), initial()), b(ideal(), initial());
    a.command(force(-10));
    a.command(force(10));
    b.command(force(10));
    same(a.advance(100), b.advance(100));
}

TEST(Plant, RejectsBadCommandsAndResetWithoutMutation) {
    Plant a(ideal(), initial()), b(ideal(), initial());
    a.command(force(10));
    b.command(force(10));
    EXPECT_THROW(a.command(Eigen::VectorXd::Zero(2)), std::invalid_argument);
    EXPECT_THROW(a.command(force(std::numeric_limits<double>::quiet_NaN())), std::invalid_argument);

    auto bad = initial();
    bad.position.x() = -1;
    EXPECT_THROW(a.reset(bad), std::invalid_argument);
    same(a.advance(100), b.advance(100));
    EXPECT_EQ(a.observe().generation, 0U);
}

TEST(Plant, RejectsImpossibleConfiguration) {
    auto p = ideal();
    p.body.mass = -1;
    EXPECT_THROW(Plant(p, initial()), std::invalid_argument);
    p = ideal();
    p.thrusters.push_back(p.thrusters[0]);
    EXPECT_THROW(Plant(p, initial()), std::invalid_argument);
    p = ideal();
    p.thrusters[0].direction *= 2;
    EXPECT_THROW(Plant(p, initial()), std::invalid_argument);
    p = ideal();
    p.timestep = std::chrono::nanoseconds(0);
    EXPECT_THROW(Plant(p, initial()), std::invalid_argument);
    p = ideal();
    p.body.collision_radius = 6;
    EXPECT_THROW(Plant(p, initial()), std::invalid_argument);
    p = ideal();
    p.command_timeout = -1;
    EXPECT_THROW(Plant(p, initial()), std::invalid_argument);
    auto s = initial();
    s.orientation.coeffs().setZero();
    EXPECT_THROW(Plant(ideal(), s), std::invalid_argument);
}

TEST(Plant, AdvanceOverflowRejectedBeforeMutation) {
    Plant plant(ideal(), initial());
    const auto before = plant.observe();
    EXPECT_THROW(plant.advance(std::numeric_limits<std::uint64_t>::max()), std::overflow_error);
    same(before, plant.observe());
    same(before, plant.advance(0));
}

TEST(Plant, FloorStopsInwardMotionAndAllowsTangentialMotion) {
    auto s = initial();
    s.position.z() = -4.79;
    s.linear_velocity = {0.1, 0, -1};
    Plant plant(ideal(), s);
    // The 0.2 m sphere settles on the 5 m floor at z = -4.8 m while it keeps sliding in x.
    const auto end = plant.advance(20);
    EXPECT_NEAR(end.body.position.z(), -4.8, 1e-12);
    EXPECT_NEAR(end.body.linear_velocity.z(), 0, 1e-12);
    EXPECT_NEAR(end.body.linear_velocity.x(), 0.1, 1e-12);
    EXPECT_NEAR(end.body.position.x(), 5.004, 1e-12);
}

TEST(Plant, CornerContactsDoNotAddKineticEnergy) {
    auto p = ideal();
    p.body.added_mass(0, 0) = p.body.added_mass(1, 1) = 3;
    p.body.added_mass(0, 1) = p.body.added_mass(1, 0) = 1;
    auto s = initial();
    s.position = {0.201, 0.201, -2};
    s.linear_velocity = {-1, -2, 0};
    const Eigen::Matrix3d mass = 10 * Eigen::Matrix3d::Identity() + p.body.added_mass.topLeftCorner<3, 3>();
    const double before = 0.5 * s.linear_velocity.dot(mass * s.linear_velocity);
    Plant plant(p, s);
    const auto end = plant.advance(10);
    EXPECT_GE(end.body.position.x(), 0.2);
    EXPECT_GE(end.body.position.y(), 0.2);
    EXPECT_GE(end.body.linear_velocity.x(), -1e-10);
    EXPECT_GE(end.body.linear_velocity.y(), -1e-10);
    EXPECT_LE(0.5 * end.body.linear_velocity.dot(mass * end.body.linear_velocity), before);
}

TEST(Plant, WaterSurfaceIsNotACollisionCeiling) {
    auto s = initial();
    s.position.z() = -0.01;
    s.linear_velocity.z() = 5;
    Plant plant(ideal(), s);
    EXPECT_GT(plant.advance(100).body.position.z(), 0.2);
}

TEST(Plant, SupportsPassiveBodiesWithoutThrusters) {
    auto p = ideal();
    p.thrusters.clear();
    Plant plant(p, initial());
    plant.command(Eigen::VectorXd{});
    const auto end = plant.advance(500);
    EXPECT_EQ(end.thruster_forces.size(), 0);
    EXPECT_NEAR((end.body.position - initial().position).norm(), 0, 1e-12);
}

TEST(Plant, NumericalFailureRequiresResetAndKeepsLastSnapshot) {
    auto bad = initial();
    bad.linear_velocity = {1e200, 1e200, 0};
    Plant plant(ideal(), bad);
    const auto before = plant.observe();
    EXPECT_THROW(plant.advance(), std::invalid_argument);
    same(before, plant.observe());
    EXPECT_THROW(plant.advance(), std::logic_error);
    EXPECT_THROW(plant.command(force(1)), std::logic_error);
    plant.reset(initial());
    EXPECT_EQ(plant.advance().tick, 1U);
}

TEST(Motion, InertialAccelerationIsNotBodyVelocityDerivative) {
    auto state = initial();
    state.angular_velocity.z() = 2;
    state.linear_velocity.x() = 3;
    Plant plant({}, state);
    const auto first = plant.motion();
    EXPECT_NEAR(first.acceleration_body.norm(), 0, 1e-12);
    EXPECT_NEAR(first.angular_acceleration_body.norm(), 0, 1e-12);
    EXPECT_EQ(plant.motion().state.tick, 0U);
    EXPECT_EQ(plant.observe().body.position, state.position);
    EXPECT_TRUE(first.acceleration_valid);
}

TEST(Motion, DerivativesIncludeThrustTorqueAndContactValidity) {
    PlantParameters parameters;
    Thruster thruster;
    thruster.id = "motor";
    thruster.position.y() = 0.2;
    thruster.delay = thruster.rise_time = thruster.fall_time = thruster.slew_rate = 0;
    parameters.thrusters = {thruster};
    Plant plant(parameters, initial());
    plant.command(Eigen::VectorXd::Constant(1, 10));
    plant.advance();
    EXPECT_NEAR(plant.motion().acceleration_body.x(), 1, 1e-10);
    EXPECT_NEAR(plant.motion().angular_acceleration_body.z(), -2, 1e-10);

    // Resting on the floor makes the instantaneous acceleration invalid (impulsive contact).
    auto contact = initial();
    contact.position.z() = -parameters.pool.depth + parameters.body.collision_radius;
    plant.reset(contact);
    EXPECT_FALSE(plant.motion().acceleration_valid);
    plant.reset(initial());
    EXPECT_TRUE(plant.motion().acceleration_valid);
    EXPECT_NEAR(plant.motion().acceleration_body.norm(), 0, 1e-12);
}

TEST(Plant, ActuatorCalibrationOrdersDeadbandScaleSaturationAndEfficiency) {
    auto p = ideal();
    auto &t = p.thrusters.front();
    t.deadband = 2;
    t.forward_scale = 2;
    t.reverse_scale = .5;
    t.forward_limit = 8;
    t.reverse_limit = 2;
    t.efficiency = .5;
    Plant plant(p, initial());
    // (command, force): deadband 2 N, scale x2 forward / x0.5 reverse, limits 8 / 2 N, then efficiency 0.5.
    for (const auto &command : std::vector<std::pair<double, double>>{{1.9, 0}, {2, 2}, {5, 4}, {-5, -1}}) {
        plant.command(force(command.first));
        EXPECT_DOUBLE_EQ(plant.advance().thruster_forces[0], command.second);
    }

    t.efficiency = 1.1;
    EXPECT_THROW(Plant(p, initial()), std::invalid_argument);
    t.efficiency = .5;
    t.forward_scale = -1;
    EXPECT_THROW(Plant(p, initial()), std::invalid_argument);
}

TEST(Plant, StopCancelsDelayedCommandsAndAllowsSubsequentExplicitCommands) {
    auto p = ideal();
    p.thrusters.front().delay = .05;
    Plant plant(p, initial());
    plant.command(force(10));
    const auto before = plant.observe();
    plant.stopThrusters();
    plant.stopThrusters();
    same(before, plant.observe());
    EXPECT_DOUBLE_EQ(plant.advance(100).thruster_forces[0], 0);
    plant.command(force(4));
    EXPECT_DOUBLE_EQ(plant.advance(50).thruster_forces[0], 4);
}

TEST(Plant, ContactSelectionDoesNotImposeUnselectedSphereConstraints) {
    auto p = ideal();
    p.body.collision_radius = -1; // Irrelevant to explicitly disabled or box contacts.
    auto state = initial();
    state.position = {-2, -2, -10};
    p.contacts.model = ContactModel::Disabled;
    Plant disabled(p, state);
    EXPECT_EQ(disabled.advance().body.position, state.position);
    p.contacts.model = ContactModel::BoxScene;
    Plant empty_scene(p, state);
    EXPECT_EQ(empty_scene.advance().body.position, state.position);
    EXPECT_NO_THROW(empty_scene.reset(state));
    p.contacts.model = ContactModel::SpherePool;
    EXPECT_THROW(Plant(p, state), std::invalid_argument);
    p.contacts.model = static_cast<ContactModel>(-1);
    EXPECT_THROW(Plant(p, state), std::invalid_argument);
}

TEST(Plant, BoxScenePermitsInitialDepenetrationAndResetWithoutHiddenState) {
    auto p = ideal();
    p.contacts.model = ContactModel::BoxScene;
    p.contacts.body_boxes = {{"hull", {.4, .4, .4}, {0, 0, 0}}};
    p.contacts.world_boxes = {{"floor", {10, 10, 1}, {0, 0, -.5}}};
    auto state = initial();
    state.position = {0, 0, .1};
    state.linear_velocity = {.1, .2, -.3};
    Plant plant(p, state);
    EXPECT_EQ(plant.observe().body.position, state.position); // Correction belongs to stepping.
    const auto first = plant.advance(10);
    EXPECT_GT(first.body.position.z(), state.position.z());
    EXPECT_NEAR(first.body.orientation.norm(), 1, 1e-14);
    EXPECT_TRUE(plant.motion().acceleration_valid); // Post-impulse derivative; excludes impulse.
    plant.reset(state);
    same(first, plant.advance(10));
}

TEST(Plant, PlacedSpherePoolMatchesTransformedCornerContactAndHydrostatics) {
    PlantParameters local;
    local.body.added_mass = Matrix6::Identity();
    local.body.added_mass(0, 4) = local.body.added_mass(4, 0) = .2;
    BodyState start;
    start.position = {.201, .201, -4.79};
    start.linear_velocity = {-.4, -.3, -.2};

    // The same motion in a pool moved and yawed in the world must match after the rigid transform.
    auto placed = local;
    placed.pool.origin_xy_world = {-10, -12};
    placed.pool.yaw_world = .7;
    placed.pool.water_level = 3;
    const Eigen::Quaterniond rotation(Eigen::AngleAxisd(.7, Eigen::Vector3d::UnitZ()));
    const Eigen::Vector3d translation(-10, -12, 3);
    auto transformed = start;
    transformed.position = translation + rotation * start.position;
    transformed.orientation = rotation * start.orientation;
    Plant a(local, start), b(placed, transformed);
    for (int i = 0; i < 200; ++i) {
        const auto x = a.advance(), y = b.advance();
        EXPECT_TRUE(y.body.position.isApprox(translation + rotation * x.body.position, 1e-11));
        EXPECT_TRUE(y.body.orientation.coeffs().isApprox((rotation * x.body.orientation).coeffs(), 1e-11));
        EXPECT_NEAR((y.body.linear_velocity - x.body.linear_velocity).norm(), 0, 1e-10);
        EXPECT_NEAR((y.body.angular_velocity - x.body.angular_velocity).norm(), 0, 1e-10);
        EXPECT_EQ(a.motion().acceleration_valid, b.motion().acceleration_valid);
    }

    EXPECT_NO_THROW(b.reset(transformed));
    EXPECT_THROW(b.reset(start), std::invalid_argument);
    placed.pool.yaw_world = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW((Plant(placed, transformed)), std::invalid_argument);
}

TEST(Plant, PlacedPoolKeepsExactBoundaryLegalAndRejectsExterior) {
    for (double yaw : {.3, .7, 1.1}) {
        PlantParameters params;
        params.pool.origin_xy_world = {-10, -12};
        params.pool.yaw_world = yaw;
        const Eigen::Quaterniond rotation(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        auto state = initial();
        // Exactly on the radius-inset corner boundary is legal despite transform rounding; just outside is not.
        state.position = Eigen::Vector3d(-10, -12, 0) + rotation * Eigen::Vector3d(.2, .2, -2);
        EXPECT_NO_THROW((Plant(params, state)));
        state.position -= rotation * Eigen::Vector3d(1e-8, 0, 0);
        EXPECT_THROW((Plant(params, state)), std::invalid_argument);
    }
}

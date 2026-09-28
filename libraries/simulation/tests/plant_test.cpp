#include "robotics/simulation/plant.hpp"
#include <gtest/gtest.h>
#include <limits>

using namespace robotics::simulation;
namespace {
BodyState initial() {
    BodyState s;
    s.position = {5, 5, -2};
    return s;
}
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

TEST(Plant, OffCenterThrusterProducesExpectedTorque) {
    auto p = ideal();
    p.thrusters[0].position.y() = 0.2;
    Plant plant(p, initial());
    plant.command(force(10));
    const auto end = plant.advance();
    EXPECT_NEAR(end.body.angular_velocity.z(), -2 * 0.002, 1e-12);
}

TEST(Plant, AddedMassReducesAcceleration) {
    auto p = ideal();
    p.body.added_mass(0, 0) = 10;
    Plant plant(p, initial());
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
    const Eigen::Matrix3d mass =
        10 * Eigen::Matrix3d::Identity() + p.body.added_mass.topLeftCorner<3, 3>();
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
    for (const auto &command :
         std::vector<std::pair<double, double>>{{1.9, 0}, {2, 2}, {5, 4}, {-5, -1}}) {
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

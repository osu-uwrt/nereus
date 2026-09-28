#include "robotics/sensors/models.hpp"
#include "robotics/sensors/runtime.hpp"
#include <array>
#include <cmath>
#include <gtest/gtest.h>
#include <limits>

using namespace robotics::sensors;
namespace sim = robotics::simulation;
using namespace std::chrono_literals;
namespace {
sim::BodyState initial() {
    sim::BodyState state;
    state.position = {5, 5, -2};
    return state;
}
sim::MotionSample motion() {
    sim::MotionSample result;
    result.state.body = initial();
    return result;
}
Device device(std::string id = "imu") {
    return {std::move(id), "sensor", 5ms, 3ms, 64};
}
NoiseParameters noisy() {
    NoiseParameters parameters;
    parameters.bias = {0.1, -0.2, 0.3};
    parameters.white_stddev.setConstant(0.02);
    parameters.walk_stddev.setConstant(0.01);
    return parameters;
}
void same(const std::vector<Sample<ImuReading>> &left,
          const std::vector<Sample<ImuReading>> &right) {
    ASSERT_EQ(left.size(), right.size());
    for (std::size_t i = 0; i < left.size(); ++i) {
        EXPECT_EQ(left[i].header.acquired, right[i].header.acquired);
        EXPECT_EQ(left[i].header.delivered, right[i].header.delivered);
        EXPECT_EQ(left[i].header.sequence, right[i].header.sequence);
        ASSERT_TRUE(left[i].measurement.value);
        ASSERT_TRUE(right[i].measurement.value);
        EXPECT_EQ(left[i].measurement.value->specific_force,
                  right[i].measurement.value->specific_force);
        EXPECT_EQ(left[i].measurement.value->angular_velocity,
                  right[i].measurement.value->angular_velocity);
        EXPECT_EQ(left[i].measurement.value->force_covariance,
                  right[i].measurement.value->force_covariance);
    }
}
// A model-owned payload verifies extension without modifying a core enum or hierarchy.
struct Probe {
    using Reading = double;
    void reset(std::uint64_t, const std::string &) {}
    Measurement<Reading> sample(const sim::MotionSample &, double elapsed) {
        return {elapsed, {}};
    }
};
} // namespace

TEST(Imu, SpecificForceAtRestAndFreeFall) {
    Imu imu;
    auto input = motion();
    auto rest = imu.sample(input, 0.01);
    ASSERT_TRUE(rest.value);
    EXPECT_EQ(rest.value->specific_force, Eigen::Vector3d(0, 0, 9.80665));
    EXPECT_EQ(rest.value->angular_velocity, Eigen::Vector3d::Zero());
    EXPECT_EQ(rest.value->force_covariance, Eigen::Matrix3d::Zero());
    input.acceleration_body = input.gravity_world;
    EXPECT_EQ(imu.sample(input, 0.01).value->specific_force, Eigen::Vector3d::Zero());
}

TEST(Imu, RotatedMountIncludesTangentialAndCentripetalAcceleration) {
    Mount mount;
    mount.position_body.x() = 2;
    mount.sensor_to_body = Eigen::AngleAxisd(std::acos(-1.0) / 2, Eigen::Vector3d::UnitY());
    Imu imu(mount);
    auto input = motion();
    input.state.body.orientation = Eigen::AngleAxisd(std::acos(-1.0) / 2, Eigen::Vector3d::UnitX());
    input.state.body.angular_velocity = {0, 0, 3};
    input.angular_acceleration_body = {0, 0, 4};
    input.acceleration_body = {1, 2, 3};
    const auto sample = imu.sample(input, 0.01).value.value();
    // Body inertial acceleration=(-17,10,3), gravity=(0,-g,0); mount maps (x,y,z) to (-z,y,x).
    EXPECT_TRUE(sample.specific_force.isApprox(Eigen::Vector3d(-3, 19.80665, -17), 1e-12));
    EXPECT_TRUE(sample.angular_velocity.isApprox(Eigen::Vector3d(-3, 0, 0), 1e-12));
}

TEST(Imu, ContactIsUnavailableWithoutFreezingNoiseHistory) {
    Imu interrupted({}, noisy()), continuous({}, noisy());
    interrupted.reset(1, "imu");
    continuous.reset(1, "imu");
    auto input = motion();
    input.acceleration_valid = false;
    const auto missing = interrupted.sample(input, 0.1);
    EXPECT_FALSE(missing.value);
    EXPECT_FALSE(missing.unavailable_reason.empty());
    continuous.sample(motion(), 0.1);
    EXPECT_EQ(interrupted.sample(motion(), 0.1).value->specific_force,
              continuous.sample(motion(), 0.1).value->specific_force);
}

TEST(Fog, ProjectsSignedRatesAndCovarianceOntoConfiguredAxes) {
    Mount mount;
    mount.sensor_to_body = Eigen::AngleAxisd(std::acos(-1.0) / 2, Eigen::Vector3d::UnitZ());
    NoiseParameters noise;
    noise.white_stddev = {1, 2, 3};
    Fog ideal(mount,
              {Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitZ()});
    auto input = motion();
    input.state.body.angular_velocity = {2, -3, 4};
    const auto result = ideal.sample(input, 0.01).value.value();
    EXPECT_TRUE(result.angular_rates.isApprox(Eigen::Vector3d(-3, -2, -4), 1e-12));
    Fog uncertain({}, {Eigen::Vector3d::UnitX(), Eigen::Vector3d(1, 1, 0).normalized()}, noise);
    auto covariance = uncertain.sample(input, 0.01).value->covariance;
    EXPECT_NEAR(covariance(0, 0), 1, 1e-12);
    EXPECT_NEAR(covariance(1, 1), 2.5, 1e-12);
    EXPECT_NEAR(covariance(0, 1), std::sqrt(0.5), 1e-12);
}

TEST(Dvl, BottomVelocityUsesMountLeverArmAndSensorFrame) {
    DvlParameters parameters;
    parameters.mount.position_body.x() = 1;
    parameters.mount.sensor_to_body =
        Eigen::AngleAxisd(std::acos(-1.0) / 2, Eigen::Vector3d::UnitZ());
    Dvl dvl(parameters, [](const Eigen::Vector3d &origin, const Eigen::Vector3d &direction) {
        EXPECT_TRUE(origin.isApprox(Eigen::Vector3d(5, 6, -2), 1e-12));
        EXPECT_TRUE(direction.isApprox(-Eigen::Vector3d::UnitZ(), 1e-12));
        return std::optional<BottomHit>{{3, Eigen::Vector3d(0, 1, 0)}};
    });
    auto input = motion();
    input.state.body.orientation = Eigen::AngleAxisd(std::acos(-1.0) / 2, Eigen::Vector3d::UnitZ());
    input.state.body.linear_velocity = {2, 3, 0};
    input.state.body.angular_velocity = {0, 0, 2};
    const auto result = dvl.sample(input, 0.01).value.value();
    EXPECT_TRUE(result.bottom_relative_velocity.isApprox(Eigen::Vector3d(5, -1, 0), 1e-12));
    EXPECT_DOUBLE_EQ(result.bottom_distance, 3);
}

TEST(Dvl, FinitePoolRangeAndMissingBottomHaveExplicitLockLoss) {
    PoolBottom bottom(sim::Pool{});
    DvlParameters parameters;
    parameters.maximum_range = 2;
    Dvl dvl(parameters, bottom);
    auto result = dvl.sample(motion(), 0.01);
    EXPECT_FALSE(result.value);
    EXPECT_EQ(result.unavailable_reason, "bottom out of range");
    parameters.maximum_range = 3;
    Dvl locked(parameters, bottom);
    EXPECT_DOUBLE_EQ(locked.sample(motion(), 0.01).value->bottom_distance, 3);
    EXPECT_FALSE(bottom({5, 5, -2}, Eigen::Vector3d::UnitZ()));
    EXPECT_FALSE(bottom({5, 5, 1}, -Eigen::Vector3d::UnitZ()));
    EXPECT_FALSE(bottom({5, 5, -6}, -Eigen::Vector3d::UnitZ()));
    EXPECT_FALSE(bottom({-1, 5, -2}, -Eigen::Vector3d::UnitZ()));
    EXPECT_FALSE(bottom({19, 5, -2}, Eigen::Vector3d(1, 0, -1).normalized()));
    auto inverted = motion();
    inverted.state.body.orientation = Eigen::AngleAxisd(std::acos(-1.0), Eigen::Vector3d::UnitX());
    EXPECT_EQ(locked.sample(inverted, 0.01).unavailable_reason, "no bottom intersection");
    Dvl invalid(
        {}, [](const auto &, const auto &) { return std::optional<BottomHit>{{-1, {0, 0, 0}}}; });
    EXPECT_THROW(invalid.sample(motion(), 0.01), std::runtime_error);
}

TEST(Noise, SeedIdentityResetAndCovarianceAreExplicit) {
    Noise3 a(noisy()), b(noisy()), other(noisy());
    a.reset(42, "gyro", "rate");
    b.reset(42, "gyro", "rate");
    other.reset(42, "other", "rate");
    const auto first = a.sample(0.1);
    EXPECT_EQ(first, b.sample(0.1));
    EXPECT_NE(first, other.sample(0.1));
    a.sample(0.4);
    EXPECT_NEAR(a.covariance()(0, 0), 0.0004 + 0.5 * 0.0001, 1e-15);
    a.reset(42, "gyro", "rate");
    EXPECT_EQ(first, a.sample(0.1));
    a.reset(43, "gyro", "rate");
    EXPECT_NE(first, a.sample(0.1));
    a.reset(42, "gyr", "orate");
    EXPECT_NE(first, a.sample(0.1));
    EXPECT_THROW(a.sample(0), std::invalid_argument);
}

TEST(Sensors, RejectInvalidConfiguration) {
    Mount invalid;
    invalid.sensor_to_body.coeffs().setZero();
    EXPECT_THROW(Imu{invalid}, std::invalid_argument);
    EXPECT_THROW((Fog{{}, {}}), std::invalid_argument);
    EXPECT_THROW((Fog{{}, {Eigen::Vector3d(2, 0, 0)}}), std::invalid_argument);
    NoiseParameters noise;
    noise.walk_stddev.x() = -1;
    EXPECT_THROW(Noise3{noise}, std::invalid_argument);
    noise.walk_stddev.x() = std::numeric_limits<double>::infinity();
    EXPECT_THROW(Noise3{noise}, std::invalid_argument);
    DvlParameters parameters;
    parameters.maximum_range = parameters.minimum_range;
    EXPECT_THROW((Dvl{parameters, PoolBottom(sim::Pool{})}), std::invalid_argument);
    EXPECT_THROW((Dvl{{}, {}}), std::invalid_argument);
    auto input = motion();
    input.state.body.linear_velocity.x() = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(Imu{}.sample(input, 0.01), std::invalid_argument);
}

TEST(Scheduler, NonIntegralPeriodsDoNotDriftAndLatencyIsSeparate) {
    Runtime runtime({}, initial());
    auto stream = runtime.add(device(), Probe{});
    EXPECT_FALSE(stream->latest());
    runtime.advance(13); // 26ms, period 5ms, latency 3ms, physics 2ms.
    const auto samples = stream->drain();
    ASSERT_EQ(samples.size(), 4U);
    const std::array<int, 4> acquired{6, 10, 16, 20}, delivered{10, 14, 20, 24}, dt{6, 4, 6, 4};
    for (std::size_t i = 0; i < samples.size(); ++i) {
        EXPECT_EQ(samples[i].header.sequence, i);
        EXPECT_EQ(samples[i].header.scheduled, (i + 1) * 5ms);
        EXPECT_EQ(samples[i].header.acquired, acquired[i] * 1ms);
        EXPECT_EQ(samples[i].header.delivered, delivered[i] * 1ms);
        EXPECT_DOUBLE_EQ(samples[i].measurement.value.value(), dt[i] * 0.001);
    }
    EXPECT_EQ(stream->stats().acquired, 5U);
    EXPECT_EQ(stream->stats().delivered, 4U);
    EXPECT_EQ(stream->latest()->header.sequence, 3U);
    EXPECT_TRUE(stream->drain().empty());
}

TEST(Scheduler, BatchPartitionPollingAndOtherDevicesCannotChangeNoise) {
    Runtime a({}, initial(), 42), b({}, initial(), 42);
    auto left = a.add(device(), Imu({}, noisy(), noisy()));
    auto extra_device = device("extra");
    extra_device.period = 3ms;
    extra_device.capacity = 128;
    b.add(extra_device, Fog{});
    auto right = b.add(device(), Imu({}, noisy(), noisy()));
    a.advance(100);
    std::vector<Sample<ImuReading>> right_samples;
    for (int i = 0; i < 100; ++i) {
        b.advance();
        right->latest();
        b.observe();
        for (auto &sample : right->drain()) {
            right_samples.push_back(std::move(sample));
        }
    }
    same(left->drain(), right_samples);
}

TEST(Scheduler, ResetClearsPendingAndReadyDataAndRestartsSeed) {
    Runtime runtime({}, initial(), 42);
    auto stream = runtime.add(device(), Imu({}, noisy(), noisy()));
    runtime.advance(13);
    const auto first = stream->drain();
    EXPECT_TRUE(stream->latest());
    EXPECT_EQ(runtime.reset(initial(), 42).generation, 1U);
    EXPECT_TRUE(stream->active());
    EXPECT_FALSE(stream->latest());
    EXPECT_EQ(stream->stats().acquired, 0U);
    runtime.advance(13);
    auto replay = stream->drain();
    same(first, replay);
    for (const auto &sample : replay) {
        EXPECT_EQ(sample.header.generation, 1U);
    }
    EXPECT_THROW(runtime.add(device("late"), Imu{}), std::logic_error);
}

TEST(Scheduler, QueueOverflowFaultsAndInvalidatesUntilReset) {
    Runtime runtime({}, initial());
    auto config = device();
    config.period = 2ms;
    config.latency = 0ns;
    config.capacity = 1;
    auto stream = runtime.add(config, Imu{});
    runtime.advance();
    EXPECT_TRUE(stream->latest());
    EXPECT_THROW(runtime.advance(), std::runtime_error);
    EXPECT_TRUE(runtime.faulted());
    EXPECT_FALSE(stream->active());
    EXPECT_FALSE(stream->latest());
    EXPECT_TRUE(stream->drain().empty());
    EXPECT_THROW(runtime.advance(), std::logic_error);
    EXPECT_THROW(runtime.command(Eigen::VectorXd{}), std::logic_error);
    runtime.reset(initial(), 0);
    runtime.advance();
    EXPECT_TRUE(stream->latest());
}

TEST(Scheduler, ExplicitDropOldestCountsBothQueueTypes) {
    Runtime runtime({}, initial());
    auto config = device();
    config.period = 2ms;
    config.latency = 0ns;
    config.capacity = 1;
    config.overflow = OverflowPolicy::DropOldest;
    auto ready = runtime.add(config, Imu{});
    config.id = "pending";
    config.latency = 10ms;
    auto pending = runtime.add(config, Imu{});
    runtime.advance(5);
    EXPECT_EQ(ready->stats().dropped_delivered, 4U);
    EXPECT_EQ(ready->drain().front().header.sequence, 4U);
    EXPECT_EQ(pending->stats().dropped_pending, 4U);
    EXPECT_FALSE(pending->latest());
}

TEST(Scheduler, ValidatesRegistrationAndEmptyRuntime) {
    Runtime runtime({}, initial());
    EXPECT_EQ(runtime.advance(0).tick, 0U);
    auto config = device();
    config.period = 1ms;
    EXPECT_THROW(runtime.add(config, Imu{}), std::invalid_argument);
    config = device();
    config.latency = -1ns;
    EXPECT_THROW(runtime.add(config, Imu{}), std::invalid_argument);
    config = device();
    config.capacity = 0;
    EXPECT_THROW(runtime.add(config, Imu{}), std::invalid_argument);
    auto stream = runtime.add(device(), Imu{});
    EXPECT_THROW(runtime.add(device(), Imu{}), std::invalid_argument);
    EXPECT_THROW(runtime.advance(std::numeric_limits<std::uint64_t>::max()), std::overflow_error);
    EXPECT_FALSE(runtime.faulted());
    EXPECT_TRUE(stream->active());
    Runtime empty({}, initial());
    EXPECT_EQ(empty.advance(10).tick, 10U);
}

TEST(Scheduler, HandlesOutliveRuntimeAndSamplesAreIndependentCopies) {
    std::shared_ptr<SensorStream<ImuReading>> stream;
    {
        Runtime runtime({}, initial());
        stream = runtime.add(device(), Imu{});
        runtime.advance(5);
        auto copy = stream->latest();
        ASSERT_TRUE(copy);
        copy->measurement.value->specific_force.setZero();
        EXPECT_GT(stream->latest()->measurement.value->specific_force.norm(), 9);
    }
    EXPECT_FALSE(stream->active());
    EXPECT_FALSE(stream->latest());
    EXPECT_TRUE(stream->drain().empty());
}

TEST(Scheduler, ReportsUnavailableSamplesAndRejectsBrokenExtensionResults) {
    Runtime runtime({}, initial());
    DvlParameters parameters;
    parameters.maximum_range = 1;
    auto stream = runtime.add(device("dvl"), Dvl(parameters, PoolBottom(sim::Pool{})));
    runtime.advance(5);
    ASSERT_TRUE(stream->latest());
    EXPECT_FALSE(stream->latest()->measurement.value);
    EXPECT_EQ(stream->stats().unavailable, 2U);
    struct Broken {
        using Reading = int;
        void reset(std::uint64_t, const std::string &) {}
        Measurement<int> sample(const sim::MotionSample &, double) {
            return {};
        }
    };
    Runtime broken({}, initial());
    broken.add(device(), Broken{});
    EXPECT_THROW(broken.advance(5), std::logic_error);
    EXPECT_TRUE(broken.faulted());
}

TEST(Scheduler, PendingOverflowAndInvalidResetPreserveDefinedLifecycle) {
    Runtime runtime({}, initial());
    auto config = device();
    config.period = 2ms;
    config.latency = 100ms;
    config.capacity = 1;
    auto stream = runtime.add(config, Imu{});
    runtime.advance();
    auto invalid = initial();
    invalid.position.x() = -1;
    EXPECT_THROW(runtime.reset(invalid, 99), std::invalid_argument);
    EXPECT_EQ(runtime.observe().tick, 1U);
    EXPECT_TRUE(stream->active());
    EXPECT_EQ(stream->stats().acquired, 1U);
    EXPECT_THROW(runtime.advance(), std::runtime_error);
    EXPECT_TRUE(runtime.faulted());
    EXPECT_FALSE(stream->active());
    runtime.reset(initial(), 0);
    EXPECT_FALSE(runtime.faulted());
    EXPECT_EQ(stream->stats().acquired, 0U);
}

TEST(Scheduler, ExtensionResetFailureInvalidatesEveryStreamAndCanRecover) {
    struct ThrowOnSeed {
        using Reading = double;
        void reset(std::uint64_t seed, const std::string &) {
            if (seed == 99) {
                throw std::runtime_error("reset failed");
            }
        }
        Measurement<double> sample(const sim::MotionSample &, double) {
            return {1, {}};
        }
    };
    Runtime runtime({}, initial());
    auto imu = runtime.add(device(), Imu{});
    auto custom = runtime.add(device("extension"), ThrowOnSeed{});
    runtime.advance(5);
    EXPECT_THROW(runtime.reset(initial(), 99), std::runtime_error);
    EXPECT_TRUE(runtime.faulted());
    EXPECT_FALSE(imu->active());
    EXPECT_FALSE(custom->active());
    EXPECT_TRUE(imu->drain().empty());
    EXPECT_TRUE(custom->drain().empty());
    runtime.reset(initial(), 0);
    runtime.advance(5);
    ASSERT_TRUE(custom->latest());
    EXPECT_EQ(custom->latest()->header.generation, 2U);
}

TEST(Scheduler, DeliveryTimeOverflowFaultsBeforePublishingInvalidTimestamps) {
    Runtime runtime({}, initial());
    auto config = device();
    config.latency = Nanoseconds::max();
    auto stream = runtime.add(config, Probe{});
    EXPECT_THROW(runtime.advance(3), std::overflow_error);
    EXPECT_TRUE(runtime.faulted());
    EXPECT_FALSE(stream->latest());
}

TEST(Dvl, AcceptedMountRoundoffDoesNotInvalidateBottomQuery) {
    DvlParameters parameters;
    parameters.mount.sensor_to_body = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitX());
    parameters.mount.sensor_to_body.coeffs() *= 1.0 + 0.9e-9;
    parameters.bottom_axis *= 1.0 + 0.9e-9;
    Dvl dvl(parameters, PoolBottom(sim::Pool{}));
    const auto result = dvl.sample(motion(), 0.01);
    ASSERT_TRUE(result.value);
    EXPECT_NEAR(result.value->bottom_distance, 3.0 / std::cos(0.5), 1e-12);
}

TEST(Pressure, HydrostaticDepthUsesMountedPositionAndMeasuredPressure) {
    PressureParameters parameters;
    parameters.mount.position_body.x() = 0.5;
    Pressure pressure(parameters, HydrostaticPressure(1));
    auto input = motion();
    input.state.body.orientation = Eigen::AngleAxisd(std::acos(-1.0) / 2, Eigen::Vector3d::UnitY());
    const auto sample = pressure.sample(input, 0.1).value.value();
    EXPECT_NEAR(sample.absolute_pressure, 101325 + 1000 * 9.80665 * 3.5, 1e-9);
    EXPECT_NEAR(sample.depth, 3.5, 1e-12);
    EXPECT_DOUBLE_EQ(sample.depth_variance, 0);
    parameters.noise.bias = 9806.65;
    Pressure biased(parameters, HydrostaticPressure(1));
    EXPECT_NEAR(biased.sample(input, 0.1).value->depth, 4.5, 1e-12);
    parameters.reference_density = 2000;
    Pressure calibrated(parameters, HydrostaticPressure(1));
    EXPECT_NEAR(calibrated.sample(input, 0.1).value->depth, 2.25, 1e-12);
}

TEST(Pressure, SurfaceAndAirReturnAtmosphericPressureWithoutClampingEstimatedDepth) {
    PressureParameters parameters;
    parameters.reference_pressure = 102325;
    Pressure pressure(parameters, HydrostaticPressure(0));
    auto input = motion();
    for (double height : {0.0, 2.0}) {
        input.state.body.position.z() = height;
        const auto result = pressure.sample(input, 0.1).value.value();
        EXPECT_DOUBLE_EQ(result.absolute_pressure, 101325);
        EXPECT_NEAR(result.depth, -1000.0 / 9806.65, 1e-12);
    }
}

TEST(Pressure, NoiseVarianceAndRuntimeResetAreReproducible) {
    PressureParameters parameters;
    parameters.noise = {2, 3, 4};
    Runtime runtime({}, initial(), 42);
    auto stream = runtime.add(device("pressure"), Pressure(parameters, HydrostaticPressure(0)));
    runtime.advance(5);
    auto first = stream->drain();
    ASSERT_EQ(first.size(), 1U);
    const auto expected = first[0].measurement.value.value();
    EXPECT_NEAR(expected.pressure_variance, 9 + 16 * 0.006, 1e-12);
    EXPECT_NEAR(expected.depth_variance, expected.pressure_variance / (9806.65 * 9806.65), 1e-16);
    runtime.reset(initial(), 42);
    EXPECT_FALSE(stream->latest());
    runtime.advance(5);
    EXPECT_DOUBLE_EQ(stream->latest()->measurement.value->absolute_pressure,
                     expected.absolute_pressure);
    EXPECT_EQ(stream->latest()->header.generation, 1U);
}

TEST(Pressure, MissingEnvironmentRangeAndInvalidProviderAreDistinct) {
    PressureParameters parameters;
    parameters.maximum_pressure = 110000;
    Pressure outside(parameters, HydrostaticPressure(0));
    EXPECT_EQ(outside.sample(motion(), 0.1).unavailable_reason, "pressure out of range");
    Pressure missing({}, [](const auto &) -> std::optional<double> { return std::nullopt; });
    EXPECT_EQ(missing.sample(motion(), 0.1).unavailable_reason, "pressure environment unavailable");
    Pressure broken({}, [](const auto &) { return std::optional<double>(-1); });
    EXPECT_THROW(broken.sample(motion(), 0.1), std::runtime_error);
    EXPECT_THROW((Pressure{{}, {}}), std::invalid_argument);
    parameters.reference_density = 0;
    EXPECT_THROW((Pressure{parameters, HydrostaticPressure(0)}), std::invalid_argument);
    EXPECT_THROW((HydrostaticPressure{0, -1}), std::invalid_argument);
}

TEST(Scheduler, StopCoastsPropulsionWithoutResettingClockOrSensorSchedule) {
    sim::PlantParameters parameters;
    sim::Thruster thruster;
    thruster.id = "propeller";
    thruster.delay = .02;
    thruster.fall_time = .05;
    thruster.slew_rate = 0;
    parameters.thrusters.push_back(thruster);
    parameters.command_timeout = 0;
    Runtime runtime(parameters, initial(), 42);
    auto stream = runtime.add(device(), Imu());
    runtime.command(Eigen::VectorXd::Constant(1, 14));
    auto before = runtime.advance(31); // 62ms: deliberately not a 5ms acquisition boundary.
    ASSERT_GT(before.thruster_forces[0], 0);
    const auto acquired = stream->stats().acquired;
    runtime.command(Eigen::VectorXd::Constant(1, -20));
    runtime.stopThrusters();
    EXPECT_EQ(runtime.observe().thruster_forces, before.thruster_forces);
    EXPECT_EQ(runtime.observe().elapsed, before.elapsed);
    EXPECT_EQ(runtime.observe().generation, before.generation);
    EXPECT_EQ(stream->stats().acquired, acquired);
    double force = before.thruster_forces[0];
    for (int tick = 0; tick < 30; ++tick) {
        const auto after = runtime.advance();
        EXPECT_GE(after.thruster_forces[0], 0); // Queued reverse command must never activate.
        EXPECT_LT(after.thruster_forces[0], force);
        force = after.thruster_forces[0];
    }
    EXPECT_EQ(runtime.observe().elapsed, before.elapsed + 60ms);
    EXPECT_EQ(stream->stats().acquired, 24U); // Original 5ms schedule, no restart on stop.
    EXPECT_EQ(stream->latest()->header.generation, before.generation);
}

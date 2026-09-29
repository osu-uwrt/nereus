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

TEST(Imu, ReportedGravityPreservesInertialAccelerationAndMountGeometry) {
    ImuReporting reporting;
    reporting.gravity_magnitude = 9.755455;
    auto input = motion();
    Imu upright({}, {}, {}, reporting);
    EXPECT_DOUBLE_EQ(upright.sample(input, .01).value->specific_force.z(), 9.755455);
    input.acceleration_body = input.gravity_world;
    EXPECT_NEAR(upright.sample(input, .01).value->specific_force.z(), -.051195, 1e-12);
    Mount mount;
    mount.position_body = {.2, -.1, .3};
    mount.sensor_to_body = Eigen::AngleAxisd(.7, Eigen::Vector3d::UnitX());
    input.state.body.orientation = Eigen::AngleAxisd(-.9, Eigen::Vector3d::UnitY());
    input.acceleration_body = {1, 2, 3};
    input.angular_acceleration_body = {.3, .4, -.1};
    input.state.body.angular_velocity = {2, -3, 4};
    Imu calibrated(mount, {}, {}, reporting), physical(mount);
    const auto actual = calibrated.sample(input, .01).value.value();
    const auto original = physical.sample(input, .01).value.value();
    const Eigen::Vector3d expected_correction =
        mount.sensor_to_body.conjugate() *
        (input.state.body.orientation.conjugate() * Eigen::Vector3d(0, 0, -.051195));
    EXPECT_TRUE(
        (actual.specific_force - original.specific_force).isApprox(expected_correction, 1e-11));
    EXPECT_EQ(actual.angular_velocity, original.angular_velocity);
    // Calibration follows environmental direction, not a hardcoded world Z axis.
    input = motion();
    input.gravity_world = {-3, 0, 0};
    EXPECT_EQ(upright.sample(input, .01).value->specific_force, Eigen::Vector3d(9.755455, 0, 0));
}

TEST(Imu, ReportedVariancesDoNotChangeNoiseHistoryOrReset) {
    ImuReporting reporting;
    reporting.force_variance = Eigen::Vector3d(.01, .02, .03);
    reporting.angular_variance = Eigen::Vector3d(0, .04, .05);
    Mount mount;
    mount.sensor_to_body = Eigen::AngleAxisd(.7, Eigen::Vector3d::UnitX());
    Imu reported(mount, noisy(), noisy(), reporting), generated(mount, noisy(), noisy());
    std::vector<Eigen::Vector3d> first;
    for (int replay = 0; replay < 2; ++replay) {
        reported.reset(42, "device");
        generated.reset(42, "device");
        for (int i = 0; i < 20; ++i) {
            const auto a = reported.sample(motion(), .02).value.value();
            const auto b = generated.sample(motion(), .02).value.value();
            EXPECT_EQ(a.specific_force, b.specific_force);
            EXPECT_EQ(a.angular_velocity, b.angular_velocity);
            EXPECT_EQ(a.force_covariance, Eigen::Matrix3d(reporting.force_variance->asDiagonal()));
            EXPECT_EQ(a.angular_covariance,
                      Eigen::Matrix3d(reporting.angular_variance->asDiagonal()));
            EXPECT_NE(a.force_covariance, b.force_covariance);
            if (replay == 0)
                first.push_back(a.specific_force);
            else
                EXPECT_EQ(a.specific_force, first[i]);
        }
    }
    // Reported uncertainty also survives noise-disabled acquisition.
    Imu noiseless({}, {}, {}, reporting);
    EXPECT_EQ(noiseless.sample(motion(), .01).value->force_covariance,
              Eigen::Matrix3d(reporting.force_variance->asDiagonal()));
}

TEST(Imu, RejectsInvalidReportingAndUndefinedGravityDirection) {
    ImuReporting reporting;
    for (const double invalid : {0., -1., std::numeric_limits<double>::infinity(),
                                 std::numeric_limits<double>::quiet_NaN()}) {
        reporting.gravity_magnitude = invalid;
        EXPECT_THROW((Imu({}, {}, {}, reporting)), std::invalid_argument);
    }
    reporting.gravity_magnitude = 9.755455;
    for (const double invalid :
         {-1., std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        reporting.force_variance = Eigen::Vector3d(0, invalid, 0);
        EXPECT_THROW((Imu({}, {}, {}, reporting)), std::invalid_argument);
        reporting.force_variance.reset();
        reporting.angular_variance = Eigen::Vector3d(0, 0, invalid);
        EXPECT_THROW((Imu({}, {}, {}, reporting)), std::invalid_argument);
        reporting.angular_variance.reset();
    }
    auto input = motion();
    input.gravity_world.setZero();
    Imu calibrated({}, {}, {}, reporting), physical;
    EXPECT_THROW(calibrated.sample(input, .01), std::invalid_argument);
    EXPECT_EQ(physical.sample(input, .01).value->specific_force, Eigen::Vector3d::Zero());
}

TEST(Attitude, UsesAbsoluteTimeAndOriginalWorldNoiseMountOrder) {
    Mount mount;
    mount.sensor_to_body = Eigen::AngleAxisd(.7, Eigen::Vector3d::UnitX());
    AttitudeParameters parameters;
    parameters.angle_stddev = .1;
    parameters.heading_drift_rate = -.3;
    parameters.heading_axis_world = Eigen::Vector3d(1, 1, 0).normalized();
    parameters.reported_variance = Eigen::Vector3d(.01, .02, .03);
    Attitude model(mount, parameters);
    auto noise_parameters = parameters;
    noise_parameters.heading_drift_rate = 0;
    Attitude noise({}, noise_parameters);
    model.reset(7, "attitude");
    noise.reset(7, "attitude");
    auto input = motion();
    input.state.elapsed = 2400ms;
    input.state.body.orientation = Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitY());
    input.acceleration_valid = false; // Attitude doesn't require acceleration.
    const auto random_rotation = noise.sample(motion(), .01).value->sensor_to_world;
    const Eigen::Quaterniond expected = Eigen::AngleAxisd(-.72, parameters.heading_axis_world) *
                                        random_rotation * input.state.body.orientation *
                                        mount.sensor_to_body;
    const auto actual = model.sample(input, .01).value.value();
    EXPECT_LT(actual.sensor_to_world.angularDistance(expected), 1e-12);
    EXPECT_NEAR(actual.sensor_to_world.norm(), 1, 1e-15);
    EXPECT_EQ(actual.covariance, Eigen::Matrix3d(parameters.reported_variance->asDiagonal()));
    model.reset(7, "attitude");
    EXPECT_EQ(model.sample(input, .04).value->sensor_to_world.coeffs(),
              actual.sensor_to_world.coeffs());
}

TEST(Attitude, IsotropicAngleNoiseHasDeclaredSmallAngleCovariance) {
    AttitudeParameters parameters;
    parameters.angle_stddev = .01;
    Attitude model({}, parameters);
    model.reset(18, "orientation");
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    Eigen::Matrix3d second = Eigen::Matrix3d::Zero();
    double fourth_moment = 0;
    constexpr int count = 20000;
    for (int i = 0; i < count; ++i) {
        const auto value = model.sample(motion(), .02).value.value();
        const Eigen::AngleAxisd rotation(value.sensor_to_world);
        const Eigen::Vector3d error = rotation.axis() * rotation.angle();
        mean += error;
        second += error * error.transpose();
        fourth_moment += std::pow(error.norm(), 4);
        EXPECT_DOUBLE_EQ(value.covariance(0, 0), .0001 / 3);
    }
    // Scalar Gaussian angle has E[angle^4] = 3 sigma^4; Gaussian-vector noise does not.
    EXPECT_NEAR(fourth_moment / count, 3e-8, .4e-8);
    mean /= count;
    second = second / count - mean * mean.transpose();
    EXPECT_LT(mean.norm(), .0002);
    EXPECT_LT((second - Eigen::Matrix3d::Identity() * (.0001 / 3)).norm(), .000004);
}

TEST(Ahrs, CompositionKeepsIndependentNoiseAcrossUnavailableAcquisitionsAndReset) {
    AhrsParameters parameters;
    parameters.acceleration_noise = noisy();
    parameters.gyro_noise = noisy();
    parameters.attitude.angle_stddev = .01;
    parameters.attitude.heading_drift_rate = .02;
    Ahrs combined({}, parameters);
    Imu raw({}, parameters.acceleration_noise, parameters.gyro_noise);
    Attitude attitude({}, parameters.attitude);
    std::vector<Eigen::Vector4d> first;
    for (int replay = 0; replay < 2; ++replay) {
        combined.reset(42, "imu");
        raw.reset(42, "imu");
        attitude.reset(42, "imu");
        for (int i = 0; i < 10; ++i) {
            auto input = motion();
            input.state.elapsed = (i + 1) * 20ms;
            input.acceleration_valid = i != 3;
            const auto actual = combined.sample(input, .02);
            const auto inertial = raw.sample(input, .02);
            const auto orientation = attitude.sample(input, .02);
            if (!inertial.value) {
                EXPECT_FALSE(actual.value);
                EXPECT_EQ(actual.unavailable_reason, inertial.unavailable_reason);
                continue;
            }
            ASSERT_TRUE(actual.value);
            EXPECT_EQ(actual.value->inertial.specific_force, inertial.value->specific_force);
            EXPECT_EQ(actual.value->inertial.angular_velocity, inertial.value->angular_velocity);
            EXPECT_EQ(actual.value->attitude.sensor_to_world.coeffs(),
                      orientation.value->sensor_to_world.coeffs());
            if (replay == 0)
                first.push_back(actual.value->attitude.sensor_to_world.coeffs());
            else
                EXPECT_EQ(actual.value->attitude.sensor_to_world.coeffs(),
                          first[i > 3 ? i - 1 : i]);
        }
    }
}

TEST(Attitude, RejectsInvalidParametersAndTimeOverflow) {
    AttitudeParameters p;
    for (const double invalid : {-1., std::numeric_limits<double>::infinity(),
                                 std::numeric_limits<double>::quiet_NaN(), 1e200}) {
        p.angle_stddev = invalid;
        EXPECT_THROW((Attitude({}, p)), std::invalid_argument);
    }
    p = {};
    p.heading_axis_world = Eigen::Vector3d(0, 0, 2);
    EXPECT_THROW((Attitude({}, p)), std::invalid_argument);
    p.heading_axis_world = Eigen::Vector3d(0, 0, 1 + 1e-10);
    EXPECT_NO_THROW((Attitude({}, p)));
    p.reported_variance = Eigen::Vector3d(0, -1, 0);
    EXPECT_THROW((Attitude({}, p)), std::invalid_argument);
    p.reported_variance.reset();
    p.heading_drift_rate = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW((Attitude({}, p)), std::invalid_argument);
    p.heading_drift_rate = std::numeric_limits<double>::max();
    Attitude model({}, p);
    auto input = motion();
    input.state.elapsed = 10s;
    EXPECT_THROW(model.sample(input, .01), std::overflow_error);
    input.state.elapsed = -1ns;
    EXPECT_THROW(model.sample(input, .01), std::invalid_argument);
    EXPECT_THROW(model.sample(motion(), 0), std::invalid_argument);
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

TEST(Dvl, PlacedPoolPreservesFiniteRayQueriesAndWaterLevel) {
    sim::Pool local, placed;
    placed.origin_xy_world = {-10, -12};
    placed.yaw_world = .7;
    placed.water_level = 3;
    const Eigen::Quaterniond rotation(Eigen::AngleAxisd(.7, Eigen::Vector3d::UnitZ()));
    const Eigen::Vector3d translation(-10, -12, 3);
    PoolBottom a(local), b(placed);
    for (const Eigen::Vector3d &origin :
         {Eigen::Vector3d(2, 1, -2), Eigen::Vector3d(-1, 1, -2), Eigen::Vector3d(19, 5, -2),
          Eigen::Vector3d(2, 1, 1), Eigen::Vector3d(2, 1, -5)}) {
        for (const Eigen::Vector3d &direction :
             {Eigen::Vector3d(0, 0, -1), Eigen::Vector3d(0, 0, 1),
              Eigen::Vector3d(1, 0, -1).normalized().eval()}) {
            const auto first = a(origin, direction);
            const auto second = b(translation + rotation * origin, rotation * direction);
            ASSERT_EQ(first.has_value(), second.has_value());
            if (first) {
                EXPECT_NEAR(first->distance, second->distance, 1e-12);
                EXPECT_TRUE(second->velocity_world.isZero());
            }
        }
    }
    HydrostaticPressure pressure_a(local.water_level), pressure_b(placed.water_level);
    EXPECT_DOUBLE_EQ(*pressure_a({2, 1, -2}),
                     *pressure_b(translation + rotation * Eigen::Vector3d(2, 1, -2)));
    placed.origin_xy_world.x() = std::numeric_limits<double>::infinity();
    EXPECT_THROW((PoolBottom(placed)), std::invalid_argument);
}

TEST(Dvl, PlacedFloorIncludesBoundaryOriginsAndHitsWithoutExtendingItsFootprint) {
    for (double yaw : {.3, .7, 1.1}) {
        sim::Pool pool;
        pool.origin_xy_world = {-10, -12};
        pool.yaw_world = yaw;
        PoolBottom bottom(pool);
        const Eigen::Quaterniond rotation(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        const Eigen::Vector3d translation(-10, -12, 0);
        for (const Eigen::Vector3d &origin :
             {Eigen::Vector3d(0, 5, -2), Eigen::Vector3d(5, 10, -2)})
            EXPECT_TRUE(bottom(translation + rotation * origin, -Eigen::Vector3d::UnitZ()));
        EXPECT_FALSE(bottom(translation + rotation * Eigen::Vector3d(-1e-8, 5, -2),
                            -Eigen::Vector3d::UnitZ()));
        EXPECT_TRUE(bottom(translation + rotation * Eigen::Vector3d(5, 5, -2),
                           rotation * Eigen::Vector3d(0, 5, -3).normalized()));
        EXPECT_FALSE(bottom(translation + rotation * Eigen::Vector3d(5, 5, -2),
                            rotation * Eigen::Vector3d(0, 5.000001, -3).normalized()));
    }
}

TEST(Fog, IndependentReportedVariancePreservesProjectionAndNoise) {
    const std::vector<Eigen::Vector3d> axes{Eigen::Vector3d::UnitX(),
                                            Eigen::Vector3d(1, 1, 0).normalized()};
    Fog derived({}, axes, noisy()), reported({}, axes, noisy(), Eigen::Vector3d(4, 9, 16));
    derived.reset(18, "fog");
    reported.reset(18, "fog");
    for (int i = 0; i < 10; ++i) {
        const auto a = derived.sample(motion(), .01).value.value();
        const auto b = reported.sample(motion(), .01).value.value();
        EXPECT_EQ(a.angular_rates, b.angular_rates);
        EXPECT_NEAR(b.covariance(0, 0), 4, 1e-12);
        EXPECT_NEAR(b.covariance(1, 1), 6.5, 1e-12);
        EXPECT_NEAR(b.covariance(0, 1), 4 / std::sqrt(2.), 1e-12);
    }
    for (const double invalid :
         {-1., std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
        EXPECT_THROW((Fog({}, axes, {}, Eigen::Vector3d(0, invalid, 0))), std::invalid_argument);
}

TEST(ReferenceVelocity, MovingReferenceIncludesMountLeverArmWithoutBottomAvailability) {
    ReferenceVelocityParameters p;
    p.mount.position_body = {0, 2, 0};
    p.mount.sensor_to_body = Eigen::AngleAxisd(std::acos(-1.) / 2, Eigen::Vector3d::UnitY());
    p.reference_velocity_world = {1, 2, 0};
    p.reported_variance = Eigen::Vector3d(.01, .02, .03);
    auto input = motion();
    input.state.body.position = {-100, 500, 20}; // No water or bottom is required.
    input.state.body.orientation = Eigen::AngleAxisd(std::acos(-1.) / 2, Eigen::Vector3d::UnitZ());
    input.state.body.linear_velocity = {2, 3, 4};
    input.state.body.angular_velocity = {0, 0, 3};
    input.acceleration_valid = false;
    ReferenceVelocity model(p);
    const auto actual = model.sample(input, .01).value.value();
    EXPECT_TRUE(actual.reference_relative_velocity.isApprox(Eigen::Vector3d(-4, 4, -6), 1e-12));
    EXPECT_EQ(actual.covariance, Eigen::Matrix3d(p.reported_variance->asDiagonal()));
    Dvl bottom({}, PoolBottom(sim::Pool{}));
    EXPECT_FALSE(bottom.sample(input, .01).value);
}

TEST(ReferenceVelocity, InclinationGatePreservesNoiseHistoryAndReset) {
    ReferenceVelocityParameters p;
    p.noise = noisy();
    ReferenceVelocity continuous(p);
    p.inclination_limit = InclinationLimit{};
    p.inclination_limit->maximum_angle = .4;
    ReferenceVelocity gated(p);
    continuous.reset(7, "velocity");
    gated.reset(7, "velocity");
    auto input = motion();
    input.state.body.orientation = Eigen::AngleAxisd(.400001, Eigen::Vector3d::UnitX());
    EXPECT_FALSE(gated.sample(input, .01).value);
    continuous.sample(input, .01);
    input.state.body.orientation = Eigen::AngleAxisd(.399999, Eigen::Vector3d::UnitX());
    const auto expected = continuous.sample(input, .01).value.value();
    EXPECT_EQ(gated.sample(input, .01).value->reference_relative_velocity,
              expected.reference_relative_velocity);
    gated.reset(7, "velocity");
    continuous.reset(7, "velocity");
    EXPECT_EQ(gated.sample(input, .01).value->reference_relative_velocity,
              continuous.sample(input, .01).value->reference_relative_velocity);
    p.mount.sensor_to_body = Eigen::AngleAxisd(std::acos(-1.) / 2, Eigen::Vector3d::UnitY());
    p.inclination_limit->sensor_axis = Eigen::Vector3d::UnitZ();
    p.inclination_limit->reference_axis_world = Eigen::Vector3d::UnitX();
    ReferenceVelocity custom(p);
    EXPECT_TRUE(custom.sample(motion(), .01).value);
    input.state.body.orientation = Eigen::AngleAxisd(std::acos(-1.) / 2, Eigen::Vector3d::UnitZ());
    EXPECT_FALSE(custom.sample(input, .01).value);
}

TEST(ReferenceVelocity, RejectsInvalidReferenceCovarianceAndInclination) {
    ReferenceVelocityParameters p;
    p.reference_velocity_world.x() = std::numeric_limits<double>::infinity();
    EXPECT_THROW((ReferenceVelocity(p)), std::invalid_argument);
    p.reference_velocity_world.setZero();
    p.reported_variance = Eigen::Vector3d(0, -1, 0);
    EXPECT_THROW((ReferenceVelocity(p)), std::invalid_argument);
    p.reported_variance.reset();
    p.inclination_limit = InclinationLimit{};
    for (const double angle : {-1., 4., std::numeric_limits<double>::quiet_NaN()}) {
        p.inclination_limit->maximum_angle = angle;
        EXPECT_THROW((ReferenceVelocity(p)), std::invalid_argument);
    }
    p.inclination_limit->maximum_angle = .5;
    p.inclination_limit->sensor_axis.setZero();
    EXPECT_THROW((ReferenceVelocity(p)), std::invalid_argument);
    p.inclination_limit->sensor_axis = -Eigen::Vector3d::UnitZ();
    p.inclination_limit->reference_axis_world *= 2;
    EXPECT_THROW((ReferenceVelocity(p)), std::invalid_argument);
}

TEST(ReferenceVelocity, InclinationAcceptsRotatedAlignmentAndRejectsExteriorAngles) {
    ReferenceVelocityParameters p;
    p.mount.sensor_to_body = Eigen::AngleAxisd(.06, Eigen::Vector3d(3, 1, 2).normalized());
    auto input = motion();
    input.state.body.orientation = Eigen::AngleAxisd(.02, Eigen::Vector3d(1, 2, 3).normalized());
    p.inclination_limit = InclinationLimit{};
    auto &limit = *p.inclination_limit;
    limit.reference_axis_world =
        (input.state.body.orientation * (p.mount.sensor_to_body * limit.sensor_axis)).normalized();
    ReferenceVelocity aligned(p);
    EXPECT_TRUE(aligned.sample(input, .01).value);
    input.state.body.orientation =
        Eigen::AngleAxisd(1e-8, limit.reference_axis_world.unitOrthogonal()) *
        input.state.body.orientation;
    EXPECT_FALSE(aligned.sample(input, .01).value);
    p.mount = {};
    limit.reference_axis_world = -Eigen::Vector3d::UnitZ();
    limit.maximum_angle = .4;
    ReferenceVelocity boundary(p);
    input.state.body.orientation = Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitX());
    EXPECT_TRUE(boundary.sample(input, .01).value);
    input.state.body.orientation = Eigen::AngleAxisd(.4 + 1e-8, Eigen::Vector3d::UnitX());
    EXPECT_FALSE(boundary.sample(input, .01).value);
}

TEST(ReferenceAltitude, CorrectsMountedHeightUsingTheSameAcquisitionPose) {
    ReferenceAltitudeParameters parameters;
    parameters.mount.position_body = {1, 2, 3};
    parameters.mount.sensor_to_body = Eigen::AngleAxisd(.7, Eigen::Vector3d::UnitY());
    parameters.target_position_body = Eigen::Vector3d(-2, 3, -4);
    parameters.noise.bias = .25;
    auto input = motion();
    input.state.body.orientation = Eigen::AngleAxisd(std::acos(-1.) / 2, Eigen::Vector3d::UnitX());
    input.acceleration_valid = false; // Height needs pose, not differentiated acceleration.
    ReferenceAltitude model(parameters);
    const auto result = model.sample(input, .05).value.value();
    EXPECT_NEAR(result.mounted_world_z, .25, 1e-12);
    EXPECT_NEAR(result.target_world_z, 1.25, 1e-12);
    EXPECT_EQ(result.variance, 0);
    input.state.body.orientation = Eigen::Quaterniond::Identity();
    const auto upright = model.sample(input, .05).value.value();
    EXPECT_DOUBLE_EQ(upright.mounted_world_z, 1.25);
    EXPECT_DOUBLE_EQ(upright.target_world_z, -5.75);
    parameters.target_position_body.reset();
    ReferenceAltitude at_mount(parameters);
    const auto same = at_mount.sample(input, .05).value.value();
    EXPECT_EQ(same.target_world_z, same.mounted_world_z);
}

TEST(ReferenceAltitude, NoiseIsSharedAndReportedVarianceDoesNotChangeReplay) {
    ReferenceAltitudeParameters parameters;
    parameters.mount.position_body.z() = 1;
    parameters.target_position_body = Eigen::Vector3d(0, 0, -2);
    parameters.noise = {.1, .02, .03};
    ReferenceAltitude generated(parameters);
    parameters.reported_variance = .9;
    ReferenceAltitude reported(parameters);
    std::vector<double> first;
    for (int replay = 0; replay < 2; ++replay) {
        generated.reset(42, "depth");
        reported.reset(42, "depth");
        for (int i = 0; i < 20; ++i) {
            const auto a = generated.sample(motion(), .05).value.value();
            const auto b = reported.sample(motion(), .05).value.value();
            EXPECT_EQ(a.mounted_world_z, b.mounted_world_z);
            EXPECT_EQ(a.target_world_z, b.target_world_z);
            EXPECT_NEAR(a.target_world_z - a.mounted_world_z, -3, 1e-14);
            EXPECT_NEAR(a.variance, .02 * .02 + (i + 1) * .05 * .03 * .03, 1e-14);
            EXPECT_EQ(b.variance, .9);
            if (replay == 0)
                first.push_back(a.mounted_world_z);
            else
                EXPECT_EQ(a.mounted_world_z, first[i]);
        }
    }
}

TEST(ReferenceAltitude, UnboundedWorldHeightIsDistinctFromHydrostaticPressure) {
    ReferenceAltitude altitude;
    Pressure pressure({}, HydrostaticPressure(0));
    auto input = motion();
    for (double z : {-100., -2., 0., 20., 100.}) {
        input.state.body.position.z() = z;
        const auto measured = altitude.sample(input, .05).value.value();
        EXPECT_EQ(measured.mounted_world_z, z);
        EXPECT_EQ(measured.target_world_z, z);
        if (z >= 0) {
            EXPECT_EQ(pressure.sample(input, .05).value->absolute_pressure, 101325);
            EXPECT_EQ(pressure.sample(input, .05).value->depth, 0);
        }
    }
}

TEST(ReferenceAltitude, RejectsInvalidParametersAndOverflow) {
    ReferenceAltitudeParameters p;
    p.target_position_body = Eigen::Vector3d(0, 0, std::numeric_limits<double>::infinity());
    EXPECT_THROW((ReferenceAltitude(p)), std::invalid_argument);
    p.target_position_body.reset();
    for (double variance : {-1., std::numeric_limits<double>::quiet_NaN()}) {
        p.reported_variance = variance;
        EXPECT_THROW((ReferenceAltitude(p)), std::invalid_argument);
    }
    p.reported_variance.reset();
    p.noise.white_stddev = -1;
    EXPECT_THROW((ReferenceAltitude(p)), std::invalid_argument);
    p.noise.white_stddev = 0;
    p.mount.position_body.z() = std::numeric_limits<double>::max();
    auto input = motion();
    input.state.body.position.z() = std::numeric_limits<double>::max();
    ReferenceAltitude model(p);
    EXPECT_THROW(model.sample(input, .05), std::overflow_error);
}

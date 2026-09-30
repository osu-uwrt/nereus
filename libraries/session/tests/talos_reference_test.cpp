// Talos plant and sensor formulas against trajectories recorded from the original UWRT simulator,
// with the plant built from the Talos pack.
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <gtest/gtest.h>
#include <nereus/sensors/models.hpp>
#include <nereus/session/session.hpp>
#include <sstream>

// The reference trajectories were recorded on aarch64. Other ISAs round differently (Eigen uses SSE instead
// of NEON, no fused multiply-add), so only aarch64 is a bit-level check. On x86-64 the largest difference is
// ~4e-3 in angular acceleration right after a thruster command (the delayed thruster engages a tick apart);
// positions and velocities agree far more closely.
#if defined(__aarch64__)
constexpr double kReferenceTolerance = 1e-9;
#else
constexpr double kReferenceTolerance = 1e-2;
#endif

TEST(TalosReference, OriginalDynamicsActuatorsImmersionAndPoolContacts) {
    auto pack = nereus::session::createRuntime(nereus::session::loadResolvedScenario(NEREUS_RESOLVED_TALOS));
    // The recordings placed the pool corner with a 1e-15 m residual; initial wall contact is
    // sensitive to last-bit point ordering, so use the identical input.
    pack.parameters.pool.origin_xy_world.x() = -1.1948633889920896e-15;
    // Recorded script: 1500 ticks; [4, 4, 2, 2, -2, -2, 4, 4] N every 200 ticks until 800, zero at
    // 1000.
    const std::uint64_t ticks = 1500;
    std::vector<std::pair<std::uint64_t, Eigen::VectorXd>> commands;
    Eigen::VectorXd forces(8);
    forces << 4, 4, 2, 2, -2, -2, 4, 4;
    for (std::uint64_t tick = 0; tick <= 800; tick += 200)
        commands.emplace_back(tick, forces);
    commands.emplace_back(1000, Eigen::VectorXd::Zero(8));
    std::array<std::ifstream, 2> fixtures{
        std::ifstream(std::string(NEREUS_SOURCE_DIR) + "/tests/fixtures/legacy_talos.csv"),
        std::ifstream(std::string(NEREUS_SOURCE_DIR) + "/tests/fixtures/legacy_talos_unoptimized.csv")};
    std::array<double, 2> error{};
    std::array<std::string, 2> worst;
    std::string row;
    for (auto &fixture : fixtures) {
        ASSERT_TRUE(fixture);
        ASSERT_TRUE(std::getline(fixture, row));
    }
    for (int scenario = 0; scenario < 3; ++scenario) {
        auto initial = pack.initial;
        if (scenario == 1) {
            initial.position = {11.43, -5.4864, .05};
            initial.orientation = Eigen::AngleAxisd(-.3, Eigen::Vector3d::UnitZ()) *
                                  Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(.1, Eigen::Vector3d::UnitX());
            initial.linear_velocity = {.1, -.05, .02};
            initial.angular_velocity = {.3, .2, .1};
        } else if (scenario == 2) {
            initial.position = {11.43, -5.4864, -1.8836};
            initial.linear_velocity = {.2, .1, -.8};
        }
        nereus::simulation::Plant plant(pack.parameters, initial);
        std::size_t next_command = 0;
        for (std::uint64_t tick = 0; tick <= ticks; ++tick) {
            {
                const auto motion = plant.motion();
                const auto &state = motion.state;
                const auto &body = state.body;
                Eigen::Matrix<double, 27, 1> actual;
                actual << body.position, body.orientation.w(), body.orientation.x(), body.orientation.y(),
                    body.orientation.z(), body.linear_velocity, body.angular_velocity, state.thruster_forces,
                    motion.acceleration_body, motion.angular_acceleration_body;
                ASSERT_TRUE(actual.allFinite());
                EXPECT_EQ(state.tick, tick);
                EXPECT_EQ(state.elapsed.count(), static_cast<std::int64_t>(tick) * 2000000);
                for (std::size_t candidate = 0; candidate < fixtures.size(); ++candidate) {
                    ASSERT_TRUE(std::getline(fixtures[candidate], row));
                    std::istringstream fields(row);
                    std::string value;
                    ASSERT_TRUE(std::getline(fields, value, ','));
                    ASSERT_EQ(std::stoi(value), scenario);
                    ASSERT_TRUE(std::getline(fields, value, ','));
                    ASSERT_EQ(std::stoull(value), tick);
                    for (int field = 0; field < actual.size(); ++field) {
                        ASSERT_TRUE(std::getline(fields, value, ','));
                        const auto expected = std::stod(value);
                        ASSERT_TRUE(std::isfinite(expected));
                        const auto difference = std::abs(actual[field] - expected);
                        if (difference > error[candidate]) {
                            error[candidate] = difference;
                            worst[candidate] = "case=" + std::to_string(scenario) + " tick=" + std::to_string(tick) +
                                               " field=" + std::to_string(field);
                        }
                    }
                    EXPECT_FALSE(std::getline(fields, value, ','));
                }
            }
            if (tick == ticks)
                break;
            if (next_command < commands.size() && commands[next_command].first == tick)
                plant.command(commands[next_command++].second);
            plant.advance();
        }
    }
    for (auto &fixture : fixtures)
        EXPECT_FALSE(std::getline(fixture, row));
    // One complete original trajectory must match; never mix candidate values per field.
    EXPECT_LE(std::min(error[0], error[1]), kReferenceTolerance)
        << "optimized max=" << error[0] << " " << worst[0] << "; unoptimized max=" << error[1] << " " << worst[1];
}

TEST(TalosReference, OriginalNoiseDisabledSensorFormulas) {
    namespace sensors = nereus::sensors;
    const auto pack = nereus::session::createRuntime(nereus::session::loadResolvedScenario(NEREUS_RESOLVED_TALOS));
    const auto &frames = *pack.frames;
    const auto mount = [&frames](const char *frame) {
        const auto &pose = frames.fromRoot(frame);
        return sensors::Mount{pose.translation, pose.rotation};
    };
    sensors::AhrsParameters parameters;
    parameters.inertial_reporting.gravity_magnitude = 9.755455;
    parameters.inertial_reporting.force_variance = Eigen::Vector3d::Constant(.01);
    parameters.inertial_reporting.angular_variance = Eigen::Vector3d::Constant(.01);
    parameters.attitude.reported_variance = Eigen::Vector3d(.00005, .00001, .01);
    sensors::Ahrs ahrs(mount("imu_mount"), parameters);
    sensors::Fog fog(mount("fog_mount"), {Eigen::Vector3d::UnitZ()}, {},
                     Eigen::Vector3d(0, 0, std::pow(.01 * std::acos(-1.) / 180, 2)));
    sensors::ReferenceVelocityParameters velocity_parameters;
    velocity_parameters.mount = mount("dvl_mount");
    velocity_parameters.reported_variance = Eigen::Vector3d::Constant(.000001);
    sensors::ReferenceVelocity velocity(velocity_parameters);
    sensors::ReferenceAltitudeParameters altitude_parameters;
    altitude_parameters.mount = mount("depth_mount");
    altitude_parameters.target_position_body = frames.fromRoot("base_link").translation;
    altitude_parameters.reported_variance = .0001;
    sensors::ReferenceAltitude altitude(altitude_parameters);
    std::ifstream fixture(std::string(NEREUS_SOURCE_DIR) + "/tests/fixtures/legacy_sensor_kinematics.csv");
    ASSERT_TRUE(fixture);
    std::string row;
    ASSERT_TRUE(std::getline(fixture, row));
    int count = 0;
    while (std::getline(fixture, row)) {
        std::istringstream fields(row);
        std::string value;
        Eigen::Matrix<double, 48, 1> values;
        for (auto &element : values) {
            ASSERT_TRUE(std::getline(fields, value, ','));
            element = std::stod(value);
        }
        EXPECT_FALSE(std::getline(fields, value, ','));
        ASSERT_TRUE(values.allFinite());
        EXPECT_EQ(values[0], count++);
        nereus::simulation::MotionSample input;
        auto &body = input.state.body;
        body.position = values.segment<3>(1);
        body.orientation = Eigen::Quaterniond(values[4], values[5], values[6], values[7]);
        body.linear_velocity = values.segment<3>(8);
        body.angular_velocity = values.segment<3>(11);
        input.acceleration_body = values.segment<3>(14) + body.angular_velocity.cross(body.linear_velocity);
        input.angular_acceleration_body = values.segment<3>(17);
        const auto imu = ahrs.sample(input, .02).value.value();
        const auto gyro = fog.sample(input, .002).value.value();
        Eigen::Quaterniond orientation = imu.attitude.sensor_to_world;
        const Eigen::Quaterniond expected_orientation(values[26], values[27], values[28], values[29]);
        if (orientation.dot(expected_orientation) < 0)
            orientation.coeffs() *= -1;
        const auto speed = velocity.sample(input, .125).value.value();
        const auto depth = altitude.sample(input, .05).value.value();
        Eigen::Matrix<double, 28, 1> actual;
        actual << imu.inertial.specific_force, imu.inertial.angular_velocity, orientation.w(), orientation.x(),
            orientation.y(), orientation.z(), imu.attitude.covariance.diagonal(),
            imu.inertial.angular_covariance.diagonal(), imu.inertial.force_covariance.diagonal(), gyro.angular_rates[0],
            gyro.covariance(0, 0), speed.reference_relative_velocity, speed.covariance(0, 0), depth.mounted_world_z,
            depth.target_world_z, depth.variance;
        ASSERT_TRUE(actual.allFinite());
        EXPECT_LT((actual - values.segment<28>(20)).cwiseAbs().maxCoeff(), 1e-12) << "case " << values[0];
    }
    EXPECT_EQ(count, 24);
}

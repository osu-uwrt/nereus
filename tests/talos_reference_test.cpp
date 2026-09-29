#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <gtest/gtest.h>
#include <robotics/config/scenario.hpp>
#include <robotics/sensors/models.hpp>
#include <sstream>

TEST(TalosReference, OriginalDynamicsActuatorsImmersionAndPoolContacts) {
    const auto config =
        robotics::config::loadScenario(std::string(RP_TEST_CONTENT) + "/examples/talos_pool.yaml");
    std::array<std::ifstream, 2> fixtures{
        std::ifstream(std::string(RP_FIXTURES) + "/legacy_talos.csv"),
        std::ifstream(std::string(RP_FIXTURES) + "/legacy_talos_unoptimized.csv")};
    std::array<double, 2> error{};
    std::array<std::string, 2> worst;
    std::string row;
    for (auto &fixture : fixtures) {
        ASSERT_TRUE(fixture);
        ASSERT_TRUE(std::getline(fixture, row));
    }
    for (int scenario = 0; scenario < 3; ++scenario) {
        auto initial = config.initial;
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
        robotics::simulation::Plant plant(config.plant, initial);
        std::size_t next_command = 0;
        for (std::uint64_t tick = 0; tick <= config.ticks; ++tick) {
            {
                const auto motion = plant.motion();
                const auto &state = motion.state;
                const auto &body = state.body;
                Eigen::Matrix<double, 27, 1> actual;
                actual << body.position, body.orientation.w(), body.orientation.x(),
                    body.orientation.y(), body.orientation.z(), body.linear_velocity,
                    body.angular_velocity, state.thruster_forces, motion.acceleration_body,
                    motion.angular_acceleration_body;
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
                            worst[candidate] = "case=" + std::to_string(scenario) +
                                               " tick=" + std::to_string(tick) +
                                               " field=" + std::to_string(field);
                        }
                    }
                    EXPECT_FALSE(std::getline(fields, value, ','));
                }
            }
            if (tick == config.ticks)
                break;
            if (next_command < config.commands.size() && config.commands[next_command].tick == tick)
                plant.command(config.commands[next_command++].forces);
            plant.advance();
        }
    }
    for (auto &fixture : fixtures)
        EXPECT_FALSE(std::getline(fixture, row));
    // One complete original trajectory must match; never mix candidate values per field.
    EXPECT_LE(std::min(error[0], error[1]), 1e-9)
        << "optimized max=" << error[0] << " " << worst[0] << "; unoptimized max=" << error[1]
        << " " << worst[1];
}

TEST(TalosReference, OriginalNoiseDisabledInertialMeasurementFormulas) {
    namespace sensors = robotics::sensors;
    const auto config = robotics::config::loadScenario(std::string(RP_TEST_CONTENT) +
                                                       "/examples/talos_inertial_pool.yaml");
    const auto mount = [&config](const char *frame) {
        const auto &pose = config.body_frames.fromRoot(frame);
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
    std::ifstream fixture(std::string(RP_FIXTURES) + "/legacy_sensor_kinematics.csv");
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
        robotics::simulation::MotionSample input;
        auto &body = input.state.body;
        body.position = values.segment<3>(1);
        body.orientation = Eigen::Quaterniond(values[4], values[5], values[6], values[7]);
        body.linear_velocity = values.segment<3>(8);
        body.angular_velocity = values.segment<3>(11);
        input.acceleration_body =
            values.segment<3>(14) + body.angular_velocity.cross(body.linear_velocity);
        input.angular_acceleration_body = values.segment<3>(17);
        const auto imu = ahrs.sample(input, .02).value.value();
        const auto gyro = fog.sample(input, .002).value.value();
        Eigen::Quaterniond orientation = imu.attitude.sensor_to_world;
        const Eigen::Quaterniond expected_orientation(values[26], values[27], values[28],
                                                      values[29]);
        if (orientation.dot(expected_orientation) < 0)
            orientation.coeffs() *= -1;
        Eigen::Matrix<double, 21, 1> actual;
        actual << imu.inertial.specific_force, imu.inertial.angular_velocity, orientation.w(),
            orientation.x(), orientation.y(), orientation.z(), imu.attitude.covariance.diagonal(),
            imu.inertial.angular_covariance.diagonal(), imu.inertial.force_covariance.diagonal(),
            gyro.angular_rates[0], gyro.covariance(0, 0);
        ASSERT_TRUE(actual.allFinite());
        EXPECT_LT((actual - values.segment<21>(20)).cwiseAbs().maxCoeff(), 1e-12)
            << "case " << values[0];
        // Remaining captured DVL and depth columns become gates with their declared models.
    }
    EXPECT_EQ(count, 24);
}

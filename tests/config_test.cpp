#include "robotics/config/scenario.hpp"
#include <gtest/gtest.h>

TEST(Configuration, MissingFileIncludesSourcePath) {
    try {
        robotics::config::loadScenario("/does/not/exist/scenario.yaml");
        FAIL() << "Expected load failure";
    } catch (const std::invalid_argument &error) {
        EXPECT_NE(std::string(error.what()).find("/does/not/exist/scenario.yaml"),
                  std::string::npos);
    }
}

#include "robotics/sensors/readings.hpp"
#include <chrono>
#include <fstream>
#include <random>

namespace {
class Profiles : public testing::Test {
  protected:
    std::filesystem::path root;
    Profiles() {
        root = std::filesystem::temp_directory_path() /
               ("robotics-profiles-" +
                std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + "-" +
                std::to_string(std::random_device{}()));
        if (!std::filesystem::create_directory(root)) {
            throw std::runtime_error("cannot create test directory");
        }
        std::filesystem::copy(RP_TEST_CONTENT, root, std::filesystem::copy_options::recursive);
    }
    ~Profiles() override {
        std::error_code error;
        std::filesystem::remove_all(root, error);
    }
    std::filesystem::path scenario() const {
        return root / "examples/profile_pool.yaml";
    }
    std::string read(const std::string &relative) const {
        std::ifstream file(root / relative);
        return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
    }
    void write(const std::string &relative, const std::string &content) const {
        std::ofstream file(root / relative);
        file << content;
        if (!file) {
            throw std::runtime_error("cannot write fixture");
        }
    }
    void replace(const std::string &relative, const std::string &from,
                 const std::string &to) const {
        auto content = read(relative);
        const auto position = content.find(from);
        if (position == std::string::npos) {
            throw std::runtime_error("fixture token missing: " + from);
        }
        content.replace(position, from.size(), to);
        write(relative, content);
    }
};
} // namespace

TEST_F(Profiles, ResolvesDeclaringFilesAndConstructsAllSensorFamilies) {
    const auto config = robotics::config::loadScenario(scenario());
    EXPECT_EQ(config.sources.size(), 4U);
    EXPECT_EQ(config.sensors.size(), 4U);
    EXPECT_EQ(config.seed, 42U);
    auto runtime = robotics::config::makeRuntime(config);
    runtime->advance(60);
    const auto pressure = runtime->stream<robotics::sensors::PressureReading>("pressure")->latest();
    ASSERT_TRUE(pressure);
    EXPECT_NEAR(pressure->measurement.value->depth, 2.1, 0.01);
    const auto dvl = runtime->stream<robotics::sensors::DvlReading>("dvl")->latest();
    ASSERT_TRUE(dvl);
    EXPECT_EQ(dvl->header.acquired.count(), 100000000);
    EXPECT_EQ(dvl->header.delivered.count(), 120000000);
    EXPECT_NEAR(dvl->measurement.value->bottom_distance, 2.9, 1e-12);
    EXPECT_TRUE(runtime->stream<robotics::sensors::ImuReading>("imu")->latest());
    EXPECT_TRUE(runtime->stream<robotics::sensors::FogReading>("fog")->latest());
    EXPECT_THROW(runtime->stream<robotics::sensors::FogReading>("imu"), std::invalid_argument);
    EXPECT_THROW(runtime->stream<robotics::sensors::ImuReading>("missing"), std::invalid_argument);
}

TEST_F(Profiles, ResolvedConfigurationSurvivesSourceRemovalAndInstancesReplayIndependently) {
    const auto config = robotics::config::loadScenario(scenario());
    std::filesystem::remove_all(root);
    auto first = robotics::config::makeRuntime(config),
         second = robotics::config::makeRuntime(config);
    const auto a = first->stream<robotics::sensors::ImuReading>("imu");
    const auto b = second->stream<robotics::sensors::ImuReading>("imu");
    first->advance(60);
    for (int i = 0; i < 60; ++i) {
        second->advance();
    }
    const auto expected = a->latest()->measurement.value->specific_force;
    EXPECT_EQ(expected, b->latest()->measurement.value->specific_force);
    first->reset(config.initial, config.seed);
    EXPECT_FALSE(a->latest());
    first->advance(60);
    EXPECT_EQ(expected, a->latest()->measurement.value->specific_force);
    EXPECT_EQ(a->latest()->header.generation, 1U);
    EXPECT_EQ(b->latest()->header.generation, 0U);
}

TEST_F(Profiles, WorldPressureEnvironmentDoesNotOverwriteSensorCalibration) {
    auto low = robotics::config::makeRuntime(robotics::config::loadScenario(scenario()));
    replace("worlds/empty_pool.yaml", "water_level_m: 0", "water_level_m: 1");
    auto high = robotics::config::makeRuntime(robotics::config::loadScenario(scenario()));
    low->advance(25);
    high->advance(25);
    const auto a = low->stream<robotics::sensors::PressureReading>("pressure")
                       ->latest()
                       ->measurement.value.value();
    const auto b = high->stream<robotics::sensors::PressureReading>("pressure")
                       ->latest()
                       ->measurement.value.value();
    EXPECT_NEAR(b.absolute_pressure - a.absolute_pressure, 9806.65, 1e-9);
    EXPECT_NEAR(b.depth - a.depth, 1, 1e-12);
}

TEST_F(Profiles, FullMatricesAndDampingOffsetsRemainAvailableToRobotProfiles) {
    replace("robots/synthetic_auv.yaml", "inertia_diagonal: [1, 1, 1]",
            "inertia_matrix: [[2, 0.1, 0], [0.1, 3, 0], [0, 0, 4]]\n  damping_center_m: [0.1, 0.2, "
            "0.3]");
    const auto config = robotics::config::loadScenario(scenario());
    EXPECT_DOUBLE_EQ(config.plant.body.inertia(0, 1), 0.1);
    EXPECT_EQ(config.plant.body.damping_center, Eigen::Vector3d(0.1, 0.2, 0.3));
    replace("robots/synthetic_auv.yaml", "mass_kg: 10",
            "mass_kg: 10\n  inertia_diagonal: [1, 1, 1]");
    EXPECT_THROW(robotics::config::loadScenario(scenario()), std::invalid_argument);
}

TEST_F(Profiles, EmptySensorListsAreValid) {
    auto robot = read("robots/synthetic_auv.yaml");
    robot.erase(robot.find("sensors:"));
    write("robots/synthetic_auv.yaml", robot + "sensors: []\n");
    const auto config = robotics::config::loadScenario(scenario());
    EXPECT_TRUE(config.sensors.empty());
    EXPECT_EQ(robotics::config::makeRuntime(config)->advance(10).tick, 10U);
}

TEST_F(Profiles, RejectsMalformedSensorDeclarationsBeforeReturningConfiguration) {
    const auto original = read("robots/synthetic_auv.yaml");
    const std::vector<std::pair<std::string, std::string>> changes{
        {"id: fog", "id: imu"},
        {"period_ns: 10000000", "period_ns: 1"},
        {"period_ns: 10000000", "period_ns: 10000000\n    capacity: 0"},
        {"period_ns: 10000000", "period_ns: 10000000\n    latency_ns: 9223372036854775808"},
        {"model: fog", "model: sonar"},
        {"axes: [[0, 0, 1]]", "axes: [[0, 0, 2]]"},
        {"id: fog", "id: fog\n    id: duplicate"},
        {"id: fog", "id: fog\n    mystery: 1"},
        {"id: fog", "id: null"},
        {"white_stddev: 10", "white_stddev: -1"},
        {"white_stddev: 10", "white_stddev: .nan"},
        {"profile: ../sensors/imu.yaml", "profile: ../sensors/imu.yaml\n    model: imu"},
        {"profile: ../sensors/imu.yaml", "profile: package://robot/imu.yaml"},
        {"mount: {position_m: [0, 0, 0], orientation_wxyz: [1, 0, 0, 0]}",
         "mount: {position_m: [0, 0, 0], orientation_wxyz: [0, 0, 0, 0]}"},
        {"minimum_range_m: 0.1", "minimum_range_m: 20"},
    };
    for (const auto &change : changes) {
        SCOPED_TRACE(change.second);
        write("robots/synthetic_auv.yaml", original);
        replace("robots/synthetic_auv.yaml", change.first, change.second);
        EXPECT_THROW(robotics::config::loadScenario(scenario()), std::invalid_argument);
    }
}

TEST_F(Profiles, NestedProfileFailuresIdentifyTheirSource) {
    replace("sensors/imu.yaml", "gyro_noise:", "unknown_noise:");
    try {
        robotics::config::loadScenario(scenario());
        FAIL();
    } catch (const std::invalid_argument &error) {
        EXPECT_NE(std::string(error.what()).find("sensors/imu.yaml"), std::string::npos);
        EXPECT_NE(std::string(error.what()).find("unknown_noise"), std::string::npos);
    }
    write("sensors/imu.yaml", "schema_version: 1\nkind: world\nmodel: imu\nparameters: {}\n");
    EXPECT_THROW(robotics::config::loadScenario(scenario()), std::invalid_argument);
    std::filesystem::remove(root / "sensors/imu.yaml");
    EXPECT_THROW(robotics::config::loadScenario(scenario()), std::invalid_argument);
}

TEST_F(Profiles, NativeWorldAndRobotDeclareFlowAndImmersionIndependently) {
    replace("worlds/empty_pool.yaml", "current_m_s: [0, 0, 0]",
            "current_m_s: [0, 0, 0]\n  current_oscillation_amplitude_m_s: [0.2, 0.1, 0.04]\n"
            "  current_oscillation_frequency_hz: 0.7");
    replace("robots/synthetic_auv.yaml", "reverse_limit_n: 28",
            "reverse_limit_n: 28\n      propeller_radius_m: 0.05");
    const auto config = robotics::config::loadScenario(scenario());
    EXPECT_EQ(config.plant.pool.current_oscillation_amplitude, Eigen::Vector3d(.2, .1, .04));
    EXPECT_DOUBLE_EQ(config.plant.pool.current_oscillation_frequency, .7);
    ASSERT_TRUE(config.plant.thrusters.front().propeller_radius);
    EXPECT_DOUBLE_EQ(*config.plant.thrusters.front().propeller_radius, .05);
    EXPECT_NO_THROW(robotics::config::makeRuntime(config)->advance());
    replace("robots/synthetic_auv.yaml", "propeller_radius_m: 0.05", "propeller_radius_m: -1");
    EXPECT_THROW(robotics::config::loadScenario(scenario()), std::invalid_argument);
}

TEST_F(Profiles, RobotActuatorCalibrationReachesNativeRuntime) {
    replace("robots/synthetic_auv.yaml", "reverse_limit_n: 28",
            "reverse_limit_n: 28\n      deadband_n: 2\n      forward_scale: 1.5\n"
            "      reverse_scale: 0.8\n      efficiency: 0.7");
    const auto config = robotics::config::loadScenario(scenario());
    const auto &thruster = config.plant.thrusters.front();
    EXPECT_DOUBLE_EQ(thruster.deadband, 2);
    EXPECT_DOUBLE_EQ(thruster.forward_scale, 1.5);
    EXPECT_DOUBLE_EQ(thruster.reverse_scale, .8);
    EXPECT_DOUBLE_EQ(thruster.efficiency, .7);
    replace("robots/synthetic_auv.yaml", "efficiency: 0.7", "efficiency: 1.01");
    EXPECT_THROW(robotics::config::loadScenario(scenario()), std::invalid_argument);
}

TEST_F(Profiles, ContactPolicyComposesRobotAndWorldGeometryIndependently) {
    replace("examples/profile_pool.yaml", "schema_version: 2",
            "schema_version: 2\ncontacts: {model: box_scene, restitution: 0.2, friction: 0.3}");
    write("worlds/empty_pool.yaml",
          read("worlds/empty_pool.yaml") +
              "\ncollision_boxes:\n  - {id: floor, size_m: [20, 10, 1], center_m: [10, 5, -3.5], "
              "orientation_wxyz: [1, 0, 0, 0]}\n");
    const auto config = robotics::config::loadScenario(scenario());
    EXPECT_EQ(config.plant.contacts.model, robotics::simulation::ContactModel::BoxScene);
    ASSERT_EQ(config.plant.contacts.body_boxes.size(), 1U);
    ASSERT_EQ(config.plant.contacts.world_boxes.size(), 1U);
    EXPECT_EQ(config.plant.contacts.body_boxes[0].id, "hull");
    EXPECT_EQ(config.plant.contacts.world_boxes[0].center, Eigen::Vector3d(10, 5, -3.5));
    EXPECT_DOUBLE_EQ(config.plant.contacts.restitution, .2);
    EXPECT_DOUBLE_EQ(config.plant.contacts.friction, .3);
    EXPECT_NO_THROW(robotics::config::makeRuntime(config)->advance());
    replace("examples/profile_pool.yaml", "model: box_scene", "model: typo");
    EXPECT_THROW(robotics::config::loadScenario(scenario()), std::invalid_argument);
}

TEST_F(Profiles, NamedMountsMatchExplicitSensorGeometryAndSurviveSourceRemoval) {
    replace("examples/profile_pool.yaml", "linear_velocity_m_s: [0, 0, 0]",
            "linear_velocity_m_s: [0.3, -0.2, 0.1]");
    const std::string simple_mount =
        "mount: {position_m: [0, 0, -0.1], orientation_wxyz: [1, 0, 0, 0]}";
    const std::string inline_mount = "mount: {position_m: [0, 0, -0.1], orientation_wxyz: "
                                     "[0.7071067811865476, 0, 0, 0.7071067811865476]}";
    replace("robots/synthetic_auv.yaml", simple_mount, inline_mount);
    replace("robots/synthetic_auv.yaml", simple_mount, inline_mount);
    const auto baseline = robotics::config::loadScenario(scenario());
    const auto original = read("robots/synthetic_auv.yaml");
    const std::string frames = R"(
frames:
  root: center
  transforms:
    - {parent: offset, child: mount, position_m: [0, 0.2, -0.3], orientation_wxyz: [1, 0, 0, 0]}
    - {parent: center, child: offset, position_m: [0.2, 0, 0.2], orientation_wxyz: [0.7071067811865476, 0, 0, 0.7071067811865476]}
)";
    write("robots/synthetic_auv.yaml", original + frames);
    replace("robots/synthetic_auv.yaml", inline_mount, "mount_frame: mount"); // DVL
    replace("robots/synthetic_auv.yaml", inline_mount, "mount_frame: mount"); // Pressure
    const auto named = robotics::config::loadScenario(scenario());
    EXPECT_EQ(named.body_frames.root(), "center");
    EXPECT_TRUE(named.body_frames.fromRoot("mount").translation.isApprox(Eigen::Vector3d(0, 0, -.1),
                                                                         1e-14));
    std::filesystem::remove_all(root);
    auto a = robotics::config::makeRuntime(baseline);
    auto b = robotics::config::makeRuntime(named);
    a->advance(60);
    b->advance(60);
    const auto ap = a->stream<robotics::sensors::PressureReading>("pressure")->latest();
    const auto bp = b->stream<robotics::sensors::PressureReading>("pressure")->latest();
    ASSERT_TRUE(ap && bp && ap->measurement.value && bp->measurement.value);
    EXPECT_DOUBLE_EQ(ap->measurement.value->absolute_pressure,
                     bp->measurement.value->absolute_pressure);
    const auto ad = a->stream<robotics::sensors::DvlReading>("dvl")->latest();
    const auto bd = b->stream<robotics::sensors::DvlReading>("dvl")->latest();
    ASSERT_TRUE(ad && bd && ad->measurement.value && bd->measurement.value);
    EXPECT_TRUE(ad->measurement.value->bottom_relative_velocity.isApprox(
        bd->measurement.value->bottom_relative_velocity, 1e-14));
}
TEST_F(Profiles, RejectsMissingOrAmbiguousNamedMounts) {
    const auto original = read("robots/synthetic_auv.yaml");
    replace("robots/synthetic_auv.yaml", "id: imu", "id: imu\n    mount_frame: absent");
    EXPECT_THROW(robotics::config::loadScenario(scenario()), std::invalid_argument);
    write("robots/synthetic_auv.yaml", original);
    replace("robots/synthetic_auv.yaml",
            "mount: {position_m: [0, 0, 0], orientation_wxyz: [1, 0, 0, 0]}",
            "mount_frame: absent");
    EXPECT_THROW(robotics::config::loadScenario(scenario()), std::invalid_argument);
    write("robots/synthetic_auv.yaml",
          original + "\nframes: {root: center, transforms: [{parent: unknown, child: mount, "
                     "position_m: [0,0,0], orientation_wxyz: [1,0,0,0]}]}\n");
    EXPECT_THROW(robotics::config::loadScenario(scenario()), std::invalid_argument);
}

namespace {
using namespace robotics::spatial;
TEST(ConfigurationFrames, TalosCadAndBaseRenderingAgreeWithComPose) {
    const Eigen::Vector3d cad_com(-.157, .040, -.048), cad_base(-.140, .030, -.090);
    FixedFrames frames("com", {{"com", "cad", {-cad_com, Eigen::Quaterniond::Identity()}},
                               {"cad", "base", {cad_base, Eigen::Quaterniond::Identity()}}});
    const Pose world_com{
        {-2, 5, -.4},
        Eigen::Quaterniond(Eigen::AngleAxisd(1.2, Eigen::Vector3d(1, 2, 3).normalized()))};
    const auto world_base = compose(world_com, frames.fromRoot("base"));
    const auto world_cad = compose(world_base, frames.lookup("base", "cad"));
    for (const Eigen::Vector3d &point :
         {Eigen::Vector3d(-.181, .1573, .094), Eigen::Vector3d(-.134, -.369, .242)}) {
        EXPECT_TRUE(apply(world_cad, point).isApprox(apply(world_com, point - cad_com), 1e-14));
    }
}
} // namespace

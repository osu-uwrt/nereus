// Offline test wrapper: original kernels/equations plus pinned original YAML/URDF values.
#include <algorithm>
#include <c_simulator/MarineDynamics.h>
#include <c_simulator/ThrusterDynamics.h>
#include <c_simulator/collisionBox.h>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <yaml-cpp/yaml.h>

#define RCLCPP_DEBUG(...) ((void)0)
quat state2quat(const vXd &state) {
    return quat(state[3], state[4], state[5], state[6]).normalized();
}
v3d vector3(const YAML::Node &node) {
    return {node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
}
quat rpy(double roll, double pitch, double yaw) {
    // Same fixed-axis RPY convention as tf2::Quaternion::setRPY in Robot::rpy2quat.
    const double cr = cos(roll / 2), sr = sin(roll / 2), cp = cos(pitch / 2), sp = sin(pitch / 2),
                 cy = cos(yaw / 2), sy = sin(yaw / 2);
    return quat(cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)
        .normalized();
}
c_simulator::Matrix6d matrix6(const YAML::Node &node) {
    c_simulator::Matrix6d result;
    for (int row = 0; row < 6; ++row)
        for (int col = 0; col < 6; ++col)
            result(row, col) = node[row][col].as<double>();
    return result;
}
struct Reference {
    c_simulator::MarineDynamics marineDynamics;
    c_simulator::ThrusterDynamics actuator;
    struct Robot {
        c_simulator::Matrix6d inverse;
        c_simulator::Matrix6d getInverseMass() const {
            return inverse;
        }
    } robot;
    std::vector<collisionBox> robotBoxes, obstacleBoxes;
    double restitution = .1, contactFriction = .4;
    v3d waterCurrentWorld, currentAmplitude;
    double currentFrequency, propellerRadius;
    int thrusterCount;
    std::vector<v3d> thrusterPositions, thrusterDirections;
    Eigen::MatrixXd thrusterMatrix;
    explicit Reference(const std::filesystem::path &root) {
        const auto vehicle = YAML::LoadFile((root / "vehicle.yaml").string());
        const auto hydro = YAML::LoadFile((root / "hydro.yaml").string());
        const auto world = YAML::LoadFile((root / "world.yaml").string());
        const auto mapping = YAML::LoadFile((root / "mapping.yaml").string());
        const auto proxies = YAML::LoadFile((root / "proxies.json").string());
        const auto com = vector3(vehicle["com"]);
        m3d inertia;
        for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 3; ++col)
                inertia(row, col) = hydro["rigid_body_inertia3x3"][row * 3 + col].as<double>();
        marineDynamics.configure(vehicle["mass"].as<double>(), inertia,
                                 matrix6(hydro["added_mass6x6"]));
        c_simulator::Vector6d quadratic;
        for (int i = 0; i < 6; ++i)
            quadratic[i] = hydro["quadratic_damping"][i].as<double>();
        marineDynamics.configureDamping(matrix6(hydro["linear_damping6x6"]), quadratic,
                                        vector3(hydro["damping_center_relative"]));
        marineDynamics.configureHydrostatics(
            world["water_density"].as<double>(), hydro["displaced_volume"].as<double>(),
            vector3(hydro["cob_relative"]), vector3(hydro["buoyancy_radii"]), 9.80665,
            world["water_level"].as<double>());
        robot.inverse = marineDynamics.inverseMass();
        waterCurrentWorld = vector3(world["current_velocity"]);
        currentAmplitude = vector3(world["current_oscillation_amplitude"]);
        currentFrequency = world["current_oscillation_frequency"].as<double>();
        const auto dynamics = hydro["thruster_dynamics"];
        propellerRadius = dynamics["propeller_radius"].as<double>();
        thrusterCount = static_cast<int>(vehicle["thrusters"].size());
        thrusterMatrix.resize(thrusterCount, 6);
        std::vector<c_simulator::ThrusterParameters> parameters;
        for (int i = 0; i < thrusterCount; ++i) {
            const auto pose = vehicle["thrusters"][i]["pose"];
            const v3d position = vector3(pose) - com;
            const v3d direction =
                rpy(pose[3].as<double>(), pose[4].as<double>(), pose[5].as<double>()) *
                v3d::UnitX();
            thrusterPositions.push_back(position);
            thrusterDirections.push_back(direction);
            thrusterMatrix.row(i) << direction.transpose(), position.cross(direction).transpose();
            c_simulator::ThrusterParameters p;
            p.delay = dynamics["delay"].as<double>();
            p.rise = dynamics["rise_time_constant"].as<double>();
            p.fall = dynamics["fall_time_constant"].as<double>();
            p.slew = dynamics["slew_rate"].as<double>();
            p.deadband = dynamics["force_deadband"].as<double>();
            p.forwardLimit = dynamics["forward_max_force"].as<double>();
            p.reverseLimit = dynamics["reverse_max_force"].as<double>();
            p.forwardScale = dynamics["forward_scale"].as<double>();
            p.reverseScale = dynamics["reverse_scale"].as<double>();
            p.efficiency = hydro["thruster_efficiencies"][i].as<double>();
            parameters.push_back(p);
        }
        actuator.configure(parameters, dynamics["command_timeout"].as<double>());
        for (std::size_t i = 0; i < proxies.size(); ++i) {
            const auto p = proxies[i];
            const auto size = vector3(p["size"]), angles = vector3(p["rpy"]);
            robotBoxes.emplace_back("body" + std::to_string(i), size.x(), size.y(), size.z(),
                                    v3d::Zero(), vector3(p["xyz"]), quat::Identity(),
                                    rpy(angles.x(), angles.y(), angles.z()));
        }
        const auto origin = mapping["/**/zed_faker"]["ros__parameters"]["map_origin_pool"];
        Eigen::Isometry3d mapToPool = Eigen::Isometry3d::Identity();
        mapToPool.translate(v3d(origin[0].as<double>(), origin[1].as<double>(), 0));
        mapToPool.rotate(Eigen::AngleAxisd(origin[2].as<double>() * M_PI / 180, v3d::UnitZ()));
        const auto poolToMap = mapToPool.inverse();
        auto poolBox = [&](const std::string &name, const v3d &size, const v3d &center) {
            obstacleBoxes.emplace_back(name, size.x(), size.y(), size.z(), poolToMap * center,
                                       v3d::Zero(), quat(poolToMap.rotation()));
        };
        const double length = world["length"].as<double>(), width = world["width"].as<double>(),
                     depth = world["depth"].as<double>(),
                     surface = world["water_level"].as<double>();
        const double height = depth + world["deck_height"].as<double>() + 1.,
                     middle = surface - depth - .5 + height / 2;
        poolBox("floor", {length, width, 1}, {length / 2, width / 2, surface - depth - .5});
        poolBox("west", {1, width + 2, height}, {-.5, width / 2, middle});
        poolBox("east", {1, width + 2, height}, {length + .5, width / 2, middle});
        poolBox("south", {length, 1, height}, {length / 2, -.5, middle});
        poolBox("north", {length, 1, height}, {length / 2, width + .5, middle});
    }
#include "legacy_contacts.inc"
    vXd stateDerivative(const vXd &, double stageOffset = 0) const;
    c_simulator::Vector6d propulsionWrench(const vXd &) const;
    vXd calcStateDot(const vXd &state, double offset = 0) const {
        return stateDerivative(state, offset);
    }
    vXd step(vXd state) {
        constexpr double physicsStep = .002;
        actuator.advance(physicsStep / 2);
        state = handleCollisions(state);
#include "legacy_rk4.inc"
        advanced = handleCollisions(advanced);
        const quat q = state2quat(advanced);
        advanced[3] = q.w();
        advanced.segment<3>(4) = q.vec();
        actuator.advance(physicsStep / 2);
        return advanced;
    }
};
#include "legacy_equations.inc"
int main(int argc, char **argv) {
    if (argc != 2)
        return 2;
    std::cout << std::setprecision(17) << "case,tick,x,y,z,qw,qx,qy,qz,u,v,w,p,q,r";
    for (int i = 0; i < 8; ++i)
        std::cout << ",force" << i;
    std::cout << ",ax,ay,az,alphax,alphay,alphaz\n";
    for (int scenario = 0; scenario < 3; ++scenario) {
        Reference reference(argv[1]);
        vXd state = vXd::Zero(13);
        state[3] = 1;
        if (scenario == 0)
            state.head<3>() << 0, 0, -1;
        else if (scenario == 1) {
            state.head<3>() << 11.43, -5.4864, .05;
            const auto q = rpy(.1, .2, -.3);
            state.segment<4>(3) << q.w(), q.x(), q.y(), q.z();
            state.segment<3>(7) << .1, -.05, .02;
            state.tail<3>() << .3, .2, .1;
        } else {
            state.head<3>() << 11.43, -5.4864, -1.8836;
            state.segment<3>(7) << .2, .1, -.8;
        }
        for (int tick = 0; tick <= 1500; ++tick) {
            {
                const auto derivative = reference.stateDerivative(state);
                const v3d acceleration =
                    derivative.segment<3>(7) + state.tail<3>().cross(state.segment<3>(7));
                std::cout << scenario << ',' << tick;
                for (int i = 0; i < 13; ++i)
                    std::cout << ',' << state[i];
                for (int i = 0; i < 8; ++i)
                    std::cout << ',' << reference.actuator.forces()[i];
                for (int i = 0; i < 3; ++i)
                    std::cout << ',' << acceleration[i];
                for (int i = 10; i < 13; ++i)
                    std::cout << ',' << derivative[i];
                std::cout << '\n';
            }
            if (tick == 1500)
                break;
            if (tick <= 1000 && tick % 200 == 0) {
                vXd command(8);
                command << 4, 4, 2, 2, -2, -2, 4, 4;
                if (tick == 1000)
                    command.setZero();
                reference.actuator.command(command);
            }
            state = reference.step(state);
        }
    }
}

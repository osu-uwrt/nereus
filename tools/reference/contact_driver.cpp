// Test-only wrapper around extracted legacy static-box contact equations.
#include <algorithm>
#include <c_simulator/MarineDynamics.h>
#include <c_simulator/collisionBox.h>
#include <iomanip>
#include <iostream>
#include <memory>

#define RCLCPP_DEBUG(...) ((void)0)
quat state2quat(const vXd &state) {
    return quat(state[3], state[4], state[5], state[6]).normalized();
}
struct Reference {
    struct Robot {
        c_simulator::Matrix6d inverse;
        c_simulator::Matrix6d getInverseMass() const {
            return inverse;
        }
    } robot;
    c_simulator::MarineDynamics dynamics;
    std::vector<collisionBox> robotBoxes, obstacleBoxes;
    double restitution = .1, contactFriction = .4;
#include "legacy_contacts.inc"
    vXd calcStateDot(const vXd &state, double = 0) const {
        return dynamics.derivative(state, c_simulator::Vector6d::Zero());
    }
    vXd step(vXd state) {
        constexpr double physicsStep = .002;
        state = handleCollisions(state);
#include "legacy_rk4.inc"
        advanced = handleCollisions(advanced);
        const quat q = state2quat(advanced);
        advanced[3] = q.w();
        advanced.segment<3>(4) = q.vec();
        return advanced;
    }
};
int main(int argc, char **) {
    const bool integrate = argc > 1;
    std::cout << std::setprecision(17) << (integrate ? "case,tick," : "case,")
              << "x,y,z,qw,qx,qy,qz,u,v,w,p,q,r\n";
    for (int scenario = 0; scenario < 11; ++scenario) {
        Reference reference;
        auto &dynamics = reference.dynamics;
        c_simulator::Matrix6d added = c_simulator::Matrix6d::Identity();
        added(0, 4) = added(4, 0) = .2;
        dynamics.configure(10, Eigen::Vector3d(.3, .4, .5).asDiagonal(), added);
        reference.robot.inverse = dynamics.inverseMass();
        const quat local =
            scenario == 10 ? quat(Eigen::AngleAxisd(.3, v3d::UnitX())) : quat::Identity();
        reference.robotBoxes.emplace_back("hull", .6, .4, .3, v3d::Zero(), v3d(.05, -.02, .03),
                                          quat::Identity(), local);
        reference.robotBoxes.emplace_back("probe", .2, .05, .05, v3d::Zero(), v3d(.4, -.2, -.1));
        reference.obstacleBoxes.emplace_back("floor", 10, 10, 1, v3d(0, 0, -.5));
        reference.obstacleBoxes.emplace_back("wall", 1, 10, 3, v3d(1, 0, 1));
        vXd state = vXd::Zero(13);
        state.head<3>() << (scenario >= 4 ? .45 : 0), 0, (scenario < 4 ? .1 : .3);
        const quat q = quat(Eigen::AngleAxisd(.1 * scenario, v3d::UnitZ())) *
                       quat(Eigen::AngleAxisd(.03 * scenario, v3d::UnitY()));
        state.segment<4>(3) << q.w(), q.x(), q.y(), q.z();
        if (scenario == 6)
            state.segment<4>(3) *= 1.01; // Integration-stage quaternion need not be normalized yet.
        state.segment<3>(7) << (scenario >= 4 ? .5 : .2), .3, (scenario % 2 == 0 ? -.4 : .4);
        state.tail<3>() << .1, -.2, .3;
        if (scenario == 8) {
            reference.obstacleBoxes.pop_back();
            state.head<3>() << 0, 0, .1;
            state.segment<4>(3) << 1, 0, 0, 0;
            state.segment<3>(7) << .2, .3, .4;
            state.tail<3>().setZero();
        } else if (scenario == 9) {
            state.head<3>() << -2, 0, 2; // Disjoint SAT queries with nonempty obstacles.
        }
        if (scenario == 3) {
            reference.obstacleBoxes.clear(); // No contacts.
        } else if (scenario == 7) {
            reference.obstacleBoxes.clear();
            reference.obstacleBoxes.emplace_back("tilted", 1, 3, 3, v3d(1, 0, 1), v3d::Zero(),
                                                 quat(Eigen::AngleAxisd(.4, v3d::UnitZ())));
        }
        if (integrate) {
            const auto normalized = state2quat(state);
            state[3] = normalized.w();
            state.segment<3>(4) = normalized.vec();
        }
        for (int tick = 1; tick <= (integrate ? 50 : 1); ++tick) {
            state = integrate ? reference.step(state) : reference.handleCollisions(state);
            std::cout << scenario;
            if (integrate)
                std::cout << ',' << tick;
            for (int i = 0; i < 13; ++i)
                std::cout << ',' << state[i];
            std::cout << '\n';
        }
    }
}

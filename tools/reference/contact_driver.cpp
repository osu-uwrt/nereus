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
    std::vector<collisionBox> robotBoxes, obstacleBoxes;
    double restitution = .1, contactFriction = .4;
#include "legacy_contacts.inc"
};
int main() {
    std::cout << std::setprecision(17) << "case,x,y,z,qw,qx,qy,qz,u,v,w,p,q,r\n";
    for (int scenario = 0; scenario < 11; ++scenario) {
        Reference reference;
        c_simulator::MarineDynamics dynamics;
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
        const auto result = reference.handleCollisions(state);
        std::cout << scenario;
        for (int i = 0; i < 13; ++i)
            std::cout << ',' << result[i];
        std::cout << '\n';
    }
}

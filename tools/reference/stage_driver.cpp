// Test-only driver for pinned legacy equations; never linked into the platform.
#include <algorithm>
#include <c_simulator/MarineDynamics.h>
#include <c_simulator/ThrusterDynamics.h>
#include <cmath>
#include <iomanip>
#include <iostream>

using vXd = Eigen::VectorXd;
using quat = Eigen::Quaterniond;
struct Reference {
    c_simulator::MarineDynamics marineDynamics;
    c_simulator::ThrusterDynamics actuator;
    Eigen::Vector3d waterCurrentWorld{.15, -.1, .05}, currentAmplitude{.2, .1, .04};
    double currentFrequency{.7}, propellerRadius{.05};
    int thrusterCount{1};
    std::vector<Eigen::Vector3d> thrusterPositions{{.1, -.2, .025}};
    std::vector<Eigen::Vector3d> thrusterDirections{Eigen::Vector3d::UnitX()};
    Eigen::Matrix<double, 1, 6> thrusterMatrix;
    Reference() {
        c_simulator::Matrix6d added = c_simulator::Matrix6d::Zero();
        added.diagonal() << 2, 3, 4, .1, .2, .3;
        marineDynamics.configure(10, Eigen::Vector3d(.3, .4, .5).asDiagonal(), added);
        marineDynamics.configureDamping(c_simulator::Matrix6d::Identity() * .3,
                                        c_simulator::Vector6d::Constant(.1));
        marineDynamics.configureHydrostatics(1000, .01, Eigen::Vector3d::Zero(), {.2, .2, .2});
        c_simulator::ThrusterParameters p;
        p.delay = .004;
        actuator.configure({p}, .5);
        thrusterMatrix << thrusterDirections[0].transpose(),
            thrusterPositions[0].cross(thrusterDirections[0]).transpose();
    }
    vXd stateDerivative(const vXd &, double stageOffset = 0) const;
    c_simulator::Vector6d propulsionWrench(const vXd &) const;
    vXd calcStateDot(const vXd &state, double offset = 0) const {
        return stateDerivative(state, offset);
    }
    vXd step(vXd state) {
        constexpr double physicsStep = .002;
        actuator.advance(physicsStep / 2);
#include "legacy_rk4.inc"
        quat q(advanced[3], advanced[4], advanced[5], advanced[6]);
        q.normalize();
        advanced[3] = q.w();
        advanced.segment(4, 3) = q.vec();
        actuator.advance(physicsStep / 2);
        return advanced;
    }
};
#include "legacy_equations.inc"
int main() {
    std::cout << std::setprecision(17);
    std::cout << "case,tick,x,y,z,qw,qx,qy,qz,u,v,w,p,q,r,force,ax,ay,az,alphax,alphay,alphaz\n";
    for (int scenario = 0; scenario < 3; ++scenario) {
        Reference reference;
        vXd state = vXd::Zero(13);
        state.head<3>() << 5, 5, (scenario == 0 ? -.025 : scenario == 1 ? -2 : .2);
        const quat orientation(Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitY()));
        state.segment<4>(3) << orientation.w(), orientation.x(), orientation.y(), orientation.z();
        state.segment<3>(7) << .1, -.05, .02;
        state.tail<3>() << .3, .2, .1;
        for (int tick = 0; tick <= 250; ++tick) {
            const auto derivative = reference.stateDerivative(state);
            const Eigen::Vector3d acceleration =
                derivative.segment<3>(7) + state.tail<3>().cross(state.segment<3>(7));
            std::cout << scenario << ',' << tick;
            for (int i = 0; i < 13; ++i)
                std::cout << ',' << state[i];
            std::cout << ',' << reference.actuator.forces()[0];
            for (int i = 0; i < 3; ++i)
                std::cout << ',' << acceleration[i];
            for (int i = 10; i < 13; ++i)
                std::cout << ',' << derivative[i];
            std::cout << '\n';
            if (tick == 250)
                break;
            if (tick == 0 || tick == 100 || tick == 200)
                reference.actuator.command(Eigen::VectorXd::Constant(1, tick == 0     ? 12
                                                                        : tick == 100 ? -6
                                                                                      : 0));
            state = reference.step(state);
        }
    }
}

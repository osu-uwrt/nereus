// Offline wrapper for extracted original measurement expressions; never linked to the platform.
#include "settings.h"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <vector>
#include <yaml-cpp/yaml.h>
using v3d = Eigen::Vector3d;
using vXd = Eigen::VectorXd;
using quat = Eigen::Quaterniond;
v3d std2v3d(const std::vector<double> &v) {
    return {v.at(0), v.at(1), v.at(2)};
}
quat rpy2quat(double roll, double pitch, double yaw) {
    const double cr = cos(roll / 2), sr = sin(roll / 2), cp = cos(pitch / 2), sp = sin(pitch / 2),
                 cy = cos(yaw / 2), sy = sin(yaw / 2);
    return quat(cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)
        .normalized();
}
quat state2quat(const vXd &state) {
    return quat(state[3], state[4], state[5], state[6]).normalized();
}
template <class T> T declare_parameter(const char *, T value) {
    return value;
}
struct Robot {
    vXd state = vXd::Zero(13);
    v3d r_com, r_base, r_imu, r_depth, r_dvl, linAccel, angAccel;
    quat q_imu, q_gyro = quat::Identity(), q_dvl;
    double imu_rate, imu_yawDrift, imu_sigmaAccel, imu_sigmaOmega, imu_sigmaAngle, depth_rate,
        depth_sigma, dvl_rate, dvl_sigma;
    v3d imu_orientationVariance, imu_angularVelocityVariance, imu_linearAccelerationVariance;
    explicit Robot(const std::filesystem::path &root) {
        const auto vehicle_config = YAML::LoadFile((root / "vehicle.yaml").string());
        const auto simulator_config = YAML::LoadFile((root / "simulator.yaml").string());
        r_com = std2v3d(vehicle_config["com"].as<std::vector<double>>());
        r_base = std2v3d(vehicle_config["base_link"].as<std::vector<double>>()) - r_com;
#include "sensor_init.inc"
    }
    const vXd &getState() const {
        return state;
    }
    v3d getLatestLinAccel() const {
        return linAccel;
    }
    v3d getLatestAngAccel() const {
        return angAccel;
    }
    v3d getIMUOffset() const {
        return r_imu;
    }
    quat getIMUQuat() const {
        return q_imu;
    }
    quat getGyroQuat() const {
        return q_gyro;
    }
    v3d getDVLOffset() const {
        return r_dvl;
    }
    quat getDVLQuat() const {
        return q_dvl;
    }
    v3d getDepthOffset() const {
        return r_depth;
    }
    v3d getBaseLinkOffset() const {
        return r_base;
    }
    void setAccel(const vXd &stateDot);
};
#include "acceleration.inc"
int main(int argc, char **argv) {
    if (argc != 2)
        return 2;
    const std::filesystem::path root(argv[1]);
    Robot robot(root);
    const auto overrides = YAML::LoadFile(
        (root / "sensors.yaml").string())["/**/physics_simulator"]["ros__parameters"];
    const double imuGravity = overrides["imu_gravity"].as<double>();
    double gyroSigma, gyroVariance;
#include "gyro_defaults.inc"
    std::cout << std::setprecision(17)
              << "case,x,y,z,qw,qx,qy,qz,u,v,w,p,q,r,du,dv,dw,dp,dq,dr,fx,fy,fz,gx,gy,gz,ow,ox,oy,"
                 "oz,ovx,ovy,ovz,gvx,gvy,gvz,fvx,fvy,fvz,fog,fog_var,dvl_x,dvl_y,dvl_z,dvl_var,"
                 "depth_point_z,base_z,depth_var\n";
    for (int i = 0; i < 24; ++i) {
        auto &state = robot.state;
        state.head<3>() << -1 + .2 * i, 2 - .1 * i, -.08 * (i - 2);
        const auto body_q = rpy2quat(.13 * i - .9, .11 * (i % 9) - .4, .27 * i - .6);
        state.segment<4>(3) << body_q.w(), body_q.x(), body_q.y(), body_q.z();
        state.segment<3>(7) << .2 * cos(i), .3 * sin(i), -.1 + .02 * i;
        state.tail<3>() << .4 * sin(i), .3 * cos(i), .01 * i;
        vXd derivative = vXd::Zero(13);
        derivative.tail<6>() << .1 * i - .4, -.2 * cos(i), .3 * sin(i), .2 * cos(i), -.1 * sin(i),
            .03 * i;
        if (i == 0 || i == 1) {
            state.segment<4>(3) << 1, 0, 0, 0;
            state.tail<6>().setZero();
            derivative.setZero();
            if (i == 1)
                derivative[9] = -GRAVITY; // Physical free fall.
        }
        robot.setAccel(derivative);
        std::cout << i;
        for (const auto value : state)
            std::cout << ',' << value;
        for (const auto value : derivative.tail<6>())
            std::cout << ',' << value;
        {
#include "imu.inc"
            for (const auto value : imuAccel)
                std::cout << ',' << value;
            for (const auto value : angularVel)
                std::cout << ',' << value;
            std::cout << ',' << q.w() << ',' << q.x() << ',' << q.y() << ',' << q.z();
            for (const auto &variance :
                 {robot.imu_orientationVariance, robot.imu_angularVelocityVariance,
                  robot.imu_linearAccelerationVariance})
                for (const auto value : variance)
                    std::cout << ',' << value;
        }
        {
#include "fog.inc"
            std::cout << ',' << omega.z() << ',' << gyroVariance;
        }
        {
#include "dvl.inc"
            for (const auto value : dvlData)
                std::cout << ',' << value;
            std::cout << ',' << overrides["dvl_variance"].as<double>();
        }
        {
#include "depth.inc"
            std::cout << ',' << depthData << ',' << base_z << ','
                      << robot.depth_sigma * robot.depth_sigma;
        }
        std::cout << '\n';
    }
}

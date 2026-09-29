#!/usr/bin/env python3
"""Capture noise-disabled original sensor formulas at prescribed COM kinematics."""

import argparse
import hashlib
import json
import subprocess
import tempfile
from pathlib import Path

from capture_stage_reference import REVISION, ROOT
from capture_talos_reference import VEHICLE_REVISION


def section(source, start, end):
    begin = source.index(start)
    return source[begin : source.index(end, begin)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("simulator_repo", type=Path)
    parser.add_argument("vehicle_repo", type=Path)
    args = parser.parse_args()
    sources = {}

    def read(repo, revision, name):
        data = subprocess.check_output(["git", "-C", str(repo), "show", f"{revision}:{name}"])
        sources[f"{revision}:{name}"] = hashlib.sha256(data).hexdigest()
        return data

    def sim(name):
        return read(args.simulator_repo, REVISION, "c_simulator/" + name)

    robot = sim("src/robot_class.cpp").decode()
    physics = sim("src/physics_simulator.cpp").decode()
    driver = ROOT / "tools/reference/sensor_driver.cpp"
    with tempfile.TemporaryDirectory(prefix="legacy-sensor-reference-") as directory:
        root = Path(directory)
        for filename, path in (
            ("vehicle.yaml", "talos.yaml"),
            ("simulator.yaml", "simulator.yaml"),
        ):
            (root / filename).write_bytes(
                read(args.vehicle_repo, VEHICLE_REVISION, "riptide_descriptions/config/" + path)
            )
        (root / "sensors.yaml").write_bytes(sim("robots/talos/config/sensors.yaml"))
        (root / "settings.h").write_bytes(sim("include/c_simulator/settings.h"))
        (root / "sensor_init.inc").write_text(
            section(robot, "    std::vector<double> imu_pose", "    // Creating thruster forces")
        )
        (root / "acceleration.inc").write_text(
            section(robot, "void Robot::setAccel(", "void Robot::addToThrusterQue(")
        )
        imu = section(
            physics, "    void publishFakeIMUData()", "    void publishFakeAcousticsData()"
        )
        (root / "imu.inc").write_text(
            section(imu, "            vXd state =", "            // Add noise to sensor data")
        )
        fog = section(physics, "    void publishFakeGyroData()", "    void publishFakeIMUData()")
        (root / "fog.inc").write_text(
            section(fog, "        const v3d omega", "        geometry_msgs::")
        )
        (root / "gyro_defaults.inc").write_text(
            section(
                physics,
                "        gyroSigma = declare_parameter<double>",
                "        if (!std::isfinite(gyroRate)",
            )
        )
        dvl = section(physics, "    void publishFakeDVLData()", "    void publishFakeGyroData()")
        (root / "dvl.inc").write_text(
            section(dvl, "            const v3d angularVel", "            // Add nonise")
        )
        depth = section(physics, "    void publishFakeDepthData()", "    void publishFakeDVLData()")
        depth_point = section(depth, "        double depthData", "        if (noiseEnabled")
        correction = section(
            depth,
            "        depthMsg.pose.pose.position.z =",
            "        depthMsg.pose.pose.orientation",
        )
        (root / "depth.inc").write_text(
            depth_point + correction.replace("depthMsg.pose.pose.position.z", "const double base_z")
        )
        flags = ["-std=c++17", "-O2"]
        subprocess.run(
            [
                "c++",
                *flags,
                "-I/usr/include/eigen3",
                f"-I{root}",
                str(driver),
                "-lyaml-cpp",
                "-o",
                str(root / "capture"),
            ],
            check=True,
        )
        output = subprocess.check_output([str(root / "capture"), str(root)])
    destination = ROOT / "tests/fixtures/legacy_sensor_kinematics.csv"
    destination.write_bytes(output)
    destination.with_suffix(".json").write_text(
        json.dumps(
            {
                "sources_sha256": sources,
                "driver_sha256": hashlib.sha256(driver.read_bytes()).hexdigest(),
                "capture_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                "fixture_sha256": hashlib.sha256(output).hexdigest(),
                "compiler_flags": flags,
                "compiler": subprocess.check_output(["c++", "--version"], text=True).splitlines()[
                    0
                ],
                "scope": "24 prescribed COM states and body velocity derivatives. Extracted original setAccel, noise-disabled IMU/FOG, default no-lock-gate DVL, mounted depth and corrected base Z. Original YAML and settings only; no native profile/model/importer input. Excludes RNG equality, drift, schedules, plant integration, contacts, cameras and ROS transport.",
            },
            indent=2,
        )
        + "\n"
    )
    print(f"Captured {len(output.splitlines()) - 1} prescribed-state sensor references")


if __name__ == "__main__":
    main()

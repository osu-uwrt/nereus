#!/usr/bin/env python3
"""Capture original Talos physics independently of the native importer and platform."""

import argparse
import hashlib
import json
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from capture_stage_reference import FILES as STAGE_FILES
from capture_stage_reference import REVISION, ROOT

VEHICLE_REVISION = "7f37bdd62ab90113844137a23806c89b23a8ab9b"
FILES = tuple(
    dict.fromkeys(
        (
            *STAGE_FILES,
            "c_simulator/include/c_simulator/collisionBox.h",
            "c_simulator/include/c_simulator/settings.h",
            "c_simulator/src/collisionBox_class.cpp",
            "c_simulator/robots/talos/config/hydrodynamics.yaml",
            "c_simulator/worlds/competition_pool.yaml",
            "c_simulator/tasks/2026/config/mapping.yaml",
            "c_simulator/collision_files/robots/talos.urdf",
        )
    )
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("simulator_repo", type=Path)
    parser.add_argument("vehicle_repo", type=Path)
    args = parser.parse_args()
    sources = {
        name: subprocess.check_output(
            ["git", "-C", str(args.simulator_repo), "show", f"{REVISION}:{name}"]
        )
        for name in FILES
    }
    vehicle = subprocess.check_output(
        [
            "git",
            "-C",
            str(args.vehicle_repo),
            "show",
            f"{VEHICLE_REVISION}:riptide_descriptions/config/talos.yaml",
        ]
    )
    driver = ROOT / "tools/reference/talos_driver.cpp"
    with tempfile.TemporaryDirectory(prefix="legacy-talos-reference-") as directory:
        temporary = Path(directory)
        for name, data in sources.items():
            destination = temporary / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(data)
        for short, original in {
            "hydro.yaml": "c_simulator/robots/talos/config/hydrodynamics.yaml",
            "world.yaml": "c_simulator/worlds/competition_pool.yaml",
            "mapping.yaml": "c_simulator/tasks/2026/config/mapping.yaml",
        }.items():
            (temporary / short).write_bytes(sources[original])
        (temporary / "vehicle.yaml").write_bytes(vehicle)
        # Lossless numeric extraction of pinned URDF collision entries; no native profile input.
        proxies = []
        xml = ET.fromstring(sources["c_simulator/collision_files/robots/talos.urdf"])
        for collision in xml.findall(".//collision"):
            proxies.append(
                {
                    "size": [
                        float(v) for v in collision.find("geometry/box").attrib["size"].split()
                    ],
                    "xyz": [float(v) for v in collision.find("origin").attrib["xyz"].split()],
                    "rpy": [float(v) for v in collision.find("origin").attrib["rpy"].split()],
                }
            )
        (temporary / "proxies.json").write_text(json.dumps(proxies))
        robot = sources["c_simulator/src/robot_class.cpp"].decode()
        equations = robot[
            robot.index("vXd Robot::stateDerivative(") : robot.index("void Robot::stopThrusters()")
        ]
        (temporary / "legacy_equations.inc").write_text(equations.replace("Robot::", "Reference::"))
        physics = sources["c_simulator/src/physics_simulator.cpp"].decode()
        start = physics.index("    vXd handleCollisions(vXd state)")
        end = physics.index("\n    /**", start)
        response = physics[start:end]
        task_hook = "        if (taskContacts)\n            state = taskContacts->resolve(state, robot.getInverseMass(), contactFriction);\n"
        if response.count(task_hook) != 1:
            raise RuntimeError("unexpected original task contact hook")
        response = response.replace(task_hook, "")
        start = physics.index("    collisionResult computeCollision(")
        end = physics.index("\n    /**", start)
        (temporary / "legacy_contacts.inc").write_text(response + "\n" + physics[start:end])
        start = physics.index("const vXd K1 = calcStateDot(")
        end = physics.index("vXd advanced = state + stateDelta;", start) + len(
            "vXd advanced = state + stateDelta;"
        )
        (temporary / "legacy_rk4.inc").write_text(physics[start:end])
        for optimization in ("-O2", "-O0"):
            flags = ["-std=c++17", optimization]
            subprocess.run(
                [
                    "c++",
                    *flags,
                    "-I/usr/include/eigen3",
                    f"-I{temporary / 'c_simulator/include'}",
                    f"-I{temporary}",
                    str(driver),
                    *(
                        str(temporary / f"c_simulator/src/{name}.cpp")
                        for name in ("marine_dynamics", "thruster_dynamics", "collisionBox_class")
                    ),
                    "-lyaml-cpp",
                    "-o",
                    str(temporary / "capture"),
                ],
                check=True,
            )
            output = subprocess.check_output([str(temporary / "capture"), str(temporary)])
            suffix = "_unoptimized" if optimization == "-O0" else ""
            destination = ROOT / f"tests/fixtures/legacy_talos{suffix}.csv"
            destination.write_bytes(output)
            metadata = {
                "revision": REVISION,
                "vehicle_revision": VEHICLE_REVISION,
                "sources_sha256": {
                    name: hashlib.sha256(data).hexdigest() for name, data in sources.items()
                },
                "vehicle_sha256": hashlib.sha256(vehicle).hexdigest(),
                "driver_sha256": hashlib.sha256(driver.read_bytes()).hexdigest(),
                "capture_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                "fixture_sha256": hashlib.sha256(output).hexdigest(),
                "compiler_flags": flags,
                "compiler": subprocess.check_output(["c++", "--version"], text=True).splitlines()[
                    0
                ],
                "scope": "Original Talos/pool values and numerical kernels with extracted stage forcing, RK4 and static contact response. Startup, partial immersion and floor approach, 1500 steps each, compared at every tick. Scripted forces. No devices, course props/tasks, ROS or renderer. Native importer/profile files are not inputs.",
            }
            destination.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")
            print(f"Captured {len(output.splitlines()) - 1} states: {destination.name}")


if __name__ == "__main__":
    main()

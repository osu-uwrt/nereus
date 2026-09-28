#!/usr/bin/env python3
"""Explicit offline capture from pinned legacy code; no runtime/build dependency."""

import argparse
import hashlib
import json
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REVISION = "07647eebe706f96ea7b76db3cc9802735a146698"
FILES = (
    "c_simulator/include/c_simulator/MarineDynamics.h",
    "c_simulator/include/c_simulator/ThrusterDynamics.h",
    "c_simulator/src/marine_dynamics.cpp",
    "c_simulator/src/thruster_dynamics.cpp",
    "c_simulator/src/robot_class.cpp",
    "c_simulator/src/physics_simulator.cpp",
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference_repo", type=Path)
    args = parser.parse_args()
    sources = {
        name: subprocess.check_output(
            ["git", "-C", str(args.reference_repo), "show", f"{REVISION}:{name}"]
        )
        for name in FILES
    }
    driver = ROOT / "tools/reference/stage_driver.cpp"
    with tempfile.TemporaryDirectory(prefix="legacy-stage-reference-") as directory:
        temporary = Path(directory)
        for name, data in sources.items():
            path = temporary / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
        robot = sources["c_simulator/src/robot_class.cpp"].decode()
        equations = robot[
            robot.index("vXd Robot::stateDerivative(") : robot.index("void Robot::stopThrusters()")
        ]
        (temporary / "legacy_equations.inc").write_text(equations.replace("Robot::", "Reference::"))
        physics = sources["c_simulator/src/physics_simulator.cpp"].decode()
        start = physics.index("const vXd K1 = calcStateDot(")
        end = physics.index("vXd advanced = state + stateDelta;", start) + len(
            "vXd advanced = state + stateDelta;"
        )
        (temporary / "legacy_rk4.inc").write_text(physics[start:end])
        subprocess.run(
            [
                "c++",
                "-std=c++17",
                "-O2",
                "-I/usr/include/eigen3",
                f"-I{temporary / 'c_simulator/include'}",
                f"-I{temporary}",
                str(driver),
                str(temporary / "c_simulator/src/marine_dynamics.cpp"),
                str(temporary / "c_simulator/src/thruster_dynamics.cpp"),
                "-o",
                str(temporary / "capture"),
            ],
            check=True,
        )
        output = subprocess.check_output([str(temporary / "capture")])
    destination = ROOT / "tests/fixtures/legacy_stage_dynamics.csv"
    destination.write_bytes(output)
    metadata = {
        "revision": REVISION,
        "sources_sha256": {
            name: hashlib.sha256(data).hexdigest() for name, data in sources.items()
        },
        "driver_sha256": hashlib.sha256(driver.read_bytes()).hexdigest(),
        "fixture_sha256": hashlib.sha256(output).hexdigest(),
        "compiler": subprocess.check_output(["c++", "--version"], text=True).splitlines()[0],
        "scope": "Noiseless free motion; exact legacy propulsion/current equations and RK4 stage schedule; excludes contacts, sensors, ROS and Talos configuration.",
    }
    destination.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(f"Captured {len(output.splitlines()) - 1} reference states to {destination}")


if __name__ == "__main__":
    main()

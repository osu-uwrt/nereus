#!/usr/bin/env python3
"""Capture the pinned original static-box collision response without ROS or task bodies."""

import argparse
import hashlib
import json
import subprocess
import tempfile
from pathlib import Path

from capture_stage_reference import REVISION, ROOT

FILES = (
    "c_simulator/include/c_simulator/MarineDynamics.h",
    "c_simulator/include/c_simulator/collisionBox.h",
    "c_simulator/include/c_simulator/settings.h",
    "c_simulator/src/marine_dynamics.cpp",
    "c_simulator/src/collisionBox_class.cpp",
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
    driver = ROOT / "tools/reference/contact_driver.cpp"
    with tempfile.TemporaryDirectory(prefix="legacy-contact-reference-") as directory:
        temporary = Path(directory)
        for name, data in sources.items():
            path = temporary / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
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
        outputs = {}
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
                    str(temporary / "c_simulator/src/marine_dynamics.cpp"),
                    str(temporary / "c_simulator/src/collisionBox_class.cpp"),
                    "-o",
                    str(temporary / "capture"),
                ],
                check=True,
            )
            if optimization == "-O2":
                outputs["legacy_box_contacts"] = (
                    subprocess.check_output([str(temporary / "capture")]),
                    flags,
                )
            name = "legacy_box_steps" + ("_unoptimized" if optimization == "-O0" else "")
            outputs[name] = (subprocess.check_output([str(temporary / "capture"), "steps"]), flags)
    for name, (output, flags) in outputs.items():
        destination = ROOT / "tests/fixtures" / (name + ".csv")
        destination.write_bytes(output)
        scope = (
            "Single static-box contact resolution."
            if name.endswith("contacts")
            else "50 RK4 steps per case, with pre/post static contact resolution; zero propulsion/current/volume."
        )
        destination.with_suffix(".json").write_text(
            json.dumps(
                {
                    "revision": REVISION,
                    "sources_sha256": {
                        name: hashlib.sha256(data).hexdigest() for name, data in sources.items()
                    },
                    "driver_sha256": hashlib.sha256(driver.read_bytes()).hexdigest(),
                    "fixture_sha256": hashlib.sha256(output).hexdigest(),
                    "compiler": subprocess.check_output(
                        ["c++", "--version"], text=True
                    ).splitlines()[0],
                    "compiler_flags": flags,
                    "scope": scope
                    + " Original SAT/contact point/impulse/friction. Logging disabled; optional task-contact hook omitted. No Talos content, sensors or ROS.",
                },
                indent=2,
            )
            + "\n"
        )
        print(f"Captured {len(output.splitlines()) - 1} rows to {destination.name}")


if __name__ == "__main__":
    main()

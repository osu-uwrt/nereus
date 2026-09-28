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
                str(temporary / "c_simulator/src/collisionBox_class.cpp"),
                "-o",
                str(temporary / "capture"),
            ],
            check=True,
        )
        output = subprocess.check_output([str(temporary / "capture")])
    destination = ROOT / "tests/fixtures/legacy_box_contacts.csv"
    destination.write_bytes(output)
    destination.with_suffix(".json").write_text(
        json.dumps(
            {
                "revision": REVISION,
                "sources_sha256": {
                    name: hashlib.sha256(data).hexdigest() for name, data in sources.items()
                },
                "driver_sha256": hashlib.sha256(driver.read_bytes()).hexdigest(),
                "fixture_sha256": hashlib.sha256(output).hexdigest(),
                "compiler": subprocess.check_output(["c++", "--version"], text=True).splitlines()[
                    0
                ],
                "scope": "Single static-box contact resolution at supplied states; original SAT/contact point/impulse/friction. Logging disabled; optional task-contact hook omitted. No integration, Talos content, sensors or ROS.",
            },
            indent=2,
        )
        + "\n"
    )
    print(f"Captured {len(output.splitlines()) - 1} static-box contact cases")


if __name__ == "__main__":
    main()

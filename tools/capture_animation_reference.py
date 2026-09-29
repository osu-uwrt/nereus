#!/usr/bin/env python3
"""Capture pinned original rotor/indicator behavior without a renderer or ROS host."""

import argparse
import csv
import hashlib
import io
import json
import platform
import subprocess
import tarfile
import tempfile
from pathlib import Path

import yaml
from capture_stage_reference import REVISION, ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("simulator_repo", type=Path)
    args = parser.parse_args()
    paths = [
        "camera_faker/include/pool_viewer/camera.hpp",
        "camera_faker/include/pool_viewer/thruster_visuals.hpp",
        "camera_faker/include/pool_viewer/status_lights.hpp",
        "camera_faker/include/external/glm",
        "camera_faker/models/talos3/thrusters.yaml",
        "camera_faker/models/talos3/rotors",
        "c_simulator/robots/talos/config/status_lights.yaml",
    ]
    archive = subprocess.check_output(
        ["git", "-C", str(args.simulator_repo), "archive", REVISION, *paths]
    )
    driver = ROOT / "tools/reference/animation_driver.cpp"
    prefix = ROOT / "tests/fixtures/animation_reference"
    with tempfile.TemporaryDirectory(prefix="original-animation-") as directory:
        work = Path(directory)
        # Pinned trusted Git archive; never unpack caller-provided archive contents.
        with tarfile.open(fileobj=io.BytesIO(archive)) as bundle:
            bundle.extractall(work)
        flags = subprocess.check_output(
            ["pkg-config", "--cflags", "--libs", "opencv4", "yaml-cpp"], text=True
        ).split()
        subprocess.run(
            ["c++", "-std=c++17", "-O2", str(driver),
             "-I" + str(work / "camera_faker/include"),
             "-I" + str(work / "camera_faker/include/external"),
             *flags, "-o", str(work / "capture")], check=True,
        )
        rotor_path = work / "camera_faker/models/talos3/thrusters.yaml"
        light_path = work / "c_simulator/robots/talos/config/status_lights.yaml"
        subprocess.run([work / "capture", rotor_path, light_path, prefix], check=True)
        rotors = yaml.safe_load(rotor_path.read_text())
        lights = yaml.safe_load(light_path.read_text())
        with Path(str(prefix) + "_config.csv").open("w") as output:
            writer = csv.writer(output, lineterminator="\n")
            writer.writerow([rotors["timeout"], rotors["force_deadband"], rotors["speed_scale"],
                             lights["flash_duration"], *rotors["force_to_rpm"]["forward"],
                             *rotors["force_to_rpm"]["reverse"]])
            for rotor in rotors["rotors"]:
                writer.writerow([rotor["input_index"], rotor["direction"], *rotor["pivot"], *rotor["axis"]])
    outputs = sorted(prefix.parent.glob(prefix.name + "_*"))
    manifest = {
        "source_revision": REVISION,
        "compiler": subprocess.check_output(["c++", "--version"], text=True).splitlines()[0],
        "compile_options": ["-std=c++17", "-O2"],
        "architecture": platform.machine(),
        "source_paths": paths,
        "archive_sha256": hashlib.sha256(archive).hexdigest(),
        "driver_sha256": hashlib.sha256(driver.read_bytes()).hexdigest(),
        "capture_script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "scope": "Original unchanged rotor integration/matrices and clamped indicator colors. No ROS routing, renderer, live-source delivery or LED geometry acceptance.",
        "outputs_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in outputs},
    }
    (ROOT / "docs/reference/animation_sources.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print("Captured original rotor and indicator state references")


if __name__ == "__main__":
    main()

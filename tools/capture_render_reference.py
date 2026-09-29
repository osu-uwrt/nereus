#!/usr/bin/env python3
"""Run the pinned original renderer at fixed inputs on the current GL backend; offline development tool."""

import argparse
import hashlib
import io
import json
import os
import shlex
import subprocess
import tarfile
import tempfile
from pathlib import Path

from capture_stage_reference import REVISION, ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("simulator_repo", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--optimization", choices=("0", "2"), default="2")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    inputs = [
        "camera_faker/include/pool_viewer",
        "camera_faker/include/external",
        "camera_faker/src/pool_viewer/renderer.cpp",
        "camera_faker/shaders/pool",
        "camera_faker/models/talos3/Talos3_body.glb",
        "camera_faker/models/talos3/rotors",
        "camera_faker/textures/objects/April Tag.jpg",
        "c_simulator/robots/talos/config/status_lights.yaml",
    ]
    archive = subprocess.check_output(
        ["git", "-C", str(args.simulator_repo), "archive", REVISION, *inputs]
    )
    sources = {}
    driver = ROOT / "tools/reference/render_driver.cpp"
    with tempfile.TemporaryDirectory(prefix="original-render-") as directory:
        temp = Path(directory)
        with tarfile.open(fileobj=io.BytesIO(archive)) as bundle:
            # Only the explicitly pinned trusted source revision is extracted.
            for entry in bundle.getmembers():
                if entry.isfile():
                    sources[entry.name] = hashlib.sha256(
                        bundle.extractfile(entry).read()
                    ).hexdigest()
            bundle.extractall(temp)
        (temp / "mapping.yaml").write_text(
            '"/**/zed_faker":\n  ros__parameters:\n    config_frame: tag\n    map_origin_pool: [19.5136, 0, 90]\n"/fixture/riptide_mapping2":\n  ros__parameters:\n    init_data: {}\n'
        )
        (temp / "markers.yaml").write_text(
            '"/**/marker_publisher":\n  ros__parameters:\n    markers: {}\n'
        )
        (temp / "scene.yaml").write_text(
            "world: {length: 50, width: 22.86, depth: 2.1336, water_level: 0, deck_height: 0.305288888}\nrobot: {model: {riptide_mesh: fixture}}\nentities: []\n"
        )
        include = temp / "camera_faker/include"
        subprocess.run(
            [
                "cc",
                "-I",
                str(include / "external"),
                "-c",
                str(include / "external/glad/glad.c"),
                "-o",
                str(temp / "glad.o"),
            ],
            check=True,
        )
        flags = shlex.split(
            subprocess.check_output(
                ["pkg-config", "--cflags", "--libs", "opencv4", "glfw3", "assimp", "yaml-cpp"],
                text=True,
            )
        )
        subprocess.run(
            [
                "c++",
                "-std=c++17",
                "-O" + args.optimization,
                "-I",
                str(include),
                "-I",
                str(include / "external"),
                "-I",
                str(ROOT / "applications/render_capture"),
                str(driver),
                str(temp / "camera_faker/src/pool_viewer/renderer.cpp"),
                str(temp / "glad.o"),
                *flags,
                "-ldl",
                "-o",
                str(temp / "capture"),
            ],
            check=True,
        )
        info = subprocess.check_output([str(temp / "capture"), str(temp), str(output)], text=True)
    metadata = {
        "source_revision": REVISION,
        "optimization": "-O" + args.optimization,
        "sources_sha256": sources,
        "capture_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "benchmark_sha256": hashlib.sha256(
            (ROOT / "applications/render_capture/benchmark.hpp").read_bytes()
        ).hexdigest(),
        "performance": json.loads((output / "benchmark.json").read_text()),
        "driver_sha256": hashlib.sha256(driver.read_bytes()).hexdigest(),
        "backend": info.strip(),
        "display": os.environ.get("DISPLAY"),
        "dimensions": [640, 400],
        "time_seconds": 12.5,
        "cases": list(range(9)),
        "scope": "Six fixed body/rotor/pool views plus three original indicator closeups (off, red, individual RGB). No animated rotors, task mechanisms, overlays, ROS or mission parity.",
        "outputs_sha256": {
            p.name: hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(output.iterdir())
            if p.suffix in (".view", ".rgba", ".opaque", ".depth", ".composite", ".composite-depth", ".lights")
        },
    }
    (output / "manifest.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(info, end="")
    print(f"Captured fixed original frames in {output}")


if __name__ == "__main__":
    main()

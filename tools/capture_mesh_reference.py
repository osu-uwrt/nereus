#!/usr/bin/env python3
"""Capture pinned original CPU mesh loading without ROS, OpenGL or platform code."""

import argparse
import hashlib
import io
import json
import subprocess
import tarfile
import tempfile
from pathlib import Path

from capture_stage_reference import REVISION, ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("simulator_repo", type=Path)
    args = parser.parse_args()
    sources = {}

    def read(name):
        data = subprocess.check_output(
            ["git", "-C", str(args.simulator_repo), "show", f"{REVISION}:{name}"]
        )
        sources[name] = hashlib.sha256(data).hexdigest()
        return data

    source = read("camera_faker/src/pool_viewer/renderer.cpp").decode()
    read("camera_faker/include/pool_viewer/renderer.hpp")
    matrix = source[
        source.index("glm::mat4 aiMatrix(") : source.index("std::shared_ptr<Mesh> cube()")
    ]
    loader = source[
        source.index("std::vector<std::shared_ptr<Mesh>> Renderer::load(") : source.index(
            "void Renderer::box("
        )
    ]
    driver = ROOT / "tools/reference/mesh_driver.cpp"
    with tempfile.TemporaryDirectory(prefix="legacy-mesh-reference-") as directory:
        temp = Path(directory)
        # Trusted, pinned repository objects only, including the original GLM implementation.
        glm_path = "camera_faker/include/external/glm"
        archive = subprocess.check_output(
            ["git", "-C", str(args.simulator_repo), "archive", REVISION, glm_path]
        )
        sources[glm_path + " (git archive)"] = hashlib.sha256(archive).hexdigest()
        with tarfile.open(fileobj=io.BytesIO(archive)) as bundle:
            bundle.extractall(temp)
        (temp / "mesh_loader.inc").write_text(matrix + loader)
        files = ["Talos3_body.glb"] + [
            f"rotors/{name}.glb"
            for name in ("VUS", "VUP", "HUS", "HUP", "HLS", "HLP", "VLS", "VLP")
        ]
        inputs = []
        for name in files:
            path = temp / Path(name).name
            path.write_bytes(read("camera_faker/models/talos3/" + name))
            inputs.append(str(path))
        subprocess.run(
            [
                "c++",
                "-std=c++17",
                "-O2",
                "-I",
                str(temp),
                "-I",
                str(temp / "camera_faker/include/external"),
                str(driver),
                "-lassimp",
                "-o",
                str(temp / "capture"),
            ],
            check=True,
        )
        output = subprocess.check_output([str(temp / "capture"), *inputs])
    fixture = ROOT / "tests/fixtures/legacy_mesh_assets.csv"
    fixture.write_bytes(output)
    metadata = {
        "scope": "Original unmodified CPU load method; texture-free Talos body and eight rotors. Counts, order, index hashes, color/alpha, bounds and vertex means. No GL or appearance parity claim.",
        "source_revision": REVISION,
        "sources_sha256": sources,
        "driver_sha256": hashlib.sha256(driver.read_bytes()).hexdigest(),
        "capture_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "output_sha256": hashlib.sha256(output).hexdigest(),
        "compiler": subprocess.check_output(["c++", "--version"], text=True).splitlines()[0],
        "assimp_version": subprocess.check_output(
            ["pkg-config", "--modversion", "assimp"], text=True
        ).strip(),
        "flags": ["-std=c++17", "-O2"],
    }
    fixture.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(f"Captured {len(output.splitlines()) - 1} original submeshes")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Verify a relocated CPU asset install with no graphics, simulation or original workspace."""

import argparse
import hashlib
import json
import shutil
import sys
import tempfile
from pathlib import Path

from check import ROOT, clean_environment, run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preset", choices=("assets", "assets-asan"), default="assets")
    args = parser.parse_args()
    manifest = json.loads((ROOT / "content/visuals/talos/provenance/manifest.json").read_text())
    for name, expected in manifest["outputs_sha256"].items():
        if hashlib.sha256((ROOT / name).read_bytes()).hexdigest() != expected:
            raise ValueError(f"visual resource hash mismatch: {name}")
    run([sys.executable, ROOT / "tools/check.py", "--preset", args.preset])
    cmake = shutil.which("cmake")
    env = clean_environment()
    with tempfile.TemporaryDirectory(prefix="asset-install-") as directory:
        temp = Path(directory)
        source = temp / "source"
        source.mkdir()
        for folder in (
            "cmake",
            "libraries/spatial",
            "libraries/rendering",
            "content/visuals",
            "docs",
        ):
            shutil.copytree(ROOT / folder, source / folder)
        for file in ("CMakeLists.txt", "LICENSE", "NOTICE"):
            shutil.copy2(ROOT / file, source / file)
        flags = [
            "-DRP_BUILD_SIMULATION=OFF",
            "-DRP_BUILD_CLI=OFF",
            "-DRP_BUILD_MESH_ASSETS=ON",
            "-DRP_INSTALL_REFERENCE_VISUALS=ON",
            "-DBUILD_TESTING=OFF",
            "-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF",
            "-DCMAKE_BUILD_TYPE=Release",
        ]
        if args.preset == "assets-asan":
            flags += ["-DCMAKE_BUILD_TYPE=Debug", "-DRP_ENABLE_SANITIZERS=ON"]
        run([cmake, "-S", source, "-B", temp / "build", *flags], env=env)
        run([cmake, "--build", temp / "build", "--parallel", "2"], env=env)
        run([cmake, "--install", temp / "build", "--prefix", temp / "staging"], env=env)
        installed = temp / "relocated"
        (temp / "staging").rename(installed)
        shutil.rmtree(source)
        shutil.rmtree(temp / "build")
        for path in installed.rglob("*.cmake"):
            data = path.read_text()
            forbidden = [str(source), str(ROOT)]
            if path.name.startswith("NereusTargets"):
                forbidden += ["OpenGL", "GLEW", "glfw", "nereus_simulation", "yaml-cpp"]
            if any(token in data for token in forbidden):
                raise RuntimeError(f"unwanted asset dependency: {path}")
        shutil.copytree(ROOT / "examples/mesh_assets", temp / "consumer")
        run(
            [
                cmake,
                "-S",
                temp / "consumer",
                "-B",
                temp / "consumer-build",
                f"-DCMAKE_PREFIX_PATH={installed}",
                "-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF",
            ],
            env=env,
        )
        run([cmake, "--build", temp / "consumer-build", "--parallel", "2"], env=env)
        run(
            [
                temp / "consumer-build/mesh_asset_demo",
                installed / "share/nereus/visuals/talos",
            ],
            cwd=temp,
            env=env,
        )
    print("Pinned resources and relocated CPU asset consumer passed.")


if __name__ == "__main__":
    main()

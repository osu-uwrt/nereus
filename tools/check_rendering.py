#!/usr/bin/env python3
"""Validate context-owned rendering and a relocated install without viewer/simulation sources."""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from check import ROOT, clean_environment, run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preset", choices=("rendering", "rendering-asan"), default="rendering")
    args = parser.parse_args()
    run([sys.executable, ROOT / "tools/check.py", "--preset", args.preset])
    cmake = shutil.which("cmake")
    env = clean_environment()
    graphics = dict(env)
    for key in (
        "DISPLAY",
        "XAUTHORITY",
        "WAYLAND_DISPLAY",
        "XDG_RUNTIME_DIR",
        "LIBGL_ALWAYS_SOFTWARE",
    ):
        if key in os.environ:
            graphics[key] = os.environ[key]
    if args.preset == "rendering-asan":
        graphics["ASAN_OPTIONS"] = "detect_leaks=0"
        graphics["UBSAN_OPTIONS"] = "halt_on_error=1:print_stacktrace=1"
    with tempfile.TemporaryDirectory(prefix="render-install-") as directory:
        temp = Path(directory)
        source = temp / "source"
        source.mkdir()
        for folder in (
            "cmake",
            "libraries/spatial",
            "libraries/rendering",
            "applications/render_capture",
            "third_party/glm",
            "content/visuals",
            "docs",
        ):
            shutil.copytree(ROOT / folder, source / folder)
        for file in ("CMakeLists.txt", "LICENSE.md"):
            shutil.copy2(ROOT / file, source / file)
        flags = [
            "-DRP_BUILD_SIMULATION=OFF",
            "-DRP_BUILD_CLI=OFF",
            "-DRP_BUILD_SCENE_RENDERER=ON",
            "-DRP_BUILD_RENDER_CAPTURE=ON",
            "-DRP_INSTALL_REFERENCE_VISUALS=ON",
            "-DBUILD_TESTING=OFF",
            "-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF",
            "-DCMAKE_BUILD_TYPE=Release",
        ]
        if args.preset == "rendering-asan":
            flags += ["-DCMAKE_BUILD_TYPE=Debug", "-DRP_ENABLE_SANITIZERS=ON"]
        run([cmake, "-S", source, "-B", temp / "build", *flags], env=env)
        run([cmake, "--build", temp / "build", "--parallel", "2"], env=env)
        run([cmake, "--install", temp / "build", "--prefix", temp / "staging"], env=env)
        installed = temp / "relocated"
        (temp / "staging").rename(installed)
        shutil.rmtree(source)
        shutil.rmtree(temp / "build")
        for path in installed.rglob("*.cmake"):
            text = path.read_text()
            forbidden = [str(source), str(ROOT)]
            if path.name.startswith("RoboticsPlatformTargets"):
                forbidden += [
                    "glfw",
                    "rp_simulation",
                    "rp_config",
                    "yaml-cpp",
                    "rp_visualization",
                    "imgui",
                ]
            if any(token in text for token in forbidden):
                raise RuntimeError(f"unwanted renderer dependency: {path}")
        shutil.copytree(ROOT / "examples/rendering", temp / "consumer")
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
        run([temp / "consumer-build/scene_demo"], cwd=temp, env=env)
        capture = ROOT / "build" / args.preset / "captures"
        data = installed / "share/robotics_platform"
        command = [
            installed / "bin/robotics-render-capture",
            data / "shaders",
            data / "visuals/talos",
            capture,
        ]
        info = subprocess.check_output(list(map(str, command)), cwd=temp, env=graphics, text=True)
        print(info, end="")
    print("Renderer contracts and isolated installed captures passed.")


if __name__ == "__main__":
    main()

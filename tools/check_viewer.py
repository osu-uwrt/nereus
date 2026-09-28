#!/usr/bin/env python3
"""Check viewer contracts and a relocated install built without simulation sources."""

import argparse
import os
import shutil
import sys
import tempfile
from pathlib import Path

from check import ROOT, clean_environment, run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preset", choices=("viewer", "viewer-asan"), default="viewer")
    parser.add_argument("--tidy", action="store_true")
    parser.add_argument(
        "--graphics", action="store_true", help="also exercise installed OpenGL UI on DISPLAY"
    )
    args = parser.parse_args()
    command = [sys.executable, ROOT / "tools/check.py", "--preset", args.preset]
    if args.tidy:
        command.append("--tidy")
    run(command)
    cmake = shutil.which("cmake")
    env = clean_environment()
    with tempfile.TemporaryDirectory(prefix="viewer-install-") as directory:
        temp = Path(directory)
        source = temp / "source"
        source.mkdir()
        # Deliberately omit plant, sensors, configuration, Python, and robot/world content.
        for folder in (
            "cmake",
            "libraries/visualization",
            "libraries/viewer_io",
            "libraries/rendering",
            "applications/viewer",
            "third_party",
            "workspaces",
            "docs",
        ):
            shutil.copytree(ROOT / folder, source / folder)
        for file in ("CMakeLists.txt", "LICENSE.md"):
            shutil.copy2(ROOT / file, source / file)
        build = temp / "build"
        flags = [
            "-DRP_BUILD_SIMULATION=OFF",
            "-DRP_BUILD_CLI=OFF",
            "-DRP_BUILD_VIEWER=ON",
            "-DBUILD_TESTING=OFF",
            "-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF",
        ]
        if args.preset == "viewer-asan":
            flags += ["-DRP_ENABLE_SANITIZERS=ON", "-DCMAKE_BUILD_TYPE=Debug"]
        else:
            flags += ["-DCMAKE_BUILD_TYPE=Release"]
        run([cmake, "-S", source, "-B", build, *flags], env=env)
        run([cmake, "--build", build, "--parallel", "2"], env=env)
        run([cmake, "--install", build, "--prefix", temp / "staging"], env=env)
        installed = temp / "relocated"
        (temp / "staging").rename(installed)
        shutil.rmtree(source)
        shutil.rmtree(build)
        for file in installed.rglob("*.cmake"):
            text = file.read_text()
            if str(source) in text or str(ROOT) in text or "rp_simulation" in text:
                raise RuntimeError(f"Unwanted dependency in {file}")
        example = temp / "extension"
        shutil.copytree(ROOT / "examples/viewer", example)
        run(
            [
                cmake,
                "-S",
                example,
                "-B",
                temp / "consumer",
                f"-DCMAKE_PREFIX_PATH={installed}",
                "-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF",
            ],
            env=env,
        )
        run([cmake, "--build", temp / "consumer", "--parallel", "2"], env=env)
        run([temp / "consumer/viewer_extension"], cwd=temp, env=env)
        executable = installed / "bin/robotics-viewer"
        run([executable, "--help"], cwd=temp, env=env)
        if args.graphics:
            graphics_env = dict(env)
            for key in (
                "DISPLAY",
                "XAUTHORITY",
                "WAYLAND_DISPLAY",
                "XDG_RUNTIME_DIR",
                "LIBGL_ALWAYS_SOFTWARE",
            ):
                if key in os.environ:
                    graphics_env[key] = os.environ[key]
            if args.preset == "viewer-asan":
                # Mesa/desktop driver process globals are outside the app's ownership.
                graphics_env["ASAN_OPTIONS"] = "detect_leaks=0"
                graphics_env["UBSAN_OPTIONS"] = "halt_on_error=1:print_stacktrace=1"
            fixture = installed / "share/robotics_platform/workspaces/local_demo.yaml"
            for name, arguments in (
                ("empty", []),
                ("start", [fixture]),
                ("motion", [fixture, "--time-ns", "6000000000"]),
            ):
                capture = ROOT / "build" / args.preset / f"{name}.ppm"
                run(
                    [executable, *arguments, "--hidden", "--frames", "3", "--screenshot", capture],
                    cwd=temp,
                    env=graphics_env,
                )
                data = capture.read_bytes()
                if not data.startswith(b"P6\n") or len(data) < 10000:
                    raise RuntimeError("Missing framebuffer capture")
            first = (ROOT / "build" / args.preset / "start.ppm").read_bytes()
            second = (ROOT / "build" / args.preset / "motion.ppm").read_bytes()
            if first == second:
                raise RuntimeError("Playback did not change the captured view")
    print("Viewer contracts and isolated installed extension passed.")


if __name__ == "__main__":
    main()

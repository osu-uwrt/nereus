#!/usr/bin/env python3
"""Run the contributor checks without inheriting a sourced ROS workspace."""

import argparse
import ast
import json
import os
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def run(args, *, cwd=ROOT, env=None):
    """Echo a command shell-style, then run it; a non-zero exit raises."""
    print("+ " + " ".join(map(str, args)), flush=True)
    subprocess.run(list(map(str, args)), cwd=cwd, env=env, check=True)


def clean_environment():
    """A minimal environment: system PATH only, so no sourced ROS or Python overlay leaks in."""
    return {
        "HOME": os.environ["HOME"],
        "PATH": "/usr/local/bin:/usr/bin:/bin",
        "LANG": "C.UTF-8",
        "LC_ALL": "C.UTF-8",
    }


def main():
    """Format check, dependency-boundary checks, configure/build/test, then optional clang-tidy."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preset", choices=("release", "dev", "asan"), default="release")
    parser.add_argument("--tidy", action="store_true")
    args = parser.parse_args()
    env = clean_environment()
    # Resolve developer tools before clearing PATH; execute with no ROS/Python overlays.
    cmake = shutil.which("cmake")
    ctest = shutil.which("ctest")
    formatter = shutil.which("clang-format")
    if not all((cmake, ctest, formatter)):
        parser.error("cmake, ctest, and clang-format are required (see CONTRIBUTING.md)")

    # Every C++ source of the core libraries and extensions must be clang-format clean.
    sources = sorted(
        p
        for folder in ("libraries", "extensions")
        for p in (ROOT / folder).rglob("*")
        if p.suffix in (".cpp", ".hpp")
    )
    run([formatter, "--dry-run", "--Werror", *sources], env=env)

    # The core never includes ROS, the UWRT stack, GLFW (outside tests) or yaml-cpp.
    for path in sources:
        for line in path.read_text().splitlines():
            if not line.startswith("#include"):
                continue
            for token in ("rclcpp", "rclpy", "tf2", "riptide", "GLFW", "yaml-cpp"):
                if token not in line:
                    continue
                if token == "GLFW" and "/tests/" in path.as_posix():
                    continue  # tests may open a window for a GL context
                raise RuntimeError(f"Forbidden dependency in {path}: {line}")

    # Layering: spatial and rendering sit below simulation/sensors; spatial has no graphics.
    for path in sources:
        relative = path.relative_to(ROOT).as_posix()
        if relative.startswith(("libraries/spatial/", "libraries/rendering/")):
            for line in path.read_text().splitlines():
                if line.startswith("#include") and any(
                    token in line for token in ("nereus/simulation", "nereus/sensors", "pybind11")
                ):
                    raise RuntimeError(f"Rendering depends on simulation: {path}: {line}")
        if relative.startswith("libraries/spatial/"):
            for line in path.read_text().splitlines():
                if line.startswith("#include") and any(
                    token in line for token in ("GL/", "GLFW/", "imgui")
                ):
                    raise RuntimeError(f"Spatial library depends on graphics: {path}: {line}")

    # Every Python file must at least parse.
    for folder in ("tools", "tests", "python"):
        for path in (ROOT / folder).rglob("*.py"):
            ast.parse(path.read_text(), filename=str(path))

    # Configure, build and test the chosen preset in the clean environment.
    run([cmake, "--preset", args.preset, "-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF"], env=env)
    run([cmake, "--build", "--preset", args.preset], env=env)
    run([ctest, "--preset", args.preset], env=env)

    # clang-tidy on the compiled .cpp files under src/ and applications/ only.
    if args.tidy:
        tidy = shutil.which("clang-tidy")
        if not tidy:
            parser.error("--tidy requires clang-tidy")
        database = json.loads((ROOT / "build" / args.preset / "compile_commands.json").read_text())
        compiled = {Path(entry["file"]).resolve() for entry in database}
        for path in sources:
            if path.resolve() not in compiled:
                continue
            if path.suffix == ".cpp" and (
                "/src/" in path.as_posix() or "/applications/" in path.as_posix()
            ):
                run([tidy, path, "-p", ROOT / "build" / args.preset], env=env)
    print("All requested checks passed.")


if __name__ == "__main__":
    main()

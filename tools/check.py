#!/usr/bin/env python3
"""Run the contributor checks without inheriting a sourced ROS workspace."""
import argparse
import ast
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def run(args, *, cwd=ROOT, env=None):
    print("+ " + " ".join(map(str, args)), flush=True)
    subprocess.run(list(map(str, args)), cwd=cwd, env=env, check=True)


def clean_environment():
    return {
        "HOME": os.environ["HOME"],
        "PATH": "/usr/local/bin:/usr/bin:/bin",
        "LANG": "C.UTF-8",
        "LC_ALL": "C.UTF-8",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preset", choices=("scene-viewer", "scene-viewer-asan", "rendering", "rendering-asan", "assets", "assets-asan", "release", "dev", "asan", "viewer", "viewer-asan", "simulation-view", "simulation-view-asan", "simulator-viewer", "simulator-viewer-asan"), default="release")
    parser.add_argument("--install-check", action="store_true")
    parser.add_argument("--tidy", action="store_true")
    args = parser.parse_args()
    env = clean_environment()
    # Resolve developer tools before clearing PATH; execute with no ROS/Python overlays.
    cmake = shutil.which("cmake")
    ctest = shutil.which("ctest")
    formatter = shutil.which("clang-format")
    if not all((cmake, ctest, formatter)):
        parser.error("cmake, ctest, and clang-format are required (see CONTRIBUTING.md)")
    sources = sorted(p for folder in ("libraries", "applications", "bindings", "integrations/simulation_view", "examples", "tests")
                     for p in (ROOT / folder).rglob("*") if p.suffix in (".cpp", ".hpp"))
    run([formatter, "--dry-run", "--Werror", *sources], env=env)
    for path in sources:
        for line in path.read_text().splitlines():
            if not line.startswith("#include"):
                continue
            for token in ("rclcpp", "rclpy", "tf2", "riptide", "c_simulator", "GLFW", "yaml-cpp"):
                if token not in line:
                    continue
                if token == "yaml-cpp" and any(part in path.as_posix() for part in ("config/src", "viewer_io/src", "scene_view/src")):
                    continue
                if token == "GLFW" and any(part in path.as_posix() for part in ("applications/viewer", "applications/render_capture")):
                    continue
                raise RuntimeError(f"Forbidden dependency in {path}: {line}")
    for path in sources:
        relative = path.relative_to(ROOT).as_posix()
        if relative.startswith(("libraries/spatial/", "libraries/visualization/", "libraries/viewer_io/", "libraries/rendering/", "applications/viewer/")):
            for line in path.read_text().splitlines():
                if line.startswith("#include") and any(token in line for token in ("robotics/simulation", "robotics/sensors", "robotics/config", "pybind11")):
                    raise RuntimeError(f"Viewer depends on simulation: {path}: {line}")
        if relative.startswith(("libraries/spatial/", "libraries/visualization/", "libraries/viewer_io/")):
            for line in path.read_text().splitlines():
                if line.startswith("#include") and any(token in line for token in ("GL/", "GLFW/", "imgui")):
                    raise RuntimeError(f"Neutral viewer library depends on graphics: {path}: {line}")
    for folder in ("tools", "tests", "python", "examples/python"):
        for path in (ROOT / folder).rglob("*.py"):
            ast.parse(path.read_text(), filename=str(path))
    run([cmake, "--preset", args.preset, "-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF"], env=env)
    run([cmake, "--build", "--preset", args.preset], env=env)
    run([ctest, "--preset", args.preset], env=env)
    if args.tidy:
        tidy = shutil.which("clang-tidy")
        if not tidy:
            parser.error("--tidy requires clang-tidy")
        database = json.loads((ROOT / "build" / args.preset / "compile_commands.json").read_text())
        compiled = {Path(entry["file"]).resolve() for entry in database}
        for path in sources:
            if path.resolve() not in compiled:
                continue
            if path.suffix == ".cpp" and ("/src/" in path.as_posix() or "/applications/" in path.as_posix()):
                run([tidy, path, "-p", ROOT / "build" / args.preset], env=env)
    if args.install_check and args.preset.startswith(("viewer", "scene-viewer", "assets", "rendering")):
        parser.error("Use tools/check_viewer.py, tools/check_assets.py or tools/check_rendering.py for these installation checks")
    if args.install_check:
        # Relocate the installed prefix and copy the downstream example. The exported
        # build graph must refer only to installed artifacts and system dependencies.
        with tempfile.TemporaryDirectory(prefix="robotics-install-") as directory:
            temp = Path(directory)
            prefix = temp / "staging"
            run([cmake, "--install", ROOT / "build" / args.preset, "--prefix", prefix], env=env)
            installed = temp / "relocated"
            prefix.rename(installed)
            for target_file in installed.rglob("*.cmake"):
                if str(ROOT) in target_file.read_text():
                    raise RuntimeError(f"Source-tree path leaked into {target_file}")
            consumer = temp / "consumer"
            shutil.copytree(ROOT / "examples/cpp_consumer", consumer)
            run([cmake, "-S", consumer, "-B", temp / "build",
                 f"-DCMAKE_PREFIX_PATH={installed}", "-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF"], env=env)
            run([cmake, "--build", temp / "build", "--parallel", "2"], env=env)
            run([temp / "build/consumer"], cwd=temp, env=env)
            sensor_consumer = temp / "sensors"
            shutil.copytree(ROOT / "examples/sensors", sensor_consumer)
            run([cmake, "-S", sensor_consumer, "-B", temp / "sensor-build",
                 f"-DCMAKE_PREFIX_PATH={installed}", "-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF"], env=env)
            run([cmake, "--build", temp / "sensor-build", "--parallel", "2"], env=env)
            with (temp / "sensors.csv").open("w") as output:
                subprocess.run([str(temp / "sensor-build/sensor_demo")], cwd=temp,
                               env=env, stdout=output, check=True)
            with (temp / "trajectory.csv").open("w") as output:
                subprocess.run([str(installed / "bin/robotics-sim"),
                                str(installed / "share/robotics_platform/examples/empty_pool.yaml")],
                               cwd=temp, env=env, stdout=output, check=True)
            profile_consumer = temp / "profiles"
            shutil.copytree(ROOT / "examples/profiles", profile_consumer)
            run([cmake, "-S", profile_consumer, "-B", temp / "profile-build",
                 f"-DCMAKE_PREFIX_PATH={installed}", "-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF"], env=env)
            run([cmake, "--build", temp / "profile-build", "--parallel", "2"], env=env)
            run([temp / "profile-build/profile_demo",
                 installed / "share/robotics_platform/examples/profile_pool.yaml"], cwd=temp, env=env)
            with (temp / "profile-trajectory.csv").open("w") as output:
                subprocess.run([str(installed / "bin/robotics-sim"),
                                str(installed / "share/robotics_platform/examples/profile_pool.yaml"),
                                "--sensors", str(temp / "profile-sensors.csv")],
                               cwd=temp, env=env, stdout=output, check=True)
            if args.preset.startswith("simulation-view"):
                adapter_consumer = temp / "simulation-view"
                shutil.copytree(ROOT / "examples/simulation_view", adapter_consumer)
                run([cmake, "-S", adapter_consumer, "-B", temp / "adapter-build",
                     f"-DCMAKE_PREFIX_PATH={installed}", "-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF"], env=env)
                run([cmake, "--build", temp / "adapter-build", "--parallel", "2"], env=env)
                run([temp / "adapter-build/simulation_view_demo"], cwd=temp, env=env)
    print("All requested checks passed.")


if __name__ == "__main__":
    main()

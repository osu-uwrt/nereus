#!/usr/bin/env python3
"""Run the contributor checks without inheriting a sourced ROS workspace."""
import argparse
import ast
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
    parser.add_argument("--preset", choices=("release", "dev", "asan"), default="release")
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
    sources = sorted(p for folder in ("libraries", "applications", "examples", "tests")
                     for p in (ROOT / folder).rglob("*") if p.suffix in (".cpp", ".hpp"))
    run([formatter, "--dry-run", "--Werror", *sources], env=env)
    for path in sources:
        for line in path.read_text().splitlines():
            if not line.startswith("#include"):
                continue
            for token in ("rclcpp", "rclpy", "tf2", "riptide", "c_simulator", "GLFW", "yaml-cpp"):
                if token not in line:
                    continue
                if token == "yaml-cpp" and "config/src" in path.as_posix():
                    continue
                raise RuntimeError(f"Forbidden dependency in {path}: {line}")
    for folder in ("tools", "tests"):
        for path in (ROOT / folder).glob("*.py"):
            ast.parse(path.read_text(), filename=str(path))
    run([cmake, "--preset", args.preset, "-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF"], env=env)
    run([cmake, "--build", "--preset", args.preset], env=env)
    run([ctest, "--preset", args.preset], env=env)
    if args.tidy:
        tidy = shutil.which("clang-tidy")
        if not tidy:
            parser.error("--tidy requires clang-tidy")
        for path in sources:
            if path.suffix == ".cpp" and ("/src/" in path.as_posix() or "/applications/" in path.as_posix()):
                run([tidy, path, "-p", ROOT / "build" / args.preset], env=env)
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
            with (temp / "trajectory.csv").open("w") as output:
                subprocess.run([str(installed / "bin/robotics-sim"),
                                str(installed / "share/robotics_platform/examples/empty_pool.yaml")],
                               cwd=temp, env=env, stdout=output, check=True)
    print("All requested checks passed.")


if __name__ == "__main__":
    main()

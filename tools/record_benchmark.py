#!/usr/bin/env python3
"""Record headless measurement output with machine/build/content identity."""

import argparse
import hashlib
import json
import platform
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    """Run the benchmark executable, check its modes agree, and write results plus identity."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("executable", type=Path)
    parser.add_argument("scenario", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--iterations", type=int, default=1000)
    args = parser.parse_args()
    executable = args.executable.resolve(strict=True)
    scenario = args.scenario.resolve(strict=True)

    # The benchmark prints one JSON line per mode; both must end on the same trajectory.
    results = [
        json.loads(line)
        for line in subprocess.check_output(
            [str(executable), str(scenario), str(args.iterations)], text=True
        ).splitlines()
    ]
    if (
        len(results) != 2
        or results[0]["final_position_checksum"] != results[1]["final_position_checksum"]
    ):
        raise RuntimeError(
            "benchmark modes did not finish with the same physical trajectory checksum"
        )

    # Machine identity: the lscpu fields that affect timing.
    fields = {"Architecture:", "CPU(s):", "Vendor ID:", "Model name:", "CPU max MHz:"}
    cpu = [
        item
        for item in json.loads(subprocess.check_output(["lscpu", "--json"], text=True))["lscpu"]
        if item["field"] in fields
    ]

    # Build identity: compiler and flags from the CMake cache next to the executable.
    cache = (executable.parent / "CMakeCache.txt").read_text()
    build_keys = {
        "CMAKE_BUILD_TYPE",
        "CMAKE_CXX_COMPILER",
        "CMAKE_CXX_FLAGS",
        "CMAKE_CXX_FLAGS_RELEASE",
    }
    build = {}
    for line in cache.splitlines():
        if ":" in line and "=" in line and line.split(":", 1)[0] in build_keys:
            build[line.split(":", 1)[0]] = line.split("=", 1)[1]

    # Hashes pin the exact code, build and content the measurements came from.
    metadata = {
        "scope": "Bundled headless scenario; no renderer/cameras/ROS/tasks. Construction excluded; 3 warmup runs per mode.",
        "machine": {"platform": platform.platform(), "cpu": cpu},
        "build": build,
        "revision": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
        ).strip(),
        "worktree_diff_sha256": hashlib.sha256(
            subprocess.check_output(["git", "diff", "HEAD"], cwd=ROOT)
        ).hexdigest(),
        "executable_sha256": hashlib.sha256(executable.read_bytes()).hexdigest(),
        "benchmark_driver_sha256": hashlib.sha256(
            (ROOT / "applications/benchmark/main.cpp").read_bytes()
        ).hexdigest(),
        "compiler": subprocess.check_output(["c++", "--version"], text=True).splitlines()[0],
        "build_cache_sha256": hashlib.sha256(
            (executable.parent / "CMakeCache.txt").read_bytes()
        ).hexdigest(),
        "content_sha256": {
            str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted((ROOT / "content").rglob("*.yaml"))
        },
        "scenario": str(scenario.relative_to(ROOT)),
        "iterations": args.iterations,
        "measurements": results,
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(metadata, indent=2) + "\n")
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()

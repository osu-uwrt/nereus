#!/usr/bin/env bash
# One-step setup and build: the Python environment (uv sync -> .venv) and the C++ preset.
#   ./build.sh            ros-viewer when a ROS 2 workspace is sourced, else datasets (headless renderer)
#   ./build.sh <preset>   any configure preset in CMakePresets.json (release, dev, datasets, ros-viewer, ...)
# NEREUS_JOBS overrides the parallel build jobs. Default: one per 3 GB of available memory after keeping 4 GB for
# the desktop, at most nproc (heavy translation units peak near 3 GB; too many jobs OOM-kill the session).
set -euo pipefail
cd "$(dirname "$0")"

preset=${1:-}
if [[ -z $preset ]]; then
    if [[ -n ${ROS_DISTRO:-} ]]; then preset=ros-viewer; else preset=datasets; fi
fi
available_gb=$(awk '/MemAvailable/ {print int($2 / 1048576)}' /proc/meminfo)
jobs=$(((available_gb - 4) / 3))
((jobs > $(nproc))) && jobs=$(nproc)
((jobs < 1)) && jobs=1
jobs=${NEREUS_JOBS:-$jobs}

if ! command -v uv >/dev/null; then
    echo "uv not found: install it once with 'curl -LsSf https://astral.sh/uv/install.sh | sh'" >&2
    echo "(or 'pip install --user uv'), then rerun." >&2
    exit 1
fi
uv sync --locked

# The build resolves packs with Python; use the project environment, not whatever python3 is on PATH.
cmake --preset "$preset" -DPython_EXECUTABLE="$PWD/.venv/bin/python"
echo "Building preset '$preset' with $jobs jobs (${available_gb} GB available; NEREUS_JOBS overrides)."
cmake --build --preset "$preset" -j "$jobs"

echo
echo "Built preset '$preset' in build/$preset."
echo "Python tools: 'source .venv/bin/activate' (nereus-dataset, nereus-packs), or prefix commands with 'uv run'."

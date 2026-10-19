#!/usr/bin/env bash
# One-step setup and build: the Python environment (uv sync -> .venv) and the C++ preset.
#   ./build.sh            ros-viewer when a ROS 2 workspace is sourced, else datasets (headless renderer)
#   ./build.sh <preset>   any configure preset in CMakePresets.json (release, dev, datasets, ros-viewer, ...)
# NEREUS_JOBS overrides the parallel build jobs. Default: one per 3 GB of available memory after keeping 4 GB for
# the desktop, at most nproc (heavy translation units peak near 3 GB; too many jobs OOM-kill the session).
set -euo pipefail
cd "$(dirname "$0")"

# Pick the preset: the first argument, else ros-viewer inside a sourced ROS 2 environment, else datasets.
preset=${1:-}
if [[ -z $preset ]]; then
    if [[ -n ${ROS_DISTRO:-} ]]; then preset=ros-viewer; else preset=datasets; fi
fi

# Size the parallel jobs from available memory (see the header), clamped to [1, nproc].
available_gb=$(awk '/MemAvailable/ {print int($2 / 1048576)}' /proc/meminfo)
jobs=$(((available_gb - 4) / 3))
((jobs > $(nproc))) && jobs=$(nproc)
((jobs < 1)) && jobs=1
jobs=${NEREUS_JOBS:-$jobs}

# Keep an active conda environment out of the build: CMake finds packages through PATH, and conda's libraries
# (libcurl, gtest, yaml-cpp, ...) then shadow the system ones that OpenCV/GDAL link against, failing the link.
if [[ -n ${CONDA_PREFIX:-} ]]; then
    conda_root=${_CONDA_ROOT:-$CONDA_PREFIX}
    # Literal prefix match (the root may hold regex characters).
    kept=()
    IFS=: read -ra entries <<<"$PATH"
    for entry in "${entries[@]}"; do
        [[ $entry == "$conda_root"* ]] || kept+=("$entry")
    done
    PATH=$(IFS=:; echo "${kept[*]}")
    unset CONDA_PREFIX
    # A cache configured inside conda keeps its package paths; start that preset's configure over.
    if grep -qsF "$conda_root" "build/$preset/CMakeCache.txt"; then
        echo "Reconfiguring build/$preset: its CMake cache was made with conda active."
        rm -rf "build/$preset/CMakeCache.txt" "build/$preset/CMakeFiles"
    fi
fi

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
# ROS builds register `ros2 launch nereus ...` (cmake/NereusRosIndex.cmake); point at it unless ~/.bashrc has it.
prefix_setup="$PWD/build/$preset/ros/setup.bash"
if [[ -f $prefix_setup ]] && ! grep -qsF "$prefix_setup" ~/.bashrc; then
    echo "ROS launch: 'source $prefix_setup', then 'ros2 launch nereus sim.launch.py'"
    echo "(./setup_bashrc.sh can add it to ~/.bashrc)."
fi

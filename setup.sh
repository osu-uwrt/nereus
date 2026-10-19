#!/usr/bin/env bash
# One-time machine setup: the system libraries the build needs (apt) and uv, if it isn't installed yet.
# On a ROS 2 machine, also runs ./setup_bashrc.sh, which offers to make `ros2 launch nereus ...` work in new shells
# (a block in ~/.bashrc; NEREUS_BASHRC=y|n answers without asking). Then build with ./build.sh (see the README).
set -euo pipefail
root=$(cd "$(dirname "$0")" && pwd)

sudo apt-get update
sudo apt-get install -y cmake g++ libeigen3-dev libyaml-cpp-dev nlohmann-json3-dev libgtest-dev \
    libbullet-dev libassimp-dev libglfw3-dev libglew-dev libegl-dev libpng-dev libjpeg-dev libopencv-dev \
    libfontconfig1-dev

if ! command -v uv >/dev/null; then
    curl -LsSf https://astral.sh/uv/install.sh | sh
fi

ros_setups=(/opt/ros/*/setup.bash)
if [[ -n ${ROS_DISTRO:-} || -e ${ros_setups[0]} ]]; then
    "$root/setup_bashrc.sh"
fi

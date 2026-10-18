#!/usr/bin/env bash
# One-time machine setup: the system libraries the build needs (apt) and uv, if it isn't installed yet.
# On a ROS 2 machine, also offers to make `ros2 launch nereus ...` work in new shells (a block in ~/.bashrc;
# NEREUS_BASHRC=y|n answers without asking). Then build with ./build.sh (see the README).
set -euo pipefail
root=$(cd "$(dirname "$0")" && pwd)

sudo apt-get update
sudo apt-get install -y cmake g++ libeigen3-dev libyaml-cpp-dev nlohmann-json3-dev libgtest-dev \
    libbullet-dev libassimp-dev libglfw3-dev libglew-dev libegl-dev libpng-dev libjpeg-dev libopencv-dev \
    libfontconfig1-dev

if ! command -v uv >/dev/null; then
    curl -LsSf https://astral.sh/uv/install.sh | sh
fi

# `ros2 launch nereus ...`: source the ament prefix the ros-viewer build writes (cmake/NereusRosIndex.cmake). The
# [ -f ] guard keeps new shells quiet until the first ./build.sh; rerunning replaces the block (e.g. after moving
# the repository).
ros_setups=(/opt/ros/*/setup.bash)
if [[ -n ${ROS_DISTRO:-} || -e ${ros_setups[0]} ]]; then
    prefix_setup="$root/build/ros-viewer/ros/setup.bash"
    begin="# >>> nereus >>>"
    end="# <<< nereus <<<"
    block="$begin
[ -f \"$prefix_setup\" ] && source \"$prefix_setup\"
$end"
    bashrc="$HOME/.bashrc"
    if grep -qsxF "[ -f \"$prefix_setup\" ] && source \"$prefix_setup\"" "$bashrc"; then
        echo "~/.bashrc already sets up 'ros2 launch nereus ...'."
    else
        answer=${NEREUS_BASHRC:-}
        if [[ -z $answer && -t 0 ]]; then
            read -rp "Add 'ros2 launch nereus ...' support to ~/.bashrc? [Y/n] " answer || true
            answer=${answer:-y}
        fi
        if [[ $answer =~ ^[Yy] ]]; then
            if grep -qsxF "$begin" "$bashrc"; then
                # Drop the old block, keeping the file itself (permissions, symlink).
                kept=$(awk -v b="$begin" -v e="$end" '$0 == b {skip = 1} !skip {print} $0 == e {skip = 0}' "$bashrc")
                printf '%s\n' "$kept" >"$bashrc"
            fi
            printf '\n%s\n' "$block" >>"$bashrc"
            echo "Added to ~/.bashrc (new shells; remove the '$begin' block to undo)."
        else
            echo "For 'ros2 launch nereus ...', add to ~/.bashrc after your ROS setup:"
            echo "  [ -f \"$prefix_setup\" ] && source \"$prefix_setup\""
        fi
    fi
fi

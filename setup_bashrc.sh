#!/usr/bin/env bash
# Makes `ros2 launch nereus ...` work in new shells: a block in ~/.bashrc that sources the ament prefix the
# ros-viewer build writes (cmake/NereusRosIndex.cmake). ./setup.sh runs this on ROS 2 machines; run it on its own
# to add the block later or refresh it (e.g. after moving the repository). NEREUS_BASHRC=y|n answers without asking.
# The [ -f ] guard keeps new shells quiet until the first ./build.sh; rerunning replaces the block.
set -euo pipefail
root=$(cd "$(dirname "$0")" && pwd)

prefix_setup="$root/build/ros-viewer/ros/setup.bash"
begin="# >>> nereus >>>"
end="# <<< nereus <<<"
block="$begin
[ -f \"$prefix_setup\" ] && source \"$prefix_setup\"
$end"
bashrc="$HOME/.bashrc"
if grep -qsxF "[ -f \"$prefix_setup\" ] && source \"$prefix_setup\"" "$bashrc"; then
    echo "~/.bashrc already sets up 'ros2 launch nereus ...'."
    exit 0
fi

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

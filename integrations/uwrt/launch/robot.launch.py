"""Real-robot operator interface: the pool viewer against a running Talos (no simulator, no stack).

    ros2 launch integrations/uwrt/launch/robot.launch.py [robot_only:=true] [rmw:=<rmw implementation>]
        [pool:=robosub|rpac] [scenario:=<pack folder>] [config:=<viewer host yaml>]

Resolves the scenario pack (robot model, cameras, frames, panels) with
`python -m nereus.packs resolve` and runs nereus-viewer with --pose-source estimate:
the robot is drawn from the localization estimate (map -> <ns>/base_link), wall-clock time (no /clock),
simulator-only panels hidden, camera cards and point clouds from the robot's ROS topics.
robot_only:=true hides the simulated pool and course layout (it does not match a real pool).
The robot must be reachable with the same RMW / ROS_DOMAIN_ID (Zenoh: a router connected to the robot's).
"""

import os
import subprocess
import sys
import tempfile
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration as LC

ROOT = Path(__file__).resolve().parents[3]
# The resolver needs the pack tools' dependencies, which ./build.sh installs into .venv; ros2 launch runs
# under the system Python.
PYTHON = str(ROOT / ".venv/bin/python") if (ROOT / ".venv/bin/python").exists() else sys.executable
# pool:=<name> picks the matching UWRT scenario; scenario:=<folder> picks any pack instead.
POOLS = {"robosub": "talos_uwrt", "rpac": "talos_uwrt_rpac"}


def _scenario(context):
    scenario, pool = LC("scenario").perform(context), LC("pool").perform(context)
    if scenario and pool:
        raise RuntimeError("pass pool:= or scenario:=, not both")
    if scenario:
        return os.path.abspath(scenario)
    if (pool or "robosub") not in POOLS:
        raise RuntimeError(f"unknown pool '{pool}' (known: {', '.join(POOLS)})")
    return str(ROOT / "content/packs/scenarios" / POOLS[pool or "robosub"])



def _processes(context):
    rmw = LC("rmw").perform(context)
    actions = [SetEnvironmentVariable("RMW_IMPLEMENTATION", rmw)] if rmw else []
    # The resolver and viewer run in the repository root: relative paths mean the caller's directory.
    scenario = _scenario(context)
    config = LC("config").perform(context)
    resolved = str(Path(tempfile.gettempdir()) / "nereus_robot_resolved.json")
    subprocess.run(
        [PYTHON, "-m", "nereus.packs", "resolve", scenario, "-o", resolved],
        check=True, cwd=str(ROOT),
        env={**os.environ, "PYTHONPATH": os.pathsep.join([str(ROOT / "python/src"), os.environ.get("PYTHONPATH", "")])})
    command = [LC("viewer_binary").perform(context), "--scenario", resolved, "--pose-source", "estimate",
               "--use-sim-time", "false"]
    if config:
        command += ["--config", os.path.abspath(config)]
    if LC("robot_only").perform(context).lower() in ("true", "1", "yes"):
        command.append("--robot-only")
    actions.append(ExecuteProcess(cmd=command, cwd=str(ROOT), output="screen", name="pool_viewer"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("pool", default_value="",
                              description=f"pool to run in: {' | '.join(POOLS)} (empty: robosub)"),
        DeclareLaunchArgument("scenario", default_value="",
                              description="scenario pack folder (robot model, cameras, frames); overrides pool"),
        DeclareLaunchArgument("robot_only", default_value="false",
                              description="hide the simulated pool and course; draw only the robot"),
        DeclareLaunchArgument("config", default_value="",
                              description="viewer host yaml (default: content/viewer/talos_uwrt_host.yaml)"),
        DeclareLaunchArgument("viewer_binary", default_value=str(ROOT / "build/ros-viewer/nereus-viewer"),
                              description="nereus-viewer executable"),
        DeclareLaunchArgument("rmw", default_value="",
                              description="RMW for the viewer (e.g. rmw_zenoh_cpp); empty keeps the shell's"),
        OpaqueFunction(function=_processes),
    ])

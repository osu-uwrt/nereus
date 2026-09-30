"""Real-robot operator interface: the pool viewer against a running Talos (no simulator, no stack).

    ros2 launch integrations/uwrt/launch/robot.launch.py [robot_only:=true] [rmw:=<rmw implementation>]
        [scenario:=<pack folder>] [config:=<viewer host yaml>]

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


def _processes(context):
    rmw = LC("rmw").perform(context)
    actions = [SetEnvironmentVariable("RMW_IMPLEMENTATION", rmw)] if rmw else []
    resolved = str(Path(tempfile.gettempdir()) / "nereus_robot_resolved.json")
    subprocess.run(
        [sys.executable, "-m", "nereus.packs", "resolve", LC("scenario").perform(context), "-o", resolved],
        check=True, cwd=str(ROOT),
        env={**os.environ, "PYTHONPATH": os.pathsep.join([str(ROOT / "python/src"), os.environ.get("PYTHONPATH", "")])})
    command = [LC("viewer_binary").perform(context), "--scenario", resolved, "--pose-source", "estimate",
               "--use-sim-time", "false"]
    if LC("config").perform(context):
        command += ["--config", LC("config").perform(context)]
    if LC("robot_only").perform(context).lower() in ("true", "1", "yes"):
        command.append("--robot-only")
    actions.append(ExecuteProcess(cmd=command, cwd=str(ROOT), output="screen", name="pool_viewer"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value=str(ROOT / "content/packs/scenarios/talos_uwrt"),
                              description="scenario pack folder (robot model, cameras, frames)"),
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

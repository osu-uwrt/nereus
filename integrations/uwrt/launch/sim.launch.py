"""One-command UWRT simulation: new simulator bridge + unchanged robot stack + pool viewer.

    ros2 launch integrations/uwrt/launch/sim.launch.py [stack:=false] [viewer:=false]
        [scenario:=<pack folder>] [output:=<run dir>] [rmw:=rmw_fastrtps_cpp]

Run records (resolved.json, execution.json, summary.json, tasks.json) go to `output`
(default /tmp/robotics_sim/<timestamp>). Ctrl-C stops everything and writes the records.
"""

import os
import time
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC

ROOT = Path(__file__).resolve().parents[3]


def _processes(context):
    rmw = LC("rmw").perform(context)
    actions = [SetEnvironmentVariable("RMW_IMPLEMENTATION", rmw)] if rmw else []
    output = LC("output").perform(context) or f"/tmp/robotics_sim/{time.strftime('%Y%m%d-%H%M%S')}"
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    python_path = os.pathsep.join(
        [str(ROOT / "integrations/ros2/python"), str(ROOT / "python/src"),
         os.environ.get("PYTHONPATH", "")])
    actions.append(ExecuteProcess(
        cmd=["python3", "-m", "robotics_platform_ros", LC("scenario").perform(context),
             "--output", output],
        cwd=str(ROOT), additional_env={"PYTHONPATH": python_path}, output="screen",
        sigterm_timeout="15", name="simulator"))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ROOT / "integrations/uwrt/acceptance/mission_stack.launch.py")),
        condition=IfCondition(LC("stack"))))
    actions.append(ExecuteProcess(
        cmd=[str(ROOT / "build/ros-viewer/robotics-pool-viewer")], cwd=str(ROOT),
        output="screen", name="pool_viewer", condition=IfCondition(LC("viewer"))))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value=str(ROOT / "content/packs/scenarios/talos_uwrt")),
        DeclareLaunchArgument("output", default_value=""),
        DeclareLaunchArgument("stack", default_value="true", description="launch the UWRT stack"),
        DeclareLaunchArgument("viewer", default_value="true", description="launch the pool viewer"),
        DeclareLaunchArgument("rmw", default_value="rmw_fastrtps_cpp",
                              description="RMW for every process; empty keeps the shell's"),
        OpaqueFunction(function=_processes),
    ])

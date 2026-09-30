"""One-command UWRT simulation: new simulator bridge + unchanged robot stack + pool viewer.

    ros2 launch integrations/uwrt/launch/sim.launch.py [stack:=false] [viewer:=false]
        [scenario:=<pack folder>] [output:=<run dir>] [rmw:=<rmw implementation>]
        [bridge:=python|cpp] [cameras:=true|false] [always_cameras:=true|false]
        [active_control_model:=mpc] [mpc_model:=sim|<name>|<path>] [mpc_odom_topic:=simulator/ground_truth]
        [mpc_state_source:=sensors|odometry]

bridge:=cpp runs the rclcpp simulator (build/ros-viewer/.../nereus-sim) on the pack resolved
with `python -m nereus.packs resolve`; cameras:=false passes --no-cameras and
always_cameras:=true renders every camera output regardless of subscribers.

The controller arguments pass through to riptide_bringup2 (mission_stack.launch.py); empty keeps bringup's
default controller. MPC on the simulator plant: active_control_model:=mpc mpc_model:=sim.

Run records (resolved.json, execution.json, summary.json, tasks.json) go to `output`
(default /tmp/nereus_sim/<timestamp>). Ctrl-C stops everything and writes the records.
"""

import os
import subprocess
import sys
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
STACK = ROOT / "integrations/uwrt/acceptance/mission_stack.launch.py"
# Controller selection forwarded to mission_stack.launch.py (and on to riptide_bringup2); empty = its default.
CONTROLLER_ARGS = {
    "active_control_model": "controller: 'mpc' runs riptide_mpc, anything else complete_controller",
    "active_control_enabled": "launch the active controller (True/False)",
    "mpc_model": "MPC model: '' = vehicle estimate, 'sim' = the sim plant copy, a name or a path",
    "mpc_odom_topic": "MPC state feedback topic, e.g. simulator/ground_truth to bypass the EKF",
    "mpc_state_source": "MPC feedback: sensors (default) or odometry",
}


def _processes(context):
    rmw = LC("rmw").perform(context)
    actions = [SetEnvironmentVariable("RMW_IMPLEMENTATION", rmw)] if rmw else []
    output = LC("output").perform(context) or f"/tmp/nereus_sim/{time.strftime('%Y%m%d-%H%M%S')}"
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    scenario = LC("scenario").perform(context)
    if LC("bridge").perform(context) == "cpp":
        binary = LC("bridge_binary").perform(context)
        resolved = f"{output}.resolved.json"
        subprocess.run(
            [sys.executable, "-m", "nereus.packs", "resolve", scenario, "-o", resolved],
            check=True, cwd=str(ROOT),
            env={**os.environ, "PYTHONPATH": os.pathsep.join(
                [str(ROOT / "python/src"), os.environ.get("PYTHONPATH", "")])})
        command = [binary, resolved, "--output", output]
        if LC("cameras").perform(context).lower() in ("false", "0", "no"):
            command.append("--no-cameras")
        elif LC("always_cameras").perform(context).lower() in ("true", "1", "yes"):
            command.append("--always-cameras")
        actions.append(ExecuteProcess(cmd=command, cwd=str(ROOT), output="screen",
                                      sigterm_timeout="15", name="simulator"))
    else:
        python_path = os.pathsep.join(
            [str(ROOT / "integrations/ros2/python"), str(ROOT / "python/src"),
             os.environ.get("PYTHONPATH", "")])
        cmd = ["python3", "-m", "nereus_ros", scenario, "--output", output]
        if LC("cameras").perform(context).lower() in ("false", "0", "no"):
            cmd.append("--no-cameras")
        actions.append(ExecuteProcess(
            cmd=cmd, cwd=str(ROOT), additional_env={"PYTHONPATH": python_path}, output="screen",
            sigterm_timeout="15", name="simulator"))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(STACK)),
        launch_arguments=[(k, LC(k)) for k in CONTROLLER_ARGS],
        condition=IfCondition(LC("stack"))))
    actions.append(ExecuteProcess(
        cmd=[str(ROOT / "build/ros-viewer/nereus-viewer")], cwd=str(ROOT),
        output="screen", name="pool_viewer", condition=IfCondition(LC("viewer"))))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value=str(ROOT / "content/packs/scenarios/talos_uwrt")),
        DeclareLaunchArgument("output", default_value=""),
        DeclareLaunchArgument("stack", default_value="true", description="launch the UWRT stack"),
        DeclareLaunchArgument("viewer", default_value="true", description="launch the pool viewer"),
        DeclareLaunchArgument("bridge", default_value="cpp", description="simulator bridge: cpp or python (reference)"),
        DeclareLaunchArgument("bridge_binary", default_value=str(
            ROOT / "build/ros-viewer/integrations/ros2/bridge/nereus-sim"),
            description="nereus-sim executable used by bridge:=cpp"),
        DeclareLaunchArgument("cameras", default_value="true", description="run camera acquisition"),
        DeclareLaunchArgument("always_cameras", default_value="false",
                              description="cpp bridge: render cameras regardless of subscribers"),
        # Default: the shell's RMW (UWRT uses rmw_zenoh_cpp with a running `ros2 run rmw_zenoh_cpp
        # rmw_zenohd`). FastDDS showed 0.4-0.9 s reliable-delivery stalls of the simulator's /tf under
        # full-stack load with camera traffic; Zenoh delivered the same run without stalls.
        DeclareLaunchArgument("rmw", default_value="",
                              description="RMW for every process (e.g. rmw_zenoh_cpp); empty keeps the shell's"),
        *[DeclareLaunchArgument(k, default_value="", description=d) for k, d in CONTROLLER_ARGS.items()],
        OpaqueFunction(function=_processes),
    ])

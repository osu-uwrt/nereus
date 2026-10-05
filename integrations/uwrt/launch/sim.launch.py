"""One-command UWRT simulation: nereus-sim + the UWRT robot stack + the pool viewer.

    ros2 launch integrations/uwrt/launch/sim.launch.py [stack:=false] [viewer:=false] [close_with_viewer:=false]
        [pool:=robosub|rpac] [scenario:=<pack folder>] [output:=<run dir>] [rmw:=<rmw implementation>]
        [cameras:=true|false] [always_cameras:=true|false] [camera_supersample:=1..4]
        [active_control_model:=mpc] [mpc_model:=sim|<name>|<path>] [mpc_odom_topic:=simulator/ground_truth]
        [mpc_state_source:=sensors|odometry] [nvidia:=auto|true|false]

The simulator (build/ros-viewer/.../nereus-sim) runs the scenario resolved with
`python -m nereus.packs resolve`; cameras:=false passes --no-cameras and
always_cameras:=true renders every camera output regardless of subscribers. camera_supersample sets the
camera anti-aliasing factor (empty: the simulator's default, off; 2..4 supersample).

The controller arguments pass through to riptide_bringup2 (mission_stack.launch.py); empty keeps bringup's
default controller. With active_control_model:=mpc the MPC models THIS run's plant by default
(mpc_model empty or sim): mpc_sim_model.py writes its vehicle, hydrodynamics and mpc.yaml (DVL stamped at measurement)
from the resolved scenario into <output>.mpc/. mpc_model:=talos (or another name/path) uses that model instead,
e.g. the pool-identified estimate of the real vehicle.

Run records (resolved.json, execution.json, summary.json, tasks.json) go to `output`
(default /tmp/nereus_sim/<timestamp>). Ctrl-C, or closing the viewer, stops everything and writes the records;
close_with_viewer:=false keeps the simulator and stack running when the viewer is closed.

The simulator runs under sim_supervisor.py, which restarts it in another pool when the viewer asks (View > Pool;
or `ros2 topic pub --once /talos/simulator/load_scenario std_msgs/msg/String "{data: rpac}"`). Each restart writes
its records to <output>-2, <output>-3, ...; the robot stack keeps running.
"""

import ctypes.util
import json
import os
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
    Shutdown,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sim_supervisor  # noqa: E402  (resolving and the pool table, shared with the supervisor)

ROOT = Path(__file__).resolve().parents[3]
STACK = ROOT / "integrations/uwrt/acceptance/mission_stack.launch.py"
SUPERVISOR = Path(__file__).resolve().parent / "sim_supervisor.py"
# Controller selection forwarded to mission_stack.launch.py (and on to riptide_bringup2); empty = its default.
CONTROLLER_ARGS = {
    "active_control_model": "controller: 'mpc' runs riptide_mpc, anything else complete_controller",
    "active_control_enabled": "launch the active controller (True/False)",
    "mpc_model": "MPC model: '' or 'sim' = this run's plant (generated), 'talos' = vehicle estimate, a name or a path",
    "mpc_odom_topic": "MPC state feedback topic, e.g. simulator/ground_truth to bypass the EKF",
    "mpc_state_source": "MPC feedback: sensors (default) or odometry",
}


# pool:=<name> picks the matching UWRT scenario; scenario:=<folder> picks any pack instead.
POOLS = sim_supervisor.POOLS


def _scenario(context):
    scenario, pool = LC("scenario").perform(context), LC("pool").perform(context)
    if scenario and pool:
        raise RuntimeError("pass pool:= or scenario:=, not both")
    if scenario:
        return os.path.abspath(scenario)
    if (pool or "robosub") not in POOLS:
        raise RuntimeError(f"unknown pool '{pool}' (known: {', '.join(POOLS)})")
    return str(ROOT / "content/packs/scenarios" / POOLS[pool or "robosub"])


def _gpu_env(context):
    """PRIME render offload for the viewer: on a hybrid laptop GLX opens windows on the integrated GPU unless
    asked for NVIDIA (the simulator's EGL cameras already pick NVIDIA). auto: offload when the NVIDIA GLX
    driver is installed."""
    mode = LC("nvidia").perform(context).lower()
    if mode not in ("auto", "true", "false"):
        raise RuntimeError(f"nvidia:= must be auto, true or false (got '{mode}')")
    if mode == "false" or (mode == "auto" and not ctypes.util.find_library("GLX_nvidia")):
        return {}
    return {"__NV_PRIME_RENDER_OFFLOAD": "1", "__GLX_VENDOR_LIBRARY_NAME": "nvidia"}


def _mpc_sim_model(context, resolved, output):
    """MPC model files for this run's plant (see mpc_sim_model.py), as mission_stack arguments."""
    if LC("active_control_model").perform(context) != "mpc" or LC("mpc_model").perform(context) not in ("", "sim"):
        return []
    from ament_index_python.packages import get_package_share_directory as share
    import mpc_sim_model

    robot = json.loads(Path(resolved).read_text())["robot"]["id"]
    vehicle, hydro, params = mpc_sim_model.write(
        resolved, f"{output}.mpc", Path(share("riptide_descriptions2"), "config", f"{robot}.yaml"),
        Path(share("riptide_mpc"), "config", "mpc.yaml"))
    print(f"MPC model: this run's plant ({output}.mpc)")
    return [("mpc_vehicle_config", vehicle), ("mpc_hydrodynamics_config", hydro), ("mpc_config", params)]


def _processes(context):
    rmw = LC("rmw").perform(context)
    actions = [SetEnvironmentVariable("RMW_IMPLEMENTATION", rmw)] if rmw else []
    # The resolver and simulator run in the repository root: relative paths mean the caller's directory.
    output = os.path.abspath(LC("output").perform(context) or f"/tmp/nereus_sim/{time.strftime('%Y%m%d-%H%M%S')}")
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    scenario = _scenario(context)
    binary = LC("bridge_binary").perform(context)
    resolved = f"{output}.resolved.json"
    sim_supervisor.resolve(scenario, resolved)
    stack_args = [(k, LC(k)) for k in CONTROLLER_ARGS] + _mpc_sim_model(context, resolved, output)
    # The supervisor runs `binary <resolved> --output <dir> <options>` and restarts it in another pool on request.
    command = [sys.executable, str(SUPERVISOR), "--resolved", resolved, "--scenario", scenario, "--output", output,
               "--", binary]
    if LC("cameras").perform(context).lower() in ("false", "0", "no"):
        command.append("--no-cameras")
    elif LC("always_cameras").perform(context).lower() in ("true", "1", "yes"):
        command.append("--always-cameras")
    if LC("camera_supersample").perform(context):
        command += ["--camera-supersample", LC("camera_supersample").perform(context)]
    actions.append(ExecuteProcess(cmd=command, cwd=str(ROOT), output="screen",
                                  sigterm_timeout="20", name="simulator"))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(STACK)),
        launch_arguments=stack_args,
        condition=IfCondition(LC("stack"))))
    # Closing the viewer ends the whole launch (as Ctrl-C: the simulator writes its records), unless
    # close_with_viewer:=false keeps the simulator and stack running for a viewer reopened by hand.
    close = LC("close_with_viewer").perform(context).lower() not in ("false", "0", "no")
    actions.append(ExecuteProcess(
        cmd=[str(ROOT / "build/ros-viewer/nereus-viewer")], cwd=str(ROOT),
        output="screen", name="pool_viewer", additional_env=_gpu_env(context),
        on_exit=[Shutdown(reason="the pool viewer was closed")] if close else None,
        condition=IfCondition(LC("viewer"))))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("pool", default_value="",
                              description=f"pool to run in: {' | '.join(POOLS)} (empty: robosub)"),
        DeclareLaunchArgument("scenario", default_value="", description="scenario pack folder (overrides pool)"),
        DeclareLaunchArgument("output", default_value=""),
        DeclareLaunchArgument("stack", default_value="true", description="launch the UWRT stack"),
        DeclareLaunchArgument("viewer", default_value="true", description="launch the pool viewer"),
        DeclareLaunchArgument("close_with_viewer", default_value="true",
                              description="closing the viewer stops the simulator and the stack (false: keep them)"),
        DeclareLaunchArgument("bridge_binary", default_value=str(
            ROOT / "build/ros-viewer/integrations/ros2/bridge/nereus-sim"),
            description="nereus-sim executable"),
        DeclareLaunchArgument("nvidia", default_value="auto",
                              description="viewer on the NVIDIA GPU via PRIME offload: auto | true | false"),
        DeclareLaunchArgument("cameras", default_value="true", description="run camera acquisition"),
        DeclareLaunchArgument("always_cameras", default_value="false",
                              description="render cameras regardless of subscribers"),
        DeclareLaunchArgument("camera_supersample", default_value="",
                              description="camera anti-aliasing factor 1..4 (empty: the simulator's default)"),
        # Default: the shell's RMW (UWRT uses rmw_zenoh_cpp with a running `ros2 run rmw_zenoh_cpp
        # rmw_zenohd`). FastDDS showed 0.4-0.9 s reliable-delivery stalls of the simulator's /tf under
        # full-stack load with camera traffic; Zenoh delivered the same run without stalls.
        DeclareLaunchArgument("rmw", default_value="",
                              description="RMW for every process (e.g. rmw_zenoh_cpp); empty keeps the shell's"),
        *[DeclareLaunchArgument(k, default_value="", description=d) for k, d in CONTROLLER_ARGS.items()],
        OpaqueFunction(function=_processes),
    ])

"""Launch the robotics-pool-viewer ROS 2 client next to a running simulator bridge.

The viewer takes its scene from the bridge's latched scenario topic; nothing here names a simulator
config file. Arguments mirror the executable's flags (see `robotics-pool-viewer --help`).

  ros2 launch integrations/ros2/viewer/launch/robotics_pool_viewer.launch.py
  ros2 launch ... scenario:=/path/to/resolved.json pack_dir:=content/packs/scenarios/talos_uwrt demo:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _viewer(context, *_):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    command = [value("executable")]
    for flag, name in (("--scenario", "scenario"), ("--pack-dir", "pack_dir"), ("--config", "config"),
                       ("--panels", "panels"), ("--scenario-topic", "scenario_topic"), ("--focus", "focus"),
                       ("--screenshot", "screenshot"), ("--shaders", "shaders")):
        if value(name):
            command += [flag, value(name)]
    if value("frames") != "0":
        command += ["--frames", value("frames")]
    for flag, name in (("--demo", "demo"), ("--hidden", "hidden"), ("--show-tf", "show_tf"),
                       ("--detections", "detections"), ("--mpc-path", "mpc_path"),
                       ("--show-scorecard", "show_scorecard")):
        if value(name).lower() in ("true", "1"):
            command.append(flag)
    command += ["--use-sim-time", value("use_sim_time")]
    return [ExecuteProcess(cmd=command, output="screen")]


def generate_launch_description():
    arguments = {
        "executable": "robotics-pool-viewer",
        "scenario": "", "pack_dir": "", "config": "", "panels": "", "scenario_topic": "", "focus": "",
        "screenshot": "", "shaders": "", "frames": "0", "demo": "false", "hidden": "false", "show_tf": "false",
        "detections": "false", "mpc_path": "false", "show_scorecard": "false", "use_sim_time": "true",
    }
    return LaunchDescription([DeclareLaunchArgument(name, default_value=default)
                              for name, default in arguments.items()] + [OpaqueFunction(function=_viewer)])

"""The UWRT stack as riptide_bringup2 simulation.launch.py brings it up (hardware:=none), minus the
simulator itself: every node uses the simulation clock; navigation, control, perception, mapping
and autonomy come from bringup. The AprilTag detector (hardware.launch.py's, which hardware:=none skips)
runs on the simulated forward camera, for the mapping panel's tag calibration.

Controller selection is passed through to bringup / control_system.launch.py / riptide_mpc (empty =
that launch file's own default):
    active_control_model:=mpc  mpc_model:=sim  mpc_odom_topic:=simulator/ground_truth  mpc_state_source:=...
"""

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import PushRosNamespace, SetParameter

# Launch arguments forwarded to bringup when non-empty, with their descriptions.
PASSTHROUGH = {
    "active_control_model": "controller: 'mpc' runs riptide_mpc, anything else complete_controller",
    "active_control_enabled": "launch the active controller (True/False)",
    "mpc_model": "MPC model: '' = config/models/<robot>.yaml (vehicle estimate), 'sim' = the sim plant copy, a name or a path",
    "mpc_odom_topic": "MPC state feedback topic, e.g. simulator/ground_truth to bypass the EKF",
    "mpc_state_source": "MPC feedback: sensors (default) or odometry",
    "mpc_config": "riptide_mpc params file (default: riptide_mpc config/mpc.yaml)",
    "mpc_vehicle_config": "vehicle YAML for the MPC model (overrides mpc_model)",
    "mpc_hydrodynamics_config": "hydrodynamics YAML for the MPC model (overrides mpc_model)",
}


def _stack(context):
    """Bringup (hardware:=none) plus the AprilTag detector, with only the explicitly set arguments."""
    arguments = {"hardware": "none", "robot": "talos"}
    arguments.update({k: v for k in PASSTHROUGH if (v := LC(k).perform(context))})
    # forwarding=False: the stack sees only `arguments`, never the caller's launch configurations. An
    # inherited empty value (e.g. active_control_enabled="") would override bringup's own defaults.
    return [
        GroupAction(
            forwarding=False,
            launch_configurations={},
            actions=[
                SetParameter(name="use_sim_time", value=True),
                SetParameter(name="write_ff_autotune", value=False),
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource(
                        share("riptide_bringup2") + "/launch/bringup.launch.py"
                    ),
                    launch_arguments=arguments.items(),
                ),
                # The AprilTag detector, under the robot namespace like hardware.launch.py runs it.
                GroupAction(
                    [
                        PushRosNamespace("talos"),
                        IncludeLaunchDescription(
                            AnyLaunchDescriptionSource(
                                share("riptide_hardware2") + "/launch/apriltag.launch.py"
                            ),
                            launch_arguments={"robot": "talos"}.items(),
                        ),
                    ]
                ),
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value="", description=d) for k, d in PASSTHROUGH.items()]
        + [OpaqueFunction(function=_stack)]
    )

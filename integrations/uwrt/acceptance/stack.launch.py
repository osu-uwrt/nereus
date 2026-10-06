"""The UWRT navigation (EKF) and control stack for Talos on the simulation clock, plus the world -> map
chameleon_tf, for the controller/EKF hold acceptance (hold.py) against a separately launched simulator."""

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    return LaunchDescription(
        [
            GroupAction(
                [
                    # Every node on sim time; never write feedforward autotune results back to config.
                    SetParameter(name="use_sim_time", value=True),
                    SetParameter(name="write_ff_autotune", value=False),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            share("riptide_hardware2") + "/launch/navigation.launch.py"
                        ),
                        launch_arguments={"robot": "talos"}.items(),
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            share("riptide_controllers2") + "/launch/control_system.launch.py"
                        ),
                        launch_arguments={"robot": "talos"}.items(),
                    ),
                    Node(
                        package="chameleon_tf",
                        executable="chameleon_tf",
                        name="world_to_map",
                        namespace="talos",
                        parameters=[
                            {
                                "source_frame": "world",
                                "target_frame": "map",
                                "initial_translation": [0.0, 0.0, 0.0],
                                "initial_rotation": [0.0, 0.0, 0.0],
                                "transform_locks": [False, False, True, True, True, False],
                                "stddev_threshold": 0.5,
                            }
                        ],
                    ),
                ]
            )
        ]
    )

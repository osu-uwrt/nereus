"""The unchanged UWRT stack exactly as the original simulator brought it up (hardware:=none).

Same as riptide_bringup2 simulation.launch.py minus the old simulator: every node uses the
simulation clock; navigation, control, perception, mapping and autonomy come from bringup.
"""

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import SetParameter


def generate_launch_description():
    return LaunchDescription(
        [
            GroupAction(
                [
                    SetParameter(name="use_sim_time", value=True),
                    SetParameter(name="write_ff_autotune", value=False),
                    IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(
                            share("riptide_bringup2") + "/launch/bringup.launch.py"
                        ),
                        launch_arguments={"hardware": "none", "robot": "talos"}.items(),
                    ),
                ]
            )
        ]
    )

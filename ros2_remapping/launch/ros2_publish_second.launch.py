import os

# import toml
import launch_ros.actions
import launch_ros.descriptions
from launch import LaunchDescription


def generate_launch_description():
    deva_root = ""
    ros2_remapping_runner = launch_ros.actions.Node(
        package="ros2_remapping",
        executable="my_second_publish_exe",
        parameters=[
            {"ego_gnss_same": False},
        ],
        remappings=[
            (
                "/your/test",
                "/my/test",
            )
        ],
    )

    return LaunchDescription([ros2_remapping_runner])

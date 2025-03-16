import os

import yaml
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    runner = Node(
        package="message_filter",
        executable="message_filter_exe"
    )

    return LaunchDescription([runner])

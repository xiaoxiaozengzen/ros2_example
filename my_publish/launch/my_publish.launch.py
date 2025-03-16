import os

import yaml
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():


    runner = Node(
        package="my_publish",
        executable="my_publish_exe"
    )

    return LaunchDescription([runner])

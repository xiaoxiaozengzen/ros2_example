import os

import yaml
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():


    runner = Node(
        package="python_example",
        executable="python_example_test1_exe"
    )

    return LaunchDescription([runner])

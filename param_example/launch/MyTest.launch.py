import os

import yaml
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():

    config = "/mnt/workspace/cgz_workspace/Exercise/ros2_example/param_example/config/para_example.yaml"
    config_2 = "/mnt/workspace/cgz_workspace/Exercise/ros2_example/param_example/config/para_example_2.yaml"
    runner = Node(
        package="para_example",
        executable="para_example_exe",
        parameters=[
          {"path": "nihaoaaaa"},
          {"map1": {"key1": 1, "key2": "value2"}},
          {"map2": {"key1": "values", "key2": "value2"}},
          {"list1": [1, 2, 3, 4]},
          [config],
          # [config_2]
        ]
    )

    return LaunchDescription([runner])

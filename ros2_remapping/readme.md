# Overview

用于测试ros2中的remapping功能

# launch

```bash
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

```

其中 `remappings` 就是用来重定向消息名字的：

* 第一个位置是节点内部的消息：例如/your/test就是你节点内部原始的订阅或者发布的消息
* 第二个位置是映射后的消息名字：例如/my/test就是你节点内部映射完之后，重新订阅或者发布的消息

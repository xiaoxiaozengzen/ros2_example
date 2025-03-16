# parameters

```bash
ros2 run para_example para_example_exe --ros-args -p path:="nihaoa"
```

```bash
# 或者
./install/para_example/lib/para_example/para_example_exe --ros-args -p path:="nihaoa"
```

# launch

launch启动的话，就是将launch.py中的内容拼接成命令行参数

```bash
ros2 launch para_example MyTest.launch.py

# 实际等价
para_example_exe --ros-args --params-file /tmp/launch_params_3lp_qd47

# 会生成临时文件，/tmp/launch_params_3lp_qd47，内容如下：
/**:
  ros__parameters:
    path: nihaoaaaa
```

## launch.py

```pythion
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
```

可以看到，parameters中可以添加多种类型参数，且每一行参数都会对应一个临时文件，[config]也会被作为参数文件进行追加，如下：

```bash
[para_example_exe-1] argv[0]: /mnt/workspace/cgz_workspace/Exercise/ros2_example/install/para_example/lib/para_example/para_example_exe
[para_example_exe-1] argv[1]: --ros-args
[para_example_exe-1] argv[2]: --params-file
[para_example_exe-1] argv[3]: /tmp/launch_params_8w8ylin9
[para_example_exe-1] argv[4]: --params-file
[para_example_exe-1] argv[5]: /tmp/launch_params_o2vt4h7k
[para_example_exe-1] argv[6]: --params-file
[para_example_exe-1] argv[7]: /tmp/launch_params_cqazyyw2
[para_example_exe-1] argv[8]: --params-file
[para_example_exe-1] argv[9]: /tmp/launch_params_ufepcqld
[para_example_exe-1] argv[10]: --params-file
[para_example_exe-1] argv[11]: /mnt/workspace/cgz_workspace/Exercise/ros2_example/param_example/config/para_example.yaml

```

**注意**

launch启动时如果不同数组中的元素的类型有差异，就会报错

# para file

也可以自己写一个yaml，传给节点进行解析:

```bash
ros2 run para_example para_example_exe --ros-args --params-file ./param_example/config/para_example.yaml

# 注意 配置文件中节点名字一定要写清除，和创建Node传入的名字要一致，才会自动解析
```

**注意：** 

配置文件中的第一行是节点名字，如果节点名字不对，则无法成功位当前节点解析。也可以写成 `/**` ，表示匹配任意节点

配置文件含义：
* `para_example.yaml` ：正常的参数配置文件，用于示例
* `para_example_2.yaml` ：数组重复但是类别不同，用于验证节点遇到重复数组，但是类别不同的时候会报错
* `para_example_3.yaml` ：节点使用 `/**` ，表示匹配任意节点
* `para_example_4.yaml` ：使用错误的节点名字，用于验证节点无法解析对应的参数

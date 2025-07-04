# Overview

基于本地ros2，使用cmake进行构建

## 编译

```bash
source /opt/ros/foxy/setup.sh

mkdir build && cd build

cmake .. -DCMAKE_TOOLCHAIN_FILE=$(pwd)/../ros2_toolchain.cmake
make

```

## 运行

```bash
# 终端1用来跑程序

source /opt/ros/foxy/setup.sh
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<project_path>/build/lib

cd <project_path>
./build/bin/my_test

# terminal 2，使用ros2来接受消息
source /opt/ros/foxy/setup.sh
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<project_path>/build/lib
export PYTHONPATH=$PYTHONPATH:<project_path>/build/devastator_perception_msgs/rosidl_generator_py
ros2 topic echo /my/test
```

其中 `project_path` 表示当前项目的主目录

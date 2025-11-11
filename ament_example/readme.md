# Overview

本文章主要是讨论ros2中使用的colcon和ament构建工具的使用

# colcon

## 加载ros2环境变量

首先执行`source /opt/ros/foxy/setup.sh`，然后使用colcon进行编译

```bash
colcon build --packages-up-to hello
```

发现，
1. 此时环境变量存在: `AMENT_PREFIX_PATH`为`/opt/ros/foxy`
2. 编译之前找不到环境变量`CMAKE_PREFIX_PATH`
3. 编译得时候：设置`CMAKE_PREFIX_PATH`为`/opt/ros/foxy`
4. 编译结束后，使用printenv找不到`CMAKE_PREFIX_PATH`。

看起来，基于`AMENT_PREFIX_PATH`，间接找到ros2提供的相关cmake package

**注意**
source了ros2的环境变量的时候
1. 会发现其设置了`AMENT_PREFIX_PATH=/opt/ros/foxy`，colcon会根据这个环境变量去查找相关的包的位置
2. 其还会设置 `PYTHONPATH` ，让colcon能找到一些ros相关的python包

`https://github1s.com/colcon/colcon-ros/blob/master/colcon_ros/task/__init__.py#L17` 
代码中发现其会在`CMAKE_PREFIX_PATH`中添加`AMENT_PREFIX_PATH`中的内容

## 加载install下的脚本

`source ./install/setup.sh`后，打印当前环境中的变量，会发现：

```bash
COLCON_PREFIX_PATH=/mnt/workspace/cgz_workspace/Exercise/ros2_example/ament_example/hello/install

CMAKE_PREFIX_PATH=/mnt/workspace/cgz_workspace/Exercise/ros2_example/ament_example/hello/install/hello

AMENT_PREFIX_PATH=/mnt/workspace/cgz_workspace/Exercise/ros2_example/ament_example/hello/install/hello:/opt/ros/foxy
```

**注意**
AMENT_PREFIX_PATH:多次source不同的install/setup.sh脚本，后来者会在前者的基础上进行添加，不会有重复的

COLCON_PREFIX_PATH:多次source会重复进行添加，即会有重复的

# ament

`ament`可以看成集成了一些了的cmake函数跟宏，它会读取你的`package.xml`


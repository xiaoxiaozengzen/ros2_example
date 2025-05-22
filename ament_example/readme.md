# Overview

本文章主要是讨论ros2中使用的colcon和ament构建工具的使用

# colcon

## 加载ros2环境变量

首先执行`source /opt/ros/foxy/setup.sh`，然后使用colcon进行编译

```bash
colcon build --packages-up-to hello
```

发现，其在编译期间会设置`CMAKE_PREFIX_PATH`为`/opt/ros/foxy`，但是在编译前跟编译后，都printenv这个环境变量。这也解释了怎么找到ros2提供的相关cmake package

**注意**
source了ros2的环境变量的时候，会发现其设置了`AMENT_PREFIX_PATH=/opt/ros/foxy`，难道是根据这个在编译的时候设置了环境变量？

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


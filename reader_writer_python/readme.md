# Overview

主要用于使用mcap接口解析和生成mcap文件

# 概念

## channel

```bash
$ mcap info z18_141839.mcap
library:   mcap go v1.7.1; mbridges
profile:
messages:  6668
duration:  39.995158625s
start:     2025-04-27T14:18:39.252101558+08:00 (1745734719.252101558)
end:       2025-04-27T14:19:19.247260183+08:00 (1745734759.247260183)
compression:
        zstd: [26/26 chunks] [104.58 MiB/50.33 MiB (51.87%)] [1.26 MiB/sec]
channels:
        (26) /sensor/cam_front_120/h264      401 msgs (10.03 Hz)    : foxglove_msgs/msg/CompressedVideo [ros2msg]
        (33) /sensor/vehicle_report_common  4000 msgs (100.01 Hz)   : deva_control_msgs/msg/VehicleReportCommon [ros2msg]
        (79) /sensor/radar0_object           667 msgs (16.68 Hz)    : deva_perception_msgs/msg/RadarObjectArray [ros2msg]
        (80) /sensor/radar1_object           800 msgs (20.00 Hz)    : deva_perception_msgs/msg/RadarObjectArray [ros2msg]
        (81) /sensor/radar2_object           800 msgs (20.00 Hz)    : deva_perception_msgs/msg/RadarObjectArray [ros2msg]
channels: 5
attachments: 0
metadata: 0
```

`mcap info`可以查看消息得channel通道数，例如`/sensor/vehicle_report_common`的channel id就是33

## schemas

```bash
$ mcap list schemas z18_141839.mcap
22      deva_control_msgs/msg/VehicleReportCommon       ros2msg         deva_common_msgs/Header header
```

这里schemas会存储消息类型的定义，对于类型 `deva_control_msgs/msg/VehicleReportCommon`其schemas id=22
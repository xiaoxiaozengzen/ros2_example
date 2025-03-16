# perception msg

## Input

- Lidar: sensor_msgs/PointCloud2.msg
- Image: sensor_msgs/Image.msg

## Output

- TrafficLight: TrafficLight.msg
- Lane: Lane.msg
  - 目前是 2D相机坐标系上的点
  - 后续会慢慢把 Lane 换成线条加属性的方式.
- Obstacle: 通过点云和图像结果找到的障碍物坐标位置. Obstacle.msg

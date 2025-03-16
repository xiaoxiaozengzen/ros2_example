uint8 id  # 障碍物id

# 障碍物位置
# 实际有效的为position.pose.position.x和position.pose.position.y
# 其中坐标为radar坐标系下的坐标，单位m
geometry_msgs/PoseWithCovariance position

# 障碍物的线速度
# 实际有效的为relative_velocity.twist.linear.x和relative_velocity.twist.linear.y
# 单位 m/s
geometry_msgs/TwistWithCovariance relative_velocity

# 障碍物的加速度值，目前无效
geometry_msgs/AccelWithCovariance relative_acceleration

float64 rcs     # 障碍物雷达目标有效截面（dBm）

# 障碍物的动态特性，取值如下:
# 0x0: Moving
# 0x1: Stationary
# 0x2: Oncoming
# 0x3: Stationary candidate
# 0x4: Unknown
# 0x5: Crossing stationary
# 0x6: Crossing moving
# 0x7: Stopped
# 0x8: Receding
uint8 dynamic_property

bool quality_valid  # 信号质量相关字段内容是否有效

# 障碍物测量状态，当quality_valid=true时值有效，取值如下：
# 0x0: Deleted
# 0x1: New
# 0x2: Measured
# 0x3: Predicted
# 0x4: Deleted for merge
# 0x5: New from merge
uint8 meas_state

# 障碍物存在的可能性，当quality_valid=true时值有效，取值如下：
# 0x0: invalid
# 0x1: <25%
# 0x2: <50%
# 0x3: <75%
# 0x4: <90%
# 0x5: <99%
# 0x6: <99.9%
# 0x7: <=100%
uint8 prob_of_exist

bool extended_valid  # 扩展字段是否有效
float64 length  # 障碍物长度，单位m，当extended_valid=true时值有效
float64 width   # 障碍物宽度，单位m，当extended_valid=true时值有效
float64 center_latitude  # 障碍物轮廓中心点相对于障碍物位置的y坐标
float64 center_longitude  # 障碍物轮廓中心点相对于障碍物位置的x坐标
float64 orientation_angle  # 障碍物方位角，单位deg，当extended_valid=true时值有效

# 障碍物类型，当extended_valid=true时值有效，取值如下:
# 0x0: Point
# 0x1: Car
# 0x2: Truck
# 0x3: Pedestrian
# 0x4: Motorcycle
# 0x5: Bicycle
# 0x6: Wide
# 0x7: 未知值
# 0x8: Vehicle of unknown class
# 0x9: Animal
uint8 class_type

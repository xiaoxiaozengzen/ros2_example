float64 confidence              #
uint32 lane_id                  # 禁止使用id 0！！！
uint32 left_lane_id             # id为0表示无左/右车道！！！
uint32 right_lane_id
uint32 fork_lane_id

# 像素坐标系
deva_common_msgs/Point2d[] imagepoints
uint32 v_min           # 像素坐标系v轴最小值
uint32 v_max           # 像素坐标系v轴最大值

# ego 坐标系
geometry_msgs/Point[] waypoints
float64 x_min          # ego坐标系x轴最小值
float64 x_max          # ego坐标系x轴最大值

# ego 坐标系
geometry_msgs/Point[] waypoints_extend
float64 x_min_extend         # 延长车道线ego坐标系x轴最小值
float64 x_max_extend         # 延长车道线ego坐标系x轴最大值


# 车道线属性，取值如下:
uint8 UNKNOWN = 0               # 0x0: 其他
uint8 DOTTED_YELLOW = 1         # 0x1: 黄色虚线
uint8 DOTTED_WHITE = 2          # 0x2: 白色虚线
uint8 SOLID_YELLOW = 3          # 0x3: 黄色实线
uint8 SOLID_WHITE = 4           # 0x4: 白色实线
uint8 DOUBLE_YELLOW = 5         # 0x5: 双黄线
uint8 CURB = 6                  # 0x6: 路沿
uint8 GUARDRAIL = 7             # 0x7: 栏杆
uint8 DOUBLE_DOTTED_SOLID_YELLOW = 8
uint8 DOUBLE_SOLID_DOTTED_YELLOW = 9
uint8 DOUBLE_DOTTED_SOLID_WHITE = 10
uint8 DOUBLE_SOLID_DOTTED_WHITE = 11
uint8 STOPLINE = 12
uint8 ROADEDGELANE = 13
uint8 DOUBLEROADEDGELANE = 14
uint8 lane_property
LanePropertySegment lane_property_segment

# 分叉线属性，取值如下:
uint8 NORMAL = 0         # 0x0: 非叉线
uint8 SPLIT = 1          # 0x1: 分叉线
uint8 MERGE = 2          # 0x2: 合流线
uint8 SPLIT_DIVERSION = 3          # 0x1: 分叉导流线
uint8 MERGE_DIVERSION = 4          # 0x2: 合流导流线
uint8 fork_property
deva_common_msgs/Point2d rv_fork_point
geometry_msgs/Point bev_fork_point


# 拟合 ego 坐标系车道线下的样条曲线.
# lane_xy: xy平面下拟合曲线， lane_xz: xz平面下拟合曲线
# x_min, x_max 分别为取样的上界和下界
deva_common_msgs/Line2d lane_xy
deva_common_msgs/Line2d lane_xz
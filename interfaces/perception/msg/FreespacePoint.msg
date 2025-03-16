# 障碍物相对于ego垂心的rfu坐标系
#
#               y axis
#		              ^
#                     |
#                  *  |  *
#               *     |     *
#        \    *       |       *    /
#          \ *     range(r)    * /
#            \        |        /
#              \      |      /
#                \    |    /
#                  \  |  /         x axis
#                     --------------->
#

float64 x
float64 y
float64 z
float64 rfu_x
float64 rfu_y
float64 rfu_z
float64 azim
float64 range
# 0x0: Static
# 0x1: Dynamic
uint8 class_type
uint32 ins_id

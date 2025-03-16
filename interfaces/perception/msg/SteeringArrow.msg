uint32 id
float64 confidence
geometry_msgs/Point center_point
geometry_msgs/Point[] vertices
int32 center_line_id
uint8 property

uint8 UNKNOWN = 0
uint8 GUIDE_TURN = 1
uint8 GUIDE_TURN_THROUGH = 2
uint8 GUIDE_TURN_LEFT = 3
uint8 GUIDE_TURN_RIGHT = 4
uint8 GUIDE_LEFT = 5
uint8 GUIDE_LEFT_THROUGH = 6
uint8 GUIDE_LEFT_RIGHT = 7
uint8 GUIDE_LEFT_THROUGH_RIGHT = 8
uint8 GUIDE_THROUGH = 9
uint8 GUIDE_THROUGH_RIGHT = 10
uint8 GUIDE_RIGHT = 11
uint8 FORBID_TURN = 12
uint8 FORBID_LEFT = 13
uint8 FORBID_LEFT_RIGHT = 14
uint8 FORBID_THROUGH = 15
uint8 FORBID_THROUGH_RIGHT = 16
uint8 FORBID_THROUGH_LEFT = 17
uint8 FORBID_RIGHT = 18
uint8 MERGE_LEFT = 19
uint8 MERGE_RIGHT = 20
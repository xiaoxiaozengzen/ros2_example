# 车道线属性
uint8 UNKNOWN = 0              
uint8 DASHED_LINE  = 1         
uint8 SOLID_LINE  = 2         
uint8 L_DASHED_R_SOLID  = 3         
uint8 L_SOLID_R_DASHED  = 4          
uint8 DOUBLE_DASHED = 5        
uint8 DOUBLE_SOLID = 6                 
uint8 CURB = 7            
uint8 GUARDRAIL = 8
uint8 CENTER_LINE = 9
uint8 VIRTUAL_JUNCTION = 10
uint8 VIRTUAL = 11

uint8 lane_type
# ColorType 
uint8 WHITE = 1
uint8 YELLOW = 2
uint8 DOUBLE_YELLOW  = 3
uint8 lane_color

float64 s_start
float64 s_end
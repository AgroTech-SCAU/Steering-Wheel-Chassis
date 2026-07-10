std_msgs/Header header

uint8 STATE_IDLE=0
uint8 STATE_RUNNING=1
uint8 STATE_SUCCEEDED=2
uint8 STATE_FAILED=3
uint8 STATE_CANCELLED=4

uint8 state
string backend
string waypoint_id
float64 target_x_m
float64 target_y_m
float64 target_yaw_rad
float64 distance_error_m
float64 yaw_error_rad
int32 error_code
string message

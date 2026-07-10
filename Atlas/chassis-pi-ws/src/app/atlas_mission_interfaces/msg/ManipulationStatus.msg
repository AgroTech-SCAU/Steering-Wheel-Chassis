std_msgs/Header header

uint8 STATE_IDLE=0
uint8 STATE_RUNNING=1
uint8 STATE_SUCCEEDED=2
uint8 STATE_FAILED=3
uint8 STATE_CANCELLED=4

uint8 state
string backend
string waypoint_id
string task_id
string step_name
int32 error_code
string message

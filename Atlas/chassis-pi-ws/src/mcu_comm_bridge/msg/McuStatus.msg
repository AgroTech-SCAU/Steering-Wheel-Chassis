std_msgs/Header header

uint8 STATE_IDLE=0
uint8 STATE_MANUAL=1
uint8 STATE_AUTO_PI=2
uint8 STATE_FAULT=3
uint8 STATE_ESTOP=4
uint8 STATE_FINISHED=5

uint32 mcu_stamp_ms
uint8 app_state
uint8 manual_mode
uint8 ready_flags
uint8 online_flags
uint8 fault_source
uint8 fault_level
int16 fault_code
bool auto_start_latched

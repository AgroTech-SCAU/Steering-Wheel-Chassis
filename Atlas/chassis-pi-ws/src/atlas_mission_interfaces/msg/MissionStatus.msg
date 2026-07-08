std_msgs/Header header

uint8 STATE_BOOTSTRAP=0
uint8 STATE_WAIT_MCU_STATUS=1
uint8 STATE_WAIT_START=2
uint8 STATE_PRECHECK=3
uint8 STATE_INITIALIZING=4
uint8 STATE_RUNNING=5
uint8 STATE_ABORTING=6
uint8 STATE_REPORTING_DONE=7
uint8 STATE_WAIT_MCU_FINISHED=8
uint8 STATE_REPORTING_FAIL=9
uint8 STATE_WAIT_MCU_FAULT=10
uint8 STATE_WAIT_RESET=11
uint8 STATE_RECOVERY_REQUIRED=12
uint8 STATE_SHUTTING_DOWN=13

uint8 state
uint32 local_run_id
bool active
bool mcu_status_fresh
bool auto_start_latched
uint8 mcu_app_state
bool result_reported
int32 error_code
string state_name
string message

# Asynchronous MCU IK/servo acceptance, not motion completion
uint8 ACCEPTED=0
uint8 NO_SOLUTION=1
uint8 INVALID_PARAM=2
uint8 KINEMATICS_FAILED=3
uint8 SERVO_FAILED=4
uint8 TIMEOUT=5
uint8 UNKNOWN=6
uint16 command_seq
uint8 result
# Original MCU ArmStatus, or -1 when execution never started
int32 arm_status

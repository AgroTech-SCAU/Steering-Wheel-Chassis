std_msgs/Header header

# ASRPRO 串口桥运行状态
uint8 STATE_OFFLINE=0
uint8 STATE_CONNECTING=1
uint8 STATE_READY=2
uint8 STATE_SPEAKING=3
uint8 STATE_ERROR=4

uint8 state
bool serial_connected
bool device_ready
bool speech_busy
bool listen_enabled
uint32 rx_sequence
uint32 tx_sequence
uint32 reconnect_count
string firmware_version
string boot_id
string last_intent
string message

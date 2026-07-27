std_msgs/Header header

# ASRPRO 事件类型
uint8 EVENT_HELLO=1
uint8 EVENT_ACK=2
uint8 EVENT_NACK=3
uint8 EVENT_ASR=4
uint8 EVENT_SPEAK_DONE=5
uint8 EVENT_PONG=6
uint8 EVENT_PROTOCOL_ERROR=7
uint8 EVENT_DISCONNECTED=8

uint8 event
uint32 sequence
string intent
string phrase_id
string payload
string message

#ifndef _binary_frame_h_
#define _binary_frame_h_

/**
 * @file binary_frame.h
 * @brief 通用二进制帧协议辅助接口
 */

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 宏 定 义 声 明 ========================= ! //

#define BINARY_FRAME_SOF0 0xA5u
#define BINARY_FRAME_SOF1 0x5Au
#define BINARY_FRAME_PROTOCOL_VERSION 0x01u
#define BINARY_FRAME_HEADER_SIZE 2u
#define BINARY_FRAME_LENGTH_FIELD_SIZE 2u
#define BINARY_FRAME_BODY_PREFIX_SIZE 4u
#define BINARY_FRAME_CRC_SIZE 2u
#define BINARY_FRAME_FIXED_OVERHEAD (BINARY_FRAME_HEADER_SIZE + BINARY_FRAME_LENGTH_FIELD_SIZE + BINARY_FRAME_CRC_SIZE)

#define BINARY_FRAME_FLAG_NEED_ACK 0x01u

#define BINARY_FRAME_MSG_PC_HEARTBEAT 0x10u
#define BINARY_FRAME_MSG_PC_MASTER_JOINTS 0x11u

#define BINARY_FRAME_MSG_MCU_STATUS 0x21u
#define BINARY_FRAME_MSG_MCU_START_SENSOR_EVENT 0x22u
#define BINARY_FRAME_MSG_MCU_ACK 0x23u
#define BINARY_FRAME_MSG_MCU_FAULT_EVENT 0x24u
#define BINARY_FRAME_MSG_MCU_IMU 0x25u
#define BINARY_FRAME_MSG_MCU_ODOM 0x26u
#define BINARY_FRAME_MSG_MCU_ARM_STATE 0x27u

#define BINARY_FRAME_MSG_PI_HEARTBEAT 0x30u
#define BINARY_FRAME_MSG_PI_CONTROL 0x31u
#define BINARY_FRAME_MSG_PI_ARM_ACTION 0x40u
#define BINARY_FRAME_MSG_PI_YAW_ACTION 0x41u
#define BINARY_FRAME_MSG_PI_MISSION_EVENT 0x42u
#define BINARY_FRAME_MSG_PI_ESTOP 0x43u
#define BINARY_FRAME_MSG_PI_ACK 0x44u

#define BINARY_FRAME_PI_CONTROL_MASK_CHASSIS_VALID 0x01u
#define BINARY_FRAME_PI_CONTROL_MASK_ARM_VALID 0x02u
#define BINARY_FRAME_PI_CONTROL_MASK_BRAKE_REQUEST 0x08u

#define BINARY_FRAME_PI_ARM_MODE_NONE 0x00u
#define BINARY_FRAME_PI_ARM_MODE_JOINTS 0x01u
#define BINARY_FRAME_PI_ARM_MODE_POSE_5D 0x02u
#define BINARY_FRAME_PI_ARM_MODE_POSITION 0x03u
#define BINARY_FRAME_PI_ARM_MODE_ORIENTATION_2D 0x04u

#define BINARY_FRAME_PI_ARM_ACTION_ENABLE 0x01u
#define BINARY_FRAME_PI_ARM_ACTION_STOP 0x02u
#define BINARY_FRAME_PI_ARM_ACTION_SEQUENCE 0x03u

#define BINARY_FRAME_PI_YAW_ACTION_HOLD_ENABLE 0x01u
#define BINARY_FRAME_PI_YAW_ACTION_HOLD_DISABLE 0x02u
#define BINARY_FRAME_PI_YAW_ACTION_TARGET_SET 0x03u

#define BINARY_FRAME_PI_MISSION_EVENT_DONE 0x01u
#define BINARY_FRAME_PI_MISSION_EVENT_FAIL 0x02u

#define BINARY_FRAME_STATUS_READY_CHASSIS 0x01u
#define BINARY_FRAME_STATUS_READY_ARM 0x02u
#define BINARY_FRAME_STATUS_READY_ODOM 0x04u
#define BINARY_FRAME_STATUS_READY_REMOTE 0x08u
#define BINARY_FRAME_STATUS_READY_PC 0x10u
#define BINARY_FRAME_STATUS_READY_PI 0x20u

#define BINARY_FRAME_STATUS_ONLINE_REMOTE 0x01u
#define BINARY_FRAME_STATUS_ONLINE_PC 0x02u
#define BINARY_FRAME_STATUS_ONLINE_PI 0x04u
#define BINARY_FRAME_STATUS_HAS_FAULT 0x08u
#define BINARY_FRAME_STATUS_ESTOP 0x10u

// ! ========================= 类 型 声 明 ========================= ! //

typedef struct {
    uint8_t version;
    uint8_t msg_id;
    uint8_t seq;
    uint8_t flags;
    const uint8_t* payload;
    uint16_t payload_len;
} BinaryFrameView;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

bool binary_frame_pack(uint8_t msg_id,
                       uint8_t seq,
                       uint8_t flags,
                       const uint8_t* payload,
                       uint16_t payload_len,
                       uint8_t* out,
                       uint16_t out_size,
                       uint16_t* out_len);

bool binary_frame_parse_body(const uint8_t* body,
                             uint16_t body_len,
                             BinaryFrameView* view);

uint16_t binary_frame_crc16_ccitt(const uint8_t* data, uint16_t len);

uint16_t binary_frame_read_u16_le(const uint8_t* data);
int16_t binary_frame_read_i16_le(const uint8_t* data);
uint32_t binary_frame_read_u32_le(const uint8_t* data);
int32_t binary_frame_read_i32_le(const uint8_t* data);

void binary_frame_write_u16_le(uint8_t* data, uint16_t value);
void binary_frame_write_i16_le(uint8_t* data, int16_t value);
void binary_frame_write_u32_le(uint8_t* data, uint32_t value);
void binary_frame_write_i32_le(uint8_t* data, int32_t value);

float binary_frame_urad_to_rad(int32_t urad);
int32_t binary_frame_rad_to_urad(float rad);
float binary_frame_mrad_to_rad(int32_t mrad);
int16_t binary_frame_rad_to_mrad_i16(float rad);
float binary_frame_mm_to_m(int32_t mm);
int16_t binary_frame_m_to_mm_i16(float m);
int32_t binary_frame_m_to_mm_i32(float m);
int32_t binary_frame_unit_to_e6_i32(float value);
int16_t binary_frame_unit_to_q15_i16(float value);

#endif

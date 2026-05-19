#ifndef _dm_motor_h_
#define _dm_motor_h_

#include "bus_motor.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 达妙电机命令帧长度, 单位 byte
 */
#define DM_MOTOR_CMD_LEN 8u

/**
 * @brief 达妙电机支持的最大协议 ID
 */
#define DM_MOTOR_MAX_ID 32u

/**
 * @brief 达妙电机 AGV 转向场景默认位置环参数
 */
#define DM_MOTOR_DEFAULT_KP 3.0f
#define DM_MOTOR_DEFAULT_KD 0.032f

/**
 * @brief 达妙电机通用模式枚举
 */
typedef enum {
    DM_MOTOR_MODE_MIT = 1u,
    DM_MOTOR_MODE_POS_VEL = 2u,
    DM_MOTOR_MODE_SPEED = 3u,
    DM_MOTOR_MODE_POS_VEL_TOR = 4u,
} DmMotorMode;

/**
 * @brief 达妙电机实例
 */
extern const BusMotorInterface dm_motor_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

BusMotorStatus dm_motor_parse_feedback_frame(uint32_t frame_id,
    const uint8_t data[DM_MOTOR_CMD_LEN],
    BusMotorFeedback* feedback);
BusMotorStatus dm_motor_clear_error(uint16_t id);
BusMotorStatus dm_motor_save_zero(uint16_t id);

#endif

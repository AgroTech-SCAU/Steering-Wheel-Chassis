#ifndef _dji_motor_h_
#define _dji_motor_h_

#include "bus_motor.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief DJI 电机命令帧长度, 单位 byte
 */
#define DJI_MOTOR_CMD_LEN 8u

/**
 * @brief DJI M3508 常用 ID 数量, 对应反馈帧 0x201 ~ 0x204
 */
#define DJI_MOTOR_MAX_ID 4u

/**
 * @brief DJI M3508 减速比
 */
#define DJI_MOTOR_M3508_REDUCTION_RATIO (3591.0f / 187.0f)

/**
 * @brief DJI 电机模式枚举
 */
typedef enum {
    DJI_MOTOR_MODE_SPEED = 1u,
} DjiMotorMode;

/**
 * @brief DJI 电机实例
 */
extern const BusMotorInterface dji_motor_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

BusMotorStatus dji_motor_parse_feedback_frame(uint32_t frame_id,
    const uint8_t data[DJI_MOTOR_CMD_LEN],
    BusMotorFeedback* feedback);

#endif

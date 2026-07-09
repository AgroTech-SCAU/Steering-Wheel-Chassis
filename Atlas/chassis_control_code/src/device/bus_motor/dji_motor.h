#ifndef _dji_motor_h_
#define _dji_motor_h_

#include "bus_motor.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @file dji_motor.h
 * @brief DJI M3508/C620 与 M2006/C610 总线电机驱动接口
 */

/**
 * @brief DJI 电机命令帧长度，单位 byte
 */
#define DJI_MOTOR_CMD_LEN 8u

/**
 * @brief DJI 电机反馈 ID 数量，对应反馈帧 0x201 ~ 0x204
 */
#define DJI_MOTOR_MAX_ID 4u

/**
 * @brief DJI 电机型号
 */
typedef enum {
    DJI_MOTOR_MODEL_M3508 = 0u,
    DJI_MOTOR_MODEL_M2006,
    DJI_MOTOR_MODEL_COUNT
} DjiMotorModel;

/**
 * @brief DJI 电机私有初始化配置
 *
 * 长期推荐由上层显式传入 `DjiMotorConfig`；
 * `NULL` 仅作为当前工程的兼容默认行为
 */
typedef struct {
    DjiMotorModel model;
} DjiMotorConfig;

/**
 * @brief DJI 电机控制模式
 */
typedef enum {
    DJI_MOTOR_MODE_SPEED = 1u, /**< 速度闭环模式 */
} DjiMotorMode;

/**
 * @brief DJI 电机统一接口实例
 */
extern const BusMotorInterface dji_motor_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 解析 DJI 电机反馈帧
 * @param frame_id CAN 反馈帧 ID
 * @param data 8 字节反馈数据
 * @param feedback 输出通用电机反馈
 * @return 电机状态码
 */
BusMotorStatus dji_motor_parse_feedback_frame(uint32_t frame_id,
                                              const uint8_t data[DJI_MOTOR_CMD_LEN],
                                              BusMotorFeedback* feedback);

/**
 * @brief 获取 DJI 电机当前型号
 * @return 当前型号
 */
DjiMotorModel dji_motor_get_model(void);

/**
 * @brief 将 DJI 电机型号转换为字符串
 * @param model DJI 电机型号
 * @return 型号名称字符串，未知型号返回 "UNKNOWN"
 */
const char* dji_motor_model_str(DjiMotorModel model);

#endif

#include "agv_motor.h"

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 当前绑定的转向电机实例
 */
const BusMotorInterface* steer_motor_instance = 0;

/**
 * @brief 当前绑定的驱动电机实例
 */
const BusMotorInterface* drive_motor_instance = 0;

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 绑定转向电机实例
 */
BusMotorStatus steer_motor_set_instance(const BusMotorInterface* instance) {
    if(instance == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    steer_motor_instance = instance;
    return MOTOR_STATUS_OK;
}

/**
 * @brief 绑定驱动电机实例
 */
BusMotorStatus drive_motor_set_instance(const BusMotorInterface* instance) {
    if(instance == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    drive_motor_instance = instance;
    return MOTOR_STATUS_OK;
}

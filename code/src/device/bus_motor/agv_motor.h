#ifndef _agv_motor_h_
#define _agv_motor_h_

#include "bus_motor.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 转向电机入口单例
 */
#define steer_motor (*steer_motor_instance)

/**
 * @brief 驱动电机入口单例
 */
#define drive_motor (*drive_motor_instance)

/**
 * @brief 当前绑定的转向电机实例
 */
extern const BusMotorInterface* steer_motor_instance;

/**
 * @brief 当前绑定的驱动电机实例
 */
extern const BusMotorInterface* drive_motor_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

BusMotorStatus steer_motor_set_instance(const BusMotorInterface* instance);
BusMotorStatus drive_motor_set_instance(const BusMotorInterface* instance);

#endif

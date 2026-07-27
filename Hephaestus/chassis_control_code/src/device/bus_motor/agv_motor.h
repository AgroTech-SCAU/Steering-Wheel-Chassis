#ifndef _agv_motor_h_
#define _agv_motor_h_

#include "bus_motor.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @file agv_motor.h
 * @brief AGV 底盘转向电机与驱动电机的角色绑定接口
 */

/**
 * @brief 当前转向电机实例的便捷访问宏
 */
#define steer_motor (*steer_motor_instance)

/**
 * @brief 当前驱动电机实例的便捷访问宏
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

/**
 * @brief 绑定转向电机具体实例
 * @param instance 电机接口表，例如 `&dm_motor_instance`
 * @return 电机状态码
 */
BusMotorStatus steer_motor_set_instance(const BusMotorInterface* instance);

/**
 * @brief 绑定驱动电机具体实例
 * @param instance 电机接口表，例如 `&dji_motor_instance`
 * @return 电机状态码
 */
BusMotorStatus drive_motor_set_instance(const BusMotorInterface* instance);

#endif

#ifndef _app_control_h_
#define _app_control_h_

/**
 * @file app_control.h
 * @brief MCU 控制映射与安全执行接口
 */

// ! ========================= 类 型 声 明 ========================= ! //

/**
 * @brief 控制执行结果
 * @details 供 `app_runtime` 判断当前周期的控制输出是正常执行, 被安全策略跳过, 还是执行失败
 */
typedef enum {
    APP_CONTROL_RESULT_OK = 0,
    APP_CONTROL_RESULT_SKIPPED,
    APP_CONTROL_RESULT_REJECTED,
    APP_CONTROL_RESULT_CHASSIS_ERROR,
    APP_CONTROL_RESULT_ARM_ERROR,
    APP_CONTROL_RESULT_ODOM_ERROR,
    APP_CONTROL_RESULT_COMMAND_INVALID,
    APP_CONTROL_RESULT_UNSUPPORTED
} AppControlResult;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化控制映射模块
 * @details 在 `app_runtime_init()` 中调用一次, 完成 yaw hold 等辅助控制器初始化
 */
void app_control_init(void);

/**
 * @brief 执行 ManualChassisPcArm 控制分支
 * @details FS-iA10B 控制底盘, PC 主臂关节角控制机械臂
 * @return AppControlResult 执行结果
 */
AppControlResult app_control_apply_manual_chassis_pc_arm(void);

/**
 * @brief 执行 ManualArmFs 控制分支
 * @details FS-iA10B 控制机械臂, 底盘保持刹车
 * @return AppControlResult 执行结果
 */
AppControlResult app_control_apply_manual_arm_fs(void);

/**
 * @brief 执行 AutoPi 控制分支
 * @details Pi 控制底盘, yaw 和机械臂, 所有命令都要经过限幅, 新鲜度和安全检查
 * @return AppControlResult 执行结果
 */
AppControlResult app_control_apply_auto_pi(void);

/**
 * @brief 请求底盘进入刹车状态
 * @return AppControlResult 执行结果
 */
AppControlResult app_control_brake_chassis(void);

/**
 * @brief 请求机械臂进入安全停止状态
 * @return AppControlResult 执行结果
 */
AppControlResult app_control_stop_arm(void);

/**
 * @brief 同时收口底盘和机械臂输出
 * @return AppControlResult 执行结果
 */
AppControlResult app_control_stop_all(void);

#endif

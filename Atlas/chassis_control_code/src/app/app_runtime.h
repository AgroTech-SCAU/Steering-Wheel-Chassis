#ifndef _app_runtime_h_
#define _app_runtime_h_

/**
 * @file app_runtime.h
 * @brief MCU 应用运行时主控接口
 */

#include "app_fsm.h"

#include <stdbool.h>

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化 MCU 应用运行时主控
 * @details 负责初始化系统级状态机, 控制映射模块和运行时缓存
 */
void app_runtime_init(void);

/**
 * @brief 执行一次 MCU 应用运行时轮询
 * @details 在 500Hz 主循环中统一完成输入更新, 模式仲裁, 安全检查和控制输出
 */
void app_runtime_process(void);

/**
 * @brief 获取当前系统级状态
 * @return AppFsmStateId 当前状态
 */
AppFsmStateId app_runtime_get_state(void);

/**
 * @brief 获取当前 Manual 子模式
 * @return AppManualMode 当前 Manual 子模式
 */
AppManualMode app_runtime_get_manual_mode(void);

/**
 * @brief 获取当前锁存故障
 * @return const AppFault* 故障只读指针
 */
const AppFault* app_runtime_get_fault(void);

/**
 * @brief 判断当前是否存在锁存故障
 * @return bool `true` 表示存在锁存故障
 */
bool app_runtime_has_fault(void);

/**
 * @brief 判断当前是否锁存了自动启动事件
 * @return bool `true` 表示锁存了自动启动事件
 */
bool app_runtime_is_auto_start_latched(void);

#endif

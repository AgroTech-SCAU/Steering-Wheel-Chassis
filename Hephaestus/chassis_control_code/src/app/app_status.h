#ifndef _app_status_h_
#define _app_status_h_

/**
 * @file app_status.h
 * @brief MCU 状态显示与低频日志接口
 */

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化状态显示模块
 * @details 在 `app_runtime_init()` 后调用一次
 */
void app_status_init(void);

/**
 * @brief 轮询 LED 和低频日志输出
 * @details 建议在后台循环持续调用
 */
void app_status_process(void);

#endif

#ifndef _suction_h_
#define _suction_h_

/**
 * @file suction.h
 * @brief 吸盘继电器控制接口
 */

#include <stdbool.h>

// ! ========================= 类 型 声 明 ========================= ! //

/**
 * @brief 吸盘控制结果
 */
typedef enum {
    SUCTION_RESULT_OK = 0,
    SUCTION_RESULT_INVALID_PARAM,
    SUCTION_RESULT_NOT_INITIALIZED
} SuctionResult;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化吸盘控制模块
 */
void suction_init(void);

/**
 * @brief 控制吸盘继电器
 * @param enable true-打开吸盘, false-关闭吸盘
 * @return SuctionResult 执行结果
 */
SuctionResult suction_set(bool enable);

/**
 * @brief 获取吸盘当前状态
 * @return bool true-吸盘已打开, false-吸盘已关闭
 */
bool suction_get_state(void);

#endif
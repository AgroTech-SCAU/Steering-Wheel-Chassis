/**
 * @file assemble_suction.c
 * @brief 吸盘控制模块组装实现
 */

#include "assemble.h"

#include "suction.h"

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_suction(void) {
    // 初始化吸盘控制模块
    suction_init();

    return SYSTEM_STATUS_OK;
}
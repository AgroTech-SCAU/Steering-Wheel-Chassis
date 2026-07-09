/**
 * @file suction.c
 * @brief 吸盘继电器控制实现
 */

#include "suction.h"
#include "main.h" // IWYU pragma: keep

// ! ========================= 宏 定 义 声 明 ========================= ! //

// 定义吸盘继电器控制引脚
// 请根据实际硬件连接修改以下定义
#define SUCTION_RELAY_PIN GPIO_PIN_9
#define SUCTION_RELAY_PORT GPIOE
#define SUCTION_RELAY_PIN2 GPIO_PIN_13
#define SUCTION_RELAY_PORT2 GPIOE

// ! ========================= 变 量 声 明 ========================= ! //

static bool s_suction_enabled = false;
static bool s_initialized = false;

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void suction_init(void) {
    // 默认关闭吸盘（继电器低电平驱动，所以输出高电平关闭）
    HAL_GPIO_WritePin(SUCTION_RELAY_PORT, SUCTION_RELAY_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SUCTION_RELAY_PORT2, SUCTION_RELAY_PIN2, GPIO_PIN_SET);
    s_suction_enabled = false;
    s_initialized = true;
}

SuctionResult suction_set(bool enable) {
    if(!s_initialized) {
        return SUCTION_RESULT_NOT_INITIALIZED;
    }

    if(enable) {
        // 继电器低电平驱动，输出低电平打开吸盘
        HAL_GPIO_WritePin(SUCTION_RELAY_PORT, SUCTION_RELAY_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SUCTION_RELAY_PORT2, SUCTION_RELAY_PIN2, GPIO_PIN_RESET);
        if(!s_suction_enabled) {
            s_suction_enabled = true;
        }
    }
    else {
        // 继电器低电平驱动，输出高电平关闭吸盘
        HAL_GPIO_WritePin(SUCTION_RELAY_PORT, SUCTION_RELAY_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(SUCTION_RELAY_PORT2, SUCTION_RELAY_PIN2, GPIO_PIN_SET);
        if(s_suction_enabled) {
            s_suction_enabled = false;
        }
    }

    return SUCTION_RESULT_OK;
}

bool suction_get_state(void) {
    return s_suction_enabled;
}
#ifndef _stm32_hal_exti_h_
#define _stm32_hal_exti_h_

#include "main.h" // IWYU pragma: keep

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //



// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 为指定 EXTI 引脚注册回调函数
 * @param gpio_pin EXTI 引脚掩码，必须为单一位
 * @param callback 回调函数，参数为触发的引脚掩码
 */
void exti_register_callback(uint16_t gpio_pin, void(*callback)(uint16_t gpio_pin));

#endif

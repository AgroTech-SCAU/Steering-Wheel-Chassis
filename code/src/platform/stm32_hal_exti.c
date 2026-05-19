#include "stm32_hal_exti.h"

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief EXTI 0~15 号引脚的回调函数表
 */
static void(*exti_callbacks[16])(uint16_t gpio_pin) = { 0 };

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 将 GPIO 引脚掩码转换为 EXTI 线索引
 * @param gpio_pin GPIO 引脚掩码
 * @return int8_t 成功返回 [0, 15]，失败返回 -1
 */
static int8_t exti_gpio_pin_to_index(uint16_t gpio_pin);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 为指定 EXTI 引脚注册回调函数
 * @param gpio_pin EXTI 引脚掩码，必须为单一位
 * @param callback 回调函数，参数为触发的引脚掩码
 */
void exti_register_callback(uint16_t gpio_pin, void(*callback)(uint16_t gpio_pin)) {
    int8_t pin_index = exti_gpio_pin_to_index(gpio_pin);

    if(pin_index < 0) {
        return;
    }

    exti_callbacks[pin_index] = callback;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief HAL EXTI 中断统一回调入口
 * @param GPIO_Pin 触发中断的 GPIO 引脚掩码
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    int8_t pin_index = exti_gpio_pin_to_index(GPIO_Pin);

    if(pin_index < 0) {
        return;
    }

    if(exti_callbacks[pin_index] != NULL) {
        exti_callbacks[pin_index](GPIO_Pin);
    }
}

/**
 * @brief 将 GPIO 引脚掩码转换为 EXTI 线索引
 * @param gpio_pin GPIO 引脚掩码
 * @return int8_t 成功返回 [0, 15]，失败返回 -1
 */
static int8_t exti_gpio_pin_to_index(uint16_t gpio_pin) {
    if((gpio_pin == 0U) || ((gpio_pin & (gpio_pin - 1U)) != 0U)) {
        return -1;
    }

    for(int8_t pin_index = 0; pin_index < 16; pin_index++) {
        if(gpio_pin == (uint16_t)(1U << pin_index)) {
            return pin_index;
        }
    }

    return -1;
}

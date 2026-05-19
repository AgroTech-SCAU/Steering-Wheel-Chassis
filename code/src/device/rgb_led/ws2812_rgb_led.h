#ifndef _ws2812_rgb_led_h_
#define _ws2812_rgb_led_h_

#include "rgb_led.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief WS2812 单个 RGB 灯珠编码后的 bit 数
 */
#define WS2812_RGB_LED_BITS_PER_PIXEL 24u

/**
 * @brief WS2812 RGB LED 默认像素数量
 */
#define WS2812_RGB_LED_DEFAULT_PIXEL_COUNT 8u

/**
 * @brief WS2812 RGB LED 默认复位时间对应的字节数
 *
 * 以 800KHz 的速率发送数据时，复位时间为 50us，约等于 80 字节的发送时间
 */
#define WS2812_RGB_LED_DEFAULT_RESET_BYTES 80u

/**
 * @brief WS2812 RGB LED 实例
 *
 * service 可通过 rgb_led_set_instance(&ws2812_rgb_led_instance)
 * 将其绑定为 rgb_led 统一入口的具体实现
 */
extern const RgbLedInterface ws2812_rgb_led_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

RgbLedStatus ws2812_rgb_led_make_config(RgbLedConfig* out_config, const RgbLedPortOps* ops,
    uint8_t* color_buffer, uint32_t color_buffer_size,
    uint8_t* tx_buffer, uint32_t tx_buffer_size);

#endif

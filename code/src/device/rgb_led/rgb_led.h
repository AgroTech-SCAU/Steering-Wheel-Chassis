#ifndef _rgb_led_h_
#define _rgb_led_h_

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief RGB LED 入口单例，用户上层统一调用 rgb_led.xxx 或 rgb_led_xxx
 */
#define rgb_led (*rgb_led_instance)

/**
 * @brief 单个 RGB 颜色占用字节数
 */
#define RGB_LED_COLOR_BYTES 3u

/**
 * @brief RGB LED 状态码表，使用 X-Macro 定义，方便维护和扩展
 * @param OK 操作成功
 * @param ERROR 通用错误
 * @param INVALID_PARAM 参数无效
 * @param PORT_ERROR PortOps 未提供或底层端口失败
 * @param NO_INSTANCE 未绑定具体 RGB LED 实例
 * @param NOT_INITIALIZE 具体 RGB LED 实例尚未初始化
 * @param BUFFER_TOO_SMALL 调用方提供的缓存容量不足
 * @param INDEX_OUT_OF_RANGE 灯珠索引超出范围
 */
#define RGB_LED_STATUS_TABLE \
    X(OK, 0) \
    X(ERROR, 1) \
    X(INVALID_PARAM, 2) \
    X(PORT_ERROR, 3) \
    X(NO_INSTANCE, 4) \
    X(NOT_INITIALIZE, 5) \
    X(BUFFER_TOO_SMALL, 6) \
    X(INDEX_OUT_OF_RANGE, 7) \
    X(BUSY, 8)

#define X(name, value) RGB_LED_STATUS_##name = value,
/**
 * @brief RGB LED 通用状态码
 */
typedef enum {
    RGB_LED_STATUS_TABLE
} RgbLedStatus;
#undef X

/**
 * @brief RGB 颜色值
 * @param r 红色通道，范围 0~255
 * @param g 绿色通道，范围 0~255
 * @param b 蓝色通道，范围 0~255
 */
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} RgbLedColor;

/**
 * @brief RGB LED 底层端口函数表，由 service init 绑定 platform 或 adapter
 */
typedef struct {
    /**
     * @brief 写出已经编码好的灯带时序数据
     * @param data 发送数据缓冲区
     * @param len 发送数据长度，单位 byte
     * @return true 表示成功，false 表示输出失败
     */
    bool (*write)(const uint8_t* data, uint32_t len);
} RgbLedPortOps;

/**
 * @brief RGB LED 初始化配置
 */
typedef struct {
    /** 底层端口函数表，不能为空 */
    const RgbLedPortOps* ops;
    /** 灯珠数量，单位 pixel */
    uint16_t pixel_count;
    /** 颜色缓存，由调用方提供，容量至少为 pixel_count * RGB_LED_COLOR_BYTES */
    uint8_t* color_buffer;
    /** 颜色缓存容量，单位 byte */
    uint32_t color_buffer_size;
    /** 发送缓存，由调用方提供，容量由具体实例决定 */
    uint8_t* tx_buffer;
    /** 发送缓存容量，单位 byte */
    uint32_t tx_buffer_size;
    /** reset 低电平填充字节数，具体含义由实例决定 */
    uint16_t reset_bytes;
    /** true 表示 write 返回后底层仍可能继续读取 tx_buffer */
    bool async_write;
} RgbLedConfig;

/**
 * @brief RGB LED 统一接口表，不同 RGB LED 实例提供同一组接口
 */
typedef struct {
    /**
     * @brief 初始化具体 RGB LED 实例
     * @param config 初始化配置
     * @return RgbLedStatus 状态码
     */
    RgbLedStatus(*init)(const RgbLedConfig* config);
    /**
     * @brief 状态码转字符串
     * @param status RGB LED 状态码
     * @return const char* 状态码名称
     */
    const char* (*status_str)(RgbLedStatus status);
    /**
     * @brief 设置单个灯珠颜色，只更新缓存，不立即发送
     * @param index 灯珠索引，从 0 开始
     * @param r 红色通道，范围 0~255
     * @param g 绿色通道，范围 0~255
     * @param b 蓝色通道，范围 0~255
     * @return RgbLedStatus 状态码
     */
    RgbLedStatus(*set_rgb)(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
    /**
     * @brief 设置全部灯珠颜色，只更新缓存，不立即发送
     * @param r 红色通道，范围 0~255
     * @param g 绿色通道，范围 0~255
     * @param b 蓝色通道，范围 0~255
     * @return RgbLedStatus 状态码
     */
    RgbLedStatus(*fill)(uint8_t r, uint8_t g, uint8_t b);
    /**
     * @brief 清空全部灯珠颜色，只更新缓存，不立即发送
     * @return RgbLedStatus 状态码
     */
    RgbLedStatus(*clear)(void);
    /**
     * @brief 将颜色缓存编码并发送到真实灯带
     * @return RgbLedStatus 状态码
     */
    RgbLedStatus(*show)(void);
    /**
     * @brief 通知具体实例上一段异步写入已经完成
     * @return RgbLedStatus 状态码
     */
    RgbLedStatus(*write_complete)(void);
    /**
     * @brief 获取单个灯珠当前缓存颜色
     * @param index 灯珠索引，从 0 开始
     * @param out 输出颜色指针
     * @return RgbLedStatus 状态码
     */
    RgbLedStatus(*get_color)(uint16_t index, RgbLedColor* out);
} RgbLedInterface;

/**
 * @brief 当前绑定的 RGB LED 具体实例
 */
extern const RgbLedInterface* rgb_led_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 绑定具体 RGB LED 实例
 * @param instance 具体实例接口表，例如 ws2812_rgb_led_instance
 * @return RgbLedStatus 状态码
 */
RgbLedStatus rgb_led_set_instance(const RgbLedInterface* instance);

/**
 * @brief 初始化当前绑定的 RGB LED 实例
 * @param config 初始化配置
 * @return RgbLedStatus 状态码
 */
RgbLedStatus rgb_led_init(const RgbLedConfig* config);

/**
 * @brief 状态码转字符串
 * @param status RGB LED 状态码
 * @return const char* 状态码名称
 */
const char* rgb_led_status_str(RgbLedStatus status);

/**
 * @brief 设置单个灯珠颜色，只更新缓存，不立即发送
 * @param index 灯珠索引，从 0 开始
 * @param r 红色通道，范围 0~255
 * @param g 绿色通道，范围 0~255
 * @param b 蓝色通道，范围 0~255
 * @return RgbLedStatus 状态码
 */
RgbLedStatus rgb_led_set_rgb(uint16_t index, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 设置全部灯珠颜色，只更新缓存，不立即发送
 * @param r 红色通道，范围 0~255
 * @param g 绿色通道，范围 0~255
 * @param b 蓝色通道，范围 0~255
 * @return RgbLedStatus 状态码
 */
RgbLedStatus rgb_led_fill(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 清空全部灯珠颜色，只更新缓存，不立即发送
 * @return RgbLedStatus 状态码
 */
RgbLedStatus rgb_led_clear(void);

/**
 * @brief 将颜色缓存编码并发送到真实灯带
 * @return RgbLedStatus 状态码
 */
RgbLedStatus rgb_led_show(void);

/**
 * @brief 通知当前实例上一段异步写入已经完成
 * @return RgbLedStatus 状态码
 */
RgbLedStatus rgb_led_write_complete(void);

/**
 * @brief 获取单个灯珠当前缓存颜色
 * @param index 灯珠索引，从 0 开始
 * @param out 输出颜色指针
 * @return RgbLedStatus 状态码
 */
RgbLedStatus rgb_led_get_color(uint16_t index, RgbLedColor* out);

#endif

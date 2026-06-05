# rgb_led SDK 接口文档

## 模块定位

`rgb_led.*` 提供 RGB LED 设备层统一入口，供 service 和 app 调用

`ws2812_rgb_led.*` 是 WS2812 灯带实例，负责 GRB 颜色缓存、bit 编码和底层写入 PortOps

模块目标

- 上层只依赖 `rgb_led.h` 和 `rgb_led.xxx`
- 具体实例负责颜色缓存和时序编码
- platform 只需要实现 `RgbLedPortOps.write`
- `write` 可对接阻塞 SPI 或 SPI DMA

推荐初始化流程

```text
service init
-> 组装 RgbLedPortOps
-> rgb_led_set_instance(&ws2812_rgb_led_instance)
-> ws2812_rgb_led_make_config(...)
-> rgb_led.init(&config)
-> 上层统一调用 rgb_led.xxx 或 rgb_led_xxx
```

## 文件结构

```text
src/device/rgb_led/
├── README.md
├── rgb_led.h
├── rgb_led.c
├── ws2812_rgb_led.h
└── ws2812_rgb_led.c
```

## 统一接口

```c
#define rgb_led (*rgb_led_instance)

extern const RgbLedInterface* rgb_led_instance;

RgbLedStatus rgb_led_set_instance(const RgbLedInterface* instance);
RgbLedStatus rgb_led_init(const RgbLedConfig* config);
RgbLedStatus rgb_led_fill(uint8_t r, uint8_t g, uint8_t b);
RgbLedStatus rgb_led_show(void);
RgbLedStatus rgb_led_write_complete(void);
```

上层不直接依赖具体 RGB LED 实现，只在初始化阶段绑定实例

```c
rgb_led_set_instance(&ws2812_rgb_led_instance);
rgb_led.init(&config);
rgb_led.fill(255, 0, 0);
rgb_led.show();
```

也可以使用函数形式

```c
rgb_led_init(&config);
rgb_led_fill(255, 0, 0);
rgb_led_show();
```

## PortOps

```c
typedef struct {
    bool (*write)(const uint8_t* data, uint32_t len);
} RgbLedPortOps;
```

`PortOps` 由 service init 绑定 platform 或 adapter，RGB LED SDK 不直接依赖 HAL、FSP 或 CubeMX

`write` 接收的是已经由具体实例编码好的时序发送缓存

返回 `true` 表示底层已接受本次发送，返回 `false` 表示底层输出失败

## 阻塞 SPI 最小示例

```c
static bool board_rgb_write(const uint8_t* data, uint32_t len) {
    return HAL_SPI_Transmit(&hspi6, (uint8_t*)data, (uint16_t)len, HAL_MAX_DELAY) == HAL_OK;
}

static const RgbLedPortOps rgb_ops = {
    .write = board_rgb_write,
};

static uint8_t color_buffer[WS2812_RGB_LED_DEFAULT_PIXEL_COUNT * RGB_LED_COLOR_BYTES];
static uint8_t tx_buffer[WS2812_RGB_LED_DEFAULT_PIXEL_COUNT * WS2812_RGB_LED_BITS_PER_PIXEL + WS2812_RGB_LED_DEFAULT_RESET_BYTES];

void app_init(void) {
    RgbLedConfig config;

    rgb_led_set_instance(&ws2812_rgb_led_instance);
    ws2812_rgb_led_make_config(&config, &rgb_ops, color_buffer, sizeof(color_buffer), tx_buffer, sizeof(tx_buffer));
    config.async_write = false;

    rgb_led.init(&config);
    rgb_led.fill(255, 0, 0);
    rgb_led.show();
}
```

## SPI DMA 最小示例

```c
static bool board_rgb_write(const uint8_t* data, uint32_t len) {
    return HAL_SPI_Transmit_DMA(&hspi6, (uint8_t*)data, (uint16_t)len) == HAL_OK;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi) {
    if(hspi == &hspi6) {
        rgb_led_write_complete();
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi) {
    if(hspi == &hspi6) {
        rgb_led_write_complete();
    }
}

void app_init(void) {
    RgbLedConfig config;

    rgb_led_set_instance(&ws2812_rgb_led_instance);
    ws2812_rgb_led_make_config(&config, &rgb_ops, color_buffer, sizeof(color_buffer), tx_buffer, sizeof(tx_buffer));
    config.async_write = true;

    rgb_led.init(&config);
}
```

## 异步模式注意事项

- `async_write` 为 `false` 时，`write` 返回前必须完成对 `data` 的读取
- `async_write` 为 `true` 时，`write` 返回后底层可继续读取 `data`
- `async_write` 为 `true` 时，发送完成或错误后必须调用 `rgb_led_write_complete()`
- WS2812 实例只有一份 `tx_buffer`
- DMA 未完成时再次 `show` 会返回 `RGB_LED_STATUS_BUSY`
- 连续排队多帧刷新需要额外双缓冲或帧队列

## WS2812 缓冲区

- `rgb_color_buffer` 保存每个像素的 RGB 颜色缓存
- `rgb_tx_buffer` 保存 WS2812 编码后的 SPI 发送数据和 reset 低电平数据
- `rgb_led.fill(...)` 只更新颜色缓存
- `rgb_led.show()` 才会编码并发送到真实灯带

## 设计约束

- 上层优先依赖 `rgb_led.h` 统一接口
- 具体实现只负责自己的颜色缓存、时序编码和发送实现
- `ws2812_rgb_led.*` 不直接 include platform 头文件
- platform 对接统一放在 service init 或 adapter 中
- 调用 `rgb_led.xxx` 前需要先 `rgb_led_set_instance(...)`
- 修改颜色后需要调用 `rgb_led.show()` 才会刷新到真实灯带

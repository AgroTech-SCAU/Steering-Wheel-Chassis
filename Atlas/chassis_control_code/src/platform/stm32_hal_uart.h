#ifndef _stm32_hal_uart_h_
#define _stm32_hal_uart_h_

#include "main.h" // IWYU pragma: keep

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 类 型 声 明 ========================= ! //

/**
 * @brief UART 默认超时时间, 单位 ms
 */
#define UART_TIMEOUT 100u

typedef enum {
    PI_TX_HAL_OK = 0,
    PI_TX_HAL_BUSY,
    PI_TX_HAL_TIMEOUT,
    PI_TX_HAL_ERROR
} PiTxHalResult;

/**
 * @brief USART1 句柄
 * @details 同时用于日志发送和 PC 链路接收
 */
extern UART_HandleTypeDef huart1;

/**
 * @brief UART5 句柄
 * @details 当前用于 FS-iA10B i.BUS 接收
 */
extern UART_HandleTypeDef huart5;

/**
 * @brief UART7 句柄
 * @details 当前用于机械臂串口设备
 */
extern UART_HandleTypeDef huart7;

/**
 * @brief USART10 句柄
 * @details 当前用于树莓派链路
 */
extern UART_HandleTypeDef huart10;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 通过 USART1 DMA 发送一段字符串数据
 * @param data 数据缓冲区
 * @param len 数据长度, 单位 byte
 * @return bool `true` 表示发送启动成功
 */
bool uart1_write(const char* data, uint32_t len);

/**
 * @brief 通过 USART1 阻塞发送一段字符串数据
 * @param data 数据缓冲区
 * @param len 数据长度, 单位 byte
 * @return bool `true` 表示发送完成
 */
bool uart1_write_blocking(const char* data, uint32_t len);

/**
 * @brief 通过 UART7 阻塞发送一段字符串数据
 * @param data 数据缓冲区
 * @param len 数据长度, 单位 byte
 * @return bool `true` 表示发送完成
 */
bool uart7_write_blocking(const char* data, uint32_t len);

/**
 * @brief 通过 USART10 阻塞发送一段字符串数据
 * @param data 数据缓冲区
 * @param len 数据长度, 单位 byte
 * @return bool `true` 表示发送完成
 */
bool uart10_write_blocking(const char* data, uint32_t len);
PiTxHalResult uart10_get_last_tx_result(void);

/**
 * @brief 启动 UART 中断接收
 * @param huart UART 句柄
 * @param data 接收缓冲区
 * @param len 接收长度
 * @return bool `true` 表示启动成功
 */
bool uart_receive_it(UART_HandleTypeDef* huart, uint8_t* data, uint16_t len);

/**
 * @brief 启动 UART 空闲线接收 DMA
 * @param huart UART 句柄
 * @param data 接收缓冲区
 * @param len 接收长度
 * @return bool `true` 表示启动成功
 */
bool uart_receive_to_idle_dma(UART_HandleTypeDef* huart, uint8_t* data, uint16_t len);

/**
 * @brief 中止 UART 中断接收
 * @param huart UART 句柄
 * @return bool `true` 表示中止成功
 */
bool uart_abort_receive_it(UART_HandleTypeDef* huart);

/**
 * @brief 中止 UART DMA 接收
 * @param huart UART 句柄
 * @return bool `true` 表示中止成功
 */
bool uart_abort_receive_dma(UART_HandleTypeDef* huart);

/**
 * @brief 注册 UART 发送完成回调
 * @param huart UART 句柄
 * @param callback 回调函数
 */
void uart_register_tx_complete_callback(UART_HandleTypeDef* huart, void (*callback)(void));

/**
 * @brief 注册 UART 接收完成回调
 * @param huart UART 句柄
 * @param callback 回调函数
 */
void uart_register_rx_complete_callback(UART_HandleTypeDef* huart, void (*callback)(void));

/**
 * @brief 注册 UART 接收事件回调
 * @param huart UART 句柄
 * @param callback 回调函数, 参数为本次接收长度
 */
void uart_register_rx_event_callback(UART_HandleTypeDef* huart, void (*callback)(uint16_t size));

/**
 * @brief 注册 UART 错误回调
 * @param huart UART 句柄
 * @param callback 回调函数
 */
void uart_register_error_callback(UART_HandleTypeDef* huart, void (*callback)(void));

#endif

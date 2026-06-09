#ifndef _stm32_hal_can_h_
#define _stm32_hal_can_h_

#include <stdint.h>

#include "main.h" // IWYU pragma: keep

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 底盘转向电机所用 FDCAN1 句柄
 */
extern FDCAN_HandleTypeDef hfdcan1;
/**
 * @brief 底盘驱动电机所用 FDCAN2 句柄
 */
extern FDCAN_HandleTypeDef hfdcan2;

/**
 * @brief STM32 HAL CAN 抽象层状态码表
 */
#define STM32_HAL_CAN_STATUS_TABLE \
    X(OK, "OK") \
    X(INVALID_PARAM, "Invalid Parameter") \
    X(INVALID_DLC, "Invalid DLC") \
    X(FILTER_CONFIG_FAILED, "CAN Filter Config Failed") \
    X(START_FAILED, "CAN Start Failed") \
    X(NOTIFICATION_FAILED, "CAN Notification Activation Failed") \
    X(TX_MAILBOX_TIMEOUT, "CAN TX Mailbox Timeout") \
    X(TX_FAILED, "CAN TX Failed") \
    X(RX_FAILED, "CAN RX Failed") \
    X(NO_CALLBACK_SLOT, "No CAN RX Callback Slot")

/**
 * @brief STM32 HAL CAN 抽象层状态码
 */
#define X(name, str) STM32_HAL_CAN_##name,
typedef enum {
    STM32_HAL_CAN_STATUS_TABLE
} BspCanStatus;
#undef X

/**
 * @brief CAN 接收回调函数类型
 * @param hcan 触发回调的 CAN 句柄
 * @param header 接收帧头
 * @param data 接收数据区，固定为 8 字节缓冲区
 * @param user 注册时绑定的用户指针
 */
typedef void (*STM32HalCanRxCallback)(FDCAN_HandleTypeDef* hcan,
    const FDCAN_RxHeaderTypeDef* header,
    const uint8_t data[8],
    void* user);

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化全局 CAN 过滤器
 * @return BspCanStatus 状态码
 */
BspCanStatus can_filter_init(void);

/**
 * @brief 启动指定 CAN 外设并使能接收中断
 * @param hcan CAN 句柄
 * @return BspCanStatus 状态码
 */
BspCanStatus can_start(FDCAN_HandleTypeDef* hcan);

/**
 * @brief 发送一帧经典 CAN 数据
 * @param hcan CAN 句柄
 * @param id 帧 ID
 * @param data 数据缓冲区
 * @param len 数据长度，最大 8 字节
 * @return BspCanStatus 状态码
 */
BspCanStatus can_send(FDCAN_HandleTypeDef* hcan, uint32_t id, const uint8_t* data, uint8_t len);

/**
 * @brief 注册指定 CAN 句柄的接收回调函数
 * @param hcan CAN 句柄
 * @param callback 回调函数
 * @param user 用户上下文指针
 * @return BspCanStatus 状态码
 */
BspCanStatus can_register_rx_callback(FDCAN_HandleTypeDef* hcan, STM32HalCanRxCallback callback, void* user);

/**
 * @brief 将 CAN 抽象层状态码转换为静态字符串
 * @param status 状态码
 * @return const char* 状态码名称
 */
const char* can_error_code_to_str(BspCanStatus status);

#endif

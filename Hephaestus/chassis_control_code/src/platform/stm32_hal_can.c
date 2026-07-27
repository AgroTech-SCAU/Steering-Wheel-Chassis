#include "stm32_hal_can.h"

#include <stddef.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief CAN 发送等待邮箱空闲的超时时间，单位 ms
 */
/**
 * @brief 可注册的 CAN 接收回调槽位数量
 */
#define CAN_RX_CALLBACK_SLOT_NUM 4u
#define CAN_RX_MAX_FRAMES_PER_IRQ 8u

/**
 * @brief CAN 接收回调注册槽位
 * @param hcan 对应的 CAN 句柄
 * @param callback 已注册的回调函数
 * @param user 用户上下文指针
 */
typedef struct {
    FDCAN_HandleTypeDef* hcan;
    STM32HalCanRxCallback callback;
    void* user;
} CanRxCallbackSlot;

/**
 * @brief CAN 接收回调槽位表
 */
static CanRxCallbackSlot s_rx_slots[CAN_RX_CALLBACK_SLOT_NUM];

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 将字节长度转换为 FDCAN DLC 编码
 * @param len 数据长度
 * @return uint32_t DLC 编码值
 */
static uint32_t can_len_to_dlc(uint8_t len);
/**
 * @brief 配置指定 CAN 句柄的全局过滤器
 * @param hcan CAN 句柄
 * @return BspCanStatus 状态码
 */
static BspCanStatus can_config_global_filter(FDCAN_HandleTypeDef* hcan);
/**
 * @brief 分发一帧接收到的数据到已注册回调
 * @param hcan CAN 句柄
 * @param header 接收帧头
 * @param data 接收数据
 */
static void can_dispatch_rx(FDCAN_HandleTypeDef* hcan, const FDCAN_RxHeaderTypeDef* header, const uint8_t data[8]);
/**
 * @brief 使能 CAN 收发器
 * @param hcan CAN 句柄
 */
static void can_enable_transceiver(FDCAN_HandleTypeDef* hcan);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化全局 CAN 过滤器
 * @return BspCanStatus 状态码
 */
BspCanStatus can_filter_init(void) {
    BspCanStatus status;

    status = can_config_global_filter(&hfdcan1);
    if(status != STM32_HAL_CAN_OK) {
        return status;
    }

    status = can_config_global_filter(&hfdcan2);
    if(status != STM32_HAL_CAN_OK) {
        return status;
    }

    return STM32_HAL_CAN_OK;
}

/**
 * @brief 启动指定 CAN 外设并使能接收中断
 * @param hcan CAN 句柄
 * @return BspCanStatus 状态码
 */
BspCanStatus can_start(FDCAN_HandleTypeDef* hcan) {
    if(hcan == NULL) {
        return STM32_HAL_CAN_INVALID_PARAM;
    }

    if(HAL_FDCAN_ConfigInterruptLines(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0) != HAL_OK) {
        return STM32_HAL_CAN_NOTIFICATION_FAILED;
    }

    if(HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0u) != HAL_OK) {
        return STM32_HAL_CAN_NOTIFICATION_FAILED;
    }

    can_enable_transceiver(hcan);

    if(HAL_FDCAN_Start(hcan) != HAL_OK) {
        return STM32_HAL_CAN_START_FAILED;
    }

    return STM32_HAL_CAN_OK;
}

/**
 * @brief 发送一帧经典 CAN 数据
 * @param hcan CAN 句柄
 * @param id 帧 ID
 * @param data 数据缓冲区
 * @param len 数据长度，最大 8 字节
 * @return BspCanStatus 状态码
 */
BspCanStatus can_send(FDCAN_HandleTypeDef* hcan, uint32_t id, const uint8_t* data, uint8_t len) {
    FDCAN_TxHeaderTypeDef tx_header = { 0 };

    if(hcan == NULL || data == NULL) {
        return STM32_HAL_CAN_INVALID_PARAM;
    }
    if(len > 8u) {
        return STM32_HAL_CAN_INVALID_DLC;
    }

    tx_header.Identifier = id;
    tx_header.IdType = (id <= 0x7FFu) ? FDCAN_STANDARD_ID : FDCAN_EXTENDED_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = can_len_to_dlc(len);
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0u;

    if(HAL_FDCAN_GetTxFifoFreeLevel(hcan) == 0u) {
        uint32_t pending = hcan->Instance->TXBRP;

        if(pending != 0u) {
            (void)HAL_FDCAN_AbortTxRequest(hcan, pending);
        }

        return STM32_HAL_CAN_TX_MAILBOX_TIMEOUT;
    }

    if(HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx_header, (uint8_t*)data) != HAL_OK) {
        return STM32_HAL_CAN_TX_FAILED;
    }

    return STM32_HAL_CAN_OK;
}

/**
 * @brief 注册指定 CAN 句柄的接收回调函数
 * @param hcan CAN 句柄
 * @param callback 回调函数
 * @param user 用户上下文指针
 * @return BspCanStatus 状态码
 */
BspCanStatus can_register_rx_callback(FDCAN_HandleTypeDef* hcan, STM32HalCanRxCallback callback, void* user) {
    uint8_t i;

    if(hcan == NULL || callback == NULL) {
        return STM32_HAL_CAN_INVALID_PARAM;
    }

    for(i = 0u; i < CAN_RX_CALLBACK_SLOT_NUM; ++i) {
        if(s_rx_slots[i].hcan == hcan && s_rx_slots[i].callback == callback) {
            s_rx_slots[i].user = user;
            return STM32_HAL_CAN_OK;
        }
    }

    for(i = 0u; i < CAN_RX_CALLBACK_SLOT_NUM; ++i) {
        if(s_rx_slots[i].callback == NULL) {
            s_rx_slots[i].hcan = hcan;
            s_rx_slots[i].callback = callback;
            s_rx_slots[i].user = user;
            return STM32_HAL_CAN_OK;
        }
    }

    return STM32_HAL_CAN_NO_CALLBACK_SLOT;
}

/**
 * @brief 将 CAN 抽象层状态码转换为静态字符串
 * @param status 状态码
 * @return const char* 状态码名称
 */
#define X(name, str) case STM32_HAL_CAN_##name: return str;
const char* can_error_code_to_str(BspCanStatus status) {
    switch(status) {
        STM32_HAL_CAN_STATUS_TABLE
        default: return "UNKNOWN";
    }
}
#undef X

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief HAL FDCAN FIFO0 接收回调入口
 * @param hfdcan 触发回调的 CAN 句柄
 * @param rx_fifo0_its FIFO0 中断标志
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t rx_fifo0_its) {
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    uint8_t frame_count = 0u;

    if((rx_fifo0_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0u) {
        return;
    }

    while(HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0u
        && frame_count < CAN_RX_MAX_FRAMES_PER_IRQ) {
        memset(rx_data, 0, sizeof(rx_data));
        if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
            return;
        }
        can_dispatch_rx(hfdcan, &rx_header, rx_data);
        ++frame_count;
    }
}

/**
 * @brief 将字节长度转换为 FDCAN DLC 编码
 * @param len 数据长度
 * @return uint32_t DLC 编码值
 */
static uint32_t can_len_to_dlc(uint8_t len) {
    static const uint32_t dlc_table[9] = {
        FDCAN_DLC_BYTES_0,
        FDCAN_DLC_BYTES_1,
        FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3,
        FDCAN_DLC_BYTES_4,
        FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6,
        FDCAN_DLC_BYTES_7,
        FDCAN_DLC_BYTES_8
    };

    return dlc_table[len];
}

/**
 * @brief 配置指定 CAN 句柄的全局过滤器
 * @param hcan CAN 句柄
 * @return BspCanStatus 状态码
 */
static BspCanStatus can_config_global_filter(FDCAN_HandleTypeDef* hcan) {
    if(hcan == NULL) {
        return STM32_HAL_CAN_INVALID_PARAM;
    }

    if(HAL_FDCAN_ConfigGlobalFilter(hcan,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE) != HAL_OK) {
        return STM32_HAL_CAN_FILTER_CONFIG_FAILED;
    }

    return STM32_HAL_CAN_OK;
}

/**
 * @brief 分发一帧接收到的数据到已注册回调
 * @param hcan CAN 句柄
 * @param header 接收帧头
 * @param data 接收数据
 */
static void can_dispatch_rx(FDCAN_HandleTypeDef* hcan, const FDCAN_RxHeaderTypeDef* header, const uint8_t data[8]) {
    uint8_t i;

    for(i = 0u; i < CAN_RX_CALLBACK_SLOT_NUM; ++i) {
        if(s_rx_slots[i].hcan == hcan && s_rx_slots[i].callback != NULL) {
            s_rx_slots[i].callback(hcan, header, data, s_rx_slots[i].user);
        }
    }
}

/**
 * @brief 使能 CAN 收发器
 * @param hcan CAN 句柄
 */
static void can_enable_transceiver(FDCAN_HandleTypeDef* hcan) {
    if(hcan == NULL) {
        return;
    }

    if(hcan->Instance == FDCAN1) {
        HAL_GPIO_WritePin(CAN1_EN_GPIO_Port, CAN1_EN_Pin, GPIO_PIN_SET);
    }
    else if(hcan->Instance == FDCAN2) {
        HAL_GPIO_WritePin(CAN2_EN_GPIO_Port, CAN2_EN_Pin, GPIO_PIN_SET);
    }
}

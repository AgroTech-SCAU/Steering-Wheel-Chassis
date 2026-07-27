#include "stm32_hal_uart.h"

// ! ========================= 变 量 声 明 ========================= ! //

static PiTxHalResult s_uart10_last_tx_result = PI_TX_HAL_OK;

static void (*uart1_tx_complete_callback)(void) = NULL;
static void (*uart1_rx_complete_callback)(void) = NULL;
static void (*uart1_rx_event_callback)(uint16_t size) = NULL;
static void (*uart1_error_callback)(void) = NULL;

static void (*uart5_rx_complete_callback)(void) = NULL;
static void (*uart5_rx_event_callback)(uint16_t size) = NULL;
static void (*uart5_error_callback)(void) = NULL;

static void (*uart10_rx_complete_callback)(void) = NULL;
static void (*uart10_rx_event_callback)(uint16_t size) = NULL;
static void (*uart10_error_callback)(void) = NULL;

// ! ========================= 接 口 函 数 实 现 ========================= ! //

bool uart1_write(const char* data, uint32_t len) {
    if(data == NULL || len == 0u || len > UINT16_MAX) {
        return false;
    }

    return HAL_UART_Transmit_DMA(&huart1, (uint8_t*)data, (uint16_t)len) == HAL_OK;
}

bool uart1_write_blocking(const char* data, uint32_t len) {
    if(data == NULL || len == 0u || len > UINT16_MAX) {
        return false;
    }

    return HAL_UART_Transmit(&huart1, (uint8_t*)data, (uint16_t)len, 10) == HAL_OK;
}

bool uart7_write_blocking(const char* data, uint32_t len) {
    if(data == NULL || len == 0u || len > UINT16_MAX) {
        return false;
    }

    return HAL_UART_Transmit(&huart7, (uint8_t*)data, (uint16_t)len, 10) == HAL_OK;
}

bool uart10_write_blocking(const char* data, uint32_t len) {
    HAL_StatusTypeDef status;

    if(data == NULL || len == 0u || len > UINT16_MAX) {
        s_uart10_last_tx_result = PI_TX_HAL_ERROR;
        return false;
    }

    status = HAL_UART_Transmit(&huart10, (uint8_t*)data, (uint16_t)len, 10);
    switch(status) {
        case HAL_OK:
            s_uart10_last_tx_result = PI_TX_HAL_OK;
            return true;

        case HAL_BUSY:
            s_uart10_last_tx_result = PI_TX_HAL_BUSY;
            return false;

        case HAL_TIMEOUT:
            s_uart10_last_tx_result = PI_TX_HAL_TIMEOUT;
            return false;

        case HAL_ERROR:
        default:
            s_uart10_last_tx_result = PI_TX_HAL_ERROR;
            return false;
    }
}

PiTxHalResult uart10_get_last_tx_result(void) {
    return s_uart10_last_tx_result;
}

bool uart_receive_it(UART_HandleTypeDef* huart, uint8_t* data, uint16_t len) {
    if(huart == NULL || data == NULL || len == 0u) {
        return false;
    }

    return HAL_UART_Receive_IT(huart, data, len) == HAL_OK;
}

bool uart_receive_to_idle_dma(UART_HandleTypeDef* huart, uint8_t* data, uint16_t len) {
    HAL_StatusTypeDef status;

    if(huart == NULL || data == NULL || len == 0u) {
        return false;
    }

    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    status = HAL_UARTEx_ReceiveToIdle_DMA(huart, data, len);
    if(status != HAL_OK) {
        return false;
    }

    if(huart->hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }

    return true;
}

bool uart_abort_receive_it(UART_HandleTypeDef* huart) {
    if(huart == NULL) {
        return false;
    }

    return HAL_UART_AbortReceive_IT(huart) == HAL_OK;
}

bool uart_abort_receive_dma(UART_HandleTypeDef* huart) {
    if(huart == NULL) {
        return false;
    }

    return HAL_UART_AbortReceive(huart) == HAL_OK;
}

void uart_register_tx_complete_callback(UART_HandleTypeDef* huart, void (*callback)(void)) {
    if(huart == &huart1) {
        uart1_tx_complete_callback = callback;
    }
}

void uart_register_rx_complete_callback(UART_HandleTypeDef* huart, void (*callback)(void)) {
    if(huart == &huart1) {
        uart1_rx_complete_callback = callback;
    }
    else if(huart == &huart5) {
        uart5_rx_complete_callback = callback;
    }
    else if(huart == &huart10) {
        uart10_rx_complete_callback = callback;
    }
}

void uart_register_rx_event_callback(UART_HandleTypeDef* huart, void (*callback)(uint16_t size)) {
    if(huart == &huart1) {
        uart1_rx_event_callback = callback;
    }
    else if(huart == &huart5) {
        uart5_rx_event_callback = callback;
    }
    else if(huart == &huart10) {
        uart10_rx_event_callback = callback;
    }
}

void uart_register_error_callback(UART_HandleTypeDef* huart, void (*callback)(void)) {
    if(huart == &huart1) {
        uart1_error_callback = callback;
    }
    else if(huart == &huart5) {
        uart5_error_callback = callback;
    }
    else if(huart == &huart10) {
        uart10_error_callback = callback;
    }
}

// ! ========================= HAL 回 调 实 现 ========================= ! //

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    if(huart == &huart1 && uart1_tx_complete_callback != NULL) {
        uart1_tx_complete_callback();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if(huart == &huart1 && uart1_rx_complete_callback != NULL) {
        uart1_rx_complete_callback();
    }
    else if(huart == &huart5 && uart5_rx_complete_callback != NULL) {
        uart5_rx_complete_callback();
    }
    else if(huart == &huart10 && uart10_rx_complete_callback != NULL) {
        uart10_rx_complete_callback();
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    if(huart == &huart1 && uart1_rx_event_callback != NULL) {
        uart1_rx_event_callback(Size);
    }
    else if(huart == &huart5 && uart5_rx_event_callback != NULL) {
        uart5_rx_event_callback(Size);
    }
    else if(huart == &huart10 && uart10_rx_event_callback != NULL) {
        uart10_rx_event_callback(Size);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    if(huart == &huart1 && uart1_error_callback != NULL) {
        uart1_error_callback();
    }
    else if(huart == &huart5 && uart5_error_callback != NULL) {
        uart5_error_callback();
    }
    else if(huart == &huart10 && uart10_error_callback != NULL) {
        uart10_error_callback();
    }
}

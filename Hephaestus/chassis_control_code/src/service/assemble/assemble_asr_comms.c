#include "assemble.h"

#include "asr_comms.h"
#include "log.h"
#include "stm32_hal_uart.h"

// ! ========================= 变 量 声 明 ========================= ! //

static uint8_t s_asr_comms_rx_byte = 0u;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static bool assemble_asr_comms_write(const char* data, uint32_t len);
static void assemble_asr_comms_on_rx_complete(void);
static void assemble_asr_comms_on_error(void);
static void assemble_asr_comms_start_receive(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_asr_comms(void) {
    AsrCommsConfig config;

    config.port_ops.write = assemble_asr_comms_write;
    if(asr_comms_init(&config) != ASR_COMMS_STATUS_OK) {
        return SYSTEM_STATUS_ERROR;
    }

    uart_register_rx_complete_callback(&huart10, assemble_asr_comms_on_rx_complete);
    uart_register_error_callback(&huart10, assemble_asr_comms_on_error);
    assemble_asr_comms_start_receive();
    log_info("ASR_COMMS init done");
    return SYSTEM_STATUS_OK;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static bool assemble_asr_comms_write(const char* data, uint32_t len) {
    return uart10_write_blocking(data, len);
}

static void assemble_asr_comms_on_rx_complete(void) {
    asr_comms_on_rx_byte(s_asr_comms_rx_byte);
    assemble_asr_comms_start_receive();
}

static void assemble_asr_comms_on_error(void) {
    assemble_asr_comms_start_receive();
}

static void assemble_asr_comms_start_receive(void) {
    (void)uart_receive_it(&huart10, &s_asr_comms_rx_byte, 1u);
}

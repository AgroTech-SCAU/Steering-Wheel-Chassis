#include "assemble.h"

#include "delay.h"
#include "log.h"
#include "pi_comms.h"
#include "stm32_hal_uart.h"

// ! ========================= 变 量 声 明 ========================= ! //

static uint8_t s_pi_comms_rx_byte = 0u;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static bool assemble_pi_comms_write(const char* data, uint32_t len);
static int assemble_pi_comms_last_tx_result(void);
static uint32_t assemble_pi_comms_now_ms(void);
static void assemble_pi_comms_on_rx_complete(void);
static void assemble_pi_comms_on_error(void);
static void assemble_pi_comms_start_receive(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_pi_comms(void) {
    PiCommsConfig config;

    config.port_ops.write = assemble_pi_comms_write;
    config.port_ops.get_last_tx_result = assemble_pi_comms_last_tx_result;
    config.port_ops.now_ms = assemble_pi_comms_now_ms;
    if(pi_comms_init(&config) != PI_COMMS_STATUS_OK) {
        return SYSTEM_STATUS_ERROR;
    }

    uart_register_rx_complete_callback(&huart10, assemble_pi_comms_on_rx_complete);
    uart_register_error_callback(&huart10, assemble_pi_comms_on_error);
    assemble_pi_comms_start_receive();
    log_info("PI_COMMS init done");
    return SYSTEM_STATUS_OK;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static bool assemble_pi_comms_write(const char* data, uint32_t len) {
    return uart10_write_blocking(data, len);
}

static int assemble_pi_comms_last_tx_result(void) {
    return (int)uart10_get_last_tx_result();
}

static uint32_t assemble_pi_comms_now_ms(void) {
    return delay_now_ms();
}

static void assemble_pi_comms_on_rx_complete(void) {
    pi_comms_on_rx_byte(s_pi_comms_rx_byte);
    assemble_pi_comms_start_receive();
}

static void assemble_pi_comms_on_error(void) {
    assemble_pi_comms_start_receive();
}

static void assemble_pi_comms_start_receive(void) {
    (void)uart_receive_it(&huart10, &s_pi_comms_rx_byte, 1u);
}

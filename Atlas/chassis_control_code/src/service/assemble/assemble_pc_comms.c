#include "assemble.h"

#include "delay.h"
#include "log.h"
#include "pc_comms.h"
#include "stm32_hal_uart.h"

// ! ========================= 变 量 声 明 ========================= ! //

static uint8_t s_pc_comms_rx_byte = 0u;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static uint32_t assemble_pc_comms_now_ms(void);
static void assemble_pc_comms_on_rx_complete(void);
static void assemble_pc_comms_on_error(void);
static void assemble_pc_comms_start_receive(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_pc_comms(void) {
    PcCommsConfig config;

    config.port_ops.now_ms = assemble_pc_comms_now_ms;
    if(pc_comms_init(&config) != PC_COMMS_STATUS_OK) {
        return SYSTEM_STATUS_ERROR;
    }

    uart_register_rx_complete_callback(&huart1, assemble_pc_comms_on_rx_complete);
    uart_register_error_callback(&huart1, assemble_pc_comms_on_error);
    assemble_pc_comms_start_receive();
    log_info("PC_COMMS init done");
    return SYSTEM_STATUS_OK;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static uint32_t assemble_pc_comms_now_ms(void) {
    return delay_now_ms();
}

static void assemble_pc_comms_on_rx_complete(void) {
    pc_comms_on_rx_byte(s_pc_comms_rx_byte);
    assemble_pc_comms_start_receive();
}

static void assemble_pc_comms_on_error(void) {
    assemble_pc_comms_start_receive();
}

static void assemble_pc_comms_start_receive(void) {
    (void)uart_receive_it(&huart1, &s_pc_comms_rx_byte, 1u);
}

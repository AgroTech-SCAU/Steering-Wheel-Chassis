/**
 * @file asr_comms.c
 * @brief ASRPro 最小双向 ASCII 通信服务实现
 */

#include "asr_comms.h"

#include <string.h>

// ! ========================= 宏 定 义 声 明 ========================= ! //

#define ASR_COMMS_RX_RING_SIZE 128u
#define ASR_COMMS_RX_RING_MASK (ASR_COMMS_RX_RING_SIZE - 1u)
#define ASR_COMMS_RX_LINE_MAX 32u

// ! ========================= 变 量 声 明 ========================= ! //

static AsrCommsConfig s_asr_comms_config = { 0 };
static uint8_t s_asr_comms_rx_ring[ASR_COMMS_RX_RING_SIZE] = { 0 };
static volatile uint8_t s_asr_comms_rx_head = 0u;
static volatile uint8_t s_asr_comms_rx_tail = 0u;
static volatile bool s_asr_comms_rx_overflowed = false;
static char s_asr_comms_rx_line[ASR_COMMS_RX_LINE_MAX] = { 0 };
static uint8_t s_asr_comms_rx_line_len = 0u;
static bool s_asr_comms_rx_discard_until_newline = false;
static bool s_asr_comms_auto_start_event_pending = false;
static bool s_asr_comms_initialized = false;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void asr_comms_process_rx_byte(uint8_t data);
static void asr_comms_process_line(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

AsrCommsStatus asr_comms_init(const AsrCommsConfig* config) {
    if(config == NULL) {
        return ASR_COMMS_STATUS_INVALID_PARAM;
    }

    if(config->port_ops.write == NULL) {
        return ASR_COMMS_STATUS_DEPENDENCY_MISSING;
    }

    s_asr_comms_config = *config;
    memset(s_asr_comms_rx_ring, 0, sizeof(s_asr_comms_rx_ring));
    memset(s_asr_comms_rx_line, 0, sizeof(s_asr_comms_rx_line));
    s_asr_comms_rx_head = 0u;
    s_asr_comms_rx_tail = 0u;
    s_asr_comms_rx_overflowed = false;
    s_asr_comms_rx_line_len = 0u;
    s_asr_comms_rx_discard_until_newline = false;
    s_asr_comms_auto_start_event_pending = false;
    s_asr_comms_initialized = true;
    return ASR_COMMS_STATUS_OK;
}

void asr_comms_on_rx_byte(uint8_t data) {
    const uint8_t head = s_asr_comms_rx_head;
    const uint8_t next_head = (uint8_t)((head + 1u) & ASR_COMMS_RX_RING_MASK);

    if(!s_asr_comms_initialized) {
        return;
    }

    if(next_head == s_asr_comms_rx_tail) {
        s_asr_comms_rx_overflowed = true;
        return;
    }

    s_asr_comms_rx_ring[head] = data;
    s_asr_comms_rx_head = next_head;
}

void asr_comms_process(void) {
    if(s_asr_comms_rx_overflowed) {
        s_asr_comms_rx_overflowed = false;
        s_asr_comms_rx_line_len = 0u;
        s_asr_comms_rx_line[0] = '\0';
        s_asr_comms_rx_discard_until_newline = true;
    }

    while(s_asr_comms_rx_tail != s_asr_comms_rx_head) {
        const uint8_t tail = s_asr_comms_rx_tail;
        const uint8_t data = s_asr_comms_rx_ring[tail];

        s_asr_comms_rx_tail = (uint8_t)((tail + 1u) & ASR_COMMS_RX_RING_MASK);
        asr_comms_process_rx_byte(data);
    }
}

bool asr_comms_speak(uint16_t phrase_id) {
    const char* command = NULL;
    uint32_t command_len = 0u;

    if(!s_asr_comms_initialized || s_asr_comms_config.port_ops.write == NULL) {
        return false;
    }

    switch(phrase_id) {
        case ASR_COMMS_PHRASE_VOICE_GATE:
            command = "SPK,1\n";
            command_len = 6u;
            break;

        case ASR_COMMS_PHRASE_AUTONOMOUS_START:
            command = "SPK,2\n";
            command_len = 6u;
            break;

        case ASR_COMMS_PHRASE_DELIVERY_COMPLETE:
            command = "SPK,4\n";
            command_len = 6u;
            break;

        case ASR_COMMS_PHRASE_TASK_COMPLETE:
            command = "SPK,5\n";
            command_len = 6u;
            break;

        case ASR_COMMS_PHRASE_STAGE_SKIPPED:
            command = "SPK,6\n";
            command_len = 6u;
            break;

        default:
            return false;
    }

    return s_asr_comms_config.port_ops.write(command, command_len);
}

bool asr_comms_take_auto_start_event(void) {
    if(!s_asr_comms_auto_start_event_pending) {
        return false;
    }

    s_asr_comms_auto_start_event_pending = false;
    return true;
}

void asr_comms_clear_pending_auto_start_event(void) {
    s_asr_comms_auto_start_event_pending = false;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static void asr_comms_process_rx_byte(uint8_t data) {
    if(data == (uint8_t)'\n') {
        if(!s_asr_comms_rx_discard_until_newline) {
            asr_comms_process_line();
        }

        s_asr_comms_rx_line_len = 0u;
        s_asr_comms_rx_line[0] = '\0';
        s_asr_comms_rx_discard_until_newline = false;
        return;
    }

    if(s_asr_comms_rx_discard_until_newline) {
        return;
    }

    if(s_asr_comms_rx_line_len < ASR_COMMS_RX_LINE_MAX - 1u) {
        s_asr_comms_rx_line[s_asr_comms_rx_line_len++] = (char)data;
        return;
    }

    s_asr_comms_rx_line_len = 0u;
    s_asr_comms_rx_line[0] = '\0';
    s_asr_comms_rx_discard_until_newline = true;
}

static void asr_comms_process_line(void) {
    while(s_asr_comms_rx_line_len > 0u &&
          s_asr_comms_rx_line[s_asr_comms_rx_line_len - 1u] == '\r') {
        s_asr_comms_rx_line_len--;
    }

    s_asr_comms_rx_line[s_asr_comms_rx_line_len] = '\0';
    if(strcmp(s_asr_comms_rx_line, "EVT,AUTO_START") == 0) {
        s_asr_comms_auto_start_event_pending = true;
    }
}

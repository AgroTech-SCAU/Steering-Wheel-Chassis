#ifndef _service_asr_comms_h_
#define _service_asr_comms_h_

/**
 * @file asr_comms.h
 * @brief ASRPro 通信服务接口
 */

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 类 型 声 明 ========================= ! //

typedef enum {
    ASR_COMMS_STATUS_OK = 0,
    ASR_COMMS_STATUS_INVALID_PARAM,
    ASR_COMMS_STATUS_DEPENDENCY_MISSING
} AsrCommsStatus;

typedef struct {
    bool (*write)(const char* data, uint32_t len);
} AsrCommsPortOps;

typedef struct {
    AsrCommsPortOps port_ops;
} AsrCommsConfig;

typedef enum {
    ASR_COMMS_PHRASE_VOICE_GATE = 1,
    ASR_COMMS_PHRASE_AUTONOMOUS_START = 2,
    ASR_COMMS_PHRASE_DELIVERY_COMPLETE = 4,
    ASR_COMMS_PHRASE_TASK_COMPLETE = 5,
    ASR_COMMS_PHRASE_STAGE_SKIPPED = 6
} AsrCommsPhrase;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

AsrCommsStatus asr_comms_init(const AsrCommsConfig* config);
void asr_comms_on_rx_byte(uint8_t data);
void asr_comms_process(void);
bool asr_comms_speak(uint16_t phrase_id);
bool asr_comms_take_auto_start_event(void);
void asr_comms_clear_pending_auto_start_event(void);

#endif

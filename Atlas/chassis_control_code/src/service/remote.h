#ifndef _service_remote_h_
#define _service_remote_h_

/**
 * @file remote.h
 * @brief FS-iA10B remote input service interface
 */

#include "fs_ia10b.h"

#include <stdbool.h>
#include <stdint.h>

#define REMOTE_CH_RIGHT_X 0u
#define REMOTE_CH_RIGHT_Y 1u
#define REMOTE_CH_LEFT_X 3u
#define REMOTE_CH_SWA 4u
#define REMOTE_CH_SWB 5u
#define REMOTE_CH_SWC 6u
#define REMOTE_CH_SWD 7u
#define REMOTE_CH_VRA 8u
#define REMOTE_CH_VRB 9u

#define REMOTE_CENTER 1500u
#define REMOTE_SPAN 500.0f
#define REMOTE_DEADBAND 10u

#define REMOTE_SW_LOW 2000u
#define REMOTE_SW_CENTER 1500u
#define REMOTE_SW_HIGH 1000u

#define REMOTE_AUTO_THRESHOLD 1800u
#define REMOTE_VR_LOW_THRESHOLD 1200u

typedef enum {
    REMOTE_MANUAL_SOURCE_NONE = 0,
    REMOTE_MANUAL_SOURCE_CHASSIS,
    REMOTE_MANUAL_SOURCE_ARM
} RemoteManualSource;

typedef struct {
    bool online;
    bool manual_request;
    bool auto_request;
    RemoteManualSource manual_source;
    FsIa10bData rc_data;
    uint32_t stamp_ms;
} RemoteState;

void remote_init(void);

void remote_process(void);

bool remote_get_state(RemoteState* out);

bool remote_is_online(uint32_t timeout_ms);

bool remote_is_manual_requested(void);

bool remote_is_auto_requested(void);

bool remote_take_auto_start_event(void);

bool remote_take_clear_reset_event(void);

void remote_clear_pending_auto_start_event(void);

RemoteManualSource remote_get_manual_source(void);

const FsIa10bData* remote_get_raw_data(void);

#endif

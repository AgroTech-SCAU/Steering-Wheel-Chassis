/**
 * @file remote.c
 * @brief FS-iA10B remote input service implementation
 */

#include "remote.h"

#include "delay.h"

#include <string.h>

#define REMOTE_TIMEOUT_MS 100u

static RemoteState s_remote_state = { 0 };
static bool s_auto_start_event_pending = false;
static bool s_clear_reset_event_pending = false;
static bool s_auto_start_edge_armed = false;
static bool s_clear_reset_edge_armed = false;
static bool s_auto_condition_active = false;
static bool s_clear_condition_active = false;

void remote_init(void) {
    memset(&s_remote_state, 0, sizeof(s_remote_state));
    s_auto_start_event_pending = false;
    s_clear_reset_event_pending = false;
    s_auto_start_edge_armed = false;
    s_clear_reset_edge_armed = false;
    s_auto_condition_active = false;
    s_clear_condition_active = false;
}

void remote_process(void) {
    FsIa10bData rc_data;
    bool auto_condition;
    bool clear_condition;

    ibus_maintain();

    if(!ibus_get_data(&rc_data) || !ibus_is_online(REMOTE_TIMEOUT_MS)) {
        s_remote_state.online = false;
        s_remote_state.manual_request = false;
        s_remote_state.auto_request = false;
        s_remote_state.manual_source = REMOTE_MANUAL_SOURCE_NONE;
        s_auto_start_event_pending = false;
        s_clear_reset_event_pending = false;
        s_auto_start_edge_armed = false;
        s_clear_reset_edge_armed = false;
        s_auto_condition_active = false;
        s_clear_condition_active = false;
        return;
    }

    s_remote_state.online = true;
    s_remote_state.rc_data = rc_data;
    s_remote_state.stamp_ms = delay_now_ms();
    s_remote_state.manual_request = (rc_data.channel[REMOTE_CH_SWD] == REMOTE_SW_LOW);
    s_remote_state.auto_request = (rc_data.channel[REMOTE_CH_SWD] == REMOTE_SW_HIGH &&
                                   rc_data.channel[REMOTE_CH_VRA] >= REMOTE_AUTO_THRESHOLD &&
                                   rc_data.channel[REMOTE_CH_VRB] >= REMOTE_AUTO_THRESHOLD);
    auto_condition = s_remote_state.auto_request;
    clear_condition = (rc_data.channel[REMOTE_CH_SWC] == REMOTE_SW_HIGH &&
                       rc_data.channel[REMOTE_CH_VRA] <= REMOTE_VR_LOW_THRESHOLD &&
                       rc_data.channel[REMOTE_CH_VRB] <= REMOTE_VR_LOW_THRESHOLD);

    if(!auto_condition) {
        s_auto_condition_active = false;
        s_auto_start_edge_armed = true;
    }
    else {
        if(s_auto_start_edge_armed && !s_auto_condition_active) {
            s_auto_start_event_pending = true;
        }
        s_auto_condition_active = true;
    }

    if(!clear_condition) {
        s_clear_condition_active = false;
        s_clear_reset_edge_armed = true;
    }
    else {
        if(s_clear_reset_edge_armed && !s_clear_condition_active) {
            s_clear_reset_event_pending = true;
        }
        s_clear_condition_active = true;
    }

    if(rc_data.channel[REMOTE_CH_SWA] == REMOTE_SW_HIGH) {
        s_remote_state.manual_source = REMOTE_MANUAL_SOURCE_CHASSIS;
    }
    else if(rc_data.channel[REMOTE_CH_SWA] == REMOTE_SW_LOW) {
        s_remote_state.manual_source = REMOTE_MANUAL_SOURCE_ARM;
    }
    else {
        s_remote_state.manual_source = REMOTE_MANUAL_SOURCE_NONE;
    }
}

bool remote_get_state(RemoteState* out) {
    if(out == NULL) {
        return false;
    }

    *out = s_remote_state;
    return true;
}

bool remote_is_online(uint32_t timeout_ms) {
    if(!s_remote_state.online) {
        return false;
    }

    return (delay_now_ms() - s_remote_state.stamp_ms) <= timeout_ms;
}

bool remote_is_manual_requested(void) {
    return s_remote_state.online && s_remote_state.manual_request;
}

bool remote_is_auto_requested(void) {
    return s_remote_state.online && s_remote_state.auto_request;
}

bool remote_take_auto_start_event(void) {
    if(!s_auto_start_event_pending) {
        return false;
    }

    s_auto_start_event_pending = false;
    return true;
}

bool remote_take_clear_reset_event(void) {
    if(!s_clear_reset_event_pending) {
        return false;
    }

    s_clear_reset_event_pending = false;
    return true;
}

void remote_clear_pending_auto_start_event(void) {
    s_auto_start_event_pending = false;
}

RemoteManualSource remote_get_manual_source(void) {
    if(!s_remote_state.online) {
        return REMOTE_MANUAL_SOURCE_NONE;
    }

    return s_remote_state.manual_source;
}

const FsIa10bData* remote_get_raw_data(void) {
    if(!s_remote_state.online) {
        return NULL;
    }

    return &s_remote_state.rc_data;
}

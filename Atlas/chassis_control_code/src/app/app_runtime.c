/**
 * @file app_runtime.c
 * @brief MCU 应用运行时主控实现
 */

#include "app_runtime.h"

#include "app_control.h"
#include "app_fsm.h"
#include "arm.h"
#include "chassis.h"
#include "chassis_yaw_hold.h"
#include "delay.h"
#include "log.h"
#include "odom.h"
#include "pc_comms.h"
#include "pi_comms.h"
#include "remote.h"

#include <string.h>

// ! ========================= 宏 定 义 声 明 ========================= ! //

#define APP_RUNTIME_CHASSIS_NOT_READY_FAULT_CODE 1
#define APP_RUNTIME_ARM_NOT_READY_FAULT_CODE 1
#define APP_RUNTIME_ODOM_NOT_READY_FAULT_CODE 1
#define APP_RUNTIME_PI_TIMEOUT_FAULT_CODE 1
#define APP_RUNTIME_CONTROL_EXEC_FAULT_CODE 2
#define APP_RUNTIME_RESULT_LOG_PERIOD_MS 1000u
#define APP_RUNTIME_EVENT_LOG_PERIOD_MS 1000u

// ! ========================= 变 量 声 明 ========================= ! //

static RemoteState s_remote_state = { 0 };
static ms_t s_command_invalid_log_timer = 0u;
static ms_t s_unsupported_log_timer = 0u;
static ms_t s_auto_start_accept_log_timer = 0u;
static ms_t s_auto_start_already_latched_log_timer = 0u;
static ms_t s_auto_start_invalid_state_log_timer = 0u;
static ms_t s_auto_start_pi_offline_log_timer = 0u;
static ms_t s_auto_start_chassis_not_ready_log_timer = 0u;
static ms_t s_auto_start_odom_not_ready_log_timer = 0u;
static ms_t s_auto_start_fault_log_timer = 0u;
static ms_t s_auto_start_estop_log_timer = 0u;
static ms_t s_remote_reset_log_timer = 0u;
static bool s_auto_start_latched = false;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void app_runtime_update_inputs(void);
static void app_runtime_update_mode(void);
static bool app_runtime_apply_safety(void);
static void app_runtime_apply_control(void);
static void app_runtime_leave_manual_chassis_pc_arm(void);
static void app_runtime_leave_auto_pi(void);
static void app_runtime_raise_fault_once(AppFaultSource source, AppFaultLevel level, int32_t code);
static bool app_runtime_pi_arm_cmd_pending(void);
static bool app_runtime_try_accept_auto_start_event(void);
static bool app_runtime_can_accept_auto_start(void);
static void app_runtime_set_auto_start_latched(bool latched);
static void app_runtime_handle_remote_clear_reset(void);
static void app_runtime_reset_auto_task_context(void);
static void app_runtime_finish_reset_transition(void);
static void app_runtime_handle_control_result(AppControlResult result);
static void app_runtime_handle_pi_mission_event(const PiCommsMissionEvent* event);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void app_runtime_init(void) {
    memset(&s_remote_state, 0, sizeof(s_remote_state));
    s_command_invalid_log_timer = 0u;
    s_unsupported_log_timer = 0u;
    s_auto_start_accept_log_timer = 0u;
    s_auto_start_already_latched_log_timer = 0u;
    s_auto_start_invalid_state_log_timer = 0u;
    s_auto_start_pi_offline_log_timer = 0u;
    s_auto_start_chassis_not_ready_log_timer = 0u;
    s_auto_start_odom_not_ready_log_timer = 0u;
    s_auto_start_fault_log_timer = 0u;
    s_auto_start_estop_log_timer = 0u;
    s_remote_reset_log_timer = 0u;
    s_auto_start_latched = false;

    app_control_init();
    app_fsm_init();
}

void app_runtime_process(void) {
    app_runtime_update_inputs();
    app_runtime_update_mode();
    app_fsm_process();

    if(!app_runtime_apply_safety()) {
        return;
    }

    app_runtime_apply_control();
}

AppFsmStateId app_runtime_get_state(void) {
    return app_fsm_get_state();
}

AppManualMode app_runtime_get_manual_mode(void) {
    return app_fsm_get_manual_mode();
}

const AppFault* app_runtime_get_fault(void) {
    return app_fsm_get_fault();
}

bool app_runtime_has_fault(void) {
    return app_fsm_has_fault();
}

bool app_runtime_is_auto_start_latched(void) {
    return s_auto_start_latched;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static void app_runtime_update_inputs(void) {
    (void)remote_get_state(&s_remote_state);
}

static void app_runtime_update_mode(void) {
    PiCommsMissionEvent mission_event;
    PiCommsEstopEvent estop_event;

    if(pi_comms_take_estop(&estop_event)) {
        log_warn("APP_RUNTIME estop event requested by Pi: reason=%u", estop_event.reason);
        (void)app_fsm_request_estop();
        return;
    }

    if(remote_take_clear_reset_event()) {
        app_runtime_handle_remote_clear_reset();
        return;
    }

    if(pi_comms_take_mission_event(&mission_event)) {
        app_runtime_handle_pi_mission_event(&mission_event);
        return;
    }

    if(app_fsm_get_state() == APP_FSM_STATE_FAULT || app_fsm_get_state() == APP_FSM_STATE_ESTOP) {
        return;
    }

    if(remote_is_manual_requested()) {
        const AppManualMode next_manual_mode = remote_get_manual_source() == REMOTE_MANUAL_SOURCE_ARM
                                                   ? APP_MANUAL_MODE_ARM_FS
                                                   : APP_MANUAL_MODE_CHASSIS_PC_ARM;

        if(app_fsm_get_state() == APP_FSM_STATE_AUTO_PI) {
            app_runtime_leave_auto_pi();
        }
        if(app_fsm_get_state() == APP_FSM_STATE_MANUAL && app_fsm_get_manual_mode() == APP_MANUAL_MODE_CHASSIS_PC_ARM &&
           next_manual_mode != APP_MANUAL_MODE_CHASSIS_PC_ARM) {
            app_runtime_leave_manual_chassis_pc_arm();
        }

        app_fsm_set_manual_mode(next_manual_mode);
        (void)app_fsm_post(APP_FSM_EVENT_SWITCH_TO_MANUAL);
        return;
    }

    if(remote_take_auto_start_event()) {
        (void)app_runtime_try_accept_auto_start_event();
        return;
    }

    if(app_fsm_get_state() == APP_FSM_STATE_MANUAL && !s_remote_state.manual_request) {
        if(app_fsm_get_manual_mode() == APP_MANUAL_MODE_CHASSIS_PC_ARM) {
            app_runtime_leave_manual_chassis_pc_arm();
        }
        (void)app_fsm_post(APP_FSM_EVENT_STOP);
    }
}

static bool app_runtime_apply_safety(void) {
    const AppFsmStateId state = app_fsm_get_state();
    const AppManualMode manual_mode = app_fsm_get_manual_mode();
    bool allow_control = true;

    if(state == APP_FSM_STATE_IDLE || state == APP_FSM_STATE_FINISHED) {
        (void)app_control_stop_all();
        return false;
    }

    if(state == APP_FSM_STATE_FAULT || state == APP_FSM_STATE_ESTOP) {
        (void)app_control_stop_all();
        return false;
    }

    if(state == APP_FSM_STATE_MANUAL) {
        if(!chassis.is_ready()) {
            app_runtime_raise_fault_once(APP_FAULT_SOURCE_CHASSIS,
                                         APP_FAULT_LEVEL_RECOVERABLE,
                                         APP_RUNTIME_CHASSIS_NOT_READY_FAULT_CODE);
            allow_control = false;
        }

        if(manual_mode == APP_MANUAL_MODE_CHASSIS_PC_ARM) {
            if(!odom.is_ready()) {
                app_runtime_raise_fault_once(APP_FAULT_SOURCE_ODOM,
                                             APP_FAULT_LEVEL_RECOVERABLE,
                                             APP_RUNTIME_ODOM_NOT_READY_FAULT_CODE);
                allow_control = false;
            }

            if(!arm.is_ready()) {
                log_warn("APP_RUNTIME manual chassis+pc arm degraded: arm not ready, pc arm disabled");
            }
        }
        else if(!arm.is_ready()) {
            app_runtime_raise_fault_once(APP_FAULT_SOURCE_ARM,
                                         APP_FAULT_LEVEL_RECOVERABLE,
                                         APP_RUNTIME_ARM_NOT_READY_FAULT_CODE);
            allow_control = false;
        }
    }

    if(state == APP_FSM_STATE_AUTO_PI) {
        if(!s_auto_start_latched) {
            if(delay_nb_ms(&s_auto_start_invalid_state_log_timer, APP_RUNTIME_EVENT_LOG_PERIOD_MS)) {
                log_warn("APP_RUNTIME auto_pi aborted: auto_start_latched=0");
            }
            app_runtime_leave_auto_pi();
            (void)app_control_stop_all();
            (void)app_fsm_post(APP_FSM_EVENT_STOP);
            app_fsm_process();
            return false;
        }

        if(!chassis.is_ready()) {
            app_runtime_raise_fault_once(APP_FAULT_SOURCE_CHASSIS,
                                         APP_FAULT_LEVEL_RECOVERABLE,
                                         APP_RUNTIME_CHASSIS_NOT_READY_FAULT_CODE);
            allow_control = false;
        }

        if(!odom.is_ready()) {
            app_runtime_raise_fault_once(APP_FAULT_SOURCE_ODOM,
                                         APP_FAULT_LEVEL_RECOVERABLE,
                                         APP_RUNTIME_ODOM_NOT_READY_FAULT_CODE);
            allow_control = false;
        }

        if(!pi_comms_is_online()) {
            app_runtime_raise_fault_once(APP_FAULT_SOURCE_PI_LINK,
                                         APP_FAULT_LEVEL_RECOVERABLE,
                                         APP_RUNTIME_PI_TIMEOUT_FAULT_CODE);
            pi_comms_clear_controls();
            allow_control = false;
        }

        if(!arm.is_ready() && app_runtime_pi_arm_cmd_pending()) {
            app_runtime_raise_fault_once(APP_FAULT_SOURCE_ARM,
                                         APP_FAULT_LEVEL_RECOVERABLE,
                                         APP_RUNTIME_ARM_NOT_READY_FAULT_CODE);
            allow_control = false;
        }
    }

    if(!allow_control) {
        (void)app_control_stop_all();
    }

    return allow_control;
}

static void app_runtime_apply_control(void) {
    AppControlResult result = APP_CONTROL_RESULT_SKIPPED;

    switch(app_fsm_get_state()) {
        case APP_FSM_STATE_MANUAL:
            if(app_fsm_get_manual_mode() == APP_MANUAL_MODE_ARM_FS) {
                result = app_control_apply_manual_arm_fs();
            }
            else {
                result = app_control_apply_manual_chassis_pc_arm();
            }
            break;

        case APP_FSM_STATE_AUTO_PI:
            result = app_control_apply_auto_pi();
            break;

        case APP_FSM_STATE_FAULT:
        case APP_FSM_STATE_ESTOP:
        case APP_FSM_STATE_IDLE:
        case APP_FSM_STATE_FINISHED:
        default:
            result = app_control_stop_all();
            break;
    }

    app_runtime_handle_control_result(result);
}

static void app_runtime_leave_manual_chassis_pc_arm(void) {
    pc_comms_clear_master_joints();
}

static void app_runtime_leave_auto_pi(void) {
    pi_comms_clear_controls();
}

static void app_runtime_raise_fault_once(AppFaultSource source, AppFaultLevel level, int32_t code) {
    AppFault fault;

    if(app_fsm_has_fault()) {
        return;
    }

    if(app_fsm_get_state() == APP_FSM_STATE_MANUAL && app_fsm_get_manual_mode() == APP_MANUAL_MODE_CHASSIS_PC_ARM) {
        app_runtime_leave_manual_chassis_pc_arm();
    }
    if(app_fsm_get_state() == APP_FSM_STATE_AUTO_PI) {
        app_runtime_leave_auto_pi();
    }

    fault.source = source;
    fault.level = level;
    fault.code = code;
    fault.stamp_ms = delay_now_ms();
    (void)app_fsm_raise_fault(&fault);
    app_fsm_process();
}

static bool app_runtime_pi_arm_cmd_pending(void) {
    if(pi_comms_has_pending_arm_action()) {
        return true;
    }

    return pi_comms_has_pending_arm_control();
}

static bool app_runtime_try_accept_auto_start_event(void) {
    const AppFsmStateId state = app_fsm_get_state();

    if(s_auto_start_latched) {
        if(delay_nb_ms(&s_auto_start_already_latched_log_timer, APP_RUNTIME_EVENT_LOG_PERIOD_MS)) {
            log_info("AUTO start event ignored, already latched");
        }
        return false;
    }

    if(state == APP_FSM_STATE_ESTOP) {
        if(delay_nb_ms(&s_auto_start_estop_log_timer, APP_RUNTIME_EVENT_LOG_PERIOD_MS)) {
            log_info("AUTO start rejected: estop active");
        }
        return false;
    }

    if(state != APP_FSM_STATE_IDLE) {
        if(delay_nb_ms(&s_auto_start_invalid_state_log_timer, APP_RUNTIME_EVENT_LOG_PERIOD_MS)) {
            log_info("AUTO start rejected: invalid state=%s", app_fsm_state_str(state));
        }
        return false;
    }

    if(app_fsm_has_fault()) {
        if(delay_nb_ms(&s_auto_start_fault_log_timer, APP_RUNTIME_EVENT_LOG_PERIOD_MS)) {
            log_info("AUTO start rejected: fault latched");
        }
        return false;
    }

    if(!pi_comms_is_online()) {
        if(delay_nb_ms(&s_auto_start_pi_offline_log_timer, APP_RUNTIME_EVENT_LOG_PERIOD_MS)) {
            log_info("AUTO start rejected: Pi offline");
        }
        return false;
    }

    if(!chassis.is_ready()) {
        if(delay_nb_ms(&s_auto_start_chassis_not_ready_log_timer, APP_RUNTIME_EVENT_LOG_PERIOD_MS)) {
            log_info("AUTO start rejected: chassis not ready");
        }
        return false;
    }

    if(!odom.is_ready()) {
        if(delay_nb_ms(&s_auto_start_odom_not_ready_log_timer, APP_RUNTIME_EVENT_LOG_PERIOD_MS)) {
            log_info("AUTO start rejected: odom not ready");
        }
        return false;
    }

    if(!app_runtime_can_accept_auto_start()) {
        return false;
    }

    (void)app_control_stop_all();
    pi_comms_clear_controls();
    chassis_yaw_hold_reset();
    app_runtime_set_auto_start_latched(true);
    (void)app_fsm_post(APP_FSM_EVENT_SWITCH_TO_AUTO_PI);
    if(delay_nb_ms(&s_auto_start_accept_log_timer, APP_RUNTIME_EVENT_LOG_PERIOD_MS)) {
        log_info("AUTO start event accepted, latched=1");
    }
    return true;
}

static bool app_runtime_can_accept_auto_start(void) {
    const AppFsmStateId state = app_fsm_get_state();

    return !s_auto_start_latched &&
           state == APP_FSM_STATE_IDLE &&
           !app_fsm_has_fault() &&
           state != APP_FSM_STATE_ESTOP &&
           pi_comms_is_online() &&
           chassis.is_ready() &&
           odom.is_ready();
}

static void app_runtime_set_auto_start_latched(bool latched) {
    s_auto_start_latched = latched;
}

static void app_runtime_handle_remote_clear_reset(void) {
    app_runtime_set_auto_start_latched(false);
    remote_clear_pending_auto_start_event();
    app_runtime_reset_auto_task_context();
    app_runtime_finish_reset_transition();
    if(delay_nb_ms(&s_remote_reset_log_timer, APP_RUNTIME_EVENT_LOG_PERIOD_MS)) {
        log_info("AUTO task reset by remote gesture, latched=0");
    }
}

static void app_runtime_reset_auto_task_context(void) {
    pc_comms_clear_master_joints();
    pi_comms_clear_controls();
    chassis_yaw_hold_reset();
    (void)app_control_stop_all();
}

static void app_runtime_finish_reset_transition(void) {
    const AppFsmStateId state = app_fsm_get_state();
    const AppFault* fault = app_fsm_get_fault();

    if(state == APP_FSM_STATE_FAULT) {
        if(fault != NULL && fault->level == APP_FAULT_LEVEL_RECOVERABLE && app_fsm_clear_fault()) {
            app_fsm_process();
        }
        return;
    }

    if(state == APP_FSM_STATE_AUTO_PI || state == APP_FSM_STATE_FINISHED) {
        (void)app_fsm_post(APP_FSM_EVENT_STOP);
        app_fsm_process();
    }
}

static void app_runtime_handle_control_result(AppControlResult result) {
    switch(result) {
        case APP_CONTROL_RESULT_CHASSIS_ERROR:
            if(app_fsm_get_state() == APP_FSM_STATE_MANUAL) {
                (void)app_control_stop_all();
            }
            app_runtime_raise_fault_once(APP_FAULT_SOURCE_CHASSIS,
                                         APP_FAULT_LEVEL_RECOVERABLE,
                                         APP_RUNTIME_CONTROL_EXEC_FAULT_CODE);
            break;

        case APP_CONTROL_RESULT_ARM_ERROR:
            if(app_fsm_get_state() == APP_FSM_STATE_MANUAL) {
                if(app_fsm_get_manual_mode() == APP_MANUAL_MODE_CHASSIS_PC_ARM) {
                    (void)app_control_stop_arm();
                    log_warn("APP_RUNTIME manual pc arm degraded: arm execute failed");
                }
                else {
                    (void)app_control_stop_all();
                    app_runtime_raise_fault_once(APP_FAULT_SOURCE_ARM,
                                                 APP_FAULT_LEVEL_RECOVERABLE,
                                                 APP_RUNTIME_CONTROL_EXEC_FAULT_CODE);
                }
            }
            else if(app_fsm_get_state() == APP_FSM_STATE_AUTO_PI) {
                app_runtime_raise_fault_once(APP_FAULT_SOURCE_ARM,
                                             APP_FAULT_LEVEL_RECOVERABLE,
                                             APP_RUNTIME_CONTROL_EXEC_FAULT_CODE);
            }
            break;

        case APP_CONTROL_RESULT_ODOM_ERROR:
            if(app_fsm_get_state() == APP_FSM_STATE_MANUAL ||
               app_fsm_get_state() == APP_FSM_STATE_AUTO_PI) {
                app_runtime_raise_fault_once(APP_FAULT_SOURCE_ODOM,
                                             APP_FAULT_LEVEL_RECOVERABLE,
                                             APP_RUNTIME_CONTROL_EXEC_FAULT_CODE);
            }
            break;

        case APP_CONTROL_RESULT_REJECTED:
            break;

        case APP_CONTROL_RESULT_COMMAND_INVALID:
            if(delay_nb_ms(&s_command_invalid_log_timer, APP_RUNTIME_RESULT_LOG_PERIOD_MS)) {
                log_warn("APP_RUNTIME control command invalid: state=%s",
                         app_fsm_state_str(app_fsm_get_state()));
            }
            break;

        case APP_CONTROL_RESULT_UNSUPPORTED:
            if(delay_nb_ms(&s_unsupported_log_timer, APP_RUNTIME_RESULT_LOG_PERIOD_MS)) {
                log_warn("APP_RUNTIME auto_pi received unsupported command");
            }
            break;

        case APP_CONTROL_RESULT_OK:
        case APP_CONTROL_RESULT_SKIPPED:
        default:
            break;
    }
}

static void app_runtime_handle_pi_mission_event(const PiCommsMissionEvent* event) {
    if(event == NULL) {
        return;
    }

    if(app_fsm_get_state() != APP_FSM_STATE_AUTO_PI) {
        return;
    }

    if(event->type == PI_COMMS_MISSION_EVENT_DONE) {
        app_runtime_leave_auto_pi();
        (void)app_control_stop_all();
        (void)app_fsm_post(APP_FSM_EVENT_FINISHED);
        return;
    }

    if(event->type == PI_COMMS_MISSION_EVENT_FAIL) {
        app_runtime_leave_auto_pi();
        app_runtime_raise_fault_once(APP_FAULT_SOURCE_PI_MISSION,
                                     APP_FAULT_LEVEL_RECOVERABLE,
                                     event->fail_code);
    }
}

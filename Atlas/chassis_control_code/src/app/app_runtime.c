/**
 * @file app_runtime.c
 * @brief MCU 应用运行时主控实现
 */

#include "app_runtime.h"

#include "app_control.h"
#include "app_fsm.h"
#include "arm.h"
#include "chassis.h"
#include "delay.h"
#include "log.h"
#include "odom.h"
#include "pc_link.h"
#include "pi_link.h"
#include "remote.h"

#include <string.h>

// ! ========================= 宏 定 义 声 明 ========================= ! //

#define APP_RUNTIME_CHASSIS_NOT_READY_FAULT_CODE 1
#define APP_RUNTIME_ARM_NOT_READY_FAULT_CODE 1
#define APP_RUNTIME_ODOM_NOT_READY_FAULT_CODE 1
#define APP_RUNTIME_PI_TIMEOUT_FAULT_CODE 1
#define APP_RUNTIME_CONTROL_EXEC_FAULT_CODE 2

// ! ========================= 变 量 声 明 ========================= ! //

static RemoteState s_remote_state = { 0 };
static PcCommand s_pc_command = { 0 };
static bool s_pc_command_valid = false;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void app_runtime_update_inputs(void);
static void app_runtime_update_mode(void);
static bool app_runtime_apply_safety(void);
static void app_runtime_apply_control(void);
static void app_runtime_raise_fault_once(AppFaultSource source, AppFaultLevel level, int32_t code);
static bool app_runtime_pi_arm_cmd_pending(void);
static void app_runtime_handle_control_result(AppControlResult result);
static void app_runtime_handle_pi_mission_event(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void app_runtime_init(void) {
    memset(&s_remote_state, 0, sizeof(s_remote_state));
    memset(&s_pc_command, 0, sizeof(s_pc_command));
    s_pc_command_valid = false;

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

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static void app_runtime_update_inputs(void) {
    (void)remote_get_state(&s_remote_state);
    s_pc_command_valid = pc_link_get_command(&s_pc_command);
}

static void app_runtime_update_mode(void) {
    PiMissionEvent mission_event;

    if(pi_link_get_estop_requested()) {
        pi_link_clear_estop_request();
        (void)app_fsm_request_estop();
        return;
    }

    if(pi_link_get_mission_event(&mission_event)) {
        app_runtime_handle_pi_mission_event();
        return;
    }

    if(s_pc_command_valid) {
        switch(s_pc_command.id) {
            case PC_COMMAND_ARM_STOP:
                (void)app_control_apply_pc_command(s_pc_command.id);
                pc_link_clear_command();
                s_pc_command_valid = false;
                break;

            case PC_COMMAND_ARM_ENABLE:
                if(app_fsm_get_state() != APP_FSM_STATE_FAULT &&
                   app_fsm_get_state() != APP_FSM_STATE_ESTOP &&
                   app_fsm_get_state() != APP_FSM_STATE_AUTO_PI) {
                    (void)app_control_apply_pc_command(s_pc_command.id);
                }
                else {
                    log_warn("APP_RUNTIME pc arm_enable rejected: state=%s",
                             app_fsm_state_str(app_fsm_get_state()));
                }
                pc_link_clear_command();
                s_pc_command_valid = false;
                break;

            case PC_COMMAND_STOP:
            case PC_COMMAND_BRAKE:
                (void)app_control_apply_pc_command(s_pc_command.id);
                (void)app_fsm_post(APP_FSM_EVENT_STOP);
                pc_link_clear_command();
                s_pc_command_valid = false;
                return;

            case PC_COMMAND_CLEAR_FAULT:
                if(app_fsm_get_state() == APP_FSM_STATE_FAULT) {
                    (void)app_fsm_clear_fault();
                }
                pc_link_clear_command();
                s_pc_command_valid = false;
                return;

            case PC_COMMAND_ESTOP:
                (void)app_fsm_request_estop();
                pc_link_clear_command();
                s_pc_command_valid = false;
                return;

            case PC_COMMAND_START:
                log_warn("APP_RUNTIME pc start ignored: use mode switch instead");
                pc_link_clear_command();
                s_pc_command_valid = false;
                break;

            case PC_COMMAND_NONE:
            default:
                break;
        }
    }

    if(app_fsm_get_state() == APP_FSM_STATE_FAULT || app_fsm_get_state() == APP_FSM_STATE_ESTOP) {
        return;
    }

    if(remote_is_manual_requested()) {
        if(remote_get_manual_source() == REMOTE_MANUAL_SOURCE_ARM) {
            app_fsm_set_manual_mode(APP_MANUAL_MODE_ARM_FS);
        }
        else {
            app_fsm_set_manual_mode(APP_MANUAL_MODE_CHASSIS_PC_ARM);
        }

        (void)app_fsm_post(APP_FSM_EVENT_SWITCH_TO_MANUAL);
        return;
    }

    if(remote_is_auto_requested()) {
        (void)app_fsm_post(APP_FSM_EVENT_SWITCH_TO_AUTO_PI);
        return;
    }

    if(app_fsm_get_state() == APP_FSM_STATE_MANUAL && !s_remote_state.manual_request) {
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

        if(!pi_link_is_online()) {
            app_runtime_raise_fault_once(APP_FAULT_SOURCE_PI_LINK,
                                         APP_FAULT_LEVEL_RECOVERABLE,
                                         APP_RUNTIME_PI_TIMEOUT_FAULT_CODE);
            pi_link_clear_commands();
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

static void app_runtime_raise_fault_once(AppFaultSource source, AppFaultLevel level, int32_t code) {
    AppFault fault;

    if(app_fsm_has_fault()) {
        return;
    }

    fault.source = source;
    fault.level = level;
    fault.code = code;
    fault.stamp_ms = delay_now_ms();
    (void)app_fsm_raise_fault(&fault);
    app_fsm_process();
}

static bool app_runtime_pi_arm_cmd_pending(void) {
    PiArmCommand cmd;

    if(!pi_link_arm_cmd_is_fresh(200u) || !pi_link_get_arm_cmd(&cmd)) {
        return false;
    }

    return cmd.type != PI_ARM_COMMAND_NONE;
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

        case APP_CONTROL_RESULT_COMMAND_INVALID:
            if(app_fsm_get_state() == APP_FSM_STATE_AUTO_PI) {
                app_runtime_raise_fault_once(APP_FAULT_SOURCE_COMMAND,
                                             APP_FAULT_LEVEL_RECOVERABLE,
                                             APP_RUNTIME_CONTROL_EXEC_FAULT_CODE);
            }
            break;

        case APP_CONTROL_RESULT_UNSUPPORTED:
            log_warn("APP_RUNTIME auto_pi received unsupported command");
            break;

        case APP_CONTROL_RESULT_OK:
        case APP_CONTROL_RESULT_SKIPPED:
        default:
            break;
    }
}

static void app_runtime_handle_pi_mission_event(void) {
    PiMissionEvent event;

    if(!pi_link_get_mission_event(&event)) {
        return;
    }

    if(app_fsm_get_state() != APP_FSM_STATE_AUTO_PI) {
        pi_link_clear_mission_event();
        return;
    }

    if(event.type == PI_MISSION_EVENT_DONE) {
        (void)app_control_stop_all();
        (void)app_fsm_post(APP_FSM_EVENT_FINISHED);
        pi_link_clear_mission_event();
        return;
    }

    if(event.type == PI_MISSION_EVENT_FAIL) {
        pi_link_clear_mission_event();
        app_runtime_raise_fault_once(APP_FAULT_SOURCE_PI_MISSION,
                                     APP_FAULT_LEVEL_RECOVERABLE,
                                     event.fail_code);
    }
}

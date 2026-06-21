/**
 * @file app_status.c
 * @brief MCU 状态显示与低频日志实现
 */

#include "app_status.h"

#include "app_fsm.h"
#include "app_runtime.h"
#include "arm.h"
#include "binary_frame.h"
#include "chassis.h"
#include "delay.h"
#include "log.h"
#include "odom.h"
#include "pc_comms.h"
#include "pi_comms.h"
#include "remote.h"
#include "rgb_led/rgb_led.h"

#include <stddef.h>

// ! ========================= 类 型 声 明 ========================= ! //

typedef enum {
    APP_LED_STATE_NOT_READY = 0,
    APP_LED_STATE_READY,
    APP_LED_STATE_MANUAL,
    APP_LED_STATE_AUTO_PI,
    APP_LED_STATE_FAULT,
    APP_LED_STATE_ESTOP
} AppLedState;

// ! ========================= 变 量 声 明 ========================= ! //

static ms_t s_log_timer = 0u;
static ms_t s_heartbeat_timer = 0u;
static ms_t s_pi_status_send_timer = 0u;
static ms_t s_pi_imu_odom_send_timer = 0u;
static AppLedState s_led_state = APP_LED_STATE_NOT_READY;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void app_status_update_led(void);
static void app_status_send_pi_status(void);
static void app_status_send_pi_imu_odom(void);
static void app_status_log(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void app_status_init(void) {
    s_log_timer = 0u;
    s_heartbeat_timer = 0u;
    s_pi_status_send_timer = 0u;
    s_pi_imu_odom_send_timer = 0u;
    s_led_state = APP_LED_STATE_NOT_READY;
}

void app_status_process(void) {
    app_status_update_led();
    app_status_send_pi_status();
    app_status_send_pi_imu_odom();
    app_status_log();
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static void app_status_update_led(void) {
    AppLedState target_state;
    const AppFsmStateId state = app_runtime_get_state();

    if(!delay_nb_ms(&s_heartbeat_timer, 200u)) {
        return;
    }

    switch(state) {
        case APP_FSM_STATE_ESTOP:
            target_state = APP_LED_STATE_ESTOP;
            break;

        case APP_FSM_STATE_FAULT:
            target_state = APP_LED_STATE_FAULT;
            break;

        case APP_FSM_STATE_MANUAL:
            target_state = APP_LED_STATE_MANUAL;
            break;

        case APP_FSM_STATE_AUTO_PI:
            target_state = APP_LED_STATE_AUTO_PI;
            break;

        case APP_FSM_STATE_IDLE:
        case APP_FSM_STATE_FINISHED:
        default:
            if(!chassis.is_ready() || !arm.is_ready()) {
                target_state = APP_LED_STATE_NOT_READY;
            }
            else {
                target_state = APP_LED_STATE_READY;
            }
            break;
    }

    if(target_state == s_led_state) {
        return;
    }

    switch(target_state) {
        case APP_LED_STATE_NOT_READY:
            rgb_led.fill(255u, 0u, 0u);
            break;

        case APP_LED_STATE_READY:
            rgb_led.fill(0u, 255u, 0u);
            break;

        case APP_LED_STATE_MANUAL:
            rgb_led.fill(0u, 0u, 255u);
            break;

        case APP_LED_STATE_AUTO_PI:
            rgb_led.fill(0u, 255u, 255u);
            break;

        case APP_LED_STATE_FAULT:
            rgb_led.fill(255u, 128u, 0u);
            break;

        case APP_LED_STATE_ESTOP:
        default:
            rgb_led.fill(255u, 0u, 255u);
            break;
    }

    if(rgb_led.show() == RGB_LED_STATUS_OK) {
        s_led_state = target_state;
    }
}

static void app_status_send_pi_status(void) {
    PiCommsStatusSnapshot status = { 0 };
    const AppFault* fault;
    uint8_t ready_flags = 0u;
    uint8_t online_flags = 0u;
    const bool remote_online = remote_is_online(100u);
    const bool pc_online = pc_comms_is_online();
    const bool pi_online = pi_comms_is_online();
    const bool has_fault = app_runtime_has_fault();
    const bool estop = app_runtime_get_state() == APP_FSM_STATE_ESTOP;

    if(!delay_nb_ms(&s_pi_status_send_timer, 100u)) {
        return;
    }

    fault = app_runtime_get_fault();
    ready_flags |= chassis.is_ready() ? BINARY_FRAME_STATUS_READY_CHASSIS : 0u;
    ready_flags |= arm.is_ready() ? BINARY_FRAME_STATUS_READY_ARM : 0u;
    ready_flags |= odom.is_ready() ? BINARY_FRAME_STATUS_READY_ODOM : 0u;
    ready_flags |= remote_online ? BINARY_FRAME_STATUS_READY_REMOTE : 0u;
    ready_flags |= pc_online ? BINARY_FRAME_STATUS_READY_PC : 0u;
    ready_flags |= pi_online ? BINARY_FRAME_STATUS_READY_PI : 0u;

    online_flags |= remote_online ? BINARY_FRAME_STATUS_ONLINE_REMOTE : 0u;
    online_flags |= pc_online ? BINARY_FRAME_STATUS_ONLINE_PC : 0u;
    online_flags |= pi_online ? BINARY_FRAME_STATUS_ONLINE_PI : 0u;
    online_flags |= has_fault ? BINARY_FRAME_STATUS_HAS_FAULT : 0u;
    online_flags |= estop ? BINARY_FRAME_STATUS_ESTOP : 0u;

    status.stamp_ms = delay_now_ms();
    status.app_state = (uint8_t)app_runtime_get_state();
    status.manual_mode = (uint8_t)app_runtime_get_manual_mode();
    status.ready_flags = ready_flags;
    status.online_flags = online_flags;
    status.fault_source = fault != NULL ? (uint8_t)fault->source : 0u;
    status.fault_level = fault != NULL ? (uint8_t)fault->level : 0u;
    status.fault_code = fault != NULL ? (int16_t)fault->code : 0;
    (void)pi_comms_send_status(&status);
}

static void app_status_send_pi_imu_odom(void) {
    PiCommsImuOdomSnapshot snapshot = { 0 };
    Vector3 angle = { 0 };
    Vector3 gyro = { 0 };
    Vector3 odom_vec = { 0 };

    if(!delay_nb_ms(&s_pi_imu_odom_send_timer, 50u)) {
        return;
    }

    (void)odom.get_angle(&angle);
    (void)odom.get_gyro_corrected(&gyro);
    (void)odom.get_odom(&odom_vec);

    snapshot.stamp_ms = delay_now_ms();
    snapshot.angle_x = angle.x;
    snapshot.angle_y = angle.y;
    snapshot.angle_z = angle.z;
    snapshot.gyro_x = gyro.x;
    snapshot.gyro_y = gyro.y;
    snapshot.gyro_z = gyro.z;
    snapshot.odom_x = odom_vec.x;
    snapshot.odom_y = odom_vec.y;
    snapshot.odom_yaw = odom_vec.z;
    (void)pi_comms_send_imu_odom(&snapshot);
}

static void app_status_log(void) {
    const AppFault* fault;

    if(!delay_nb_ms(&s_log_timer, 1000u)) {
        return;
    }

    fault = app_runtime_get_fault();
    log_info("Heartbeat state=%s manual=%s remote=%u pc=%u pi=%u fault=%u src=%u level=%u code=%ld",
             app_fsm_state_str(app_runtime_get_state()),
             app_fsm_manual_mode_str(app_runtime_get_manual_mode()),
             remote_is_online(100u) ? 1u : 0u,
             pc_comms_is_online() ? 1u : 0u,
             pi_comms_is_online() ? 1u : 0u,
             app_runtime_has_fault() ? 1u : 0u,
             fault != NULL ? (unsigned int)fault->source : 0u,
             fault != NULL ? (unsigned int)fault->level : 0u,
             fault != NULL ? (long)fault->code : 0l);
}

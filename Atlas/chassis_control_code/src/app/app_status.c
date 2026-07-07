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

#include <math.h>
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
static uint32_t s_pi_tx_last_absolute_slot = 0u;
static bool s_pi_tx_slot_initialized = false;
static uint32_t s_pi_tx_last_log_ms = 0u;
static AppLedState s_led_state = APP_LED_STATE_NOT_READY;
static uint16_t s_pi_imu_sequence_count = 0u;
static uint16_t s_pi_arm_state_sequence_count = 0u;

typedef struct {
    PiCommsStats stats;
} AppStatusPiStatsSnapshot;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void app_status_update_led(void);
static void app_status_process_pi_tx_slot(void);
static void app_status_send_pi_status_now(void);
static void app_status_send_pi_imu_now(void);
static void app_status_send_pi_odom_now(void);
static void app_status_send_pi_arm_state_now(void);
static void app_status_build_pi_imu_snapshot(PiCommsImuSnapshot* snapshot);
static void app_status_build_pi_odom_snapshot(PiCommsOdomSnapshot* snapshot);
static void app_status_build_pi_arm_state_snapshot(PiCommsArmStateSnapshot* snapshot);
static void app_status_snapshot_pi_stats(AppStatusPiStatsSnapshot* snapshot);
static void app_status_log(void);
static bool app_status_pose_has_finite_position(const FiveDofArmPose* pose);
static bool app_status_try_normalize_quaternion(const SerialArmQuaternion* in, SerialArmQuaternion* out);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void app_status_init(void) {
    s_log_timer = 0u;
    s_heartbeat_timer = 0u;
    s_pi_tx_last_absolute_slot = 0u;
    s_pi_tx_slot_initialized = false;
    s_pi_tx_last_log_ms = 0u;
    s_led_state = APP_LED_STATE_NOT_READY;
    s_pi_imu_sequence_count = 0u;
    s_pi_arm_state_sequence_count = 0u;
}

void app_status_process(void) {
    app_status_update_led();
    app_status_process_pi_tx_slot();
    app_status_log();
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 非阻塞周期更新 LED 状态
 */
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

static void app_status_process_pi_tx_slot(void) {
    const uint32_t absolute_slot = delay_now_ms() / 2u; // 0~500
    const uint32_t slot_mod = absolute_slot % 50u;      // 0~49

    if(!s_pi_tx_slot_initialized) {
        s_pi_tx_last_absolute_slot = absolute_slot;
        s_pi_tx_slot_initialized = true;
    }
    else if(absolute_slot == s_pi_tx_last_absolute_slot) {
        return;
    }
    else {
        s_pi_tx_last_absolute_slot = absolute_slot;
    }

    if((slot_mod % 5u) == 0u) { // 0,5,10,15,20,25,30,35,40,45
        app_status_send_pi_imu_now();
        return;
    }

    if((slot_mod % 10u) == 2u) { // 2,12,22,32,42
        app_status_send_pi_odom_now();
        return;
    }

    if((slot_mod % 10u) == 7u) { // 7,17,27,37,47
        app_status_send_pi_arm_state_now();
        return;
    }

    if(slot_mod == 4u) {
        app_status_send_pi_status_now();
    }
}

static void app_status_send_pi_status_now(void) {
    PiCommsStatusSnapshot status = { 0 };
    const AppFault* fault;
    uint8_t ready_flags = 0u;
    uint8_t online_flags = 0u;
    const bool remote_online = remote_is_online(100u);
    const bool pc_online = pc_comms_is_online();
    const bool pi_online = pi_comms_is_online();
    const bool has_fault = app_runtime_has_fault();
    const bool estop = app_runtime_get_state() == APP_FSM_STATE_ESTOP;
    const ms_t now_ms = delay_now_ms();

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

    status.stamp_ms = now_ms;
    status.app_state = (uint8_t)app_runtime_get_state();
    status.manual_mode = (uint8_t)app_runtime_get_manual_mode();
    status.ready_flags = ready_flags;
    status.online_flags = online_flags;
    status.fault_source = fault != NULL ? (uint8_t)fault->source : 0u;
    status.fault_level = fault != NULL ? (uint8_t)fault->level : 0u;
    status.fault_code = fault != NULL ? (int16_t)fault->code : 0;
    status.auto_start_latched = app_runtime_is_auto_start_latched() ? 1u : 0u;
    (void)pi_comms_send_status(&status);
}

/**
 * @brief 按 100Hz 周期发送 MCU_IMU
 */
static void app_status_send_pi_imu_now(void) {
    PiCommsImuSnapshot snapshot = { 0 };

    app_status_build_pi_imu_snapshot(&snapshot);
    (void)pi_comms_send_imu(&snapshot);
}

/**
 * @brief 按 50Hz 周期发送 MCU_ODOM
 */
static void app_status_send_pi_odom_now(void) {
    PiCommsOdomSnapshot snapshot = { 0 };

    app_status_build_pi_odom_snapshot(&snapshot);
    (void)pi_comms_send_odom(&snapshot);
}

/**
 * @brief 按 50Hz 周期发送 MCU_ARM_STATE
 */
static void app_status_send_pi_arm_state_now(void) {
    PiCommsArmStateSnapshot snapshot = { 0 };

    app_status_build_pi_arm_state_snapshot(&snapshot);
    (void)pi_comms_send_arm_state(&snapshot);
}

/**
 * @brief 组装 MCU_IMU 快照
 * @details yaw 使用融合后的 angle.z，gyro 优先使用去 bias 后的 gyro_corrected
 */
static void app_status_build_pi_imu_snapshot(PiCommsImuSnapshot* snapshot) {
    Vector3 angle = { 0 };
    Vector3 acc = { 0 };
    Vector3 gyro = { 0 };
    uint16_t status_flags = 0u;

    if(snapshot == NULL) {
        return;
    }

    snapshot->stamp_ms = delay_now_ms();
    snapshot->sequence_count = s_pi_imu_sequence_count++;

    if(odom.get_acc(&acc) == ODOM_OK) {
        status_flags |= PI_COMMS_IMU_STATUS_ACC_VALID;
        snapshot->acc_x_mm_s2 = binary_frame_m_to_mm_i32(acc.x);
        snapshot->acc_y_mm_s2 = binary_frame_m_to_mm_i32(acc.y);
        snapshot->acc_z_mm_s2 = binary_frame_m_to_mm_i32(acc.z);
    }

    if(odom.get_gyro_corrected(&gyro) == ODOM_OK) {
        status_flags |= PI_COMMS_IMU_STATUS_GYRO_VALID | PI_COMMS_IMU_STATUS_BIAS_CORRECTED;
        snapshot->gyro_x_urad_s = binary_frame_rad_to_urad(gyro.x);
        snapshot->gyro_y_urad_s = binary_frame_rad_to_urad(gyro.y);
        snapshot->gyro_z_urad_s = binary_frame_rad_to_urad(gyro.z);
    }

    if(odom.get_angle(&angle) == ODOM_OK) {
        status_flags |= PI_COMMS_IMU_STATUS_ANGLE_VALID | PI_COMMS_IMU_STATUS_YAW_FUSED_WITH_CHASSIS;
        snapshot->roll_urad = binary_frame_rad_to_urad(angle.x);
        snapshot->pitch_urad = binary_frame_rad_to_urad(angle.y);
        snapshot->yaw_urad = binary_frame_rad_to_urad(angle.z);
    }

    if(odom.is_ready()) {
        status_flags |= PI_COMMS_IMU_STATUS_IMU_READY;
    }

    snapshot->status_flags = status_flags;
}

/**
 * @brief 组装 MCU_ODOM 快照
 * @details x/y 属于 odom 坐标系，vx/vy/wz 属于 base_link 坐标系，yaw 与 IMU 帧同源
 */
static void app_status_build_pi_odom_snapshot(PiCommsOdomSnapshot* snapshot) {
    Vector3 angle = { 0 };
    Vector3 odom_vec = { 0 };
    Vector3 velocity = { 0 };
    uint16_t status_flags = 0u;

    if(snapshot == NULL) {
        return;
    }

    snapshot->stamp_ms = delay_now_ms();

    if(odom.get_odom(&odom_vec) == ODOM_OK) {
        status_flags |= PI_COMMS_ODOM_STATUS_ODOM_READY | PI_COMMS_ODOM_STATUS_POSE_VALID;
        snapshot->x_mm = binary_frame_m_to_mm_i32(odom_vec.x);
        snapshot->y_mm = binary_frame_m_to_mm_i32(odom_vec.y);
    }

    if(odom.get_angle(&angle) == ODOM_OK) {
        status_flags |= PI_COMMS_ODOM_STATUS_IMU_FUSED;
        snapshot->yaw_urad = binary_frame_rad_to_urad(angle.z);
    }

    if(odom.get_velocity(&velocity) == ODOM_OK) {
        status_flags |= PI_COMMS_ODOM_STATUS_TWIST_VALID | PI_COMMS_ODOM_STATUS_WHEEL_FUSED;
        snapshot->vx_mm_s = binary_frame_m_to_mm_i32(velocity.x);
        snapshot->vy_mm_s = binary_frame_m_to_mm_i32(velocity.y);
        snapshot->wz_urad_s = binary_frame_rad_to_urad(velocity.z);
    }

    snapshot->status_flags = status_flags;
}

/**
 * @brief 组装 MCU_ARM_STATE 快照
 * @details 只读取 arm 服务缓存，不在此处主动刷新机械臂当前状态
 */
static void app_status_build_pi_arm_state_snapshot(PiCommsArmStateSnapshot* snapshot) {
    const FiveDofArmJointArray* joints;
    const FiveDofArmPose* pose;
    SerialArmQuaternion normalized_quat = { 0 };
    uint16_t status_flags = 0u;

    if(snapshot == NULL) {
        return;
    }

    snapshot->stamp_ms = delay_now_ms();
    snapshot->sequence_count = s_pi_arm_state_sequence_count++;

    if(arm.is_ready()) {
        status_flags |= PI_COMMS_ARM_STATE_STATUS_ARM_READY;
    }

    joints = arm.get_current_joints();
    if(joints != NULL && joints->dof >= FIVE_DOF_ARM_DOF) {
        status_flags |= PI_COMMS_ARM_STATE_STATUS_JOINT_VALID;
        snapshot->q0_urad = binary_frame_rad_to_urad(joints->q[0]);
        snapshot->q1_urad = binary_frame_rad_to_urad(joints->q[1]);
        snapshot->q2_urad = binary_frame_rad_to_urad(joints->q[2]);
        snapshot->q3_urad = binary_frame_rad_to_urad(joints->q[3]);
        snapshot->q4_urad = binary_frame_rad_to_urad(joints->q[4]);
    }

    pose = arm.get_current_pose();
    if(pose != NULL) {
        snapshot->x_mm = binary_frame_m_to_mm_i32(pose->position.x);
        snapshot->y_mm = binary_frame_m_to_mm_i32(pose->position.y);
        snapshot->z_mm = binary_frame_m_to_mm_i32(pose->position.z);

        if(app_status_pose_has_finite_position(pose) &&
           app_status_try_normalize_quaternion(&pose->orientation, &normalized_quat)) {
            status_flags |= PI_COMMS_ARM_STATE_STATUS_POSE_VALID;
            snapshot->quat_x_q15 = binary_frame_unit_to_q15_i16(normalized_quat.x);
            snapshot->quat_y_q15 = binary_frame_unit_to_q15_i16(normalized_quat.y);
            snapshot->quat_z_q15 = binary_frame_unit_to_q15_i16(normalized_quat.z);
            snapshot->quat_w_q15 = binary_frame_unit_to_q15_i16(normalized_quat.w);
        }
        else {
            snapshot->quat_x_q15 = 0;
            snapshot->quat_y_q15 = 0;
            snapshot->quat_z_q15 = 0;
            snapshot->quat_w_q15 = 32767;
        }
    }
    else {
        snapshot->quat_x_q15 = 0;
        snapshot->quat_y_q15 = 0;
        snapshot->quat_z_q15 = 0;
        snapshot->quat_w_q15 = 32767;
    }

    snapshot->status_flags = status_flags;
}

static void app_status_snapshot_pi_stats(AppStatusPiStatsSnapshot* snapshot) {
    if(snapshot == NULL) {
        return;
    }

    (void)pi_comms_get_stats(&snapshot->stats);
}

static void app_status_log(void) {
    const AppFault* fault;
    static AppStatusPiStatsSnapshot last_snapshot = { 0 };
    AppStatusPiStatsSnapshot snapshot = { 0 };
    uint32_t elapsed_ms;
    float status_rate_hz;
    float imu_rate_hz;
    float odom_rate_hz;
    float arm_rate_hz;
    uint32_t status_attempt_delta;
    uint32_t status_ok_delta;
    uint32_t status_fail_delta;
    uint32_t imu_attempt_delta;
    uint32_t imu_ok_delta;
    uint32_t imu_fail_delta;
    uint32_t odom_attempt_delta;
    uint32_t odom_ok_delta;
    uint32_t odom_fail_delta;
    uint32_t arm_attempt_delta;
    uint32_t arm_ok_delta;
    uint32_t arm_fail_delta;
    uint32_t total_ok_delta;
    uint32_t total_fail_delta;

    if(!delay_nb_ms(&s_log_timer, 1000u)) {
        return;
    }

    app_status_snapshot_pi_stats(&snapshot);
    elapsed_ms = s_pi_tx_last_log_ms == 0u ? 1000u : (delay_now_ms() - s_pi_tx_last_log_ms);
    s_pi_tx_last_log_ms = delay_now_ms();
    if(elapsed_ms == 0u) {
        elapsed_ms = 1u;
    }
    status_attempt_delta = snapshot.stats.tx_status_attempt_count - last_snapshot.stats.tx_status_attempt_count;
    status_ok_delta = snapshot.stats.tx_status_ok_count - last_snapshot.stats.tx_status_ok_count;
    status_fail_delta = snapshot.stats.tx_status_fail_count - last_snapshot.stats.tx_status_fail_count;
    imu_attempt_delta = snapshot.stats.tx_imu_attempt_count - last_snapshot.stats.tx_imu_attempt_count;
    imu_ok_delta = snapshot.stats.tx_imu_ok_count - last_snapshot.stats.tx_imu_ok_count;
    imu_fail_delta = snapshot.stats.tx_imu_fail_count - last_snapshot.stats.tx_imu_fail_count;
    odom_attempt_delta = snapshot.stats.tx_odom_attempt_count - last_snapshot.stats.tx_odom_attempt_count;
    odom_ok_delta = snapshot.stats.tx_odom_ok_count - last_snapshot.stats.tx_odom_ok_count;
    odom_fail_delta = snapshot.stats.tx_odom_fail_count - last_snapshot.stats.tx_odom_fail_count;
    arm_attempt_delta = snapshot.stats.tx_arm_state_attempt_count - last_snapshot.stats.tx_arm_state_attempt_count;
    arm_ok_delta = snapshot.stats.tx_arm_state_ok_count - last_snapshot.stats.tx_arm_state_ok_count;
    arm_fail_delta = snapshot.stats.tx_arm_state_fail_count - last_snapshot.stats.tx_arm_state_fail_count;
    total_ok_delta = snapshot.stats.tx_frame_count - last_snapshot.stats.tx_frame_count;
    total_fail_delta = snapshot.stats.tx_fail_count - last_snapshot.stats.tx_fail_count;
    status_rate_hz = ((float)status_attempt_delta * 1000.0f) / (float)elapsed_ms;
    imu_rate_hz = ((float)imu_attempt_delta * 1000.0f) / (float)elapsed_ms;
    odom_rate_hz = ((float)odom_attempt_delta * 1000.0f) / (float)elapsed_ms;
    arm_rate_hz = ((float)arm_attempt_delta * 1000.0f) / (float)elapsed_ms;

    fault = app_runtime_get_fault();
    log_info("--------------------------------------------------");
    log_info("Heartbeat state=%s manual=%s remote=%u pc=%u pi=%u auto_start=%u fault=%u src=%u level=%u code=%ld",
             app_fsm_state_str(app_runtime_get_state()),
             app_fsm_manual_mode_str(app_runtime_get_manual_mode()),
             remote_is_online(100u) ? 1u : 0u,
             pc_comms_is_online() ? 1u : 0u,
             pi_comms_is_online() ? 1u : 0u,
             app_runtime_is_auto_start_latched() ? 1u : 0u,
             app_runtime_has_fault() ? 1u : 0u,
             fault != NULL ? (unsigned int)fault->source : 0u,
             fault != NULL ? (unsigned int)fault->level : 0u,
             fault != NULL ? (long)fault->code : 0l);
    log_info("PI_TX rate_hz status=%.1f imu=%.1f odom=%.1f arm=%.1f",
             (double)status_rate_hz,
             (double)imu_rate_hz,
             (double)odom_rate_hz,
             (double)arm_rate_hz);
    log_info("PI_TX result(attempt/ok/fail) status=%lu/%lu/%lu imu=%lu/%lu/%lu odom=%lu/%lu/%lu arm=%lu/%lu/%lu",
             (unsigned long)status_attempt_delta,
             (unsigned long)status_ok_delta,
             (unsigned long)status_fail_delta,
             (unsigned long)imu_attempt_delta,
             (unsigned long)imu_ok_delta,
             (unsigned long)imu_fail_delta,
             (unsigned long)odom_attempt_delta,
             (unsigned long)odom_ok_delta,
             (unsigned long)odom_fail_delta,
             (unsigned long)arm_attempt_delta,
             (unsigned long)arm_ok_delta,
             (unsigned long)arm_fail_delta);
    log_info("PI_TX error(pack/fail) pack=%lu crc_pre=%lu crc_post=%lu hal_busy=%lu hal_timeout=%lu hal_error=%lu total_ok=%lu total_fail=%lu",
             (unsigned long)(snapshot.stats.tx_pack_fail_count - last_snapshot.stats.tx_pack_fail_count),
             (unsigned long)(snapshot.stats.tx_crc_precheck_fail_count - last_snapshot.stats.tx_crc_precheck_fail_count),
             (unsigned long)(snapshot.stats.tx_crc_postcheck_fail_count - last_snapshot.stats.tx_crc_postcheck_fail_count),
             (unsigned long)(snapshot.stats.tx_hal_busy_count - last_snapshot.stats.tx_hal_busy_count),
             (unsigned long)(snapshot.stats.tx_hal_timeout_count - last_snapshot.stats.tx_hal_timeout_count),
             (unsigned long)(snapshot.stats.tx_hal_error_count - last_snapshot.stats.tx_hal_error_count),
             (unsigned long)total_ok_delta,
             (unsigned long)total_fail_delta);

    log_info("--------------------------------------------------");

    last_snapshot = snapshot;
}

static bool app_status_pose_has_finite_position(const FiveDofArmPose* pose) {
    return pose != NULL &&
           isfinite(pose->position.x) &&
           isfinite(pose->position.y) &&
           isfinite(pose->position.z);
}

static bool app_status_try_normalize_quaternion(const SerialArmQuaternion* in, SerialArmQuaternion* out) {
    const float max_component = 1.000001f;
    float norm;

    if(in == NULL || out == NULL) {
        return false;
    }
    if(!isfinite(in->x) || !isfinite(in->y) || !isfinite(in->z) || !isfinite(in->w)) {
        return false;
    }
    if(in->x < -max_component || in->x > max_component ||
       in->y < -max_component || in->y > max_component ||
       in->z < -max_component || in->z > max_component ||
       in->w < -max_component || in->w > max_component) {
        return false;
    }

    norm = sqrtf(in->x * in->x +
                 in->y * in->y +
                 in->z * in->z +
                 in->w * in->w);
    if(!isfinite(norm) || norm <= 1e-6f) {
        return false;
    }

    out->x = in->x / norm;
    out->y = in->y / norm;
    out->z = in->z / norm;
    out->w = in->w / norm;
    return isfinite(out->x) && isfinite(out->y) && isfinite(out->z) && isfinite(out->w);
}

/**
 * @file app_control.c
 * @brief MCU 控制映射与安全执行实现
 */

#include "app_control.h"

#include "arm.h"
#include "chassis.h"
#include "chassis_yaw_hold.h"
#include "delay.h"
#include "log.h"
#include "odom.h"
#include "pi_link.h"
#include "remote.h"

#include <math.h>

// ! ========================= 宏 定 义 声 明 ========================= ! //

#define REMOTE_CONTROL_PERIOD_S 0.010f

#define REMOTE_FAST_MAX_VX_MPS 2.0f
#define REMOTE_FAST_MAX_VY_MPS 2.0f
#define REMOTE_FAST_MAX_WZ_RAD_S 8.0f
#define REMOTE_MID_MAX_VX_MPS 1.0f
#define REMOTE_MID_MAX_VY_MPS 1.0f
#define REMOTE_MID_MAX_WZ_RAD_S 4.0f
#define REMOTE_SLOW_MAX_VX_MPS 0.5f
#define REMOTE_SLOW_MAX_VY_MPS 0.5f
#define REMOTE_SLOW_MAX_WZ_RAD_S 2.0f

#define APP_CONTROL_PC_JOINT_TIMEOUT_MS 200u
#define APP_CONTROL_PI_CHASSIS_TIMEOUT_MS 200u
#define APP_CONTROL_PI_YAW_TIMEOUT_MS 200u
#define APP_CONTROL_PI_ARM_TIMEOUT_MS 200u

#define APP_CONTROL_PI_MAX_VX_MPS 1.5f
#define APP_CONTROL_PI_MAX_VY_MPS 1.5f
#define APP_CONTROL_PI_MAX_WZ_RAD_S 4.0f
#define APP_CONTROL_ARM_MAX_SPEED_RAD_S 50.24f
#define APP_CONTROL_ARM_LIMIT_EPS_RAD 0.001f
#define APP_CONTROL_ARM_FALLBACK_SPEED_RAD_S 3.14f
#define APP_CONTROL_SKIP_LOG_PERIOD_MS 1000u
#define APP_CONTROL_ARM_STOP_RETRY_MS 1000u
#define APP_CONTROL_ARM_STOP_LOG_MS 1000u

// ! ========================= 类 型 声 明 ========================= ! //

typedef struct {
    float max_vx;
    float max_vy;
    float max_wz;
} RemoteSpeedLimit;

typedef struct {
    float base_end_yaw_rate_rad_s;
    float reach_speed_m_s;
    float z_speed_m_s;
    float end_pitch_rate_rad_s;
    float end_yaw_rate_rad_s;
    float servo_speed_rad_s;
} RemoteArmSpeedLimit;

// ! ========================= 变 量 声 明 ========================= ! //

static uint16_t s_last_arm_swc = 0u;
static ms_t s_stop_arm_skip_log_timer = 0u;
static bool s_arm_stop_done = false;
static bool s_arm_active = false;
static ms_t s_stop_arm_retry_last_ms = 0u;
static ms_t s_stop_arm_fail_log_last_ms = 0u;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 合并两个控制执行结果
 * @param lhs 当前结果
 * @param rhs 新结果
 * @return AppControlResult 优先级更高的结果
 */
static AppControlResult app_control_merge_result(AppControlResult lhs, AppControlResult rhs);

/**
 * @brief 将底盘返回码转换为控制层结果, 并在失败时记录告警
 * @param status 底盘返回值
 * @param action 当前动作描述
 * @return AppControlResult 转换后的执行结果
 */
static AppControlResult app_control_result_from_chassis(ChassisErrorCode status, const char* action);

/**
 * @brief 将机械臂返回码转换为控制层结果, 并在失败时记录告警
 * @param status 机械臂返回值
 * @param action 当前动作描述
 * @return AppControlResult 转换后的执行结果
 */
static AppControlResult app_control_result_from_arm(ArmStatus status, const char* action);

/**
 * @brief 对浮点值做绝对值限幅
 * @param value 输入值
 * @param limit 绝对值上限
 * @return float 限幅后的结果
 */
static float app_control_limit_abs(float value, float limit);

/**
 * @brief 将遥控通道值映射到 [-1, 1]
 * @param value 原始通道值
 * @param deadband 死区阈值
 * @return float 归一化结果
 */
static float app_control_channel_to_norm(uint16_t value, uint16_t deadband);

/**
 * @brief 根据 SWB 挡位获取底盘速度限制
 * @param swb 遥控挡位值
 * @return RemoteSpeedLimit 对应速度限制
 */
static RemoteSpeedLimit app_control_get_speed_limit(uint16_t swb);

/**
 * @brief 根据 SWB 挡位获取机械臂速度限制
 * @param swb 遥控挡位值
 * @return RemoteArmSpeedLimit 对应速度限制
 */
static RemoteArmSpeedLimit app_control_get_arm_speed_limit(uint16_t swb);

/**
 * @brief 获取最新遥控状态并检查在线
 * @param state 输出遥控状态
 * @return bool `true` 表示遥控在线且状态有效
 */
static bool app_control_get_remote_state(RemoteState* state);

/**
 * @brief 应用遥控底盘控制
 * @param state 当前遥控状态
 * @return AppControlResult 执行结果
 */
static AppControlResult app_control_apply_remote_chassis(const RemoteState* state);

/**
 * @brief 应用遥控机械臂控制
 * @param state 当前遥控状态
 * @return AppControlResult 执行结果
 */
static AppControlResult app_control_apply_remote_arm(const RemoteState* state);

/**
 * @brief 应用 PC 主臂关节控制
 * @return AppControlResult 执行结果
 */
static AppControlResult app_control_apply_pc_arm(void);

/**
 * @brief 应用 Pi 底盘控制
 * @return AppControlResult 执行结果
 */
static AppControlResult app_control_apply_pi_chassis(void);

/**
 * @brief 应用 Pi yaw 控制
 * @return AppControlResult 执行结果
 */
static AppControlResult app_control_apply_pi_yaw(void);

/**
 * @brief 应用 Pi 机械臂控制
 * @return AppControlResult 执行结果
 */
static AppControlResult app_control_apply_pi_arm(void);

/**
 * @brief 获取机械臂默认速度
 * @return float 默认关节速度
 */
static float app_control_get_arm_default_speed(void);

/**
 * @brief 判断机械臂速度参数是否有效
 * @param speed_rad_s 待检查速度
 * @return bool `true` 表示速度有效
 */
static bool app_control_arm_speed_valid(float speed_rad_s);

/**
 * @brief 判断机械臂关节数组是否有效
 * @param joints 待检查关节数组
 * @return bool `true` 表示关节目标有效
 */
static bool app_control_arm_joints_valid(const FiveDofArmJointArray* joints);

/**
 * @brief 判断指定时间间隔是否到期, 到期时更新最后触发时间
 * @param last_ms 上次触发时间
 * @param interval_ms 间隔
 * @return bool `true` 表示本次允许触发
 */
static bool app_control_interval_due(ms_t* last_ms, uint32_t interval_ms);

/**
 * @brief 标记当前进入机械臂有效控制分支
 * @details 仅在从非激活切换到激活时清除 stop 锁存, 以便退出后重新执行 stop
 */
static void app_control_mark_arm_active(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void app_control_init(void) {
    ChassisYawHoldConfig yaw_hold_config = chassis_yaw_hold_default_config();

    yaw_hold_config.kp = 48.0f;
    yaw_hold_config.kd = 2.0f;
    yaw_hold_config.v_deadband = 0.01f;
    yaw_hold_config.wz_deadband = 0.08f;
    yaw_hold_config.wz_limit = 1.0f;
    chassis_yaw_hold_init(&yaw_hold_config);

    s_last_arm_swc = 0u;
    s_stop_arm_skip_log_timer = 0u;
    s_arm_stop_done = false;
    s_arm_active = false;
    s_stop_arm_retry_last_ms = 0u;
    s_stop_arm_fail_log_last_ms = 0u;
}

AppControlResult app_control_apply_manual_chassis_pc_arm(void) {
    RemoteState state;
    AppControlResult result = APP_CONTROL_RESULT_OK;

    app_control_mark_arm_active();

    if(!app_control_get_remote_state(&state)) {
        return app_control_stop_all();
    }

    result = app_control_merge_result(result, app_control_apply_remote_chassis(&state));
    result = app_control_merge_result(result, app_control_apply_pc_arm());
    return result;
}

AppControlResult app_control_apply_manual_arm_fs(void) {
    RemoteState state;
    AppControlResult result;

    app_control_mark_arm_active();

    result = app_control_brake_chassis();
    if(!app_control_get_remote_state(&state)) {
        return app_control_merge_result(result, app_control_stop_arm());
    }

    return app_control_merge_result(result, app_control_apply_remote_arm(&state));
}

AppControlResult app_control_apply_auto_pi(void) {
    AppControlResult result = APP_CONTROL_RESULT_OK;

    app_control_mark_arm_active();

    result = app_control_merge_result(result, app_control_apply_pi_yaw());
    result = app_control_merge_result(result, app_control_apply_pi_chassis());
    result = app_control_merge_result(result, app_control_apply_pi_arm());
    return result;
}

AppControlResult app_control_apply_pc_command(PcCommandId command_id) {
    switch(command_id) {
        case PC_COMMAND_STOP:
        case PC_COMMAND_BRAKE:
            return app_control_stop_all();

        case PC_COMMAND_ARM_ENABLE:
            if(!arm.is_ready()) {
                log_warn("APP_CONTROL pc arm_enable skipped: arm not ready");
                return APP_CONTROL_RESULT_SKIPPED;
            }
            return app_control_result_from_arm(arm.enable(), "pc arm_enable");

        case PC_COMMAND_ARM_STOP:
            return app_control_stop_arm();

        case PC_COMMAND_NONE:
        case PC_COMMAND_START:
        case PC_COMMAND_CLEAR_FAULT:
        case PC_COMMAND_ESTOP:
        default:
            return APP_CONTROL_RESULT_SKIPPED;
    }
}

AppControlResult app_control_brake_chassis(void) {
    chassis_yaw_hold_reset();
    return app_control_result_from_chassis(chassis.brake(), "brake chassis");
}

AppControlResult app_control_stop_arm(void) {
    ArmStatus status;

    s_arm_active = false;

    if(s_arm_stop_done) {
        return APP_CONTROL_RESULT_SKIPPED;
    }

    if(!arm.is_ready()) {
        if(delay_nb_ms(&s_stop_arm_skip_log_timer, APP_CONTROL_SKIP_LOG_PERIOD_MS)) {
            log_warn("APP_CONTROL arm stop skipped: arm not ready");
        }
        return APP_CONTROL_RESULT_SKIPPED;
    }

    if(!app_control_interval_due(&s_stop_arm_retry_last_ms, APP_CONTROL_ARM_STOP_RETRY_MS)) {
        return APP_CONTROL_RESULT_SKIPPED;
    }

    status = arm.stop();
    if(status == ARM_OK) {
        s_arm_stop_done = true;
        return APP_CONTROL_RESULT_OK;
    }

    if(app_control_interval_due(&s_stop_arm_fail_log_last_ms, APP_CONTROL_ARM_STOP_LOG_MS)) {
        log_warn("APP_CONTROL stop arm failed: %s", arm.status_str(status));
    }

    return APP_CONTROL_RESULT_ARM_ERROR;
}

AppControlResult app_control_stop_all(void) {
    AppControlResult result = app_control_brake_chassis();
    return app_control_merge_result(result, app_control_stop_arm());
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static AppControlResult app_control_merge_result(AppControlResult lhs, AppControlResult rhs) {
    if(lhs == APP_CONTROL_RESULT_CHASSIS_ERROR || lhs == APP_CONTROL_RESULT_ARM_ERROR ||
       lhs == APP_CONTROL_RESULT_ODOM_ERROR || lhs == APP_CONTROL_RESULT_COMMAND_INVALID ||
       lhs == APP_CONTROL_RESULT_UNSUPPORTED) {
        return lhs;
    }

    if(rhs == APP_CONTROL_RESULT_CHASSIS_ERROR || rhs == APP_CONTROL_RESULT_ARM_ERROR ||
       rhs == APP_CONTROL_RESULT_ODOM_ERROR || rhs == APP_CONTROL_RESULT_COMMAND_INVALID ||
       rhs == APP_CONTROL_RESULT_UNSUPPORTED) {
        return rhs;
    }

    if(lhs == APP_CONTROL_RESULT_OK) {
        return rhs;
    }

    return lhs;
}

static AppControlResult app_control_result_from_chassis(ChassisErrorCode status, const char* action) {
    if(status == CHASSIS_OK) {
        return APP_CONTROL_RESULT_OK;
    }

    log_warn("APP_CONTROL %s failed: %s", action, chassis.error_code_to_str(status));
    return APP_CONTROL_RESULT_CHASSIS_ERROR;
}

static AppControlResult app_control_result_from_arm(ArmStatus status, const char* action) {
    if(status == ARM_OK) {
        return APP_CONTROL_RESULT_OK;
    }

    log_warn("APP_CONTROL %s failed: %s", action, arm.status_str(status));
    return APP_CONTROL_RESULT_ARM_ERROR;
}

static float app_control_limit_abs(float value, float limit) {
    if(value > limit) {
        return limit;
    }
    if(value < -limit) {
        return -limit;
    }
    return value;
}

static float app_control_channel_to_norm(uint16_t value, uint16_t deadband) {
    int32_t diff = (int32_t)value - (int32_t)REMOTE_CENTER;
    float normalized;

    if(diff < 0) {
        if((uint32_t)(-diff) <= deadband) {
            return 0.0f;
        }
    }
    else if((uint32_t)diff <= deadband) {
        return 0.0f;
    }

    normalized = (float)diff / REMOTE_SPAN;
    return app_control_limit_abs(normalized, 1.0f);
}

static RemoteSpeedLimit app_control_get_speed_limit(uint16_t swb) {
    RemoteSpeedLimit limit;

    if(swb == REMOTE_SW_LOW) {
        limit.max_vx = REMOTE_FAST_MAX_VX_MPS;
        limit.max_vy = REMOTE_FAST_MAX_VY_MPS;
        limit.max_wz = REMOTE_FAST_MAX_WZ_RAD_S;
    }
    else if(swb == REMOTE_SW_HIGH) {
        limit.max_vx = REMOTE_SLOW_MAX_VX_MPS;
        limit.max_vy = REMOTE_SLOW_MAX_VY_MPS;
        limit.max_wz = REMOTE_SLOW_MAX_WZ_RAD_S;
    }
    else {
        limit.max_vx = REMOTE_MID_MAX_VX_MPS;
        limit.max_vy = REMOTE_MID_MAX_VY_MPS;
        limit.max_wz = REMOTE_MID_MAX_WZ_RAD_S;
    }

    return limit;
}

static RemoteArmSpeedLimit app_control_get_arm_speed_limit(uint16_t swb) {
    RemoteArmSpeedLimit limit;

    if(swb == REMOTE_SW_LOW) {
        limit.base_end_yaw_rate_rad_s = 50.24f;
        limit.reach_speed_m_s = 1.5f;
        limit.z_speed_m_s = 3.0f;
        limit.end_pitch_rate_rad_s = 21.0f;
        limit.end_yaw_rate_rad_s = 21.0f;
        limit.servo_speed_rad_s = 50.24f;
    }
    else if(swb == REMOTE_SW_HIGH) {
        limit.base_end_yaw_rate_rad_s = 12.56f;
        limit.reach_speed_m_s = 0.5f;
        limit.z_speed_m_s = 1.0f;
        limit.end_pitch_rate_rad_s = 7.0f;
        limit.end_yaw_rate_rad_s = 7.0f;
        limit.servo_speed_rad_s = 12.56f;
    }
    else {
        limit.base_end_yaw_rate_rad_s = 25.12f;
        limit.reach_speed_m_s = 1.0f;
        limit.z_speed_m_s = 2.0f;
        limit.end_pitch_rate_rad_s = 14.0f;
        limit.end_yaw_rate_rad_s = 14.0f;
        limit.servo_speed_rad_s = 25.12f;
    }

    return limit;
}

static bool app_control_get_remote_state(RemoteState* state) {
    if(state == NULL || !remote_get_state(state)) {
        return false;
    }

    return state->online;
}

static AppControlResult app_control_apply_remote_chassis(const RemoteState* state) {
    const uint16_t swb = state->rc_data.channel[REMOTE_CH_SWB];
    const uint16_t swc = state->rc_data.channel[REMOTE_CH_SWC];
    const uint16_t vra = state->rc_data.channel[REMOTE_CH_VRA];
    const uint16_t vrb = state->rc_data.channel[REMOTE_CH_VRB];
    const float ch_right_y = app_control_channel_to_norm(state->rc_data.channel[REMOTE_CH_RIGHT_Y], REMOTE_DEADBAND);
    const float ch_right_x = app_control_channel_to_norm(state->rc_data.channel[REMOTE_CH_RIGHT_X], REMOTE_DEADBAND);
    const float ch_left_x = app_control_channel_to_norm(state->rc_data.channel[REMOTE_CH_LEFT_X], REMOTE_DEADBAND);
    float vx;
    float vy;
    float wz;
    RemoteSpeedLimit speed_limit;
    AppControlResult result = APP_CONTROL_RESULT_OK;

    if(swc == REMOTE_SW_LOW) {
        result = app_control_merge_result(result,
                                          app_control_result_from_chassis(chassis.set_steer_then_drive_enabled(false),
                                                                          "remote set_steer_then_drive disable"));
    }
    else if(swc == REMOTE_SW_CENTER) {
        result = app_control_merge_result(result,
                                          app_control_result_from_chassis(chassis.set_steer_then_drive_enabled(true),
                                                                          "remote set_steer_then_drive enable"));
    }

    if(swc == REMOTE_SW_HIGH || vra <= REMOTE_VR_LOW_THRESHOLD) {
        return app_control_merge_result(result, app_control_brake_chassis());
    }

    if(vrb > REMOTE_VR_LOW_THRESHOLD) {
        chassis_yaw_hold_reset();
        return app_control_merge_result(result,
                                        app_control_result_from_chassis(chassis.set_velocity(0.0f, 0.0f, 0.0f),
                                                                        "remote zero chassis velocity"));
    }

    speed_limit = app_control_get_speed_limit(swb);
    vx = ch_right_y * speed_limit.max_vx;
    vy = -ch_right_x * speed_limit.max_vy;
    wz = -ch_left_x * speed_limit.max_wz;

    if(chassis_yaw_hold_is_active()) {
        Vector3 angle = { 0 };
        Vector3 gyro_corrected = { 0 };

        if(odom.get_angle(&angle) != ODOM_OK || odom.get_gyro_corrected(&gyro_corrected) != ODOM_OK) {
            log_warn("APP_CONTROL remote chassis yaw_hold skipped: odom not ready");
            return APP_CONTROL_RESULT_ODOM_ERROR;
        }

        wz = chassis_yaw_hold_apply(vx, vy, wz, angle.z, gyro_corrected.z, REMOTE_CONTROL_PERIOD_S);
    }

    return app_control_merge_result(result,
                                    app_control_result_from_chassis(chassis.set_velocity(vx, vy, wz),
                                                                    "remote chassis set_velocity"));
}

static AppControlResult app_control_apply_remote_arm(const RemoteState* state) {
    const uint16_t swb = state->rc_data.channel[REMOTE_CH_SWB];
    const uint16_t swc = state->rc_data.channel[REMOTE_CH_SWC];
    const uint16_t vrb = state->rc_data.channel[REMOTE_CH_VRB];
    const float ch_left_x = app_control_channel_to_norm(state->rc_data.channel[REMOTE_CH_LEFT_X], REMOTE_DEADBAND);
    const float ch_right_y = app_control_channel_to_norm(state->rc_data.channel[REMOTE_CH_RIGHT_Y], REMOTE_DEADBAND);
    const float ch_right_x = app_control_channel_to_norm(state->rc_data.channel[REMOTE_CH_RIGHT_X], REMOTE_DEADBAND);
    const RemoteArmSpeedLimit speed_limit = app_control_get_arm_speed_limit(swb);
    const FiveDofArmJointArray* current_joints;
    const FiveDofArmPose* current_pose;
    AppControlResult result = APP_CONTROL_RESULT_SKIPPED;

    if(!arm.is_ready()) {
        s_last_arm_swc = swc;
        log_warn("APP_CONTROL remote arm skipped: arm not ready");
        return APP_CONTROL_RESULT_SKIPPED;
    }

    if(swc == REMOTE_SW_HIGH) {
        AppControlResult result = APP_CONTROL_RESULT_SKIPPED;

        if(s_last_arm_swc != REMOTE_SW_HIGH) {
            result = app_control_result_from_arm(arm.move_servo_zero(speed_limit.servo_speed_rad_s),
                                                 "remote arm move_servo_zero");
        }
        s_last_arm_swc = swc;
        return result;
    }

    if(vrb > REMOTE_VR_LOW_THRESHOLD) {
        s_last_arm_swc = swc;
        return APP_CONTROL_RESULT_SKIPPED;
    }

    current_joints = arm.get_current_joints();
    current_pose = arm.get_current_pose();
    if(current_joints == NULL || current_pose == NULL) {
        s_last_arm_swc = swc;
        log_warn("APP_CONTROL remote arm skipped: arm state unavailable");
        return APP_CONTROL_RESULT_SKIPPED;
    }

    if(swc == REMOTE_SW_LOW) {
        static FiveDofArmJointArray target_joints;

        if(s_last_arm_swc != swc) {
            target_joints = *current_joints;
        }

        target_joints.q[3] = current_joints->q[3] + ch_right_y * speed_limit.end_pitch_rate_rad_s * REMOTE_CONTROL_PERIOD_S;
        target_joints.q[4] = current_joints->q[4] + ch_right_x * speed_limit.end_yaw_rate_rad_s * REMOTE_CONTROL_PERIOD_S;
        s_last_arm_swc = swc;
        return app_control_result_from_arm(arm.move_joints(&target_joints, speed_limit.servo_speed_rad_s),
                                           "remote arm move_joints");
    }

    if(ch_left_x != 0.0f) {
        const float target_base_yaw = current_joints->q[0] + ch_left_x * speed_limit.base_end_yaw_rate_rad_s * REMOTE_CONTROL_PERIOD_S;
        result = app_control_result_from_arm(arm.move_joint(0u, target_base_yaw, speed_limit.servo_speed_rad_s),
                                             "remote arm move_joint");
        s_last_arm_swc = swc;
        if(result != APP_CONTROL_RESULT_OK) {
            return result;
        }
    }

    if(ch_right_y != 0.0f || ch_right_x != 0.0f) {
        const FiveDofArmJointArray* updated_joints = arm.get_current_joints();
        const FiveDofArmPose* updated_pose = arm.get_current_pose();

        if(updated_joints != NULL && updated_pose != NULL) {
            const float base_yaw = updated_joints->q[0];
            const float reach_delta = -ch_right_y * speed_limit.reach_speed_m_s * REMOTE_CONTROL_PERIOD_S;
            const float target_x = updated_pose->position.x + cosf(base_yaw) * reach_delta;
            const float target_y = updated_pose->position.y + sinf(base_yaw) * reach_delta;
            const float target_z = updated_pose->position.z - ch_right_x * speed_limit.z_speed_m_s * REMOTE_CONTROL_PERIOD_S;

            s_last_arm_swc = swc;
            return app_control_merge_result(result,
                                            app_control_result_from_arm(arm.move_position(target_x,
                                                                                          target_y,
                                                                                          target_z,
                                                                                          speed_limit.servo_speed_rad_s),
                                                                        "remote arm move_position"));
        }
    }

    s_last_arm_swc = swc;
    return result;
}

static AppControlResult app_control_apply_pc_arm(void) {
    FiveDofArmJointArray joints;

    if(!arm.is_ready()) {
        log_warn("APP_CONTROL pc arm skipped: arm not ready");
        return APP_CONTROL_RESULT_SKIPPED;
    }

    if(!pc_link_master_joints_is_fresh(APP_CONTROL_PC_JOINT_TIMEOUT_MS)) {
        return APP_CONTROL_RESULT_SKIPPED;
    }

    if(!pc_link_get_master_joints(&joints)) {
        return APP_CONTROL_RESULT_SKIPPED;
    }

    if(!app_control_arm_joints_valid(&joints)) {
        log_warn("APP_CONTROL pc arm rejected: invalid master joints");
        return APP_CONTROL_RESULT_COMMAND_INVALID;
    }

    return app_control_result_from_arm(arm.move_joints(&joints, app_control_get_arm_default_speed()),
                                       "pc arm move_joints");
}

static AppControlResult app_control_apply_pi_chassis(void) {
    PiChassisCommand cmd;
    PiYawCommand yaw_cmd;
    float vx;
    float vy;
    float wz;

    if(!pi_link_chassis_cmd_is_fresh(APP_CONTROL_PI_CHASSIS_TIMEOUT_MS) || !pi_link_get_chassis_cmd(&cmd)) {
        return app_control_brake_chassis();
    }

    vx = app_control_limit_abs(cmd.vx, APP_CONTROL_PI_MAX_VX_MPS);
    vy = app_control_limit_abs(cmd.vy, APP_CONTROL_PI_MAX_VY_MPS);
    wz = app_control_limit_abs(cmd.wz, APP_CONTROL_PI_MAX_WZ_RAD_S);

    if(pi_link_yaw_cmd_is_fresh(APP_CONTROL_PI_YAW_TIMEOUT_MS) &&
       pi_link_get_yaw_cmd(&yaw_cmd) &&
       yaw_cmd.mode == PI_YAW_MODE_RATE_SET &&
       isfinite(yaw_cmd.yaw_rate)) {
        chassis_yaw_hold_disable();
        wz = app_control_limit_abs(yaw_cmd.yaw_rate, APP_CONTROL_PI_MAX_WZ_RAD_S);
    }

    if(chassis_yaw_hold_is_active()) {
        Vector3 angle = { 0 };
        Vector3 gyro_corrected = { 0 };

        if(odom.get_angle(&angle) != ODOM_OK || odom.get_gyro_corrected(&gyro_corrected) != ODOM_OK) {
            log_warn("APP_CONTROL pi chassis yaw_hold failed: odom not ready");
            return APP_CONTROL_RESULT_ODOM_ERROR;
        }

        wz = chassis_yaw_hold_apply(vx, vy, wz, angle.z, gyro_corrected.z, REMOTE_CONTROL_PERIOD_S);
    }

    return app_control_result_from_chassis(chassis.set_velocity(vx, vy, wz), "pi chassis set_velocity");
}

static AppControlResult app_control_apply_pi_yaw(void) {
    PiYawCommand cmd;

    if(!pi_link_take_yaw_cmd(&cmd)) {
        return APP_CONTROL_RESULT_SKIPPED;
    }

    switch(cmd.mode) {
        case PI_YAW_MODE_HOLD_ENABLE: {
            Vector3 angle = { 0 };

            if(odom.get_angle(&angle) != ODOM_OK) {
                log_warn("APP_CONTROL pi yaw hold_enable failed: odom not ready");
                return APP_CONTROL_RESULT_ODOM_ERROR;
            }

            chassis_yaw_hold_set_target(angle.z);
            return APP_CONTROL_RESULT_OK;
        }

        case PI_YAW_MODE_HOLD_DISABLE:
            chassis_yaw_hold_disable();
            return APP_CONTROL_RESULT_OK;

        case PI_YAW_MODE_TARGET_SET:
            if(!isfinite(cmd.target_yaw)) {
                log_warn("APP_CONTROL pi yaw target rejected: invalid target");
                return APP_CONTROL_RESULT_COMMAND_INVALID;
            }
            chassis_yaw_hold_set_target(cmd.target_yaw);
            return APP_CONTROL_RESULT_OK;

        case PI_YAW_MODE_RATE_SET:
            chassis_yaw_hold_disable();
            return APP_CONTROL_RESULT_OK;

        case PI_YAW_MODE_NONE:
        default:
            return APP_CONTROL_RESULT_SKIPPED;
    }
}

static AppControlResult app_control_apply_pi_arm(void) {
    PiArmCommand cmd;

    if(pi_link_take_arm_cmd(&cmd)) {
        if(!arm.is_ready()) {
            log_warn("APP_CONTROL pi arm skipped: arm not ready");
            return APP_CONTROL_RESULT_SKIPPED;
        }

        switch(cmd.type) {
            case PI_ARM_COMMAND_STOP:
                return app_control_stop_arm();

            case PI_ARM_COMMAND_ENABLE:
                return app_control_result_from_arm(arm.enable(), "pi arm enable");

            case PI_ARM_COMMAND_SEQUENCE_ID:
                log_warn("APP_CONTROL pi arm seq unsupported: id=%u", cmd.sequence_id);
                (void)app_control_stop_arm();
                /**
                 * TODO:
                 * ARM:SEQ 当前仅完成协议解析, 尚未接入本地机械臂动作序列执行器
                 */
                return APP_CONTROL_RESULT_UNSUPPORTED;

            case PI_ARM_COMMAND_NONE:
            case PI_ARM_COMMAND_JOINT_TARGET:
            default:
                break;
        }
    }

    if(!pi_link_arm_cmd_is_fresh(APP_CONTROL_PI_ARM_TIMEOUT_MS) || !pi_link_get_arm_cmd(&cmd)) {
        return APP_CONTROL_RESULT_SKIPPED;
    }

    if(!arm.is_ready()) {
        log_warn("APP_CONTROL pi arm skipped: arm not ready");
        return APP_CONTROL_RESULT_SKIPPED;
    }

    switch(cmd.type) {
        case PI_ARM_COMMAND_STOP:
            return app_control_result_from_arm(arm.stop(), "pi arm stop");

        case PI_ARM_COMMAND_ENABLE:
            return app_control_result_from_arm(arm.enable(), "pi arm enable");

        case PI_ARM_COMMAND_JOINT_TARGET:
            if(app_control_arm_joints_valid(&cmd.joints)) {
                float speed_rad_s = cmd.speed_rad_s;

                if(!app_control_arm_speed_valid(speed_rad_s)) {
                    speed_rad_s = app_control_get_arm_default_speed();
                }

                return app_control_result_from_arm(arm.move_joints(&cmd.joints, speed_rad_s),
                                                   "pi arm move_joints");
            }

            log_warn("APP_CONTROL pi arm rejected: invalid joint target");
            return APP_CONTROL_RESULT_COMMAND_INVALID;

        case PI_ARM_COMMAND_SEQUENCE_ID:
            log_warn("APP_CONTROL pi arm seq unsupported: id=%u", cmd.sequence_id);
            (void)app_control_stop_arm();
            /**
             * TODO:
             * ARM:SEQ 当前仅完成协议解析, 尚未接入本地机械臂动作序列执行器
             */
            return APP_CONTROL_RESULT_UNSUPPORTED;

        case PI_ARM_COMMAND_NONE:
        default:
            return APP_CONTROL_RESULT_SKIPPED;
    }
}

static float app_control_get_arm_default_speed(void) {
    const Arm* arm_view = arm.get_arm();

    if(arm_view != NULL && app_control_arm_speed_valid(arm_view->config.default_speed_rad_s)) {
        return arm_view->config.default_speed_rad_s;
    }

    return APP_CONTROL_ARM_FALLBACK_SPEED_RAD_S;
}

static bool app_control_arm_speed_valid(float speed_rad_s) {
    return isfinite(speed_rad_s) && speed_rad_s > 0.0f && speed_rad_s <= APP_CONTROL_ARM_MAX_SPEED_RAD_S;
}

static bool app_control_arm_joints_valid(const FiveDofArmJointArray* joints) {
    const Arm* arm_view = arm.get_arm();
    const SerialArmModel* model = NULL;
    uint8_t i;

    if(joints == NULL || joints->dof < FIVE_DOF_ARM_DOF) {
        return false;
    }

    if(arm_view != NULL && arm_view->config.has_kinematic_model) {
        model = &arm_view->config.kinematic_model;
        if(model->dof < FIVE_DOF_ARM_DOF) {
            return false;
        }
    }

    for(i = 0u; i < FIVE_DOF_ARM_DOF; i++) {
        if(!isfinite(joints->q[i])) {
            return false;
        }

        if(model != NULL &&
           (joints->q[i] < (model->link[i].q_min - APP_CONTROL_ARM_LIMIT_EPS_RAD) ||
            joints->q[i] > (model->link[i].q_max + APP_CONTROL_ARM_LIMIT_EPS_RAD))) {
            return false;
        }
    }

    return true;
}

static bool app_control_interval_due(ms_t* last_ms, uint32_t interval_ms) {
    const ms_t now_ms = delay_now_ms();

    if(last_ms == NULL) {
        return false;
    }

    if(*last_ms == 0u || (now_ms - *last_ms) >= interval_ms) {
        *last_ms = now_ms;
        return true;
    }

    return false;
}

static void app_control_mark_arm_active(void) {
    if(!s_arm_active) {
        s_arm_stop_done = false;
        s_stop_arm_retry_last_ms = 0u;
        s_stop_arm_fail_log_last_ms = 0u;
    }

    s_arm_active = true;
}

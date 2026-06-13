/**
 * @file remote.c
 * @brief 遥控应用层实现
 */
#include "remote.h"

#include "arm.h"
#include "chassis.h"
#include "chassis_yaw_hold.h"
#include "fs_ia10b.h"
#include "odom.h"

#include <math.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 右摇杆横向通道索引
 */
#define REMOTE_CH_RIGHT_X 0u

/**
 * @brief 右摇杆纵向通道索引
 */
#define REMOTE_CH_RIGHT_Y 1u

/**
 * @brief 左摇杆横向通道索引
 */
#define REMOTE_CH_LEFT_X 3u

/**
 * @brief SWA 三挡开关通道索引
 */
#define REMOTE_CH_SWA 4u

/**
 * @brief SWB 三挡开关通道索引
 */
#define REMOTE_CH_SWB 5u

/**
 * @brief SWC 三挡开关通道索引
 */
#define REMOTE_CH_SWC 6u

/**
 * @brief SWD 三挡开关通道索引
 */
#define REMOTE_CH_SWD 7u

/**
 * @brief VRA 旋钮通道索引
 */
#define REMOTE_CH_VRA 8u

/**
 * @brief VRB 旋钮通道索引
 */
#define REMOTE_CH_VRB 9u

/**
 * @brief 遥控器中位原始值
 */
#define REMOTE_CENTER 1500

/**
 * @brief 遥控通道归一化半量程
 */
#define REMOTE_SPAN 500.0f

/**
 * @brief 摇杆输入死区
 */
#define REMOTE_DEADBAND 10u

/**
 * @brief 遥控在线判定超时时间，单位 ms
 */
#define REMOTE_TIMEOUT_MS 100u

/**
 * @brief 遥控处理周期，单位 s
 */
#define REMOTE_CONTROL_PERIOD_S 0.010f

/**
 * @brief 高速档最大 x 向速度，单位 m/s
 */
#define REMOTE_FAST_MAX_VX_MPS 2.0f

/**
 * @brief 高速档最大 y 向速度，单位 m/s
 */
#define REMOTE_FAST_MAX_VY_MPS 2.0f

/**
 * @brief 高速档最大角速度，单位 rad/s
 */
#define REMOTE_FAST_MAX_WZ_RAD_S 8.0f

/**
 * @brief 中速档最大 x 向速度，单位 m/s
 */
#define REMOTE_MID_MAX_VX_MPS 1.0f

/**
 * @brief 中速档最大 y 向速度，单位 m/s
 */
#define REMOTE_MID_MAX_VY_MPS 1.0f

/**
 * @brief 中速档最大角速度，单位 rad/s
 */
#define REMOTE_MID_MAX_WZ_RAD_S 4.0f

/**
 * @brief 低速档最大 x 向速度，单位 m/s
 */
#define REMOTE_SLOW_MAX_VX_MPS 0.5f

/**
 * @brief 低速档最大 y 向速度，单位 m/s
 */
#define REMOTE_SLOW_MAX_VY_MPS 0.5f

/**
 * @brief 低速档最大角速度，单位 rad/s
 */
#define REMOTE_SLOW_MAX_WZ_RAD_S 2.0f

/**
 * @brief 旋钮低位阈值
 */
#define REMOTE_VR_LOW_THRESHOLD 1200u

/**
 * @brief 三挡开关低位原始值
 */
#define REMOTE_SW_LOW 2000u
#define REMOTE_SW_CENTER 1500u

/**
 * @brief 三挡开关高位原始值
 */
#define REMOTE_SW_HIGH 1000u
#define REMOTE_ARM_YAW_JOINT_INDEX 0u

/**
 * @brief 遥控三挡速度上限配置
 */
typedef struct {
    float max_vx;
    float max_vy;
    float max_wz;
} RemoteSpeedLimit;

typedef struct {
    float yaw_rate_rad_s;
    float reach_speed_m_s;
    float z_speed_m_s;
    float pitch_rate_rad_s;
    float servo_speed_rad_s;
} RemoteArmSpeedLimit;

/**
 * @brief 最近一次对外输出的遥控命令快照
 */
static RemoteCommand s_command = { 0 };
static uint16_t s_last_arm_swc = 0u;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 将原始通道值归一化到 [-1, 1]
 * @param value 遥控通道原始值
 * @param deadband 死区阈值
 * @return float 归一化后的通道值
 */
static float channel_to_norm(uint16_t value, uint16_t deadband);

/**
 * @brief 根据 SWB 挡位选择当前速度上限
 * @param swb SWB 通道原始值
 * @return RemoteSpeedLimit 当前挡位的速度上限
 */
static RemoteSpeedLimit get_speed_limit(uint16_t swb);
static RemoteArmSpeedLimit get_arm_speed_limit(uint16_t swb);

static void chassis_control_task(FsIa10bData rc_data);

static void arm_control_task(FsIa10bData rc_data);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化遥控应用层状态
 */
void remote_init(void) {
    ChassisYawHoldConfig yaw_hold_config = chassis_yaw_hold_default_config();

    memset(&s_command, 0, sizeof(s_command));

    yaw_hold_config.kp = 48.0f;
    yaw_hold_config.kd = 2.0f;
    yaw_hold_config.v_deadband = 0.01f;
    yaw_hold_config.wz_deadband = 0.08f;
    yaw_hold_config.wz_limit = 1.0f;
    chassis_yaw_hold_init(&yaw_hold_config);
}

/**
 * @brief 执行一次遥控应用层轮询
 */
void remote_process(void) {
    FsIa10bData rc_data;
    ibus_maintain();

    if(!ibus_get_data(&rc_data) || !ibus_is_online(REMOTE_TIMEOUT_MS) || rc_data.channel[REMOTE_CH_SWD] == REMOTE_SW_HIGH)
        return;

    if(rc_data.channel[REMOTE_CH_SWA] == REMOTE_SW_HIGH)
        chassis_control_task(rc_data);
    else if(rc_data.channel[REMOTE_CH_SWA] == REMOTE_SW_LOW)
        arm_control_task(rc_data);
}

/**
 * @brief 获取最近一次遥控应用层输出的命令
 * @param out 输出命令缓冲区
 * @return bool `true` 表示遥控链路在线
 */
bool remote_get_command(RemoteCommand* out) {
    if(out == NULL) {
        return false;
    }

    *out = s_command;
    return s_command.online;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 将原始通道值归一化到 [-1, 1]
 * @param value 遥控通道原始值
 * @param deadband 死区阈值
 * @return float 归一化后的通道值
 */
static float channel_to_norm(uint16_t value, uint16_t deadband) {
    int32_t diff = (int32_t)value - REMOTE_CENTER;
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
    if(normalized > 1.0f) {
        return 1.0f;
    }
    if(normalized < -1.0f) {
        return -1.0f;
    }

    return normalized;
}

/**
 * @brief 根据 SWB 挡位选择当前速度上限
 * @param swb SWB 通道原始值
 * @return RemoteSpeedLimit 当前挡位的速度上限
 */
static RemoteSpeedLimit get_speed_limit(uint16_t swb) {
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

static RemoteArmSpeedLimit get_arm_speed_limit(uint16_t swb) {
    RemoteArmSpeedLimit limit;

    if(swb == REMOTE_SW_LOW) {
        limit.yaw_rate_rad_s = 1.5f;
        limit.reach_speed_m_s = 0.18f;
        limit.z_speed_m_s = 0.18f;
        limit.pitch_rate_rad_s = 1.5f;
        limit.servo_speed_rad_s = 3.0f;
    }
    else if(swb == REMOTE_SW_HIGH) {
        limit.yaw_rate_rad_s = 0.5f;
        limit.reach_speed_m_s = 0.06f;
        limit.z_speed_m_s = 0.06f;
        limit.pitch_rate_rad_s = 0.5f;
        limit.servo_speed_rad_s = 1.0f;
    }
    else {
        limit.yaw_rate_rad_s = 1.0f;
        limit.reach_speed_m_s = 0.12f;
        limit.z_speed_m_s = 0.12f;
        limit.pitch_rate_rad_s = 1.0f;
        limit.servo_speed_rad_s = 2.0f;
    }

    return limit;
}

static void chassis_control_task(FsIa10bData rc_data) {
    if(rc_data.channel[REMOTE_CH_SWC] == REMOTE_SW_LOW) {
        (void)chassis.set_steer_then_drive_enabled(false);
    }
    else if(rc_data.channel[REMOTE_CH_SWC] == REMOTE_CENTER) {
        (void)chassis.set_steer_then_drive_enabled(true);
    }

    if(rc_data.channel[REMOTE_CH_SWC] == REMOTE_SW_HIGH || rc_data.channel[REMOTE_CH_VRA] <= REMOTE_VR_LOW_THRESHOLD) {
        s_command.vx = 0.0f;
        s_command.vy = 0.0f;
        s_command.wz = 0.0f;
        s_command.online = true;
        chassis_yaw_hold_reset();
        (void)chassis.brake();
        return;
    }

    if(rc_data.channel[REMOTE_CH_VRB] <= REMOTE_VR_LOW_THRESHOLD) {
        RemoteSpeedLimit speed_limit = get_speed_limit(rc_data.channel[REMOTE_CH_SWB]);
        s_command.vx = channel_to_norm(rc_data.channel[REMOTE_CH_RIGHT_Y], REMOTE_DEADBAND) * speed_limit.max_vx;
        s_command.vy = -channel_to_norm(rc_data.channel[REMOTE_CH_RIGHT_X], REMOTE_DEADBAND) * speed_limit.max_vy;
        s_command.wz = -channel_to_norm(rc_data.channel[REMOTE_CH_LEFT_X], REMOTE_DEADBAND) * speed_limit.max_wz;
        s_command.online = true;

        if(chassis_yaw_hold_is_active()) {
            Vector3 angle = { 0.0f, 0.0f, 0.0f };
            Vector3 gyro_corrected = { 0.0f, 0.0f, 0.0f };

            (void)odom.get_angle(&angle);
            (void)odom.get_gyro_corrected(&gyro_corrected);
            s_command.wz = chassis_yaw_hold_apply(
                s_command.vx,
                s_command.vy,
                s_command.wz,
                angle.z,
                gyro_corrected.z,
                REMOTE_CONTROL_PERIOD_S);
        }

        (void)chassis.set_velocity(s_command.vx, s_command.vy, s_command.wz);
    }
    else {
        s_command.online = true;
        s_command.vx = 0.0f;
        s_command.vy = 0.0f;
        s_command.wz = 0.0f;
        chassis_yaw_hold_reset();
        (void)chassis.set_velocity(0.0f, 0.0f, 0.0f);
    }
}

static void arm_control_task(FsIa10bData rc_data) {
    const RemoteArmSpeedLimit speed_limit = get_arm_speed_limit(rc_data.channel[REMOTE_CH_SWB]);
    const uint16_t swc = rc_data.channel[REMOTE_CH_SWC];
    const float yaw_input = channel_to_norm(rc_data.channel[REMOTE_CH_LEFT_X], REMOTE_DEADBAND);
    const float reach_input = channel_to_norm(rc_data.channel[REMOTE_CH_RIGHT_Y], REMOTE_DEADBAND);
    const float z_input = channel_to_norm(rc_data.channel[REMOTE_CH_RIGHT_X], REMOTE_DEADBAND);
    const FiveDofArmJointArray* current_joints;
    const FiveDofArmPose* current_pose;
    SerialArmRPY rpy;

    if(!arm.is_ready()) {
        s_last_arm_swc = swc;
        return;
    }

    if(rc_data.channel[REMOTE_CH_VRB] > REMOTE_VR_LOW_THRESHOLD) {
        s_last_arm_swc = swc;
        return;
    }

    if(swc == REMOTE_SW_HIGH) {
        if(s_last_arm_swc != REMOTE_SW_HIGH) {
            (void)arm.move_servo_zero(speed_limit.servo_speed_rad_s);
        }
        s_last_arm_swc = swc;
        return;
    }

    current_joints = arm.get_current_joints();
    current_pose = arm.get_current_pose();
    if(current_joints == NULL || current_pose == NULL) {
        s_last_arm_swc = swc;
        return;
    }

    if(swc == REMOTE_SW_LOW) {
        if(serial_arm.quat_to_rpy(current_pose->orientation, &rpy) == SERIAL_ARM_STATUS_SUCCESS) {
            const float target_yaw = rpy.yaw - 0.1 * reach_input * speed_limit.yaw_rate_rad_s * REMOTE_CONTROL_PERIOD_S;
            (void)arm.move_orientation(rpy.roll, rpy.pitch, target_yaw, speed_limit.servo_speed_rad_s);
        }
        s_last_arm_swc = swc;
        return;
    }

    if(yaw_input != 0.0f) {
        const float target_yaw = current_joints->q[REMOTE_ARM_YAW_JOINT_INDEX] - yaw_input * speed_limit.yaw_rate_rad_s * REMOTE_CONTROL_PERIOD_S;
        (void)arm.move_joint(REMOTE_ARM_YAW_JOINT_INDEX, target_yaw, speed_limit.servo_speed_rad_s);
    }

    if(reach_input != 0.0f || z_input != 0.0f) {
        const FiveDofArmJointArray* updated_joints = arm.get_current_joints();
        const FiveDofArmPose* updated_pose = arm.get_current_pose();

        if(updated_joints != NULL && updated_pose != NULL) {
            const float base_yaw = updated_joints->q[REMOTE_ARM_YAW_JOINT_INDEX];
            const float reach_delta = reach_input * speed_limit.reach_speed_m_s * REMOTE_CONTROL_PERIOD_S;
            const float target_x = updated_pose->position.x + cosf(base_yaw) * reach_delta;
            const float target_y = updated_pose->position.y + sinf(base_yaw) * reach_delta;
            const float target_z = updated_pose->position.z - z_input * speed_limit.z_speed_m_s * REMOTE_CONTROL_PERIOD_S;

            (void)arm.move_position(target_x, target_y, target_z, speed_limit.servo_speed_rad_s);
        }
    }

    s_last_arm_swc = swc;
}

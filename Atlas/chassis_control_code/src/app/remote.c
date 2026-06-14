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

/**
 * @brief 三挡开关中位原始值
 */
#define REMOTE_SW_CENTER 1500u

/**
 * @brief 三挡开关高位原始值
 */
#define REMOTE_SW_HIGH 1000u

/**
 * @brief 遥控三挡速度上限配置
 */
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

/**
 * @brief 最近一次对外输出的遥控命令快照
 */
static RemoteCommand s_command = { 0 };

/**
 * @brief 最近一次 SWC 挡位，用于检测挡位切换
 */
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
    if(!ibus_get_data(&rc_data) || !ibus_is_online(REMOTE_TIMEOUT_MS) || rc_data.channel[REMOTE_CH_SWD] == REMOTE_SW_HIGH) {
        s_command.online = false;
        return;
    }

    s_command.online = true;
    const uint16_t swa = rc_data.channel[REMOTE_CH_SWA];

    if(swa == REMOTE_SW_HIGH)
        chassis_control_task(rc_data);
    else if(swa == REMOTE_SW_LOW)
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

/**
 * @brief 获取机械臂三挡速度限制
 * @param swb ch_swb 当前档位
 * @return 当前档位对应的速度限制
 */
static RemoteArmSpeedLimit get_arm_speed_limit(uint16_t swb) {
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

/**
 * @brief 机械底盘控制任务
 * @param rc_data 当前遥控数据
 */
static void chassis_control_task(FsIa10bData rc_data) {
    const uint16_t swb = rc_data.channel[REMOTE_CH_SWB];
    const uint16_t swc = rc_data.channel[REMOTE_CH_SWC];
    const uint16_t vra = rc_data.channel[REMOTE_CH_VRA];
    const uint16_t vrb = rc_data.channel[REMOTE_CH_VRB];
    const float ch_right_y = channel_to_norm(rc_data.channel[REMOTE_CH_RIGHT_Y], REMOTE_DEADBAND);
    const float ch_right_x = channel_to_norm(rc_data.channel[REMOTE_CH_RIGHT_X], REMOTE_DEADBAND);
    const float ch_left_x = channel_to_norm(rc_data.channel[REMOTE_CH_LEFT_X], REMOTE_DEADBAND);

    /* SWC: 底盘模式选择 */
    if(swc == REMOTE_SW_LOW) {
        (void)chassis.set_steer_then_drive_enabled(false);
    }
    else if(swc == REMOTE_CENTER) {
        (void)chassis.set_steer_then_drive_enabled(true);
    }

    /* SWC 高位或 VRA 关闭时，直接刹车退出 */
    if(swc == REMOTE_SW_HIGH || vra <= REMOTE_VR_LOW_THRESHOLD) {
        s_command.vx = 0.0f;
        s_command.vy = 0.0f;
        s_command.wz = 0.0f;
        chassis_yaw_hold_reset();
        (void)chassis.brake();
        return;
    }

    /* VRB 高位时不允许底盘运动 */
    if(vrb > REMOTE_VR_LOW_THRESHOLD) {
        s_command.vx = 0.0f;
        s_command.vy = 0.0f;
        s_command.wz = 0.0f;
        chassis_yaw_hold_reset();
        (void)chassis.set_velocity(0.0f, 0.0f, 0.0f);
        return;
    }

    /* 摇杆映射到底盘速度 */
    {
        const RemoteSpeedLimit speed_limit = get_speed_limit(swb);

        s_command.vx = ch_right_y * speed_limit.max_vx;
        s_command.vy = -ch_right_x * speed_limit.max_vy;
        s_command.wz = -ch_left_x * speed_limit.max_wz;
    }

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

/**
 * @brief 机械臂控制任务
 * @param rc_data 当前遥控数据
 */
static void arm_control_task(FsIa10bData rc_data) {
    const uint16_t swb = rc_data.channel[REMOTE_CH_SWB];
    const uint16_t swc = rc_data.channel[REMOTE_CH_SWC];
    const uint16_t vrb = rc_data.channel[REMOTE_CH_VRB];
    const RemoteArmSpeedLimit speed_limit = get_arm_speed_limit(swb);
    const float ch_left_x = channel_to_norm(rc_data.channel[REMOTE_CH_LEFT_X], REMOTE_DEADBAND);
    const float ch_right_y = channel_to_norm(rc_data.channel[REMOTE_CH_RIGHT_Y], REMOTE_DEADBAND);
    const float ch_right_x = channel_to_norm(rc_data.channel[REMOTE_CH_RIGHT_X], REMOTE_DEADBAND);
    const FiveDofArmJointArray* current_joints;
    const FiveDofArmPose* current_pose;

    if(!arm.is_ready()) {
        s_last_arm_swc = swc;
        return;
    }

    /* SWC 高位: 回机械臂零位 */
    if(swc == REMOTE_SW_HIGH) {
        if(s_last_arm_swc != REMOTE_SW_HIGH) {
            (void)arm.move_servo_zero(speed_limit.servo_speed_rad_s);
        }
        s_last_arm_swc = swc;
        return;
    }

    /* VRB 高位时不处理机械臂动作 */
    if(vrb > REMOTE_VR_LOW_THRESHOLD) {
        s_last_arm_swc = swc;
        return;
    }

    current_joints = arm.get_current_joints();
    current_pose = arm.get_current_pose();
    if(current_joints == NULL || current_pose == NULL) {
        s_last_arm_swc = swc;
        return;
    }

    /* SWC 低位: 末端关节模式，右杆直接控制腕部 pitch/yaw */
    if(swc == REMOTE_SW_LOW) {
        static FiveDofArmJointArray target_joints;

        if(s_last_arm_swc != swc)
            target_joints = *current_joints;

        target_joints.q[3] = current_joints->q[3] + ch_right_y * speed_limit.end_pitch_rate_rad_s * REMOTE_CONTROL_PERIOD_S;
        target_joints.q[4] = current_joints->q[4] + ch_right_x * speed_limit.end_yaw_rate_rad_s * REMOTE_CONTROL_PERIOD_S;
        (void)arm.move_joints(&target_joints, speed_limit.servo_speed_rad_s);

        s_last_arm_swc = swc;
        return;
    }

    /* SWC 中位: 左杆 X 控底座偏航 */
    if(ch_left_x != 0.0f) {
        const float target_base_yaw = current_joints->q[0] + ch_left_x * speed_limit.base_end_yaw_rate_rad_s * REMOTE_CONTROL_PERIOD_S;
        (void)arm.move_joint(0, target_base_yaw, speed_limit.servo_speed_rad_s);
    }

    /* SWC 中位: 右杆控制工作空间中的前伸和升降 */
    if(ch_right_y != 0.0f || ch_right_x != 0.0f) {
        const FiveDofArmJointArray* updated_joints = arm.get_current_joints();
        const FiveDofArmPose* updated_pose = arm.get_current_pose();

        if(updated_joints != NULL && updated_pose != NULL) {
            const float base_yaw = updated_joints->q[0];
            const float reach_delta = -ch_right_y * speed_limit.reach_speed_m_s * REMOTE_CONTROL_PERIOD_S;
            const float target_x = updated_pose->position.x + cosf(base_yaw) * reach_delta;
            const float target_y = updated_pose->position.y + sinf(base_yaw) * reach_delta;
            const float target_z = updated_pose->position.z - ch_right_x * speed_limit.z_speed_m_s * REMOTE_CONTROL_PERIOD_S;

            (void)arm.move_position(target_x, target_y, target_z, speed_limit.servo_speed_rad_s);
        }
    }

    s_last_arm_swc = swc;
}

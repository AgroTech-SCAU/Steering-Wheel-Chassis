/**
 * @file chassis_yaw_hold.c
 * @brief 底盘航向保持实现
 */
#include "chassis_yaw_hold.h"

#include "pid.h"

#include <math.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 圆周率常量
 */
#define CHASSIS_YAW_HOLD_PI      3.14159265358979323846f

/**
 * @brief 两倍圆周率常量
 */
#define CHASSIS_YAW_HOLD_2PI     (2.0f * CHASSIS_YAW_HOLD_PI)

/**
 * @brief 航向保持模块内部状态
 */
typedef struct {
    ChassisYawHoldConfig config;
    Pid pd;
    bool initialized;
    bool active;
    float yaw_ref;
    float last_yaw_error;
    float last_fb_wz;
    float last_output_wz;
} ChassisYawHoldState;

/**
 * @brief 航向保持模块全局状态实例
 */
static ChassisYawHoldState s_yaw_hold = { 0 };

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 将角度归一化到 [-pi, pi)
 * @param angle 原始角度，单位 rad
 * @return float 归一化后的角度，单位 rad
 */
static float chassis_yaw_hold_wrap_pi(float angle);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 获取默认航向保持配置
 * @return ChassisYawHoldConfig 默认配置
 */
ChassisYawHoldConfig chassis_yaw_hold_default_config(void) {
    ChassisYawHoldConfig config;

    config.enabled = true;
    config.kp = 48.0f;
    config.kd = 2.0f;
    config.v_deadband = 0.01f;
    config.wz_deadband = 0.08f;
    config.wz_limit = 1.0f;
    return config;
}

/**
 * @brief 初始化航向保持模块
 * @param config 配置指针，传入 `NULL` 使用默认配置
 */
void chassis_yaw_hold_init(const ChassisYawHoldConfig* config) {
    ChassisYawHoldConfig default_config = chassis_yaw_hold_default_config();

    memset(&s_yaw_hold, 0, sizeof(s_yaw_hold));
    s_yaw_hold.config = (config != NULL) ? *config : default_config;
    pid_init(&s_yaw_hold.pd, PID_MODE_P, PID_FEAT_OUTPUT_LIMIT);
    pid_set_gains(&s_yaw_hold.pd, s_yaw_hold.config.kp, 0.0f, s_yaw_hold.config.kd);
    pid_set_params(&s_yaw_hold.pd, s_yaw_hold.config.wz_limit, 0.0f, 0.0f, 0.0f, 0.0f);
    s_yaw_hold.initialized = true;
}

/**
 * @brief 设置航向保持目标角
 * @param yaw_ref 目标 yaw，单位 rad
 */
void chassis_yaw_hold_set_target(float yaw_ref) {
    if(!s_yaw_hold.initialized) {
        chassis_yaw_hold_init(NULL);
    }

    s_yaw_hold.yaw_ref = chassis_yaw_hold_wrap_pi(yaw_ref);
    s_yaw_hold.active = true;
    s_yaw_hold.last_yaw_error = 0.0f;
    s_yaw_hold.last_fb_wz = 0.0f;
    s_yaw_hold.last_output_wz = 0.0f;
    pid_reset(&s_yaw_hold.pd);
}

/**
 * @brief 关闭航向保持
 */
void chassis_yaw_hold_disable(void) {
    if(!s_yaw_hold.initialized) {
        chassis_yaw_hold_init(NULL);
    }

    s_yaw_hold.active = false;
    s_yaw_hold.yaw_ref = 0.0f;
    s_yaw_hold.last_yaw_error = 0.0f;
    s_yaw_hold.last_fb_wz = 0.0f;
    s_yaw_hold.last_output_wz = 0.0f;
    pid_reset(&s_yaw_hold.pd);
}

/**
 * @brief 重置航向保持内部状态
 */
void chassis_yaw_hold_reset(void) {
    s_yaw_hold.last_yaw_error = 0.0f;
    s_yaw_hold.last_fb_wz = 0.0f;
    s_yaw_hold.last_output_wz = 0.0f;
    pid_reset(&s_yaw_hold.pd);
}

/**
 * @brief 应用航向保持角度环
 * @param vx_cmd 底盘 x 方向速度命令，单位 m/s
 * @param vy_cmd 底盘 y 方向速度命令，单位 m/s
 * @param wz_cmd 用户原始旋转命令，单位 rad/s
 * @param yaw 当前 yaw，单位 rad
 * @param gyro_z_corrected 当前 z 轴角速度，单位 rad/s
 * @param dt_s 控制周期，单位 s
 * @return float 叠加角度环后的目标角速度
 */
float chassis_yaw_hold_apply(float vx_cmd, float vy_cmd, float wz_cmd, float yaw, float gyro_z_corrected, float dt_s) {
    const ChassisYawHoldConfig* config = &s_yaw_hold.config;
    bool user_rotating;
    bool user_translating;
    float yaw_error;
    float p_wz;
    float d_wz;

    if(!s_yaw_hold.initialized) {
        chassis_yaw_hold_init(NULL);
    }

    if(!config->enabled || !isfinite(yaw) || !isfinite(gyro_z_corrected)
        || dt_s <= 0.0f || dt_s > 0.05f) {
        s_yaw_hold.last_yaw_error = 0.0f;
        s_yaw_hold.last_fb_wz = 0.0f;
        s_yaw_hold.last_output_wz = wz_cmd;
        pid_reset(&s_yaw_hold.pd);
        return wz_cmd;
    }

    user_rotating = fabsf(wz_cmd) > config->wz_deadband;
    user_translating = fabsf(vx_cmd) > config->v_deadband || fabsf(vy_cmd) > config->v_deadband;

    if(user_rotating) {
        s_yaw_hold.last_yaw_error = 0.0f;
        s_yaw_hold.last_fb_wz = 0.0f;
        s_yaw_hold.last_output_wz = wz_cmd;
        pid_reset(&s_yaw_hold.pd);
        return wz_cmd;
    }

    if(!user_translating || !s_yaw_hold.active) {
        s_yaw_hold.last_yaw_error = 0.0f;
        s_yaw_hold.last_fb_wz = 0.0f;
        s_yaw_hold.last_output_wz = wz_cmd;
        pid_reset(&s_yaw_hold.pd);
        return wz_cmd;
    }

    yaw_error = chassis_yaw_hold_wrap_pi(s_yaw_hold.yaw_ref - yaw);
    p_wz = pid_calculate(&s_yaw_hold.pd, yaw_error, 0.0f, dt_s);
    d_wz = -config->kd * gyro_z_corrected;
    s_yaw_hold.last_fb_wz = p_wz + d_wz;
    if(s_yaw_hold.last_fb_wz > config->wz_limit) {
        s_yaw_hold.last_fb_wz = config->wz_limit;
    }
    else if(s_yaw_hold.last_fb_wz < -config->wz_limit) {
        s_yaw_hold.last_fb_wz = -config->wz_limit;
    }
    s_yaw_hold.last_yaw_error = yaw_error;
    s_yaw_hold.last_output_wz = wz_cmd + s_yaw_hold.last_fb_wz;
    return s_yaw_hold.last_output_wz;
}

/**
 * @brief 查询航向保持是否处于激活状态
 * @return bool `true` 表示已经锁定参考 yaw
 */
bool chassis_yaw_hold_is_active(void) {
    return s_yaw_hold.active;
}

/**
 * @brief 获取当前参考 yaw
 * @return float 当前参考 yaw，单位 rad
 */
float chassis_yaw_hold_get_yaw_ref(void) {
    return s_yaw_hold.yaw_ref;
}

/**
 * @brief 获取最近一次角度误差
 * @return float 最近一次 yaw 误差，单位 rad
 */
float chassis_yaw_hold_get_yaw_error(void) {
    return s_yaw_hold.last_yaw_error;
}

/**
 * @brief 获取最近一次角度环反馈输出
 * @return float 最近一次反馈输出，单位 rad/s
 */
float chassis_yaw_hold_get_fb_wz(void) {
    return s_yaw_hold.last_fb_wz;
}

/**
 * @brief 获取最近一次最终输出
 * @return float 最近一次输出角速度，单位 rad/s
 */
float chassis_yaw_hold_get_output_wz(void) {
    return s_yaw_hold.last_output_wz;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 将角度归一化到 [-pi, pi)
 * @param angle 原始角度，单位 rad
 * @return float 归一化后的角度，单位 rad
 */
static float chassis_yaw_hold_wrap_pi(float angle) {
    if(!isfinite(angle)) {
        return 0.0f;
    }

    angle = fmodf(angle, CHASSIS_YAW_HOLD_2PI);
    if(angle >= CHASSIS_YAW_HOLD_PI) {
        angle -= CHASSIS_YAW_HOLD_2PI;
    }
    else if(angle < -CHASSIS_YAW_HOLD_PI) {
        angle += CHASSIS_YAW_HOLD_2PI;
    }

    return angle;
}

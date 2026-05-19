#include "steer_wheel_kine.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define sw steer_wheel_interface

#define SW_PI 3.14159265358979323846f
#define SW_2PI (2.0f * SW_PI)
#define SW_HALF_PI (0.5f * SW_PI)
#define SW_EPS 1e-6f
#define SW_OPTIMIZE_HYSTERESIS_RAD 0.08f

/**
 * @brief 舵轮运动学入口单例定义表
 */
#define X(name, str) .name = STEER_WHEEL_##name,
const struct SteerWheelInterface steer_wheel_interface = {
    {
        STEER_WHEEL_STATUS_TABLE
    },
    .init = steer_wheel_init,
    .fk = steer_wheel_fk,
    .ik = steer_wheel_ik,
    .error_code_to_str = steer_wheel_error_code_to_str
};
#undef X

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static float sw_absf(float x);
static float sw_wrap_pi(float angle);
static void sw_get_wheel_pos(const SteerWheelModel* model, float x[4], float y[4]);
static void sw_optimize_module(float* target_angle, float* target_speed, float current_angle);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化舵轮运动学实例
 * @param steer_wheel 舵轮运动学实例指针
 * @param model 底盘模型参数
 * @return SteelWheelErrorCode 错误码
 */
SteelWheelErrorCode steer_wheel_init(SteerWheel* steer_wheel, SteerWheelModel model) {
    if(steer_wheel == NULL) return sw.INVALID_PARAM;
    if(model.length <= 0.0f || model.width <= 0.0f || model.wheel_radius <= 0.0f || model.max_wheel_linear_speed < 0.0f) return sw.INVALID_MODEL;

    steer_wheel->model = model;

    for(uint8_t i = 0; i < 4; ++i) {
        steer_wheel->control.wheels[i].wheel_omega = 0.0f;
        steer_wheel->control.wheels[i].steer_angle = 0.0f;
        steer_wheel->state.cur_wheels[i].wheel_omega = 0.0f;
        steer_wheel->state.cur_wheels[i].steer_angle = 0.0f;
    }

    steer_wheel->control.vx = 0.0f;
    steer_wheel->control.vy = 0.0f;
    steer_wheel->control.wz = 0.0f;
    steer_wheel->state.cur_vx = 0.0f;
    steer_wheel->state.cur_vy = 0.0f;
    steer_wheel->state.cur_wz = 0.0f;
    steer_wheel->initialized = true;

    return sw.OK;
}

/**
 * @brief 正运动学解算，由四个轮模块反馈解算底盘速度
 * @param steer_wheel 舵轮运动学实例指针
 * @return SteelWheelErrorCode 错误码
 */
SteelWheelErrorCode steer_wheel_fk(SteerWheel* steer_wheel) {
    if(steer_wheel == NULL) return sw.INVALID_PARAM;
    if(steer_wheel->initialized == false) return sw.NOT_INITIALIZE;
    if(steer_wheel->model.length <= 0.0f || steer_wheel->model.width <= 0.0f || steer_wheel->model.wheel_radius <= 0.0f) return sw.INVALID_MODEL;

    float x[4];
    float y[4];
    sw_get_wheel_pos(&steer_wheel->model, x, y);

    float sum_vix = 0.0f;
    float sum_viy = 0.0f;
    float sum_w_numer = 0.0f;
    float sum_w_denom = 0.0f;

    for(uint8_t i = 0; i < 4; ++i) {
        const float wheel_linear_speed = steer_wheel->state.cur_wheels[i].wheel_omega * steer_wheel->model.wheel_radius;
        const float steer_angle = steer_wheel->state.cur_wheels[i].steer_angle;

        const float vix = wheel_linear_speed * cosf(steer_angle);
        const float viy = wheel_linear_speed * sinf(steer_angle);

        sum_vix += vix;
        sum_viy += viy;
        sum_w_numer += (-y[i] * vix + x[i] * viy);
        sum_w_denom += (x[i] * x[i] + y[i] * y[i]);
    }

    steer_wheel->state.cur_vx = sum_vix / (float)4;
    steer_wheel->state.cur_vy = sum_viy / (float)4;
    steer_wheel->state.cur_wz = (sum_w_denom > SW_EPS) ? (sum_w_numer / sum_w_denom) : 0.0f;

    return sw.OK;
}

/**
 * @brief 逆运动学解算，由底盘速度指令解算四个轮模块目标
 * @param steer_wheel 舵轮运动学实例指针
 * @return SteelWheelErrorCode 错误码
 */
SteelWheelErrorCode steer_wheel_ik(SteerWheel* steer_wheel) {
    if(steer_wheel == NULL) return sw.INVALID_PARAM;
    if(steer_wheel->initialized == false) return sw.NOT_INITIALIZE;
    if(steer_wheel->model.length <= 0.0f || steer_wheel->model.width <= 0.0f || steer_wheel->model.wheel_radius <= 0.0f) return sw.INVALID_MODEL;

    float x[4];
    float y[4];
    sw_get_wheel_pos(&steer_wheel->model, x, y);

    const float vx = steer_wheel->control.vx;
    const float vy = steer_wheel->control.vy;
    const float wz = steer_wheel->control.wz;

    float max_abs_linear_speed = 0.0f;

    for(uint8_t i = 0; i < 4; ++i) {
        const float vix = vx - wz * y[i];
        const float viy = vy + wz * x[i];

        float target_linear_speed = sqrtf(vix * vix + viy * viy);
        float target_steer_angle;

        if(target_linear_speed < SW_EPS) {
            target_linear_speed = 0.0f;
            target_steer_angle = steer_wheel->state.cur_wheels[i].steer_angle;
        }
        else target_steer_angle = atan2f(viy, vix);

        sw_optimize_module(&target_steer_angle, &target_linear_speed, steer_wheel->state.cur_wheels[i].steer_angle);

        steer_wheel->control.wheels[i].wheel_omega = target_linear_speed / steer_wheel->model.wheel_radius;
        steer_wheel->control.wheels[i].steer_angle = target_steer_angle;

        if(sw_absf(target_linear_speed) > max_abs_linear_speed) max_abs_linear_speed = sw_absf(target_linear_speed);
    }

    if(steer_wheel->model.max_wheel_linear_speed > SW_EPS && max_abs_linear_speed > steer_wheel->model.max_wheel_linear_speed) {
        const float scale = steer_wheel->model.max_wheel_linear_speed / max_abs_linear_speed;

        for(uint8_t i = 0; i < 4; ++i) steer_wheel->control.wheels[i].wheel_omega *= scale;
    }

    return sw.OK;
}

/**
 * @brief 舵轮运动学错误码转字符串
 * @param status 错误码
 * @return const char* 错误码字符串
 */
#define X(name, str) case STEER_WHEEL_##name: return str;
const char* steer_wheel_error_code_to_str(SteelWheelErrorCode status) {
    switch(status) {
        STEER_WHEEL_STATUS_TABLE
        default: return "UNKNOWN";
    }
}
#undef X

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 计算 float 绝对值
 * @param x 输入值
 * @return float 绝对值
 */
static float sw_absf(float x) {
    return (x >= 0.0f) ? x : -x;
}

/**
 * @brief 将角度归一化到 (-pi, pi]
 * @param angle 输入角度，单位 rad
 * @return float 归一化后的角度，单位 rad
 */
static float sw_wrap_pi(float angle) {
    while(angle > SW_PI) {
        angle -= SW_2PI;
    }
    while(angle <= -SW_PI) {
        angle += SW_2PI;
    }
    return angle;
}

/**
 * @brief 根据底盘模型计算四个轮模块相对底盘中心的位置
 * @param model 底盘模型参数
 * @param x 输出四个轮模块 x 坐标，单位 m，顺序为 FL、FR、RL、RR
 * @param y 输出四个轮模块 y 坐标，单位 m，顺序为 FL、FR、RL、RR
 */
static void sw_get_wheel_pos(const SteerWheelModel* model, float x[4], float y[4]) {
    const float hx = model->length * 0.5f;
    const float hy = model->width * 0.5f;

    /* 约定顺序: FL, FR, RR, RL */
    x[0] = hx;  y[0] = -hy;
    x[1] = hx;  y[1] = hy;
    x[2] = -hx;  y[2] = hy;
    x[3] = -hx;  y[3] = -hy;
}

/**
 * @brief 优化单个舵轮目标角和目标速度，避免舵向旋转超过 90 度
 * @param target_angle 目标舵向角输入输出，单位 rad
 * @param target_speed 目标轮线速度输入输出，单位 m/s
 * @param current_angle 当前舵向角，单位 rad
 */
static void sw_optimize_module(float* target_angle, float* target_speed, float current_angle) {
    float err = sw_wrap_pi(*target_angle - current_angle);

    if(sw_absf(err) > (SW_HALF_PI + SW_OPTIMIZE_HYSTERESIS_RAD)) {
        *target_angle = sw_wrap_pi(*target_angle + SW_PI);
        *target_speed = -*target_speed;
    }
}

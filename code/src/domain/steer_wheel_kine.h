#ifndef _steer_wheel_kine_h_
#define _steer_wheel_kine_h_

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 舵轮运动学入口单例，用户自定义名称
 */
#define swheel steer_wheel_interface

/**
 * @brief 舵轮运动学状态码表，使用 X-Macro 定义，方便维护和扩展
 * @param OK 操作成功
 * @param INVALID_PARAM 参数无效
 * @param INVALID_MODEL 车型模型参数无效
 * @param NOT_INITIALIZE 舵轮运动学实例未初始化
 */
#define STEER_WHEEL_STATUS_TABLE \
    X(OK, "OK") \
    X(INVALID_PARAM, "Invalid Parameter") \
    X(INVALID_MODEL, "Invalid Model") \
    X(NOT_INITIALIZE, "Not Initialize")

/**
 * @brief 舵轮运动学错误码
 */
#define X(name, str) STEER_WHEEL_##name,
typedef enum {
    STEER_WHEEL_STATUS_TABLE
} SteelWheelErrorCode;
#undef X

/**
 * @brief 单个舵轮模块输出或反馈
 * @param wheel_omega 车轮角速度，单位 rad/s
 * @param steer_angle 舵向角，单位 rad
 */
typedef struct {
    float wheel_omega;
    float steer_angle;
} WheelModule;

/**
 * @brief 舵轮底盘模型参数
 * @param length 前后轮距，单位 m
 * @param width 左右轮距，单位 m
 * @param wheel_radius 车轮半径，单位 m
 * @param max_wheel_linear_speed 单个车轮最大线速度，单位 m/s，0 表示不启用限幅
 */
typedef struct {
    float length;
    float width;
    float wheel_radius;
    float max_wheel_linear_speed;
} SteerWheelModel;

/**
 * @brief 舵轮底盘控制输入和 IK 输出
 * @param wheels 四个舵轮目标输出，顺序为 FL、FR、RL、RR
 * @param vx 底盘目标 x 方向线速度，单位 m/s
 * @param vy 底盘目标 y 方向线速度，单位 m/s
 * @param wz 底盘目标 z 轴角速度，单位 rad/s
 */
typedef struct {
    WheelModule wheels[4];
    float vx;
    float vy;
    float wz;
} SteerWheelControl;

/**
 * @brief 舵轮底盘反馈状态和 FK 输出
 * @param cur_wheels 四个舵轮当前反馈，顺序为 FL、FR、RL、RR
 * @param cur_vx 底盘当前 x 方向线速度，单位 m/s
 * @param cur_vy 底盘当前 y 方向线速度，单位 m/s
 * @param cur_wz 底盘当前 z 轴角速度，单位 rad/s
 */
typedef struct {
    WheelModule cur_wheels[4];
    float cur_vx;
    float cur_vy;
    float cur_wz;
} SteerWheelState;

/**
 * @brief 舵轮运动学实例数据
 * @param model 底盘模型参数
 * @param control 控制输入和 IK 输出
 * @param state 反馈状态和 FK 输出
 * @param initialized 初始化标志，true 表示已初始化
 */
typedef struct {
    SteerWheelModel model;
    SteerWheelControl control;
    SteerWheelState state;
    bool initialized;
} SteerWheel;

/**
 * @brief 舵轮运动学入口接口
 */
#define X(name, str) SteelWheelErrorCode name;
extern const struct SteerWheelInterface {
    struct {
        STEER_WHEEL_STATUS_TABLE
    };
    /**
     * @brief 初始化舵轮运动学实例
     * @param steer_wheel 舵轮运动学实例指针
     * @param model 底盘模型参数
     * @return SteelWheelErrorCode 错误码
     */
    SteelWheelErrorCode(*init)(SteerWheel* steer_wheel, SteerWheelModel model);
    /**
     * @brief 正运动学解算，由四个轮模块反馈解算底盘速度
     * @param steer_wheel 舵轮运动学实例指针
     * @return SteelWheelErrorCode 错误码
     */
    SteelWheelErrorCode(*fk)(SteerWheel* steer_wheel);
    /**
     * @brief 逆运动学解算，由底盘速度指令解算四个轮模块目标
     * @param steer_wheel 舵轮运动学实例指针
     * @return SteelWheelErrorCode 错误码
     */
    SteelWheelErrorCode(*ik)(SteerWheel* steer_wheel);
    /**
     * @brief 舵轮运动学错误码转字符串
     * @param status 错误码
     * @return const char* 错误码字符串
     */
    const char* (*error_code_to_str)(SteelWheelErrorCode status);
} steer_wheel_interface;
#undef X

// ! ========================= 接 口 函 数 声 明 ========================= ! //

SteelWheelErrorCode steer_wheel_init(SteerWheel* steer_wheel, SteerWheelModel model);
SteelWheelErrorCode steer_wheel_fk(SteerWheel* steer_wheel);
SteelWheelErrorCode steer_wheel_ik(SteerWheel* steer_wheel);
const char* steer_wheel_error_code_to_str(SteelWheelErrorCode status);

#endif

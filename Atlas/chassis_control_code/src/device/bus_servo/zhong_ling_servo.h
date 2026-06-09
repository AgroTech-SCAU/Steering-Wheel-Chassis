#ifndef _zhong_ling_servo_h_
#define _zhong_ling_servo_h_

#include "bus_servo.h"

#include <stddef.h>

// ! ========================= 接 口 类 型 / Typedef 声 明 ========================= ! //

/**
 * @brief 众灵 ZL-IS2 特色入口单例
 */
#define zhong_ling_servo (*zhong_ling_servo_instance)

/**
 * @brief ZL-IS2 协议允许的最大舵机 ID
 */
#define ZHONG_LING_SERVO_MAX_ID 254u

/**
 * @brief ZL-IS2 协议中的 PWM 和时间字段范围
 */
#define ZHONG_LING_SERVO_PWM_MIN 500u
#define ZHONG_LING_SERVO_PWM_CENTER 1500u
#define ZHONG_LING_SERVO_PWM_MAX 2500u
#define ZHONG_LING_SERVO_TIME_MS_MAX 9999u

/**
 * @brief 批量下发时使用的单舵机 PWM-时间指令
 */
typedef struct {
    uint8_t id;
    uint16_t pwm;
    uint16_t time_ms;
} ZhongLingServoPwmCmd;

typedef struct {
    uint8_t id;
    float pos_rad;
    float spd_rad_s;
} ZhongLingServoPosSpdCmd;

/**
 * @brief 众灵舵机初始化配置
 *
 * 这里的范围与零位均由具体实例配置决定
 * 对于 270° 舵机，通常可配置为：
 * `pos_min_rad = -3pi/4`，`pos_center_rad = 0`，`pos_max_rad = 3pi/4`
 * 同时 `pwm_min = 500`，`pwm_center = 1500`，`pwm_max = 2500`
 */
typedef struct {
    const BusServoPortOps* ops;
    uint32_t timeout_ms;
    uint8_t retry_count;
    float pos_min_rad;
    float pos_center_rad;
    float pos_max_rad;
    uint16_t pwm_min;
    uint16_t pwm_center;
    uint16_t pwm_max;
    bool invert;
    bool allow_torque_ignore;
} ZhongLingServoConfig;

/**
 * @brief ZL-IS2 协议专属扩展接口
 */
typedef struct {
    /**
     * @brief 复位舵机
     */
    BusServoStatus(*reset)(void);
    /**
     * @brief 设置用户自定义波特率
     * @param baudrate 波特率值, 例如 115200
     */
    BusServoStatus(*set_user_baudrate)(uint32_t baudrate);
    /**
     * @brief 停止所有舵机
     */
    BusServoStatus(*stop_all)(void);
    /**
     * @brief 停止指定舵机
     * @param id 舵机 ID
     */
    BusServoStatus(*stop_one)(uint8_t id);
    /**
     * @brief 设置偏差值
     * @param id 舵机 ID
     * @param deviation 偏差值, 范围 -500 ~ 500, 单位 0.1us
     */
    BusServoStatus(*set_deviation)(uint8_t id, int16_t deviation);
    /**
     * @brief 设置单舵机 PWM 和时间
     * @param id 舵机 ID
     * @param pwm PWM 值, 范围 500 ~ 2500, 单位 0.1us
     * @param time_ms 时间值, 范围 0 ~ 9999, 单位 ms
     */
    BusServoStatus(*set_pwm_time)(uint8_t id, uint16_t pwm, uint16_t time_ms);
    /**
     * @brief 批量设置多个舵机的 PWM 和时间
     * @param cmds 指令数组
     * @param count 指令数量, 最大为 255
     */
    BusServoStatus(*set_multi_pwm_time)(const ZhongLingServoPwmCmd* cmds, size_t count);
    /**
     * @brief 批量设置多个舵机的位置和速度
     * @param cmds 指令数组
     * @param count 指令数量, 最大为 255
     */
    BusServoStatus(*set_multi_pos_spd)(const ZhongLingServoPosSpdCmd* cmds, size_t count);
} ZhongLingServoInterface;

extern const BusServoInterface zhong_ling_servo_common_instance;
extern const ZhongLingServoInterface* zhong_ling_servo_instance;

#endif

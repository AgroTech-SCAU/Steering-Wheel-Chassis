#ifndef _bus_servo_h_
#define _bus_servo_h_

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 类 型 / Typedef 声 明 ========================= ! //

/**
 * @brief 舵机入口单例，上层可统一调用 `bus_servo.xxx` 或 `bus_servo_xxx`
 */
#define bus_servo (*bus_servo_instance)

/**
 * @brief 舵机通用状态码表
 */
#define SERVO_STATUS_TABLE \
    X(OK, 0)               \
    X(ERROR, 1)            \
    X(INVALID_PARAM, 2)    \
    X(PORT_ERROR, 3)       \
    X(TIMEOUT, 4)          \
    X(ID_MISMATCH, 5)      \
    X(CHECKSUM_ERROR, 6)   \
    X(NO_INSTANCE, 7)      \
    X(NOT_INITIALIZE, 8)   \
    X(BUFFER_TOO_SMALL, 9) \
    X(UNSUPPORTED, 10)     \
    X(NO_FEEDBACK, 11)

#define X(name, value) SERVO_STATUS_##name = value,
/**
 * @brief 舵机通用状态码
 */
typedef enum {
    SERVO_STATUS_TABLE
} BusServoStatus;
#undef X

/**
 * @brief 具体舵机总线 16 位寄存器的字节序
 *
 * 仅供需要该概念的具体驱动在自身配置结构里复用
 */
typedef enum {
    SERVO_ENDIAN_LITTLE = 0,
    SERVO_ENDIAN_BIG = 1,
} BusServoEndian;

/**
 * @brief 最近一次解析出的通用舵机反馈
 */
typedef struct {
    uint8_t id;
    uint8_t error_code;
    float position;
    float speed;
    float torque;
} BusServoFeedback;

/**
 * @brief 舵机底层端口函数表
 *
 * 该类型保留为通用辅助类型；是否使用、如何放进配置结构，由具体实例决定
 */
typedef struct {
    bool (*write)(const uint8_t* data, uint16_t len);
    int (*read)(uint8_t* data, uint16_t len);
    uint32_t (*now_ms)(void);
    void (*delay_ms)(uint32_t ms);
    void (*flush_rx)(void);
} BusServoPortOps;

/**
 * @brief 舵机通用接口表
 *
 * 这里只放所有舵机都应具备的通用能力
 * 具体型号、协议和初始化配置均由实例自行定义
 */
typedef struct {
    BusServoStatus (*init)(const void* config);
    const char* (*status_str)(BusServoStatus status);
    BusServoStatus (*set_speed)(uint8_t id, float speed);
    BusServoStatus (*set_pos_spd)(uint8_t id, float position, float velocity);
    BusServoStatus (*set_pos_spd_tor)(uint8_t id, float position, float velocity, float torque);
    float (*get_position)(uint8_t id);
    float (*get_speed)(uint8_t id);
    float (*get_torque)(uint8_t id);
    BusServoStatus (*update_feedback)(uint8_t id, BusServoFeedback* feedback);
} BusServoInterface;

/**
 * @brief 当前绑定的具体舵机实例
 */
extern const BusServoInterface* bus_servo_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

BusServoStatus bus_servo_set_instance(const BusServoInterface* instance);
BusServoStatus bus_servo_init(const void* config);
const char* bus_servo_status_str(BusServoStatus status);
BusServoStatus bus_servo_set_speed(uint8_t id, float speed);
BusServoStatus bus_servo_set_pos_spd(uint8_t id, float position, float velocity);
BusServoStatus bus_servo_set_pos_spd_tor(uint8_t id, float position, float velocity, float torque);
float bus_servo_get_position(uint8_t id);
float bus_servo_get_speed(uint8_t id);
float bus_servo_get_torque(uint8_t id);
BusServoStatus bus_servo_update_feedback(uint8_t id, BusServoFeedback* feedback);

#endif

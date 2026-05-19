#ifndef _bus_motor_h_
#define _bus_motor_h_

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 电机入口单例, 上层可统一调用 bus_motor.xxx 或 bus_motor_xxx
 */
#define bus_motor (*bus_motor_instance)

/**
 * @brief 电机通用状态码表
 * @param OK 操作成功
 * @param ERROR 通用错误
 * @param INVALID_PARAM 参数无效
 * @param PORT_ERROR 端口函数表未提供或底层端口失败
 * @param TIMEOUT 等待反馈超时
 * @param ID_MISMATCH 反馈 ID 与期望 ID 不一致
 * @param NO_INSTANCE 未绑定具体电机实例
 * @param NOT_INITIALIZE 具体电机实例尚未初始化
 * @param UNSUPPORTED 当前电机不支持该通用能力
 * @param NO_FEEDBACK 指定 ID 尚无有效反馈缓存
 */
#define MOTOR_STATUS_TABLE \
    X(OK, 0) \
    X(ERROR, 1) \
    X(INVALID_PARAM, 2) \
    X(PORT_ERROR, 3) \
    X(TIMEOUT, 4) \
    X(ID_MISMATCH, 5) \
    X(NO_INSTANCE, 6) \
    X(NOT_INITIALIZE, 7) \
    X(UNSUPPORTED, 8) \
    X(NO_FEEDBACK, 9)

#define X(name, value) MOTOR_STATUS_##name = value,
/**
 * @brief 电机通用状态码
 */
typedef enum {
    MOTOR_STATUS_TABLE
} BusMotorStatus;
#undef X

/**
 * @brief 电机通用模式值类型
 *
 * bus_motor 仅透传模式值, 不定义具体含义
 * 每种具体电机应在各自头文件中定义自己的模式枚举
 */
typedef uint32_t BusMotorMode;

/**
 * @brief 最近一次解析出的通用电机反馈
 * @param id 电机协议 ID
 * @param error_code 最近一次反馈帧中的原始错误码
 * @param position 当前位置, 单位 rad
 * @param speed 当前速度, 单位 rad/s
 * @param torque 当前扭矩或负载估计, 单位 N*m
 */
typedef struct {
    uint16_t id;
    uint8_t error_code;
    float position;
    float speed;
    float torque;
} BusMotorFeedback;

/**
 * @brief 电机底层端口函数表, 由 service 或 platform 注入
 */
typedef struct {
    /**
     * @brief 向电机总线发送一帧数据
     * @param id 帧 ID
     * @param data 待发送缓冲区
     * @param len 待发送长度, 单位 byte
     * @return 为真表示发送成功, 为假表示发送失败
     */
    bool (*send)(uint32_t id, const uint8_t* data, uint8_t len);
    /**
     * @brief 从电机总线读取一帧数据
     * @param id 帧 ID 输出指针
     * @param data 接收缓冲区
     * @param len 输入时为缓冲区长度, 输出时为实际长度
     * @return 为真表示读取成功, 为假表示暂无数据或读取失败
     */
    bool (*read)(uint32_t* id, uint8_t* data, uint8_t* len);
    /**
     * @brief 获取当前单调递增时间
     * @return 当前时间, 单位 ms
     */
    uint32_t(*now_ms)(void);
    /**
     * @brief 可选的阻塞延时函数
     * @param ms 延时时间, 单位 ms
     */
    void (*delay_ms)(uint32_t ms);
    /**
     * @brief 可选的接收缓冲区清空函数
     */
    void (*flush_rx)(void);
} BusMotorPortOps;

/**
 * @brief 电机通用初始化配置
 * @param ops 底层端口函数表
 * @param timeout_ms 超时时间, 单位 ms, 传 0 由具体驱动决定默认值
 * @param retry_count 预留重试次数, 由具体驱动按需使用
 */
typedef struct {
    const BusMotorPortOps* ops;
    uint32_t timeout_ms;
    uint8_t retry_count;
} BusMotorConfig;

/**
 * @brief 电机通用接口表
 *
 * 这里只放所有电机都应具备的通用能力
 * 具体模式的枚举值和协议含义由具体电机实例自己定义
 */
typedef struct {
    /**
     * @brief 初始化具体电机驱动
     * @param config 初始化配置
     * @return 状态码
     */
    BusMotorStatus(*init)(const BusMotorConfig* config);
    /**
     * @brief 将状态码转换为常量字符串
     * @param status 状态码
     * @return 状态码名称字符串
     */
    const char* (*status_str)(BusMotorStatus status);
    /**
     * @brief 将模式值转换为常量字符串
     * @param mode 具体电机定义的模式值
     * @return 模式名称字符串
     */
    const char* (*mode_str)(BusMotorMode mode);
    /**
     * @brief 使能电机
     * @param id 电机 ID
     * @return 状态码
     */
    BusMotorStatus(*enable)(uint16_t id);
    /**
     * @brief 失能电机
     * @param id 电机 ID
     * @return 状态码
     */
    BusMotorStatus(*disable)(uint16_t id);
    /**
     * @brief 切换电机模式
     * @param id 电机 ID
     * @param mode 具体电机定义的模式值
     * @return 状态码
     */
    BusMotorStatus(*switch_mode)(uint16_t id, BusMotorMode mode);
    /**
     * @brief 设定目标位置
     * @param id 电机 ID
     * @param position 目标位置, 单位 rad
     * @return 状态码
     */
    BusMotorStatus(*set_pos)(uint16_t id, float position);
    /**
     * @brief 设定目标速度
     * @param id 电机 ID
     * @param speed 目标速度, 单位 rad/s
     * @return 状态码
     */
    BusMotorStatus(*set_spd)(uint16_t id, float speed);
    /**
     * @brief 设定目标位置和速度
     * @param id 电机 ID
     * @param position 目标位置, 单位 rad
     * @param speed 目标速度, 单位 rad/s
     * @return 状态码
     */
    BusMotorStatus(*set_pos_vel)(uint16_t id, float position, float speed);
    /**
     * @brief 设定目标扭矩或等效前馈
     * @param id 电机 ID
     * @param torque 目标扭矩, 单位 N*m
     * @return 状态码
     */
    BusMotorStatus(*set_tor)(uint16_t id, float torque);
    /**
     * @brief 设定位置环 PD 参数
     * @param id 电机 ID
     * @param kp 比例系数
     * @param kd 微分系数
     * @return 状态码
     */
    BusMotorStatus(*set_pd)(uint16_t id, float kp, float kd);
    /**
     * @brief 主动请求并更新反馈缓存
     * @param id 电机 ID
     * @param feedback 可选的反馈输出指针, 可为 NULL
     * @return 状态码
     */
    BusMotorStatus(*update_feedback)(uint16_t id, BusMotorFeedback* feedback);
    /**
     * @brief 从本地反馈缓存获取最近位置
     * @param id 电机 ID
     * @return 最近位置, 单位 rad
     */
    float (*get_pos)(uint16_t id);
    /**
     * @brief 从本地反馈缓存获取最近速度
     * @param id 电机 ID
     * @return 最近速度, 单位 rad/s
     */
    float (*get_spd)(uint16_t id);
    /**
     * @brief 从本地反馈缓存获取最近扭矩
     * @param id 电机 ID
     * @return 最近扭矩或负载估计, 单位 N*m
     */
    float (*get_tor)(uint16_t id);
    /**
     * @brief 停止电机
     * @param id 电机 ID
     * @return 状态码
     */
    BusMotorStatus(*stop)(uint16_t id);
    /**
     * @brief 制动电机
     * @param id 电机 ID
     * @return 状态码
     */
    BusMotorStatus(*brake)(uint16_t id);
} BusMotorInterface;

/**
 * @brief 当前绑定的具体电机实例
 */
extern const BusMotorInterface* bus_motor_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

BusMotorStatus bus_motor_set_instance(const BusMotorInterface* instance);
BusMotorStatus bus_motor_init(const BusMotorConfig* config);
const char* bus_motor_status_str(BusMotorStatus status);
const char* bus_motor_mode_str(BusMotorMode mode);
BusMotorStatus bus_motor_enable(uint16_t id);
BusMotorStatus bus_motor_disable(uint16_t id);
BusMotorStatus bus_motor_switch_mode(uint16_t id, BusMotorMode mode);
BusMotorStatus bus_motor_set_pos(uint16_t id, float position);
BusMotorStatus bus_motor_set_spd(uint16_t id, float speed);
BusMotorStatus bus_motor_set_pos_vel(uint16_t id, float position, float speed);
BusMotorStatus bus_motor_set_tor(uint16_t id, float torque);
BusMotorStatus bus_motor_set_pd(uint16_t id, float kp, float kd);
BusMotorStatus bus_motor_update_feedback(uint16_t id, BusMotorFeedback* feedback);
float bus_motor_get_pos(uint16_t id);
float bus_motor_get_spd(uint16_t id);
float bus_motor_get_tor(uint16_t id);
BusMotorStatus bus_motor_stop(uint16_t id);
BusMotorStatus bus_motor_brake(uint16_t id);

#endif

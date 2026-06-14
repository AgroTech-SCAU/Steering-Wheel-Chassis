#ifndef _ft_scs_servo_h_
#define _ft_scs_servo_h_

#include "bus_servo.h"

// ! ========================= 接 口 类 型 ========================= ! //

/**
 * @brief FEETECH SCS 特色入口单例
 */
#define ft_scs_servo (*ft_scs_servo_instance)

/**
 * @brief SCS 广播 ID
 */
#define FT_SCS_SERVO_BROADCAST_ID 0xFEu

/**
 * @brief SCS 指令码
 */
#define FT_SCS_SERVO_INST_PING 0x01u
#define FT_SCS_SERVO_INST_READ 0x02u
#define FT_SCS_SERVO_INST_WRITE 0x03u
#define FT_SCS_SERVO_INST_REG_WRITE 0x04u
#define FT_SCS_SERVO_INST_ACTION 0x05u
#define FT_SCS_SERVO_INST_RESTORE 0x06u
#define FT_SCS_SERVO_INST_REBOOT 0x08u
#define FT_SCS_SERVO_INST_BACKUP 0x09u
#define FT_SCS_SERVO_INST_RESET 0x0Au
#define FT_SCS_SERVO_INST_CALIBRATION 0x0Bu
#define FT_SCS_SERVO_INST_SYNC_READ 0x82u
#define FT_SCS_SERVO_INST_SYNC_WRITE 0x83u

/**
 * @brief 本驱动使用的 STS/SCS 常用寄存器地址
 */
#define FT_SCS_SERVO_ID 5u
#define FT_SCS_SERVO_MODE 33u
#define FT_SCS_SERVO_TORQUE_ENABLE 40u
#define FT_SCS_SERVO_ACC 41u
#define FT_SCS_SERVO_GOAL_POSITION_L 42u
#define FT_SCS_SERVO_GOAL_POSITION_H 43u
#define FT_SCS_SERVO_GOAL_TIME_L 44u
#define FT_SCS_SERVO_GOAL_TIME_H 45u
#define FT_SCS_SERVO_GOAL_SPEED_L 46u
#define FT_SCS_SERVO_GOAL_SPEED_H 47u
#define FT_SCS_SERVO_TORQUE_LIMIT_L 48u
#define FT_SCS_SERVO_TORQUE_LIMIT_H 49u
#define FT_SCS_SERVO_LOCK 55u
#define FT_SCS_SERVO_PRESENT_POSITION_L 56u
#define FT_SCS_SERVO_PRESENT_POSITION_H 57u
#define FT_SCS_SERVO_PRESENT_SPEED_L 58u
#define FT_SCS_SERVO_PRESENT_SPEED_H 59u
#define FT_SCS_SERVO_PRESENT_LOAD_L 60u
#define FT_SCS_SERVO_PRESENT_LOAD_H 61u
#define FT_SCS_SERVO_PRESENT_VOLTAGE 62u
#define FT_SCS_SERVO_PRESENT_TEMPERATURE 63u
#define FT_SCS_SERVO_MOVING 66u
#define FT_SCS_SERVO_PRESENT_CURRENT_L 69u
#define FT_SCS_SERVO_PRESENT_CURRENT_H 70u

/**
 * @brief FEETECH SCS 初始化配置
 */
typedef struct {
    const BusServoPortOps* ops;
    uint32_t timeout_ms;
    uint8_t retry_count;
    BusServoEndian endian;
} FtScsServoConfig;

/**
 * @brief FEETECH SCS 协议特色接口表
 *
 * 这些能力属于 SCS 协议专属能力, 因此不放入通用 BusServoInterface
 */
typedef struct {
    /**
     * @brief 发送 PING 指令并等待状态帧
     * @param id 舵机 ID
     * @param out_id 可选的应答 ID 输出指针, 可为 NULL
     * @return 状态码
     */
    BusServoStatus (*ping)(uint8_t id, uint8_t* out_id);
    /**
     * @brief 打开舵机扭矩输出
     * @param id 舵机 ID
     * @return 状态码
     */
    BusServoStatus (*enable_torque)(uint8_t id);
    /**
     * @brief 关闭舵机扭矩输出
     * @param id 舵机 ID
     * @return 状态码
     */
    BusServoStatus (*disable_torque)(uint8_t id);
    /**
     * @brief 触发已经预写的 REG_WRITE 命令
     * @param id 舵机 ID 或 FT_SCS_SERVO_BROADCAST_ID
     * @return 状态码
     */
    BusServoStatus (*action)(uint8_t id);
    /**
     * @brief 向控制表写入 1 字节
     * @param id 舵机 ID
     * @param addr 寄存器地址
     * @param value 写入值
     * @return 状态码
     */
    BusServoStatus (*write_u8)(uint8_t id, uint8_t addr, uint8_t value);
    /**
     * @brief 向控制表写入 2 字节
     * @param id 舵机 ID
     * @param addr 寄存器地址
     * @param value 写入值
     * @return 状态码
     */
    BusServoStatus (*write_u16)(uint8_t id, uint8_t addr, uint16_t value);
    /**
     * @brief 从控制表读取 1 字节
     * @param id 舵机 ID
     * @param addr 寄存器地址
     * @param value 读取值输出指针
     * @return 状态码
     */
    BusServoStatus (*read_u8)(uint8_t id, uint8_t addr, uint8_t* value);
    /**
     * @brief 从控制表读取 2 字节
     * @param id 舵机 ID
     * @param addr 寄存器地址
     * @param value 读取值输出指针
     * @return 状态码
     */
    BusServoStatus (*read_u16)(uint8_t id, uint8_t addr, uint16_t* value);
    /**
     * @brief 发送原始 SCS 指令帧
     * @param id 舵机 ID
     * @param instruction SCS 指令码
     * @param params 参数缓冲区, params_len 为 0 时可为 NULL
     * @param params_len 参数长度, 单位 byte
     * @param need_ack 为真表示非广播 ID 时等待状态帧
     * @return 状态码
     */
    BusServoStatus (*write_packet)(uint8_t id, uint8_t instruction, const uint8_t* params, uint8_t params_len, bool need_ack);
} FtScsBusServoInterface;

/**
 * @brief FEETECH SCS 驱动实现的通用舵机实例
 */
extern const BusServoInterface ft_scs_servo_common_instance;

/**
 * @brief FEETECH SCS 特色实例
 */
extern const FtScsBusServoInterface* ft_scs_servo_instance;

// ! ========================= 接 口 函 数 ========================= ! //

/**
 * @brief 将 SCS 原始带符号值转换为普通有符号值
 * @param value 原始寄存器值
 * @param sign_bit 符号位索引
 * @return 有符号整数
 */
int16_t ft_scs_servo_raw_to_signed(uint16_t value, uint8_t sign_bit);

/**
 * @brief 将普通有符号值转换为 SCS bit15 符号编码
 * @param value 有符号整数
 * @return 原始寄存器值
 */
uint16_t ft_scs_servo_signed_to_raw(int16_t value);

/**
 * @brief 同步读取多个舵机的反馈
 * @param ids 舵机 ID 数组
 * @param count ID 数量
 * @param feedbacks 反馈输出数组
 * @param feedback_cap 反馈输出数组容量
 * @return 状态码
 */
BusServoStatus ft_scs_sync_read_feedback(const uint8_t* ids,
                                         uint8_t count,
                                         BusServoFeedback* feedbacks,
                                         uint8_t feedback_cap);

/**
 * @brief 同步写入多个舵机的位置和速度
 * @param ids 舵机 ID 数组
 * @param positions 目标位置数组, 单位度
 * @param count ID 数量
 * @param velocity 目标速度, 单位度/秒
 * @return 状态码
 */
BusServoStatus ft_scs_sync_write_pos_spd(const uint8_t* ids,
                                         const float* positions,
                                         uint8_t count,
                                         float velocity);

#endif

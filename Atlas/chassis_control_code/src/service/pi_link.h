#ifndef _service_pi_link_h_
#define _service_pi_link_h_

/**
 * @file pi_link.h
 * @brief 树莓派串口链路服务接口
 */

#include "serial_arm/five_dof_arm_kine.h"

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 类 型 声 明 ========================= ! //

/**
 * @brief 树莓派下发的底盘速度命令
 */
typedef struct {
    float vx;
    float vy;
    float wz;
    uint32_t stamp_ms;
} PiChassisCommand;

/**
 * @brief 树莓派下发的 yaw 控制模式
 * @details 语义约定如下
 * - `YAW:HOLD_ENABLE` `YAW:HOLD_DISABLE` `YAW:TARGET,value` 使用本地 yaw hold
 * - `YAW:RATE,value` 表示树莓派直接给定 yaw_rate
 * - `YAW:RATE` 新鲜时, 优先覆盖 `CHASSIS:vx,vy,wz` 中的 `wz`
 * - `YAW:RATE` 过期后, 不再继续覆盖 `wz`
 */
typedef enum {
    PI_YAW_MODE_NONE = 0,
    PI_YAW_MODE_HOLD_ENABLE,
    PI_YAW_MODE_HOLD_DISABLE,
    PI_YAW_MODE_TARGET_SET,
    PI_YAW_MODE_RATE_SET
} PiYawMode;

/**
 * @brief 树莓派下发的 yaw 命令
 */
typedef struct {
    PiYawMode mode;
    float target_yaw;
    float yaw_rate;
    uint32_t stamp_ms;
} PiYawCommand;

/**
 * @brief 树莓派下发的机械臂命令类型
 */
typedef enum {
    PI_ARM_COMMAND_NONE = 0,
    PI_ARM_COMMAND_JOINT_TARGET,
    PI_ARM_COMMAND_SEQUENCE_ID,
    PI_ARM_COMMAND_STOP,
    PI_ARM_COMMAND_ENABLE
} PiArmCommandType;

/**
 * @brief 树莓派下发的机械臂命令
 */
typedef struct {
    PiArmCommandType type;
    FiveDofArmJointArray joints;
    uint16_t sequence_id;
    float speed_rad_s;
    uint32_t stamp_ms;
} PiArmCommand;

/**
 * @brief 树莓派上报的自动任务结果类型
 * @details Pi 是自动任务主控, `MISSION:DONE` 和 `MISSION:FAIL,code` 用于上报自动任务结果
 */
typedef enum {
    PI_MISSION_EVENT_NONE = 0,
    PI_MISSION_EVENT_DONE,
    PI_MISSION_EVENT_FAIL
} PiMissionEventType;

/**
 * @brief 树莓派上报的自动任务结果
 */
typedef struct {
    PiMissionEventType type;
    int32_t fail_code;
    uint32_t stamp_ms;
} PiMissionEvent;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化 Pi 串口链路服务
 */
void pi_link_init(void);

/**
 * @brief 轮询 Pi 串口接收缓冲并解析报文
 */
void pi_link_process(void);

/**
 * @brief 判断 Pi 链路是否在线
 * @return bool `true` 表示 Pi 链路在线
 */
bool pi_link_is_online(void);

/**
 * @brief 获取最近一帧底盘命令
 * @param cmd 输出命令缓冲区
 * @return bool `true` 表示存在有效命令
 */
bool pi_link_get_chassis_cmd(PiChassisCommand* cmd);

/**
 * @brief 获取最近一帧 yaw 命令
 * @param cmd 输出命令缓冲区
 * @return bool `true` 表示存在有效命令
 */
bool pi_link_get_yaw_cmd(PiYawCommand* cmd);

/**
 * @brief 获取最近一帧机械臂命令
 * @param cmd 输出命令缓冲区
 * @return bool `true` 表示存在有效命令
 */
bool pi_link_get_arm_cmd(PiArmCommand* cmd);

/**
 * @brief 判断底盘命令是否仍然新鲜
 * @param timeout_ms 超时阈值, 单位 ms
 * @return bool `true` 表示命令仍有效
 */
bool pi_link_chassis_cmd_is_fresh(uint32_t timeout_ms);

/**
 * @brief 判断 yaw 命令是否仍然新鲜
 * @param timeout_ms 超时阈值, 单位 ms
 * @return bool `true` 表示命令仍有效
 */
bool pi_link_yaw_cmd_is_fresh(uint32_t timeout_ms);

/**
 * @brief 判断机械臂命令是否仍然新鲜
 * @param timeout_ms 超时阈值, 单位 ms
 * @return bool `true` 表示命令仍有效
 */
bool pi_link_arm_cmd_is_fresh(uint32_t timeout_ms);

/**
 * @brief 查询树莓派是否请求急停
 * @return bool `true` 表示存在待消费的 Pi 急停请求
 */
bool pi_link_get_estop_requested(void);

/**
 * @brief 清除待消费的 Pi 急停请求
 */
void pi_link_clear_estop_request(void);

/**
 * @brief 获取最近一次自动任务结果事件
 * @param event 输出事件缓冲区
 * @return bool `true` 表示存在待消费的任务结果事件
 */
bool pi_link_get_mission_event(PiMissionEvent* event);

/**
 * @brief 清除待消费的自动任务结果事件
 */
void pi_link_clear_mission_event(void);

/**
 * @brief 向 Pi 周期发送 IMU 与里程计摘要
 */
void pi_link_send_imu_odom(void);

/**
 * @brief 发送 MCU 状态帧
 * @details ASCII 状态帧格式如下
 * `MCU_STATUS:stamp_ms,STATE=...,MANUAL=...,CHASSIS=...,ARM=...,ODOM=...,REMOTE=...,PC=...,PI=...,FAULT=...,FAULT_SRC=...,FAULT_LEVEL=...,FAULT_CODE=...`
 */
void pi_link_send_mcu_status(void);

/**
 * @brief 清空当前缓存的 Pi 控制命令
 */
void pi_link_clear_commands(void);

#endif

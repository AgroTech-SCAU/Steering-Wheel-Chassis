#ifndef _service_pc_link_h_
#define _service_pc_link_h_

/**
 * @file pc_link.h
 * @brief PC 串口链路服务接口
 */

#include "serial_arm/five_dof_arm_kine.h"

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 类 型 声 明 ========================= ! //

/**
 * @brief PC 侧控制命令类型
 */
typedef enum {
    PC_COMMAND_NONE = 0,
    PC_COMMAND_START,
    PC_COMMAND_STOP,
    PC_COMMAND_CLEAR_FAULT,
    PC_COMMAND_BRAKE,
    PC_COMMAND_ARM_ENABLE,
    PC_COMMAND_ARM_STOP,
    PC_COMMAND_ESTOP
} PcCommandId;

/**
 * @brief PC 命令缓存
 */
typedef struct {
    PcCommandId id;
    uint32_t stamp_ms;
} PcCommand;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化 PC 串口链路服务
 * @details 保留 USART1 log 发送能力, PC 接收侧当前仍使用单字节中断加软件 ring
 */
void pc_link_init(void);

/**
 * @brief 轮询 PC 链路缓存并解析新消息
 * @details 建议在 100Hz 调度点调用
 */
void pc_link_process(void);

/**
 * @brief 判断 PC 链路是否在线
 * @return bool `true` 表示最近收到过有效报文
 */
bool pc_link_is_online(void);

/**
 * @brief 获取最近一条 PC 控制命令
 * @param command 输出命令缓冲区
 * @return bool `true` 表示存在有效命令
 */
bool pc_link_get_command(PcCommand* command);

/**
 * @brief 获取最近一组 PC 主臂关节角目标
 * @param joints 输出关节数组
 * @return bool `true` 表示存在缓存目标
 */
bool pc_link_get_master_joints(FiveDofArmJointArray* joints);

/**
 * @brief 判断 PC 主臂关节角数据是否新鲜
 * @param timeout_ms 超时阈值, 单位 ms
 * @return bool `true` 表示关节角数据仍然有效
 */
bool pc_link_master_joints_is_fresh(uint32_t timeout_ms);

/**
 * @brief 清除最近一条待消费的 PC 命令
 */
void pc_link_clear_command(void);

/**
 * @brief 清除缓存的 PC 主臂关节目标
 */
void pc_link_clear_master_joints(void);

#endif

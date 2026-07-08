#ifndef _service_pc_comms_h_
#define _service_pc_comms_h_

/**
 * @file pc_comms.h
 * @brief PC 通信服务接口
 */

#include "serial_arm/five_dof_arm_kine.h"

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 类 型 声 明 ========================= ! //

/**
 * @brief PC 通信服务状态码
 */
typedef struct {
    uint32_t (*now_ms)(void);
} PcCommsPortOps;

typedef enum {
    PC_COMMS_STATUS_OK = 0,
    PC_COMMS_STATUS_INVALID_PARAM,
    PC_COMMS_STATUS_DEPENDENCY_MISSING
} PcCommsStatus;

typedef struct {
    PcCommsPortOps port_ops;
} PcCommsConfig;

typedef struct {
    uint32_t rx_frame_count;
    uint32_t rx_bad_crc_count;
    uint32_t rx_bad_len_count;
    uint32_t rx_unknown_msg_count;
    uint8_t rx_last_msg_id;
    uint8_t rx_last_seq;
    uint32_t last_rx_ms;
    uint32_t tx_frame_count;
} PcCommsStats;

typedef struct {
    FiveDofArmJointArray joints;
    bool end_set;
    uint32_t stamp_ms;
} PcCommsMasterJoints;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化 PC 通信服务
 * @param config 通信配置
 * @return PcCommsStatus 初始化结果
 */
PcCommsStatus pc_comms_init(const PcCommsConfig* config);

/**
 * @brief 向 PC 通信服务喂入一个接收字节
 * @param data 本次收到的字节
 */
void pc_comms_on_rx_byte(uint8_t data);

/**
 * @brief 轮询 PC 通信缓存并解析新消息
 * @details 建议在 100Hz 调度点调用
 */
void pc_comms_process(void);

/**
 * @brief 判断 PC 通信链路是否在线
 * @return bool `true` 表示最近收到过有效报文
 */
bool pc_comms_is_online(void);

/**
 * @brief 获取最近一组 PC 主臂关节角目标
 * @param joints 输出关节数组
 * @return bool `true` 表示存在缓存目标
 */
bool pc_comms_get_master_joints(FiveDofArmJointArray* joints);
bool pc_comms_get_master_joints_snapshot(PcCommsMasterJoints* snapshot);
bool pc_comms_get_master_end_set(bool* end_set);

/**
 * @brief 判断 PC 主臂关节角数据是否新鲜
 * @param timeout_ms 超时阈值, 单位 ms
 * @return bool `true` 表示关节角数据仍然有效
 */
bool pc_comms_master_joints_is_fresh(uint32_t timeout_ms);

/**
 * @brief 清除缓存的 PC 主臂关节目标
 */
void pc_comms_clear_master_joints(void);

/**
 * @brief 读取 PC 通信统计信息
 * @param stats 输出统计信息
 * @return bool `true` 表示读取成功
 */
bool pc_comms_get_stats(PcCommsStats* stats);

#endif

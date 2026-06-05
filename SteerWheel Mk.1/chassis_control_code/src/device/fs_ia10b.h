#ifndef _fs_ia10b_h_
#define _fs_ia10b_h_

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @file fs_ia10b.h
 * @brief FlySky FS-iA10B i.BUS 接收机驱动接口
 */

/**
 * @brief i.BUS 单帧固定长度，单位 byte
 */
#define FS_IA10B_IBUS_FRAME_LEN 32u

/**
 * @brief FS-iA10B 接收机通道数量
 */
#define FS_IA10B_CHANNEL_COUNT 14u

/**
 * @brief i.BUS 解码后的接收机数据
 */
typedef struct {
    uint16_t channel[FS_IA10B_CHANNEL_COUNT]; /**< 通道原始值，通常约为 1000~2000 */
    uint32_t frame_count;                     /**< 已解析的有效帧数量 */
    uint32_t error_count;                     /**< 校验失败或格式错误计数 */
    uint32_t last_update_ms;                  /**< 最近一次有效帧时间戳，单位 ms */
    bool valid;                               /**< true 表示已经收到过有效帧 */
} FsIa10bData;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化 FS-iA10B i.BUS 接收驱动
 *
 * UART 参数由 CubeMX/.ioc 配置；本函数负责注册回调和初始化驱动状态
 */
void ibus_init(void);

/**
 * @brief 维护 i.BUS 接收状态
 *
 * 遥控离线或冷启动未收到有效帧时，该函数会按限频策略重启 DMA 接收
 */
void ibus_maintain(void);

/**
 * @brief 获取最近一次有效 i.BUS 数据
 * @param out 输出接收机数据
 * @return true 表示已经收到有效数据；false 表示参数无效或尚无有效帧
 */
bool ibus_get_data(FsIa10bData* out);

/**
 * @brief 判断 i.BUS 遥控链路是否在线
 * @param timeout_ms 在线超时时间，单位 ms
 * @return true 表示链路在线；false 表示链路离线
 */
bool ibus_is_online(uint32_t timeout_ms);

/**
 * @brief 读取指定遥控通道原始值
 * @param index 通道索引，从 0 开始
 * @return 通道原始值；索引越界时返回 0
 */
uint16_t ibus_get_channel(uint8_t index);

#endif

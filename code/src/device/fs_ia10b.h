#ifndef _fs_ia10b_h_
#define _fs_ia10b_h_

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief i.BUS 单帧固定长度
 *
 * FS-iA10B 输出的 i.BUS 帧包含帧头、通道数据和校验和；
 * 本驱动按该长度收集并校验完整帧
 */
#define FS_IA10B_IBUS_FRAME_LEN 32u

/**
 * @brief FS-iA10B 接收机通道数量
 *
 * 通道数组按 i.BUS 帧内顺序保存；
 * 上层通过索引读取对应摇杆、开关和旋钮通道
 */
#define FS_IA10B_CHANNEL_COUNT 14u

/**
 * @brief i.BUS 解码后的接收机数据
 *
 * 该结构保存最近一帧有效遥控数据；
 * 在线判断依赖 valid 和 last_update_ms 字段
 */
typedef struct {
    uint16_t channel[FS_IA10B_CHANNEL_COUNT];
    uint32_t frame_count;
    uint32_t error_count;
    uint32_t last_update_ms;
    bool valid;
} FsIa10bData;

/**
 * @brief i.BUS 接收调试信息
 *
 * 该结构用于观察最近接收字节和解析计数；
 * 冷启动排查时可判断 UART 是否真正收到数据
 */
typedef struct {
    uint8_t bytes[FS_IA10B_IBUS_FRAME_LEN];
    uint32_t rx_byte_count;
    uint32_t header0_count;
    uint32_t header01_count;
    uint8_t latest_byte;
} FsIa10bDebug;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化 FS-iA10B i.BUS 接收驱动
 *
 * UART5 的波特率、反相等硬件参数由 CubeMX/.ioc 配置；
 * 本函数只注册回调；空闲行 DMA 接收由 ibus_maintain() 在主循环中启动
 */
void ibus_init(void);

/**
 * @brief 维护 i.BUS 接收状态
 *
 * 遥控离线或冷启动未收到有效帧时；
 * 函数会按限频策略重启 DMA 接收，不会重新初始化 UART5
 */
void ibus_maintain(void);

/**
 * @brief 获取最近一次有效 i.BUS 数据
 *
 * 函数会在临界区内复制共享数据；
 * 调用者拿到的是稳定快照
 *
 * @param out 输出接收机数据的缓存指针
 * @return true 已经收到过有效数据
 * @return false 参数无效或尚未收到有效数据
 */
bool ibus_get_data(FsIa10bData* out);

/**
 * @brief 获取 i.BUS 调试信息快照
 *
 * 该接口用于冷启动和接收异常排查；
 * 不参与正常控制逻辑
 *
 * @param out 输出调试信息的缓存指针
 * @return true 调试信息复制成功
 * @return false 输出指针为空
 */
bool ibus_get_debug(FsIa10bDebug* out);

/**
 * @brief 判断 i.BUS 遥控链路是否在线
 *
 * 根据最近一次有效帧的更新时间判断；
 * 超过指定时间没有新帧则认为离线
 *
 * @param timeout_ms 在线超时时间，单位毫秒
 * @return true 遥控链路在线
 * @return false 遥控链路离线
 */
bool ibus_is_online(uint32_t timeout_ms);

/**
 * @brief 读取指定遥控通道原始值
 *
 * 如果索引越界则返回零；
 * 有效通道值通常位于 1000 到 2000 附近
 *
 * @param index 通道索引
 * @return uint16_t 通道原始值
 */
uint16_t ibus_get_channel(uint8_t index);

#endif

#include "fs_ia10b.h"

#include <string.h>

#include "stm32_hal_uart.h"
#include "log.h"

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief i.BUS 帧头第一个字节
 *
 * FlySky i.BUS 有效数据帧固定以该字节开头；
 * 解析器用它寻找帧边界
 */
#define IBUS_HEADER_0 0x20u

/**
 * @brief i.BUS 帧头第二个字节
 *
 * 只有连续匹配两个帧头字节后；
 * 后续字节才会被当作完整 i.BUS 帧内容接收
 */
#define IBUS_HEADER_1 0x40u

/**
 * @brief 遥控通道允许的最小原始值
 *
 * 低于该值的帧通常表示解析错位或数据异常；
 * 因此会被判定为无效帧
 */
#define IBUS_CHANNEL_MIN 800u

/**
 * @brief 遥控通道允许的最大原始值
 *
 * 高于该值的帧通常表示解析错位或数据异常；
 * 因此会被判定为无效帧
 */
#define IBUS_CHANNEL_MAX 2200u

/**
 * @brief 接收自动重启的最小间隔
 *
 * 接收机冷启动未稳定时可能暂时无有效帧；
 * 该间隔用于限制恢复尝试频率
 */
#define IBUS_RX_RESTART_INTERVAL_MS 200u

static uint8_t s_rx_byte = 0u;

/**
 * @brief i.BUS 单帧解析缓存
 *
 * 解析器逐字节填充该缓存；
 * 填满后再进行校验和通道范围检查
 */
static uint8_t s_frame[FS_IA10B_IBUS_FRAME_LEN];

/**
 * @brief 当前 i.BUS 帧缓存写入位置
 *
 * 该索引用于跟踪帧头同步状态；
 * 校验完成或发现错位后会回到零
 */
static uint8_t s_frame_index = 0u;

/**
 * @brief 上一次重启接收的系统时间戳
 *
 * 单位为 HAL_GetTick() 的毫秒计数；
 * 用于限制冷启动恢复动作的频率
 */
static uint32_t s_last_restart_ms = 0u;

/**
 * @brief UART5 单字节中断接收是否已经启动并在等待数据
 */
static volatile bool s_rx_active = false;

/**
 * @brief 当前在线周期是否已经记录过日志
 *
 * 该标志用于日志限频；
 * 避免接收机稳定在线后持续刷屏
 */
static bool s_online_logged = false;

/**
 * @brief 最近一次成功解析出的接收机数据
 *
 * 回调中更新该数据；
 * 业务层读取时会临时关中断以保证一致性
 */
static volatile FsIa10bData s_data;

/**
 * @brief 是否有待处理的接收错误需要重启接收
 *
 * 该标志在 UART 错误回调中置位；
 * 主循环看到后会执行重启流程
 */
static volatile bool s_ibus_restart_pending = false;

/**
 * @brief 下一次重启接收前是否需要先 abort HAL 当前接收状态
 */
static volatile bool s_abort_before_restart = false;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 处理 UART5 单字节接收完成事件
 */
static void ibus_rx_complete_callback(void);

/**
 * @brief 处理 UART5 错误回调
 *
 * 出错后会清空当前帧同步状态；
 * 然后标记主循环重新启动接收以等待下一帧
 */
static void ibus_error_callback(void);

/**
 * @brief 重启 UART5 单字节中断接收
 *
 * @return true 接收启动成功
 * @return false 接收启动失败
 */
static bool ibus_restart_receive(void);

/**
 * @brief 向 i.BUS 帧解析器输入一个字节
 *
 * 解析器会自动寻找帧头并累计完整帧；
 * 满 32 字节后执行校验和通道范围检查
 *
 * @param byte 新收到的一个字节
 */
static void ibus_feed_byte(uint8_t byte);

/**
 * @brief 校验 i.BUS 帧头和校验和
 *
 * 该函数只判断帧结构和校验和；
 * 通道数值范围由后续函数单独检查
 *
 * @param frame 待检查的 i.BUS 帧
 * @return true 帧头和校验和有效
 * @return false 帧头或校验和无效
 */
static bool ibus_check_frame(const uint8_t frame[FS_IA10B_IBUS_FRAME_LEN]);

/**
 * @brief 检查 i.BUS 通道值是否处于合理范围
 *
 * 该检查用于过滤错位帧；
 * 所有通道均落在限定范围内才会更新接收机数据
 *
 * @param frame 待检查的 i.BUS 帧
 * @return true 所有通道值均合理
 * @return false 至少一个通道值异常
 */
static bool ibus_channels_in_range(const uint8_t frame[FS_IA10B_IBUS_FRAME_LEN]);

/**
 * @brief 将有效 i.BUS 帧解析为共享接收机状态
 *
 * 函数会更新通道值、帧计数和最近更新时间；
 * 上层据此判断遥控链路是否在线
 *
 * @param frame 已通过校验的 i.BUS 帧
 */
static void ibus_parse_frame(const uint8_t frame[FS_IA10B_IBUS_FRAME_LEN]);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void ibus_init(void) {
    memset((void*)&s_data, 0, sizeof(s_data));
    memset(s_frame, 0, sizeof(s_frame));
    s_rx_byte = 0u;

    s_frame_index = 0u;
    s_last_restart_ms = 0u;
    s_rx_active = false;
    s_online_logged = false;
    s_ibus_restart_pending = true;
    s_abort_before_restart = false;

    uart_register_rx_complete_callback(&huart5, ibus_rx_complete_callback);
    uart_register_error_callback(&huart5, ibus_error_callback);
}

void ibus_maintain(void) {
    uint32_t now = HAL_GetTick();

    if(s_data.valid && (now - s_data.last_update_ms) <= IBUS_RX_RESTART_INTERVAL_MS) {
        if(s_online_logged == false) {
            log_info("IBUS online frame_count=%lu", (unsigned long)s_data.frame_count);
            s_online_logged = true;
        }
    }
    else if(s_online_logged) {
        s_online_logged = false;
    }

    if(s_rx_active && !s_ibus_restart_pending) {
        return;
    }

    if(!s_ibus_restart_pending && (now - s_last_restart_ms) < IBUS_RX_RESTART_INTERVAL_MS) {
        return;
    }

    s_ibus_restart_pending = false;
    s_frame_index = 0u;
    if(s_abort_before_restart) {
        s_abort_before_restart = false;
        (void)uart_abort_receive_it(&huart5);
    }

    if(!ibus_restart_receive()) {
        s_ibus_restart_pending = true;
    }
}

bool ibus_get_data(FsIa10bData* out) {
    uint32_t primask;

    if(out == NULL) {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *out = s_data;
    if(primask == 0u) {
        __enable_irq();
    }

    return out->valid;
}

bool ibus_is_online(uint32_t timeout_ms) {
    uint32_t elapsed;

    if(!s_data.valid) {
        return false;
    }

    elapsed = HAL_GetTick() - s_data.last_update_ms;
    return elapsed <= timeout_ms;
}

uint16_t ibus_get_channel(uint8_t index) {
    uint16_t value = 0u;
    uint32_t primask;

    if(index >= FS_IA10B_CHANNEL_COUNT) {
        return 0u;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    value = s_data.channel[index];
    if(primask == 0u) {
        __enable_irq();
    }

    return value;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static void ibus_rx_complete_callback(void) {
    s_rx_active = false;
    ibus_feed_byte(s_rx_byte);

    if(!ibus_restart_receive()) {
        s_ibus_restart_pending = true;
    }
}

static void ibus_error_callback(void) {
    s_rx_active = false;
    s_frame_index = 0u;
    s_data.error_count++;
    s_abort_before_restart = true;
    s_ibus_restart_pending = true;
}

static bool ibus_restart_receive(void) {
    s_rx_byte = 0u;
    s_last_restart_ms = HAL_GetTick();
    s_rx_active = uart_receive_it(&huart5, &s_rx_byte, 1u);
    return s_rx_active;
}

static void ibus_feed_byte(uint8_t byte) {
    if(s_frame_index == 0u) {
        if(byte == IBUS_HEADER_0) {
            s_frame[0] = byte;
            s_frame_index = 1u;
        }
        return;
    }

    if(s_frame_index == 1u) {
        if(byte == IBUS_HEADER_1) {
            s_frame[1] = byte;
            s_frame_index = 2u;
        }
        else if(byte == IBUS_HEADER_0) {
            s_frame[0] = byte;
            s_frame_index = 1u;
        }
        else {
            s_frame_index = 0u;
        }
        return;
    }

    s_frame[s_frame_index] = byte;
    s_frame_index++;

    if(s_frame_index >= FS_IA10B_IBUS_FRAME_LEN) {
        if(ibus_check_frame(s_frame) && ibus_channels_in_range(s_frame)) {
            ibus_parse_frame(s_frame);
        }
        else {
            s_data.error_count++;
        }

        s_frame_index = 0u;
    }
}

static bool ibus_check_frame(const uint8_t frame[FS_IA10B_IBUS_FRAME_LEN]) {
    uint16_t checksum = 0xFFFFu;
    uint16_t rx_checksum;
    uint8_t i;

    if(frame[0] != IBUS_HEADER_0 || frame[1] != IBUS_HEADER_1) {
        return false;
    }

    for(i = 0u; i < 30u; ++i) {
        checksum = (uint16_t)(checksum - frame[i]);
    }

    rx_checksum = (uint16_t)frame[30] | ((uint16_t)frame[31] << 8);
    return checksum == rx_checksum;
}

static bool ibus_channels_in_range(const uint8_t frame[FS_IA10B_IBUS_FRAME_LEN]) {
    uint8_t i;

    for(i = 0u; i < FS_IA10B_CHANNEL_COUNT; ++i) {
        uint8_t offset = (uint8_t)(2u + i * 2u);
        uint16_t value = (uint16_t)frame[offset] | ((uint16_t)frame[offset + 1u] << 8);

        if(value < IBUS_CHANNEL_MIN || value > IBUS_CHANNEL_MAX) {
            return false;
        }
    }

    return true;
}

static void ibus_parse_frame(const uint8_t frame[FS_IA10B_IBUS_FRAME_LEN]) {
    FsIa10bData data;
    uint8_t i;

    memset(&data, 0, sizeof(data));

    for(i = 0u; i < FS_IA10B_CHANNEL_COUNT; ++i) {
        uint8_t offset = (uint8_t)(2u + i * 2u);
        data.channel[i] = (uint16_t)frame[offset] | ((uint16_t)frame[offset + 1u] << 8);
    }

    data.valid = true;
    data.last_update_ms = HAL_GetTick();
    data.frame_count = s_data.frame_count + 1u;
    data.error_count = s_data.error_count;

    s_data = data;
}

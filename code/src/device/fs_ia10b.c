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
 * @brief UART5 DMA 接收缓存长度
 *
 * i.BUS 单帧长度为 32 字节；
 * 这里使用 64 字节以容纳一次空闲中断中的多帧片段
 */
#define IBUS_DMA_RX_BUF_LEN 64u

/**
 * @brief 接收 DMA 自动重启的最小间隔
 *
 * 接收机冷启动未稳定时可能暂时无有效帧；
 * 该间隔用于限制恢复尝试频率
 */
#define IBUS_RX_RESTART_INTERVAL_MS 200u

/**
 * @brief UART5 完整重新初始化的最小间隔
 *
 * 如果单纯重启 DMA 仍无法恢复接收；
 * 会按该间隔重新配置 UART5 反相接收
 */
#define IBUS_UART_REINIT_INTERVAL_MS 1000u

/**
 * @brief UART5 i.BUS 字节流的 DMA 接收缓存
 *
 * 缓存按 32 字节对齐；
 * 这样在解析前可以安全地执行 DCache 失效操作
 */
__attribute__((aligned(32))) static uint8_t s_dma_rx_buf[IBUS_DMA_RX_BUF_LEN];

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
 * @brief 上一次重启接收 DMA 的系统时间戳
 *
 * 单位为 HAL_GetTick() 的毫秒计数；
 * 用于限制冷启动恢复动作的频率
 */
static uint32_t s_last_restart_ms = 0u;

/**
 * @brief 上一次完整重初始化 UART5 的系统时间戳
 *
 * 单位为 HAL_GetTick() 的毫秒计数；
 * 用于避免频繁 DeInit/Init UART 外设
 */
static uint32_t s_last_uart_reinit_ms = 0u;

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
 * @brief i.BUS 解析调试信息
 *
 * 该结构保存最近字节和计数器；
 * 用于排查冷启动接收机是否真正有数据进来
 */
static volatile FsIa10bDebug s_debug;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 处理 UART 空闲行接收事件
 *
 * 平台层在 UART5 ReceiveToIdle DMA 回调中调用本函数；
 * 函数会先维护缓存一致性，再逐字节喂给 i.BUS 解析器
 *
 * @param size HAL 本次报告的接收字节数
 */
static void ibus_rx_event_callback(uint16_t size);

/**
 * @brief 处理 UART5 错误回调
 *
 * 出错后会清空当前帧同步状态；
 * 然后重新启动 DMA 接收以等待下一帧
 */
static void ibus_error_callback(void);

/**
 * @brief 重启 UART5 空闲行 DMA 接收
 *
 * 每次启动前会清空 DMA 缓存并维护 DCache；
 * 成功后等待下一次接收事件回调
 *
 * @return true DMA 接收启动成功
 * @return false DMA 接收启动失败
 */
static bool ibus_restart_receive(void);

/**
 * @brief 让 DMA 接收缓存对 CPU 可见
 *
 * STM32H7 开启缓存后，DMA 写入内存不会自动刷新 CPU 缓存；
 * 解析前必须对接收区域执行 DCache 失效
 *
 * @param size 需要失效的接收字节数
 */
static void ibus_invalidate_dma_buffer(uint16_t size);

/**
 * @brief 使用 i.BUS 所需的反相接收配置重初始化 UART5
 *
 * 冷启动时接收机可能比主控慢稳定；
 * 重新初始化可以从 UART 错误或半帧状态中恢复
 *
 * @return true UART5 配置成功并已启动接收
 * @return false UART5 配置或接收启动失败
 */
static bool ibus_uart_init_inverted(void);

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
    log_info("IBUS init begin");
    memset((void*)&s_data, 0, sizeof(s_data));
    memset((void*)&s_debug, 0, sizeof(s_debug));
    memset(s_frame, 0, sizeof(s_frame));
    memset(s_dma_rx_buf, 0, sizeof(s_dma_rx_buf));

    s_frame_index = 0u;
    s_last_restart_ms = 0u;
    s_last_uart_reinit_ms = 0u;

    uart_register_rx_event_callback(&huart5, ibus_rx_event_callback);
    uart_register_error_callback(&huart5, ibus_error_callback);
    if(ibus_uart_init_inverted()) {
        log_info("IBUS init done");
    }
    else {
        log_error("IBUS init failed");
    }
}

void ibus_maintain(void) {
    uint32_t now = HAL_GetTick();

    if(s_data.valid && (now - s_data.last_update_ms) <= IBUS_RX_RESTART_INTERVAL_MS) {
        if(s_online_logged == false) {
            log_info("IBUS online frame_count=%lu", (unsigned long)s_data.frame_count);
            s_online_logged = true;
        }
        return;
    }

    if(s_online_logged) {
        s_online_logged = false;
    }

    if((now - s_last_restart_ms) < IBUS_RX_RESTART_INTERVAL_MS) {
        return;
    }

    s_frame_index = 0u;

    if((now - s_last_uart_reinit_ms) >= IBUS_UART_REINIT_INTERVAL_MS) {
        s_last_uart_reinit_ms = now;
        (void)ibus_uart_init_inverted();
    }
    else {
        (void)uart_abort_receive_dma(&huart5);
        (void)ibus_restart_receive();
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

bool ibus_get_debug(FsIa10bDebug* out) {
    uint32_t primask;

    if(out == NULL) {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *out = s_debug;
    if(primask == 0u) {
        __enable_irq();
    }

    return true;
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

static void ibus_rx_event_callback(uint16_t size) {
    uint16_t i;

    if(size > IBUS_DMA_RX_BUF_LEN) {
        size = IBUS_DMA_RX_BUF_LEN;
    }

    ibus_invalidate_dma_buffer(size);

    for(i = 0u; i < size; ++i) {
        ibus_feed_byte(s_dma_rx_buf[i]);
    }

    ibus_restart_receive();
}

static void ibus_error_callback(void) {
    s_frame_index = 0u;
    s_data.error_count++;

    (void)uart_abort_receive_dma(&huart5);
    ibus_restart_receive();
}

static bool ibus_restart_receive(void) {
    memset(s_dma_rx_buf, 0, sizeof(s_dma_rx_buf));
    ibus_invalidate_dma_buffer(IBUS_DMA_RX_BUF_LEN);
    s_last_restart_ms = HAL_GetTick();
    return uart_receive_to_idle_dma(&huart5, s_dma_rx_buf, IBUS_DMA_RX_BUF_LEN);
}

static void ibus_invalidate_dma_buffer(uint16_t size) {
    uintptr_t start = ((uintptr_t)s_dma_rx_buf) & ~(uintptr_t)31U;
    uintptr_t end = ((uintptr_t)s_dma_rx_buf + size + 31U) & ~(uintptr_t)31U;

    if(size == 0u) {
        return;
    }

    SCB_InvalidateDCache_by_Addr((uint32_t*)start, (int32_t)(end - start));
}

static bool ibus_uart_init_inverted(void) {
    (void)uart_abort_receive_dma(&huart5);
    (void)HAL_UART_DeInit(&huart5);

    /*
     * 当前硬件接线需要开启 UART RX 反相；
     * 这样 FS-iA10B 的 i.BUS 帧才能按 115200 8N1 正常解析
     */
    huart5.Init.BaudRate = 115200;
    huart5.Init.WordLength = UART_WORDLENGTH_8B;
    huart5.Init.StopBits = UART_STOPBITS_1;
    huart5.Init.Parity = UART_PARITY_NONE;
    huart5.Init.Mode = UART_MODE_TX_RX;
    huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart5.Init.OverSampling = UART_OVERSAMPLING_16;
    huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT;
    huart5.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;

    if(HAL_UART_Init(&huart5) != HAL_OK) {
        return false;
    }

    if(HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        return false;
    }

    if(HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        return false;
    }

    if(HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK) {
        return false;
    }

    memset(s_frame, 0, sizeof(s_frame));
    memset(s_dma_rx_buf, 0, sizeof(s_dma_rx_buf));
    memset((void*)&s_debug, 0, sizeof(s_debug));
    s_frame_index = 0u;
    ibus_restart_receive();
    return true;
}

static void ibus_feed_byte(uint8_t byte) {
    uint8_t i;

    s_debug.rx_byte_count++;
    s_debug.latest_byte = byte;
    for(i = 0u; i < (FS_IA10B_IBUS_FRAME_LEN - 1u); ++i) {
        s_debug.bytes[i] = s_debug.bytes[i + 1u];
    }
    s_debug.bytes[FS_IA10B_IBUS_FRAME_LEN - 1u] = byte;

    if(s_frame_index == 0u) {
        if(byte == IBUS_HEADER_0) {
            s_debug.header0_count++;
            s_frame[0] = byte;
            s_frame_index = 1u;
        }
        return;
    }

    if(s_frame_index == 1u) {
        if(byte == IBUS_HEADER_1) {
            s_debug.header01_count++;
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

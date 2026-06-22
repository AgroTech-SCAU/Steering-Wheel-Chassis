#pragma once

#include <cstdint>
#include <optional>
#include <vector>

namespace mcu_comm_bridge {

// ! ========================= 协 议 常 量 ========================= ! //

constexpr uint8_t SOF0 = 0xA5u;                             /**< 帧头字节 0 */
constexpr uint8_t SOF1 = 0x5Au;                             /**< 帧头字节 1 */
constexpr uint8_t PROTOCOL_VERSION = 0x01u;                 /**< 协议版本 */
constexpr uint8_t FLAG_NEED_ACK = 0x01u;                    /**< 标志：需要 ACK */

constexpr uint8_t MSG_MCU_STATUS = 0x21u;                   /**< MCU -> PI: 状态 */
constexpr uint8_t MSG_MCU_START_SENSOR_EVENT = 0x22u;       /**< MCU -> PI: 传感器启动事件 */
constexpr uint8_t MSG_MCU_ACK = 0x23u;                      /**< MCU -> PI: ACK */
constexpr uint8_t MSG_MCU_FAULT_EVENT = 0x24u;              /**< MCU -> PI: 故障事件 */
constexpr uint8_t MSG_MCU_IMU = 0x25u;                      /**< MCU -> PI: IMU 数据 */
constexpr uint8_t MSG_MCU_ODOM = 0x26u;                     /**< MCU -> PI: 里程计 */

constexpr uint8_t MSG_PI_HEARTBEAT = 0x30u;                 /**< PI -> MCU: 心跳 */
constexpr uint8_t MSG_PI_CONTROL = 0x31u;                   /**< PI -> MCU: 自动模式连续控制 */
constexpr uint8_t MSG_PI_YAW_ACTION = 0x41u;                /**< PI -> MCU: 一次性 yaw 动作 */
constexpr uint8_t MSG_PI_ESTOP = 0x43u;                     /**< PI -> MCU: 急停事件 */
constexpr uint8_t MSG_PI_ACK = 0x44u;                       /**< PI -> MCU: ACK */

constexpr uint16_t BODY_PREFIX_LEN = 4u;                    /**< body 前缀长度 */
constexpr uint16_t FRAME_OVERHEAD_LEN = 6u;                 /**< 帧开销长度（SOF/CRC 等） */
constexpr uint16_t PAYLOAD_MCU_STATUS_LEN = 16u;            /**< STATUS 负载长度 */
constexpr uint16_t PAYLOAD_MCU_START_SENSOR_EVENT_LEN = 8u; /**< START_SENSOR_EVENT 负载长度 */
constexpr uint16_t PAYLOAD_MCU_ACK_LEN = 4u;                /**< MCU ACK 负载长度 */
constexpr uint16_t PAYLOAD_MCU_FAULT_EVENT_LEN = 8u;        /**< FAULT_EVENT 负载长度 */
constexpr uint16_t PAYLOAD_MCU_IMU_LEN = 48u;               /**< IMU 负载长度 */
constexpr uint16_t PAYLOAD_MCU_ODOM_LEN = 32u;              /**< ODOM 负载长度 */
constexpr uint16_t PAYLOAD_PI_CONTROL_LEN = 38u;            /**< PI_CONTROL 负载长度 */
constexpr uint16_t PAYLOAD_PI_YAW_ACTION_LEN = 12u;         /**< PI_YAW_ACTION 负载长度 */
constexpr uint16_t PAYLOAD_PI_ESTOP_LEN = 8u;               /**< PI_ESTOP 负载长度 */
constexpr uint16_t PAYLOAD_PI_ACK_LEN = 4u;                 /**< PI ACK 负载长度 */

// ! ========================= 类 型 声 明 ========================= ! //

/**
 * @brief 已解析帧的数据结构
 */
struct Frame {
    uint8_t version = 0;           /**< 协议版本 */
    uint8_t msg_id = 0;            /**< 消息 ID */
    uint8_t seq = 0;               /**< 序列号 */
    uint8_t flags = 0;             /**< 标志位 */
    std::vector<uint8_t> payload;  /**< 负载数据 */
};

/**
 * @brief 解析器统计信息
 */
struct ParserStats {
    uint64_t rx_bytes = 0;     /**< 接收字节数 */
    uint64_t rx_frames = 0;    /**< 成功解析的帧数 */
    uint64_t crc_error = 0;    /**< CRC 校验失败计数 */
    uint64_t len_error = 0;    /**< 长度错误计数 */
    uint64_t version_error = 0;/**< 协议版本不匹配计数 */
    uint64_t resync = 0;       /**< 重同步次数 */
};

/**
 * @brief 通用二进制帧流式解析器
 *
 * 帧级 LEN/CRC 使用大端，payload 内部多字节字段使用小端
 */
class BinaryFrameParser {
public:
    /**
     * @brief 构造解析器
     * @param max_body_len 最大允许的 body 长度（字节）
     */
    explicit BinaryFrameParser(uint16_t max_body_len = 256u);

    /**
     * @brief 向解析器喂入单字节并尝试解析帧
     * @param byte 输入字节
     * @return 当解析出完整帧时返回 `Frame`，否则返回 `std::nullopt`
     */
    std::optional<Frame> feed(uint8_t byte);

    /**
     * @brief 重置解析器状态，准备解析新一帧
     */
    void reset();

    /**
     * @brief 获取解析器统计信息（只读）
     */
    const ParserStats& stats() const { return stats_; }

private:
    enum class State {
        WaitSof0,
        WaitSof1,
        LenHigh,
        LenLow,
        Body,
        CrcHigh,
        CrcLow,
    };

    std::optional<Frame> finish_frame();

    uint16_t max_body_len_ = 256u;
    State state_ = State::WaitSof0;
    uint16_t body_len_ = 0u;
    uint16_t body_read_ = 0u;
    uint16_t rx_crc_ = 0u;
    std::vector<uint8_t> buffer_;
    ParserStats stats_;
};

// ! ========================= 接 口 函 数 ========================= ! //

uint16_t crc16_ccitt(const uint8_t* data, size_t len);
std::vector<uint8_t> pack_frame(uint8_t msg_id, uint8_t seq, uint8_t flags, const std::vector<uint8_t>& payload);

uint16_t read_u16_le(const std::vector<uint8_t>& data, size_t offset);
int16_t read_i16_le(const std::vector<uint8_t>& data, size_t offset);
uint32_t read_u32_le(const std::vector<uint8_t>& data, size_t offset);
int32_t read_i32_le(const std::vector<uint8_t>& data, size_t offset);

void write_u16_le(std::vector<uint8_t>& data, size_t offset, uint16_t value);
void write_i16_le(std::vector<uint8_t>& data, size_t offset, int16_t value);
void write_u32_le(std::vector<uint8_t>& data, size_t offset, uint32_t value);
void write_i32_le(std::vector<uint8_t>& data, size_t offset, int32_t value);

double mm_to_m(int32_t mm);
double mm_s_to_m_s(int32_t mm_s);
double mm_s2_to_m_s2(int32_t mm_s2);
double urad_to_rad(int32_t urad);
double urad_s_to_rad_s(int32_t urad_s);

}  // namespace mcu_comm_bridge

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace mcu_comm_bridge {

constexpr uint8_t SOF0 = 0xA5u;
constexpr uint8_t SOF1 = 0x5Au;
constexpr uint8_t PROTOCOL_VERSION = 0x01u;
constexpr uint8_t FLAG_NEED_ACK = 0x01u;

constexpr uint8_t MSG_MCU_STATUS = 0x21u;
constexpr uint8_t MSG_MCU_START_SENSOR_EVENT = 0x22u;
constexpr uint8_t MSG_MCU_ACK = 0x23u;
constexpr uint8_t MSG_MCU_FAULT_EVENT = 0x24u;
constexpr uint8_t MSG_MCU_IMU = 0x25u;
constexpr uint8_t MSG_MCU_ODOM = 0x26u;
constexpr uint8_t MSG_MCU_ARM_STATE = 0x27u;

constexpr uint8_t MSG_PI_HEARTBEAT = 0x30u;
constexpr uint8_t MSG_PI_CONTROL = 0x31u;
constexpr uint8_t MSG_PI_ARM_ACTION = 0x40u;
constexpr uint8_t MSG_PI_YAW_ACTION = 0x41u;
constexpr uint8_t MSG_PI_MISSION_EVENT = 0x42u;
constexpr uint8_t MSG_PI_ESTOP = 0x43u;
constexpr uint8_t MSG_PI_ACK = 0x44u;

constexpr uint16_t BODY_PREFIX_LEN = 4u;
constexpr uint16_t FRAME_OVERHEAD_LEN = 6u;
constexpr uint16_t PAYLOAD_MCU_STATUS_LEN = 16u;
constexpr uint16_t PAYLOAD_MCU_START_SENSOR_EVENT_LEN = 8u;
constexpr uint16_t PAYLOAD_MCU_ACK_LEN = 4u;
constexpr uint16_t PAYLOAD_MCU_FAULT_EVENT_LEN = 8u;
constexpr uint16_t PAYLOAD_MCU_IMU_LEN = 48u;
constexpr uint16_t PAYLOAD_MCU_ODOM_LEN = 32u;
constexpr uint16_t PAYLOAD_MCU_ARM_STATE_LEN = 48u;
constexpr uint16_t PAYLOAD_PI_CONTROL_LEN = 38u;
constexpr uint16_t PAYLOAD_PI_YAW_ACTION_LEN = 12u;
constexpr uint16_t PAYLOAD_PI_MISSION_EVENT_LEN = 8u;
constexpr uint16_t PAYLOAD_PI_ESTOP_LEN = 8u;
constexpr uint16_t PAYLOAD_PI_ACK_LEN = 4u;

constexpr uint8_t PI_MISSION_EVENT_DONE = 0x01u;
constexpr uint8_t PI_MISSION_EVENT_FAIL = 0x02u;

constexpr uint16_t ARM_STATE_FLAG_ARM_READY = 0x0001u;
constexpr uint16_t ARM_STATE_FLAG_JOINT_VALID = 0x0002u;
constexpr uint16_t ARM_STATE_FLAG_FK_VALID = 0x0004u;
constexpr uint16_t ARM_STATE_FLAG_POSE_VALID = ARM_STATE_FLAG_FK_VALID;

struct Frame {
    uint8_t version = 0;
    uint8_t msg_id = 0;
    uint8_t seq = 0;
    uint8_t flags = 0;
    std::vector<uint8_t> payload;
};

enum class ParserErrorKind {
    CrcError,
    LengthError,
    VersionError,
    KnownMessageBadLength,
    RawBufferOverflow,
};

struct ParserErrorEvent {
    ParserErrorKind kind = ParserErrorKind::CrcError;
    std::optional<uint8_t> msg_id;
};

struct ParserStats {
    uint64_t rx_bytes = 0;
    uint64_t rx_frames = 0;
    uint64_t crc_error = 0;
    uint64_t len_error = 0;
    uint64_t version_error = 0;
    uint64_t known_msg_bad_length = 0;
    uint64_t raw_buffer_overflow = 0;
    uint64_t resync = 0;
};

class BinaryFrameParser {
public:
    explicit BinaryFrameParser(uint16_t max_body_len = 256u, size_t raw_buffer_capacity = 4096u);

    std::vector<Frame> feed(const uint8_t* data, size_t size);
    std::optional<Frame> feed(uint8_t byte);
    std::vector<ParserErrorEvent> take_error_events();
    void reset();

    void set_max_body_len(uint16_t max_body_len);
    void set_raw_buffer_capacity(size_t raw_buffer_capacity);

    ParserStats stats() const { return stats_; }

private:
    std::vector<Frame> drain_frames();
    void record_error(ParserErrorKind kind, std::optional<uint8_t> msg_id = std::nullopt);
    void trim_raw_buffer_if_needed();
    void discard_current_candidate();
    bool candidate_header_is_plausible(size_t offset) const;
    static std::optional<uint16_t> expected_payload_length(uint8_t msg_id);

    uint16_t max_body_len_;
    size_t raw_buffer_capacity_;
    std::vector<uint8_t> raw_buffer_;
    std::vector<ParserErrorEvent> error_events_;
    ParserStats stats_{};
};

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

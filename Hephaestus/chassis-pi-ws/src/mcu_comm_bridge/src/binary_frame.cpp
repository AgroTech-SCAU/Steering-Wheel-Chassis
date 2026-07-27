#include "mcu_comm_bridge/binary_frame.hpp"

#include <algorithm>
#include <array>
#include <iterator>
#include <stdexcept>

namespace mcu_comm_bridge {
namespace {

constexpr std::array<uint8_t, 2> kSof = { SOF0, SOF1 };

uint16_t crc16_update(uint16_t crc, uint8_t data) {
    crc ^= static_cast<uint16_t>(data) << 8;
    for(uint8_t i = 0; i < 8; ++i) {
        crc = (crc & 0x8000u) != 0u
            ? static_cast<uint16_t>((crc << 1) ^ 0x1021u)
            : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

void check_range(const std::vector<uint8_t>& data, size_t offset, size_t len) {
    if(offset + len > data.size()) {
        throw std::out_of_range("payload offset out of range");
    }
}

}  // namespace

BinaryFrameParser::BinaryFrameParser(uint16_t max_body_len, size_t raw_buffer_capacity)
    : max_body_len_(max_body_len),
      raw_buffer_capacity_(raw_buffer_capacity) {
    raw_buffer_.reserve(raw_buffer_capacity_);
}

std::vector<Frame> BinaryFrameParser::feed(const uint8_t* data, size_t size) {
    if(data == nullptr || size == 0u) {
        return {};
    }

    stats_.rx_bytes += static_cast<uint64_t>(size);
    raw_buffer_.insert(raw_buffer_.end(), data, data + size);
    trim_raw_buffer_if_needed();
    return drain_frames();
}

std::optional<Frame> BinaryFrameParser::feed(uint8_t byte) {
    const auto frames = feed(&byte, 1u);
    if(frames.empty()) {
        return std::nullopt;
    }
    return frames.front();
}

std::vector<ParserErrorEvent> BinaryFrameParser::take_error_events() {
    std::vector<ParserErrorEvent> out;
    out.swap(error_events_);
    return out;
}

void BinaryFrameParser::reset() {
    raw_buffer_.clear();
    error_events_.clear();
}

void BinaryFrameParser::set_max_body_len(uint16_t max_body_len) {
    max_body_len_ = max_body_len;
}

void BinaryFrameParser::set_raw_buffer_capacity(size_t raw_buffer_capacity) {
    raw_buffer_capacity_ = raw_buffer_capacity;
    trim_raw_buffer_if_needed();
}

std::vector<Frame> BinaryFrameParser::drain_frames() {
    std::vector<Frame> frames;

    while(raw_buffer_.size() >= FRAME_OVERHEAD_LEN) {
        const auto sof_it = std::search(raw_buffer_.begin(), raw_buffer_.end(), kSof.begin(), kSof.end());
        if(sof_it == raw_buffer_.end()) {
            if(!raw_buffer_.empty()) {
                stats_.resync++;
                raw_buffer_.clear();
            }
            break;
        }

        if(sof_it != raw_buffer_.begin()) {
            stats_.resync++;
            raw_buffer_.erase(raw_buffer_.begin(), sof_it);
        }

        if(raw_buffer_.size() < FRAME_OVERHEAD_LEN) {
            break;
        }

        const uint16_t body_len = static_cast<uint16_t>(raw_buffer_[2] << 8) | raw_buffer_[3];
        if(body_len < BODY_PREFIX_LEN || body_len > max_body_len_) {
            record_error(ParserErrorKind::LengthError);
            discard_current_candidate();
            continue;
        }

        const auto msg_id = std::optional<uint8_t>(raw_buffer_[5]);
        if(raw_buffer_[4] != PROTOCOL_VERSION) {
            record_error(ParserErrorKind::VersionError, msg_id);
            discard_current_candidate();
            continue;
        }

        const size_t payload_len = static_cast<size_t>(body_len - BODY_PREFIX_LEN);
        const auto expected_len = expected_payload_length(*msg_id);
        if(expected_len.has_value() && payload_len != expected_len.value()) {
            record_error(ParserErrorKind::KnownMessageBadLength, msg_id);
            discard_current_candidate();
            continue;
        }

        const size_t frame_len = static_cast<size_t>(FRAME_OVERHEAD_LEN + body_len);
        if(raw_buffer_.size() < frame_len) {
            break;
        }

        const uint16_t stored_crc = static_cast<uint16_t>(raw_buffer_[frame_len - 2u] << 8) |
                                    raw_buffer_[frame_len - 1u];
        const uint16_t calculated_crc = crc16_ccitt(raw_buffer_.data(), static_cast<size_t>(body_len + 4u));
        if(calculated_crc != stored_crc) {
            record_error(ParserErrorKind::CrcError, msg_id);
            discard_current_candidate();
            continue;
        }

        Frame frame;
        frame.version = raw_buffer_[4];
        frame.msg_id = *msg_id;
        frame.seq = raw_buffer_[6];
        frame.flags = raw_buffer_[7];
        frame.payload.assign(raw_buffer_.begin() + 8, raw_buffer_.begin() + 8 + static_cast<std::ptrdiff_t>(payload_len));
        frames.push_back(std::move(frame));
        stats_.rx_frames++;
        raw_buffer_.erase(raw_buffer_.begin(), raw_buffer_.begin() + static_cast<std::ptrdiff_t>(frame_len));
    }

    return frames;
}

void BinaryFrameParser::record_error(ParserErrorKind kind, std::optional<uint8_t> msg_id) {
    switch(kind) {
        case ParserErrorKind::CrcError:
            stats_.crc_error++;
            break;
        case ParserErrorKind::LengthError:
            stats_.len_error++;
            break;
        case ParserErrorKind::VersionError:
            stats_.version_error++;
            break;
        case ParserErrorKind::KnownMessageBadLength:
            stats_.known_msg_bad_length++;
            break;
        case ParserErrorKind::RawBufferOverflow:
            stats_.raw_buffer_overflow++;
            break;
    }

    error_events_.push_back(ParserErrorEvent{ kind, msg_id });
}

void BinaryFrameParser::discard_current_candidate() {
    const auto search_begin = raw_buffer_.begin() + 1;
    auto next_sof = std::search(search_begin, raw_buffer_.end(), kSof.begin(), kSof.end());
    while(next_sof != raw_buffer_.end()) {
        const auto offset = static_cast<size_t>(std::distance(raw_buffer_.begin(), next_sof));
        if(candidate_header_is_plausible(offset)) {
            raw_buffer_.erase(raw_buffer_.begin(), next_sof);
            stats_.resync++;
            return;
        }
        next_sof = std::search(next_sof + 1, raw_buffer_.end(), kSof.begin(), kSof.end());
    }

    if(!raw_buffer_.empty() && raw_buffer_.back() == SOF0) {
        raw_buffer_.erase(raw_buffer_.begin(), raw_buffer_.end() - 1);
    }
    else {
        raw_buffer_.clear();
    }
    stats_.resync++;
}

bool BinaryFrameParser::candidate_header_is_plausible(size_t offset) const {
    const size_t remaining = raw_buffer_.size() - offset;
    if(remaining < 4u) {
        return true;
    }

    const uint16_t body_len = static_cast<uint16_t>(raw_buffer_[offset + 2u] << 8) | raw_buffer_[offset + 3u];
    if(body_len < BODY_PREFIX_LEN || body_len > max_body_len_) {
        return false;
    }

    if(remaining < 6u) {
        return true;
    }

    if(raw_buffer_[offset + 4u] != PROTOCOL_VERSION) {
        return false;
    }

    const uint8_t msg_id = raw_buffer_[offset + 5u];
    const auto expected_len = expected_payload_length(msg_id);
    if(expected_len.has_value() && static_cast<size_t>(body_len - BODY_PREFIX_LEN) != expected_len.value()) {
        return false;
    }

    return true;
}

void BinaryFrameParser::trim_raw_buffer_if_needed() {
    if(raw_buffer_.size() <= raw_buffer_capacity_) {
        return;
    }

    const size_t overflow = raw_buffer_.size() - raw_buffer_capacity_;
    raw_buffer_.erase(raw_buffer_.begin(), raw_buffer_.begin() + static_cast<std::ptrdiff_t>(overflow));
    record_error(ParserErrorKind::RawBufferOverflow);
}

std::optional<uint16_t> BinaryFrameParser::expected_payload_length(uint8_t msg_id) {
    switch(msg_id) {
        case MSG_MCU_STATUS:
            return PAYLOAD_MCU_STATUS_LEN;
        case MSG_MCU_START_SENSOR_EVENT:
            return PAYLOAD_MCU_START_SENSOR_EVENT_LEN;
        case MSG_MCU_ACK:
            return PAYLOAD_MCU_ACK_LEN;
        case MSG_MCU_FAULT_EVENT:
            return PAYLOAD_MCU_FAULT_EVENT_LEN;
        case MSG_MCU_IMU:
            return PAYLOAD_MCU_IMU_LEN;
        case MSG_MCU_ODOM:
            return PAYLOAD_MCU_ODOM_LEN;
        case MSG_MCU_ARM_STATE:
            return PAYLOAD_MCU_ARM_STATE_LEN;
        case MSG_PI_CONTROL:
            return PAYLOAD_PI_CONTROL_LEN;
        case MSG_PI_ARM_ACTION:
            return 8u;
        case MSG_PI_YAW_ACTION:
            return PAYLOAD_PI_YAW_ACTION_LEN;
        case MSG_PI_MISSION_EVENT:
            return PAYLOAD_PI_MISSION_EVENT_LEN;
        case MSG_PI_ESTOP:
            return PAYLOAD_PI_ESTOP_LEN;
        case MSG_PI_ACK:
            return PAYLOAD_PI_ACK_LEN;
        case MSG_PI_HEARTBEAT:
            return 0u;
        default:
            return std::nullopt;
    }
}

uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFFu;
    for(size_t i = 0; i < len; ++i) {
        crc = crc16_update(crc, data[i]);
    }
    return crc;
}

std::vector<uint8_t> pack_frame(uint8_t msg_id, uint8_t seq, uint8_t flags, const std::vector<uint8_t>& payload) {
    const uint16_t body_len = static_cast<uint16_t>(BODY_PREFIX_LEN + payload.size());
    std::vector<uint8_t> out(static_cast<size_t>(FRAME_OVERHEAD_LEN + body_len), 0u);

    out[0] = SOF0;
    out[1] = SOF1;
    out[2] = static_cast<uint8_t>(body_len >> 8);
    out[3] = static_cast<uint8_t>(body_len & 0xFFu);
    out[4] = PROTOCOL_VERSION;
    out[5] = msg_id;
    out[6] = seq;
    out[7] = flags;
    std::copy(payload.begin(), payload.end(), out.begin() + 8);

    const uint16_t crc = crc16_ccitt(out.data(), static_cast<size_t>(body_len + 4u));
    out[8 + payload.size()] = static_cast<uint8_t>(crc >> 8);
    out[9 + payload.size()] = static_cast<uint8_t>(crc & 0xFFu);
    return out;
}

uint16_t read_u16_le(const std::vector<uint8_t>& data, size_t offset) {
    check_range(data, offset, 2u);
    return static_cast<uint16_t>(data[offset]) |
           static_cast<uint16_t>(static_cast<uint16_t>(data[offset + 1]) << 8);
}

int16_t read_i16_le(const std::vector<uint8_t>& data, size_t offset) {
    return static_cast<int16_t>(read_u16_le(data, offset));
}

uint32_t read_u32_le(const std::vector<uint8_t>& data, size_t offset) {
    check_range(data, offset, 4u);
    return static_cast<uint32_t>(data[offset]) |
           (static_cast<uint32_t>(data[offset + 1]) << 8) |
           (static_cast<uint32_t>(data[offset + 2]) << 16) |
           (static_cast<uint32_t>(data[offset + 3]) << 24);
}

int32_t read_i32_le(const std::vector<uint8_t>& data, size_t offset) {
    return static_cast<int32_t>(read_u32_le(data, offset));
}

void write_u16_le(std::vector<uint8_t>& data, size_t offset, uint16_t value) {
    check_range(data, offset, 2u);
    data[offset] = static_cast<uint8_t>(value & 0xFFu);
    data[offset + 1] = static_cast<uint8_t>(value >> 8);
}

void write_i16_le(std::vector<uint8_t>& data, size_t offset, int16_t value) {
    write_u16_le(data, offset, static_cast<uint16_t>(value));
}

void write_u32_le(std::vector<uint8_t>& data, size_t offset, uint32_t value) {
    check_range(data, offset, 4u);
    data[offset] = static_cast<uint8_t>(value & 0xFFu);
    data[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
    data[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xFFu);
    data[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xFFu);
}

void write_i32_le(std::vector<uint8_t>& data, size_t offset, int32_t value) {
    write_u32_le(data, offset, static_cast<uint32_t>(value));
}

double mm_to_m(int32_t mm) {
    return static_cast<double>(mm) * 1e-3;
}

double mm_s_to_m_s(int32_t mm_s) {
    return static_cast<double>(mm_s) * 1e-3;
}

double mm_s2_to_m_s2(int32_t mm_s2) {
    return static_cast<double>(mm_s2) * 1e-3;
}

double urad_to_rad(int32_t urad) {
    return static_cast<double>(urad) * 1e-6;
}

double urad_s_to_rad_s(int32_t urad_s) {
    return static_cast<double>(urad_s) * 1e-6;
}

}  // namespace mcu_comm_bridge

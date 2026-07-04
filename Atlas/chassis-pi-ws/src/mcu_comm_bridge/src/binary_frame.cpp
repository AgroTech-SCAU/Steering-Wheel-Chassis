#include "mcu_comm_bridge/binary_frame.hpp"

#include <cstddef>
#include <stdexcept>

namespace mcu_comm_bridge {
namespace {

/**
 * @brief 计算并更新 CRC-16-CCITT 的中间值
 *
 * @param crc 当前 CRC 值
 * @param data 要加入计算的单字节数据
 * @return uint16_t 更新后的 CRC 值
 */
uint16_t crc16_update(uint16_t crc, uint8_t data) {
    crc ^= static_cast<uint16_t>(data) << 8;
    for(uint8_t i = 0; i < 8; ++i) {
        crc = (crc & 0x8000u) ? static_cast<uint16_t>((crc << 1) ^ 0x1021u)
            : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

/**
 * @brief 检查向量访问范围是否合法
 *
 * @param data 待检查的字节向量
 * @param offset 读取/写入偏移
 * @param len 所需的字节长度
 * @throws std::out_of_range 越界时抛出
 */
void check_range(const std::vector<uint8_t>& data, size_t offset, size_t len) {
    if(offset + len > data.size()) {
        throw std::out_of_range("payload offset out of range");
    }
}

}  // namespace

/**
 * @brief 构造 BinaryFrameParser
 *
 * @param max_body_len 最大允许的 body 长度（字节）
 */
BinaryFrameParser::BinaryFrameParser(uint16_t max_body_len)
    : max_body_len_(max_body_len) {
    buffer_.reserve(static_cast<size_t>(max_body_len_) + FRAME_OVERHEAD_LEN);
}

/**
 * @brief 重置解析器状态以开始新一帧解析
 */
void BinaryFrameParser::reset() {
    state_ = State::WaitSof0;
    body_len_ = 0u;
    body_read_ = 0u;
    rx_crc_ = 0u;
    buffer_.clear();
}

void BinaryFrameParser::recover_partial_sof(uint8_t last) {
    reset();
    if(last == SOF0) {
        buffer_.push_back(SOF0);
        state_ = State::WaitSof1;
    }
}

void BinaryFrameParser::reset_and_recover(uint8_t prev, uint8_t last) {
    reset();

    if(prev == SOF0 && last == SOF1) {
        buffer_.push_back(SOF0);
        buffer_.push_back(SOF1);
        state_ = State::LenHigh;
        stats_.resync++;
        return;
    }

    if(last == SOF0) {
        buffer_.push_back(SOF0);
        state_ = State::WaitSof1;
        stats_.resync++;
    }
}

/**
 * @brief 向解析器喂入一个字节并尝试解析帧
 *
 * @param byte 输入字节
 * @return std::optional<Frame> 完整帧解析完成时返回，否则为空
 */
std::optional<Frame> BinaryFrameParser::feed(uint8_t byte) {
    stats_.rx_bytes++;

    switch(state_) {
        case State::WaitSof0:
            if(byte == SOF0) {
                buffer_.clear();
                buffer_.push_back(byte);
                state_ = State::WaitSof1;
            }
            return std::nullopt;

        case State::WaitSof1:
            if(byte == SOF1) {
                buffer_.push_back(byte);
                state_ = State::LenHigh;
            }
            else {
                stats_.resync++;
                recover_partial_sof(byte);
            }
            return std::nullopt;

        case State::LenHigh:
            body_len_ = static_cast<uint16_t>(byte) << 8;
            buffer_.push_back(byte);
            state_ = State::LenLow;
            return std::nullopt;

        case State::LenLow:
            body_len_ |= byte;
            buffer_.push_back(byte);
            if(body_len_ < BODY_PREFIX_LEN || body_len_ > max_body_len_) {
                stats_.len_error++;
                reset_and_recover(buffer_[2], buffer_[3]);
                return std::nullopt;
            }
            body_read_ = 0u;
            state_ = State::Body;
            return std::nullopt;

        case State::Body:
            buffer_.push_back(byte);
            body_read_++;
            if(body_read_ >= body_len_) {
                state_ = State::CrcHigh;
            }
            return std::nullopt;

        case State::CrcHigh:
            rx_crc_ = static_cast<uint16_t>(byte) << 8;
            state_ = State::CrcLow;
            return std::nullopt;

        case State::CrcLow:
            rx_crc_ |= byte;
            return finish_frame();
    }

    reset();
    return std::nullopt;
}

/**
 * @brief 在已读到 CRC 字节时完成帧解析并返回 Frame
 *
 * 校验 CRC、验证协议版本并提取 payload
 */
std::optional<Frame> BinaryFrameParser::finish_frame() {
    const uint16_t calc_crc = crc16_ccitt(buffer_.data(), buffer_.size());
    if(calc_crc != rx_crc_) {
        stats_.crc_error++;
        reset_and_recover(static_cast<uint8_t>(rx_crc_ >> 8), static_cast<uint8_t>(rx_crc_ & 0xFFu));
        return std::nullopt;
    }

    Frame frame;
    frame.version = buffer_[4];
    frame.msg_id = buffer_[5];
    frame.seq = buffer_[6];
    frame.flags = buffer_[7];

    if(frame.version != PROTOCOL_VERSION) {
        stats_.version_error++;
        reset();
        return std::nullopt;
    }

    const size_t payload_len = static_cast<size_t>(body_len_ - BODY_PREFIX_LEN);
    frame.payload.assign(buffer_.begin() + 8, buffer_.begin() + 8 + static_cast<std::ptrdiff_t>(payload_len));

    stats_.rx_frames++;
    reset();
    return frame;
}

uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFFu;
    for(size_t i = 0; i < len; ++i) {
        crc = crc16_update(crc, data[i]);
    }
    return crc;
}

/**
 * @brief 将消息按协议打包为字节流
 *
 * @param msg_id 消息 ID
 * @param seq 序列号
 * @param flags 标志位
 * @param payload 负载数据
 * @return std::vector<uint8_t> 打包后的完整帧字节数组
 */

std::vector<uint8_t> pack_frame(uint8_t msg_id, uint8_t seq, uint8_t flags, const std::vector<uint8_t>& payload) {
    const uint16_t body_len = static_cast<uint16_t>(BODY_PREFIX_LEN + payload.size());
    std::vector<uint8_t> out;
    out.resize(static_cast<size_t>(FRAME_OVERHEAD_LEN + body_len));

    out[0] = SOF0;
    out[1] = SOF1;
    out[2] = static_cast<uint8_t>(body_len >> 8);
    out[3] = static_cast<uint8_t>(body_len & 0xFFu);
    out[4] = PROTOCOL_VERSION;
    out[5] = msg_id;
    out[6] = seq;
    out[7] = flags;

    for(size_t i = 0; i < payload.size(); ++i) {
        out[8 + i] = payload[i];
    }

    const uint16_t crc = crc16_ccitt(out.data(), static_cast<size_t>(4u + body_len));
    out[8 + payload.size()] = static_cast<uint8_t>(crc >> 8);
    out[9 + payload.size()] = static_cast<uint8_t>(crc & 0xFFu);
    return out;
}

/**
 * @brief 从字节向量中以小端读取无符号 16 位整数
 *
 * @param data 源字节向量
 * @param offset 偏移位置
 * @return uint16_t 读取到的值
 * @throws std::out_of_range 如果超出范围
 */

uint16_t read_u16_le(const std::vector<uint8_t>& data, size_t offset) {
    check_range(data, offset, 2u);
    return static_cast<uint16_t>(data[offset]) |
        static_cast<uint16_t>(static_cast<uint16_t>(data[offset + 1]) << 8);
}

/**
 * @brief 从字节向量中以小端读取有符号 16 位整数
 */

int16_t read_i16_le(const std::vector<uint8_t>& data, size_t offset) {
    return static_cast<int16_t>(read_u16_le(data, offset));
}

/**
 * @brief 从字节向量中以小端读取无符号 32 位整数
 */

uint32_t read_u32_le(const std::vector<uint8_t>& data, size_t offset) {
    check_range(data, offset, 4u);
    return static_cast<uint32_t>(data[offset]) |
        (static_cast<uint32_t>(data[offset + 1]) << 8) |
        (static_cast<uint32_t>(data[offset + 2]) << 16) |
        (static_cast<uint32_t>(data[offset + 3]) << 24);
}

/**
 * @brief 从字节向量中以小端读取有符号 32 位整数
 */

int32_t read_i32_le(const std::vector<uint8_t>& data, size_t offset) {
    return static_cast<int32_t>(read_u32_le(data, offset));
}

/**
 * @brief 以小端格式在字节向量中写入无符号 16 位整数
 *
 * @throws std::out_of_range 如果写入范围越界
 */

void write_u16_le(std::vector<uint8_t>& data, size_t offset, uint16_t value) {
    check_range(data, offset, 2u);
    data[offset] = static_cast<uint8_t>(value & 0xFFu);
    data[offset + 1] = static_cast<uint8_t>(value >> 8);
}

/**
 * @brief 以小端格式在字节向量中写入有符号 16 位整数
 */

void write_i16_le(std::vector<uint8_t>& data, size_t offset, int16_t value) {
    write_u16_le(data, offset, static_cast<uint16_t>(value));
}

/**
 * @brief 以小端格式在字节向量中写入无符号 32 位整数
 */

void write_u32_le(std::vector<uint8_t>& data, size_t offset, uint32_t value) {
    check_range(data, offset, 4u);
    data[offset] = static_cast<uint8_t>(value & 0xFFu);
    data[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
    data[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xFFu);
    data[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xFFu);
}

/**
 * @brief 以小端格式在字节向量中写入有符号 32 位整数
 */

void write_i32_le(std::vector<uint8_t>& data, size_t offset, int32_t value) {
    write_u32_le(data, offset, static_cast<uint32_t>(value));
}

/**
 * @brief 单位换算：毫米 -> 米
 *
 * @param mm 毫米值
 * @return double 米值
 */
double mm_to_m(int32_t mm) {
    return static_cast<double>(mm) * 1e-3;
}

/**
 * @brief 单位换算：毫米/秒 -> 米/秒
 *
 * @param mm_s 毫米每秒
 * @return double 米每秒
 */
double mm_s_to_m_s(int32_t mm_s) {
    return static_cast<double>(mm_s) * 1e-3;
}

/**
 * @brief 单位换算：毫米/秒^2 -> 米/秒^2
 *
 * @param mm_s2 毫米每秒平方
 * @return double 米每秒平方
 */
double mm_s2_to_m_s2(int32_t mm_s2) {
    return static_cast<double>(mm_s2) * 1e-3;
}

/**
 * @brief 单位换算：微弧度 -> 弧度
 *
 * @param urad 微弧度
 * @return double 弧度
 */
double urad_to_rad(int32_t urad) {
    return static_cast<double>(urad) * 1e-6;
}

/**
 * @brief 单位换算：微弧度/秒 -> 弧度/秒
 *
 * @param urad_s 微弧度每秒
 * @return double 弧度每秒
 */
double urad_s_to_rad_s(int32_t urad_s) {
    return static_cast<double>(urad_s) * 1e-6;
}

}  // namespace mcu_comm_bridge

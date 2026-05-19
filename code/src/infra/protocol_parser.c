#include "protocol_parser.h"

#include <stddef.h>
#include <stdint.h>

// ! ========================= 变 量 声 明 ========================= ! //



// ! ========================= 私 有 函 数 声 明 ========================= ! //

static RingBufErrorCode _rb_write(RingBuf* const self, const uint8_t data);
static RingBufErrorCode _rb_read(RingBuf* const self, uint8_t* const data);
static RingBufErrorCode _rb_clear(RingBuf* const self);

static bool _rb_is_full(RingBuf* const self);
static bool _rb_is_empty(RingBuf* const self);

static int _rb_get_size(RingBuf* const self);
static int _rb_get_capacity(RingBuf* const self);

static FrameParserErrorCode _fp_write_(FrameParser* const self, const uint8_t data);
static FrameParserErrorCode _fp_process_(FrameParser* const self);
static FrameParserErrorCode _fp_get_frame_(FrameParser* const self, uint8_t** const frame_buffer, uint16_t* const frame_length);
static FrameParserErrorCode _fp_finish_(FrameParser* const self);
static FrameParserErrorCode _fp_reset_(FrameParser* const self);

static uint16_t crc16_update(uint16_t crc, uint8_t data);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

const RingBufInterface ring_buf_interface = {
    .create = ring_buf_create,
    .write = _rb_write,
    .read = _rb_read,
    .clear = _rb_clear,
    .is_full = _rb_is_full,
    .is_empty = _rb_is_empty,
    .get_size = _rb_get_size,
    .get_capacity = _rb_get_capacity,
};

const FrameParserInterface frame_parser_interface = {
    .create = frame_parser_create,
    .write = _fp_write_,
    .process = _fp_process_,
    .get_frame = _fp_get_frame_,
    .finish = _fp_finish_,
    .reset = _fp_reset_,
};

/**
 * @brief 环形缓冲区构造函数
 * @param self 指向 RingBuf 结构体的指针
 * @param buf 指向用于存储数据的缓冲区的指针
 * @param capacity 缓冲区的容量
 * @param overwrite 是否允许覆盖旧数据，true 表示允许，false 表示不允许
 * @return RingBufErrorCode 枚举类型，表示操作结果
 */
RingBufErrorCode ring_buf_create(RingBuf* const self, uint8_t* const buf, const uint16_t capacity, const bool overwrite) {
    if(self == NULL) return RING_BUF_ERR_NULL_PTR;
    if(buf == NULL) return RING_BUF_ERR_NULL_PTR;
    if(capacity == 0) return RING_BUF_ERR_FULL;

    self->_buf_ = buf;
    self->_size_ = 0;
    self->_capacity_ = capacity;

    self->_write_idx_ = 0;
    self->_read_idx_ = 0;

    self->_overwrite_ = overwrite;
    self->_initialized_ = true;

    return RING_BUF_SUCCESS;
}

RingBufErrorCode RingBufCreate(RingBuf* const self, uint8_t* const buf, const uint16_t capacity, const bool overwrite) {
    return ring_buf_create(self, buf, capacity, overwrite);
}

/**
 * @brief 帧解析器构造函数
 * @param self 指向 FrameParser 结构体的指针
 * @param ring_buf 指向用于存储输入数据的环形缓冲区的指针
 * @param header 帧头标识指针，用于帧头匹配
 * @param header_length 帧头标识的长度，最小为 2 字节
 * @param frame_buf 指向用于存储解析完成的帧数据的缓冲区的指针
 * @param frame_buf_capacity 帧数据缓冲区的容量
 * @param crc_enabled 是否启用 CRC 校验，true 表示启用，false 表示不启用
 * @return FrameParserErrorCode 枚举类型，表示操作结果
 * @note 帧格式：[header][length_high][length_low][payload][crc_high][crc_low]，其中 crc 部分可选，由 crc_enabled 参数控制
 */
FrameParserErrorCode frame_parser_create(FrameParser* const self, RingBuf* const ring_buf, const uint8_t* const header, const uint8_t header_length, uint8_t* const frame_buf, const uint16_t frame_buf_capacity, const bool crc_enabled) {
    if(self == NULL) return FRAME_PARSER_ERR_NULL_PTR;
    if(ring_buf == NULL) return FRAME_PARSER_ERR_NULL_PTR;
    if(header == NULL) return FRAME_PARSER_ERR_NULL_PTR;
    if(header_length < 2) return FRAME_PARSER_ERR_HEADER_TOO_SHORT;
    if(frame_buf == NULL) return FRAME_PARSER_ERR_NULL_PTR;
    if(frame_buf_capacity == 0) return FRAME_PARSER_ERR_BUF_TOO_SMALL;

    self->_ring_buf_ = ring_buf;
    self->_state_ = STATE_IDLE;

    self->_header_ = header;
    self->_header_length_ = header_length;
    self->_header_match_idx_ = 0;

    self->_frame_buf_ = frame_buf;
    self->_frame_buf_capacity_ = (uint16_t)(frame_buf_capacity - (crc_enabled ? 2 : 0));

    self->_crc_enabled_ = crc_enabled;
    self->_crc_accum_ = 0;
    self->_received_crc_ = 0;
    self->_initialized_ = true;

    self->_expected_length_ = 0;
    self->_received_length_ = 0;

    return FRAME_PARSER_SUCCESS;

}

FrameParserErrorCode FrameParserCreate(FrameParser* const self, RingBuf* const ring_buf, const uint8_t* const header, const uint8_t header_length, uint8_t* const frame_buf, const uint16_t frame_buf_capacity, const bool crc_enabled) {
    return frame_parser_create(self, ring_buf, header, header_length, frame_buf, frame_buf_capacity, crc_enabled);
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 向环形缓冲区写入数据
 * @param self 指向 RingBuf 结构体的指针
 * @param data 要写入的数据
 * @return RingBufErrorCode 枚举类型，表示操作结果
 */
RingBufErrorCode _rb_write(RingBuf* const self, const uint8_t data) {
    if(self == NULL) return RING_BUF_ERR_NULL_PTR;
    if(self->_initialized_ == false) return RING_BUF_ERR_NOT_INITIALIZE;

    protocol_parser_enter_critical();

    if(self->_overwrite_) {
        self->_buf_[self->_write_idx_] = data;
        self->_write_idx_ = (uint16_t)((self->_write_idx_ + 1) % self->_capacity_);
        if(_rb_is_full(self)) self->_read_idx_ = (uint16_t)((self->_read_idx_ + 1) % self->_capacity_);
        else self->_size_++;
    }
    else {
        if(_rb_is_full(self)) {
            protocol_parser_exit_critical();
            return RING_BUF_ERR_FULL;
        }
        self->_buf_[self->_write_idx_] = data;
        self->_write_idx_ = (uint16_t)((self->_write_idx_ + 1) % self->_capacity_);
        self->_size_++;
    }

    protocol_parser_exit_critical();

    return RING_BUF_SUCCESS;
}

/**
 * @brief 从环形缓冲区读取数据
 * @param self 指向 RingBuf 结构体的指针
 * @param data 指向存储读取数据的变量的指针
 * @return RingBufErrorCode 枚举类型，表示操作结果
 */
RingBufErrorCode _rb_read(RingBuf* const self, uint8_t* const data) {
    if(self == NULL) return RING_BUF_ERR_NULL_PTR;
    if(self->_initialized_ == false) return RING_BUF_ERR_NOT_INITIALIZE;
    if(data == NULL) return RING_BUF_ERR_NULL_PTR;
    if(_rb_is_empty(self)) return RING_BUF_ERR_EMPTY;

    protocol_parser_enter_critical();

    *data = self->_buf_[self->_read_idx_];
    self->_read_idx_ = (uint16_t)((self->_read_idx_ + 1) % self->_capacity_);
    self->_size_--;

    protocol_parser_exit_critical();

    return RING_BUF_SUCCESS;
}

/**
 * @brief 清空环形缓冲区
 * @param self 指向 RingBuf 结构体的指针
 * @return RingBufErrorCode 枚举类型，表示操作结果
 */
RingBufErrorCode _rb_clear(RingBuf* const self) {
    if(self == NULL) return RING_BUF_ERR_NULL_PTR;
    if(self->_initialized_ == false) return RING_BUF_ERR_NOT_INITIALIZE;

    protocol_parser_enter_critical();

    self->_size_ = 0;
    self->_write_idx_ = 0;
    self->_read_idx_ = 0;

    protocol_parser_exit_critical();

    return RING_BUF_SUCCESS;
}

/**
 * @brief 检查环形缓冲区是否已满
 * @param self 指向 RingBuf 结构体的指针
 * @return true 表示已满，false 表示未满或参数无效
 */
bool _rb_is_full(RingBuf* const self) {
    if(self == NULL) return false;
    if(self->_initialized_ == false) return false;

    return self->_size_ >= self->_capacity_;
}

/**
 * @brief 检查环形缓冲区是否为空
 * @param self 指向 RingBuf 结构体的指针
 * @return true 表示为空，false 表示非空或参数无效
 */
bool _rb_is_empty(RingBuf* const self) {
    if(self == NULL) return false;
    if(self->_initialized_ == false) return false;

    return self->_size_ == 0;
}

/**
 * @brief 获取环形缓冲区当前存储的数据量
 * @param self 指向 RingBuf 结构体的指针
 * @return 当前存储的数据量，-1 表示错误
 */
int _rb_get_size(RingBuf* const self) {
    if(self == NULL) return -1;
    if(self->_initialized_ == false) return -1;

    return (int)(self->_size_);
}

/**
 * @brief 获取环形缓冲区的总容量
 * @param self 指向 RingBuf 结构体的指针
 * @return 环形缓冲区的总容量，-1 表示错误
 */
int _rb_get_capacity(RingBuf* const self) {
    if(self == NULL) return -1;
    if(self->_initialized_ == false) return -1;

    return (int)(self->_capacity_);
}

/**
 * @brief 向帧解析器输入单个数据
 * @param self 指向 FrameParser 结构体的指针
 * @param data 要输入的数据
 * @return FrameParserErrorCode 枚举类型，表示操作结果
 */
static FrameParserErrorCode _fp_write_(FrameParser* const self, const uint8_t data) {
    if(self == NULL) return FRAME_PARSER_ERR_NULL_PTR;
    if(self->_initialized_ == false) return FRAME_PARSER_ERR_NOT_INITIALIZE;

    switch(_rb_write(self->_ring_buf_, data)) {
        case RING_BUF_SUCCESS:
            break;
        case RING_BUF_ERR_NULL_PTR:
            return FRAME_PARSER_ERR_NULL_PTR;
        case RING_BUF_ERR_FULL:
            return FRAME_PARSER_ERR_BUFFER_FULL;
        default:
            return FRAME_PARSER_ERR_INVALID_STATE;
    }

    return FRAME_PARSER_SUCCESS;
}

/**
 * @brief 处理帧解析器状态机
 * @param self 指向 FrameParser 结构体的指针
 * @return FrameParserErrorCode 枚举类型，表示操作结果
 */
static FrameParserErrorCode _fp_process_(FrameParser* const self) {
    if(self == NULL) return FRAME_PARSER_ERR_NULL_PTR;
    if(self->_initialized_ == false) return FRAME_PARSER_ERR_NOT_INITIALIZE;

    uint8_t byte;
    while(_rb_read(self->_ring_buf_, &byte) == RING_BUF_SUCCESS) {
        switch(self->_state_) {
            case STATE_IDLE:
            {
                if(byte == self->_header_[0]) {
                    self->_crc_accum_ = crc16_update(0xFFFFU, byte);
                    self->_header_match_idx_ = 1;
                    self->_state_ = STATE_HEADER_MATCHING;
                }
                break;
            }
            case STATE_HEADER_MATCHING:
            {
                if(byte == self->_header_[self->_header_match_idx_]) {
                    self->_crc_accum_ = crc16_update(self->_crc_accum_, byte);
                    ++self->_header_match_idx_;
                    if(self->_header_match_idx_ >= self->_header_length_) self->_state_ = STATE_READ_LENGTH;
                }
                else {
                    self->_state_ = (byte == self->_header_[0]) ? STATE_HEADER_MATCHING : STATE_IDLE;
                    self->_crc_accum_ = (byte == self->_header_[0]) ? crc16_update(0xFFFFU, byte) : 0;
                    self->_header_match_idx_ = (byte == self->_header_[0]) ? 1 : 0;
                }
                break;
            }
            case STATE_READ_LENGTH:
            {
                self->_crc_accum_ = crc16_update(self->_crc_accum_, byte);
                if(self->_received_length_ == 0) {
                    self->_expected_length_ = (uint16_t)((uint16_t)byte << 8);
                    self->_received_length_ = 1;
                }
                else {
                    self->_expected_length_ |= (uint16_t)byte;
                    self->_received_length_ = 0;

                    if(self->_expected_length_ > self->_frame_buf_capacity_) {
                        self->_state_ = STATE_IDLE;
                        return FRAME_PARSER_ERR_LENGTH_EXCEED;
                    }
                    else if(self->_expected_length_ == 0) {
                        if(self->_crc_enabled_ == false) {
                            self->_state_ = STATE_FRAME_COMPLETE;
                            return FRAME_PARSER_SUCCESS;
                        }
                        self->_state_ = STATE_READ_CRC;
                    }
                    else {
                        self->_state_ = STATE_READ_PAYLOAD;
                    }
                }
                break;
            }
            case STATE_READ_PAYLOAD:
            {
                self->_crc_accum_ = crc16_update(self->_crc_accum_, byte);
                self->_frame_buf_[self->_received_length_] = byte;
                self->_received_length_++;

                if(self->_received_length_ >= self->_expected_length_) {
                    self->_received_length_ = 0;
                    self->_received_crc_ = 0;
                    self->_state_ = STATE_READ_CRC;
                }
                break;
            }
            case STATE_READ_CRC:
            {
                if(self->_crc_enabled_ == false) {
                    self->_state_ = STATE_FRAME_COMPLETE;
                    return FRAME_PARSER_SUCCESS;
                }
                if(self->_received_length_ == 0) {
                    self->_received_crc_ = (uint16_t)((uint16_t)byte << 8);
                    self->_received_length_ = 1;
                }
                else {
                    self->_received_crc_ |= (uint16_t)byte;
                    self->_received_length_ = 0;

                    if(self->_received_crc_ == self->_crc_accum_) {
                        self->_state_ = STATE_FRAME_COMPLETE;
                        return FRAME_PARSER_SUCCESS;
                    }
                    else {
                        self->_state_ = STATE_IDLE;
                        return FRAME_PARSER_ERR_CRC_MISMATCH;
                    }
                }
                break;
            }
            case STATE_FRAME_COMPLETE:
            {
                return FRAME_PARSER_SUCCESS;
            }
            default:
            {
                return FRAME_PARSER_ERR_INVALID_STATE;
            }
        }
    }

    return FRAME_PARSER_PROCESSING;
}

/**
 * @brief 获取解析完成的帧数据，仅在 process() 返回成功且状态为 STATE_FRAME_COMPLETE 时有效
 * @param self 指向 FrameParser 结构体的指针
 * @param frame_buffer 输出参数，指向存储帧数据的缓冲区指针
 * @param frame_length 输出参数，指向存储帧数据长度的变量指针
 * @return FrameParserErrorCode 枚举类型，表示操作结果
 */
static FrameParserErrorCode _fp_get_frame_(FrameParser* const self, uint8_t** const frame_buffer, uint16_t* const frame_length) {
    if(self == NULL) return FRAME_PARSER_ERR_NULL_PTR;
    if(self->_initialized_ == false) return FRAME_PARSER_ERR_NOT_INITIALIZE;
    if(frame_buffer == NULL) return FRAME_PARSER_ERR_NULL_PTR;
    if(frame_length == NULL) return FRAME_PARSER_ERR_NULL_PTR;
    if(self->_state_ != STATE_FRAME_COMPLETE) return FRAME_PARSER_ERR_NO_FRAME;

    *frame_buffer = self->_frame_buf_;
    *frame_length = self->_expected_length_;

    return FRAME_PARSER_SUCCESS;
}

/**
 * @brief 标记当前帧已处理完毕，将状态机复位至空闲
 * @param self 指向 FrameParser 结构体的指针
 * @return FrameParserErrorCode 枚举类型，表示操作结果
 */
static FrameParserErrorCode _fp_finish_(FrameParser* const self) {
    if(self == NULL) return FRAME_PARSER_ERR_NULL_PTR;
    if(self->_initialized_ == false) return FRAME_PARSER_ERR_NOT_INITIALIZE;

    self->_state_ = STATE_IDLE;
    self->_header_match_idx_ = 0;
    self->_expected_length_ = 0;
    self->_received_length_ = 0;
    self->_crc_accum_ = 0;
    self->_received_crc_ = 0;

    return FRAME_PARSER_SUCCESS;
}

/**
 * @brief 完全重置帧解析器，同时清空环形缓冲区
 * @param self 指向 FrameParser 结构体的指针
 * @return FrameParserErrorCode 枚举类型，表示操作结果
 */
static FrameParserErrorCode _fp_reset_(FrameParser* const self) {
    if(self == NULL) return FRAME_PARSER_ERR_NULL_PTR;
    if(self->_initialized_ == false) return FRAME_PARSER_ERR_NOT_INITIALIZE;

    _rb_clear(self->_ring_buf_);

    return _fp_finish_(self);
}

/**
 * @brief CRC-16/CCITT 单字节更新
 * @details 多项式: 0x1021，初始值: 0xFFFF，无输入/输出翻转，无最终异或
 *          CRC 覆盖范围: 帧头 + 长度字段 + 有效载荷
 * @param crc 当前 CRC 累加值
 * @param data 待计算的字节
 * @return 更新后的 CRC 值
 */
static uint16_t crc16_update(uint16_t crc, const uint8_t data) {
    crc ^= (uint16_t)data << 8;
    for(uint8_t i = 0; i < 8; i++) {
        crc = (crc & 0x8000U) ? ((uint16_t)(crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
    return crc;
}

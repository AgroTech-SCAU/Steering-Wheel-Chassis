#ifndef _protocol_parser_h_
#define _protocol_parser_h_

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 原子性操作，需要用户自行实现
 * @param protocol_parser_enter_critical 进入临界区，禁止中断或其他并发访问
 * @param protocol_parser_exit_critical 退出临界区，恢复中断或其他
 */
void protocol_parser_enter_critical(void);
void protocol_parser_exit_critical(void);

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 环形缓冲区入口单例，用户自定义名称
 */
#define ring_buf ring_buf_interface

/**
 * @brief 帧解析器入口单例，用户自定义名称
 */
#define frame_parser frame_parser_interface

/**
 * @brief 环形缓冲区错误枚举类型
 * @param RING_BUF_SUCCESS: 操作成功
 * @param RING_BUF_ERR_NULL_PTR: 指针为 NULL 错误
 * @param RING_BUF_ERR_IN_USE: 缓冲区正被使用错误
 * @param RING_BUF_ERR_FULL: 缓冲区已满错误
 * @param RING_BUF_ERR_EMPTY: 缓冲区为空错误
 */
typedef enum {
    RING_BUF_SUCCESS = 0,
    RING_BUF_ERR_NULL_PTR,
    RING_BUF_ERR_IN_USE,
    RING_BUF_ERR_FULL,
    RING_BUF_ERR_EMPTY,
    RING_BUF_ERR_NOT_INITIALIZE,
} RingBufErrorCode;

/**
 * @brief 帧解析器错误枚举类型
 * @param FRAME_PARSER_SUCCESS: 操作成功
 * @param FRAME_PARSER_PROCESSING: 解析中，尚未完成
 * @param FRAME_PARSER_ERR_HEADER_TOO_SHORT: 帧头过短错误
 * @param FRAME_PARSER_ERR_NULL_PTR: 指针为 NULL 错误
 * @param FRAME_PARSER_ERR_INVALID_STATE: 无效状态错误
 * @param FRAME_PARSER_ERR_BUFFER_FULL: 缓冲区已满错误
 * @param FRAME_PARSER_ERR_LENGTH_EXCEED: 帧长度超过预期错误
 * @param FRAME_PARSER_ERR_NO_FRAME: 没有可用帧错误
 * @param FRAME_PARSER_ERR_CRC_MISMATCH: CRC 校验失败错误
 */
typedef enum {
    FRAME_PARSER_SUCCESS = 0,
    FRAME_PARSER_PROCESSING,
    FRAME_PARSER_ERR_HEADER_TOO_SHORT,
    FRAME_PARSER_ERR_NULL_PTR,
    FRAME_PARSER_ERR_BUF_TOO_SMALL,
    FRAME_PARSER_ERR_INVALID_STATE,
    FRAME_PARSER_ERR_BUFFER_FULL,
    FRAME_PARSER_ERR_LENGTH_EXCEED,
    FRAME_PARSER_ERR_NO_FRAME,
    FRAME_PARSER_ERR_CRC_MISMATCH,
    FRAME_PARSER_ERR_NOT_INITIALIZE,
} FrameParserErrorCode;

/**
 * @brief 帧解析器状态枚举类型
 * @param STATE_IDLE: 空闲状态
 * @param STATE_HEADER_MATCHING: 帧头匹配状态
 * @param STATE_READ_LENGTH: 读取帧长度状态
 * @param STATE_READ_PAYLOAD: 读取帧数据状态
 * @param STATE_READ_CRC: 读取 CRC 校验值状态
 * @param STATE_FRAME_COMPLETE: 帧解析完成状态
 */
typedef enum {
    STATE_IDLE = 0,
    STATE_HEADER_MATCHING,
    STATE_READ_LENGTH,
    STATE_READ_PAYLOAD,
    STATE_READ_CRC,
    STATE_FRAME_COMPLETE,
} FrameParserState;

/**
 * @brief 环形缓冲区结构体定义
 */
typedef struct {
    // 缓冲区指针
    uint8_t* _buf_;
    // 缓冲区当前数据量
    uint16_t _size_;
    // 缓冲区总容量
    uint16_t _capacity_;

    // 写入索引
    uint16_t _write_idx_;
    // 读取索引
    uint16_t _read_idx_;

    // 是否允许覆盖旧数据
    bool _overwrite_;
    // 是否已初始化
    bool _initialized_;
} RingBuf;

/**
 * @brief 帧解析器结构体定义
 */
typedef struct {
    // 指向关联的环形缓冲区的指针
    RingBuf* _ring_buf_;
    // 帧解析器当前状态
    FrameParserState _state_;

    // 帧头
    const uint8_t* _header_;
    // 帧头长度，最小为 2 字节
    uint8_t _header_length_;
    // 帧头匹配索引
    uint8_t _header_match_idx_;

    // 预期的帧长度
    uint16_t _expected_length_;
    // 已接收的帧长度
    uint16_t _received_length_;

    // 帧数据缓冲区指针
    uint8_t* _frame_buf_;
    // 帧数据缓冲区总容量
    uint16_t _frame_buf_capacity_;

    // 是否启用 CRC 校验
    bool _crc_enabled_;
    // CRC 校验累加器
    uint16_t _crc_accum_;
    // 已接收的 CRC 校验值
    uint16_t _received_crc_;
    // 是否已初始化
    bool _initialized_;
} FrameParser;

typedef struct {
    RingBufErrorCode(*create)(RingBuf* const self, uint8_t* const buf, const uint16_t capacity, const bool overwrite);
    RingBufErrorCode(*write)(RingBuf* const self, const uint8_t data);
    RingBufErrorCode(*read)(RingBuf* const self, uint8_t* const data);
    RingBufErrorCode(*clear)(RingBuf* const self);
    bool(*is_full)(RingBuf* const self);
    bool(*is_empty)(RingBuf* const self);
    int(*get_size)(RingBuf* const self);
    int(*get_capacity)(RingBuf* const self);
} RingBufInterface;

typedef struct {
    FrameParserErrorCode(*create)(FrameParser* const self, RingBuf* const ring_buf, const uint8_t* const header, const uint8_t header_length, uint8_t* const frame_buf, const uint16_t frame_buf_capacity, const bool crc_enabled);
    FrameParserErrorCode(*write)(FrameParser* const self, const uint8_t data);
    FrameParserErrorCode(*process)(FrameParser* const self);
    FrameParserErrorCode(*get_frame)(FrameParser* const self, uint8_t** const frame_buffer, uint16_t* const frame_length);
    FrameParserErrorCode(*finish)(FrameParser* const self);
    FrameParserErrorCode(*reset)(FrameParser* const self);
} FrameParserInterface;

extern const RingBufInterface ring_buf_interface;
extern const FrameParserInterface frame_parser_interface;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

RingBufErrorCode ring_buf_create(RingBuf* const self, uint8_t* const buf, const uint16_t capacity, const bool overwrite);
FrameParserErrorCode frame_parser_create(FrameParser* const self, RingBuf* const ring_buf, const uint8_t* const header, const uint8_t header_length, uint8_t* const frame_buf, const uint16_t frame_buf_capacity, const bool crc_enabled);

#endif

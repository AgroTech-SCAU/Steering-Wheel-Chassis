#include "ft_scs_servo.h"

#include <string.h>

// ! ========================= 私 有 常 量 ========================= ! //

/**
 * @brief SCS 帧头字节
 */
#define FT_SCS_HEADER 0xFFu

/**
 * @brief 内部协议帧缓冲区容量
 */
#define FT_SCS_FRAME_MAX 128u

/**
 * @brief SCS 广播 ID
 */
#define FT_SCS_MAX_SERVO_ID 254u

/**
 * @brief 从 ACC 到 GOAL_SPEED_H 的控制数据长度
 */
#define FT_SCS_POS_PACKET_LEN 7u

/**
 * @brief 默认加速度原始值
 */
#define FT_SCS_DEFAULT_ACC 50u

/**
 * @brief 从 PRESENT_POSITION_L 到 PRESENT_LOAD_H 的反馈读取长度
 */
#define FT_SCS_FEEDBACK_LEN ((uint8_t)(FT_SCS_SERVO_PRESENT_LOAD_H - FT_SCS_SERVO_PRESENT_POSITION_L + 1u))

/**
 * @brief 默认应答超时时间, 单位 ms
 */
#define FT_SCS_DEFAULT_TIMEOUT_MS 100u

/**
 * @brief 位置换算比例, 每圈原始计数
 */
#define FT_SCS_POS_RAW_PER_REV 4096.0f

/**
 * @brief 速度换算比例, 每秒一圈对应的原始计数
 *
 * STS/SCS 速度单位约为 0.0146 rpm/count, 因此一圈每秒约等于 4096 count.
 */
#define FT_SCS_SPEED_RAW_PER_REV_S 4096.0f

/**
 * @brief STS3215 满量程扭矩估计, 单位 N*m
 */
#define FT_SCS_TORQUE_NM_FULL_SCALE 2.94f

/**
 * @brief 负载寄存器原始满量程值
 */
#define FT_SCS_LOAD_RAW_FULL_SCALE 1000.0f

/**
 * @brief 弧度换算使用的 2pi 常量
 */
#define FT_SCS_TWO_PI 6.28318530718f

// ! ========================= 私 有 类 型 ========================= ! //

/**
 * @brief FEETECH SCS 驱动运行上下文
 */
typedef struct {
    const BusServoPortOps* ops;
    uint32_t timeout_ms;
    uint8_t retry_count;
    BusServoEndian endian;
    bool initialized;
    uint8_t last_error_code;
    BusServoFeedback feedback[FT_SCS_MAX_SERVO_ID + 1u];
    bool has_feedback[FT_SCS_MAX_SERVO_ID + 1u];
    uint8_t last_tx_id;
    uint8_t last_tx_instruction;
    uint8_t last_tx_params[FT_SCS_FRAME_MAX];
    uint8_t last_tx_params_len;
    bool has_last_tx;
} FtScsBusServoContext;

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 单例运行上下文
 */
static FtScsBusServoContext s_ctx;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 初始化 SCS 通用舵机实例
 * @param config 初始化配置
 * @return 状态码
 */
static BusServoStatus ft_scs_common_init(const void* config);

/**
 * @brief 将状态码转换为字符串
 * @param status 状态码
 * @return 状态码名称字符串
 */
static const char* ft_scs_common_status_str(BusServoStatus status);

/**
 * @brief 让舵机一直以指定速度旋转
 * @param id 舵机 ID
 * @param speed 目标速度, 单位 rad/s
 * @return 状态码
 */
static BusServoStatus ft_scs_common_set_speed(uint8_t id, float speed);

/**
 * @brief 让舵机以指定速度旋转到指定位置
 * @param id 舵机 ID
 * @param position 目标位置, 单位 rad
 * @param velocity 目标速度, 单位 rad/s
 * @return 状态码
 */
static BusServoStatus ft_scs_common_set_pos_spd(uint8_t id, float position, float velocity);

/**
 * @brief 让舵机以指定速度和扭矩旋转到指定位置并保持扭矩
 * @param id 舵机 ID
 * @param position 目标位置, 单位 rad
 * @param velocity 目标速度, 单位 rad/s
 * @param torque 保持扭矩或负载限制, 单位 N*m
 * @return 状态码
 */
static BusServoStatus ft_scs_common_set_pos_spd_tor(uint8_t id, float position, float velocity, float torque);

/**
 * @brief 获取缓存的位置
 * @param id 舵机 ID
 * @return 位置, 单位 rad
 */
static float ft_scs_common_get_position(uint8_t id);

/**
 * @brief 获取缓存的速度
 * @param id 舵机 ID
 * @return 速度, 单位 rad/s
 */
static float ft_scs_common_get_speed(uint8_t id);

/**
 * @brief 获取缓存的扭矩
 * @param id 舵机 ID
 * @return 扭矩或负载估计, 单位 N*m
 */
static float ft_scs_common_get_torque(uint8_t id);

/**
 * @brief 从总线读取反馈并更新缓存
 * @param id 舵机 ID
 * @param feedback 可选的反馈输出指针
 * @return 状态码
 */
static BusServoStatus ft_scs_common_update_feedback(uint8_t id, BusServoFeedback* feedback);

/**
 * @brief 发送 PING 并等待状态帧
 * @param id 舵机 ID
 * @param out_id 可选的 ID 输出指针
 * @return 状态码
 */
static BusServoStatus ft_scs_ping(uint8_t id, uint8_t* out_id);

/**
 * @brief 打开扭矩输出
 * @param id 舵机 ID
 * @return 状态码
 */
static BusServoStatus ft_scs_enable_torque(uint8_t id);

/**
 * @brief 关闭扭矩输出
 * @param id 舵机 ID
 * @return 状态码
 */
static BusServoStatus ft_scs_disable_torque(uint8_t id);

/**
 * @brief 触发 REG_WRITE 命令
 * @param id 舵机 ID 或广播 ID
 * @return 状态码
 */
static BusServoStatus ft_scs_action(uint8_t id);

/**
 * @brief 向控制表寄存器写入 1 字节
 * @param id 舵机 ID
 * @param addr 寄存器地址
 * @param value 写入值
 * @return 状态码
 */
static BusServoStatus ft_scs_write_u8(uint8_t id, uint8_t addr, uint8_t value);

/**
 * @brief 向控制表寄存器写入 2 字节
 * @param id 舵机 ID
 * @param addr 寄存器地址
 * @param value 写入值
 * @return 状态码
 */
static BusServoStatus ft_scs_write_u16(uint8_t id, uint8_t addr, uint16_t value);

/**
 * @brief 从控制表寄存器读取 1 字节
 * @param id 舵机 ID
 * @param addr 寄存器地址
 * @param value 读取值输出指针
 * @return 状态码
 */
static BusServoStatus ft_scs_read_u8(uint8_t id, uint8_t addr, uint8_t* value);

/**
 * @brief 从控制表寄存器读取 2 字节
 * @param id 舵机 ID
 * @param addr 寄存器地址
 * @param value 读取值输出指针
 * @return 状态码
 */
static BusServoStatus ft_scs_read_u16(uint8_t id, uint8_t addr, uint16_t* value);

/**
 * @brief 发送原始 SCS 帧
 * @param id 舵机 ID
 * @param instruction SCS 指令码
 * @param params 参数缓冲区
 * @param params_len 参数长度
 * @param need_ack 为真表示尽可能等待状态帧
 * @return 状态码
 */
static BusServoStatus ft_scs_write_packet(uint8_t id, uint8_t instruction, const uint8_t* params, uint8_t params_len, bool need_ack);

/**
 * @brief 检查初始化状态和必要端口函数
 * @return 状态码
 */
static BusServoStatus validate_initialized(void);

/**
 * @brief 在超时前读取指定数量的字节
 * @param data 接收缓冲区
 * @param len 期望字节数
 * @return 状态码
 */
static BusServoStatus read_exact_until(uint8_t* data, uint16_t len, uint32_t start);

/**
 * @brief 读取并解析状态帧
 * @param expected_id 期望舵机 ID
 * @param params 参数输出缓冲区
 * @param params_cap 参数输出缓冲区容量
 * @param out_params_len 可选的已解析参数长度输出指针
 * @param out_error 可选的原始错误码输出指针
 * @return 状态码
 */
static BusServoStatus read_status_packet(uint8_t expected_id, uint8_t* params, uint8_t params_cap, uint8_t* out_params_len, uint8_t* out_error);

/**
 * @brief 读取连续控制表数据块
 * @param id 舵机 ID
 * @param addr 起始寄存器地址
 * @param data 输出缓冲区
 * @param len 读取长度
 * @return 状态码
 */
static BusServoStatus read_data(uint8_t id, uint8_t addr, uint8_t* data, uint8_t len);

/**
 * @brief 写入连续控制表数据块
 * @param id 舵机 ID
 * @param addr 起始寄存器地址
 * @param data 输入缓冲区
 * @param len 写入长度
 * @param instruction WRITE 或 REG_WRITE 指令
 * @return 状态码
 */
static BusServoStatus write_data(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t len, uint8_t instruction, bool need_ack);

/**
 * @brief 处理反馈数据并更新缓存
 * @param feedback 反馈数据
 */
static void cache_feedback(const BusServoFeedback* feedback);

/**
 * @brief 解析反馈数据并转换为反馈结构
 * @param id 舵机 ID
 * @param data 反馈数据缓冲区
 * @param len 反馈数据长度
 * @param feedback 反馈输出指针
 * @return 状态码
 */
static BusServoStatus parse_feedback(uint8_t id, const uint8_t* data, uint8_t len, BusServoFeedback* feedback);

/**
 * @brief 构造加速度, 位置, 时间和速度控制数据
 * @param data 输出缓冲区, 长度至少为 FT_SCS_POS_PACKET_LEN
 * @param position_raw 目标原始位置
 * @param speed_raw 目标原始速度
 * @param acc 原始加速度值
 */
static void build_position_data(uint8_t* data, uint16_t position_raw, uint16_t speed_raw, uint8_t acc);

/**
 * @brief 计算 SCS 校验和
 * @param data 帧头之后的数据
 * @param len 数据长度
 * @return 校验和字节
 */
static uint8_t checksum(const uint8_t* data, uint16_t len);

/**
 * @brief 判断字节是否为 SCS 指令码
 * @param value 待判断的字节
 * @return 为真表示该字节是已知 SCS 指令码
 */
static bool is_echo_instruction(uint8_t instruction, const uint8_t* params, uint8_t params_len);

/**
 * @brief 根据字节序将两个字节组合为 16 位寄存器值
 * @param low 寄存器块中的第一个字节
 * @param high 寄存器块中的第二个字节
 * @return 16 位寄存器值
 */
static uint16_t make_u16(uint8_t low, uint8_t high);

/**
 * @brief 根据字节序将 16 位寄存器值拆分为两个字节
 * @param value 16 位寄存器值
 * @param low 第一个输出字节
 * @param high 第二个输出字节
 */
static void split_u16(uint16_t value, uint8_t* low, uint8_t* high);

/**
 * @brief 将弧度位置转换为原始计数
 * @param position 位置, 单位 rad
 * @return 原始位置计数
 */
static uint16_t position_rad_to_raw(float position);

/**
 * @brief 将原始计数转换为弧度位置
 * @param raw 原始位置计数
 * @return 位置, 单位 rad
 */
static float raw_to_position_rad(uint16_t raw);

/**
 * @brief 将 rad/s 速度转换为原始带符号计数
 * @param speed 速度, 单位 rad/s
 * @return 原始带符号速度计数
 */
static int16_t speed_rad_s_to_raw(float speed);

/**
 * @brief 将原始带符号计数转换为 rad/s 速度
 * @param raw 原始带符号速度计数
 * @return 速度, 单位 rad/s
 */
static float raw_to_speed_rad_s(int16_t raw);

/**
 * @brief 将 N*m 扭矩限制转换为原始寄存器值
 * @param torque 扭矩或负载限制, 单位 N*m
 * @return 原始扭矩计数
 */
static uint16_t torque_to_raw(float torque);

/**
 * @brief 将原始带符号负载转换为 N*m 扭矩估计
 * @param raw 原始带符号负载值
 * @return 扭矩或负载估计, 单位 N*m
 */
static float raw_to_torque(int16_t raw);

// ! ========================= 接 口 实 例 ========================= ! //

/**
 * @brief SCS 驱动实现的 BusServoInterface 实例
 */
const BusServoInterface ft_scs_servo_common_instance = {
    .init = ft_scs_common_init,
    .status_str = ft_scs_common_status_str,
    .set_speed = ft_scs_common_set_speed,
    .set_pos_spd = ft_scs_common_set_pos_spd,
    .set_pos_spd_tor = ft_scs_common_set_pos_spd_tor,
    .get_position = ft_scs_common_get_position,
    .get_speed = ft_scs_common_get_speed,
    .get_torque = ft_scs_common_get_torque,
    .update_feedback = ft_scs_common_update_feedback,
};

/**
 * @brief 具体 SCS 特色接口表
 */
static const FtScsBusServoInterface s_ft_scs_servo_feature_instance = {
    .ping = ft_scs_ping,
    .enable_torque = ft_scs_enable_torque,
    .disable_torque = ft_scs_disable_torque,
    .action = ft_scs_action,
    .write_u8 = ft_scs_write_u8,
    .write_u16 = ft_scs_write_u16,
    .read_u8 = ft_scs_read_u8,
    .read_u16 = ft_scs_read_u16,
    .write_packet = ft_scs_write_packet,
};

/**
 * @brief FEETECH SCS 特色单例
 */
const FtScsBusServoInterface* ft_scs_servo_instance = &s_ft_scs_servo_feature_instance;

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 将 SCS 原始带符号值转换为普通有符号值
 */
int16_t ft_scs_servo_raw_to_signed(uint16_t value, uint8_t sign_bit) {
    uint16_t sign_mask = (uint16_t)(1u << sign_bit);
    uint16_t value_mask = (uint16_t)~sign_mask;

    if((value & sign_mask) != 0u) {
        return (int16_t)(-(int16_t)(value & value_mask));
    }

    return (int16_t)value;
}

/**
 * @brief 将普通有符号值转换为 SCS bit15 符号编码
 */
uint16_t ft_scs_servo_signed_to_raw(int16_t value) {
    uint16_t raw;

    if(value < 0) {
        raw = (uint16_t)(-value);
        raw |= (uint16_t)(1u << 15);
    }
    else {
        raw = (uint16_t)value;
    }

    return raw;
}

/**
 * @brief 同步读取多个舵机的反馈
 * @param ids 舵机 ID 数组
 * @param count 舵机数量
 * @param feedbacks 可选的反馈输出数组, 长度至少为 count
 * @param feedback_cap feedbacks 数组容量
 * @return 状态码
 */
BusServoStatus ft_scs_sync_read_feedback(const uint8_t* ids,
                                         uint8_t count,
                                         BusServoFeedback* feedbacks,
                                         uint8_t feedback_cap) {
    uint8_t params[2u + FT_SCS_MAX_SERVO_ID];
    uint8_t data[FT_SCS_FEEDBACK_LEN];
    BusServoStatus status;

    if(ids == 0 || count == 0u || count > FT_SCS_MAX_SERVO_ID) {
        return SERVO_STATUS_INVALID_PARAM;
    }
    if(feedbacks != 0 && feedback_cap < count) {
        return SERVO_STATUS_BUFFER_TOO_SMALL;
    }

    params[0] = FT_SCS_SERVO_PRESENT_POSITION_L;
    params[1] = FT_SCS_FEEDBACK_LEN;
    memcpy(&params[2], ids, count);

    status = ft_scs_write_packet(FT_SCS_SERVO_BROADCAST_ID, FT_SCS_SERVO_INST_SYNC_READ,
                                 params, (uint8_t)(count + 2u), false);
    if(status != SERVO_STATUS_OK) {
        return status;
    }

    for(uint8_t i = 0u; i < count; i++) {
        uint8_t rx_len = 0u;
        uint8_t error = 0u;
        BusServoFeedback parsed;

        status = read_status_packet(ids[i], data, sizeof(data), &rx_len, &error);
        if(status != SERVO_STATUS_OK) {
            return status;
        }
        if(error != 0u) {
            return SERVO_STATUS_ERROR;
        }
        if(rx_len != FT_SCS_FEEDBACK_LEN) {
            return SERVO_STATUS_BUFFER_TOO_SMALL;
        }

        status = parse_feedback(ids[i], data, sizeof(data), &parsed);
        if(status != SERVO_STATUS_OK) {
            return status;
        }
        cache_feedback(&parsed);
        if(feedbacks != 0) {
            feedbacks[i] = parsed;
        }
    }

    return SERVO_STATUS_OK;
}

/**
 * @brief 同步写入多个舵机的位置和速度
 * @param ids 舵机 ID 数组
 * @param positions 目标位置数组, 单位 rad
 * @param count 舵机数量
 * @param velocity 目标速度, 单位 rad/s
 * @return 状态码
 */
BusServoStatus ft_scs_sync_write_pos_spd(const uint8_t* ids,
                                         const float* positions,
                                         uint8_t count,
                                         float velocity) {
    uint8_t params[FT_SCS_FRAME_MAX];
    uint8_t frame[FT_SCS_FRAME_MAX];
    uint16_t params_len;
    uint16_t frame_len;
    uint16_t offset;
    uint16_t speed_raw;
    BusServoStatus status;

    if(ids == 0 || positions == 0 || count == 0u || count > FT_SCS_MAX_SERVO_ID) {
        return SERVO_STATUS_INVALID_PARAM;
    }
    if(velocity < 0.0f) {
        velocity = -velocity;
    }

    params_len = (uint16_t)(2u + ((uint16_t)count * (uint16_t)(1u + FT_SCS_POS_PACKET_LEN)));
    frame_len = (uint16_t)(params_len + 6u);
    if(frame_len > sizeof(frame)) {
        return SERVO_STATUS_BUFFER_TOO_SMALL;
    }

    params[0] = FT_SCS_SERVO_ACC;
    params[1] = FT_SCS_POS_PACKET_LEN;
    speed_raw = (uint16_t)speed_rad_s_to_raw(velocity);
    offset = 2u;
    for(uint8_t i = 0u; i < count; i++) {
        params[offset++] = ids[i];
        build_position_data(&params[offset], position_rad_to_raw(positions[i]), speed_raw, FT_SCS_DEFAULT_ACC);
        offset = (uint16_t)(offset + FT_SCS_POS_PACKET_LEN);
    }

    status = validate_initialized();
    if(status != SERVO_STATUS_OK) {
        return status;
    }

    frame[0] = FT_SCS_HEADER;
    frame[1] = FT_SCS_HEADER;
    frame[2] = FT_SCS_SERVO_BROADCAST_ID;
    frame[3] = (uint8_t)(params_len + 2u);
    frame[4] = FT_SCS_SERVO_INST_SYNC_WRITE;
    memcpy(&frame[5], params, params_len);
    frame[5u + params_len] = checksum(frame + 2u, (uint16_t)(3u + params_len));

    if(s_ctx.ops->flush_rx != 0) {
        s_ctx.ops->flush_rx();
    }

    return s_ctx.ops->write(frame, frame_len) ? SERVO_STATUS_OK : SERVO_STATUS_PORT_ERROR;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 初始化 SCS 通用舵机实例
 */
static BusServoStatus ft_scs_common_init(const void* config) {
    const FtScsServoConfig* scs_config = (const FtScsServoConfig*)config;

    if(scs_config == 0 || scs_config->ops == 0 || scs_config->ops->write == 0 ||
       scs_config->ops->read == 0 || scs_config->ops->now_ms == 0) {
        return SERVO_STATUS_INVALID_PARAM;
    }

    s_ctx.ops = scs_config->ops;
    s_ctx.timeout_ms = (scs_config->timeout_ms == 0u) ? FT_SCS_DEFAULT_TIMEOUT_MS : scs_config->timeout_ms;
    s_ctx.retry_count = scs_config->retry_count;
    s_ctx.endian = scs_config->endian;
    s_ctx.initialized = true;
    s_ctx.last_error_code = 0u;
    s_ctx.has_last_tx = false;
    s_ctx.last_tx_params_len = 0u;
    memset(s_ctx.feedback, 0, sizeof(s_ctx.feedback));
    memset(s_ctx.has_feedback, 0, sizeof(s_ctx.has_feedback));
    memset(s_ctx.last_tx_params, 0, sizeof(s_ctx.last_tx_params));

    return SERVO_STATUS_OK;
}

/**
 * @brief 将状态码转换为字符串
 */
static const char* ft_scs_common_status_str(BusServoStatus status) {
    switch(status) {
#define X(name, value)        \
    case SERVO_STATUS_##name: \
        return #name;
        SERVO_STATUS_TABLE
#undef X
        default:
            return "UNKNOWN";
    }
}

/**
 * @brief 让舵机一直以指定速度旋转
 */
static BusServoStatus ft_scs_common_set_speed(uint8_t id, float speed) {
    uint8_t data[FT_SCS_POS_PACKET_LEN];
    uint8_t mode = 1u;
    uint8_t torque_enable = 1u;
    int16_t speed_raw;

    BusServoStatus status = write_data(id, FT_SCS_SERVO_TORQUE_ENABLE, &torque_enable, 1u, FT_SCS_SERVO_INST_WRITE, true);
    if(status != SERVO_STATUS_OK) {
        return status;
    }

    status = write_data(id, FT_SCS_SERVO_MODE, &mode, 1u, FT_SCS_SERVO_INST_WRITE, true);
    if(status != SERVO_STATUS_OK) {
        return status;
    }

    speed_raw = speed_rad_s_to_raw(speed);
    build_position_data(data, 0u, ft_scs_servo_signed_to_raw(speed_raw), FT_SCS_DEFAULT_ACC);

    return write_data(id, FT_SCS_SERVO_ACC, data, sizeof(data), FT_SCS_SERVO_INST_WRITE, true);
}

/**
 * @brief 让舵机以指定速度旋转到指定位置
 */
static BusServoStatus ft_scs_common_set_pos_spd(uint8_t id, float position, float velocity) {
    uint8_t data[FT_SCS_POS_PACKET_LEN];
    int16_t speed_raw;

    if(velocity < 0.0f) {
        velocity = -velocity;
    }
    speed_raw = speed_rad_s_to_raw(velocity);
    build_position_data(data, position_rad_to_raw(position), (uint16_t)speed_raw, FT_SCS_DEFAULT_ACC);
    return write_data(id, FT_SCS_SERVO_ACC, data, sizeof(data), FT_SCS_SERVO_INST_WRITE, false);
}

/**
 * @brief 让舵机以指定速度和扭矩旋转到指定位置并保持扭矩
 */
static BusServoStatus ft_scs_common_set_pos_spd_tor(uint8_t id, float position, float velocity, float torque) {
    uint8_t data[2];
    BusServoStatus status;

    split_u16(torque_to_raw(torque), &data[0], &data[1]);
    status = write_data(id, FT_SCS_SERVO_TORQUE_LIMIT_L, data, sizeof(data), FT_SCS_SERVO_INST_WRITE, true);
    if(status != SERVO_STATUS_OK) {
        return status;
    }

    return ft_scs_common_set_pos_spd(id, position, velocity);
}

/**
 * @brief 获取缓存的位置
 */
static float ft_scs_common_get_position(uint8_t id) {
    if(id > FT_SCS_MAX_SERVO_ID || s_ctx.has_feedback[id] == false) {
        return 0.0f;
    }

    return s_ctx.feedback[id].position;
}

/**
 * @brief 获取缓存的速度
 */
static float ft_scs_common_get_speed(uint8_t id) {
    if(id > FT_SCS_MAX_SERVO_ID || s_ctx.has_feedback[id] == false) {
        return 0.0f;
    }

    return s_ctx.feedback[id].speed;
}

/**
 * @brief 获取缓存的扭矩
 */
static float ft_scs_common_get_torque(uint8_t id) {
    if(id > FT_SCS_MAX_SERVO_ID || s_ctx.has_feedback[id] == false) {
        return 0.0f;
    }

    return s_ctx.feedback[id].torque;
}

/**
 * @brief 从总线读取反馈并更新缓存
 */
static BusServoStatus ft_scs_common_update_feedback(uint8_t id, BusServoFeedback* feedback) {
    uint8_t data[FT_SCS_FEEDBACK_LEN];
    BusServoStatus status;
    BusServoFeedback parsed;

    status = read_data(id, FT_SCS_SERVO_PRESENT_POSITION_L, data, sizeof(data));
    if(status != SERVO_STATUS_OK) {
        return status;
    }

    status = parse_feedback(id, data, sizeof(data), &parsed);
    if(status != SERVO_STATUS_OK) {
        return status;
    }
    cache_feedback(&parsed);

    if(feedback != 0) {
        *feedback = parsed;
    }

    return SERVO_STATUS_OK;
}

/**
 * @brief 发送 PING 并等待状态帧
 */
static BusServoStatus ft_scs_ping(uint8_t id, uint8_t* out_id) {
    uint8_t error = 0u;
    BusServoStatus status = ft_scs_write_packet(id, FT_SCS_SERVO_INST_PING, 0, 0u, false);
    if(status != SERVO_STATUS_OK) {
        return status;
    }

    status = read_status_packet(id, 0, 0u, 0, &error);
    if(status != SERVO_STATUS_OK) {
        return status;
    }
    if(error != 0u) {
        return SERVO_STATUS_ERROR;
    }
    if(out_id != 0) {
        *out_id = id;
    }

    return SERVO_STATUS_OK;
}

/**
 * @brief 打开扭矩输出
 */
static BusServoStatus ft_scs_enable_torque(uint8_t id) {
    return ft_scs_write_u8(id, FT_SCS_SERVO_TORQUE_ENABLE, 1u);
}

/**
 * @brief 关闭扭矩输出
 */
static BusServoStatus ft_scs_disable_torque(uint8_t id) {
    return ft_scs_write_u8(id, FT_SCS_SERVO_TORQUE_ENABLE, 0u);
}

/**
 * @brief 触发 REG_WRITE 命令
 */
static BusServoStatus ft_scs_action(uint8_t id) {
    return ft_scs_write_packet(id, FT_SCS_SERVO_INST_ACTION, 0, 0u, id != FT_SCS_SERVO_BROADCAST_ID);
}

/**
 * @brief 向控制表寄存器写入 1 字节
 */
static BusServoStatus ft_scs_write_u8(uint8_t id, uint8_t addr, uint8_t value) {
    return write_data(id, addr, &value, 1u, FT_SCS_SERVO_INST_WRITE, true);
}

/**
 * @brief 向控制表寄存器写入 2 字节
 */
static BusServoStatus ft_scs_write_u16(uint8_t id, uint8_t addr, uint16_t value) {
    uint8_t data[2];

    split_u16(value, &data[0], &data[1]);
    return write_data(id, addr, data, sizeof(data), FT_SCS_SERVO_INST_WRITE, true);
}

/**
 * @brief 从控制表寄存器读取 1 字节
 */
static BusServoStatus ft_scs_read_u8(uint8_t id, uint8_t addr, uint8_t* value) {
    if(value == 0) {
        return SERVO_STATUS_INVALID_PARAM;
    }

    return read_data(id, addr, value, 1u);
}

/**
 * @brief 从控制表寄存器读取 2 字节
 */
static BusServoStatus ft_scs_read_u16(uint8_t id, uint8_t addr, uint16_t* value) {
    uint8_t data[2];
    BusServoStatus status;

    if(value == 0) {
        return SERVO_STATUS_INVALID_PARAM;
    }

    status = read_data(id, addr, data, sizeof(data));
    if(status != SERVO_STATUS_OK) {
        return status;
    }

    *value = make_u16(data[0], data[1]);
    return SERVO_STATUS_OK;
}

/**
 * @brief 发送原始 SCS 帧
 */
static BusServoStatus ft_scs_write_packet(uint8_t id, uint8_t instruction, const uint8_t* params, uint8_t params_len, bool need_ack) {
    BusServoStatus status = validate_initialized();
    uint8_t frame[FT_SCS_FRAME_MAX];
    uint8_t len;
    uint16_t frame_len;

    if(status != SERVO_STATUS_OK) {
        return status;
    }
    if(params == 0 && params_len != 0u) {
        return SERVO_STATUS_INVALID_PARAM;
    }

    len = (uint8_t)(params_len + 2u);
    frame_len = (uint16_t)(params_len + 6u);
    if(frame_len > sizeof(frame)) {
        return SERVO_STATUS_BUFFER_TOO_SMALL;
    }

    frame[0] = FT_SCS_HEADER;
    frame[1] = FT_SCS_HEADER;
    frame[2] = id;
    frame[3] = len;
    frame[4] = instruction;
    if(params_len > 0u) {
        memcpy(&frame[5], params, params_len);
    }
    frame[5u + params_len] = checksum(frame + 2u, (uint16_t)(3u + params_len));

    if(s_ctx.ops->flush_rx != 0) {
        s_ctx.ops->flush_rx();
    }

    s_ctx.last_tx_id = id;
    s_ctx.last_tx_instruction = instruction;
    s_ctx.last_tx_params_len = params_len;
    s_ctx.has_last_tx = true;
    if(params_len > 0u) {
        memcpy(s_ctx.last_tx_params, params, params_len);
    }

    if(s_ctx.ops->write(frame, frame_len) == false) {
        return SERVO_STATUS_PORT_ERROR;
    }

    if(need_ack && id != FT_SCS_SERVO_BROADCAST_ID) {
        uint8_t error = 0u;
        status = read_status_packet(id, 0, 0u, 0, &error);
        if(status != SERVO_STATUS_OK) {
            return status;
        }
        if(error != 0u) {
            return SERVO_STATUS_ERROR;
        }
    }

    return SERVO_STATUS_OK;
}

/**
 * @brief 检查初始化状态和必要端口函数
 */
static BusServoStatus validate_initialized(void) {
    if(s_ctx.initialized == false) {
        return SERVO_STATUS_NOT_INITIALIZE;
    }

    if(s_ctx.ops == 0 || s_ctx.ops->write == 0 || s_ctx.ops->read == 0 || s_ctx.ops->now_ms == 0) {
        return SERVO_STATUS_PORT_ERROR;
    }

    return SERVO_STATUS_OK;
}

/**
 * @brief 在超时前读取指定数量的字节
 */
static BusServoStatus read_exact_until(uint8_t* data, uint16_t len, uint32_t start) {
    uint16_t offset = 0u;

    if(data == 0 && len != 0u) {
        return SERVO_STATUS_INVALID_PARAM;
    }

    while(offset < len) {
        int got = s_ctx.ops->read(data + offset, (uint16_t)(len - offset));

        if(got > 0) {
            offset = (uint16_t)(offset + (uint16_t)got);
            continue;
        }

        if((s_ctx.ops->now_ms() - start) >= s_ctx.timeout_ms) {
            return SERVO_STATUS_TIMEOUT;
        }
    }

    return SERVO_STATUS_OK;
}

/**
 * @brief 读取并解析状态帧
 */
static BusServoStatus read_status_packet(uint8_t expected_id, uint8_t* params, uint8_t params_cap, uint8_t* out_params_len, uint8_t* out_error) {
    uint8_t b;
    uint8_t last;
    uint8_t id;
    uint8_t len;
    uint8_t err;
    uint8_t param_len;
    uint8_t packet_params[FT_SCS_FRAME_MAX];
    uint8_t rx_checksum;
    uint8_t sum;
    uint8_t i;
    uint32_t start;
    BusServoStatus status;

    status = validate_initialized();
    if(status != SERVO_STATUS_OK) {
        return status;
    }

    start = s_ctx.ops->now_ms();

    while((s_ctx.ops->now_ms() - start) < s_ctx.timeout_ms) {
        last = 0u;

        while((s_ctx.ops->now_ms() - start) < s_ctx.timeout_ms) {
            int got = s_ctx.ops->read(&b, 1u);
            if(got == 1) {
                if(last == FT_SCS_HEADER && b == FT_SCS_HEADER) {
                    break;
                }
                last = b;
            }
        }

        if((s_ctx.ops->now_ms() - start) >= s_ctx.timeout_ms) {
            return SERVO_STATUS_TIMEOUT;
        }

        status = read_exact_until(&id, 1u, start);
        if(status != SERVO_STATUS_OK) {
            return status;
        }
        status = read_exact_until(&len, 1u, start);
        if(status != SERVO_STATUS_OK) {
            return status;
        }
        status = read_exact_until(&err, 1u, start);
        if(status != SERVO_STATUS_OK) {
            return status;
        }

        if(len < 2u) {
            continue;
        }

        param_len = (uint8_t)(len - 2u);
        if(param_len > sizeof(packet_params)) {
            return SERVO_STATUS_BUFFER_TOO_SMALL;
        }

        if(param_len > 0u) {
            status = read_exact_until(packet_params, param_len, start);
            if(status != SERVO_STATUS_OK) {
                return status;
            }
        }

        status = read_exact_until(&rx_checksum, 1u, start);
        if(status != SERVO_STATUS_OK) {
            return status;
        }

        sum = (uint8_t)(id + len + err);
        for(i = 0u; i < param_len; i++) {
            sum = (uint8_t)(sum + packet_params[i]);
        }
        sum = (uint8_t)(~sum);
        if(sum != rx_checksum) {
            continue;
        }

        if(s_ctx.has_last_tx &&
           id == s_ctx.last_tx_id &&
           err == s_ctx.last_tx_instruction &&
           param_len == s_ctx.last_tx_params_len &&
           is_echo_instruction(err, packet_params, param_len) &&
           (param_len == 0u || memcmp(packet_params, s_ctx.last_tx_params, param_len) == 0)) {
            continue;
        }

        if(expected_id != FT_SCS_SERVO_BROADCAST_ID && id != expected_id) {
            continue;
        }
        if(param_len > params_cap) {
            return SERVO_STATUS_BUFFER_TOO_SMALL;
        }

        if(param_len > 0u && params != 0) {
            memcpy(params, packet_params, param_len);
        }
        if(out_params_len != 0) {
            *out_params_len = param_len;
        }
        if(out_error != 0) {
            *out_error = err;
        }

        s_ctx.last_error_code = err;
        s_ctx.has_last_tx = false;
        return SERVO_STATUS_OK;
    }

    return SERVO_STATUS_TIMEOUT;
}

/**
 * @brief 读取连续控制表数据块
 */
static BusServoStatus read_data(uint8_t id, uint8_t addr, uint8_t* data, uint8_t len) {
    uint8_t params[2];
    uint8_t rx_len = 0u;
    uint8_t error = 0u;
    BusServoStatus status;

    if(data == 0 || len == 0u) {
        return SERVO_STATUS_INVALID_PARAM;
    }

    params[0] = addr;
    params[1] = len;
    status = ft_scs_write_packet(id, FT_SCS_SERVO_INST_READ, params, sizeof(params), false);
    if(status != SERVO_STATUS_OK) {
        return status;
    }

    status = read_status_packet(id, data, len, &rx_len, &error);
    if(status != SERVO_STATUS_OK) {
        return status;
    }

    if(error != 0u) {
        return SERVO_STATUS_ERROR;
    }
    if(rx_len != len) {
        return SERVO_STATUS_BUFFER_TOO_SMALL;
    }

    return SERVO_STATUS_OK;
}

/**
 * @brief 写入连续控制表数据块
 */
static BusServoStatus write_data(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t len, uint8_t instruction, bool need_ack) {
    uint8_t params[FT_SCS_FRAME_MAX];

    if(data == 0 || len == 0u) {
        return SERVO_STATUS_INVALID_PARAM;
    }
    if((uint16_t)(len + 1u) > sizeof(params)) {
        return SERVO_STATUS_BUFFER_TOO_SMALL;
    }

    params[0] = addr;
    memcpy(&params[1], data, len);

    return ft_scs_write_packet(id, instruction, params, (uint8_t)(len + 1u), need_ack);
}

static void cache_feedback(const BusServoFeedback* feedback) {
    if(feedback == 0 || feedback->id > FT_SCS_MAX_SERVO_ID) {
        return;
    }

    s_ctx.feedback[feedback->id] = *feedback;
    s_ctx.has_feedback[feedback->id] = true;
}

static BusServoStatus parse_feedback(uint8_t id, const uint8_t* data, uint8_t len, BusServoFeedback* feedback) {
    if(data == 0 || feedback == 0 || len < FT_SCS_FEEDBACK_LEN) {
        return SERVO_STATUS_INVALID_PARAM;
    }

    feedback->id = id;
    feedback->error_code = s_ctx.last_error_code;
    feedback->position = raw_to_position_rad(make_u16(data[0], data[1]));
    feedback->speed = raw_to_speed_rad_s(ft_scs_servo_raw_to_signed(make_u16(data[2], data[3]), 15u));
    feedback->torque = raw_to_torque(ft_scs_servo_raw_to_signed(make_u16(data[4], data[5]), 10u));
    return SERVO_STATUS_OK;
}

/**
 * @brief 构造加速度, 位置, 时间和速度控制数据
 */
static void build_position_data(uint8_t* data, uint16_t position_raw, uint16_t speed_raw, uint8_t acc) {
    data[0] = acc;
    split_u16(position_raw, &data[1], &data[2]);
    split_u16(0u, &data[3], &data[4]);
    split_u16(speed_raw, &data[5], &data[6]);
}

/**
 * @brief 计算 SCS 校验和
 */
static uint8_t checksum(const uint8_t* data, uint16_t len) {
    uint16_t sum = 0u;
    uint16_t i;

    for(i = 0u; i < len; i++) {
        sum = (uint16_t)(sum + data[i]);
    }

    return (uint8_t)(~sum);
}

/**
 * @brief 判断当前帧是否像本机发送指令的回环帧
 */
static bool is_echo_instruction(uint8_t instruction, const uint8_t* params, uint8_t params_len) {
    (void)params;

    switch(instruction) {
        case FT_SCS_SERVO_INST_PING:
        case FT_SCS_SERVO_INST_ACTION:
        case FT_SCS_SERVO_INST_RESTORE:
        case FT_SCS_SERVO_INST_REBOOT:
        case FT_SCS_SERVO_INST_BACKUP:
        case FT_SCS_SERVO_INST_RESET:
        case FT_SCS_SERVO_INST_CALIBRATION:
            return params_len == 0u;
        case FT_SCS_SERVO_INST_READ:
            return params_len == 2u;
        case FT_SCS_SERVO_INST_WRITE:
        case FT_SCS_SERVO_INST_REG_WRITE:
            return params_len >= 2u;
        case FT_SCS_SERVO_INST_SYNC_READ:
        case FT_SCS_SERVO_INST_SYNC_WRITE:
            return params_len >= 2u;
        default:
            return false;
    }
}

/**
 * @brief 根据字节序将两个字节组合为 16 位寄存器值
 */
static uint16_t make_u16(uint8_t low, uint8_t high) {
    if(s_ctx.endian == SERVO_ENDIAN_BIG) {
        return ((uint16_t)low << 8) | high;
    }

    return ((uint16_t)high << 8) | low;
}

/**
 * @brief 根据字节序将 16 位寄存器值拆分为两个字节
 */
static void split_u16(uint16_t value, uint8_t* low, uint8_t* high) {
    if(s_ctx.endian == SERVO_ENDIAN_BIG) {
        *low = (uint8_t)(value >> 8);
        *high = (uint8_t)(value & 0xFFu);
    }
    else {
        *low = (uint8_t)(value & 0xFFu);
        *high = (uint8_t)(value >> 8);
    }
}

/**
 * @brief 将弧度位置转换为原始计数
 */
static uint16_t position_rad_to_raw(float position) {
    float raw;

    if(position < 0.0f) {
        position = 0.0f;
    }
    if(position > FT_SCS_TWO_PI) {
        position = FT_SCS_TWO_PI;
    }

    raw = (position * FT_SCS_POS_RAW_PER_REV) / FT_SCS_TWO_PI;
    if(raw > 4095.0f) {
        raw = 4095.0f;
    }

    return (uint16_t)(raw + 0.5f);
}

/**
 * @brief 将原始计数转换为弧度位置
 */
static float raw_to_position_rad(uint16_t raw) {
    return ((float)(raw & 0x0FFFu) * FT_SCS_TWO_PI) / FT_SCS_POS_RAW_PER_REV;
}

/**
 * @brief 将 rad/s 速度转换为原始带符号计数
 */
static int16_t speed_rad_s_to_raw(float speed) {
    float raw = (speed * FT_SCS_SPEED_RAW_PER_REV_S) / FT_SCS_TWO_PI;

    if(raw > 32767.0f) {
        raw = 32767.0f;
    }
    if(raw < -32767.0f) {
        raw = -32767.0f;
    }

    if(raw >= 0.0f) {
        return (int16_t)(raw + 0.5f);
    }

    return (int16_t)(raw - 0.5f);
}

/**
 * @brief 将原始带符号计数转换为 rad/s 速度
 */
static float raw_to_speed_rad_s(int16_t raw) {
    return ((float)raw * FT_SCS_TWO_PI) / FT_SCS_SPEED_RAW_PER_REV_S;
}

/**
 * @brief 将 N*m 扭矩限制转换为原始寄存器值
 */
static uint16_t torque_to_raw(float torque) {
    float raw;

    if(torque < 0.0f) {
        torque = -torque;
    }
    if(torque > FT_SCS_TORQUE_NM_FULL_SCALE) {
        torque = FT_SCS_TORQUE_NM_FULL_SCALE;
    }

    raw = (torque * FT_SCS_LOAD_RAW_FULL_SCALE) / FT_SCS_TORQUE_NM_FULL_SCALE;
    return (uint16_t)(raw + 0.5f);
}

/**
 * @brief 将原始带符号负载转换为 N*m 扭矩估计
 */
static float raw_to_torque(int16_t raw) {
    return ((float)raw * FT_SCS_TORQUE_NM_FULL_SCALE) / FT_SCS_LOAD_RAW_FULL_SCALE;
}

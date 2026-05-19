#include "dji_motor.h"

#include <stdbool.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define DJI_MOTOR_FEEDBACK_BASE_ID 0x200u
#define DJI_MOTOR_CTRL_FRAME_ID    0x200u
#define DJI_MOTOR_ENCODER_RANGE    8192
#define DJI_MOTOR_ENCODER_HALF     4096
#define DJI_MOTOR_RAD_PER_ROUND    6.28318530718f
#define DJI_MOTOR_RPM_TO_RAD_S     (DJI_MOTOR_RAD_PER_ROUND / 60.0f)
#define DJI_MOTOR_RAD_S_TO_RPM     (60.0f / DJI_MOTOR_RAD_PER_ROUND)

#define DJI_MOTOR_DEFAULT_KP           8.0f
#define DJI_MOTOR_DEFAULT_KI           0.1f
#define DJI_MOTOR_DEFAULT_KD           0.0f
#define DJI_MOTOR_DEFAULT_MAX_I        5000.0f
#define DJI_MOTOR_DEFAULT_MAX_CURRENT  16384.0f

typedef struct {
    float kp;
    float ki;
    float kd;
    float error;
    float last_error;
    float integral;
    float max_integral;
    float output;
    float max_output;
} DjiMotorPid;

typedef struct {
    uint16_t id;
    uint16_t angle;
    uint16_t last_angle;
    int16_t rpm;
    int16_t current;
    uint8_t temperature;
    int32_t total_tick;
    float target_speed;
    bool has_feedback;
    bool angle_initialized;
    BusMotorFeedback feedback;
    DjiMotorPid speed_pid;
} DjiMotorSlot;

static const BusMotorPortOps* s_ops = 0;
static bool s_is_initialized = false;
static DjiMotorSlot s_slots[DJI_MOTOR_MAX_ID];

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static BusMotorStatus dji_motor_init(const BusMotorConfig* config);
static const char* dji_motor_status_str(BusMotorStatus status);
static const char* dji_motor_mode_str(BusMotorMode mode);
static BusMotorStatus dji_motor_enable(uint16_t id);
static BusMotorStatus dji_motor_disable(uint16_t id);
static BusMotorStatus dji_motor_switch_mode(uint16_t id, BusMotorMode mode);
static BusMotorStatus dji_motor_set_pos(uint16_t id, float position);
static BusMotorStatus dji_motor_set_spd(uint16_t id, float speed);
static BusMotorStatus dji_motor_set_pos_vel(uint16_t id, float position, float speed);
static BusMotorStatus dji_motor_set_tor(uint16_t id, float torque);
static BusMotorStatus dji_motor_set_pd(uint16_t id, float kp, float kd);
static BusMotorStatus dji_motor_update_feedback(uint16_t id, BusMotorFeedback* feedback);
static float dji_motor_get_pos(uint16_t id);
static float dji_motor_get_spd(uint16_t id);
static float dji_motor_get_tor(uint16_t id);
static BusMotorStatus dji_motor_stop(uint16_t id);
static BusMotorStatus dji_motor_brake(uint16_t id);

static DjiMotorSlot* dji_motor_get_slot(uint16_t id);
static const DjiMotorSlot* dji_motor_get_slot_const(uint16_t id);
static void dji_motor_reset_slot(DjiMotorSlot* slot, uint16_t id);
static BusMotorStatus dji_motor_send_all_current(void);
static void dji_motor_update_pid(DjiMotorSlot* slot);
static float dji_motor_limit(float value, float limit);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

const BusMotorInterface dji_motor_instance = {
    .init = dji_motor_init,
    .status_str = dji_motor_status_str,
    .mode_str = dji_motor_mode_str,
    .enable = dji_motor_enable,
    .disable = dji_motor_disable,
    .switch_mode = dji_motor_switch_mode,
    .set_pos = dji_motor_set_pos,
    .set_spd = dji_motor_set_spd,
    .set_pos_vel = dji_motor_set_pos_vel,
    .set_tor = dji_motor_set_tor,
    .set_pd = dji_motor_set_pd,
    .update_feedback = dji_motor_update_feedback,
    .get_pos = dji_motor_get_pos,
    .get_spd = dji_motor_get_spd,
    .get_tor = dji_motor_get_tor,
    .stop = dji_motor_stop,
    .brake = dji_motor_brake,
};

/**
 * @brief 初始化 DJI 电机实例
 */
static BusMotorStatus dji_motor_init(const BusMotorConfig* config) {
    uint16_t id;

    if(config == 0 || config->ops == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }
    if(config->ops->send == 0) {
        return MOTOR_STATUS_PORT_ERROR;
    }

    s_ops = config->ops;
    for(id = 1u; id <= DJI_MOTOR_MAX_ID; ++id) {
        dji_motor_reset_slot(&s_slots[id - 1u], id);
    }
    s_is_initialized = true;

    return MOTOR_STATUS_OK;
}

/**
 * @brief 将状态码转换为常量字符串
 */
static const char* dji_motor_status_str(BusMotorStatus status) {
    switch(status) {
#define X(name, value) case MOTOR_STATUS_##name: return #name;
        MOTOR_STATUS_TABLE
#undef X
        default: return "UNKNOWN";
    }
}

/**
 * @brief 将模式值转换为常量字符串
 */
static const char* dji_motor_mode_str(BusMotorMode mode) {
    switch((DjiMotorMode)mode) {
        case DJI_MOTOR_MODE_SPEED: return "SPEED";
        default: return "UNKNOWN";
    }
}

/**
 * @brief 使能 DJI 电机
 */
static BusMotorStatus dji_motor_enable(uint16_t id) {
    if(dji_motor_get_slot(id) == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    return MOTOR_STATUS_OK;
}

/**
 * @brief 失能 DJI 电机
 */
static BusMotorStatus dji_motor_disable(uint16_t id) {
    DjiMotorSlot* slot = dji_motor_get_slot(id);

    if(slot == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    slot->target_speed = 0.0f;
    slot->speed_pid.output = 0.0f;
    return dji_motor_send_all_current();
}

/**
 * @brief 切换 DJI 电机模式
 */
static BusMotorStatus dji_motor_switch_mode(uint16_t id, BusMotorMode mode) {
    if(dji_motor_get_slot(id) == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }
    if((DjiMotorMode)mode != DJI_MOTOR_MODE_SPEED) {
        return MOTOR_STATUS_UNSUPPORTED;
    }

    return MOTOR_STATUS_OK;
}

/**
 * @brief DJI 3508 只支持速度闭环入口
 */
static BusMotorStatus dji_motor_set_pos(uint16_t id, float position) {
    (void)id;
    (void)position;
    return MOTOR_STATUS_UNSUPPORTED;
}

/**
 * @brief 设定目标速度, 单位 rad/s, 内部转换为电机端 rpm 做电流环输出
 */
static BusMotorStatus dji_motor_set_spd(uint16_t id, float speed) {
    DjiMotorSlot* slot = dji_motor_get_slot(id);

    if(slot == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    slot->target_speed = speed;
    dji_motor_update_pid(slot);
    return dji_motor_send_all_current();
}

/**
 * @brief DJI 3508 不直接支持位置和速度的复合控制入口
 */
static BusMotorStatus dji_motor_set_pos_vel(uint16_t id, float position, float speed) {
    (void)id;
    (void)position;
    (void)speed;
    return MOTOR_STATUS_UNSUPPORTED;
}

/**
 * @brief DJI 3508 不直接支持通用扭矩入口
 */
static BusMotorStatus dji_motor_set_tor(uint16_t id, float torque) {
    (void)id;
    (void)torque;
    return MOTOR_STATUS_UNSUPPORTED;
}

/**
 * @brief 设定速度 PID 的 P/D 参数
 */
static BusMotorStatus dji_motor_set_pd(uint16_t id, float kp, float kd) {
    DjiMotorSlot* slot = dji_motor_get_slot(id);

    if(slot == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    slot->speed_pid.kp = kp;
    slot->speed_pid.kd = kd;
    return MOTOR_STATUS_OK;
}

/**
 * @brief 从反馈缓存读取最近反馈
 */
static BusMotorStatus dji_motor_update_feedback(uint16_t id, BusMotorFeedback* feedback) {
    const DjiMotorSlot* slot = dji_motor_get_slot_const(id);

    if(slot == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }
    if(slot->has_feedback == false) {
        return MOTOR_STATUS_NO_FEEDBACK;
    }

    if(feedback != 0) {
        *feedback = slot->feedback;
    }

    return MOTOR_STATUS_OK;
}

/**
 * @brief 获取最近位置反馈
 */
static float dji_motor_get_pos(uint16_t id) {
    const DjiMotorSlot* slot = dji_motor_get_slot_const(id);

    if(slot == 0 || slot->has_feedback == false) {
        return 0.0f;
    }

    return slot->feedback.position;
}

/**
 * @brief 获取最近速度反馈
 */
static float dji_motor_get_spd(uint16_t id) {
    const DjiMotorSlot* slot = dji_motor_get_slot_const(id);

    if(slot == 0 || slot->has_feedback == false) {
        return 0.0f;
    }

    return slot->feedback.speed;
}

/**
 * @brief 获取最近电流反馈
 */
static float dji_motor_get_tor(uint16_t id) {
    const DjiMotorSlot* slot = dji_motor_get_slot_const(id);

    if(slot == 0 || slot->has_feedback == false) {
        return 0.0f;
    }

    return slot->feedback.torque;
}

/**
 * @brief 停止 DJI 电机
 */
static BusMotorStatus dji_motor_stop(uint16_t id) {
    return dji_motor_set_spd(id, 0.0f);
}

/**
 * @brief 制动 DJI 电机
 */
static BusMotorStatus dji_motor_brake(uint16_t id) {
    return dji_motor_set_spd(id, 0.0f);
}

/**
 * @brief 解析一帧 DJI 电机反馈并刷新本地缓存
 */
BusMotorStatus dji_motor_parse_feedback_frame(uint32_t frame_id,
    const uint8_t data[DJI_MOTOR_CMD_LEN],
    BusMotorFeedback* feedback) {
    DjiMotorSlot* slot;
    int16_t delta;
    uint16_t id;

    if(data == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }
    if(frame_id <= DJI_MOTOR_FEEDBACK_BASE_ID || frame_id > DJI_MOTOR_FEEDBACK_BASE_ID + DJI_MOTOR_MAX_ID) {
        return MOTOR_STATUS_ID_MISMATCH;
    }

    id = (uint16_t)(frame_id - DJI_MOTOR_FEEDBACK_BASE_ID);
    slot = dji_motor_get_slot(id);
    if(slot == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    slot->angle = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    slot->rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    slot->current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    slot->temperature = data[6];

    if(slot->angle_initialized == false) {
        slot->last_angle = slot->angle;
        slot->angle_initialized = true;
    }

    delta = (int16_t)(slot->angle - slot->last_angle);
    if(delta > DJI_MOTOR_ENCODER_HALF) {
        delta -= DJI_MOTOR_ENCODER_RANGE;
    }
    else if(delta < -DJI_MOTOR_ENCODER_HALF) {
        delta += DJI_MOTOR_ENCODER_RANGE;
    }

    slot->total_tick += delta;
    slot->last_angle = slot->angle;
    slot->feedback.id = id;
    slot->feedback.error_code = slot->temperature;
    slot->feedback.position =
        ((float)slot->total_tick / (float)DJI_MOTOR_ENCODER_RANGE) * DJI_MOTOR_RAD_PER_ROUND
        / DJI_MOTOR_M3508_REDUCTION_RATIO;
    slot->feedback.speed =
        ((float)slot->rpm * DJI_MOTOR_RPM_TO_RAD_S) / DJI_MOTOR_M3508_REDUCTION_RATIO;
    slot->feedback.torque = (float)slot->current;
    slot->has_feedback = true;

    if(feedback != 0) {
        *feedback = slot->feedback;
    }

    return MOTOR_STATUS_OK;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static DjiMotorSlot* dji_motor_get_slot(uint16_t id) {
    if(id == 0u || id > DJI_MOTOR_MAX_ID) {
        return 0;
    }

    return &s_slots[id - 1u];
}

static const DjiMotorSlot* dji_motor_get_slot_const(uint16_t id) {
    if(id == 0u || id > DJI_MOTOR_MAX_ID) {
        return 0;
    }

    return &s_slots[id - 1u];
}

static void dji_motor_reset_slot(DjiMotorSlot* slot, uint16_t id) {
    if(slot == 0) {
        return;
    }

    memset(slot, 0, sizeof(*slot));
    slot->id = id;
    slot->feedback.id = id;
    slot->speed_pid.kp = DJI_MOTOR_DEFAULT_KP;
    slot->speed_pid.ki = DJI_MOTOR_DEFAULT_KI;
    slot->speed_pid.kd = DJI_MOTOR_DEFAULT_KD;
    slot->speed_pid.max_integral = DJI_MOTOR_DEFAULT_MAX_I;
    slot->speed_pid.max_output = DJI_MOTOR_DEFAULT_MAX_CURRENT;
}

static BusMotorStatus dji_motor_send_all_current(void) {
    uint8_t data[DJI_MOTOR_CMD_LEN];
    uint16_t i;

    if(s_is_initialized == false) {
        return MOTOR_STATUS_NOT_INITIALIZE;
    }
    if(s_ops == 0 || s_ops->send == 0) {
        return MOTOR_STATUS_PORT_ERROR;
    }

    for(i = 0u; i < DJI_MOTOR_MAX_ID; ++i) {
        int16_t current = (int16_t)s_slots[i].speed_pid.output;
        data[i * 2u] = (uint8_t)(((uint16_t)current >> 8) & 0xFFu);
        data[i * 2u + 1u] = (uint8_t)((uint16_t)current & 0xFFu);
    }

    if(s_ops->send(DJI_MOTOR_CTRL_FRAME_ID, data, DJI_MOTOR_CMD_LEN) == false) {
        return MOTOR_STATUS_PORT_ERROR;
    }

    return MOTOR_STATUS_OK;
}

static void dji_motor_update_pid(DjiMotorSlot* slot) {
    DjiMotorPid* pid;
    float target_rpm;

    if(slot == 0) {
        return;
    }

    pid = &slot->speed_pid;
    target_rpm = slot->target_speed * DJI_MOTOR_M3508_REDUCTION_RATIO * DJI_MOTOR_RAD_S_TO_RPM;
    pid->error = target_rpm - (float)slot->rpm;
    pid->integral = dji_motor_limit(pid->integral + pid->error, pid->max_integral);
    pid->output = pid->kp * pid->error
        + pid->ki * pid->integral
        + pid->kd * (pid->error - pid->last_error);
    pid->output = dji_motor_limit(pid->output, pid->max_output);
    pid->last_error = pid->error;
}

static float dji_motor_limit(float value, float limit) {
    if(value > limit) {
        return limit;
    }
    if(value < -limit) {
        return -limit;
    }

    return value;
}

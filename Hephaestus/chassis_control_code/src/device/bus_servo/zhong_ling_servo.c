#include "zhong_ling_servo.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// ! ========================= 私 有 常 量 ========================= ! //

#define ZHONG_LING_SERVO_COUNT 255u
#define ZHONG_LING_SERVO_DEFAULT_POS_MIN_RAD (-2.35619449019f)
#define ZHONG_LING_SERVO_DEFAULT_POS_CENTER_RAD 0.0f
#define ZHONG_LING_SERVO_DEFAULT_POS_MAX_RAD 2.35619449019f
#define ZHONG_LING_SERVO_CMD_BUF_LEN 32u
#define ZHONG_LING_SERVO_MULTI_CMD_BUF_LEN 768u

// ! ========================= 私 有 类 型 ========================= ! //

typedef struct {
    float position;
    float speed;
    float torque;
    bool valid_position;
    bool valid_speed;
    bool valid_torque;
} ZhongLingServoCache;

typedef struct {
    const BusServoPortOps* ops;
    uint32_t timeout_ms;
    uint8_t retry_count;
    bool initialized;
    ZhongLingServoConfig config;
    ZhongLingServoCache cache[ZHONG_LING_SERVO_COUNT];
} ZhongLingServoContext;

// ! ========================= 私 有 声 明 ========================= ! //

static ZhongLingServoContext s_ctx;

static BusServoStatus zl_common_init(const void* config);
static const char* zl_common_status_str(BusServoStatus status);
static BusServoStatus zl_common_set_speed(uint8_t id, float speed);
static BusServoStatus zl_common_set_pos_spd(uint8_t id, float position, float velocity);
static BusServoStatus zl_common_set_pos_spd_tor(uint8_t id, float position, float velocity, float torque);
static float zl_common_get_position(uint8_t id);
static float zl_common_get_speed(uint8_t id);
static float zl_common_get_torque(uint8_t id);
static BusServoStatus zl_common_update_feedback(uint8_t id, BusServoFeedback* feedback);

static BusServoStatus zl_reset(void);
static BusServoStatus zl_set_user_baudrate(uint32_t baudrate);
static BusServoStatus zl_stop_all(void);
static BusServoStatus zl_stop_one(uint8_t id);
static BusServoStatus zl_set_deviation(uint8_t id, int16_t deviation);
static BusServoStatus zl_set_pwm_time(uint8_t id, uint16_t pwm, uint16_t time_ms);
static BusServoStatus zl_set_multi_pwm_time(const ZhongLingServoPwmCmd* cmds, size_t count);
static BusServoStatus zl_set_multi_pos_spd(const ZhongLingServoPosSpdCmd* cmds, size_t count);

static BusServoStatus validate_initialized(void);
static bool is_valid_id(uint8_t id);
static float clampf_range(float value, float min_value, float max_value);
static uint16_t clamp_u16_range(uint16_t value, uint16_t min_value, uint16_t max_value);
static uint32_t clamp_u32_range(uint32_t value, uint32_t min_value, uint32_t max_value);
static bool config_is_valid(const ZhongLingServoConfig* config);
static uint16_t position_to_pwm(float position);
static uint16_t velocity_to_time_ms(uint8_t id, float target_position, float velocity);
static BusServoStatus write_ascii(const char* cmd, size_t len);
static void update_cache_after_move(uint8_t id, float position, float velocity, float time_s);

// ! ========================= 接 口 实 例 ========================= ! //

const BusServoInterface zhong_ling_servo_common_instance = {
    .init = zl_common_init,
    .status_str = zl_common_status_str,
    .set_speed = zl_common_set_speed,
    .set_pos_spd = zl_common_set_pos_spd,
    .set_pos_spd_tor = zl_common_set_pos_spd_tor,
    .get_position = zl_common_get_position,
    .get_speed = zl_common_get_speed,
    .get_torque = zl_common_get_torque,
    .update_feedback = zl_common_update_feedback,
};

static const ZhongLingServoInterface s_zhong_ling_servo_feature_instance = {
    .reset = zl_reset,
    .set_user_baudrate = zl_set_user_baudrate,
    .stop_all = zl_stop_all,
    .stop_one = zl_stop_one,
    .set_deviation = zl_set_deviation,
    .set_pwm_time = zl_set_pwm_time,
    .set_multi_pwm_time = zl_set_multi_pwm_time,
    .set_multi_pos_spd = zl_set_multi_pos_spd,
};

const ZhongLingServoInterface* zhong_ling_servo_instance = &s_zhong_ling_servo_feature_instance;

// ! ========================= 接 口 函 数 实 现 ========================= ! //

static BusServoStatus zl_common_init(const void* config) {
    const ZhongLingServoConfig* zl_config = (const ZhongLingServoConfig*)config;

    if(config_is_valid(zl_config) == false) {
        return SERVO_STATUS_INVALID_PARAM;
    }

    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.ops = zl_config->ops;
    s_ctx.timeout_ms = zl_config->timeout_ms;
    s_ctx.retry_count = zl_config->retry_count;
    s_ctx.config = *zl_config;
    s_ctx.initialized = true;

    if(s_ctx.config.timeout_ms == 0u) {
        s_ctx.config.timeout_ms = 100u;
    }
    if(s_ctx.config.pos_min_rad >= s_ctx.config.pos_center_rad ||
        s_ctx.config.pos_center_rad >= s_ctx.config.pos_max_rad) {
        s_ctx.config.pos_min_rad = ZHONG_LING_SERVO_DEFAULT_POS_MIN_RAD;
        s_ctx.config.pos_center_rad = ZHONG_LING_SERVO_DEFAULT_POS_CENTER_RAD;
        s_ctx.config.pos_max_rad = ZHONG_LING_SERVO_DEFAULT_POS_MAX_RAD;
    }
    if(s_ctx.config.pwm_min == 0u) {
        s_ctx.config.pwm_min = ZHONG_LING_SERVO_PWM_MIN;
    }
    if(s_ctx.config.pwm_center == 0u) {
        s_ctx.config.pwm_center = ZHONG_LING_SERVO_PWM_CENTER;
    }
    if(s_ctx.config.pwm_max == 0u) {
        s_ctx.config.pwm_max = ZHONG_LING_SERVO_PWM_MAX;
    }

    return SERVO_STATUS_OK;
}

static const char* zl_common_status_str(BusServoStatus status) {
    switch(status) {
#define X(name, value) case SERVO_STATUS_##name: return #name;
        SERVO_STATUS_TABLE
#undef X
        default: return "UNKNOWN";
    }
}

static BusServoStatus zl_common_set_speed(uint8_t id, float speed) {
    (void)id;
    (void)speed;
    return SERVO_STATUS_UNSUPPORTED;
}

static BusServoStatus zl_common_set_pos_spd(uint8_t id, float position, float velocity) {
    char cmd[ZHONG_LING_SERVO_CMD_BUF_LEN];
    float target_position;
    float delta = 0.0f;
    float time_s;
    uint16_t pwm;
    uint16_t time_ms;
    int written;
    BusServoStatus status;

    status = validate_initialized();
    if(status != SERVO_STATUS_OK) {
        return status;
    }
    if(is_valid_id(id) == false) {
        return SERVO_STATUS_INVALID_PARAM;
    }

    target_position = clampf_range(position, s_ctx.config.pos_min_rad, s_ctx.config.pos_max_rad);
    pwm = position_to_pwm(target_position);
    time_ms = velocity_to_time_ms(id, target_position, velocity);

    written = snprintf(cmd, sizeof(cmd), "#%03uP%04uT%04u!",
        (unsigned int)id,
        (unsigned int)pwm,
        (unsigned int)time_ms);
    if(written <= 0 || (size_t)written >= sizeof(cmd)) {
        return SERVO_STATUS_BUFFER_TOO_SMALL;
    }

    status = write_ascii(cmd, (size_t)written);
    if(status != SERVO_STATUS_OK) {
        return status;
    }

    if(s_ctx.cache[id].valid_position) {
        delta = fabsf(target_position - s_ctx.cache[id].position);
    }
    time_s = ((float)time_ms) / 1000.0f;
    if(time_s <= 0.0f && fabsf(velocity) > 1e-4f) {
        time_s = delta / fabsf(velocity);
    }
    update_cache_after_move(id, target_position, velocity, time_s);
    return SERVO_STATUS_OK;
}

static BusServoStatus zl_common_set_pos_spd_tor(uint8_t id, float position, float velocity, float torque) {
    BusServoStatus status;

    (void)torque;

    status = validate_initialized();
    if(status != SERVO_STATUS_OK) {
        return status;
    }
    if(s_ctx.config.allow_torque_ignore == false) {
        return SERVO_STATUS_UNSUPPORTED;
    }

    return zl_common_set_pos_spd(id, position, velocity);
}

static float zl_common_get_position(uint8_t id) {
    if(is_valid_id(id) == false || s_ctx.cache[id].valid_position == false) {
        return 0.0f;
    }

    return s_ctx.cache[id].position;
}

static float zl_common_get_speed(uint8_t id) {
    if(is_valid_id(id) == false || s_ctx.cache[id].valid_speed == false) {
        return 0.0f;
    }

    return s_ctx.cache[id].speed;
}

static float zl_common_get_torque(uint8_t id) {
    if(is_valid_id(id) == false || s_ctx.cache[id].valid_torque == false) {
        return NAN;
    }

    return s_ctx.cache[id].torque;
}

static BusServoStatus zl_common_update_feedback(uint8_t id, BusServoFeedback* feedback) {
    (void)id;
    (void)feedback;
    return SERVO_STATUS_UNSUPPORTED;
}

// ! ========================= 特 色 函 数 实 现 ========================= ! //

static BusServoStatus zl_reset(void) {
    return write_ascii("$RST!", strlen("$RST!"));
}

static BusServoStatus zl_set_user_baudrate(uint32_t baudrate) {
    char cmd[24];
    int written;

    if(baudrate == 0u) {
        return SERVO_STATUS_INVALID_PARAM;
    }

    written = snprintf(cmd, sizeof(cmd), "$UBRS:1,%lu!", (unsigned long)baudrate);
    if(written <= 0 || (size_t)written >= sizeof(cmd)) {
        return SERVO_STATUS_BUFFER_TOO_SMALL;
    }

    return write_ascii(cmd, (size_t)written);
}

static BusServoStatus zl_stop_all(void) {
    return write_ascii("$DST!", strlen("$DST!"));
}

static BusServoStatus zl_stop_one(uint8_t id) {
    char cmd[16];
    int written;

    if(is_valid_id(id) == false) {
        return SERVO_STATUS_INVALID_PARAM;
    }

    written = snprintf(cmd, sizeof(cmd), "$DST:%u!", (unsigned int)id);
    if(written <= 0 || (size_t)written >= sizeof(cmd)) {
        return SERVO_STATUS_BUFFER_TOO_SMALL;
    }

    return write_ascii(cmd, (size_t)written);
}

static BusServoStatus zl_set_deviation(uint8_t id, int16_t deviation) {
    char cmd[24];
    int written;

    if(is_valid_id(id) == false) {
        return SERVO_STATUS_INVALID_PARAM;
    }
    if(deviation < -500 || deviation > 500) {
        return SERVO_STATUS_INVALID_PARAM;
    }

    written = snprintf(cmd, sizeof(cmd), "#%03uPSCK%+04d!", (unsigned int)id, (int)deviation);
    if(written <= 0 || (size_t)written >= sizeof(cmd)) {
        return SERVO_STATUS_BUFFER_TOO_SMALL;
    }

    return write_ascii(cmd, (size_t)written);
}

static BusServoStatus zl_set_pwm_time(uint8_t id, uint16_t pwm, uint16_t time_ms) {
    char cmd[ZHONG_LING_SERVO_CMD_BUF_LEN];
    int written;

    if(is_valid_id(id) == false) {
        return SERVO_STATUS_INVALID_PARAM;
    }

    pwm = clamp_u16_range(pwm, ZHONG_LING_SERVO_PWM_MIN, ZHONG_LING_SERVO_PWM_MAX);
    time_ms = clamp_u16_range(time_ms, 0u, ZHONG_LING_SERVO_TIME_MS_MAX);

    written = snprintf(cmd, sizeof(cmd), "#%03uP%04uT%04u!",
        (unsigned int)id,
        (unsigned int)pwm,
        (unsigned int)time_ms);
    if(written <= 0 || (size_t)written >= sizeof(cmd)) {
        return SERVO_STATUS_BUFFER_TOO_SMALL;
    }

    return write_ascii(cmd, (size_t)written);
}

static BusServoStatus zl_set_multi_pwm_time(const ZhongLingServoPwmCmd* cmds, size_t count) {
    char cmd[ZHONG_LING_SERVO_MULTI_CMD_BUF_LEN];
    size_t offset = 0u;
    size_t i;

    if(cmds == 0 || count == 0u) {
        return SERVO_STATUS_INVALID_PARAM;
    }

    cmd[offset++] = '{';
    for(i = 0u; i < count; i++) {
        int written;
        uint16_t pwm;
        uint16_t time_ms;

        if(is_valid_id(cmds[i].id) == false) {
            return SERVO_STATUS_INVALID_PARAM;
        }

        pwm = clamp_u16_range(cmds[i].pwm, ZHONG_LING_SERVO_PWM_MIN, ZHONG_LING_SERVO_PWM_MAX);
        time_ms = clamp_u16_range(cmds[i].time_ms, 0u, ZHONG_LING_SERVO_TIME_MS_MAX);

        written = snprintf(&cmd[offset], sizeof(cmd) - offset, "#%03uP%04uT%04u!",
            (unsigned int)cmds[i].id,
            (unsigned int)pwm,
            (unsigned int)time_ms);
        if(written <= 0 || (size_t)written >= (sizeof(cmd) - offset)) {
            return SERVO_STATUS_BUFFER_TOO_SMALL;
        }
        offset += (size_t)written;
    }

    if(offset >= (sizeof(cmd) - 1u)) {
        return SERVO_STATUS_BUFFER_TOO_SMALL;
    }

    cmd[offset++] = '}';
    cmd[offset] = '\0';
    return write_ascii(cmd, offset);
}

static BusServoStatus zl_set_multi_pos_spd(const ZhongLingServoPosSpdCmd* cmds, size_t count) {
    char cmd[ZHONG_LING_SERVO_MULTI_CMD_BUF_LEN];
    size_t offset = 0u;
    size_t i;

    if(cmds == 0 || count == 0u) {
        return SERVO_STATUS_INVALID_PARAM;
    }

    cmd[offset++] = '{';
    for(i = 0u; i < count; i++) {
        int written;
        uint16_t pwm;
        uint16_t time_ms;
        float target_position;

        if(is_valid_id(cmds[i].id) == false) {
            return SERVO_STATUS_INVALID_PARAM;
        }

        target_position = clampf_range(cmds[i].pos_rad, s_ctx.config.pos_min_rad, s_ctx.config.pos_max_rad);
        pwm = position_to_pwm(target_position);
        time_ms = velocity_to_time_ms(cmds[i].id, target_position, cmds[i].spd_rad_s);

        written = snprintf(&cmd[offset], sizeof(cmd) - offset, "#%03uP%04uT%04u!",
            (unsigned int)cmds[i].id,
            (unsigned int)pwm,
            (unsigned int)time_ms);
        if(written <= 0 || (size_t)written >= (sizeof(cmd) - offset)) {
            return SERVO_STATUS_BUFFER_TOO_SMALL;
        }
        offset += (size_t)written;
    }

    if(offset >= (sizeof(cmd) - 1u)) {
        return SERVO_STATUS_BUFFER_TOO_SMALL;
    }

    cmd[offset++] = '}';
    cmd[offset] = '\0';
    return write_ascii(cmd, offset);
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static BusServoStatus validate_initialized(void) {
    if(s_ctx.initialized == false) {
        return SERVO_STATUS_NOT_INITIALIZE;
    }
    if(s_ctx.ops == 0 || s_ctx.ops->write == 0) {
        return SERVO_STATUS_PORT_ERROR;
    }

    return SERVO_STATUS_OK;
}

static bool is_valid_id(uint8_t id) {
    return id <= ZHONG_LING_SERVO_MAX_ID;
}

static float clampf_range(float value, float min_value, float max_value) {
    if(value < min_value) {
        return min_value;
    }
    if(value > max_value) {
        return max_value;
    }
    return value;
}

static uint16_t clamp_u16_range(uint16_t value, uint16_t min_value, uint16_t max_value) {
    if(value < min_value) {
        return min_value;
    }
    if(value > max_value) {
        return max_value;
    }
    return value;
}

static uint32_t clamp_u32_range(uint32_t value, uint32_t min_value, uint32_t max_value) {
    if(value < min_value) {
        return min_value;
    }
    if(value > max_value) {
        return max_value;
    }
    return value;
}

static bool config_is_valid(const ZhongLingServoConfig* config) {
    if(config == 0 || config->ops == 0 || config->ops->write == 0) {
        return false;
    }
    if(config->pwm_min != 0u && config->pwm_center != 0u && config->pwm_max != 0u) {
        if(!(config->pwm_min < config->pwm_center && config->pwm_center < config->pwm_max)) {
            return false;
        }
    }
    if(config->pos_min_rad != 0.0f || config->pos_center_rad != 0.0f || config->pos_max_rad != 0.0f) {
        if(!(config->pos_min_rad < config->pos_center_rad && config->pos_center_rad < config->pos_max_rad)) {
            return false;
        }
    }
    return true;
}

static uint16_t position_to_pwm(float position) {
    float pos;
    float pwm;

    pos = clampf_range(position, s_ctx.config.pos_min_rad, s_ctx.config.pos_max_rad);

    if(pos >= s_ctx.config.pos_center_rad) {
        float ratio = (pos - s_ctx.config.pos_center_rad) /
            (s_ctx.config.pos_max_rad - s_ctx.config.pos_center_rad);
        pwm = (float)s_ctx.config.pwm_center +
            ratio * (float)(s_ctx.config.pwm_max - s_ctx.config.pwm_center);
    }
    else {
        float ratio = (pos - s_ctx.config.pos_min_rad) /
            (s_ctx.config.pos_center_rad - s_ctx.config.pos_min_rad);
        pwm = (float)s_ctx.config.pwm_min +
            ratio * (float)(s_ctx.config.pwm_center - s_ctx.config.pwm_min);
    }

    if(s_ctx.config.invert) {
        pwm = (float)(s_ctx.config.pwm_min + s_ctx.config.pwm_max) - pwm;
    }

    return clamp_u16_range((uint16_t)(pwm + 0.5f), ZHONG_LING_SERVO_PWM_MIN, ZHONG_LING_SERVO_PWM_MAX);
}

static uint16_t velocity_to_time_ms(uint8_t id, float target_position, float velocity) {
    float delta;
    float time_ms_f;
    uint32_t time_ms_u32;

    if(fabsf(velocity) < 1e-4f) {
        return 0u;
    }

    if(s_ctx.cache[id].valid_position) {
        delta = fabsf(target_position - s_ctx.cache[id].position);
    }
    else {
        delta = fabsf(target_position - s_ctx.config.pos_center_rad);
    }

    time_ms_f = (delta / fabsf(velocity)) * 1000.0f;
    if(time_ms_f < 0.0f) {
        time_ms_f = 0.0f;
    }

    time_ms_u32 = clamp_u32_range((uint32_t)(time_ms_f + 0.5f), 0u, ZHONG_LING_SERVO_TIME_MS_MAX);
    return (uint16_t)time_ms_u32;
}

static BusServoStatus write_ascii(const char* cmd, size_t len) {
    BusServoStatus status = validate_initialized();

    if(status != SERVO_STATUS_OK) {
        return status;
    }
    if(cmd == 0 || len == 0u) {
        return SERVO_STATUS_INVALID_PARAM;
    }

    if(s_ctx.ops->flush_rx != 0) {
        s_ctx.ops->flush_rx();
    }
    if(s_ctx.ops->write((const uint8_t*)cmd, (uint16_t)len) == false) {
        return SERVO_STATUS_PORT_ERROR;
    }

    return SERVO_STATUS_OK;
}

static void update_cache_after_move(uint8_t id, float position, float velocity, float time_s) {
    float speed_est = 0.0f;

    if(time_s > 1e-4f && s_ctx.cache[id].valid_position) {
        speed_est = (position - s_ctx.cache[id].position) / time_s;
    }
    else if(fabsf(velocity) > 1e-4f) {
        speed_est = velocity;
    }

    s_ctx.cache[id].position = position;
    s_ctx.cache[id].speed = speed_est;
    s_ctx.cache[id].torque = NAN;
    s_ctx.cache[id].valid_position = true;
    s_ctx.cache[id].valid_speed = true;
    s_ctx.cache[id].valid_torque = false;
}

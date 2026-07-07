/**
 * @file pi_comms.c
 * @brief Pi 通信服务实现
 */

#include "pi_comms.h"

#include "binary_frame.h"
#include "delay.h"
#include "log.h"
#include "stm32_hal_uart.h"
#include "protocol_parser.h"

#include <math.h>
#include <string.h>

// ! ========================= 宏 定 义 声 明 ========================= ! //

#define PI_COMMS_RX_RING_SIZE 256u
#define PI_COMMS_FRAME_BUF_SIZE 128u
#define PI_COMMS_TX_FRAME_BUF_SIZE 128u
#define PI_COMMS_ONLINE_TIMEOUT_MS 3000u
#define PI_COMMS_WARN_LOG_PERIOD_MS 1000u
#define PI_COMMS_PENDING_RETRY_MS 100u
#define PI_COMMS_PENDING_WARN_RETRY_COUNT 10u

#define PI_COMMS_PAYLOAD_CONTROL_LEN 38u
#define PI_COMMS_PAYLOAD_ARM_ACTION_LEN 8u
#define PI_COMMS_PAYLOAD_YAW_ACTION_LEN 12u
#define PI_COMMS_PAYLOAD_MISSION_EVENT_LEN 8u
#define PI_COMMS_PAYLOAD_ESTOP_LEN 8u
#define PI_COMMS_PAYLOAD_ACK_LEN 4u
#define PI_COMMS_PAYLOAD_STATUS_LEN 16u
#define PI_COMMS_PAYLOAD_START_SENSOR_EVENT_LEN 8u

// ! ========================= 变 量 声 明 ========================= ! //

static uint8_t s_pi_comms_rx_ring_buf[PI_COMMS_RX_RING_SIZE] = { 0 };
static RingBuf s_pi_comms_rx_ring = { 0 };
static uint8_t s_pi_comms_frame_buf[PI_COMMS_FRAME_BUF_SIZE] = { 0 };
static const uint8_t s_pi_comms_frame_header[2] = { BINARY_FRAME_SOF0, BINARY_FRAME_SOF1 };
static FrameParser s_pi_comms_frame_parser = { 0 };
static uint8_t s_pi_comms_tx_frame_buf[PI_COMMS_TX_FRAME_BUF_SIZE] = { 0 };
static uint32_t s_pi_comms_last_rx_ms = 0u;
static PiCommsConfig s_pi_comms_config = { 0 };
static PiCommsChassisControl s_pi_comms_chassis_control = { 0 };
static PiCommsYawAction s_pi_comms_yaw_action = { 0 };
static PiCommsArmControl s_pi_comms_arm_control = { 0 };
static uint32_t s_pi_comms_arm_control_rx_ms = 0u;
static bool s_pi_comms_arm_control_pending = false;
static bool s_pi_comms_arm_command_seq_valid = false;
static bool s_pi_comms_arm_command_seq_consumed = false;
static uint16_t s_pi_comms_arm_command_seq = 0u;
static PiCommsArmAction s_pi_comms_arm_action = { 0 };
static bool s_pi_comms_estop_pending = false;
static PiCommsEstopEvent s_pi_comms_estop_event = { 0 };
static PiCommsMissionEvent s_pi_comms_mission_event = { 0 };
static PiCommsStartSensorEvent s_pi_comms_pending_start_sensor_event = { 0 };
static bool s_pi_comms_pending_start_sensor_valid = false;
static uint8_t s_pi_comms_pending_start_sensor_seq = 0u;
static uint8_t s_pi_comms_tx_seq = 0u;
static uint32_t s_pi_comms_pending_retry_last_ms = 0u;
static uint32_t s_pi_comms_pending_retry_count = 0u;
static PiCommsStats s_pi_comms_stats = { 0 };
static uint32_t s_pi_comms_warn_last_ms = 0u;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static uint32_t pi_comms_now_ms(void);
static bool pi_comms_interval_due(uint32_t* last_ms, uint32_t interval_ms);
static void pi_comms_warn_limited(const char* message);
static uint8_t pi_comms_next_tx_seq(void);
static bool pi_comms_write_bytes(const uint8_t* data, uint16_t len);
static int pi_comms_get_last_tx_result(void);
static uint32_t* pi_comms_msg_attempt_counter(uint8_t msg_id);
static uint32_t* pi_comms_msg_ok_counter(uint8_t msg_id);
static uint32_t* pi_comms_msg_fail_counter(uint8_t msg_id);
static bool pi_comms_send_frame(uint8_t msg_id,
                                uint8_t seq,
                                uint8_t flags,
                                const uint8_t* payload,
                                uint16_t payload_len);
static void pi_comms_process_frame(const uint8_t* frame_body, uint16_t frame_len);
static void pi_comms_process_pending_event(void);
static void pi_comms_handle_heartbeat(const BinaryFrameView* frame);
static void pi_comms_handle_control(const BinaryFrameView* frame);
static void pi_comms_handle_arm_action(const BinaryFrameView* frame);
static void pi_comms_handle_yaw_action(const BinaryFrameView* frame);
static void pi_comms_handle_mission_event(const BinaryFrameView* frame);
static void pi_comms_handle_estop(const BinaryFrameView* frame);
static void pi_comms_handle_ack(const BinaryFrameView* frame);
static void pi_comms_reset_arm_control_state(void);
static void pi_comms_init_arm_control(PiCommsArmControl* control);
static bool pi_comms_arm_mode_from_wire(uint8_t arm_mode, PiCommsArmMode* mode);
static bool pi_comms_arm_control_is_finite(const PiCommsArmControl* control);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

PiCommsStatus pi_comms_init(const PiCommsConfig* config) {
    if(config == NULL || config->port_ops.write == NULL || config->port_ops.now_ms == NULL) {
        return PI_COMMS_STATUS_INVALID_PARAM;
    }

    s_pi_comms_config = *config;
    (void)ring_buf_create(&s_pi_comms_rx_ring, s_pi_comms_rx_ring_buf, PI_COMMS_RX_RING_SIZE, true);
    (void)frame_parser_create(&s_pi_comms_frame_parser,
                              &s_pi_comms_rx_ring,
                              s_pi_comms_frame_header,
                              sizeof(s_pi_comms_frame_header),
                              s_pi_comms_frame_buf,
                              PI_COMMS_FRAME_BUF_SIZE,
                              true);

    memset(&s_pi_comms_chassis_control, 0, sizeof(s_pi_comms_chassis_control));
    memset(&s_pi_comms_yaw_action, 0, sizeof(s_pi_comms_yaw_action));
    pi_comms_reset_arm_control_state();
    memset(&s_pi_comms_arm_action, 0, sizeof(s_pi_comms_arm_action));
    memset(&s_pi_comms_estop_event, 0, sizeof(s_pi_comms_estop_event));
    memset(&s_pi_comms_mission_event, 0, sizeof(s_pi_comms_mission_event));
    memset(&s_pi_comms_pending_start_sensor_event, 0, sizeof(s_pi_comms_pending_start_sensor_event));
    memset(&s_pi_comms_stats, 0, sizeof(s_pi_comms_stats));
    s_pi_comms_last_rx_ms = 0u;
    s_pi_comms_estop_pending = false;
    s_pi_comms_pending_start_sensor_valid = false;
    s_pi_comms_pending_start_sensor_seq = 0u;
    s_pi_comms_tx_seq = 0u;
    s_pi_comms_pending_retry_last_ms = 0u;
    s_pi_comms_pending_retry_count = 0u;
    s_pi_comms_warn_last_ms = 0u;
    return PI_COMMS_STATUS_OK;
}

void pi_comms_on_rx_byte(uint8_t data) {
    (void)frame_parser_write(&s_pi_comms_frame_parser, data);
}

void pi_comms_process(void) {
    for(;;) {
        uint8_t* frame_body = NULL;
        uint16_t frame_len = 0u;
        FrameParserErrorCode ret = frame_parser_process(&s_pi_comms_frame_parser);

        if(ret == FRAME_PARSER_PROCESSING) {
            break;
        }

        if(ret == FRAME_PARSER_ERR_CRC_MISMATCH) {
            s_pi_comms_stats.rx_bad_crc_count++;
            pi_comms_warn_limited("PI_COMMS frame dropped: crc mismatch");
            continue;
        }

        if(ret == FRAME_PARSER_ERR_LENGTH_EXCEED) {
            s_pi_comms_stats.rx_bad_len_count++;
            pi_comms_warn_limited("PI_COMMS frame dropped: length exceed");
            continue;
        }

        if(ret != FRAME_PARSER_SUCCESS) {
            pi_comms_warn_limited("PI_COMMS frame dropped: parser error");
            (void)frame_parser_finish(&s_pi_comms_frame_parser);
            continue;
        }

        if(frame_parser_get_frame(&s_pi_comms_frame_parser, &frame_body, &frame_len) != FRAME_PARSER_SUCCESS) {
            (void)frame_parser_finish(&s_pi_comms_frame_parser);
            continue;
        }

        pi_comms_process_frame(frame_body, frame_len);
        (void)frame_parser_finish(&s_pi_comms_frame_parser);
    }

    pi_comms_process_pending_event();
}

bool pi_comms_is_online(void) {
    if(s_pi_comms_last_rx_ms == 0u) {
        return false;
    }

    return (pi_comms_now_ms() - s_pi_comms_last_rx_ms) <= PI_COMMS_ONLINE_TIMEOUT_MS;
}

bool pi_comms_control_is_fresh(uint32_t timeout_ms) {
    return pi_comms_chassis_control_is_fresh(timeout_ms) ||
           pi_comms_arm_control_is_fresh(timeout_ms);
}

bool pi_comms_get_control(PiCommsControl* control) {
    if(control == NULL) {
        return false;
    }

    memset(control, 0, sizeof(*control));
    pi_comms_init_arm_control(&control->arm);
    control->chassis = s_pi_comms_chassis_control;
    control->arm = s_pi_comms_arm_control;
    return control->chassis.stamp_ms != 0u ||
           control->arm.stamp_ms != 0u;
}

bool pi_comms_get_chassis_control(PiCommsChassisControl* control) {
    if(control == NULL || s_pi_comms_chassis_control.stamp_ms == 0u) {
        return false;
    }

    *control = s_pi_comms_chassis_control;
    return true;
}

bool pi_comms_get_arm_control(PiCommsArmControl* control) {
    if(control == NULL || s_pi_comms_arm_control.stamp_ms == 0u) {
        return false;
    }

    *control = s_pi_comms_arm_control;
    return true;
}

bool pi_comms_chassis_control_is_fresh(uint32_t timeout_ms) {
    return s_pi_comms_chassis_control.stamp_ms != 0u &&
           (pi_comms_now_ms() - s_pi_comms_chassis_control.stamp_ms) <= timeout_ms;
}

bool pi_comms_arm_control_is_fresh(uint32_t timeout_ms) {
    return s_pi_comms_arm_control.stamp_ms != 0u &&
           (pi_comms_now_ms() - s_pi_comms_arm_control_rx_ms) <= timeout_ms;
}

bool pi_comms_take_arm_control(PiCommsArmControl* control) {
    if(control == NULL || !s_pi_comms_arm_control_pending) {
        return false;
    }

    *control = s_pi_comms_arm_control;
    s_pi_comms_arm_control_pending = false;
    s_pi_comms_arm_command_seq_consumed = true;
    log_info("PI_COMMS arm control consumed: mode=%u seq=%u",
             (unsigned int)control->mode,
             (unsigned int)control->command_seq);
    return true;
}

bool pi_comms_has_pending_arm_control(void) {
    return s_pi_comms_arm_control_pending;
}

bool pi_comms_take_yaw_action(PiCommsYawAction* action) {
    if(action == NULL || s_pi_comms_yaw_action.type == PI_COMMS_YAW_ACTION_NONE) {
        return false;
    }

    *action = s_pi_comms_yaw_action;
    memset(&s_pi_comms_yaw_action, 0, sizeof(s_pi_comms_yaw_action));
    return true;
}

bool pi_comms_take_arm_action(PiCommsArmAction* action) {
    if(action == NULL || s_pi_comms_arm_action.type == PI_COMMS_ARM_ACTION_NONE) {
        return false;
    }

    *action = s_pi_comms_arm_action;
    memset(&s_pi_comms_arm_action, 0, sizeof(s_pi_comms_arm_action));
    return true;
}

bool pi_comms_has_pending_arm_action(void) {
    return s_pi_comms_arm_action.type != PI_COMMS_ARM_ACTION_NONE;
}

bool pi_comms_take_estop(PiCommsEstopEvent* event) {
    if(!s_pi_comms_estop_pending) {
        return false;
    }

    if(event != NULL) {
        *event = s_pi_comms_estop_event;
    }

    s_pi_comms_estop_pending = false;
    memset(&s_pi_comms_estop_event, 0, sizeof(s_pi_comms_estop_event));
    return true;
}

bool pi_comms_take_mission_event(PiCommsMissionEvent* event) {
    if(event == NULL || s_pi_comms_mission_event.type == PI_COMMS_MISSION_EVENT_NONE) {
        return false;
    }

    *event = s_pi_comms_mission_event;
    memset(&s_pi_comms_mission_event, 0, sizeof(s_pi_comms_mission_event));
    return true;
}

bool pi_comms_send_status(const PiCommsStatusSnapshot* status) {
    uint8_t payload[PI_COMMS_PAYLOAD_STATUS_LEN] = { 0 };

    if(status == NULL) {
        return false;
    }

    binary_frame_write_u32_le(&payload[0], status->stamp_ms);
    payload[4] = status->app_state;
    payload[5] = status->manual_mode;
    payload[6] = status->ready_flags;
    payload[7] = status->online_flags;
    payload[8] = status->fault_source;
    payload[9] = status->fault_level;
    binary_frame_write_i16_le(&payload[10], status->fault_code);
    payload[12] = status->auto_start_latched > 0u ? 1u : 0u;
    payload[13] = 0u;
    payload[14] = 0u;
    payload[15] = 0u;
    return pi_comms_send_frame(BINARY_FRAME_MSG_MCU_STATUS, pi_comms_next_tx_seq(), 0u, payload, sizeof(payload));
}

bool pi_comms_send_imu(const PiCommsImuSnapshot* snapshot) {
    uint8_t payload[PI_COMMS_PAYLOAD_IMU_LEN] = { 0 };

    if(snapshot == NULL) {
        return false;
    }

    binary_frame_write_u32_le(&payload[0], snapshot->stamp_ms);
    binary_frame_write_u16_le(&payload[4], snapshot->status_flags);
    binary_frame_write_u16_le(&payload[6], snapshot->sequence_count);
    binary_frame_write_i32_le(&payload[8], snapshot->acc_x_mm_s2);
    binary_frame_write_i32_le(&payload[12], snapshot->acc_y_mm_s2);
    binary_frame_write_i32_le(&payload[16], snapshot->acc_z_mm_s2);
    binary_frame_write_i32_le(&payload[20], snapshot->gyro_x_urad_s);
    binary_frame_write_i32_le(&payload[24], snapshot->gyro_y_urad_s);
    binary_frame_write_i32_le(&payload[28], snapshot->gyro_z_urad_s);
    binary_frame_write_i32_le(&payload[32], snapshot->roll_urad);
    binary_frame_write_i32_le(&payload[36], snapshot->pitch_urad);
    binary_frame_write_i32_le(&payload[40], snapshot->yaw_urad);
    binary_frame_write_u32_le(&payload[44], snapshot->reserved);
    return pi_comms_send_frame(BINARY_FRAME_MSG_MCU_IMU, pi_comms_next_tx_seq(), 0u, payload, sizeof(payload));
}

bool pi_comms_send_odom(const PiCommsOdomSnapshot* snapshot) {
    uint8_t payload[PI_COMMS_PAYLOAD_ODOM_LEN] = { 0 };

    if(snapshot == NULL) {
        return false;
    }

    binary_frame_write_u32_le(&payload[0], snapshot->stamp_ms);
    binary_frame_write_u16_le(&payload[4], snapshot->status_flags);
    binary_frame_write_u16_le(&payload[6], snapshot->reset_counter);
    binary_frame_write_i32_le(&payload[8], snapshot->x_mm);
    binary_frame_write_i32_le(&payload[12], snapshot->y_mm);
    binary_frame_write_i32_le(&payload[16], snapshot->yaw_urad);
    binary_frame_write_i32_le(&payload[20], snapshot->vx_mm_s);
    binary_frame_write_i32_le(&payload[24], snapshot->vy_mm_s);
    binary_frame_write_i32_le(&payload[28], snapshot->wz_urad_s);
    return pi_comms_send_frame(BINARY_FRAME_MSG_MCU_ODOM, pi_comms_next_tx_seq(), 0u, payload, sizeof(payload));
}

bool pi_comms_send_arm_state(const PiCommsArmStateSnapshot* snapshot) {
    uint8_t payload[PI_COMMS_PAYLOAD_ARM_STATE_LEN] = { 0 };

    if(snapshot == NULL) {
        return false;
    }

    binary_frame_write_u32_le(&payload[0], snapshot->stamp_ms);
    binary_frame_write_u16_le(&payload[4], snapshot->status_flags);
    binary_frame_write_u16_le(&payload[6], snapshot->sequence_count);

    binary_frame_write_i32_le(&payload[8], snapshot->q0_urad);
    binary_frame_write_i32_le(&payload[12], snapshot->q1_urad);
    binary_frame_write_i32_le(&payload[16], snapshot->q2_urad);
    binary_frame_write_i32_le(&payload[20], snapshot->q3_urad);
    binary_frame_write_i32_le(&payload[24], snapshot->q4_urad);

    binary_frame_write_i32_le(&payload[28], snapshot->x_mm);
    binary_frame_write_i32_le(&payload[32], snapshot->y_mm);
    binary_frame_write_i32_le(&payload[36], snapshot->z_mm);
    binary_frame_write_i16_le(&payload[40], snapshot->quat_x_q15);
    binary_frame_write_i16_le(&payload[42], snapshot->quat_y_q15);
    binary_frame_write_i16_le(&payload[44], snapshot->quat_z_q15);
    binary_frame_write_i16_le(&payload[46], snapshot->quat_w_q15);

    return pi_comms_send_frame(BINARY_FRAME_MSG_MCU_ARM_STATE,
                               pi_comms_next_tx_seq(),
                               0u,
                               payload,
                               sizeof(payload));
}

bool pi_comms_publish_start_sensor_event(uint8_t sensor_id,
                                         uint8_t event_type,
                                         uint16_t value) {
    s_pi_comms_pending_start_sensor_event.sensor_id = sensor_id;
    s_pi_comms_pending_start_sensor_event.event_type = event_type;
    s_pi_comms_pending_start_sensor_event.value = value;
    s_pi_comms_pending_start_sensor_event.stamp_ms = pi_comms_now_ms();
    s_pi_comms_pending_start_sensor_valid = true;
    s_pi_comms_pending_start_sensor_seq = pi_comms_next_tx_seq();
    s_pi_comms_pending_retry_last_ms = 0u;
    s_pi_comms_pending_retry_count = 0u;
    s_pi_comms_stats.pending_event_count = 1u;
    return true;
}

bool pi_comms_send_ack(uint8_t ack_msg_id, uint8_t ack_seq, uint16_t code) {
    uint8_t payload[PI_COMMS_PAYLOAD_ACK_LEN] = { 0 };

    payload[0] = ack_msg_id;
    payload[1] = ack_seq;
    binary_frame_write_u16_le(&payload[2], code);
    return pi_comms_send_frame(BINARY_FRAME_MSG_MCU_ACK, pi_comms_next_tx_seq(), 0u, payload, sizeof(payload));
}

bool pi_comms_get_stats(PiCommsStats* stats) {
    if(stats == NULL) {
        return false;
    }

    *stats = s_pi_comms_stats;
    return true;
}

void pi_comms_clear_controls(void) {
    memset(&s_pi_comms_chassis_control, 0, sizeof(s_pi_comms_chassis_control));
    memset(&s_pi_comms_yaw_action, 0, sizeof(s_pi_comms_yaw_action));
    pi_comms_reset_arm_control_state();
    memset(&s_pi_comms_arm_action, 0, sizeof(s_pi_comms_arm_action));
    memset(&s_pi_comms_mission_event, 0, sizeof(s_pi_comms_mission_event));
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static void pi_comms_reset_arm_control_state(void) {
    memset(&s_pi_comms_arm_control, 0, sizeof(s_pi_comms_arm_control));
    pi_comms_init_arm_control(&s_pi_comms_arm_control);
    s_pi_comms_arm_control_rx_ms = 0u;
    s_pi_comms_arm_control_pending = false;
    s_pi_comms_arm_command_seq_valid = false;
    s_pi_comms_arm_command_seq_consumed = false;
    s_pi_comms_arm_command_seq = 0u;
}

static void pi_comms_init_arm_control(PiCommsArmControl* control) {
    if(control == NULL) {
        return;
    }

    memset(control, 0, sizeof(*control));
    control->mode = PI_COMMS_ARM_MODE_NONE;
    control->target.joints.dof = FIVE_DOF_ARM_DOF;
}

static bool pi_comms_arm_mode_from_wire(uint8_t arm_mode, PiCommsArmMode* mode) {
    if(mode == NULL) {
        return false;
    }

    switch(arm_mode) {
        case BINARY_FRAME_PI_ARM_MODE_JOINTS:
            *mode = PI_COMMS_ARM_MODE_JOINTS;
            return true;

        case BINARY_FRAME_PI_ARM_MODE_POSE_5D:
            *mode = PI_COMMS_ARM_MODE_POSE_5D;
            return true;

        case BINARY_FRAME_PI_ARM_MODE_POSITION:
            *mode = PI_COMMS_ARM_MODE_POSITION;
            return true;

        case BINARY_FRAME_PI_ARM_MODE_ORIENTATION_2D:
            *mode = PI_COMMS_ARM_MODE_ORIENTATION_2D;
            return true;

        default:
            *mode = PI_COMMS_ARM_MODE_NONE;
            return false;
    }
}

static bool pi_comms_arm_control_is_finite(const PiCommsArmControl* control) {
    if(control == NULL || !isfinite(control->speed_rad_s)) {
        return false;
    }

    switch(control->mode) {
        case PI_COMMS_ARM_MODE_JOINTS:
            for(uint8_t i = 0u; i < FIVE_DOF_ARM_DOF; i++) {
                if(!isfinite(control->target.joints.q[i])) {
                    return false;
                }
            }
            return true;

        case PI_COMMS_ARM_MODE_POSE_5D:
            return isfinite(control->target.pose_5d.x) &&
                   isfinite(control->target.pose_5d.y) &&
                   isfinite(control->target.pose_5d.z) &&
                   isfinite(control->target.pose_5d.pitch) &&
                   isfinite(control->target.pose_5d.yaw);

        case PI_COMMS_ARM_MODE_POSITION:
            return isfinite(control->target.position.x) &&
                   isfinite(control->target.position.y) &&
                   isfinite(control->target.position.z);

        case PI_COMMS_ARM_MODE_ORIENTATION_2D:
            return isfinite(control->target.orientation_2d.pitch) &&
                   isfinite(control->target.orientation_2d.yaw);

        case PI_COMMS_ARM_MODE_NONE:
        default:
            return false;
    }
}

static uint32_t pi_comms_now_ms(void) {
    if(s_pi_comms_config.port_ops.now_ms != NULL) {
        return s_pi_comms_config.port_ops.now_ms();
    }

    return delay_now_ms();
}

static bool pi_comms_interval_due(uint32_t* last_ms, uint32_t interval_ms) {
    const uint32_t now_ms = pi_comms_now_ms();

    if(last_ms == NULL) {
        return false;
    }

    if(*last_ms == 0u || (now_ms - *last_ms) >= interval_ms) {
        *last_ms = now_ms;
        return true;
    }

    return false;
}

static void pi_comms_warn_limited(const char* message) {
    if(message == NULL) {
        return;
    }

    if(pi_comms_interval_due(&s_pi_comms_warn_last_ms, PI_COMMS_WARN_LOG_PERIOD_MS)) {
        log_warn("%s", message);
    }
}

static uint8_t pi_comms_next_tx_seq(void) {
    return s_pi_comms_tx_seq++;
}

static bool pi_comms_write_bytes(const uint8_t* data, uint16_t len) {
    if(data == NULL || len == 0u || s_pi_comms_config.port_ops.write == NULL) {
        return false;
    }

    return s_pi_comms_config.port_ops.write((const char*)data, len);
}

static int pi_comms_get_last_tx_result(void) {
    if(s_pi_comms_config.port_ops.get_last_tx_result != NULL) {
        return s_pi_comms_config.port_ops.get_last_tx_result();
    }

    return PI_TX_HAL_ERROR;
}

static uint32_t* pi_comms_msg_attempt_counter(uint8_t msg_id) {
    switch(msg_id) {
        case BINARY_FRAME_MSG_MCU_STATUS:
            return &s_pi_comms_stats.tx_status_attempt_count;

        case BINARY_FRAME_MSG_MCU_IMU:
            return &s_pi_comms_stats.tx_imu_attempt_count;

        case BINARY_FRAME_MSG_MCU_ODOM:
            return &s_pi_comms_stats.tx_odom_attempt_count;

        case BINARY_FRAME_MSG_MCU_ARM_STATE:
            return &s_pi_comms_stats.tx_arm_state_attempt_count;

        default:
            return NULL;
    }
}

static uint32_t* pi_comms_msg_ok_counter(uint8_t msg_id) {
    switch(msg_id) {
        case BINARY_FRAME_MSG_MCU_STATUS:
            return &s_pi_comms_stats.tx_status_ok_count;

        case BINARY_FRAME_MSG_MCU_IMU:
            return &s_pi_comms_stats.tx_imu_ok_count;

        case BINARY_FRAME_MSG_MCU_ODOM:
            return &s_pi_comms_stats.tx_odom_ok_count;

        case BINARY_FRAME_MSG_MCU_ARM_STATE:
            return &s_pi_comms_stats.tx_arm_state_ok_count;

        default:
            return NULL;
    }
}

static uint32_t* pi_comms_msg_fail_counter(uint8_t msg_id) {
    switch(msg_id) {
        case BINARY_FRAME_MSG_MCU_STATUS:
            return &s_pi_comms_stats.tx_status_fail_count;

        case BINARY_FRAME_MSG_MCU_IMU:
            return &s_pi_comms_stats.tx_imu_fail_count;

        case BINARY_FRAME_MSG_MCU_ODOM:
            return &s_pi_comms_stats.tx_odom_fail_count;

        case BINARY_FRAME_MSG_MCU_ARM_STATE:
            return &s_pi_comms_stats.tx_arm_state_fail_count;

        default:
            return NULL;
    }
}

static bool pi_comms_send_frame(uint8_t msg_id,
                                uint8_t seq,
                                uint8_t flags,
                                const uint8_t* payload,
                                uint16_t payload_len) {
    uint16_t frame_len = 0u;
    uint16_t stored_crc = 0u;
    uint16_t calculated_crc = 0u;
    uint16_t after_crc = 0u;
    uint32_t* msg_attempt_count = NULL;
    uint32_t* msg_ok_count = NULL;
    uint32_t* msg_fail_count = NULL;
    int hal_result = PI_TX_HAL_ERROR;

    s_pi_comms_stats.tx_attempt_count++;
    msg_attempt_count = pi_comms_msg_attempt_counter(msg_id);
    msg_ok_count = pi_comms_msg_ok_counter(msg_id);
    msg_fail_count = pi_comms_msg_fail_counter(msg_id);
    if(msg_attempt_count != NULL) {
        (*msg_attempt_count)++;
    }

    if(!binary_frame_pack(msg_id,
                          seq,
                          flags,
                          payload,
                          payload_len,
                          s_pi_comms_tx_frame_buf,
                          sizeof(s_pi_comms_tx_frame_buf),
                          &frame_len)) {
        s_pi_comms_stats.tx_pack_fail_count++;
        s_pi_comms_stats.tx_fail_count++;
        if(msg_fail_count != NULL) {
            (*msg_fail_count)++;
        }
        return false;
    }

    stored_crc = ((uint16_t)s_pi_comms_tx_frame_buf[frame_len - 2u] << 8) |
                 (uint16_t)s_pi_comms_tx_frame_buf[frame_len - 1u];
    calculated_crc = binary_frame_crc16_ccitt(s_pi_comms_tx_frame_buf, (uint16_t)(frame_len - 2u));
    if(calculated_crc != stored_crc) {
        s_pi_comms_stats.tx_crc_precheck_fail_count++;
        s_pi_comms_stats.tx_fail_count++;
        if(msg_fail_count != NULL) {
            (*msg_fail_count)++;
        }
        return false;
    }

    if(!pi_comms_write_bytes(s_pi_comms_tx_frame_buf, frame_len)) {
        hal_result = pi_comms_get_last_tx_result();
        if(hal_result == PI_TX_HAL_BUSY) {
            s_pi_comms_stats.tx_hal_busy_count++;
        }
        else if(hal_result == PI_TX_HAL_TIMEOUT) {
            s_pi_comms_stats.tx_hal_timeout_count++;
        }
        else {
            s_pi_comms_stats.tx_hal_error_count++;
        }
        s_pi_comms_stats.tx_fail_count++;
        if(msg_fail_count != NULL) {
            (*msg_fail_count)++;
        }
        return false;
    }

    after_crc = binary_frame_crc16_ccitt(s_pi_comms_tx_frame_buf, (uint16_t)(frame_len - 2u));
    if(after_crc != stored_crc) {
        s_pi_comms_stats.tx_crc_postcheck_fail_count++;
        s_pi_comms_stats.tx_buffer_corruption_count++;
        s_pi_comms_stats.tx_fail_count++;
        if(msg_fail_count != NULL) {
            (*msg_fail_count)++;
        }
        return false;
    }

    s_pi_comms_stats.tx_frame_count++;
    s_pi_comms_stats.tx_ok_count++;
    if(msg_ok_count != NULL) {
        (*msg_ok_count)++;
    }
    return true;
}

static void pi_comms_process_frame(const uint8_t* frame_body, uint16_t frame_len) {
    BinaryFrameView frame;

    if(!binary_frame_parse_body(frame_body, frame_len, &frame)) {
        s_pi_comms_stats.rx_bad_len_count++;
        pi_comms_warn_limited("PI_COMMS frame dropped: body too short");
        return;
    }

    if(frame.version != BINARY_FRAME_PROTOCOL_VERSION) {
        s_pi_comms_stats.rx_unknown_msg_count++;
        pi_comms_warn_limited("PI_COMMS frame dropped: version mismatch");
        return;
    }

    s_pi_comms_stats.rx_last_msg_id = frame.msg_id;
    s_pi_comms_stats.rx_last_seq = frame.seq;
    s_pi_comms_stats.rx_frame_count++;

    switch(frame.msg_id) {
        case BINARY_FRAME_MSG_PI_HEARTBEAT:
            pi_comms_handle_heartbeat(&frame);
            return;

        case BINARY_FRAME_MSG_PI_CONTROL:
            pi_comms_handle_control(&frame);
            return;

        case BINARY_FRAME_MSG_PI_ARM_ACTION:
            pi_comms_handle_arm_action(&frame);
            return;

        case BINARY_FRAME_MSG_PI_YAW_ACTION:
            pi_comms_handle_yaw_action(&frame);
            return;

        case BINARY_FRAME_MSG_PI_MISSION_EVENT:
            pi_comms_handle_mission_event(&frame);
            return;

        case BINARY_FRAME_MSG_PI_ESTOP:
            pi_comms_handle_estop(&frame);
            return;

        case BINARY_FRAME_MSG_PI_ACK:
            pi_comms_handle_ack(&frame);
            return;

        default:
            s_pi_comms_stats.rx_unknown_msg_count++;
            pi_comms_warn_limited("PI_COMMS frame dropped: unknown msg");
            return;
    }
}

static void pi_comms_process_pending_event(void) {
    uint8_t payload[PI_COMMS_PAYLOAD_START_SENSOR_EVENT_LEN] = { 0 };

    if(!s_pi_comms_pending_start_sensor_valid) {
        return;
    }

    if(!pi_comms_interval_due(&s_pi_comms_pending_retry_last_ms, PI_COMMS_PENDING_RETRY_MS)) {
        return;
    }

    binary_frame_write_u32_le(&payload[0], s_pi_comms_pending_start_sensor_event.stamp_ms);
    payload[4] = s_pi_comms_pending_start_sensor_event.sensor_id;
    payload[5] = s_pi_comms_pending_start_sensor_event.event_type;
    binary_frame_write_u16_le(&payload[6], s_pi_comms_pending_start_sensor_event.value);

    if(pi_comms_send_frame(BINARY_FRAME_MSG_MCU_START_SENSOR_EVENT,
                           s_pi_comms_pending_start_sensor_seq,
                           BINARY_FRAME_FLAG_NEED_ACK,
                           payload,
                           sizeof(payload))) {
        s_pi_comms_pending_retry_count++;
        if(s_pi_comms_pending_retry_count >= PI_COMMS_PENDING_WARN_RETRY_COUNT &&
           ((s_pi_comms_pending_retry_count % PI_COMMS_PENDING_WARN_RETRY_COUNT) == 0u)) {
            s_pi_comms_stats.ack_timeout_count++;
            pi_comms_warn_limited("PI_COMMS pending sensor event not acked");
        }
    }
}

static void pi_comms_handle_heartbeat(const BinaryFrameView* frame) {
    if(frame == NULL || frame->payload_len != 0u) {
        s_pi_comms_stats.rx_bad_len_count++;
        pi_comms_warn_limited("PI_COMMS heartbeat dropped: invalid payload length");
        return;
    }

    s_pi_comms_last_rx_ms = pi_comms_now_ms();
    s_pi_comms_stats.last_rx_ms = s_pi_comms_last_rx_ms;
}

static void pi_comms_handle_control(const BinaryFrameView* frame) {
    const uint8_t* payload;
    uint8_t control_mask;
    uint32_t now_ms;
    PiCommsArmControl parsed_arm;
    bool arm_valid = false;

    if(frame == NULL || frame->payload_len != PI_COMMS_PAYLOAD_CONTROL_LEN) {
        s_pi_comms_stats.rx_bad_len_count++;
        pi_comms_warn_limited("PI_COMMS control dropped: invalid payload length");
        return;
    }

    payload = frame->payload;
    control_mask = payload[4];
    now_ms = pi_comms_now_ms();
    pi_comms_init_arm_control(&parsed_arm);

    if((control_mask & (BINARY_FRAME_PI_CONTROL_MASK_CHASSIS_VALID | BINARY_FRAME_PI_CONTROL_MASK_BRAKE_REQUEST)) != 0u) {
        s_pi_comms_chassis_control.vx = (control_mask & BINARY_FRAME_PI_CONTROL_MASK_CHASSIS_VALID) != 0u
                                            ? binary_frame_mm_to_m(binary_frame_read_i16_le(&payload[8]))
                                            : 0.0f;
        s_pi_comms_chassis_control.vy = (control_mask & BINARY_FRAME_PI_CONTROL_MASK_CHASSIS_VALID) != 0u
                                            ? binary_frame_mm_to_m(binary_frame_read_i16_le(&payload[10]))
                                            : 0.0f;
        s_pi_comms_chassis_control.wz = (control_mask & BINARY_FRAME_PI_CONTROL_MASK_CHASSIS_VALID) != 0u
                                            ? binary_frame_mrad_to_rad(binary_frame_read_i16_le(&payload[12]))
                                            : 0.0f;
        s_pi_comms_chassis_control.brake_request = (control_mask & BINARY_FRAME_PI_CONTROL_MASK_BRAKE_REQUEST) != 0u;
        s_pi_comms_chassis_control.stamp_ms = now_ms;
    }

    if((control_mask & BINARY_FRAME_PI_CONTROL_MASK_ARM_VALID) != 0u) {
        if(!pi_comms_arm_mode_from_wire(payload[5], &parsed_arm.mode)) {
            pi_comms_warn_limited("PI_COMMS control dropped: unsupported arm mode");
        }
        else {
            parsed_arm.command_seq = binary_frame_read_u16_le(&payload[6]);
            parsed_arm.stamp_ms = binary_frame_read_u32_le(&payload[0]);
            parsed_arm.speed_rad_s = binary_frame_mrad_to_rad(binary_frame_read_u16_le(&payload[34]));

            switch(parsed_arm.mode) {
                case PI_COMMS_ARM_MODE_JOINTS:
                    parsed_arm.target.joints.q[0] = binary_frame_urad_to_rad(binary_frame_read_i32_le(&payload[14]));
                    parsed_arm.target.joints.q[1] = binary_frame_urad_to_rad(binary_frame_read_i32_le(&payload[18]));
                    parsed_arm.target.joints.q[2] = binary_frame_urad_to_rad(binary_frame_read_i32_le(&payload[22]));
                    parsed_arm.target.joints.q[3] = binary_frame_urad_to_rad(binary_frame_read_i32_le(&payload[26]));
                    parsed_arm.target.joints.q[4] = binary_frame_urad_to_rad(binary_frame_read_i32_le(&payload[30]));
                    break;

                case PI_COMMS_ARM_MODE_POSE_5D:
                    parsed_arm.target.pose_5d.x = binary_frame_mm_to_m(binary_frame_read_i32_le(&payload[14]));
                    parsed_arm.target.pose_5d.y = binary_frame_mm_to_m(binary_frame_read_i32_le(&payload[18]));
                    parsed_arm.target.pose_5d.z = binary_frame_mm_to_m(binary_frame_read_i32_le(&payload[22]));
                    parsed_arm.target.pose_5d.pitch = binary_frame_urad_to_rad(binary_frame_read_i32_le(&payload[26]));
                    parsed_arm.target.pose_5d.yaw = binary_frame_urad_to_rad(binary_frame_read_i32_le(&payload[30]));
                    break;

                case PI_COMMS_ARM_MODE_POSITION:
                    parsed_arm.target.position.x = binary_frame_mm_to_m(binary_frame_read_i32_le(&payload[14]));
                    parsed_arm.target.position.y = binary_frame_mm_to_m(binary_frame_read_i32_le(&payload[18]));
                    parsed_arm.target.position.z = binary_frame_mm_to_m(binary_frame_read_i32_le(&payload[22]));
                    break;

                case PI_COMMS_ARM_MODE_ORIENTATION_2D:
                    parsed_arm.target.orientation_2d.pitch = binary_frame_urad_to_rad(binary_frame_read_i32_le(&payload[14]));
                    parsed_arm.target.orientation_2d.yaw = binary_frame_urad_to_rad(binary_frame_read_i32_le(&payload[18]));
                    break;

                case PI_COMMS_ARM_MODE_NONE:
                default:
                    break;
            }

            arm_valid = pi_comms_arm_control_is_finite(&parsed_arm);
            if(!arm_valid) {
                pi_comms_warn_limited("PI_COMMS control dropped: invalid arm target");
            }
            else {
                s_pi_comms_arm_control = parsed_arm;
                s_pi_comms_arm_control_rx_ms = now_ms;
                if(!s_pi_comms_arm_command_seq_valid || s_pi_comms_arm_command_seq != parsed_arm.command_seq) {
                    s_pi_comms_arm_command_seq_valid = true;
                    s_pi_comms_arm_command_seq = parsed_arm.command_seq;
                    s_pi_comms_arm_command_seq_consumed = false;
                    s_pi_comms_arm_control_pending = true;
                    log_info("PI_COMMS arm control received: mode=%u seq=%u",
                             (unsigned int)parsed_arm.mode,
                             (unsigned int)parsed_arm.command_seq);
                }
                else if(!s_pi_comms_arm_command_seq_consumed) {
                    s_pi_comms_arm_control_pending = true;
                }
            }
        }
    }

    s_pi_comms_last_rx_ms = now_ms;
    s_pi_comms_stats.last_rx_ms = now_ms;
}

static void pi_comms_handle_arm_action(const BinaryFrameView* frame) {
    const uint8_t* payload;
    uint8_t action_code;
    uint32_t now_ms;

    if(frame == NULL || frame->payload_len != PI_COMMS_PAYLOAD_ARM_ACTION_LEN) {
        s_pi_comms_stats.rx_bad_len_count++;
        pi_comms_warn_limited("PI_COMMS arm action dropped: invalid payload length");
        return;
    }

    payload = frame->payload;
    action_code = payload[4];
    now_ms = pi_comms_now_ms();

    memset(&s_pi_comms_arm_action, 0, sizeof(s_pi_comms_arm_action));
    s_pi_comms_arm_action.stamp_ms = now_ms;
    s_pi_comms_arm_action.sequence_id = binary_frame_read_u16_le(&payload[6]);

    if(action_code == BINARY_FRAME_PI_ARM_ACTION_ENABLE) {
        s_pi_comms_arm_action.type = PI_COMMS_ARM_ACTION_ENABLE;
    }
    else if(action_code == BINARY_FRAME_PI_ARM_ACTION_STOP) {
        s_pi_comms_arm_action.type = PI_COMMS_ARM_ACTION_STOP;
    }
    else if(action_code == BINARY_FRAME_PI_ARM_ACTION_SEQUENCE) {
        s_pi_comms_arm_action.type = PI_COMMS_ARM_ACTION_SEQUENCE_ID;
    }
    else {
        memset(&s_pi_comms_arm_action, 0, sizeof(s_pi_comms_arm_action));
        pi_comms_warn_limited("PI_COMMS arm action dropped: unsupported action");
        return;
    }

    s_pi_comms_last_rx_ms = now_ms;
    s_pi_comms_stats.last_rx_ms = now_ms;
    (void)binary_frame_read_u32_le(&payload[0]);
}

static void pi_comms_handle_yaw_action(const BinaryFrameView* frame) {
    const uint8_t* payload;
    uint8_t action_code;
    uint32_t now_ms;

    if(frame == NULL || frame->payload_len != PI_COMMS_PAYLOAD_YAW_ACTION_LEN) {
        s_pi_comms_stats.rx_bad_len_count++;
        pi_comms_warn_limited("PI_COMMS yaw action dropped: invalid payload length");
        return;
    }

    payload = frame->payload;
    action_code = payload[4];
    now_ms = pi_comms_now_ms();

    memset(&s_pi_comms_yaw_action, 0, sizeof(s_pi_comms_yaw_action));
    s_pi_comms_yaw_action.stamp_ms = now_ms;
    s_pi_comms_yaw_action.target_yaw = binary_frame_urad_to_rad(binary_frame_read_i32_le(&payload[8]));

    if(action_code == BINARY_FRAME_PI_YAW_ACTION_HOLD_ENABLE) {
        s_pi_comms_yaw_action.type = PI_COMMS_YAW_ACTION_HOLD_ENABLE;
    }
    else if(action_code == BINARY_FRAME_PI_YAW_ACTION_HOLD_DISABLE) {
        s_pi_comms_yaw_action.type = PI_COMMS_YAW_ACTION_HOLD_DISABLE;
    }
    else if(action_code == BINARY_FRAME_PI_YAW_ACTION_TARGET_SET) {
        s_pi_comms_yaw_action.type = PI_COMMS_YAW_ACTION_TARGET_SET;
    }
    else {
        memset(&s_pi_comms_yaw_action, 0, sizeof(s_pi_comms_yaw_action));
        pi_comms_warn_limited("PI_COMMS yaw action dropped: unsupported action");
        return;
    }

    s_pi_comms_last_rx_ms = now_ms;
    s_pi_comms_stats.last_rx_ms = now_ms;
    (void)binary_frame_read_u32_le(&payload[0]);
}

static void pi_comms_handle_mission_event(const BinaryFrameView* frame) {
    const uint8_t* payload;
    uint8_t event_code;
    uint32_t now_ms;

    if(frame == NULL || frame->payload_len != PI_COMMS_PAYLOAD_MISSION_EVENT_LEN) {
        s_pi_comms_stats.rx_bad_len_count++;
        pi_comms_warn_limited("PI_COMMS mission event dropped: invalid payload length");
        return;
    }

    payload = frame->payload;
    event_code = payload[4];
    now_ms = pi_comms_now_ms();

    memset(&s_pi_comms_mission_event, 0, sizeof(s_pi_comms_mission_event));
    s_pi_comms_mission_event.stamp_ms = now_ms;
    s_pi_comms_mission_event.fail_code = binary_frame_read_i16_le(&payload[6]);

    if(event_code == BINARY_FRAME_PI_MISSION_EVENT_DONE) {
        s_pi_comms_mission_event.type = PI_COMMS_MISSION_EVENT_DONE;
        s_pi_comms_mission_event.fail_code = 0;
    }
    else if(event_code == BINARY_FRAME_PI_MISSION_EVENT_FAIL) {
        s_pi_comms_mission_event.type = PI_COMMS_MISSION_EVENT_FAIL;
    }
    else {
        memset(&s_pi_comms_mission_event, 0, sizeof(s_pi_comms_mission_event));
        pi_comms_warn_limited("PI_COMMS mission event dropped: unsupported event");
        return;
    }

    s_pi_comms_last_rx_ms = now_ms;
    s_pi_comms_stats.last_rx_ms = now_ms;
    (void)binary_frame_read_u32_le(&payload[0]);
}

static void pi_comms_handle_estop(const BinaryFrameView* frame) {
    const uint8_t* payload;
    uint32_t now_ms;

    if(frame == NULL || frame->payload_len != PI_COMMS_PAYLOAD_ESTOP_LEN) {
        s_pi_comms_stats.rx_bad_len_count++;
        pi_comms_warn_limited("PI_COMMS estop dropped: invalid payload length");
        return;
    }

    payload = frame->payload;
    now_ms = pi_comms_now_ms();

    s_pi_comms_estop_pending = true;
    s_pi_comms_estop_event.stamp_ms = now_ms;
    s_pi_comms_estop_event.reason = payload[4];
    s_pi_comms_last_rx_ms = now_ms;
    s_pi_comms_stats.last_rx_ms = now_ms;
    (void)binary_frame_read_u32_le(&payload[0]);
}

static void pi_comms_handle_ack(const BinaryFrameView* frame) {
    const uint8_t* payload;
    uint8_t ack_msg_id;
    uint8_t ack_seq;
    uint32_t now_ms;

    if(frame == NULL || frame->payload_len != PI_COMMS_PAYLOAD_ACK_LEN) {
        s_pi_comms_stats.rx_bad_len_count++;
        pi_comms_warn_limited("PI_COMMS ack dropped: invalid payload length");
        return;
    }

    payload = frame->payload;
    ack_msg_id = payload[0];
    ack_seq = payload[1];
    now_ms = pi_comms_now_ms();

    if(s_pi_comms_pending_start_sensor_valid &&
       ack_msg_id == BINARY_FRAME_MSG_MCU_START_SENSOR_EVENT &&
       ack_seq == s_pi_comms_pending_start_sensor_seq) {
        s_pi_comms_pending_start_sensor_valid = false;
        s_pi_comms_stats.pending_event_count = 0u;
        s_pi_comms_pending_retry_last_ms = 0u;
        s_pi_comms_pending_retry_count = 0u;
        memset(&s_pi_comms_pending_start_sensor_event, 0, sizeof(s_pi_comms_pending_start_sensor_event));
    }
    else {
        pi_comms_warn_limited("PI_COMMS ack dropped: pending event mismatch");
    }

    s_pi_comms_last_rx_ms = now_ms;
    s_pi_comms_stats.last_rx_ms = now_ms;
    (void)binary_frame_read_u16_le(&payload[2]);
}

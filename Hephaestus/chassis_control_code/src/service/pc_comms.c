/**
 * @file pc_comms.c
 * @brief PC 通信服务实现
 */

#include "pc_comms.h"

#include "binary_frame.h"
#include "delay.h"
#include "log.h"
#include "protocol_parser.h"

#include <string.h>

// ! ========================= 宏 定 义 声 明 ========================= ! //

#define PC_COMMS_RX_RING_SIZE 256u
#define PC_COMMS_FRAME_BUF_SIZE 64u
#define PC_COMMS_ONLINE_TIMEOUT_MS 3000u
#define PC_COMMS_WARN_LOG_PERIOD_MS 1000u
#define PC_COMMS_MASTER_JOINTS_PAYLOAD_LEN 25u

// ! ========================= 变 量 声 明 ========================= ! //

static uint8_t s_pc_comms_rx_ring_buf[PC_COMMS_RX_RING_SIZE] = { 0 };
static RingBuf s_pc_comms_rx_ring = { 0 };
static uint8_t s_pc_comms_frame_buf[PC_COMMS_FRAME_BUF_SIZE] = { 0 };
static const uint8_t s_pc_comms_frame_header[2] = { BINARY_FRAME_SOF0, BINARY_FRAME_SOF1 };
static FrameParser s_pc_comms_frame_parser = { 0 };
static uint32_t s_pc_comms_last_rx_ms = 0u;
static PcCommsConfig s_pc_comms_config = { 0 };
static FiveDofArmJointArray s_pc_comms_master_joints = { 0 };
static bool s_pc_comms_master_end_set = false;
static uint32_t s_pc_comms_master_joints_stamp_ms = 0u;
static bool s_pc_comms_master_joints_valid = false;
static PcCommsStats s_pc_comms_stats = { 0 };
static uint32_t s_pc_comms_warn_last_ms = 0u;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static uint32_t pc_comms_now_ms(void);
static bool pc_comms_interval_due(uint32_t* last_ms, uint32_t interval_ms);
static void pc_comms_process_frame(const uint8_t* frame_body, uint16_t frame_len);
static void pc_comms_handle_heartbeat(const BinaryFrameView* frame);
static void pc_comms_handle_master_joints(const BinaryFrameView* frame);
static void pc_comms_warn_limited(const char* message);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

PcCommsStatus pc_comms_init(const PcCommsConfig* config) {
    if(config == NULL || config->port_ops.now_ms == NULL) {
        return PC_COMMS_STATUS_INVALID_PARAM;
    }

    s_pc_comms_config = *config;
    (void)ring_buf_create(&s_pc_comms_rx_ring, s_pc_comms_rx_ring_buf, PC_COMMS_RX_RING_SIZE, true);
    (void)frame_parser_create(&s_pc_comms_frame_parser,
                              &s_pc_comms_rx_ring,
                              s_pc_comms_frame_header,
                              sizeof(s_pc_comms_frame_header),
                              s_pc_comms_frame_buf,
                              PC_COMMS_FRAME_BUF_SIZE,
                              true);

    memset(&s_pc_comms_master_joints, 0, sizeof(s_pc_comms_master_joints));
    s_pc_comms_master_joints.dof = FIVE_DOF_ARM_DOF;
    s_pc_comms_master_end_set = false;
    s_pc_comms_master_joints_valid = false;
    s_pc_comms_master_joints_stamp_ms = 0u;
    s_pc_comms_last_rx_ms = 0u;
    s_pc_comms_warn_last_ms = 0u;
    memset(&s_pc_comms_stats, 0, sizeof(s_pc_comms_stats));
    return PC_COMMS_STATUS_OK;
}

void pc_comms_on_rx_byte(uint8_t data) {
    (void)frame_parser_write(&s_pc_comms_frame_parser, data);
}

void pc_comms_process(void) {
    for(;;) {
        uint8_t* frame_body = NULL;
        uint16_t frame_len = 0u;
        FrameParserErrorCode ret = frame_parser_process(&s_pc_comms_frame_parser);

        if(ret == FRAME_PARSER_PROCESSING) {
            break;
        }

        if(ret == FRAME_PARSER_ERR_CRC_MISMATCH) {
            s_pc_comms_stats.rx_bad_crc_count++;
            pc_comms_warn_limited("PC_COMMS frame dropped: crc mismatch");
            continue;
        }

        if(ret == FRAME_PARSER_ERR_LENGTH_EXCEED) {
            s_pc_comms_stats.rx_bad_len_count++;
            pc_comms_warn_limited("PC_COMMS frame dropped: length exceed");
            continue;
        }

        if(ret != FRAME_PARSER_SUCCESS) {
            pc_comms_warn_limited("PC_COMMS frame dropped: parser error");
            (void)frame_parser_finish(&s_pc_comms_frame_parser);
            continue;
        }

        if(frame_parser_get_frame(&s_pc_comms_frame_parser, &frame_body, &frame_len) != FRAME_PARSER_SUCCESS) {
            (void)frame_parser_finish(&s_pc_comms_frame_parser);
            continue;
        }

        pc_comms_process_frame(frame_body, frame_len);
        (void)frame_parser_finish(&s_pc_comms_frame_parser);
    }
}

bool pc_comms_is_online(void) {
    if(s_pc_comms_last_rx_ms == 0u) {
        return false;
    }

    return (pc_comms_now_ms() - s_pc_comms_last_rx_ms) <= PC_COMMS_ONLINE_TIMEOUT_MS;
}

bool pc_comms_get_master_joints(FiveDofArmJointArray* joints) {
    if(joints == NULL || !s_pc_comms_master_joints_valid) {
        return false;
    }

    *joints = s_pc_comms_master_joints;
    return true;
}

bool pc_comms_get_master_joints_snapshot(PcCommsMasterJoints* snapshot) {
    if(snapshot == NULL || !s_pc_comms_master_joints_valid) {
        return false;
    }

    snapshot->joints = s_pc_comms_master_joints;
    snapshot->end_set = s_pc_comms_master_end_set;
    snapshot->stamp_ms = s_pc_comms_master_joints_stamp_ms;
    return true;
}

bool pc_comms_get_master_end_set(bool* end_set) {
    if(end_set == NULL || !s_pc_comms_master_joints_valid) {
        return false;
    }

    *end_set = s_pc_comms_master_end_set;
    return true;
}

bool pc_comms_master_joints_is_fresh(uint32_t timeout_ms) {
    if(!s_pc_comms_master_joints_valid) {
        return false;
    }

    return (pc_comms_now_ms() - s_pc_comms_master_joints_stamp_ms) <= timeout_ms;
}

void pc_comms_clear_master_joints(void) {
    memset(&s_pc_comms_master_joints, 0, sizeof(s_pc_comms_master_joints));
    s_pc_comms_master_joints.dof = FIVE_DOF_ARM_DOF;
    s_pc_comms_master_end_set = false;
    s_pc_comms_master_joints_stamp_ms = 0u;
    s_pc_comms_master_joints_valid = false;
}

bool pc_comms_get_stats(PcCommsStats* stats) {
    if(stats == NULL) {
        return false;
    }

    *stats = s_pc_comms_stats;
    return true;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static uint32_t pc_comms_now_ms(void) {
    if(s_pc_comms_config.port_ops.now_ms != NULL) {
        return s_pc_comms_config.port_ops.now_ms();
    }

    return delay_now_ms();
}

static bool pc_comms_interval_due(uint32_t* last_ms, uint32_t interval_ms) {
    const uint32_t now_ms = pc_comms_now_ms();

    if(last_ms == NULL) {
        return false;
    }

    if(*last_ms == 0u || (now_ms - *last_ms) >= interval_ms) {
        *last_ms = now_ms;
        return true;
    }

    return false;
}

static void pc_comms_process_frame(const uint8_t* frame_body, uint16_t frame_len) {
    BinaryFrameView frame;

    if(!binary_frame_parse_body(frame_body, frame_len, &frame)) {
        s_pc_comms_stats.rx_bad_len_count++;
        pc_comms_warn_limited("PC_COMMS frame dropped: body too short");
        return;
    }

    if(frame.version != BINARY_FRAME_PROTOCOL_VERSION) {
        s_pc_comms_stats.rx_unknown_msg_count++;
        pc_comms_warn_limited("PC_COMMS frame dropped: version mismatch");
        return;
    }

    s_pc_comms_stats.rx_last_msg_id = frame.msg_id;
    s_pc_comms_stats.rx_last_seq = frame.seq;
    s_pc_comms_stats.rx_frame_count++;

    switch(frame.msg_id) {
        case BINARY_FRAME_MSG_PC_HEARTBEAT:
            pc_comms_handle_heartbeat(&frame);
            return;

        case BINARY_FRAME_MSG_PC_MASTER_JOINTS:
            pc_comms_handle_master_joints(&frame);
            return;

        default:
            s_pc_comms_stats.rx_unknown_msg_count++;
            pc_comms_warn_limited("PC_COMMS frame dropped: unknown msg");
            return;
    }
}

static void pc_comms_handle_heartbeat(const BinaryFrameView* frame) {
    if(frame == NULL || frame->payload_len != 0u) {
        s_pc_comms_stats.rx_bad_len_count++;
        pc_comms_warn_limited("PC_COMMS heartbeat dropped: invalid payload length");
        return;
    }

    s_pc_comms_last_rx_ms = pc_comms_now_ms();
    s_pc_comms_stats.last_rx_ms = s_pc_comms_last_rx_ms;
}

static void pc_comms_handle_master_joints(const BinaryFrameView* frame) {
    const uint8_t* payload;
    uint32_t now_ms;

    if(frame == NULL || frame->payload_len != PC_COMMS_MASTER_JOINTS_PAYLOAD_LEN) {
        s_pc_comms_stats.rx_bad_len_count++;
        pc_comms_warn_limited("PC_COMMS joints dropped: invalid payload length");
        return;
    }

    payload = frame->payload;
    now_ms = pc_comms_now_ms();

    (void)binary_frame_read_u32_le(&payload[0]);
    s_pc_comms_master_joints.dof = FIVE_DOF_ARM_DOF;
    s_pc_comms_master_joints.q[0] = binary_frame_urad_to_rad(binary_frame_read_i32_le(&payload[4]));
    s_pc_comms_master_joints.q[1] = binary_frame_urad_to_rad(binary_frame_read_i32_le(&payload[8]));
    s_pc_comms_master_joints.q[2] = binary_frame_urad_to_rad(binary_frame_read_i32_le(&payload[12]));
    s_pc_comms_master_joints.q[3] = binary_frame_urad_to_rad(binary_frame_read_i32_le(&payload[16]));
    s_pc_comms_master_joints.q[4] = binary_frame_urad_to_rad(binary_frame_read_i32_le(&payload[20]));
    s_pc_comms_master_end_set = payload[24] ? true : false;
    s_pc_comms_master_joints_valid = true;
    s_pc_comms_master_joints_stamp_ms = now_ms;
    s_pc_comms_last_rx_ms = now_ms;
    s_pc_comms_stats.last_rx_ms = now_ms;
}

static void pc_comms_warn_limited(const char* message) {
    if(message == NULL) {
        return;
    }

    if(pc_comms_interval_due(&s_pc_comms_warn_last_ms, PC_COMMS_WARN_LOG_PERIOD_MS)) {
        log_warn("%s", message);
    }
}

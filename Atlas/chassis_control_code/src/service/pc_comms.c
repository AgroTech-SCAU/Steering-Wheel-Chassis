/**
 * @file pc_comms.c
 * @brief PC 通信服务实现
 */

#include "pc_comms.h"

#include "delay.h"
#include "protocol_parser.h"

#include <stdio.h>
#include <string.h>

// ! ========================= 宏 定 义 声 明 ========================= ! //

#define PC_COMMS_RX_RING_SIZE 256u
#define PC_COMMS_LINE_SIZE 128u
#define PC_COMMS_ONLINE_TIMEOUT_MS 500u

// ! ========================= 变 量 声 明 ========================= ! //

static uint8_t s_pc_comms_rx_ring_buf[PC_COMMS_RX_RING_SIZE] = { 0 };
static RingBuf s_pc_comms_rx_ring = { 0 };
static char s_pc_comms_line_buf[PC_COMMS_LINE_SIZE] = { 0 };
static uint16_t s_pc_comms_line_len = 0u;
static uint32_t s_pc_comms_last_rx_ms = 0u;
static PcCommsConfig s_pc_comms_config = { 0 };
static FiveDofArmJointArray s_pc_comms_master_joints = { 0 };
static uint32_t s_pc_comms_master_joints_stamp_ms = 0u;
static bool s_pc_comms_master_joints_valid = false;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static uint32_t pc_comms_now_ms(void);
static void pc_comms_parse_line(const char* line);
static void pc_comms_parse_joints(const char* payload);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

PcCommsStatus pc_comms_init(const PcCommsConfig* config) {
    if(config == NULL || config->port_ops.now_ms == NULL) {
        return PC_COMMS_STATUS_INVALID_PARAM;
    }

    s_pc_comms_config = *config;
    (void)ring_buf_create(&s_pc_comms_rx_ring, s_pc_comms_rx_ring_buf, PC_COMMS_RX_RING_SIZE, true);

    memset(&s_pc_comms_master_joints, 0, sizeof(s_pc_comms_master_joints));
    s_pc_comms_master_joints.dof = FIVE_DOF_ARM_DOF;
    s_pc_comms_master_joints_valid = false;
    s_pc_comms_last_rx_ms = 0u;
    s_pc_comms_line_len = 0u;
    return PC_COMMS_STATUS_OK;
}

void pc_comms_on_rx_byte(uint8_t data) {
    s_pc_comms_last_rx_ms = pc_comms_now_ms();
    (void)ring_buf_write(&s_pc_comms_rx_ring, data);
}

void pc_comms_process(void) {
    uint8_t ch = 0u;

    while(ring_buf_read(&s_pc_comms_rx_ring, &ch) == RING_BUF_SUCCESS) {
        if(ch == '\r') {
            continue;
        }

        if(ch == '\n') {
            s_pc_comms_line_buf[s_pc_comms_line_len] = '\0';
            if(s_pc_comms_line_len > 0u) {
                pc_comms_parse_line(s_pc_comms_line_buf);
            }
            s_pc_comms_line_len = 0u;
            continue;
        }

        if(s_pc_comms_line_len + 1u < PC_COMMS_LINE_SIZE) {
            s_pc_comms_line_buf[s_pc_comms_line_len++] = (char)ch;
        }
        else {
            s_pc_comms_line_len = 0u;
        }
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

bool pc_comms_master_joints_is_fresh(uint32_t timeout_ms) {
    if(!s_pc_comms_master_joints_valid) {
        return false;
    }

    return (pc_comms_now_ms() - s_pc_comms_master_joints_stamp_ms) <= timeout_ms;
}

void pc_comms_clear_master_joints(void) {
    memset(&s_pc_comms_master_joints, 0, sizeof(s_pc_comms_master_joints));
    s_pc_comms_master_joints.dof = FIVE_DOF_ARM_DOF;
    s_pc_comms_master_joints_stamp_ms = 0u;
    s_pc_comms_master_joints_valid = false;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static uint32_t pc_comms_now_ms(void) {
    if(s_pc_comms_config.port_ops.now_ms != NULL) {
        return s_pc_comms_config.port_ops.now_ms();
    }

    return delay_now_ms();
}

static void pc_comms_parse_line(const char* line) {
    if(line == NULL) {
        return;
    }

    if(strncmp(line, "PING", 4u) == 0) {
        return;
    }

    if(strncmp(line, "JOINT:", 6u) == 0) {
        pc_comms_parse_joints(&line[6]);
    }
}

static void pc_comms_parse_joints(const char* payload) {
    float q0, q1, q2, q3, q4;

    if(payload == NULL) {
        return;
    }

    if(sscanf(payload, "%f,%f,%f,%f,%f", &q0, &q1, &q2, &q3, &q4) != 5) {
        return;
    }

    s_pc_comms_master_joints.dof = FIVE_DOF_ARM_DOF;
    s_pc_comms_master_joints.q[0] = q0;
    s_pc_comms_master_joints.q[1] = q1;
    s_pc_comms_master_joints.q[2] = q2;
    s_pc_comms_master_joints.q[3] = q3;
    s_pc_comms_master_joints.q[4] = q4;
    s_pc_comms_master_joints_valid = true;
    s_pc_comms_master_joints_stamp_ms = pc_comms_now_ms();
}

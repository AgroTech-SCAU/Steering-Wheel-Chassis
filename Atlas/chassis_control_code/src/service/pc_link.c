/**
 * @file pc_link.c
 * @brief PC 串口链路服务实现
 */

#include "pc_link.h"

#include "delay.h"
#include "protocol_parser.h"
#include "stm32_hal_uart.h"

#include <stdio.h>
#include <string.h>

// ! ========================= 宏 定 义 声 明 ========================= ! //

#define PC_LINK_RX_RING_SIZE 256u
#define PC_LINK_LINE_SIZE 128u
#define PC_LINK_ONLINE_TIMEOUT_MS 500u

// ! ========================= 变 量 声 明 ========================= ! //

static uint8_t s_pc_rx_byte = 0u;
static uint8_t s_pc_rx_ring_buf[PC_LINK_RX_RING_SIZE] = { 0 };
static RingBuf s_pc_rx_ring = { 0 };
static char s_pc_line_buf[PC_LINK_LINE_SIZE] = { 0 };
static uint16_t s_pc_line_len = 0u;
static uint32_t s_pc_last_rx_ms = 0u;
static PcCommand s_pc_command = { 0 };
static FiveDofArmJointArray s_pc_master_joints = { 0 };
static uint32_t s_pc_master_joints_stamp_ms = 0u;
static bool s_pc_master_joints_valid = false;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void pc_link_start_receive(void);
static void pc_link_on_rx_complete(void);
static void pc_link_on_error(void);
static void pc_link_parse_line(const char* line);
static void pc_link_parse_command(const char* payload);
static void pc_link_parse_joints(const char* payload);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void pc_link_init(void) {
    (void)ring_buf_create(&s_pc_rx_ring, s_pc_rx_ring_buf, PC_LINK_RX_RING_SIZE, true);

    memset(&s_pc_command, 0, sizeof(s_pc_command));
    memset(&s_pc_master_joints, 0, sizeof(s_pc_master_joints));
    s_pc_master_joints.dof = FIVE_DOF_ARM_DOF;
    s_pc_master_joints_valid = false;
    s_pc_last_rx_ms = 0u;
    s_pc_line_len = 0u;

    uart_register_rx_complete_callback(&huart1, pc_link_on_rx_complete);
    uart_register_error_callback(&huart1, pc_link_on_error);
    pc_link_start_receive();
}

void pc_link_process(void) {
    uint8_t ch = 0u;

    while(ring_buf_read(&s_pc_rx_ring, &ch) == RING_BUF_SUCCESS) {
        if(ch == '\r') {
            continue;
        }

        if(ch == '\n') {
            s_pc_line_buf[s_pc_line_len] = '\0';
            if(s_pc_line_len > 0u) {
                pc_link_parse_line(s_pc_line_buf);
            }
            s_pc_line_len = 0u;
            continue;
        }

        if(s_pc_line_len + 1u < PC_LINK_LINE_SIZE) {
            s_pc_line_buf[s_pc_line_len++] = (char)ch;
        }
        else {
            s_pc_line_len = 0u;
        }
    }
}

bool pc_link_is_online(void) {
    if(s_pc_last_rx_ms == 0u) {
        return false;
    }

    return (delay_now_ms() - s_pc_last_rx_ms) <= PC_LINK_ONLINE_TIMEOUT_MS;
}

bool pc_link_get_command(PcCommand* command) {
    if(command == NULL || s_pc_command.id == PC_COMMAND_NONE) {
        return false;
    }

    *command = s_pc_command;
    return true;
}

bool pc_link_get_master_joints(FiveDofArmJointArray* joints) {
    if(joints == NULL || !s_pc_master_joints_valid) {
        return false;
    }

    *joints = s_pc_master_joints;
    return true;
}

bool pc_link_master_joints_is_fresh(uint32_t timeout_ms) {
    if(!s_pc_master_joints_valid) {
        return false;
    }

    return (delay_now_ms() - s_pc_master_joints_stamp_ms) <= timeout_ms;
}

void pc_link_clear_command(void) {
    s_pc_command.id = PC_COMMAND_NONE;
    s_pc_command.stamp_ms = 0u;
}

void pc_link_clear_master_joints(void) {
    memset(&s_pc_master_joints, 0, sizeof(s_pc_master_joints));
    s_pc_master_joints.dof = FIVE_DOF_ARM_DOF;
    s_pc_master_joints_stamp_ms = 0u;
    s_pc_master_joints_valid = false;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static void pc_link_start_receive(void) {
    /**
     * TODO:
     * 当前 ASCII 协议仅用于联调
     * 后续比赛稳定版建议升级为：
     * 帧头 + 长度 + 消息类型 + 序号 + payload + CRC16
     * USART 接收建议改为 DMA ring / IDLE 中断, 发送建议改为非阻塞 DMA
     */
    (void)uart_receive_it(&huart1, &s_pc_rx_byte, 1u);
}

static void pc_link_on_rx_complete(void) {
    s_pc_last_rx_ms = delay_now_ms();
    (void)ring_buf_write(&s_pc_rx_ring, s_pc_rx_byte);
    pc_link_start_receive();
}

static void pc_link_on_error(void) {
    pc_link_start_receive();
}

static void pc_link_parse_line(const char* line) {
    if(line == NULL) {
        return;
    }

    if(strncmp(line, "PING", 4u) == 0) {
        return;
    }

    /**
     * TODO:
     * 当前保留最小 ASCII 联调协议, 不在本次重构中强制升级为二进制协议
     */
    if(strncmp(line, "CMD:", 4u) == 0) {
        pc_link_parse_command(&line[4]);
        return;
    }

    if(strncmp(line, "JOINT:", 6u) == 0) {
        pc_link_parse_joints(&line[6]);
    }
}

static void pc_link_parse_command(const char* payload) {
    if(payload == NULL) {
        return;
    }

    if(strcmp(payload, "START") == 0) {
        s_pc_command.id = PC_COMMAND_START;
    }
    else if(strcmp(payload, "STOP") == 0) {
        s_pc_command.id = PC_COMMAND_STOP;
    }
    else if(strcmp(payload, "CLEAR_FAULT") == 0) {
        s_pc_command.id = PC_COMMAND_CLEAR_FAULT;
    }
    else if(strcmp(payload, "BRAKE") == 0) {
        s_pc_command.id = PC_COMMAND_BRAKE;
    }
    else if(strcmp(payload, "ARM_ENABLE") == 0) {
        s_pc_command.id = PC_COMMAND_ARM_ENABLE;
    }
    else if(strcmp(payload, "ARM_STOP") == 0) {
        s_pc_command.id = PC_COMMAND_ARM_STOP;
    }
    else if(strcmp(payload, "ESTOP") == 0) {
        s_pc_command.id = PC_COMMAND_ESTOP;
    }
    else {
        return;
    }

    s_pc_command.stamp_ms = delay_now_ms();
}

static void pc_link_parse_joints(const char* payload) {
    float q0, q1, q2, q3, q4;

    if(payload == NULL) {
        return;
    }

    if(sscanf(payload, "%f,%f,%f,%f,%f", &q0, &q1, &q2, &q3, &q4) != 5) {
        return;
    }

    s_pc_master_joints.dof = FIVE_DOF_ARM_DOF;
    s_pc_master_joints.q[0] = q0;
    s_pc_master_joints.q[1] = q1;
    s_pc_master_joints.q[2] = q2;
    s_pc_master_joints.q[3] = q3;
    s_pc_master_joints.q[4] = q4;
    s_pc_master_joints_valid = true;
    s_pc_master_joints_stamp_ms = delay_now_ms();
}

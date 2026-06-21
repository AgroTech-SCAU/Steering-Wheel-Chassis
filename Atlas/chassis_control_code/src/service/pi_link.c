/**
 * @file pi_link.c
 * @brief 树莓派串口链路服务实现
 */

#include "pi_link.h"

#include "app_runtime.h"
#include "arm.h"
#include "chassis.h"
#include "delay.h"
#include "odom.h"
#include "pc_link.h"
#include "protocol_parser.h"
#include "remote.h"
#include "stm32_hal_uart.h"

#include <stdio.h>
#include <string.h>

// ! ========================= 宏 定 义 ========================= ! //

#define PI_LINK_RX_RING_SIZE 256u
#define PI_LINK_LINE_SIZE 160u
#define PI_LINK_ONLINE_TIMEOUT_MS 300u

// ! ========================= 变 量 声 明 ========================= ! //

static uint8_t s_pi_rx_byte = 0u;
static uint8_t s_pi_rx_ring_buf[PI_LINK_RX_RING_SIZE] = { 0 };
static RingBuf s_pi_rx_ring = { 0 };
static char s_pi_line_buf[PI_LINK_LINE_SIZE] = { 0 };
static uint16_t s_pi_line_len = 0u;
static uint32_t s_pi_last_rx_ms = 0u;
static ms_t s_pi_send_imu_timer = 0u;
static ms_t s_pi_send_status_timer = 0u;
static PiChassisCommand s_pi_chassis_cmd = { 0 };
static PiYawCommand s_pi_yaw_cmd = { 0 };
static PiYawCommand s_pi_yaw_action = { 0 };
static PiArmCommand s_pi_arm_cmd = { 0 };
static PiArmCommand s_pi_arm_action = { 0 };
static bool s_pi_estop_requested = false;
static PiMissionEvent s_pi_mission_event = { 0 };

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void pi_link_start_receive(void);
static void pi_link_on_rx_complete(void);
static void pi_link_on_error(void);
static void pi_link_parse_line(const char* line);
static void pi_link_parse_chassis(const char* payload);
static void pi_link_parse_yaw(const char* payload);
static void pi_link_parse_arm(const char* payload);
static void pi_link_parse_mission(const char* payload);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void pi_link_init(void) {
    (void)ring_buf_create(&s_pi_rx_ring, s_pi_rx_ring_buf, PI_LINK_RX_RING_SIZE, true);

    memset(&s_pi_chassis_cmd, 0, sizeof(s_pi_chassis_cmd));
    memset(&s_pi_yaw_cmd, 0, sizeof(s_pi_yaw_cmd));
    memset(&s_pi_yaw_action, 0, sizeof(s_pi_yaw_action));
    memset(&s_pi_arm_cmd, 0, sizeof(s_pi_arm_cmd));
    memset(&s_pi_arm_action, 0, sizeof(s_pi_arm_action));
    memset(&s_pi_mission_event, 0, sizeof(s_pi_mission_event));
    s_pi_arm_cmd.joints.dof = FIVE_DOF_ARM_DOF;
    s_pi_arm_action.joints.dof = FIVE_DOF_ARM_DOF;
    s_pi_last_rx_ms = 0u;
    s_pi_line_len = 0u;
    s_pi_send_imu_timer = 0u;
    s_pi_send_status_timer = 0u;
    s_pi_estop_requested = false;

    uart_register_rx_complete_callback(&huart10, pi_link_on_rx_complete);
    uart_register_error_callback(&huart10, pi_link_on_error);
    pi_link_start_receive();
}

void pi_link_process(void) {
    uint8_t ch = 0u;

    while(ring_buf_read(&s_pi_rx_ring, &ch) == RING_BUF_SUCCESS) {
        if(ch == '\r') {
            continue;
        }

        if(ch == '\n') {
            s_pi_line_buf[s_pi_line_len] = '\0';
            if(s_pi_line_len > 0u) {
                pi_link_parse_line(s_pi_line_buf);
            }
            s_pi_line_len = 0u;
            continue;
        }

        if(s_pi_line_len + 1u < PI_LINK_LINE_SIZE) {
            s_pi_line_buf[s_pi_line_len++] = (char)ch;
        }
        else {
            s_pi_line_len = 0u;
        }
    }

    if(delay_nb_ms(&s_pi_send_imu_timer, 50u)) {
        pi_link_send_imu_odom();
    }

    if(delay_nb_ms(&s_pi_send_status_timer, 100u)) {
        pi_link_send_mcu_status();
    }
}

bool pi_link_is_online(void) {
    if(s_pi_last_rx_ms == 0u) {
        return false;
    }

    return (delay_now_ms() - s_pi_last_rx_ms) <= PI_LINK_ONLINE_TIMEOUT_MS;
}

bool pi_link_get_chassis_cmd(PiChassisCommand* cmd) {
    if(cmd == NULL || s_pi_chassis_cmd.stamp_ms == 0u) {
        return false;
    }

    *cmd = s_pi_chassis_cmd;
    return true;
}

bool pi_link_get_yaw_cmd(PiYawCommand* cmd) {
    if(cmd == NULL || s_pi_yaw_cmd.mode != PI_YAW_MODE_RATE_SET || s_pi_yaw_cmd.stamp_ms == 0u) {
        return false;
    }

    *cmd = s_pi_yaw_cmd;
    return true;
}

bool pi_link_take_yaw_cmd(PiYawCommand* cmd) {
    if(cmd == NULL || s_pi_yaw_action.mode == PI_YAW_MODE_NONE) {
        return false;
    }

    *cmd = s_pi_yaw_action;
    memset(&s_pi_yaw_action, 0, sizeof(s_pi_yaw_action));
    return true;
}

bool pi_link_get_arm_cmd(PiArmCommand* cmd) {
    if(cmd == NULL || s_pi_arm_cmd.type != PI_ARM_COMMAND_JOINT_TARGET || s_pi_arm_cmd.stamp_ms == 0u) {
        return false;
    }

    *cmd = s_pi_arm_cmd;
    return true;
}

bool pi_link_take_arm_cmd(PiArmCommand* cmd) {
    if(cmd == NULL || s_pi_arm_action.type == PI_ARM_COMMAND_NONE) {
        return false;
    }

    *cmd = s_pi_arm_action;
    memset(&s_pi_arm_action, 0, sizeof(s_pi_arm_action));
    s_pi_arm_action.joints.dof = FIVE_DOF_ARM_DOF;
    return true;
}

bool pi_link_has_pending_arm_action(void) {
    return s_pi_arm_action.type != PI_ARM_COMMAND_NONE;
}

bool pi_link_chassis_cmd_is_fresh(uint32_t timeout_ms) {
    return s_pi_chassis_cmd.stamp_ms != 0u &&
           (delay_now_ms() - s_pi_chassis_cmd.stamp_ms) <= timeout_ms;
}

bool pi_link_yaw_cmd_is_fresh(uint32_t timeout_ms) {
    return s_pi_yaw_cmd.stamp_ms != 0u &&
           (delay_now_ms() - s_pi_yaw_cmd.stamp_ms) <= timeout_ms;
}

bool pi_link_arm_cmd_is_fresh(uint32_t timeout_ms) {
    return s_pi_arm_cmd.stamp_ms != 0u &&
           (delay_now_ms() - s_pi_arm_cmd.stamp_ms) <= timeout_ms;
}

bool pi_link_take_estop_requested(void) {
    if(!s_pi_estop_requested) {
        return false;
    }

    s_pi_estop_requested = false;
    return true;
}

bool pi_link_take_mission_event(PiMissionEvent* event) {
    if(event == NULL || s_pi_mission_event.type == PI_MISSION_EVENT_NONE) {
        return false;
    }

    *event = s_pi_mission_event;
    memset(&s_pi_mission_event, 0, sizeof(s_pi_mission_event));
    return true;
}

void pi_link_send_imu_odom(void) {
    char tx_buf[192];
    Vector3 angle = { 0 };
    Vector3 gyro = { 0 };
    Vector3 odom_vec = { 0 };
    const uint32_t stamp_ms = delay_now_ms();

    (void)odom.get_angle(&angle);
    (void)odom.get_gyro_corrected(&gyro);
    (void)odom.get_odom(&odom_vec);

    (void)snprintf(tx_buf,
                   sizeof(tx_buf),
                   "IMU_ODOM:%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
                   (unsigned long)stamp_ms,
                   angle.x,
                   angle.y,
                   angle.z,
                   gyro.z,
                   odom_vec.x,
                   odom_vec.y,
                   odom_vec.z);
    (void)uart10_write_blocking(tx_buf, (uint32_t)strlen(tx_buf));
}

void pi_link_send_mcu_status(void) {
    char tx_buf[256];
    const AppFault* fault = app_runtime_get_fault();

    (void)snprintf(tx_buf,
                   sizeof(tx_buf),
                   "MCU_STATUS:%lu,STATE=%s,MANUAL=%s,CHASSIS=%u,ARM=%u,ODOM=%u,REMOTE=%u,PC=%u,PI=%u,FAULT=%u,FAULT_SRC=%u,FAULT_LEVEL=%u,FAULT_CODE=%ld\r\n",
                   (unsigned long)delay_now_ms(),
                   app_fsm_state_str(app_runtime_get_state()),
                   app_fsm_manual_mode_str(app_runtime_get_manual_mode()),
                   chassis.is_ready() ? 1u : 0u,
                   arm.is_ready() ? 1u : 0u,
                   odom.is_ready() ? 1u : 0u,
                   remote_is_online(100u) ? 1u : 0u,
                   pc_link_is_online() ? 1u : 0u,
                   pi_link_is_online() ? 1u : 0u,
                   app_runtime_has_fault() ? 1u : 0u,
                   fault != NULL ? (unsigned int)fault->source : 0u,
                   fault != NULL ? (unsigned int)fault->level : 0u,
                   fault != NULL ? (long)fault->code : 0l);
    (void)uart10_write_blocking(tx_buf, (uint32_t)strlen(tx_buf));
}

void pi_link_clear_commands(void) {
    memset(&s_pi_chassis_cmd, 0, sizeof(s_pi_chassis_cmd));
    memset(&s_pi_yaw_cmd, 0, sizeof(s_pi_yaw_cmd));
    memset(&s_pi_yaw_action, 0, sizeof(s_pi_yaw_action));
    memset(&s_pi_arm_cmd, 0, sizeof(s_pi_arm_cmd));
    memset(&s_pi_arm_action, 0, sizeof(s_pi_arm_action));
    s_pi_arm_cmd.joints.dof = FIVE_DOF_ARM_DOF;
    s_pi_arm_action.joints.dof = FIVE_DOF_ARM_DOF;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static void pi_link_start_receive(void) {
    /**
     * TODO:
     * 当前 ASCII 协议仅用于联调
     * 后续比赛稳定版建议升级为
     * 帧头 + 长度 + 消息类型 + 序号 + payload + CRC16
     * USART 接收建议改为 DMA ring / IDLE 中断
     * USART 发送建议改为非阻塞 DMA
     */
    (void)uart_receive_it(&huart10, &s_pi_rx_byte, 1u);
}

static void pi_link_on_rx_complete(void) {
    s_pi_last_rx_ms = delay_now_ms();
    (void)ring_buf_write(&s_pi_rx_ring, s_pi_rx_byte);
    pi_link_start_receive();
}

static void pi_link_on_error(void) {
    pi_link_start_receive();
}

static void pi_link_parse_line(const char* line) {
    if(line == NULL) {
        return;
    }

    if(strncmp(line, "PING", 4u) == 0) {
        return;
    }

    if(strcmp(line, "ESTOP") == 0) {
        s_pi_estop_requested = true;
        return;
    }

    if(strncmp(line, "MISSION:", 8u) == 0) {
        pi_link_parse_mission(&line[8]);
        return;
    }

    if(strncmp(line, "CHASSIS:", 8u) == 0) {
        pi_link_parse_chassis(&line[8]);
        return;
    }

    if(strncmp(line, "YAW:", 4u) == 0) {
        pi_link_parse_yaw(&line[4]);
        return;
    }

    if(strncmp(line, "ARM:", 4u) == 0) {
        pi_link_parse_arm(&line[4]);
    }
}

static void pi_link_parse_chassis(const char* payload) {
    float vx, vy, wz;

    if(payload == NULL) {
        return;
    }

    if(sscanf(payload, "%f,%f,%f", &vx, &vy, &wz) != 3) {
        return;
    }

    s_pi_chassis_cmd.vx = vx;
    s_pi_chassis_cmd.vy = vy;
    s_pi_chassis_cmd.wz = wz;
    s_pi_chassis_cmd.stamp_ms = delay_now_ms();
}

static void pi_link_parse_yaw(const char* payload) {
    float value = 0.0f;

    if(payload == NULL) {
        return;
    }

    if(strcmp(payload, "HOLD_ENABLE") == 0) {
        memset(&s_pi_yaw_cmd, 0, sizeof(s_pi_yaw_cmd));
        s_pi_yaw_action.mode = PI_YAW_MODE_HOLD_ENABLE;
        s_pi_yaw_action.stamp_ms = delay_now_ms();
    }
    else if(strcmp(payload, "HOLD_DISABLE") == 0) {
        memset(&s_pi_yaw_cmd, 0, sizeof(s_pi_yaw_cmd));
        s_pi_yaw_action.mode = PI_YAW_MODE_HOLD_DISABLE;
        s_pi_yaw_action.stamp_ms = delay_now_ms();
    }
    else if(sscanf(payload, "TARGET,%f", &value) == 1) {
        memset(&s_pi_yaw_cmd, 0, sizeof(s_pi_yaw_cmd));
        s_pi_yaw_action.mode = PI_YAW_MODE_TARGET_SET;
        s_pi_yaw_action.target_yaw = value;
        s_pi_yaw_action.stamp_ms = delay_now_ms();
    }
    else if(sscanf(payload, "RATE,%f", &value) == 1) {
        s_pi_yaw_cmd.mode = PI_YAW_MODE_RATE_SET;
        s_pi_yaw_cmd.yaw_rate = value;
        s_pi_yaw_cmd.stamp_ms = delay_now_ms();
    }
    else {
        return;
    }
}

static void pi_link_parse_arm(const char* payload) {
    float q0, q1, q2, q3, q4, speed_rad_s;
    unsigned int sequence_id = 0u;

    if(payload == NULL) {
        return;
    }

    if(strcmp(payload, "STOP") == 0) {
        memset(&s_pi_arm_cmd, 0, sizeof(s_pi_arm_cmd));
        s_pi_arm_cmd.joints.dof = FIVE_DOF_ARM_DOF;
        s_pi_arm_action.type = PI_ARM_COMMAND_STOP;
        s_pi_arm_action.stamp_ms = delay_now_ms();
    }
    else if(strcmp(payload, "ENABLE") == 0) {
        memset(&s_pi_arm_cmd, 0, sizeof(s_pi_arm_cmd));
        s_pi_arm_cmd.joints.dof = FIVE_DOF_ARM_DOF;
        s_pi_arm_action.type = PI_ARM_COMMAND_ENABLE;
        s_pi_arm_action.stamp_ms = delay_now_ms();
    }
    else if(sscanf(payload, "SEQ,%u", &sequence_id) == 1) {
        memset(&s_pi_arm_cmd, 0, sizeof(s_pi_arm_cmd));
        s_pi_arm_cmd.joints.dof = FIVE_DOF_ARM_DOF;
        s_pi_arm_action.type = PI_ARM_COMMAND_SEQUENCE_ID;
        s_pi_arm_action.sequence_id = (uint16_t)sequence_id;
        s_pi_arm_action.stamp_ms = delay_now_ms();
    }
    else if(sscanf(payload,
                   "JOINT,%f,%f,%f,%f,%f,%f",
                   &q0,
                   &q1,
                   &q2,
                   &q3,
                   &q4,
                   &speed_rad_s) == 6) {
        s_pi_arm_cmd.type = PI_ARM_COMMAND_JOINT_TARGET;
        s_pi_arm_cmd.joints.dof = FIVE_DOF_ARM_DOF;
        s_pi_arm_cmd.joints.q[0] = q0;
        s_pi_arm_cmd.joints.q[1] = q1;
        s_pi_arm_cmd.joints.q[2] = q2;
        s_pi_arm_cmd.joints.q[3] = q3;
        s_pi_arm_cmd.joints.q[4] = q4;
        s_pi_arm_cmd.speed_rad_s = speed_rad_s;
        s_pi_arm_cmd.stamp_ms = delay_now_ms();
    }
    else {
        return;
    }
}

static void pi_link_parse_mission(const char* payload) {
    int code = 0;

    if(payload == NULL) {
        return;
    }

    if(strcmp(payload, "DONE") == 0) {
        s_pi_mission_event.type = PI_MISSION_EVENT_DONE;
        s_pi_mission_event.fail_code = 0;
        s_pi_mission_event.stamp_ms = delay_now_ms();
        return;
    }

    if(sscanf(payload, "FAIL,%d", &code) == 1) {
        s_pi_mission_event.type = PI_MISSION_EVENT_FAIL;
        s_pi_mission_event.fail_code = code;
        s_pi_mission_event.stamp_ms = delay_now_ms();
    }
}

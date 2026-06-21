/**
 * @file pi_comms.c
 * @brief Pi 通信服务实现
 */

#include "pi_comms.h"

#include "delay.h"
#include "protocol_parser.h"

#include <stdio.h>
#include <string.h>

// ! ========================= 宏 定 义 声 明 ========================= ! //

#define PI_COMMS_RX_RING_SIZE 256u
#define PI_COMMS_LINE_SIZE 160u
#define PI_COMMS_ONLINE_TIMEOUT_MS 300u

// ! ========================= 变 量 声 明 ========================= ! //

static uint8_t s_pi_comms_rx_ring_buf[PI_COMMS_RX_RING_SIZE] = { 0 };
static RingBuf s_pi_comms_rx_ring = { 0 };
static char s_pi_comms_line_buf[PI_COMMS_LINE_SIZE] = { 0 };
static uint16_t s_pi_comms_line_len = 0u;
static uint32_t s_pi_comms_last_rx_ms = 0u;
static PiCommsConfig s_pi_comms_config = { 0 };
static PiCommsChassisControl s_pi_comms_chassis_control = { 0 };
static PiCommsYawRateControl s_pi_comms_yaw_rate_control = { 0 };
static PiCommsYawAction s_pi_comms_yaw_action = { 0 };
static PiCommsArmJointControl s_pi_comms_arm_joint_control = { 0 };
static PiCommsArmAction s_pi_comms_arm_action = { 0 };
static bool s_pi_comms_estop_pending = false;
static uint32_t s_pi_comms_estop_stamp_ms = 0u;
static PiCommsMissionEvent s_pi_comms_mission_event = { 0 };

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static uint32_t pi_comms_now_ms(void);
static bool pi_comms_write(const char* data, uint32_t len);
static void pi_comms_parse_line(const char* line);
static void pi_comms_parse_chassis(const char* payload);
static void pi_comms_parse_yaw(const char* payload);
static void pi_comms_parse_arm(const char* payload);
static void pi_comms_parse_mission(const char* payload);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

PiCommsStatus pi_comms_init(const PiCommsConfig* config) {
    if(config == NULL || config->port_ops.write == NULL || config->port_ops.now_ms == NULL) {
        return PI_COMMS_STATUS_INVALID_PARAM;
    }

    s_pi_comms_config = *config;
    (void)ring_buf_create(&s_pi_comms_rx_ring, s_pi_comms_rx_ring_buf, PI_COMMS_RX_RING_SIZE, true);

    memset(&s_pi_comms_chassis_control, 0, sizeof(s_pi_comms_chassis_control));
    memset(&s_pi_comms_yaw_rate_control, 0, sizeof(s_pi_comms_yaw_rate_control));
    memset(&s_pi_comms_yaw_action, 0, sizeof(s_pi_comms_yaw_action));
    memset(&s_pi_comms_arm_joint_control, 0, sizeof(s_pi_comms_arm_joint_control));
    memset(&s_pi_comms_arm_action, 0, sizeof(s_pi_comms_arm_action));
    memset(&s_pi_comms_mission_event, 0, sizeof(s_pi_comms_mission_event));
    s_pi_comms_arm_joint_control.joints.dof = FIVE_DOF_ARM_DOF;
    s_pi_comms_last_rx_ms = 0u;
    s_pi_comms_line_len = 0u;
    s_pi_comms_estop_pending = false;
    s_pi_comms_estop_stamp_ms = 0u;
    return PI_COMMS_STATUS_OK;
}

void pi_comms_on_rx_byte(uint8_t data) {
    s_pi_comms_last_rx_ms = pi_comms_now_ms();
    (void)ring_buf_write(&s_pi_comms_rx_ring, data);
}

void pi_comms_process(void) {
    uint8_t ch = 0u;

    while(ring_buf_read(&s_pi_comms_rx_ring, &ch) == RING_BUF_SUCCESS) {
        if(ch == '\r') {
            continue;
        }

        if(ch == '\n') {
            s_pi_comms_line_buf[s_pi_comms_line_len] = '\0';
            if(s_pi_comms_line_len > 0u) {
                pi_comms_parse_line(s_pi_comms_line_buf);
            }
            s_pi_comms_line_len = 0u;
            continue;
        }

        if(s_pi_comms_line_len + 1u < PI_COMMS_LINE_SIZE) {
            s_pi_comms_line_buf[s_pi_comms_line_len++] = (char)ch;
        }
        else {
            s_pi_comms_line_len = 0u;
        }
    }
}

bool pi_comms_is_online(void) {
    if(s_pi_comms_last_rx_ms == 0u) {
        return false;
    }

    return (pi_comms_now_ms() - s_pi_comms_last_rx_ms) <= PI_COMMS_ONLINE_TIMEOUT_MS;
}

bool pi_comms_control_is_fresh(uint32_t timeout_ms) {
    return pi_comms_chassis_control_is_fresh(timeout_ms) || pi_comms_yaw_rate_control_is_fresh(timeout_ms) ||
           pi_comms_arm_joint_control_is_fresh(timeout_ms);
}

bool pi_comms_get_control(PiCommsControl* control) {
    if(control == NULL) {
        return false;
    }

    memset(control, 0, sizeof(*control));
    control->arm_joint.joints.dof = FIVE_DOF_ARM_DOF;
    control->chassis = s_pi_comms_chassis_control;
    control->yaw_rate = s_pi_comms_yaw_rate_control;
    control->arm_joint = s_pi_comms_arm_joint_control;
    return control->chassis.stamp_ms != 0u || control->yaw_rate.stamp_ms != 0u || control->arm_joint.stamp_ms != 0u;
}

bool pi_comms_get_chassis_control(PiCommsChassisControl* control) {
    if(control == NULL || s_pi_comms_chassis_control.stamp_ms == 0u) {
        return false;
    }

    *control = s_pi_comms_chassis_control;
    return true;
}

bool pi_comms_get_yaw_rate_control(PiCommsYawRateControl* control) {
    if(control == NULL || s_pi_comms_yaw_rate_control.stamp_ms == 0u) {
        return false;
    }

    *control = s_pi_comms_yaw_rate_control;
    return true;
}

bool pi_comms_get_arm_joint_control(PiCommsArmJointControl* control) {
    if(control == NULL || s_pi_comms_arm_joint_control.stamp_ms == 0u) {
        return false;
    }

    *control = s_pi_comms_arm_joint_control;
    return true;
}

bool pi_comms_chassis_control_is_fresh(uint32_t timeout_ms) {
    return s_pi_comms_chassis_control.stamp_ms != 0u &&
           (pi_comms_now_ms() - s_pi_comms_chassis_control.stamp_ms) <= timeout_ms;
}

bool pi_comms_yaw_rate_control_is_fresh(uint32_t timeout_ms) {
    return s_pi_comms_yaw_rate_control.stamp_ms != 0u &&
           (pi_comms_now_ms() - s_pi_comms_yaw_rate_control.stamp_ms) <= timeout_ms;
}

bool pi_comms_arm_joint_control_is_fresh(uint32_t timeout_ms) {
    return s_pi_comms_arm_joint_control.stamp_ms != 0u &&
           (pi_comms_now_ms() - s_pi_comms_arm_joint_control.stamp_ms) <= timeout_ms;
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
        event->stamp_ms = s_pi_comms_estop_stamp_ms;
    }
    s_pi_comms_estop_pending = false;
    s_pi_comms_estop_stamp_ms = 0u;
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
    char tx_buf[256];

    if(status == NULL) {
        return false;
    }

    (void)snprintf(tx_buf,
                   sizeof(tx_buf),
                   "MCU_STATUS:%lu,STATE=%s,MANUAL=%s,CHASSIS=%u,ARM=%u,ODOM=%u,REMOTE=%u,PC=%u,PI=%u,FAULT=%u,FAULT_SRC=%u,FAULT_LEVEL=%u,FAULT_CODE=%ld\r\n",
                   (unsigned long)status->stamp_ms,
                   status->state != NULL ? status->state : "Unknown",
                   status->manual != NULL ? status->manual : "Unknown",
                   (unsigned int)status->chassis_ready,
                   (unsigned int)status->arm_ready,
                   (unsigned int)status->odom_ready,
                   (unsigned int)status->remote_online,
                   (unsigned int)status->pc_online,
                   (unsigned int)status->pi_online,
                   (unsigned int)status->fault,
                   (unsigned int)status->fault_source,
                   (unsigned int)status->fault_level,
                   (long)status->fault_code);
    return pi_comms_write(tx_buf, (uint32_t)strlen(tx_buf));
}

bool pi_comms_send_imu_odom(const PiCommsImuOdomSnapshot* odom) {
    char tx_buf[192];

    if(odom == NULL) {
        return false;
    }

    (void)snprintf(tx_buf,
                   sizeof(tx_buf),
                   "IMU_ODOM:%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
                   (unsigned long)odom->stamp_ms,
                   odom->angle_x,
                   odom->angle_y,
                   odom->angle_z,
                   odom->gyro_z,
                   odom->odom_x,
                   odom->odom_y,
                   odom->odom_z);
    return pi_comms_write(tx_buf, (uint32_t)strlen(tx_buf));
}

void pi_comms_clear_controls(void) {
    memset(&s_pi_comms_chassis_control, 0, sizeof(s_pi_comms_chassis_control));
    memset(&s_pi_comms_yaw_rate_control, 0, sizeof(s_pi_comms_yaw_rate_control));
    memset(&s_pi_comms_yaw_action, 0, sizeof(s_pi_comms_yaw_action));
    memset(&s_pi_comms_arm_joint_control, 0, sizeof(s_pi_comms_arm_joint_control));
    memset(&s_pi_comms_arm_action, 0, sizeof(s_pi_comms_arm_action));
    memset(&s_pi_comms_mission_event, 0, sizeof(s_pi_comms_mission_event));
    s_pi_comms_arm_joint_control.joints.dof = FIVE_DOF_ARM_DOF;
    s_pi_comms_estop_pending = false;
    s_pi_comms_estop_stamp_ms = 0u;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static uint32_t pi_comms_now_ms(void) {
    if(s_pi_comms_config.port_ops.now_ms != NULL) {
        return s_pi_comms_config.port_ops.now_ms();
    }

    return delay_now_ms();
}

static bool pi_comms_write(const char* data, uint32_t len) {
    if(s_pi_comms_config.port_ops.write == NULL) {
        return false;
    }

    return s_pi_comms_config.port_ops.write(data, len);
}

static void pi_comms_parse_line(const char* line) {
    if(line == NULL) {
        return;
    }

    if(strncmp(line, "PING", 4u) == 0) {
        return;
    }

    if(strcmp(line, "ESTOP") == 0) {
        s_pi_comms_estop_pending = true;
        s_pi_comms_estop_stamp_ms = pi_comms_now_ms();
        return;
    }

    if(strncmp(line, "MISSION:", 8u) == 0) {
        pi_comms_parse_mission(&line[8]);
        return;
    }

    if(strncmp(line, "CHASSIS:", 8u) == 0) {
        pi_comms_parse_chassis(&line[8]);
        return;
    }

    if(strncmp(line, "YAW:", 4u) == 0) {
        pi_comms_parse_yaw(&line[4]);
        return;
    }

    if(strncmp(line, "ARM:", 4u) == 0) {
        pi_comms_parse_arm(&line[4]);
    }
}

static void pi_comms_parse_chassis(const char* payload) {
    float vx, vy, wz;

    if(payload == NULL) {
        return;
    }

    if(sscanf(payload, "%f,%f,%f", &vx, &vy, &wz) != 3) {
        return;
    }

    s_pi_comms_chassis_control.vx = vx;
    s_pi_comms_chassis_control.vy = vy;
    s_pi_comms_chassis_control.wz = wz;
    s_pi_comms_chassis_control.stamp_ms = pi_comms_now_ms();
}

static void pi_comms_parse_yaw(const char* payload) {
    float value = 0.0f;

    if(payload == NULL) {
        return;
    }

    if(strcmp(payload, "HOLD_ENABLE") == 0) {
        memset(&s_pi_comms_yaw_rate_control, 0, sizeof(s_pi_comms_yaw_rate_control));
        s_pi_comms_yaw_action.type = PI_COMMS_YAW_ACTION_HOLD_ENABLE;
        s_pi_comms_yaw_action.stamp_ms = pi_comms_now_ms();
    }
    else if(strcmp(payload, "HOLD_DISABLE") == 0) {
        memset(&s_pi_comms_yaw_rate_control, 0, sizeof(s_pi_comms_yaw_rate_control));
        s_pi_comms_yaw_action.type = PI_COMMS_YAW_ACTION_HOLD_DISABLE;
        s_pi_comms_yaw_action.stamp_ms = pi_comms_now_ms();
    }
    else if(sscanf(payload, "TARGET,%f", &value) == 1) {
        memset(&s_pi_comms_yaw_rate_control, 0, sizeof(s_pi_comms_yaw_rate_control));
        s_pi_comms_yaw_action.type = PI_COMMS_YAW_ACTION_TARGET_SET;
        s_pi_comms_yaw_action.target_yaw = value;
        s_pi_comms_yaw_action.stamp_ms = pi_comms_now_ms();
    }
    else if(sscanf(payload, "RATE,%f", &value) == 1) {
        s_pi_comms_yaw_rate_control.yaw_rate = value;
        s_pi_comms_yaw_rate_control.stamp_ms = pi_comms_now_ms();
    }
}

static void pi_comms_parse_arm(const char* payload) {
    float q0, q1, q2, q3, q4, speed_rad_s;
    unsigned int sequence_id = 0u;

    if(payload == NULL) {
        return;
    }

    if(strcmp(payload, "STOP") == 0) {
        memset(&s_pi_comms_arm_joint_control, 0, sizeof(s_pi_comms_arm_joint_control));
        s_pi_comms_arm_joint_control.joints.dof = FIVE_DOF_ARM_DOF;
        s_pi_comms_arm_action.type = PI_COMMS_ARM_ACTION_STOP;
        s_pi_comms_arm_action.stamp_ms = pi_comms_now_ms();
    }
    else if(strcmp(payload, "ENABLE") == 0) {
        memset(&s_pi_comms_arm_joint_control, 0, sizeof(s_pi_comms_arm_joint_control));
        s_pi_comms_arm_joint_control.joints.dof = FIVE_DOF_ARM_DOF;
        s_pi_comms_arm_action.type = PI_COMMS_ARM_ACTION_ENABLE;
        s_pi_comms_arm_action.stamp_ms = pi_comms_now_ms();
    }
    else if(sscanf(payload, "SEQ,%u", &sequence_id) == 1) {
        memset(&s_pi_comms_arm_joint_control, 0, sizeof(s_pi_comms_arm_joint_control));
        s_pi_comms_arm_joint_control.joints.dof = FIVE_DOF_ARM_DOF;
        s_pi_comms_arm_action.type = PI_COMMS_ARM_ACTION_SEQUENCE_ID;
        s_pi_comms_arm_action.sequence_id = (uint16_t)sequence_id;
        s_pi_comms_arm_action.stamp_ms = pi_comms_now_ms();
    }
    else if(sscanf(payload,
                   "JOINT,%f,%f,%f,%f,%f,%f",
                   &q0,
                   &q1,
                   &q2,
                   &q3,
                   &q4,
                   &speed_rad_s) == 6) {
        s_pi_comms_arm_joint_control.joints.dof = FIVE_DOF_ARM_DOF;
        s_pi_comms_arm_joint_control.joints.q[0] = q0;
        s_pi_comms_arm_joint_control.joints.q[1] = q1;
        s_pi_comms_arm_joint_control.joints.q[2] = q2;
        s_pi_comms_arm_joint_control.joints.q[3] = q3;
        s_pi_comms_arm_joint_control.joints.q[4] = q4;
        s_pi_comms_arm_joint_control.speed_rad_s = speed_rad_s;
        s_pi_comms_arm_joint_control.stamp_ms = pi_comms_now_ms();
    }
}

static void pi_comms_parse_mission(const char* payload) {
    int code = 0;

    if(payload == NULL) {
        return;
    }

    if(strcmp(payload, "DONE") == 0) {
        s_pi_comms_mission_event.type = PI_COMMS_MISSION_EVENT_DONE;
        s_pi_comms_mission_event.fail_code = 0;
        s_pi_comms_mission_event.stamp_ms = pi_comms_now_ms();
        return;
    }

    if(sscanf(payload, "FAIL,%d", &code) == 1) {
        s_pi_comms_mission_event.type = PI_COMMS_MISSION_EVENT_FAIL;
        s_pi_comms_mission_event.fail_code = code;
        s_pi_comms_mission_event.stamp_ms = pi_comms_now_ms();
    }
}

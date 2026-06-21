#ifndef _service_pi_comms_h_
#define _service_pi_comms_h_

/**
 * @file pi_comms.h
 * @brief Pi 通信服务接口
 */

#include "serial_arm/five_dof_arm_kine.h"

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 类 型 声 明 ========================= ! //

typedef enum {
    PI_COMMS_STATUS_OK = 0,
    PI_COMMS_STATUS_INVALID_PARAM,
    PI_COMMS_STATUS_DEPENDENCY_MISSING
} PiCommsStatus;

typedef struct {
    bool (*write)(const char* data, uint32_t len);
    uint32_t(*now_ms)(void);
} PiCommsPortOps;

typedef struct {
    PiCommsPortOps port_ops;
} PiCommsConfig;

typedef struct {
    float vx;
    float vy;
    float wz;
    uint32_t stamp_ms;
} PiCommsChassisControl;

typedef struct {
    float yaw_rate;
    uint32_t stamp_ms;
} PiCommsYawRateControl;

typedef struct {
    FiveDofArmJointArray joints;
    float speed_rad_s;
    uint32_t stamp_ms;
} PiCommsArmJointControl;

typedef struct {
    PiCommsChassisControl chassis;
    PiCommsYawRateControl yaw_rate;
    PiCommsArmJointControl arm_joint;
} PiCommsControl;

typedef enum {
    PI_COMMS_YAW_ACTION_NONE = 0,
    PI_COMMS_YAW_ACTION_HOLD_ENABLE,
    PI_COMMS_YAW_ACTION_HOLD_DISABLE,
    PI_COMMS_YAW_ACTION_TARGET_SET
} PiCommsYawActionType;

typedef struct {
    PiCommsYawActionType type;
    float target_yaw;
    uint32_t stamp_ms;
} PiCommsYawAction;

typedef enum {
    PI_COMMS_ARM_ACTION_NONE = 0,
    PI_COMMS_ARM_ACTION_STOP,
    PI_COMMS_ARM_ACTION_ENABLE,
    PI_COMMS_ARM_ACTION_SEQUENCE_ID
} PiCommsArmActionType;

typedef struct {
    PiCommsArmActionType type;
    uint16_t sequence_id;
    uint32_t stamp_ms;
} PiCommsArmAction;

typedef enum {
    PI_COMMS_MISSION_EVENT_NONE = 0,
    PI_COMMS_MISSION_EVENT_DONE,
    PI_COMMS_MISSION_EVENT_FAIL
} PiCommsMissionEventType;

typedef struct {
    PiCommsMissionEventType type;
    int32_t fail_code;
    uint32_t stamp_ms;
} PiCommsMissionEvent;

typedef struct {
    uint32_t stamp_ms;
} PiCommsEstopEvent;

typedef struct {
    uint32_t stamp_ms;
    const char* state;
    const char* manual;
    uint8_t chassis_ready;
    uint8_t arm_ready;
    uint8_t odom_ready;
    uint8_t remote_online;
    uint8_t pc_online;
    uint8_t pi_online;
    uint8_t fault;
    uint8_t fault_source;
    uint8_t fault_level;
    int32_t fault_code;
} PiCommsStatusSnapshot;

typedef struct {
    uint32_t stamp_ms;
    float angle_x;
    float angle_y;
    float angle_z;
    float gyro_z;
    float odom_x;
    float odom_y;
    float odom_z;
} PiCommsImuOdomSnapshot;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

PiCommsStatus pi_comms_init(const PiCommsConfig* config);
void pi_comms_on_rx_byte(uint8_t data);
void pi_comms_process(void);
bool pi_comms_is_online(void);
bool pi_comms_control_is_fresh(uint32_t timeout_ms);
bool pi_comms_get_control(PiCommsControl* control);
bool pi_comms_get_chassis_control(PiCommsChassisControl* control);
bool pi_comms_get_yaw_rate_control(PiCommsYawRateControl* control);
bool pi_comms_get_arm_joint_control(PiCommsArmJointControl* control);
bool pi_comms_chassis_control_is_fresh(uint32_t timeout_ms);
bool pi_comms_yaw_rate_control_is_fresh(uint32_t timeout_ms);
bool pi_comms_arm_joint_control_is_fresh(uint32_t timeout_ms);
bool pi_comms_take_yaw_action(PiCommsYawAction* action);
bool pi_comms_take_arm_action(PiCommsArmAction* action);
bool pi_comms_has_pending_arm_action(void);
bool pi_comms_take_estop(PiCommsEstopEvent* event);
bool pi_comms_take_mission_event(PiCommsMissionEvent* event);
bool pi_comms_send_status(const PiCommsStatusSnapshot* status);
bool pi_comms_send_imu_odom(const PiCommsImuOdomSnapshot* odom);
void pi_comms_clear_controls(void);

#endif

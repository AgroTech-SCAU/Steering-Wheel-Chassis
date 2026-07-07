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
    int (*get_last_tx_result)(void);
    uint32_t (*now_ms)(void);
} PiCommsPortOps;

typedef struct {
    PiCommsPortOps port_ops;
} PiCommsConfig;

typedef struct {
    float vx;
    float vy;
    float wz;
    bool brake_request;
    uint32_t stamp_ms;
} PiCommsChassisControl;

typedef enum {
    PI_COMMS_ARM_MODE_NONE = 0,
    PI_COMMS_ARM_MODE_JOINTS,
    PI_COMMS_ARM_MODE_POSE_5D,
    PI_COMMS_ARM_MODE_POSITION,
    PI_COMMS_ARM_MODE_ORIENTATION_2D
} PiCommsArmMode;

typedef struct {
    float x;
    float y;
    float z;
    float pitch;
    float yaw;
} PiCommsArmPose5dTarget;

typedef struct {
    float x;
    float y;
    float z;
} PiCommsArmPositionTarget;

typedef struct {
    float pitch;
    float yaw;
} PiCommsArmOrientation2dTarget;

typedef struct {
    PiCommsArmMode mode;
    union {
        FiveDofArmJointArray joints;
        PiCommsArmPose5dTarget pose_5d;
        PiCommsArmPositionTarget position;
        PiCommsArmOrientation2dTarget orientation_2d;
    } target;
    float speed_rad_s;
    uint16_t command_seq;
    uint32_t stamp_ms;
} PiCommsArmControl;

typedef struct {
    PiCommsChassisControl chassis;
    PiCommsArmControl arm;
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
    uint8_t reason;
} PiCommsEstopEvent;

typedef struct {
    uint32_t stamp_ms;
    uint8_t app_state;
    uint8_t manual_mode;
    uint8_t ready_flags;
    uint8_t online_flags;
    uint8_t fault_source;
    uint8_t fault_level;
    int16_t fault_code;
    uint8_t auto_start_latched;
} PiCommsStatusSnapshot;

/**
 * @brief MCU -> Pi IMU 快照，100Hz 周期发送
 *
 * payload 内多字节字段统一按小端打包，roll/pitch/yaw 全部来自融合姿态 angle
 * yaw 与 ODOM 帧保留同源副本，便于 Pi 端分别发布 IMU 与里程计话题
 */
typedef struct {
    uint32_t stamp_ms;
    uint16_t status_flags;
    uint16_t sequence_count;
    int32_t acc_x_mm_s2;
    int32_t acc_y_mm_s2;
    int32_t acc_z_mm_s2;
    int32_t gyro_x_urad_s;
    int32_t gyro_y_urad_s;
    int32_t gyro_z_urad_s;
    int32_t roll_urad;
    int32_t pitch_urad;
    int32_t yaw_urad;
    uint32_t reserved;
} PiCommsImuSnapshot;

/**
 * @brief MCU -> Pi 底盘局部里程计快照，50Hz 周期发送
 *
 * x/y/yaw 属于 odom 坐标系，vx/vy/wz 属于 base_link 坐标系
 * yaw 同样来自融合后的 angle.z，不使用 odom.z 充当航向角
 */
typedef struct {
    uint32_t stamp_ms;
    uint16_t status_flags;
    uint16_t reset_counter;
    int32_t x_mm;
    int32_t y_mm;
    int32_t yaw_urad;
    int32_t vx_mm_s;
    int32_t vy_mm_s;
    int32_t wz_urad_s;
} PiCommsOdomSnapshot;

typedef struct {
    uint32_t stamp_ms;
    uint16_t status_flags;
    uint16_t sequence_count;
    int32_t q0_urad;
    int32_t q1_urad;
    int32_t q2_urad;
    int32_t q3_urad;
    int32_t q4_urad;
    int32_t x_mm;
    int32_t y_mm;
    int32_t z_mm;
    int16_t quat_x_q15;
    int16_t quat_y_q15;
    int16_t quat_z_q15;
    int16_t quat_w_q15;
} PiCommsArmStateSnapshot;

typedef struct {
    uint8_t sensor_id;
    uint8_t event_type;
    uint16_t value;
    uint32_t stamp_ms;
} PiCommsStartSensorEvent;

typedef struct {
    uint32_t rx_frame_count;
    uint32_t rx_bad_crc_count;
    uint32_t rx_bad_len_count;
    uint32_t rx_unknown_msg_count;
    uint8_t rx_last_msg_id;
    uint8_t rx_last_seq;
    uint32_t last_rx_ms;
    uint32_t tx_attempt_count;
    uint32_t tx_ok_count;
    uint32_t tx_frame_count;
    uint32_t tx_fail_count;
    uint32_t tx_pack_fail_count;
    uint32_t tx_crc_precheck_fail_count;
    uint32_t tx_crc_postcheck_fail_count;
    uint32_t tx_buffer_corruption_count;
    uint32_t tx_hal_busy_count;
    uint32_t tx_hal_timeout_count;
    uint32_t tx_hal_error_count;
    uint32_t tx_status_attempt_count;
    uint32_t tx_status_ok_count;
    uint32_t tx_status_fail_count;
    uint32_t tx_imu_attempt_count;
    uint32_t tx_imu_ok_count;
    uint32_t tx_imu_fail_count;
    uint32_t tx_odom_attempt_count;
    uint32_t tx_odom_ok_count;
    uint32_t tx_odom_fail_count;
    uint32_t tx_arm_state_attempt_count;
    uint32_t tx_arm_state_ok_count;
    uint32_t tx_arm_state_fail_count;
    uint32_t pending_event_count;
    uint32_t ack_timeout_count;
} PiCommsStats;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

#define PI_COMMS_PAYLOAD_IMU_LEN 48u
#define PI_COMMS_PAYLOAD_ODOM_LEN 32u
#define PI_COMMS_PAYLOAD_ARM_STATE_LEN 48u

#define PI_COMMS_IMU_STATUS_IMU_READY 0x0001u
#define PI_COMMS_IMU_STATUS_ACC_VALID 0x0002u
#define PI_COMMS_IMU_STATUS_GYRO_VALID 0x0004u
#define PI_COMMS_IMU_STATUS_ANGLE_VALID 0x0008u
#define PI_COMMS_IMU_STATUS_YAW_FUSED_WITH_CHASSIS 0x0010u
#define PI_COMMS_IMU_STATUS_BIAS_CORRECTED 0x0020u

#define PI_COMMS_ODOM_STATUS_ODOM_READY 0x0001u
#define PI_COMMS_ODOM_STATUS_POSE_VALID 0x0002u
#define PI_COMMS_ODOM_STATUS_TWIST_VALID 0x0004u
#define PI_COMMS_ODOM_STATUS_IMU_FUSED 0x0008u
#define PI_COMMS_ODOM_STATUS_WHEEL_FUSED 0x0010u
#define PI_COMMS_ODOM_STATUS_STATIC_DETECTED 0x0020u
#define PI_COMMS_ODOM_STATUS_SLIP_SUSPECTED 0x0040u
#define PI_COMMS_ODOM_STATUS_ODOM_RESET 0x0080u

#define PI_COMMS_ARM_STATE_STATUS_ARM_READY 0x0001u
#define PI_COMMS_ARM_STATE_STATUS_JOINT_VALID 0x0002u
#define PI_COMMS_ARM_STATE_STATUS_FK_VALID 0x0004u
#define PI_COMMS_ARM_STATE_STATUS_POSE_VALID PI_COMMS_ARM_STATE_STATUS_FK_VALID

PiCommsStatus pi_comms_init(const PiCommsConfig* config);
void pi_comms_on_rx_byte(uint8_t data);
void pi_comms_process(void);
bool pi_comms_is_online(void);
bool pi_comms_control_is_fresh(uint32_t timeout_ms);
bool pi_comms_get_control(PiCommsControl* control);
bool pi_comms_get_chassis_control(PiCommsChassisControl* control);
bool pi_comms_get_arm_control(PiCommsArmControl* control);
bool pi_comms_chassis_control_is_fresh(uint32_t timeout_ms);
bool pi_comms_arm_control_is_fresh(uint32_t timeout_ms);
bool pi_comms_take_arm_control(PiCommsArmControl* control);
bool pi_comms_has_pending_arm_control(void);
bool pi_comms_take_yaw_action(PiCommsYawAction* action);
bool pi_comms_take_arm_action(PiCommsArmAction* action);
bool pi_comms_has_pending_arm_action(void);
bool pi_comms_take_estop(PiCommsEstopEvent* event);
bool pi_comms_take_mission_event(PiCommsMissionEvent* event);
bool pi_comms_send_status(const PiCommsStatusSnapshot* status);
bool pi_comms_send_imu(const PiCommsImuSnapshot* snapshot);
bool pi_comms_send_odom(const PiCommsOdomSnapshot* snapshot);
bool pi_comms_send_arm_state(const PiCommsArmStateSnapshot* snapshot);
bool pi_comms_publish_start_sensor_event(uint8_t sensor_id,
                                         uint8_t event_type,
                                         uint16_t value);
bool pi_comms_send_ack(uint8_t ack_msg_id, uint8_t ack_seq, uint16_t code);
bool pi_comms_get_stats(PiCommsStats* stats);
void pi_comms_clear_controls(void);

#endif

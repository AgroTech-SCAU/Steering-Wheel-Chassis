#ifndef _chassis_imu_odom_h_
#define _chassis_imu_odom_h_

#include <stdbool.h>

#include "kalman.h"
#include "matrix.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

#define chassis_imu_odom chassis_imu_odom_interface

#define CHASSIS_IMU_ODOM_STATUS_TABLE     \
    X(OK, "OK")                           \
    X(INVALID_PARAM, "Invalid Parameter") \
    X(KALMAN_FAILED, "Kalman Failed")     \
    X(NOT_INITIALIZE, "Not Initialize")

#define X(name, str) CHASSIS_IMU_ODOM_##name,
typedef enum {
    CHASSIS_IMU_ODOM_STATUS_TABLE
} ChassisImuOdomErrorCode;
#undef X

typedef enum {
    CHASSIS_IMU_ODOM_STATE_X = 0,
    CHASSIS_IMU_ODOM_STATE_Y,
    CHASSIS_IMU_ODOM_STATE_Z,
    CHASSIS_IMU_ODOM_STATE_ROLL,
    CHASSIS_IMU_ODOM_STATE_PITCH,
    CHASSIS_IMU_ODOM_STATE_YAW,
    CHASSIS_IMU_ODOM_STATE_VX,
    CHASSIS_IMU_ODOM_STATE_VY,
    CHASSIS_IMU_ODOM_STATE_WZ,
    CHASSIS_IMU_ODOM_STATE_DIM
} ChassisImuOdomStateIndex;

typedef struct {
    float vx;
    float vy;
    float wz;
} ChassisImuOdomChassis;

typedef struct {
    Vector3 acc;
    Vector3 gyro;
} ChassisImuOdomImu;

typedef struct {
    Vector3 angle;
    Vector3 odom;
    ChassisImuOdomChassis velocity;
} ChassisImuOdomOutput;

typedef struct {
    float pos_process_noise;
    float angle_process_noise;
    float velocity_process_noise;
    float chassis_velocity_noise;
    float imu_angle_noise;
    float imu_gyro_noise;
    float gravity;
    float acc_norm_tolerance;
} ChassisImuOdomConfig;

typedef struct {
    KalmanFilter filter;
    ChassisImuOdomConfig config;
    ChassisImuOdomOutput output;
    bool initialized;
} ChassisImuOdom;

#define X(name, str) ChassisImuOdomErrorCode name;
extern const struct ChassisImuOdomInterface {
    struct {
        CHASSIS_IMU_ODOM_STATUS_TABLE
    };
    ChassisImuOdomErrorCode (*init)(ChassisImuOdom* odom, const ChassisImuOdomConfig* config);
    ChassisImuOdomErrorCode (*update)(ChassisImuOdom* odom, ChassisImuOdomChassis chassis, ChassisImuOdomImu imu, float dt);
    ChassisImuOdomErrorCode (*get_angle)(const ChassisImuOdom* odom, Vector3* angle);
    ChassisImuOdomErrorCode (*get_odom)(const ChassisImuOdom* odom, Vector3* odom_out);
    ChassisImuOdomErrorCode (*get_output)(const ChassisImuOdom* odom, ChassisImuOdomOutput* output);
    ChassisImuOdomErrorCode (*reset_odom)(ChassisImuOdom* odom, Vector3 odom_value);
    const char* (*error_code_to_str)(ChassisImuOdomErrorCode status);
} chassis_imu_odom_interface;
#undef X

// ! ========================= 接 口 函 数 声 明 ========================= ! //

ChassisImuOdomErrorCode chassis_imu_odom_init(ChassisImuOdom* odom, const ChassisImuOdomConfig* config);
ChassisImuOdomErrorCode chassis_imu_odom_update(ChassisImuOdom* odom, ChassisImuOdomChassis chassis, ChassisImuOdomImu imu, float dt);
ChassisImuOdomErrorCode chassis_imu_odom_get_angle(const ChassisImuOdom* odom, Vector3* angle);
ChassisImuOdomErrorCode chassis_imu_odom_get_odom(const ChassisImuOdom* odom, Vector3* odom_out);
ChassisImuOdomErrorCode chassis_imu_odom_get_output(const ChassisImuOdom* odom, ChassisImuOdomOutput* output);
ChassisImuOdomErrorCode chassis_imu_odom_reset_odom(ChassisImuOdom* odom, Vector3 odom_value);
const char* chassis_imu_odom_error_code_to_str(ChassisImuOdomErrorCode status);

#endif

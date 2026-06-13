#ifndef _odom_h_
#define _odom_h_

#include <stdbool.h>

#include "kalman/chassis_imu_odom.h"
#include "matrix.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

#define odom odom_interface

#define ODOM_STATUS_TABLE \
    X(OK, "OK") \
    X(INVALID_PARAM, "Invalid Parameter") \
    X(DEPENDENCY_MISSING, "Odom Dependency Missing") \
    X(FUSION_FAILED, "Odom Fusion Failed") \
    X(NOT_INITIALIZED, "Odom Not Initialized") \
    X(NOT_READY, "Odom Not Ready")

#define X(name, str) ODOM_##name,
typedef enum {
    ODOM_STATUS_TABLE
} OdomStatus;
#undef X

typedef struct {
    ChassisImuOdomConfig fusion;
    float process_period_s;
} OdomConfig;

typedef struct {
    Vector3 acc;
    Vector3 gyro;
    Vector3 gyro_bias;
    Vector3 gyro_corrected;
    Vector3 angle;
    Vector3 odom;
    bool imu_ready;
    bool fusion_ready;
    bool initialized;
} Odom;

#define X(name, str) OdomStatus name;
extern const struct OdomInterface {
    struct {
        ODOM_STATUS_TABLE
    };
    OdomConfig(*default_config)(void);
    OdomStatus(*init)(const OdomConfig* config);
    OdomStatus(*process)(void);
    OdomStatus(*get_acc)(Vector3* acc);
    OdomStatus(*get_gyro)(Vector3* gyro);
    OdomStatus(*get_gyro_bias)(Vector3* gyro_bias);
    OdomStatus(*get_gyro_corrected)(Vector3* gyro_corrected);
    OdomStatus(*get_angle)(Vector3* angle);
    OdomStatus(*get_odom)(Vector3* odom_out);
    const Odom* (*get_state)(void);
    bool (*is_ready)(void);
    const char* (*status_str)(OdomStatus status);
} odom_interface;
#undef X

// ! ========================= 接 口 函 数 声 明 ========================= ! //

OdomConfig odom_default_config(void);
OdomStatus odom_init(const OdomConfig* config);
OdomStatus odom_process(void);
OdomStatus odom_get_acc(Vector3* acc);
OdomStatus odom_get_gyro(Vector3* gyro);
OdomStatus odom_get_gyro_bias(Vector3* gyro_bias);
OdomStatus odom_get_gyro_corrected(Vector3* gyro_corrected);
OdomStatus odom_get_angle(Vector3* angle);
OdomStatus odom_get_odom(Vector3* odom_out);
const Odom* odom_get_state(void);
bool odom_is_ready(void);
const char* odom_status_str(OdomStatus status);

#endif

#ifndef _kalman_h_
#define _kalman_h_

#include <stdbool.h>
#include <stdint.h>

#include "matrix.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

#define kalman kalman_interface

#define KALMAN_MAX_STATE_DIM 9u
#define KALMAN_MAX_MEAS_DIM 9u
#define KALMAN_MAX_CTRL_DIM 6u

#define KALMAN_STATUS_TABLE                   \
    X(OK, "OK")                               \
    X(INVALID_PARAM, "Invalid Parameter")     \
    X(INVALID_DIM, "Invalid Dimension")       \
    X(MATRIX_FAILED, "Matrix Compute Failed") \
    X(NOT_INITIALIZE, "Not Initialize")

#define X(name, str) KALMAN_##name,
typedef enum {
    KALMAN_STATUS_TABLE
} KalmanErrorCode;
#undef X

typedef struct {
    uint8_t state_dim;
    uint8_t meas_dim;
    uint8_t ctrl_dim;

    Matrix x;
    Matrix P;
    Matrix F;
    Matrix B;
    Matrix u;
    Matrix Q;
    Matrix z;
    Matrix H;
    Matrix R;

    float x_data[KALMAN_MAX_STATE_DIM];
    float P_data[KALMAN_MAX_STATE_DIM * KALMAN_MAX_STATE_DIM];
    float F_data[KALMAN_MAX_STATE_DIM * KALMAN_MAX_STATE_DIM];
    float B_data[KALMAN_MAX_STATE_DIM * KALMAN_MAX_CTRL_DIM];
    float u_data[KALMAN_MAX_CTRL_DIM];
    float Q_data[KALMAN_MAX_STATE_DIM * KALMAN_MAX_STATE_DIM];
    float z_data[KALMAN_MAX_MEAS_DIM];
    float H_data[KALMAN_MAX_MEAS_DIM * KALMAN_MAX_STATE_DIM];
    float R_data[KALMAN_MAX_MEAS_DIM * KALMAN_MAX_MEAS_DIM];

    bool initialized;
} KalmanFilter;

#define X(name, str) KalmanErrorCode name;
extern const struct KalmanInterface {
    struct {
        KALMAN_STATUS_TABLE
    };
    KalmanErrorCode (*filter_init)(KalmanFilter* filter, uint8_t state_dim, uint8_t meas_dim, uint8_t ctrl_dim);
    KalmanErrorCode (*filter_predict)(KalmanFilter* filter);
    KalmanErrorCode (*filter_update)(KalmanFilter* filter);
    const char* (*error_code_to_str)(KalmanErrorCode status);
} kalman_interface;
#undef X

// ! ========================= 接 口 函 数 声 明 ========================= ! //

KalmanErrorCode kalman_filter_init(KalmanFilter* filter, uint8_t state_dim, uint8_t meas_dim, uint8_t ctrl_dim);
KalmanErrorCode kalman_filter_predict(KalmanFilter* filter);
KalmanErrorCode kalman_filter_update(KalmanFilter* filter);
const char* kalman_error_code_to_str(KalmanErrorCode status);

#endif

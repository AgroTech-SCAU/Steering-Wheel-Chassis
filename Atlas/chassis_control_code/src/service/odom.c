#include "odom.h"

#include "chassis.h"
#include "imu/imu.h"

#include <stddef.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define od odom_interface

#define ODOM_DEFAULT_PROCESS_PERIOD_S 0.002f

static Odom s_odom = { 0 };
static ChassisImuOdom s_fusion = { 0 };
static OdomConfig s_config = { 0 };

#define X(name, str) .name = ODOM_##name,
const struct OdomInterface odom_interface = {
    {
        ODOM_STATUS_TABLE
    },
    .default_config = odom_default_config,
    .init = odom_init,
    .process = odom_process,
    .get_acc = odom_get_acc,
    .get_gyro = odom_get_gyro,
    .get_gyro_bias = odom_get_gyro_bias,
    .get_gyro_corrected = odom_get_gyro_corrected,
    .get_angle = odom_get_angle,
    .get_odom = odom_get_odom,
    .get_state = odom_get_state,
    .is_ready = odom_is_ready,
    .status_str = odom_status_str
};
#undef X

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static Vector3 odom_acc_to_vec3(ImuAcc acc);
static Vector3 odom_gyro_to_vec3(ImuGyro gyro);
static Vector3 odom_angle_to_vec3(ImuAngle angle);
static OdomStatus odom_copy_vec3(bool ready, const Vector3* src, Vector3* out);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

OdomConfig odom_default_config(void) {
    OdomConfig config = {
        .fusion = {
            .pos_process_noise = 0.01f,
            .angle_process_noise = 0.02f,
            .velocity_process_noise = 0.1f,
            .chassis_velocity_noise = 0.04f,
            .imu_angle_noise = 0.08f,
            .imu_gyro_noise = 0.03f,
            .gravity = 9.80665f,
            .acc_norm_tolerance = 2.0f,
        },
        .process_period_s = ODOM_DEFAULT_PROCESS_PERIOD_S,
    };

    return config;
}

OdomStatus odom_init(const OdomConfig* config) {
    ChassisImuOdomErrorCode status;

    memset(&s_odom, 0, sizeof(s_odom));
    memset(&s_fusion, 0, sizeof(s_fusion));
    s_config = (config != NULL) ? *config : odom_default_config();
    if(s_config.process_period_s <= 0.0f) {
        s_config.process_period_s = ODOM_DEFAULT_PROCESS_PERIOD_S;
    }

    status = chassis_imu_odom.init(&s_fusion, &s_config.fusion);
    if(status != chassis_imu_odom.OK) {
        return od.FUSION_FAILED;
    }

    s_odom.initialized = true;
    return od.OK;
}

OdomStatus odom_process(void) {
    ImuAcc acc;
    ImuGyro gyro;
    ImuGyro gyro_bias;
    ImuGyro gyro_corrected;
    ImuAngle imu_angle;
    ChassisImuOdomChassis chassis_velocity = { 0.0f, 0.0f, 0.0f };
    ChassisImuOdomImu imu_sample;
    const SteerWheelState* chassis_state;

    if(!s_odom.initialized) {
        return od.NOT_INITIALIZED;
    }

    if(imu.update() != IMU_STATUS_OK) {
        s_odom.imu_ready = false;
        return od.NOT_READY;
    }

    acc = imu.get_acc();
    gyro = imu.get_gyro();
    gyro_bias = imu_get_gyro_bias();
    gyro_corrected = imu_get_gyro_corrected();
    imu_angle = imu.get_angle();

    s_odom.acc = odom_acc_to_vec3(acc);
    s_odom.gyro = odom_gyro_to_vec3(gyro);
    s_odom.gyro_bias = odom_gyro_to_vec3(gyro_bias);
    s_odom.gyro_corrected = odom_gyro_to_vec3(gyro_corrected);
    s_odom.angle = odom_angle_to_vec3(imu_angle);
    s_odom.imu_ready = true;

    chassis_state = chassis.get_state();
    if(chassis_state != NULL) {
        chassis_velocity.vx = chassis_state->cur_vx;
        chassis_velocity.vy = chassis_state->cur_vy;
        chassis_velocity.wz = chassis_state->cur_wz;
    }

    imu_sample.acc = s_odom.acc;
    imu_sample.gyro = s_odom.gyro_corrected;
    if(chassis_imu_odom.update(&s_fusion, chassis_velocity, imu_sample, s_config.process_period_s) != chassis_imu_odom.OK) {
        s_odom.fusion_ready = false;
        return od.FUSION_FAILED;
    }

    (void)chassis_imu_odom.get_angle(&s_fusion, &s_odom.angle);
    (void)chassis_imu_odom.get_odom(&s_fusion, &s_odom.odom);
    s_odom.fusion_ready = true;

    return od.OK;
}

OdomStatus odom_get_acc(Vector3* acc) {
    return odom_copy_vec3(s_odom.imu_ready, &s_odom.acc, acc);
}

OdomStatus odom_get_gyro(Vector3* gyro) {
    return odom_copy_vec3(s_odom.imu_ready, &s_odom.gyro, gyro);
}

OdomStatus odom_get_gyro_bias(Vector3* gyro_bias) {
    return odom_copy_vec3(s_odom.imu_ready, &s_odom.gyro_bias, gyro_bias);
}

OdomStatus odom_get_gyro_corrected(Vector3* gyro_corrected) {
    return odom_copy_vec3(s_odom.imu_ready, &s_odom.gyro_corrected, gyro_corrected);
}

OdomStatus odom_get_angle(Vector3* angle) {
    return odom_copy_vec3(s_odom.fusion_ready, &s_odom.angle, angle);
}

OdomStatus odom_get_odom(Vector3* odom_out) {
    return odom_copy_vec3(s_odom.fusion_ready, &s_odom.odom, odom_out);
}

const Odom* odom_get_state(void) {
    return &s_odom;
}

bool odom_is_ready(void) {
    return s_odom.initialized && s_odom.imu_ready && s_odom.fusion_ready;
}

#define X(name, str) case ODOM_##name: return str;
const char* odom_status_str(OdomStatus status) {
    switch(status) {
        ODOM_STATUS_TABLE
        default: return "UNKNOWN";
    }
}
#undef X

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static Vector3 odom_acc_to_vec3(ImuAcc acc) {
    Vector3 out = {
        .x = acc.x,
        .y = acc.y,
        .z = acc.z,
    };

    return out;
}

static Vector3 odom_gyro_to_vec3(ImuGyro gyro) {
    Vector3 out = {
        .x = gyro.x,
        .y = gyro.y,
        .z = gyro.z,
    };

    return out;
}

static Vector3 odom_angle_to_vec3(ImuAngle angle) {
    Vector3 out = {
        .x = angle.roll,
        .y = angle.pitch,
        .z = angle.yaw,
    };

    return out;
}

static OdomStatus odom_copy_vec3(bool ready, const Vector3* src, Vector3* out) {
    if(src == NULL || out == NULL) {
        return od.INVALID_PARAM;
    }
    if(!s_odom.initialized) {
        return od.NOT_INITIALIZED;
    }
    if(!ready) {
        return od.NOT_READY;
    }

    *out = *src;
    return od.OK;
}

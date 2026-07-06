#include "odom.h"

#include "chassis.h"
#include "imu/imu.h"
#include "stm32_hal_dwt.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 里程计服务接口单例的文件内短别名
 */
#define od odom_interface

/**
 * @brief 默认里程计服务周期
 *
 * 当前由 250Hz 任务调用，周期为 0.004s
 */
#define ODOM_DEFAULT_PROCESS_PERIOD_S 0.004f
/**
 * @brief 里程计服务周期上限
 *
 * 当 IMU 样本时间戳间隔超过该值时，使用 fallback 时间戳计算周期
 */
#define ODOM_MAX_PROCESS_DT_S 0.05f

/**
 * @brief 里程计服务对外状态缓存
 */
static Odom s_odom = { 0 };
/**
 * @brief 底盘 + IMU 融合算法实例
 */
static ChassisImuOdom s_fusion = { 0 };
/**
 * @brief 当前里程计服务配置
 */
static OdomConfig s_config = { 0 };
/**
 * @brief 上次 IMU 样本时间戳，单位微秒
 */
static uint32_t s_last_gyro_timestamp_us = 0u;
/**
 * @brief 上次 fallback 时间戳，单位微秒
 */
static uint32_t s_last_fallback_timestamp_us = 0u;

/**
 * @brief 里程计服务接口单例
 */
#define X(name, str) .name = ODOM_##name,
const struct OdomInterface odom_interface = {
    { ODOM_STATUS_TABLE },
    .default_config = odom_default_config,
    .init = odom_init,
    .process = odom_process,
    .get_acc = odom_get_acc,
    .get_gyro = odom_get_gyro,
    .get_gyro_bias = odom_get_gyro_bias,
    .get_gyro_corrected = odom_get_gyro_corrected,
    .get_angle = odom_get_angle,
    .get_odom = odom_get_odom,
    .get_velocity = odom_get_velocity,
    .get_state = odom_get_state,
    .is_ready = odom_is_ready,
    .status_str = odom_status_str
};
#undef X

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 将 IMU 加速度结构转换为通用 Vector3
 */
static Vector3 odom_acc_to_vec3(ImuAcc acc);
/**
 * @brief 将 IMU 角速度结构转换为通用 Vector3
 */
static Vector3 odom_gyro_to_vec3(ImuGyro gyro);
/**
 * @brief 将 IMU 姿态角结构转换为通用 Vector3
 */
static Vector3 odom_angle_to_vec3(ImuAngle angle);
/**
 * @brief 检查数据是否可用并拷贝 Vector3 输出
 */
static OdomStatus odom_copy_vec3(bool ready, const Vector3* src, Vector3* out);
/**
 * @brief 解析 IMU 样本时间戳，计算本次里程计服务周期
 */
static float odom_resolve_process_dt_s(const ImuSample* sample);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 获取默认里程计配置
 * @return OdomConfig 默认配置
 */
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
            .chassis_linear_static_threshold = 0.003f,
            .chassis_angular_static_threshold = 0.02f,
            .imu_gyro_static_threshold = 0.01f,
        },
        .yaw_rate_scale = 1.0f,
        .process_period_s = ODOM_DEFAULT_PROCESS_PERIOD_S,
    };

    return config;
}

/**
 * @brief 初始化里程计服务
 * @param config 配置指针；NULL 表示使用默认配置
 * @return OdomStatus 状态码
 */
OdomStatus odom_init(const OdomConfig* config) {
    ChassisImuOdomErrorCode status;

    memset(&s_odom, 0, sizeof(s_odom));
    memset(&s_fusion, 0, sizeof(s_fusion));
    s_config = (config != NULL) ? *config : odom_default_config();
    s_last_gyro_timestamp_us = 0u;
    s_last_fallback_timestamp_us = 0u;
    if(s_config.process_period_s <= 0.0f) {
        s_config.process_period_s = ODOM_DEFAULT_PROCESS_PERIOD_S;
    }
    if(!isfinite(s_config.yaw_rate_scale) || s_config.yaw_rate_scale <= 0.0f) {
        s_config.yaw_rate_scale = 1.0f;
    }

    status = chassis_imu_odom.init(&s_fusion, &s_config.fusion);
    if(status != chassis_imu_odom.OK) {
        return od.FUSION_FAILED;
    }

    s_odom.initialized = true;
    return od.OK;
}

/**
 * @brief 执行一次里程计服务流程
 *
 * 该函数统一刷新 IMU，读取底盘速度，并更新底盘 + IMU 融合里程；
 * 上层任务不再直接调用 imu.update()
 *
 * @return OdomStatus 状态码
 */
OdomStatus odom_process(void) {
    ImuAcc acc;
    ImuGyro gyro;
    ImuGyro gyro_bias;
    ImuGyro gyro_corrected;
    ImuAngle imu_angle;
    ImuSample imu_raw_sample = { 0 };
    ChassisImuOdomChassis chassis_velocity = { 0.0f, 0.0f, 0.0f };
    ChassisImuOdomImu imu_sample;
    const SteerWheelState* chassis_state;
    float process_dt_s;

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
    (void)imu_get_sample(&imu_raw_sample);
    process_dt_s = odom_resolve_process_dt_s(&imu_raw_sample);

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
    chassis_velocity.wz *= s_config.yaw_rate_scale;

    imu_sample.acc = s_odom.acc;
    imu_sample.gyro = s_odom.gyro_corrected;
    imu_sample.gyro.z *= s_config.yaw_rate_scale;
    if(chassis_imu_odom.update(&s_fusion, chassis_velocity, imu_sample, process_dt_s) != chassis_imu_odom.OK) {
        s_odom.fusion_ready = false;
        return od.FUSION_FAILED;
    }

    (void)chassis_imu_odom.get_angle(&s_fusion, &s_odom.angle);
    (void)chassis_imu_odom.get_odom(&s_fusion, &s_odom.odom);
    s_odom.velocity.x = s_fusion.output.velocity.vx;
    s_odom.velocity.y = s_fusion.output.velocity.vy;
    s_odom.velocity.z = s_fusion.output.velocity.wz;
    s_odom.fusion_ready = true;

    return od.OK;
}

/**
 * @brief 获取最近一次三轴加速度
 * @param acc 输出加速度
 * @return OdomStatus 状态码
 */
OdomStatus odom_get_acc(Vector3* acc) {
    return odom_copy_vec3(s_odom.imu_ready, &s_odom.acc, acc);
}

/**
 * @brief 获取最近一次原始三轴角速度
 * @param gyro 输出角速度
 * @return OdomStatus 状态码
 */
OdomStatus odom_get_gyro(Vector3* gyro) {
    return odom_copy_vec3(s_odom.imu_ready, &s_odom.gyro, gyro);
}

/**
 * @brief 获取最近一次陀螺零偏估计
 * @param gyro_bias 输出零偏
 * @return OdomStatus 状态码
 */
OdomStatus odom_get_gyro_bias(Vector3* gyro_bias) {
    return odom_copy_vec3(s_odom.imu_ready, &s_odom.gyro_bias, gyro_bias);
}

/**
 * @brief 获取最近一次修正后三轴角速度
 * @param gyro_corrected 输出修正角速度
 * @return OdomStatus 状态码
 */
OdomStatus odom_get_gyro_corrected(Vector3* gyro_corrected) {
    return odom_copy_vec3(s_odom.imu_ready, &s_odom.gyro_corrected, gyro_corrected);
}

/**
 * @brief 获取融合后的三轴姿态角
 * @param angle 输出姿态角
 * @return OdomStatus 状态码
 */
OdomStatus odom_get_angle(Vector3* angle) {
    return odom_copy_vec3(s_odom.fusion_ready, &s_odom.angle, angle);
}

/**
 * @brief 获取融合后的三轴里程
 * @param odom_out 输出里程
 * @return OdomStatus 状态码
 */
OdomStatus odom_get_odom(Vector3* odom_out) {
    return odom_copy_vec3(s_odom.fusion_ready, &s_odom.odom, odom_out);
}

/**
 * @brief 获取融合后的底盘速度
 * @details x/y/z 分别表示 base_link 坐标系下的 vx/vy/wz
 * @param velocity_out 输出速度
 * @return OdomStatus 状态码
 */
OdomStatus odom_get_velocity(Vector3* velocity_out) {
    return odom_copy_vec3(s_odom.fusion_ready, &s_odom.velocity, velocity_out);
}

/**
 * @brief 获取里程计服务只读状态快照
 * @return const Odom* 状态快照指针
 */
const Odom* odom_get_state(void) {
    return &s_odom;
}

/**
 * @brief 判断里程计服务是否已有可用数据
 * @return true 数据可用
 * @return false 数据尚未可用
 */
bool odom_is_ready(void) {
    return s_odom.initialized && s_odom.imu_ready && s_odom.fusion_ready;
}

/**
 * @brief 将里程计服务状态码转换为静态字符串
 * @param status 状态码
 * @return const char* 状态码说明
 */
#define X(name, str)  \
    case ODOM_##name: \
        return str;
const char* odom_status_str(OdomStatus status) {
    switch(status) {
        ODOM_STATUS_TABLE
        default:
            return "UNKNOWN";
    }
}
#undef X

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 将 IMU 加速度结构转换为通用 Vector3
 * @param acc IMU 加速度
 * @return Vector3 通用三轴向量
 */
static Vector3 odom_acc_to_vec3(ImuAcc acc) {
    Vector3 out = {
        .x = acc.x,
        .y = acc.y,
        .z = acc.z,
    };

    return out;
}

/**
 * @brief 将 IMU 角速度结构转换为通用 Vector3
 * @param gyro IMU 角速度
 * @return Vector3 通用三轴向量
 */
static Vector3 odom_gyro_to_vec3(ImuGyro gyro) {
    Vector3 out = {
        .x = gyro.x,
        .y = gyro.y,
        .z = gyro.z,
    };

    return out;
}

/**
 * @brief 将 IMU 姿态角结构转换为通用 Vector3
 * @param angle IMU 姿态角
 * @return Vector3 x/y/z 分别为 roll/pitch/yaw
 */
static Vector3 odom_angle_to_vec3(ImuAngle angle) {
    Vector3 out = {
        .x = angle.roll,
        .y = angle.pitch,
        .z = angle.yaw,
    };

    return out;
}

/**
 * @brief 检查数据是否可用并拷贝 Vector3 输出
 * @param ready 对应数据是否已经更新成功
 * @param src 数据源
 * @param out 输出缓存
 * @return OdomStatus 状态码
 */
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

/**
 * @brief 解析 IMU 样本时间戳，计算本次里程计服务周期
 * @param sample IMU 样本指针；NULL 表示使用 fallback 时间戳
 * @return float 里程计服务周期，单位秒
 */
static float odom_resolve_process_dt_s(const ImuSample* sample) {
    float dt_s = s_config.process_period_s;

    if(sample != NULL && sample->gyro_timestamp_us != 0u) {
        if(s_last_gyro_timestamp_us != 0u) {
            dt_s = (float)(sample->gyro_timestamp_us - s_last_gyro_timestamp_us) * 1.0e-6f;
        }
        s_last_gyro_timestamp_us = sample->gyro_timestamp_us;
    }
    else {
        const uint32_t now_us = dwt_get_us();

        if(s_last_fallback_timestamp_us != 0u && now_us != 0u) {
            dt_s = (float)(now_us - s_last_fallback_timestamp_us) * 1.0e-6f;
        }
        s_last_fallback_timestamp_us = now_us;
    }

    if(!isfinite(dt_s) || dt_s <= 0.0f || dt_s > ODOM_MAX_PROCESS_DT_S) {
        dt_s = s_config.process_period_s;
    }

    return dt_s;
}

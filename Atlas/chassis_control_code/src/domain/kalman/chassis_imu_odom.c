#include "chassis_imu_odom.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 底盘 + IMU 里程融合接口单例的文件内短别名
 */
#define cio chassis_imu_odom_interface

/**
 * @brief 浮点计算零阈值
 */
#define CHASSIS_IMU_ODOM_EPS 1e-6f
/**
 * @brief 圆周率常量
 */
#define CHASSIS_IMU_ODOM_PI 3.14159265358979323846f
/**
 * @brief 角度完整周期
 */
#define CHASSIS_IMU_ODOM_2PI (2.0f * CHASSIS_IMU_ODOM_PI)
/**
 * @brief 融合模型观测维度：vx、vy、wz、roll、pitch、gyro_z
 */
#define CHASSIS_IMU_ODOM_MEAS_DIM 6u

/**
 * @brief 底盘 + IMU 里程融合接口单例
 */
#define X(name, str) .name = CHASSIS_IMU_ODOM_##name,
const struct ChassisImuOdomInterface chassis_imu_odom_interface = {
    { CHASSIS_IMU_ODOM_STATUS_TABLE },
    .init = chassis_imu_odom_init,
    .update = chassis_imu_odom_update,
    .get_angle = chassis_imu_odom_get_angle,
    .get_odom = chassis_imu_odom_get_odom,
    .get_output = chassis_imu_odom_get_output,
    .reset_odom = chassis_imu_odom_reset_odom,
    .error_code_to_str = chassis_imu_odom_error_code_to_str
};
#undef X

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 将矩阵清零
 */
static void chassis_imu_odom_matrix_zero(Matrix* matrix);
/**
 * @brief 将矩阵置为单位阵；非方阵时只填充主对角线
 */
static void chassis_imu_odom_matrix_identity(Matrix* matrix);
/**
 * @brief 将角度归一化到 (-pi, pi]
 */
static float chassis_imu_odom_wrap_pi(float angle);
/**
 * @brief 获取有效噪声参数；输入无效时使用默认值
 */
static float chassis_imu_odom_noise(float value, float fallback);
/**
 * @brief 获取默认融合配置
 */
static ChassisImuOdomConfig chassis_imu_odom_default_config(void);
/**
 * @brief 判断底盘和 IMU 是否处于静止窗口
 */
static bool chassis_imu_odom_is_static(const ChassisImuOdom* odom, ChassisImuOdomChassis chassis, ChassisImuOdomImu imu);
/**
 * @brief 由加速度重力方向估计 roll/pitch
 */
static Vector3 chassis_imu_odom_acc_to_angle(Vector3 acc, float gravity, float tolerance, bool* trusted);
/**
 * @brief 将底层卡尔曼状态码映射到融合状态码
 */
static ChassisImuOdomErrorCode chassis_imu_odom_from_kalman(KalmanErrorCode status);
/**
 * @brief 将滤波器状态同步到输出缓存
 */
static void chassis_imu_odom_sync_output(ChassisImuOdom* odom);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化底盘 + IMU 里程融合实例
 * @param odom 融合实例
 * @param config 配置指针；NULL 表示使用默认配置
 * @return ChassisImuOdomErrorCode 状态码
 */
ChassisImuOdomErrorCode chassis_imu_odom_init(ChassisImuOdom* odom, const ChassisImuOdomConfig* config) {
    ChassisImuOdomConfig use_config;
    KalmanErrorCode status;

    if(odom == NULL) {
        return cio.INVALID_PARAM;
    }

    memset(odom, 0, sizeof(ChassisImuOdom));
    use_config = (config != NULL) ? *config : chassis_imu_odom_default_config();
    use_config.pos_process_noise = chassis_imu_odom_noise(use_config.pos_process_noise, 0.01f);
    use_config.angle_process_noise = chassis_imu_odom_noise(use_config.angle_process_noise, 0.02f);
    use_config.velocity_process_noise = chassis_imu_odom_noise(use_config.velocity_process_noise, 0.1f);
    use_config.chassis_velocity_noise = chassis_imu_odom_noise(use_config.chassis_velocity_noise, 0.04f);
    use_config.imu_angle_noise = chassis_imu_odom_noise(use_config.imu_angle_noise, 0.08f);
    use_config.imu_gyro_noise = chassis_imu_odom_noise(use_config.imu_gyro_noise, 0.03f);
    use_config.gravity = chassis_imu_odom_noise(use_config.gravity, 9.80665f);
    use_config.acc_norm_tolerance = chassis_imu_odom_noise(use_config.acc_norm_tolerance, 2.0f);
    use_config.chassis_linear_static_threshold = chassis_imu_odom_noise(use_config.chassis_linear_static_threshold, 0.003f);
    use_config.chassis_angular_static_threshold = chassis_imu_odom_noise(use_config.chassis_angular_static_threshold, 0.02f);
    use_config.imu_gyro_static_threshold = chassis_imu_odom_noise(use_config.imu_gyro_static_threshold, 0.01f);

    status = kalman.filter_init(&odom->filter, CHASSIS_IMU_ODOM_STATE_DIM, CHASSIS_IMU_ODOM_MEAS_DIM, 0u);
    if(status != kalman.OK) {
        return chassis_imu_odom_from_kalman(status);
    }

    odom->config = use_config;
    odom->initialized = true;
    chassis_imu_odom_sync_output(odom);

    return cio.OK;
}

/**
 * @brief 使用底盘速度和 IMU 六轴数据更新一次融合状态
 *
 * 当前预测模型只积分平面 x/y 和 yaw；z 状态没有高度输入，因此保持初始化或重置值
 *
 * @param odom 融合实例
 * @param chassis 底盘速度输入
 * @param imu IMU 六轴输入
 * @param dt 更新周期，单位 s
 * @return ChassisImuOdomErrorCode 状态码
 */
ChassisImuOdomErrorCode chassis_imu_odom_update(ChassisImuOdom* odom, ChassisImuOdomChassis chassis, ChassisImuOdomImu imu, float dt) {
    KalmanFilter* filter;
    Vector3 acc_angle;
    bool acc_trusted = false;
    KalmanErrorCode status;
    float yaw;
    float cos_yaw;
    float sin_yaw;
    uint8_t i;
    bool static_window;

    if(odom == NULL) {
        return cio.INVALID_PARAM;
    }
    if(!odom->initialized || !odom->filter.initialized) {
        return cio.NOT_INITIALIZE;
    }
    if(dt <= 0.0f || !isfinite(dt)) {
        return cio.INVALID_PARAM;
    }

    filter = &odom->filter;
    static_window = chassis_imu_odom_is_static(odom, chassis, imu);
    if(static_window) {
        chassis.vx = 0.0f;
        chassis.vy = 0.0f;
        chassis.wz = 0.0f;
        imu.gyro.z = 0.0f;
        filter->x_data[CHASSIS_IMU_ODOM_STATE_VX] = 0.0f;
        filter->x_data[CHASSIS_IMU_ODOM_STATE_VY] = 0.0f;
        filter->x_data[CHASSIS_IMU_ODOM_STATE_WZ] = 0.0f;
    }

    yaw = filter->x_data[CHASSIS_IMU_ODOM_STATE_YAW];
    cos_yaw = cosf(yaw);
    sin_yaw = sinf(yaw);

    chassis_imu_odom_matrix_identity(&filter->F);
    chassis_imu_odom_matrix_zero(&filter->Q);
    chassis_imu_odom_matrix_zero(&filter->H);
    chassis_imu_odom_matrix_zero(&filter->R);

    filter->F_data[CHASSIS_IMU_ODOM_STATE_X * filter->F.col + CHASSIS_IMU_ODOM_STATE_VX] = cos_yaw * dt;
    filter->F_data[CHASSIS_IMU_ODOM_STATE_X * filter->F.col + CHASSIS_IMU_ODOM_STATE_VY] = -sin_yaw * dt;
    filter->F_data[CHASSIS_IMU_ODOM_STATE_Y * filter->F.col + CHASSIS_IMU_ODOM_STATE_VX] = sin_yaw * dt;
    filter->F_data[CHASSIS_IMU_ODOM_STATE_Y * filter->F.col + CHASSIS_IMU_ODOM_STATE_VY] = cos_yaw * dt;
    filter->F_data[CHASSIS_IMU_ODOM_STATE_YAW * filter->F.col + CHASSIS_IMU_ODOM_STATE_WZ] = dt;

    filter->x_data[CHASSIS_IMU_ODOM_STATE_ROLL] += imu.gyro.x * dt;
    filter->x_data[CHASSIS_IMU_ODOM_STATE_PITCH] += imu.gyro.y * dt;

    for(i = 0u; i < CHASSIS_IMU_ODOM_STATE_DIM; ++i) {
        float noise = odom->config.pos_process_noise;

        if(i >= CHASSIS_IMU_ODOM_STATE_ROLL && i <= CHASSIS_IMU_ODOM_STATE_YAW) {
            noise = odom->config.angle_process_noise;
        }
        else if(i >= CHASSIS_IMU_ODOM_STATE_VX) {
            noise = odom->config.velocity_process_noise;
        }

        filter->Q_data[i * filter->Q.col + i] = noise * dt;
    }

    status = kalman.filter_predict(filter);
    if(status != kalman.OK) {
        return chassis_imu_odom_from_kalman(status);
    }

    acc_angle = chassis_imu_odom_acc_to_angle(
        imu.acc,
        odom->config.gravity,
        odom->config.acc_norm_tolerance,
        &acc_trusted);

    filter->z_data[0] = chassis.vx;
    filter->z_data[1] = chassis.vy;
    filter->z_data[2] = chassis.wz;
    filter->z_data[3] = acc_trusted ? acc_angle.x : filter->x_data[CHASSIS_IMU_ODOM_STATE_ROLL];
    filter->z_data[4] = acc_trusted ? acc_angle.y : filter->x_data[CHASSIS_IMU_ODOM_STATE_PITCH];
    filter->z_data[5] = imu.gyro.z;

    filter->H_data[0 * filter->H.col + CHASSIS_IMU_ODOM_STATE_VX] = 1.0f;
    filter->H_data[1 * filter->H.col + CHASSIS_IMU_ODOM_STATE_VY] = 1.0f;
    filter->H_data[2 * filter->H.col + CHASSIS_IMU_ODOM_STATE_WZ] = 1.0f;
    filter->H_data[3 * filter->H.col + CHASSIS_IMU_ODOM_STATE_ROLL] = 1.0f;
    filter->H_data[4 * filter->H.col + CHASSIS_IMU_ODOM_STATE_PITCH] = 1.0f;
    filter->H_data[5 * filter->H.col + CHASSIS_IMU_ODOM_STATE_WZ] = 1.0f;

    filter->R_data[0 * filter->R.col + 0] = odom->config.chassis_velocity_noise;
    filter->R_data[1 * filter->R.col + 1] = odom->config.chassis_velocity_noise;
    filter->R_data[2 * filter->R.col + 2] = odom->config.chassis_velocity_noise;
    filter->R_data[3 * filter->R.col + 3] = acc_trusted ? odom->config.imu_angle_noise : 1000.0f;
    filter->R_data[4 * filter->R.col + 4] = acc_trusted ? odom->config.imu_angle_noise : 1000.0f;
    filter->R_data[5 * filter->R.col + 5] = odom->config.imu_gyro_noise;

    status = kalman.filter_update(filter);
    if(status != kalman.OK) {
        return chassis_imu_odom_from_kalman(status);
    }

    filter->x_data[CHASSIS_IMU_ODOM_STATE_ROLL] = chassis_imu_odom_wrap_pi(filter->x_data[CHASSIS_IMU_ODOM_STATE_ROLL]);
    filter->x_data[CHASSIS_IMU_ODOM_STATE_PITCH] = chassis_imu_odom_wrap_pi(filter->x_data[CHASSIS_IMU_ODOM_STATE_PITCH]);
    filter->x_data[CHASSIS_IMU_ODOM_STATE_YAW] = chassis_imu_odom_wrap_pi(filter->x_data[CHASSIS_IMU_ODOM_STATE_YAW]);
    if(static_window) {
        filter->x_data[CHASSIS_IMU_ODOM_STATE_VX] = 0.0f;
        filter->x_data[CHASSIS_IMU_ODOM_STATE_VY] = 0.0f;
        filter->x_data[CHASSIS_IMU_ODOM_STATE_WZ] = 0.0f;
    }
    chassis_imu_odom_sync_output(odom);

    return cio.OK;
}

/**
 * @brief 获取当前三轴姿态角
 * @param odom 融合实例
 * @param angle 输出三轴角度
 * @return ChassisImuOdomErrorCode 状态码
 */
ChassisImuOdomErrorCode chassis_imu_odom_get_angle(const ChassisImuOdom* odom, Vector3* angle) {
    if(odom == NULL || angle == NULL) {
        return cio.INVALID_PARAM;
    }
    if(!odom->initialized) {
        return cio.NOT_INITIALIZE;
    }

    *angle = odom->output.angle;
    return cio.OK;
}

/**
 * @brief 获取当前三轴里程
 * @param odom 融合实例
 * @param odom_out 输出三轴里程
 * @return ChassisImuOdomErrorCode 状态码
 */
ChassisImuOdomErrorCode chassis_imu_odom_get_odom(const ChassisImuOdom* odom, Vector3* odom_out) {
    if(odom == NULL || odom_out == NULL) {
        return cio.INVALID_PARAM;
    }
    if(!odom->initialized) {
        return cio.NOT_INITIALIZE;
    }

    *odom_out = odom->output.odom;
    return cio.OK;
}

/**
 * @brief 获取完整融合输出
 * @param odom 融合实例
 * @param output 输出缓存
 * @return ChassisImuOdomErrorCode 状态码
 */
ChassisImuOdomErrorCode chassis_imu_odom_get_output(const ChassisImuOdom* odom, ChassisImuOdomOutput* output) {
    if(odom == NULL || output == NULL) {
        return cio.INVALID_PARAM;
    }
    if(!odom->initialized) {
        return cio.NOT_INITIALIZE;
    }

    *output = odom->output;
    return cio.OK;
}

/**
 * @brief 重置三轴里程状态
 * @param odom 融合实例
 * @param odom_value 新的里程值
 * @return ChassisImuOdomErrorCode 状态码
 */
ChassisImuOdomErrorCode chassis_imu_odom_reset_odom(ChassisImuOdom* odom, Vector3 odom_value) {
    if(odom == NULL) {
        return cio.INVALID_PARAM;
    }
    if(!odom->initialized) {
        return cio.NOT_INITIALIZE;
    }

    odom->filter.x_data[CHASSIS_IMU_ODOM_STATE_X] = odom_value.x;
    odom->filter.x_data[CHASSIS_IMU_ODOM_STATE_Y] = odom_value.y;
    odom->filter.x_data[CHASSIS_IMU_ODOM_STATE_Z] = odom_value.z;
    chassis_imu_odom_sync_output(odom);

    return cio.OK;
}

/**
 * @brief 将融合状态码转换为静态字符串
 * @param status 状态码
 * @return const char* 状态码说明
 */
#define X(name, str)              \
    case CHASSIS_IMU_ODOM_##name: \
        return str;
const char* chassis_imu_odom_error_code_to_str(ChassisImuOdomErrorCode status) {
    switch(status) {
        CHASSIS_IMU_ODOM_STATUS_TABLE
        default:
            return "UNKNOWN";
    }
}
#undef X

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 将矩阵清零
 */
static void chassis_imu_odom_matrix_zero(Matrix* matrix) {
    unsigned int i;

    if(matrix == NULL || matrix->pdata == NULL) {
        return;
    }

    for(i = 0u; i < matrix->row * matrix->col; ++i) {
        matrix->pdata[i] = 0.0f;
    }
}

/**
 * @brief 将矩阵置为单位阵；非方阵时只填充主对角线
 */
static void chassis_imu_odom_matrix_identity(Matrix* matrix) {
    unsigned int i;
    unsigned int j;

    chassis_imu_odom_matrix_zero(matrix);
    if(matrix == NULL || matrix->pdata == NULL) {
        return;
    }

    for(i = 0u; i < matrix->row; ++i) {
        for(j = 0u; j < matrix->col; ++j) {
            matrix->pdata[i * matrix->col + j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

/**
 * @brief 将角度归一化到 (-pi, pi]
 * @param angle 输入角度，单位 rad
 * @return float 归一化后的角度，单位 rad
 */
static float chassis_imu_odom_wrap_pi(float angle) {
    if(!isfinite(angle)) {
        return 0.0f;
    }

    angle = fmodf(angle, CHASSIS_IMU_ODOM_2PI);
    if(angle > CHASSIS_IMU_ODOM_PI) {
        angle -= CHASSIS_IMU_ODOM_2PI;
    }
    else if(angle <= -CHASSIS_IMU_ODOM_PI) {
        angle += CHASSIS_IMU_ODOM_2PI;
    }

    return angle;
}

/**
 * @brief 获取有效噪声参数；输入无效时使用默认值
 * @param value 输入配置值
 * @param fallback 默认值
 * @return float 最终使用的配置值
 */
static float chassis_imu_odom_noise(float value, float fallback) {
    return (value > CHASSIS_IMU_ODOM_EPS && isfinite(value)) ? value : fallback;
}

/**
 * @brief 获取默认融合配置
 * @return ChassisImuOdomConfig 默认配置
 */
static ChassisImuOdomConfig chassis_imu_odom_default_config(void) {
    ChassisImuOdomConfig config = {
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
    };

    return config;
}

/**
 * @brief 判断底盘和 IMU 是否处于静止窗口
 * @param odom 融合实例
 * @param chassis 底盘速度输入
 * @param imu IMU 六轴输入
 * @return true 可以执行静止约束
 * @return false 不执行静止约束
 */
static bool chassis_imu_odom_is_static(const ChassisImuOdom* odom, ChassisImuOdomChassis chassis, ChassisImuOdomImu imu) {
    if(odom == NULL) {
        return false;
    }

    return fabsf(chassis.vx) <= odom->config.chassis_linear_static_threshold
        && fabsf(chassis.vy) <= odom->config.chassis_linear_static_threshold
        && fabsf(chassis.wz) <= odom->config.chassis_angular_static_threshold
        && fabsf(imu.gyro.z) <= odom->config.imu_gyro_static_threshold;
}

/**
 * @brief 由加速度重力方向估计 roll/pitch
 * @param acc 三轴加速度
 * @param gravity 重力模长参考值
 * @param tolerance 可信模长窗口
 * @param trusted 输出加速度是否可信
 * @return Vector3 x/y 为 roll/pitch，z 固定为 0
 */
static Vector3 chassis_imu_odom_acc_to_angle(Vector3 acc, float gravity, float tolerance, bool* trusted) {
    Vector3 angle = { 0.0f, 0.0f, 0.0f };
    float norm = sqrtf(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);

    if(trusted != NULL) {
        *trusted = false;
    }
    if(norm <= CHASSIS_IMU_ODOM_EPS || fabsf(norm - gravity) > tolerance) {
        return angle;
    }

    angle.x = atan2f(acc.y, acc.z);
    angle.y = atan2f(-acc.x, sqrtf(acc.y * acc.y + acc.z * acc.z));
    angle.z = 0.0f;

    if(trusted != NULL) {
        *trusted = true;
    }
    return angle;
}

/**
 * @brief 将底层卡尔曼状态码映射到融合状态码
 * @param status 卡尔曼状态码
 * @return ChassisImuOdomErrorCode 融合状态码
 */
static ChassisImuOdomErrorCode chassis_imu_odom_from_kalman(KalmanErrorCode status) {
    if(status == kalman.OK) {
        return cio.OK;
    }
    if(status == kalman.INVALID_PARAM || status == kalman.INVALID_DIM) {
        return cio.INVALID_PARAM;
    }
    if(status == kalman.NOT_INITIALIZE) {
        return cio.NOT_INITIALIZE;
    }

    return cio.KALMAN_FAILED;
}

/**
 * @brief 将滤波器状态同步到输出缓存
 * @param odom 融合实例
 */
static void chassis_imu_odom_sync_output(ChassisImuOdom* odom) {
    KalmanFilter* filter;

    if(odom == NULL || !odom->initialized) {
        return;
    }

    filter = &odom->filter;
    odom->output.odom.x = filter->x_data[CHASSIS_IMU_ODOM_STATE_X];
    odom->output.odom.y = filter->x_data[CHASSIS_IMU_ODOM_STATE_Y];
    odom->output.odom.z = filter->x_data[CHASSIS_IMU_ODOM_STATE_Z];
    odom->output.angle.x = filter->x_data[CHASSIS_IMU_ODOM_STATE_ROLL];
    odom->output.angle.y = filter->x_data[CHASSIS_IMU_ODOM_STATE_PITCH];
    odom->output.angle.z = filter->x_data[CHASSIS_IMU_ODOM_STATE_YAW];
    odom->output.velocity.vx = filter->x_data[CHASSIS_IMU_ODOM_STATE_VX];
    odom->output.velocity.vy = filter->x_data[CHASSIS_IMU_ODOM_STATE_VY];
    odom->output.velocity.wz = filter->x_data[CHASSIS_IMU_ODOM_STATE_WZ];
}

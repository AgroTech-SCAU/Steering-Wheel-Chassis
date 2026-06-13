#ifndef _chassis_imu_odom_h_
#define _chassis_imu_odom_h_

#include <stdbool.h>

#include "kalman.h"
#include "matrix.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 底盘 + IMU 里程融合接口单例别名
 */
#define chassis_imu_odom chassis_imu_odom_interface

/**
 * @brief 底盘 + IMU 里程融合状态码表
 *
 * @param OK 操作成功
 * @param INVALID_PARAM 输入参数无效
 * @param KALMAN_FAILED 底层卡尔曼滤波运算失败
 * @param NOT_INITIALIZE 实例尚未初始化
 */
#define CHASSIS_IMU_ODOM_STATUS_TABLE     \
    X(OK, "OK")                           \
    X(INVALID_PARAM, "Invalid Parameter") \
    X(KALMAN_FAILED, "Kalman Failed")     \
    X(NOT_INITIALIZE, "Not Initialize")

#define X(name, str) CHASSIS_IMU_ODOM_##name,
/**
 * @brief 底盘 + IMU 里程融合错误码
 */
typedef enum {
    CHASSIS_IMU_ODOM_STATUS_TABLE
} ChassisImuOdomErrorCode;
#undef X

/**
 * @brief 融合滤波器状态向量索引
 *
 * 当前模型用于平面底盘里程计：
 * - X/Y 由底盘速度按 yaw 积分得到
 * - Z 为预留状态，当前没有高度观测或预测输入，因此保持初始化/重置值
 * - roll/pitch 由陀螺积分并用加速度重力方向修正
 * - yaw 由 wz 积分并用底盘/IMU z 轴角速度约束
 */
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

/**
 * @brief 底盘平面速度输入
 */
typedef struct {
    float vx; /**< 底盘 x 方向线速度，单位 m/s */
    float vy; /**< 底盘 y 方向线速度，单位 m/s */
    float wz; /**< 底盘 z 轴角速度，单位 rad/s */
} ChassisImuOdomChassis;

/**
 * @brief IMU 六轴输入
 */
typedef struct {
    Vector3 acc;  /**< 三轴加速度，单位 m/s^2 */
    Vector3 gyro; /**< 三轴角速度，单位 rad/s */
} ChassisImuOdomImu;

/**
 * @brief 融合输出
 */
typedef struct {
    Vector3 angle; /**< 三轴姿态角，x/y/z 分别为 roll/pitch/yaw，单位 rad */
    Vector3 odom;  /**< 三轴里程，x/y 单位 m；z 当前为预留高度状态 */
    ChassisImuOdomChassis velocity; /**< 融合后的底盘速度估计 */
} ChassisImuOdomOutput;

/**
 * @brief 融合滤波器噪声配置
 */
typedef struct {
    float pos_process_noise;      /**< 位置过程噪声 */
    float angle_process_noise;    /**< 姿态角过程噪声 */
    float velocity_process_noise; /**< 速度过程噪声 */
    float chassis_velocity_noise; /**< 底盘速度观测噪声 */
    float imu_angle_noise;        /**< 由加速度估计 roll/pitch 的观测噪声 */
    float imu_gyro_noise;         /**< IMU z 轴角速度观测噪声 */
    float gravity;                /**< 静止重力加速度参考值，单位 m/s^2 */
    float acc_norm_tolerance;     /**< 加速度模长可信窗口，超出时不使用加速度修正姿态 */
    float chassis_linear_static_threshold;  /**< 静止判定的底盘线速度阈值，单位 m/s */
    float chassis_angular_static_threshold; /**< 静止判定的底盘角速度阈值，单位 rad/s */
    float imu_gyro_static_threshold;        /**< 静止判定的 IMU z 轴角速度阈值，单位 rad/s */
} ChassisImuOdomConfig;

/**
 * @brief 底盘 + IMU 里程融合实例
 */
typedef struct {
    KalmanFilter filter; /**< 底层通用卡尔曼滤波器 */
    ChassisImuOdomConfig config; /**< 当前融合参数 */
    ChassisImuOdomOutput output; /**< 最近一次融合输出 */
    bool initialized; /**< true 表示实例已初始化 */
} ChassisImuOdom;

/**
 * @brief 底盘 + IMU 里程融合接口表
 */
#define X(name, str) ChassisImuOdomErrorCode name;
extern const struct ChassisImuOdomInterface {
    struct {
        CHASSIS_IMU_ODOM_STATUS_TABLE
    };
    /**
     * @brief 初始化融合实例
     * @param odom 融合实例
     * @param config 配置指针；NULL 表示使用默认配置
     * @return ChassisImuOdomErrorCode 状态码
     */
    ChassisImuOdomErrorCode (*init)(ChassisImuOdom* odom, const ChassisImuOdomConfig* config);
    /**
     * @brief 使用一帧底盘速度和 IMU 数据更新里程融合
     * @param odom 融合实例
     * @param chassis 底盘速度输入
     * @param imu IMU 六轴输入
     * @param dt 更新周期，单位 s
     * @return ChassisImuOdomErrorCode 状态码
     */
    ChassisImuOdomErrorCode (*update)(ChassisImuOdom* odom, ChassisImuOdomChassis chassis, ChassisImuOdomImu imu, float dt);
    /**
     * @brief 获取当前三轴姿态角
     * @param odom 融合实例
     * @param angle 输出三轴角度
     * @return ChassisImuOdomErrorCode 状态码
     */
    ChassisImuOdomErrorCode (*get_angle)(const ChassisImuOdom* odom, Vector3* angle);
    /**
     * @brief 获取当前三轴里程
     * @param odom 融合实例
     * @param odom_out 输出三轴里程
     * @return ChassisImuOdomErrorCode 状态码
     */
    ChassisImuOdomErrorCode (*get_odom)(const ChassisImuOdom* odom, Vector3* odom_out);
    /**
     * @brief 获取完整融合输出
     * @param odom 融合实例
     * @param output 输出缓存
     * @return ChassisImuOdomErrorCode 状态码
     */
    ChassisImuOdomErrorCode (*get_output)(const ChassisImuOdom* odom, ChassisImuOdomOutput* output);
    /**
     * @brief 重置三轴里程状态
     * @param odom 融合实例
     * @param odom_value 新的里程值
     * @return ChassisImuOdomErrorCode 状态码
     */
    ChassisImuOdomErrorCode (*reset_odom)(ChassisImuOdom* odom, Vector3 odom_value);
    /**
     * @brief 将状态码转换为静态字符串
     * @param status 状态码
     * @return const char* 状态码说明
     */
    const char* (*error_code_to_str)(ChassisImuOdomErrorCode status);
} chassis_imu_odom_interface;
#undef X

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化融合实例
 * @param odom 融合实例
 * @param config 配置指针；NULL 表示使用默认配置
 * @return ChassisImuOdomErrorCode 状态码
 */
ChassisImuOdomErrorCode chassis_imu_odom_init(ChassisImuOdom* odom, const ChassisImuOdomConfig* config);
/**
 * @brief 使用一帧底盘速度和 IMU 数据更新里程融合
 * @param odom 融合实例
 * @param chassis 底盘速度输入
 * @param imu IMU 六轴输入
 * @param dt 更新周期，单位 s
 * @return ChassisImuOdomErrorCode 状态码
 */
ChassisImuOdomErrorCode chassis_imu_odom_update(ChassisImuOdom* odom, ChassisImuOdomChassis chassis, ChassisImuOdomImu imu, float dt);
/**
 * @brief 获取当前三轴姿态角
 * @param odom 融合实例
 * @param angle 输出三轴角度
 * @return ChassisImuOdomErrorCode 状态码
 */
ChassisImuOdomErrorCode chassis_imu_odom_get_angle(const ChassisImuOdom* odom, Vector3* angle);
/**
 * @brief 获取当前三轴里程
 * @param odom 融合实例
 * @param odom_out 输出三轴里程
 * @return ChassisImuOdomErrorCode 状态码
 */
ChassisImuOdomErrorCode chassis_imu_odom_get_odom(const ChassisImuOdom* odom, Vector3* odom_out);
/**
 * @brief 获取完整融合输出
 * @param odom 融合实例
 * @param output 输出缓存
 * @return ChassisImuOdomErrorCode 状态码
 */
ChassisImuOdomErrorCode chassis_imu_odom_get_output(const ChassisImuOdom* odom, ChassisImuOdomOutput* output);
/**
 * @brief 重置三轴里程状态
 * @param odom 融合实例
 * @param odom_value 新的里程值
 * @return ChassisImuOdomErrorCode 状态码
 */
ChassisImuOdomErrorCode chassis_imu_odom_reset_odom(ChassisImuOdom* odom, Vector3 odom_value);
/**
 * @brief 将状态码转换为静态字符串
 * @param status 状态码
 * @return const char* 状态码说明
 */
const char* chassis_imu_odom_error_code_to_str(ChassisImuOdomErrorCode status);

#endif

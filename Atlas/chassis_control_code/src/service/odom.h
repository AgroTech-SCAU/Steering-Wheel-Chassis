#ifndef _odom_h_
#define _odom_h_

#include <stdbool.h>

#include "kalman/chassis_imu_odom.h"
#include "matrix.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 里程计服务接口单例别名
 */
#define odom odom_interface

/**
 * @brief 里程计服务状态码表
 *
 * @param OK 操作成功
 * @param INVALID_PARAM 输入参数无效
 * @param DEPENDENCY_MISSING 服务依赖尚未装配
 * @param FUSION_FAILED 底盘 + IMU 融合更新失败
 * @param NOT_INITIALIZED 服务尚未初始化
 * @param NOT_READY 尚未得到可用 IMU 或融合数据
 */
#define ODOM_STATUS_TABLE                            \
    X(OK, "OK")                                      \
    X(INVALID_PARAM, "Invalid Parameter")            \
    X(DEPENDENCY_MISSING, "Odom Dependency Missing") \
    X(FUSION_FAILED, "Odom Fusion Failed")           \
    X(NOT_INITIALIZED, "Odom Not Initialized")       \
    X(NOT_READY, "Odom Not Ready")

#define X(name, str) ODOM_##name,
/**
 * @brief 里程计服务状态码
 */
typedef enum {
    ODOM_STATUS_TABLE
} OdomStatus;
#undef X

/**
 * @brief 里程计服务初始化配置
 */
typedef struct {
    ChassisImuOdomConfig fusion; /**< 底盘 + IMU 融合配置 */
    float yaw_rate_scale;        /**< odom yaw rate scale，无量纲 */
    float process_period_s;      /**< odom.process() 调用周期，单位 s */
} OdomConfig;

/**
 * @brief 里程计服务只读状态快照
 */
typedef struct {
    Vector3 acc;              /**< 最近一次三轴加速度，单位 m/s^2 */
    Vector3 gyro;             /**< 最近一次原始三轴角速度，单位 rad/s */
    Vector3 gyro_bias;        /**< 最近一次 IMU 姿态算法估计的陀螺零偏，单位 rad/s */
    Vector3 gyro_corrected;   /**< 最近一次扣除零偏和补偿后的三轴角速度，单位 rad/s */
    Vector3 angle;            /**< 融合后的三轴姿态角，x/y/z 分别为 roll/pitch/yaw，单位 rad */
    Vector3 odom;             /**< 融合后的三轴里程，x/y 单位 m，z 当前为预留高度状态 */
    Vector3 velocity;         /**< 融合后的底盘速度，x/y/z 分别表示 vx/vy/wz */
    float fusion_gyro_bias_z; /**< 融合估计的 IMU z 轴残余零偏，单位 rad/s */
    bool static_window;       /**< true 表示当前应用静止零速约束 */
    bool wheel_wz_rejected;   /**< true 表示当前降低了 wheel wz 观测权重 */
    bool imu_ready;           /**< true 表示最近一次 IMU 更新成功 */
    bool fusion_ready;        /**< true 表示最近一次融合更新成功 */
    bool initialized;         /**< true 表示服务已初始化 */
} Odom;

/**
 * @brief 里程计服务接口表
 */
#define X(name, str) OdomStatus name;
extern const struct OdomInterface {
    struct {
        ODOM_STATUS_TABLE
    };
    /**
     * @brief 获取默认里程计配置
     * @return OdomConfig 默认配置
     */
    OdomConfig (*default_config)(void);
    /**
     * @brief 初始化里程计服务
     * @param config 配置指针；NULL 表示使用默认配置
     * @return OdomStatus 状态码
     */
    OdomStatus (*init)(const OdomConfig* config);
    /**
     * @brief 执行一次里程计服务流程
     *
     * 该函数统一刷新 IMU，读取底盘当前速度，并更新底盘 + IMU 融合里程
     *
     * @return OdomStatus 状态码
     */
    OdomStatus (*process)(void);
    /**
     * @brief 获取最近一次三轴加速度
     * @param acc 输出加速度
     * @return OdomStatus 状态码
     */
    OdomStatus (*get_acc)(Vector3* acc);
    /**
     * @brief 获取最近一次原始三轴角速度
     * @param gyro 输出角速度
     * @return OdomStatus 状态码
     */
    OdomStatus (*get_gyro)(Vector3* gyro);
    /**
     * @brief 获取最近一次陀螺零偏估计
     * @param gyro_bias 输出零偏
     * @return OdomStatus 状态码
     */
    OdomStatus (*get_gyro_bias)(Vector3* gyro_bias);
    /**
     * @brief 获取最近一次修正后三轴角速度
     * @param gyro_corrected 输出修正角速度
     * @return OdomStatus 状态码
     */
    OdomStatus (*get_gyro_corrected)(Vector3* gyro_corrected);
    /**
     * @brief 获取融合后的三轴姿态角
     * @param angle 输出姿态角
     * @return OdomStatus 状态码
     */
    OdomStatus (*get_angle)(Vector3* angle);
    /**
     * @brief 获取融合后的三轴里程
     * @param odom_out 输出里程
     * @return OdomStatus 状态码
     */
    OdomStatus (*get_odom)(Vector3* odom_out);
    /**
     * @brief 获取融合后的底盘速度
     * @details x/y/z 分别表示 base_link 坐标系下的 vx/vy/wz
     * @param velocity_out 输出速度
     * @return OdomStatus 状态码
     */
    OdomStatus (*get_velocity)(Vector3* velocity_out);
    /**
     * @brief 获取里程计服务只读状态快照
     * @return const Odom* 状态快照指针
     */
    const Odom* (*get_state)(void);
    /**
     * @brief 判断里程计服务是否已有可用数据
     * @return true 数据可用
     * @return false 数据尚未可用
     */
    bool (*is_ready)(void);
    /**
     * @brief 将状态码转换为静态字符串
     * @param status 状态码
     * @return const char* 状态码说明
     */
    const char* (*status_str)(OdomStatus status);
} odom_interface;
#undef X

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 获取默认里程计配置
 * @return OdomConfig 默认配置
 */
OdomConfig odom_default_config(void);
/**
 * @brief 初始化里程计服务
 * @param config 配置指针；NULL 表示使用默认配置
 * @return OdomStatus 状态码
 */
OdomStatus odom_init(const OdomConfig* config);
/**
 * @brief 执行一次里程计服务流程
 * @return OdomStatus 状态码
 */
OdomStatus odom_process(void);
/**
 * @brief 获取最近一次三轴加速度
 * @param acc 输出加速度
 * @return OdomStatus 状态码
 */
OdomStatus odom_get_acc(Vector3* acc);
/**
 * @brief 获取最近一次原始三轴角速度
 * @param gyro 输出角速度
 * @return OdomStatus 状态码
 */
OdomStatus odom_get_gyro(Vector3* gyro);
/**
 * @brief 获取最近一次陀螺零偏估计
 * @param gyro_bias 输出零偏
 * @return OdomStatus 状态码
 */
OdomStatus odom_get_gyro_bias(Vector3* gyro_bias);
/**
 * @brief 获取最近一次修正后三轴角速度
 * @param gyro_corrected 输出修正角速度
 * @return OdomStatus 状态码
 */
OdomStatus odom_get_gyro_corrected(Vector3* gyro_corrected);
/**
 * @brief 获取融合后的三轴姿态角
 * @param angle 输出姿态角
 * @return OdomStatus 状态码
 */
OdomStatus odom_get_angle(Vector3* angle);
/**
 * @brief 获取融合后的三轴里程
 * @param odom_out 输出里程
 * @return OdomStatus 状态码
 */
OdomStatus odom_get_odom(Vector3* odom_out);
/**
 * @brief 获取融合后的底盘速度
 * @details x/y/z 分别表示 base_link 坐标系下的 vx/vy/wz
 * @param velocity_out 输出速度
 * @return OdomStatus 状态码
 */
OdomStatus odom_get_velocity(Vector3* velocity_out);
/**
 * @brief 获取里程计服务只读状态快照
 * @return const Odom* 状态快照指针
 */
const Odom* odom_get_state(void);
/**
 * @brief 判断里程计服务是否已有可用数据
 * @return true 数据可用
 * @return false 数据尚未可用
 */
bool odom_is_ready(void);
/**
 * @brief 将状态码转换为静态字符串
 * @param status 状态码
 * @return const char* 状态码说明
 */
const char* odom_status_str(OdomStatus status);

#endif

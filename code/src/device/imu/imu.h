#ifndef _imu_h_
#define _imu_h_

#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief IMU 入口单例, 上层可统一调用 imu.xxx 或 imu_xxx
 */
#define imu (*imu_instance)

/**
 * @brief IMU 通用状态码表
 * @param OK 操作成功
 * @param ERROR 通用错误
 * @param INVALID_PARAM 参数无效
 * @param NO_INSTANCE 未绑定具体 IMU 实例
 * @param NOT_INITIALIZE 具体 IMU 实例尚未初始化
 * @param NOT_READY 当前暂无新数据
 * @param UNSUPPORTED 当前 IMU 不支持该通用能力
 */
#define IMU_STATUS_TABLE \
    X(OK, 0) \
    X(ERROR, 1) \
    X(INVALID_PARAM, 2) \
    X(NO_INSTANCE, 3) \
    X(NOT_INITIALIZE, 4) \
    X(NOT_READY, 5) \
    X(UNSUPPORTED, 6)

#define X(name, value) IMU_STATUS_##name = value,
/**
 * @brief IMU 通用状态码
 */
typedef enum {
    IMU_STATUS_TABLE
} ImuStatus;
#undef X

/**
 * @brief IMU 通用三轴加速度结构体
 * @param x x 轴加速度, 单位 m/s^2 或具体驱动定义单位
 * @param y y 轴加速度, 单位 m/s^2 或具体驱动定义单位
 * @param z z 轴加速度, 单位 m/s^2 或具体驱动定义单位
 */
typedef struct {
    float x;
    float y;
    float z;
} ImuAcc;

/**
 * @brief IMU 通用三轴角速度结构体
 * @param x x 轴角速度, 单位 rad/s
 * @param y y 轴角速度, 单位 rad/s
 * @param z z 轴角速度, 单位 rad/s
 */
typedef struct {
    float x;
    float y;
    float z;
} ImuGyro;

/**
 * @brief IMU 通用姿态角结构体
 * @param roll 横滚角, 单位 rad
 * @param pitch 俯仰角, 单位 rad
 * @param yaw 偏航角, 单位 rad
 */
typedef struct {
    float roll;
    float pitch;
    float yaw;
} ImuAngle;

/**
 * @brief IMU 通用接口表
 *
 * 这里只放所有 IMU 都应具备的通用能力
 * 具体硬件相关的回调、配置和扩展能力由具体 IMU 实例自己定义
 */
typedef struct {
    /**
     * @brief 初始化具体 IMU 驱动
     * @return 状态码
     */
    ImuStatus(*init)(void);
    /**
     * @brief 更新具体 IMU 的最新缓存
     * @return 状态码
     */
    ImuStatus(*update)(void);
    /**
     * @brief 获取最近一次缓存的加速度
     * @return 三轴加速度结构体
     */
    ImuAcc(*get_acc)(void);
    /**
     * @brief 获取最近一次缓存的角速度
     * @return 三轴角速度结构体
     */
    ImuGyro(*get_gyro)(void);
    /**
     * @brief 获取最近一次缓存的姿态角
     * @return 姿态角结构体
     */
    ImuAngle(*get_angle)(void);
    /**
     * @brief 将状态码转换为常量字符串
     * @param status 状态码
     * @return 状态码名称字符串
     */
    const char* (*status_str)(ImuStatus status);
} ImuInterface;

/**
 * @brief 当前绑定的具体 IMU 实例
 */
extern const ImuInterface* imu_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

ImuStatus imu_set_instance(const ImuInterface* instance);
ImuStatus imu_init(void);
ImuStatus imu_update(void);
ImuAcc imu_get_acc(void);
ImuGyro imu_get_gyro(void);
ImuAngle imu_get_angle(void);
const char* imu_status_str(ImuStatus status);

#endif

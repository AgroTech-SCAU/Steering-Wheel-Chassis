#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include "stm32_hal_can.h" // IWYU pragma: keep
#include "steer_wheel_kine.h"
#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 底盘服务接口单例别名，业务代码可通过 `chassis.xxx(...)` 调用
 */
#define chassis chassis_interface

/**
 * @brief 底盘服务状态码表
 * @param OK 操作成功
 * @param INVALID_PARAM 输入参数无效
 * @param INVALID_MODEL 底盘模型参数无效
 * @param CAN_REGISTER_FAILED CAN 回调注册失败
 * @param KINEMATICS_FAILED 舵轮运动学求解失败
 * @param NOT_INITIALIZED 底盘尚未初始化
 */
#define CHASSIS_STATUS_TABLE \
    X(OK, "OK") \
    X(INVALID_PARAM, "Invalid Parameter") \
    X(INVALID_MODEL, "Invalid Model") \
    X(CAN_REGISTER_FAILED, "CAN RX Callback Register Failed") \
    X(KINEMATICS_FAILED, "Steer Wheel Kinematics Failed") \
    X(NOT_INITIALIZED, "Chassis Not Initialized")

/**
 * @brief 四个舵轮模块的固定顺序和总线映射
 *
 * 顺序约定为：
 * - FL: Front Left
 * - FR: Front Right
 * - RR: Rear Right
 * - RL: Rear Left
 */
#define CHASSIS_MODULE_TABLE \
    X(FL, 0, 1, 0) \
    X(FR, 1, 2, 1) \
    X(RR, 2, 3, 2) \
    X(RL, 3, 4, 3)

/**
 * @brief 底盘服务错误码
 */
#define X(name, str) CHASSIS_##name,
typedef enum {
    CHASSIS_STATUS_TABLE
} ChassisErrorCode;
#undef X

/**
 * @brief 底盘舵轮模块枚举
 */
#define X(name, index, dm_id, dji_index) CHASSIS_MODULE_##name = (index),
typedef enum {
    CHASSIS_MODULE_TABLE
    CHASSIS_MODULE_COUNT = 4
} ChassisModule;
#undef X

/**
 * @brief 底盘服务初始化配置
 * @param dm_hcan 转向电机所在 CAN 总线
 * @param dji_hcan 驱动电机所在 CAN 总线
 * @param model 舵轮底盘几何模型参数
 * @param wheel_drive_ratio 轮速到驱动电机速度的传动比
 */
typedef struct {
    FDCAN_HandleTypeDef* dm_hcan;
    FDCAN_HandleTypeDef* dji_hcan;
    SteerWheelModel model;
    float wheel_drive_ratio;
} ChassisConfig;

/**
 * @brief 底盘服务实例数据
 * @param kine 舵轮运动学实例和当前控制/状态
 * @param config 当前底盘配置
 * @param brake_requested 是否已请求进入驻车刹车流程
 * @param brake_latched 驻车目标角是否已到位并进入抱死状态
 * @param initialized 是否已经完成初始化
 */
typedef struct {
    SteerWheel kine;
    ChassisConfig config;
    uint8_t brake_requested;
    uint8_t brake_latched;
    /**
     * @brief 是否等待转向到位后再输出驱动速度
     *
     * 置位时保留原有“先转向再驱动”保护逻辑；
     * 清零时转向与驱动命令会同步下发。
     */
    uint8_t steer_then_drive_enabled;
    /**
     * @brief 转向电机反馈就绪标志
     *
     * 四个转向电机均收到有效反馈后置位；
     * 主循环可据此判断冷启动是否完成
     */
    uint8_t steer_ready;
    uint8_t initialized;
} Chassis;

/**
 * @brief 底盘服务接口表
 */
#define X(name, str) ChassisErrorCode name;
extern const struct ChassisInterface {
    struct {
        CHASSIS_STATUS_TABLE
    };
    /**
     * @brief 使用默认配置初始化底盘
     * @return ChassisErrorCode 状态码
     */
    ChassisErrorCode(*init)(void);
    /**
     * @brief 使用指定配置初始化底盘
     * @param config 底盘配置
     * @return ChassisErrorCode 状态码
     */
    ChassisErrorCode(*init_with_config)(const ChassisConfig* config);
    /**
     * @brief 设置底盘目标速度
     * @param vx 底盘 x 方向目标线速度，单位 m/s
     * @param vy 底盘 y 方向目标线速度，单位 m/s
     * @param wz 底盘 z 轴目标角速度，单位 rad/s
     * @return ChassisErrorCode 状态码
     */
    ChassisErrorCode(*set_velocity)(float vx, float vy, float wz);
    /**
     * @brief 设置是否启用先转向到位再驱动模式
     * @param enabled true 启用，false 关闭
     * @return ChassisErrorCode 状态码
     */
    ChassisErrorCode(*set_steer_then_drive_enabled)(bool enabled);
    /**
     * @brief 执行一次底盘控制流程
     *
     * 该函数会更新电机反馈、求解舵轮控制量，并下发驱动命令
     *
     * @return ChassisErrorCode 状态码
     */
    ChassisErrorCode(*process)(void);
    /**
     * @brief 停止底盘运动
     *
     * 该操作会清零目标速度，释放驻车流程，并停止驱动电机
     *
     * @return ChassisErrorCode 状态码
     */
    ChassisErrorCode(*stop)(void);
    /**
     * @brief 请求底盘进入驻车刹车状态
     *
     * 底盘会先将四个舵轮转到驻车目标角，角度到位后再抱死电机
     *
     * @return ChassisErrorCode 状态码
     */
    ChassisErrorCode(*brake)(void);
    /**
     * @brief 判断底盘冷启动是否已经就绪
     *
     * 该状态只表示转向电机反馈已经正常；
     * 上层若要整机可遥控，还需要同时检查遥控链路
     *
     * @return true 底盘转向反馈已就绪
     * @return false 底盘仍在等待转向反馈
     */
    bool (*is_ready)(void);
    /**
     * @brief 获取底盘实例只读视图
     * @return const Chassis* 底盘实例指针
     */
    const Chassis* (*get_chassis)(void);
    /**
     * @brief 获取底盘当前状态只读视图
     * @return const SteerWheelState* 当前状态指针
     */
    const SteerWheelState* (*get_state)(void);
    /**
     * @brief 获取底盘当前控制量只读视图
     * @return const SteerWheelControl* 当前控制量指针
     */
    const SteerWheelControl* (*get_control)(void);
    /**
     * @brief 将底盘状态码转换为静态字符串
     * @param status 底盘状态码
     * @return const char* 状态码名称
     */
    const char* (*error_code_to_str)(ChassisErrorCode status);
} chassis_interface;
#undef X

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 使用默认配置初始化底盘
 * @return ChassisErrorCode 状态码
 */
ChassisErrorCode chassis_init(void);

/**
 * @brief 使用指定配置初始化底盘
 * @param config 底盘配置
 * @return ChassisErrorCode 状态码
 */
ChassisErrorCode chassis_init_with_config(const ChassisConfig* config);

/**
 * @brief 设置底盘目标速度
 * @param vx 底盘 x 方向目标线速度，单位 m/s
 * @param vy 底盘 y 方向目标线速度，单位 m/s
 * @param wz 底盘 z 轴目标角速度，单位 rad/s
 * @return ChassisErrorCode 状态码
 */
ChassisErrorCode chassis_set_velocity(float vx, float vy, float wz);

/**
 * @brief 设置是否启用先转向到位再驱动模式
 * @param enabled true 启用，false 关闭
 * @return ChassisErrorCode 状态码
 */
ChassisErrorCode chassis_set_steer_then_drive_enabled(bool enabled);

/**
 * @brief 执行一次底盘控制流程
 * @return ChassisErrorCode 状态码
 */
ChassisErrorCode chassis_process(void);

/**
 * @brief 停止底盘运动
 * @return ChassisErrorCode 状态码
 */
ChassisErrorCode chassis_stop(void);

/**
 * @brief 请求底盘进入驻车刹车状态
 * @return ChassisErrorCode 状态码
 */
ChassisErrorCode chassis_brake(void);

/**
 * @brief 判断底盘冷启动是否已经就绪
 *
 * 该函数用于 RGB 指示和上层状态判断；
 * 不会触发新的控制动作
 *
 * @return true 底盘转向反馈已就绪
 * @return false 底盘仍在等待转向反馈
 */
bool chassis_is_ready(void);

/**
 * @brief 获取底盘实例只读视图
 * @return const Chassis* 底盘实例指针
 */
const Chassis* chassis_get_chassis(void);

/**
 * @brief 获取底盘当前状态只读视图
 * @return const SteerWheelState* 当前状态指针
 */
const SteerWheelState* chassis_get_state(void);

/**
 * @brief 获取底盘当前控制量只读视图
 * @return const SteerWheelControl* 当前控制量指针
 */
const SteerWheelControl* chassis_get_control(void);

/**
 * @brief 将底盘状态码转换为静态字符串
 * @param status 底盘状态码
 * @return const char* 状态码名称
 */
const char* chassis_error_code_to_str(ChassisErrorCode status);

#endif

#ifndef _CHASSIS_YAW_HOLD_H_
#define _CHASSIS_YAW_HOLD_H_

/**
 * @file chassis_yaw_hold.h
 * @brief 底盘航向保持接口
 */

#include <stdbool.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 底盘航向保持配置
 *
 * 该模块只负责角度环闭环，不再承载底盘机械偏航补偿；
 * 机械补偿统一由 chassis 服务处理
 */
typedef struct {
    bool enabled;       /**< `true` 启用航向保持 */
    float kp;           /**< 角度环比例增益 */
    float kd;           /**< 角度环微分增益 */
    float v_deadband;   /**< 平移速度死区，单位 m/s */
    float wz_deadband;  /**< 用户旋转输入死区，单位 rad/s */
    float wz_limit;     /**< 角度环输出限幅，单位 rad/s */
} ChassisYawHoldConfig;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 获取默认航向保持配置
 * @return ChassisYawHoldConfig 默认配置
 */
ChassisYawHoldConfig chassis_yaw_hold_default_config(void);

/**
 * @brief 初始化航向保持模块
 * @param config 配置指针，传入 `NULL` 使用默认配置
 */
void chassis_yaw_hold_init(const ChassisYawHoldConfig* config);

/**
 * @brief 设置航向保持目标角
 * @param yaw_ref 目标 yaw，单位 rad
 */
void chassis_yaw_hold_set_target(float yaw_ref);

/**
 * @brief 关闭航向保持
 *
 * 该函数会清除激活状态和目标角，
 * 后续 `apply()` 将不再参与闭环，直到再次 `set_target()`
 */
void chassis_yaw_hold_disable(void);

/**
 * @brief 重置航向保持内部状态
 *
 * 该函数只清空误差、输出和 PD 内部状态，
 * 不会清除外部已经设置的目标角
 */
void chassis_yaw_hold_reset(void);

/**
 * @brief 应用航向保持角度环
 *
 * 当外部已经设置目标角，且用户没有主动旋转时，
 * 该函数会根据目标 yaw 输出校正后的期望 `wz`
 *
 * @param vx_cmd 底盘 x 方向速度命令，单位 m/s
 * @param vy_cmd 底盘 y 方向速度命令，单位 m/s
 * @param wz_cmd 用户原始旋转命令，单位 rad/s
 * @param yaw 当前 yaw，单位 rad
 * @param gyro_z_corrected 当前 z 轴角速度，单位 rad/s
 * @return float 叠加角度环后的目标角速度
 */
float chassis_yaw_hold_apply(float vx_cmd, float vy_cmd, float wz_cmd, float yaw, float gyro_z_corrected, float dt_s);

/**
 * @brief 查询航向保持是否处于激活状态
 * @return bool `true` 表示已锁定参考 yaw
 */
bool chassis_yaw_hold_is_active(void);

/**
 * @brief 获取当前参考 yaw
 * @return float 当前参考 yaw，单位 rad
 */
float chassis_yaw_hold_get_yaw_ref(void);

/**
 * @brief 获取最近一次角度误差
 * @return float 最近一次 yaw 误差，单位 rad
 */
float chassis_yaw_hold_get_yaw_error(void);

/**
 * @brief 获取最近一次角度环反馈输出
 * @return float 最近一次反馈输出，单位 rad/s
 */
float chassis_yaw_hold_get_fb_wz(void);

/**
 * @brief 获取最近一次最终输出
 * @return float 最近一次输出角速度，单位 rad/s
 */
float chassis_yaw_hold_get_output_wz(void);

#endif

#ifndef _ARM_H_
#define _ARM_H_

/**
 * @file arm.h
 * @brief 五自由度舵机机械臂服务层接口
 *
 * `service/arm.*` 负责把 `domain/serial_arm` 的纯数学 MDH/FK/IK 结果
 * 转换为实际舵机下发命令，并向 `app/` 层提供统一控制接口
 * 与具体实物装配相关的默认舵机零位由本层配置，不放在数学模型层
 */

#include "bus_servo/bus_servo.h"
#include "serial_arm/five_dof_arm_kine.h"

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 机械臂服务层统一入口别名
 */
#define arm arm_interface
/**
 * @brief 当前机械臂自由度数量
 */
#define ARM_DOF FIVE_DOF_ARM_DOF

/**
 * @brief 机械臂服务层状态码表
 */
#define ARM_STATUS_TABLE                            \
    X(OK, "OK")                                     \
    X(INVALID_PARAM, "Invalid Parameter")           \
    X(DEPENDENCY_MISSING, "Arm Dependency Missing") \
    X(NOT_INITIALIZED, "Arm Not Initialized")       \
    X(KINEMATICS_FAILED, "Arm Kinematics Failed")   \
    X(SERVO_FAILED, "Arm Servo Command Failed")     \
    X(OUT_OF_LIMIT, "Arm Joint Out Of Limit")       \
    X(NO_SOLUTION, "Arm IK No Solution")

/**
 * @brief 机械臂服务层状态码
 */
#define X(name, str) ARM_##name,
typedef enum {
    ARM_STATUS_TABLE
} ArmStatus;
#undef X

/**
 * @brief 关节索引与舵机 ID 映射项
 */
typedef struct {
    uint8_t joint_index;
    uint8_t servo_id;
} ArmJointServoMap;

typedef BusServoStatus (*ArmServoStopFn)(uint8_t id);
typedef BusServoStatus (*ArmServoEnableFn)(uint8_t id);

typedef BusServoStatus (*ArmServoBatchSetPosSpdFn)(const uint8_t* ids,
                                                   const float* positions,
                                                   uint8_t count,
                                                   float velocity);
typedef BusServoStatus (*ArmServoBatchUpdateFeedbackFn)(const uint8_t* ids,
                                                        uint8_t count,
                                                        BusServoFeedback* feedbacks,
                                                        uint8_t feedback_cap);

/**
 * @brief 机械臂服务初始化配置
 */
typedef struct {
    /** 通用总线舵机接口，由组合层传入具体驱动实例 */
    const BusServoInterface* servo_interface;
    /** 可选的单舵机停止/卸力函数，由具体驱动适配 */
    ArmServoStopFn stop_servo;
    /** 可选的单舵机使能扭矩函数，由具体驱动适配 */
    ArmServoEnableFn enable_servo;
    /** 可选的批量写入位置和速度函数，由具体驱动适配 */
    ArmServoBatchSetPosSpdFn batch_set_pos_spd;
    /** 可选的批量更新反馈函数，由具体驱动适配 */
    ArmServoBatchUpdateFeedbackFn batch_update_feedback;
    /** 由组合层装配好的运动学模型 */
    SerialArmModel kinematic_model;
    bool has_kinematic_model;
    /** 关节序号到舵机 ID 的映射，默认 `{0,1,2,3,4}` */
    uint8_t servo_id[ARM_DOF];
    /** 实物机械臂默认零位关节角，单位 rad */
    FiveDofArmJointArray servo_zero_joints;
    /** 默认运动速度，单位 rad/s */
    float default_speed_rad_s;
    /** 初始化后是否自动运动到实物默认零位 */
    bool auto_move_servo_zero;
} ArmConfig;

/**
 * @brief 机械臂服务只读状态快照
 */
typedef struct {
    ArmConfig config;
    FiveDofArmJointArray target_joints;
    FiveDofArmPose target_pose;
    bool target_valid;
    FiveDofArmJointArray current_joints;
    FiveDofArmPose current_pose;
    bool current_valid;
    bool initialized;
} Arm;

/**
 * @brief 机械臂服务统一接口表
 */
#define X(name, str) ArmStatus name;
extern const struct ArmInterface {
    struct {
        ARM_STATUS_TABLE
    };

    /**
     * @brief 获取默认初始化配置
     * @return ArmConfig 默认配置
     */
    ArmConfig (*default_config)(void);
    /**
     * @brief 初始化机械臂服务
     * @param config 初始化配置
     * @return ArmStatus 服务状态码
     */
    ArmStatus (*init)(const ArmConfig* config);
    /**
     * @brief 按关节数组下发整机目标
     * @param joints 目标关节角，单位 rad
     * @param speed_rad_s 目标速度，单位 rad/s
     * @return ArmStatus 服务状态码
     */
    ArmStatus (*move_joints)(const FiveDofArmJointArray* joints, float speed_rad_s);
    /**
     * @brief 控制单个关节运动
     * @param joint_index 关节索引
     * @param target_rad 目标关节角，单位 rad
     * @param speed_rad_s 目标速度，单位 rad/s
     * @return ArmStatus 服务状态码
     */
    ArmStatus (*move_joint)(uint8_t joint_index, float target_rad, float speed_rad_s);
    /**
     * @brief 运动到实物机械臂默认零位
     * @param speed_rad_s 目标速度，单位 rad/s
     * @return ArmStatus 服务状态码
     */
    ArmStatus (*move_servo_zero)(float speed_rad_s);
    /**
     * @brief 运动到数学 MDH 零位
     * @param speed_rad_s 目标速度，单位 rad/s
     * @return ArmStatus 服务状态码
     */
    ArmStatus (*move_mdh_zero)(float speed_rad_s);
    /**
     * @brief 按目标位姿求逆解并执行运动
     * @param target 目标位姿
     * @param speed_rad_s 目标速度，单位 rad/s
     * @return ArmStatus 服务状态码
     */
    ArmStatus (*move_pose)(const FiveDofArmPose* target, float speed_rad_s);
    /**
     * @brief 按目标位姿求逆解并执行运动，仅使用 5D 约束
     * @param x 目标 x，单位 m
     * @param y 目标 y，单位 m
     * @param z 目标 z，单位 m
     * @param pitch 目标 pitch，单位 rad
     * @param yaw 目标 yaw，单位 rad
     * @param speed_rad_s 目标速度，单位 rad/s
     * @return ArmStatus 服务状态码
     */
    ArmStatus (*move_pose_5d)(float x, float y, float z, float pitch, float yaw, float speed_rad_s);
    /**
     * @brief 仅修改末端目标位置并保持当前姿态
     * @param x 目标 x，单位 m
     * @param y 目标 y，单位 m
     * @param z 目标 z，单位 m
     * @param speed_rad_s 目标速度，单位 rad/s
     * @return ArmStatus 服务状态码
     */
    ArmStatus (*move_position)(float x, float y, float z, float speed_rad_s);
    ArmStatus (*move_orientation_2d)(float pitch, float yaw, float speed_rad_s);
    /**
     * @brief 运动到目标末端姿态，保持当前末端位置
     * @param roll 目标 roll，单位 rad
     * @param pitch 目标 pitch，单位 rad
     * @param yaw 目标 yaw，单位 rad
     * @param speed_rad_s 目标速度，单位 rad/s
     * @return ArmStatus 服务状态码
     */
    ArmStatus (*move_orientation)(float roll, float pitch, float yaw, float speed_rad_s);
    /**
     * @brief 停止全部舵机
     * @return ArmStatus 服务状态码
     */
    ArmStatus (*stop)(void);
    /**
     * @brief 使能全部舵机扭矩
     * @return ArmStatus 服务状态码
     */
    ArmStatus (*enable)(void);
    /**
     * @brief 计算正运动学
     * @param joints 输入关节数组，单位 rad
     * @param pose 输出位姿
     * @return ArmStatus 服务状态码
     */
    ArmStatus (*fk)(const FiveDofArmJointArray* joints, FiveDofArmPose* pose);
    /**
     * @brief 计算逆运动学
     * @param target 目标位姿
     * @param joints 输出关节数组
     * @param seed 逆解初值
     * @return ArmStatus 服务状态码
     */
    ArmStatus (*ik)(const FiveDofArmPose* target, FiveDofArmJointArray* joints, const FiveDofArmJointArray* seed);
    /**
     * @brief 刷新当前状态缓存
     * @return ArmStatus 服务状态码
     */
    ArmStatus (*refresh_current_state)(void);
    /**
     * @brief 查询服务是否已经初始化完成
     * @return bool `true` 表示服务可用
     */
    bool (*is_ready)(void);
    /**
     * @brief 获取机械臂服务只读视图
     * @return const Arm* 只读视图指针
     */
    const Arm* (*get_arm)(void);
    /**
     * @brief 获取最近一次缓存的关节角
     * @return const FiveDofArmJointArray* 关节缓存指针
     */
    const FiveDofArmJointArray* (*get_current_joints)(void);
    /**
     * @brief 获取最近一次缓存的末端位姿
     * @return const FiveDofArmPose* 位姿缓存指针
     */
    const FiveDofArmPose* (*get_current_pose)(void);
    /**
     * @brief 将服务状态码转换为字符串
     * @param status 状态码
     * @return const char* 静态字符串
     */
    const char* (*status_str)(ArmStatus status);
} arm_interface;
#undef X

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 获取默认初始化配置
 * @return ArmConfig 默认配置
 */
ArmConfig arm_default_config(void);
/**
 * @brief 初始化机械臂服务
 * @param config 初始化配置
 * @return ArmStatus 服务状态码
 */
ArmStatus arm_init(const ArmConfig* config);
/**
 * @brief 按关节数组下发整机目标
 * @param joints 目标关节角，单位 rad
 * @param speed_rad_s 目标速度，单位 rad/s
 * @return ArmStatus 服务状态码
 */
ArmStatus arm_move_joints(const FiveDofArmJointArray* joints, float speed_rad_s);
/**
 * @brief 控制单个关节运动
 * @param joint_index 关节索引
 * @param target_rad 目标关节角，单位 rad
 * @param speed_rad_s 目标速度，单位 rad/s
 * @return ArmStatus 服务状态码
 */
ArmStatus arm_move_joint(uint8_t joint_index, float target_rad, float speed_rad_s);
/**
 * @brief 运动到实物机械臂默认零位
 * @param speed_rad_s 目标速度，单位 rad/s
 * @return ArmStatus 服务状态码
 */
ArmStatus arm_move_servo_zero(float speed_rad_s);
/**
 * @brief 运动到数学 MDH 零位
 * @param speed_rad_s 目标速度，单位 rad/s
 * @return ArmStatus 服务状态码
 */
ArmStatus arm_move_mdh_zero(float speed_rad_s);
/**
 * @brief 按目标位姿求逆解并执行运动
 * @param target 目标位姿
 * @param speed_rad_s 目标速度，单位 rad/s
 * @return ArmStatus 服务状态码
 */
ArmStatus arm_move_pose(const FiveDofArmPose* target, float speed_rad_s);
ArmStatus arm_move_pose_5d(float x, float y, float z, float pitch, float yaw, float speed_rad_s);
/**
 * @brief 仅修改末端目标位置并保持当前姿态
 * @param x 目标 x，单位 m
 * @param y 目标 y，单位 m
 * @param z 目标 z，单位 m
 * @param speed_rad_s 目标速度，单位 rad/s
 * @return ArmStatus 服务状态码
 */
ArmStatus arm_move_position(float x, float y, float z, float speed_rad_s);
ArmStatus arm_move_orientation_2d(float pitch, float yaw, float speed_rad_s);

/**
 * @brief 运动到目标末端姿态，保持当前末端位置
 * @param roll 目标 roll，单位 rad
 * @param pitch 目标 pitch，单位 rad
 * @param yaw 目标 yaw，单位 rad
 * @param speed_rad_s 目标速度，单位 rad/s
 * @return ArmStatus 服务状态码
 */
ArmStatus arm_move_orientation(float roll, float pitch, float yaw, float speed_rad_s);

/**
 * @brief 停止全部舵机
 * @return ArmStatus 服务状态码
 */
ArmStatus arm_stop(void);
/**
 * @brief 使能全部舵机扭矩
 * @return ArmStatus 服务状态码
 */
ArmStatus arm_enable(void);
/**
 * @brief 计算正运动学
 * @param joints 输入关节数组，单位 rad
 * @param pose 输出位姿
 * @return ArmStatus 服务状态码
 */
ArmStatus arm_fk(const FiveDofArmJointArray* joints, FiveDofArmPose* pose);
/**
 * @brief 计算逆运动学
 * @param target 目标位姿
 * @param joints 输出关节数组
 * @param seed 逆解初值
 * @return ArmStatus 服务状态码
 */
ArmStatus arm_ik(const FiveDofArmPose* target, FiveDofArmJointArray* joints, const FiveDofArmJointArray* seed);
/**
 * @brief 查询服务是否已经初始化完成
 * @return bool `true` 表示服务可用
 */
bool arm_is_ready(void);
/**
 * @brief 获取机械臂服务只读视图
 * @return const Arm* 只读视图指针
 */
const Arm* arm_get_arm(void);
/**
 * @brief 刷新当前状态缓存
 * @return ArmStatus 服务状态码
 */
ArmStatus arm_refresh_current_state(void);
/**
 * @brief 获取最近一次缓存的关节角
 * @return const FiveDofArmJointArray* 关节缓存指针
 */
const FiveDofArmJointArray* arm_get_current_joints(void);
/**
 * @brief 获取最近一次缓存的末端位姿
 * @return const FiveDofArmPose* 位姿缓存指针
 */
const FiveDofArmPose* arm_get_current_pose(void);
/**
 * @brief 将服务状态码转换为字符串
 * @param status 状态码
 * @return const char* 静态字符串
 */
const char* arm_status_str(ArmStatus status);

#endif

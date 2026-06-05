#ifndef _serial_arm_kine_h_
#define _serial_arm_kine_h_

/**
 * @file serial_arm_kine.h
 * @brief 通用串联机械臂 MDH/FK/IK 接口
 */

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 通用串联机械臂求解模块统一入口别名
 */
#define serial_arm serial_arm_kine_instance

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef M_2PI
#define M_2PI (2.0f * M_PI)
#endif

/**
 * @brief 最大支持自由度数量
 */
#define SERIAL_ARM_MAX_DOF 8u

/**
 * @brief 逆解任务空间最大维度
 */
#define SERIAL_ARM_TASK_MAX_DIM 6u

/**
 * @brief 最多缓存逆解数量
 */
#define SERIAL_ARM_MAX_SOLUTIONS 16u

/**
 * @brief 通用串联机械臂状态码
 */
typedef enum {
    SERIAL_ARM_STATUS_SUCCESS = 0,
    SERIAL_ARM_STATUS_ERROR,
    SERIAL_ARM_STATUS_NOT_INITIALIZED,
    SERIAL_ARM_STATUS_INVALID_MODEL,
    SERIAL_ARM_STATUS_INVALID_JOINTS,
    SERIAL_ARM_STATUS_INVALID_POSE,
    SERIAL_ARM_STATUS_SINGULARITY,
    SERIAL_ARM_STATUS_OUT_OF_REACH,
    SERIAL_ARM_STATUS_NO_SOLUTION,
} SerialArmStatus;

/**
 * @brief DH 参数约定类型
 */
typedef enum {
    SERIAL_ARM_DH_STANDARD = 0,
    SERIAL_ARM_DH_MODIFIED,
} SerialArmDhConvention;

/**
 * @brief 关节类型
 */
typedef enum {
    SERIAL_ARM_JOINT_REVOLUTE = 0,
    SERIAL_ARM_JOINT_PRISMATIC,
} SerialArmJointType;

/**
 * @brief 三维点或平移向量
 */
typedef struct {
    float x;
    float y;
    float z;
} SerialArmPoint;

/**
 * @brief 四元数，顺序为 `w, x, y, z`
 */
typedef struct {
    float w;
    float x;
    float y;
    float z;
} SerialArmQuaternion;

/**
 * @brief 欧拉角，顺序为 `roll, pitch, yaw`
 */
typedef struct {
    float roll;
    float pitch;
    float yaw;
} SerialArmRPY;

/**
 * @brief 位姿，包含位置和姿态
 */
typedef struct {
    SerialArmPoint position;
    SerialArmQuaternion orientation;
} SerialArmPose;

/**
 * @brief 4x4 齐次变换矩阵
 */
typedef struct {
    float m[4][4];
} SerialArmTransform;

/**
 * @brief 单个连杆的 DH/MDH 参数
 */
typedef struct {
    SerialArmJointType type;
    float theta;
    float d;
    float a;
    float alpha;
    float q_offset;
    float q_sign;
    float q_min;
    float q_max;
} SerialArmLink;

/**
 * @brief 数值逆运动学配置
 */
typedef struct {
    float max_iterations;
    float position_tolerance;
    float orientation_tolerance;
    float step_gain;
    float damping;
    float numeric_eps;
    float position_weight;
    float orientation_weight;
} SerialArmIkConfig;

/**
 * @brief 串联机械臂模型
 */
typedef struct {
    uint8_t dof;
    SerialArmDhConvention convention;
    SerialArmLink link[SERIAL_ARM_MAX_DOF];
    SerialArmTransform base_T;
    SerialArmTransform tool_T;
    SerialArmIkConfig ik;
} SerialArmModel;

/**
 * @brief 关节数组，`q[i]` 为用户层关节变量
 */
typedef struct {
    uint8_t dof;
    float q[SERIAL_ARM_MAX_DOF];
} SerialArmJointArray;

/**
 * @brief 多组逆解结果
 */
typedef struct {
    uint8_t num_solutions;
    SerialArmJointArray solution[SERIAL_ARM_MAX_SOLUTIONS];
} SerialArmJointSolutions;

/**
 * @brief 自动推断出的 IK 任务行信息
 *
 * `row` 的含义为：
 * `0=x, 1=y, 2=z, 3=rx, 4=ry, 5=rz`
 */
typedef struct {
    uint8_t task_dim;
    uint8_t row[SERIAL_ARM_TASK_MAX_DIM];
} SerialArmTaskInfo;

/**
 * @brief 通用串联机械臂统一接口表
 */
typedef struct SerialArmKineInterface {
    /**
     * @brief 重置机械臂模型，并设置自由度与 DH 约定
     * @param model 输出模型
     * @param dof 自由度数量
     * @param convention DH 约定类型
     * @return SerialArmStatus 运动学状态码
     */
    SerialArmStatus(*model_reset)(SerialArmModel* model, uint8_t dof, SerialArmDhConvention convention);
    /**
     * @brief 设置转动关节的 DH/MDH 参数
     * @param model 目标模型
     * @param index 关节索引
     * @param theta_home 零位角，单位 rad
     * @param d 固定偏移，单位 m
     * @param a 连杆长度，单位 m
     * @param alpha 连杆扭角，单位 rad
     * @param q_offset 关节变量偏移
     * @param q_min 关节最小值
     * @param q_max 关节最大值
     * @return SerialArmStatus 运动学状态码
     */
    SerialArmStatus(*model_set_revolute)(SerialArmModel* model, uint8_t index,
        float theta_home, float d, float a, float alpha,
        float q_offset, float q_min, float q_max);
    /**
     * @brief 设置移动关节的 DH/MDH 参数
     * @param model 目标模型
     * @param index 关节索引
     * @param theta 固定角，单位 rad
     * @param d_home 零位距离，单位 m
     * @param a 连杆长度，单位 m
     * @param alpha 连杆扭角，单位 rad
     * @param q_offset 关节变量偏移
     * @param q_min 关节最小值
     * @param q_max 关节最大值
     * @return SerialArmStatus 运动学状态码
     */
    SerialArmStatus(*model_set_prismatic)(SerialArmModel* model, uint8_t index,
        float theta, float d_home, float a, float alpha,
        float q_offset, float q_min, float q_max);
    /**
     * @brief 设置关节输入方向符号
     * @param model 目标模型
     * @param index 关节索引
     * @param q_sign 输入方向，正值视为 `+1`，负值视为 `-1`
     * @return SerialArmStatus 运动学状态码
     */
    SerialArmStatus(*model_set_joint_sign)(SerialArmModel* model, uint8_t index, float q_sign);
    /**
     * @brief 载入模型并初始化求解器内部状态
     * @param model 已完成配置的模型
     * @return SerialArmStatus 运动学状态码
     */
    SerialArmStatus(*init)(const SerialArmModel* model);
    /**
     * @brief 获取当前 IK 任务行配置
     * @param info 输出任务信息
     * @return SerialArmStatus 运动学状态码
     */
    SerialArmStatus(*get_task_info)(SerialArmTaskInfo* info);
    /**
     * @brief 显式设置 IK 任务行配置
     * @param info 目标任务信息
     * @return SerialArmStatus 运动学状态码
     */
    SerialArmStatus(*set_task_info)(const SerialArmTaskInfo* info);
    /**
     * @brief 恢复为按自由度自动推断的 IK 任务行配置
     * @return SerialArmStatus 运动学状态码
     */
    SerialArmStatus(*reset_task_info)(void);
    /**
     * @brief 获取任务行索引对应的名称
     * @param row 任务行索引
     * @return const char* 静态任务行名称
     */
    const char* (*task_row_name)(uint8_t row);
    /**
     * @brief 将状态码转换为静态字符串
     * @param status 状态码
     * @return const char* 静态状态字符串
     */
    const char* (*status_str)(SerialArmStatus status);
    /**
     * @brief 计算正运动学并输出末端位姿
     * @param joints 输入关节数组
     * @param pose 输出位姿
     * @return SerialArmStatus 运动学状态码
     */
    SerialArmStatus(*fk)(const SerialArmJointArray* joints, SerialArmPose* pose);
    /**
     * @brief 计算正运动学并输出齐次变换矩阵
     * @param joints 输入关节数组
     * @param T 输出 4x4 齐次变换矩阵
     * @return SerialArmStatus 运动学状态码
     */
    SerialArmStatus(*fk_matrix)(const SerialArmJointArray* joints, SerialArmTransform* T);
    /**
     * @brief 按当前任务行配置求解逆运动学
     * @param target 目标位姿
     * @param joints 输出关节解
     * @param seed 迭代初值
     * @return SerialArmStatus 运动学状态码
     */
    SerialArmStatus(*ik)(const SerialArmPose* target, SerialArmJointArray* joints,
        const SerialArmJointArray* seed);
    /**
     * @brief 搜索多组可行逆解
     * @param target 目标位姿
     * @param solutions 输出逆解集合
     * @return SerialArmStatus 运动学状态码
     */
    SerialArmStatus(*all_ik)(const SerialArmPose* target, SerialArmJointSolutions* solutions);
    /**
     * @brief 从逆解集合中选取指定解
     * @param solutions 逆解集合
     * @param index 目标解索引
     * @return SerialArmJointArray* 指向目标解的指针；无效时返回 `NULL`
     */
    SerialArmJointArray* (*solution_select)(SerialArmJointSolutions* solutions, uint8_t index);
    /**
     * @brief 将 RPY 欧拉角转换为四元数
     * @param rpy 输入 RPY 欧拉角，单位 rad
     * @param quat 输出四元数
     * @return SerialArmStatus 运动学状态码
     */
    SerialArmStatus(*rpy_to_quat)(const SerialArmRPY rpy, SerialArmQuaternion* quat);
    /**
     * @brief 将四元数转换为 RPY 欧拉角
     * @param quat 输入四元数
     * @param rpy 输出 RPY 欧拉角，单位 rad
     * @return SerialArmStatus 运动学状态码
     */
    SerialArmStatus(*quat_to_rpy)(const SerialArmQuaternion quat, SerialArmRPY* rpy);
    /**
     * @brief 由位置和 RPY 欧拉角构造目标位姿
     * @param x 目标 x，单位 m
     * @param y 目标 y，单位 m
     * @param z 目标 z，单位 m
     * @param roll 目标 roll，单位 rad
     * @param pitch 目标 pitch，单位 rad
     * @param yaw 目标 yaw，单位 rad
     * @param pose 输出位姿
     * @return SerialArmStatus 运动学状态码
     */
    SerialArmStatus(*pose_from_xyz_rpy)(float x, float y, float z,
        float roll, float pitch, float yaw, SerialArmPose* pose);
} SerialArmKineInterface;

/**
 * @brief 通用串联机械臂接口实例
 */
extern const SerialArmKineInterface serial_arm_kine_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

SerialArmStatus s_serial_arm_model_reset(SerialArmModel* model, uint8_t dof, SerialArmDhConvention convention);
SerialArmStatus s_serial_arm_model_set_revolute(SerialArmModel* model, uint8_t index,
    float theta_home, float d, float a, float alpha,
    float q_offset, float q_min, float q_max);
SerialArmStatus s_serial_arm_model_set_prismatic(SerialArmModel* model, uint8_t index,
    float theta, float d_home, float a, float alpha,
    float q_offset, float q_min, float q_max);
SerialArmStatus s_serial_arm_model_set_joint_sign(SerialArmModel* model, uint8_t index, float q_sign);
SerialArmStatus s_serial_arm_init(const SerialArmModel* model);
SerialArmStatus s_serial_arm_get_task_info(SerialArmTaskInfo* info);
/**
 * @brief 显式设置当前 IK 任务行配置
 * @param info 目标任务信息
 * @return SerialArmStatus 运动学状态码
 */
SerialArmStatus s_serial_arm_set_task_info(const SerialArmTaskInfo* info);
/**
 * @brief 恢复为按当前自由度自动推断的 IK 任务行配置
 * @return SerialArmStatus 运动学状态码
 */
SerialArmStatus s_serial_arm_reset_task_info(void);
const char* s_serial_arm_task_row_name(uint8_t row);
const char* s_serial_arm_status_str(SerialArmStatus status);
SerialArmStatus s_serial_arm_fk(const SerialArmJointArray* joints, SerialArmPose* pose);
SerialArmStatus s_serial_arm_fk_matrix(const SerialArmJointArray* joints, SerialArmTransform* T);
SerialArmStatus s_serial_arm_ik(const SerialArmPose* target, SerialArmJointArray* joints,
    const SerialArmJointArray* seed);
SerialArmStatus s_serial_arm_all_ik(const SerialArmPose* target, SerialArmJointSolutions* solutions);
SerialArmJointArray* s_serial_arm_solution_select(SerialArmJointSolutions* solutions, uint8_t index);
SerialArmStatus s_serial_arm_rpy_to_quat(const SerialArmRPY rpy, SerialArmQuaternion* quat);
SerialArmStatus s_serial_arm_quat_to_rpy(const SerialArmQuaternion quat, SerialArmRPY* rpy);
SerialArmStatus s_serial_arm_pose_from_xyz_rpy(float x, float y, float z,
    float roll, float pitch, float yaw, SerialArmPose* pose);

#endif

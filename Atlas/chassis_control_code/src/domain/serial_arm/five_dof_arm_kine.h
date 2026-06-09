#ifndef _five_dof_arm_kine_h_
#define _five_dof_arm_kine_h_

/**
 * @file five_dof_arm_kine.h
 * @brief 五自由度机械臂纯数学 MDH 模型接口
 *
 * 本文件只描述五自由度机械臂的数学模型：
 * 连杆几何、关节方向、关节限位、数学零位以及 FK/IK 接口
 * 与具体实物装配相关的默认舵机零位、标定偏置等信息不应放在这里
 */

#include "serial_arm_kine.h"

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 五自由度机械臂纯数学模型统一入口别名
 */
#define five_dof_arm five_dof_arm_kine_instance

/**
 * @brief 五自由度机械臂的自由度数量
 */
#define FIVE_DOF_ARM_DOF 5u

/**
 * @brief 单关节对称限位，单位 rad
 */
#define FIVE_DOF_ARM_Q_LIMIT_RAD (3.14159265358979323846f * 3.0f / 4.0f)

/**
 * @brief 五自由度机械臂关节索引
 */
typedef enum {
    FIVE_DOF_ARM_JOINT_BASE_YAW = 0,
    FIVE_DOF_ARM_JOINT_SHOULDER = 1,
    FIVE_DOF_ARM_JOINT_ELBOW = 2,
    FIVE_DOF_ARM_JOINT_WRIST = 3,
    FIVE_DOF_ARM_JOINT_END = 4,
} FiveDofArmJointIndex;

/**
 * @brief 五自由度机械臂领域层状态码别名
 */
typedef SerialArmStatus FiveDofArmStatus;
typedef SerialArmJointArray FiveDofArmJointArray;
typedef SerialArmJointSolutions FiveDofArmJointSolutions;
typedef SerialArmPose FiveDofArmPose;
typedef SerialArmTransform FiveDofArmTransform;

/**
 * @brief 五自由度机械臂纯数学模型统一接口
 */
typedef struct {
    /**
     * @brief 构建五自由度机械臂的纯数学 MDH 模型
     * @param model 输出模型
     * @return FiveDofArmStatus 领域层状态码
     */
    FiveDofArmStatus(*build_model)(SerialArmModel* model);
    /**
     * @brief 构建模型并初始化通用串联机械臂求解器
     * @return FiveDofArmStatus 领域层状态码
     */
    FiveDofArmStatus(*init)(void);
    /**
     * @brief 获取当前只读模型指针
     * @return const SerialArmModel* 模型指针
     */
    const SerialArmModel* (*get_model)(void);
    /**
     * @brief 获取数学 MDH 零位 `[0,0,0,0,0]`
     * @param joints 输出关节数组
     * @return FiveDofArmStatus 领域层状态码
     */
    FiveDofArmStatus(*get_mdh_zero)(FiveDofArmJointArray* joints);
    /**
     * @brief 计算正运动学并输出位姿
     * @param joints 输入关节数组，单位 rad
     * @param pose 输出位姿
     * @return FiveDofArmStatus 领域层状态码
     */
    FiveDofArmStatus(*fk)(const FiveDofArmJointArray* joints, FiveDofArmPose* pose);
    /**
     * @brief 计算正运动学并输出齐次变换矩阵
     * @param joints 输入关节数组，单位 rad
     * @param T 输出 4x4 齐次变换矩阵
     * @return FiveDofArmStatus 领域层状态码
     */
    FiveDofArmStatus(*fk_matrix)(const FiveDofArmJointArray* joints, FiveDofArmTransform* T);
    /**
     * @brief 计算单组逆运动学解
     * @param target 目标位姿
     * @param joints 输出关节数组
     * @param seed 逆解初值
     * @return FiveDofArmStatus 领域层状态码
     */
    FiveDofArmStatus(*ik)(const FiveDofArmPose* target, FiveDofArmJointArray* joints,
        const FiveDofArmJointArray* seed);
    /**
     * @brief 搜索多组可行逆解
     * @param target 目标位姿
     * @param solutions 输出逆解集合
     * @return FiveDofArmStatus 领域层状态码
     */
    FiveDofArmStatus(*all_ik)(const FiveDofArmPose* target, FiveDofArmJointSolutions* solutions);
    /**
     * @brief 将状态码转换为字符串
     * @param status 状态码
     * @return const char* 静态字符串
     */
    const char* (*status_str)(FiveDofArmStatus status);
} FiveDofArmKineInterface;

/**
 * @brief 五自由度机械臂纯数学模型接口实例
 */
extern const FiveDofArmKineInterface five_dof_arm_kine_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 构建五自由度机械臂的纯数学 MDH 模型
 * @param model 输出模型
 * @return FiveDofArmStatus 领域层状态码
 */
FiveDofArmStatus five_dof_arm_build_model(SerialArmModel* model);
/**
 * @brief 构建模型并初始化通用串联机械臂求解器
 * @return FiveDofArmStatus 领域层状态码
 */
FiveDofArmStatus five_dof_arm_init(void);
/**
 * @brief 获取当前只读模型指针
 * @return const SerialArmModel* 模型指针
 */
const SerialArmModel* five_dof_arm_get_model(void);
/**
 * @brief 获取数学 MDH 零位 `[0,0,0,0,0]`
 * @param joints 输出关节数组
 * @return FiveDofArmStatus 领域层状态码
 */
FiveDofArmStatus five_dof_arm_get_mdh_zero(FiveDofArmJointArray* joints);
/**
 * @brief 计算正运动学并输出位姿
 * @param joints 输入关节数组，单位 rad
 * @param pose 输出位姿
 * @return FiveDofArmStatus 领域层状态码
 */
FiveDofArmStatus five_dof_arm_fk(const FiveDofArmJointArray* joints, FiveDofArmPose* pose);
/**
 * @brief 计算正运动学并输出齐次变换矩阵
 * @param joints 输入关节数组，单位 rad
 * @param T 输出 4x4 齐次变换矩阵
 * @return FiveDofArmStatus 领域层状态码
 */
FiveDofArmStatus five_dof_arm_fk_matrix(const FiveDofArmJointArray* joints, FiveDofArmTransform* T);
/**
 * @brief 计算单组逆运动学解
 * @param target 目标位姿
 * @param joints 输出关节数组
 * @param seed 逆解初值
 * @return FiveDofArmStatus 领域层状态码
 */
FiveDofArmStatus five_dof_arm_ik(const FiveDofArmPose* target, FiveDofArmJointArray* joints,
    const FiveDofArmJointArray* seed);
/**
 * @brief 搜索多组可行逆解
 * @param target 目标位姿
 * @param solutions 输出逆解集合
 * @return FiveDofArmStatus 领域层状态码
 */
FiveDofArmStatus five_dof_arm_all_ik(const FiveDofArmPose* target, FiveDofArmJointSolutions* solutions);
/**
 * @brief 将状态码转换为字符串
 * @param status 状态码
 * @return const char* 静态字符串
 */
const char* five_dof_arm_status_str(FiveDofArmStatus status);

#endif

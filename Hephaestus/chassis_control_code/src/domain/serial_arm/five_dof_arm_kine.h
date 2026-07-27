#ifndef _five_dof_arm_kine_h_
#define _five_dof_arm_kine_h_

/**
 * @file five_dof_arm_kine.h
 * @brief 五自由度串联机械臂领域层封装接口
 *
 * 本文件只描述“五自由度串联机械臂”这一算法形状：
 * 5 个转动关节、关节数组类型别名、FK/IK 转发接口，以及当前已加载模型的只读访问
 *
 * 具体机器人相关的连杆几何、base/tool 外参、舵机 ID、实物零位、标定偏置、
 * 关节方向和关节限位等真实世界配置，必须由 service/assemble 层构造后通过
 * `five_dof_arm_init_with_model()` 注入，不能写死在 domain 层
 */

#include "serial_arm_kine.h"

#include <stdbool.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 五自由度机械臂领域层统一入口别名
 */
#define five_dof_arm five_dof_arm_kine_instance

/**
 * @brief 五自由度机械臂自由度数量
 */
#define FIVE_DOF_ARM_DOF 5u
/**
 * @brief 兼容旧代码的单关节对称限位常量，单位 rad
 *
 * 当前真实关节限位由外部注入的 `SerialArmModel` 决定，本常量不再参与模型构造
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
 * @brief 五自由度机械臂领域层类型别名
 */
typedef SerialArmStatus FiveDofArmStatus;
typedef SerialArmJointArray FiveDofArmJointArray;
typedef SerialArmJointSolutions FiveDofArmJointSolutions;
typedef SerialArmPose FiveDofArmPose;
typedef SerialArmTransform FiveDofArmTransform;

/**
 * @brief 五自由度机械臂领域层统一接口
 */
typedef struct {
    /**
     * @brief 构造一个默认的五转动关节空模型
     *
     * 该模型只用于算法层自检或兜底初始化，不包含任何真实机械臂几何
     * 实车请使用 `init_with_model()` 加载 service 层装配出的模型
     *
     * @param model 输出模型
     * @return FiveDofArmStatus 领域层状态码
     */
    FiveDofArmStatus (*build_model)(SerialArmModel* model);
    /**
     * @brief 使用默认空模型初始化求解器
     *
     * 该入口保留给纯算法测试使用；实车 service 初始化不应依赖此入口
     *
     * @return FiveDofArmStatus 领域层状态码
     */
    FiveDofArmStatus (*init)(void);
    /**
     * @brief 使用外部装配好的五自由度模型初始化求解器
     *
     * @param model service/assemble 层构造的模型
     * @return FiveDofArmStatus 领域层状态码
     */
    FiveDofArmStatus (*init_with_model)(const SerialArmModel* model);
    /**
     * @brief 获取当前已加载的只读模型
     *
     * @return const SerialArmModel* 模型指针；未初始化时返回 NULL
     */
    const SerialArmModel* (*get_model)(void);
    /**
     * @brief 获取数学零位 `[0, 0, 0, 0, 0]`
     *
     * @param joints 输出关节数组
     * @return FiveDofArmStatus 领域层状态码
     */
    FiveDofArmStatus (*get_mdh_zero)(FiveDofArmJointArray* joints);
    /**
     * @brief 计算正运动学并输出末端位姿
     *
     * @param joints 输入关节数组，单位 rad
     * @param pose 输出位姿
     * @return FiveDofArmStatus 领域层状态码
     */
    FiveDofArmStatus (*fk)(const FiveDofArmJointArray* joints, FiveDofArmPose* pose);
    /**
     * @brief 计算正运动学并输出齐次变换矩阵
     *
     * @param joints 输入关节数组，单位 rad
     * @param T 输出 4x4 齐次变换矩阵
     * @return FiveDofArmStatus 领域层状态码
     */
    FiveDofArmStatus (*fk_matrix)(const FiveDofArmJointArray* joints, FiveDofArmTransform* T);
    /**
     * @brief 按当前任务约束计算单组逆运动学解
     *
     * @param target 目标位姿
     * @param joints 输出关节数组
     * @param seed IK 初值
     * @return FiveDofArmStatus 领域层状态码
     */
    FiveDofArmStatus (*ik)(const FiveDofArmPose* target, FiveDofArmJointArray* joints,
                           const FiveDofArmJointArray* seed);
    /**
     * @brief 搜索多组可行逆运动学解
     *
     * @param target 目标位姿
     * @param solutions 输出解集
     * @return FiveDofArmStatus 领域层状态码
     */
    FiveDofArmStatus (*all_ik)(const FiveDofArmPose* target, FiveDofArmJointSolutions* solutions);
    /**
     * @brief 将状态码转换为静态字符串
     *
     * @param status 状态码
     * @return const char* 状态码字符串
     */
    const char* (*status_str)(FiveDofArmStatus status);
} FiveDofArmKineInterface;

/**
 * @brief 五自由度机械臂领域层接口实例
 */
extern const FiveDofArmKineInterface five_dof_arm_kine_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 构造一个默认的五转动关节空模型
 * @param model 输出模型
 * @return FiveDofArmStatus 领域层状态码
 */
FiveDofArmStatus five_dof_arm_build_model(SerialArmModel* model);
/**
 * @brief 使用默认空模型初始化求解器
 * @return FiveDofArmStatus 领域层状态码
 */
FiveDofArmStatus five_dof_arm_init(void);
/**
 * @brief 使用外部装配好的五自由度模型初始化求解器
 * @param model service/assemble 层构造的模型
 * @return FiveDofArmStatus 领域层状态码
 */
FiveDofArmStatus five_dof_arm_init_with_model(const SerialArmModel* model);
/**
 * @brief 获取当前已加载的只读模型
 * @return const SerialArmModel* 模型指针；未初始化时返回 NULL
 */
const SerialArmModel* five_dof_arm_get_model(void);
/**
 * @brief 获取数学零位 `[0, 0, 0, 0, 0]`
 * @param joints 输出关节数组
 * @return FiveDofArmStatus 领域层状态码
 */
FiveDofArmStatus five_dof_arm_get_mdh_zero(FiveDofArmJointArray* joints);
/**
 * @brief 计算正运动学并输出末端位姿
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
 * @brief 按当前任务约束计算单组逆运动学解
 * @param target 目标位姿
 * @param joints 输出关节数组
 * @param seed IK 初值
 * @return FiveDofArmStatus 领域层状态码
 */
FiveDofArmStatus five_dof_arm_ik(const FiveDofArmPose* target, FiveDofArmJointArray* joints,
                                 const FiveDofArmJointArray* seed);
/**
 * @brief 搜索多组可行逆运动学解
 * @param target 目标位姿
 * @param solutions 输出解集
 * @return FiveDofArmStatus 领域层状态码
 */
FiveDofArmStatus five_dof_arm_all_ik(const FiveDofArmPose* target, FiveDofArmJointSolutions* solutions);
/**
 * @brief 将状态码转换为静态字符串
 * @param status 状态码
 * @return const char* 状态码字符串
 */
const char* five_dof_arm_status_str(FiveDofArmStatus status);

#endif

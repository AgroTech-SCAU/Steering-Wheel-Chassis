#include "serial_arm/five_dof_arm_kine.h"

#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 当前已加载的五自由度模型缓存
 */
static SerialArmModel s_five_dof_model;
/**
 * @brief 当前模型是否已经成功加载到通用串联机械臂求解器
 */
static bool s_five_dof_initialized = false;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 将变换矩阵置为单位阵
 * @param T 目标变换矩阵
 */
static void s_tf_identity(SerialArmTransform* T);
/**
 * @brief 设置单个转动关节并附带方向符号
 * @param model 目标模型
 * @param index 关节索引
 * @param theta 固定 theta，单位 rad
 * @param d 固定 d，单位 m
 * @param a 固定 a，单位 m
 * @param alpha 固定 alpha，单位 rad
 * @param offset 关节偏移，单位 rad
 * @param sign 关节方向符号
 * @return FiveDofArmStatus 领域层状态码
 */
static FiveDofArmStatus s_set_revolute_checked(SerialArmModel* model, uint8_t index,
                                               float theta, float d, float a, float alpha, float offset, float sign);
/**
 * @brief 用给定数组填充五自由度关节结构体
 * @param joints 输出关节数组
 * @param q 输入角度数组，单位 rad
 */
static void s_fill_joint_array(FiveDofArmJointArray* joints, const float q[FIVE_DOF_ARM_DOF]);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 五自由度机械臂领域层接口实例
 */
const FiveDofArmKineInterface five_dof_arm_kine_instance = {
    .build_model = five_dof_arm_build_model,
    .init = five_dof_arm_init,
    .init_with_model = five_dof_arm_init_with_model,
    .get_model = five_dof_arm_get_model,
    .get_mdh_zero = five_dof_arm_get_mdh_zero,
    .fk = five_dof_arm_fk,
    .fk_matrix = five_dof_arm_fk_matrix,
    .ik = five_dof_arm_ik,
    .all_ik = five_dof_arm_all_ik,
    .status_str = five_dof_arm_status_str,
};

/**
 * @brief 构造一个默认的五转动关节空模型
 * @param model 输出模型
 * @return FiveDofArmStatus 领域层状态码
 */
FiveDofArmStatus five_dof_arm_build_model(SerialArmModel* model) {
    FiveDofArmStatus ret;

    if(model == NULL)
        return SERIAL_ARM_STATUS_ERROR;

    ret = serial_arm.model_reset(model, FIVE_DOF_ARM_DOF, SERIAL_ARM_DH_MODIFIED);
    if(ret != SERIAL_ARM_STATUS_SUCCESS)
        return ret;

    s_tf_identity(&model->base_T);
    s_tf_identity(&model->tool_T);

    for(uint8_t i = 0u; i < FIVE_DOF_ARM_DOF; i++) {
        ret = s_set_revolute_checked(model, i, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
        if(ret != SERIAL_ARM_STATUS_SUCCESS)
            return ret;
    }

    model->ik.max_iterations = 250.0f;
    model->ik.position_tolerance = 1e-4f;
    model->ik.orientation_tolerance = 2e-3f;
    model->ik.step_gain = 0.45f;
    model->ik.damping = 2e-3f;
    model->ik.numeric_eps = 1e-5f;
    model->ik.position_weight = 1.0f;
    model->ik.orientation_weight = 0.25f;

    return SERIAL_ARM_STATUS_SUCCESS;
}

/**
 * @brief 使用默认空模型初始化求解器
 * @return FiveDofArmStatus 领域层状态码
 */
FiveDofArmStatus five_dof_arm_init(void) {
    FiveDofArmStatus ret = five_dof_arm_build_model(&s_five_dof_model);
    if(ret != SERIAL_ARM_STATUS_SUCCESS) {
        s_five_dof_initialized = false;
        return ret;
    }

    ret = serial_arm.init(&s_five_dof_model);
    s_five_dof_initialized = (ret == SERIAL_ARM_STATUS_SUCCESS);
    return ret;
}

/**
 * @brief 使用外部装配好的五自由度模型初始化求解器
 *
 * @param model service/assemble 层构造的模型
 * @return FiveDofArmStatus 领域层状态码
 */
FiveDofArmStatus five_dof_arm_init_with_model(const SerialArmModel* model) {
    FiveDofArmStatus ret;

    if(model == NULL || model->dof != FIVE_DOF_ARM_DOF) {
        s_five_dof_initialized = false;
        return SERIAL_ARM_STATUS_INVALID_MODEL;
    }

    ret = serial_arm.init(model);
    if(ret != SERIAL_ARM_STATUS_SUCCESS) {
        s_five_dof_initialized = false;
        return ret;
    }

    s_five_dof_model = *model;
    s_five_dof_initialized = true;
    return SERIAL_ARM_STATUS_SUCCESS;
}

/**
 * @brief 获取当前已加载的只读模型
 * @return const SerialArmModel* 模型指针；未初始化时返回 NULL
 */
const SerialArmModel* five_dof_arm_get_model(void) {
    return s_five_dof_initialized ? &s_five_dof_model : NULL;
}

/**
 * @brief 获取数学零位 `[0, 0, 0, 0, 0]`
 * @param joints 输出关节数组
 * @return FiveDofArmStatus 领域层状态码
 */
FiveDofArmStatus five_dof_arm_get_mdh_zero(FiveDofArmJointArray* joints) {
    static const float q[FIVE_DOF_ARM_DOF] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

    if(joints == NULL)
        return SERIAL_ARM_STATUS_ERROR;
    s_fill_joint_array(joints, q);
    return SERIAL_ARM_STATUS_SUCCESS;
}

/**
 * @brief 计算正运动学并输出末端位姿
 * @param joints 输入关节数组，单位 rad
 * @param pose 输出位姿
 * @return FiveDofArmStatus 领域层状态码
 */
FiveDofArmStatus five_dof_arm_fk(const FiveDofArmJointArray* joints, FiveDofArmPose* pose) {
    if(!s_five_dof_initialized)
        return SERIAL_ARM_STATUS_NOT_INITIALIZED;
    return serial_arm.fk(joints, pose);
}

/**
 * @brief 计算正运动学并输出齐次变换矩阵
 * @param joints 输入关节数组，单位 rad
 * @param T 输出 4x4 齐次变换矩阵
 * @return FiveDofArmStatus 领域层状态码
 */
FiveDofArmStatus five_dof_arm_fk_matrix(const FiveDofArmJointArray* joints, FiveDofArmTransform* T) {
    if(!s_five_dof_initialized)
        return SERIAL_ARM_STATUS_NOT_INITIALIZED;
    return serial_arm.fk_matrix(joints, T);
}

/**
 * @brief 按当前任务约束计算单组逆运动学解
 * @param target 目标位姿
 * @param joints 输出关节数组
 * @param seed IK 初值
 * @return FiveDofArmStatus 领域层状态码
 */
FiveDofArmStatus five_dof_arm_ik(const FiveDofArmPose* target, FiveDofArmJointArray* joints,
                                 const FiveDofArmJointArray* seed) {
    if(!s_five_dof_initialized)
        return SERIAL_ARM_STATUS_NOT_INITIALIZED;
    return serial_arm.ik(target, joints, seed);
}

/**
 * @brief 搜索多组可行逆运动学解
 * @param target 目标位姿
 * @param solutions 输出解集
 * @return FiveDofArmStatus 领域层状态码
 */
FiveDofArmStatus five_dof_arm_all_ik(const FiveDofArmPose* target, FiveDofArmJointSolutions* solutions) {
    if(!s_five_dof_initialized)
        return SERIAL_ARM_STATUS_NOT_INITIALIZED;
    return serial_arm.all_ik(target, solutions);
}

/**
 * @brief 将状态码转换为静态字符串
 * @param status 状态码
 * @return const char* 状态码字符串
 */
const char* five_dof_arm_status_str(FiveDofArmStatus status) {
    return serial_arm.status_str(status);
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 将变换矩阵置为单位阵
 * @param T 目标变换矩阵
 */
static void s_tf_identity(SerialArmTransform* T) {
    memset(T, 0, sizeof(*T));
    T->m[0][0] = 1.0f;
    T->m[1][1] = 1.0f;
    T->m[2][2] = 1.0f;
    T->m[3][3] = 1.0f;
}

/**
 * @brief 设置单个转动关节并附带方向符号
 *
 * 默认空模型使用统一 `[0, 2pi]` 限位；真实限位由外部注入模型决定
 *
 * @param model 目标模型
 * @param index 关节索引
 * @param theta 固定 theta，单位 rad
 * @param d 固定 d，单位 m
 * @param a 固定 a，单位 m
 * @param alpha 固定 alpha，单位 rad
 * @param offset 关节偏移，单位 rad
 * @param sign 关节方向符号
 * @return FiveDofArmStatus 领域层状态码
 */
static FiveDofArmStatus s_set_revolute_checked(SerialArmModel* model, uint8_t index,
                                               float theta, float d, float a, float alpha, float offset, float sign) {
    FiveDofArmStatus ret = serial_arm.model_set_revolute(model, index,
                                                         theta, d, a, alpha,
                                                         offset,
                                                         0.0f,
                                                         2.0f * M_PI);
    if(ret != SERIAL_ARM_STATUS_SUCCESS)
        return ret;
    return serial_arm.model_set_joint_sign(model, index, sign);
}

/**
 * @brief 用给定数组填充五自由度关节结构体
 * @param joints 输出关节数组
 * @param q 输入角度数组，单位 rad
 */
static void s_fill_joint_array(FiveDofArmJointArray* joints, const float q[FIVE_DOF_ARM_DOF]) {
    memset(joints, 0, sizeof(*joints));
    joints->dof = FIVE_DOF_ARM_DOF;
    for(uint8_t i = 0u; i < FIVE_DOF_ARM_DOF; i++) {
        joints->q[i] = q[i];
    }
}

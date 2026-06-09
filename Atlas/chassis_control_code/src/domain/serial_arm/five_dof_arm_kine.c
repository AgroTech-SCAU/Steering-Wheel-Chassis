/**
 * @file five_dof_arm_kine.c
 * @brief 五自由度机械臂纯数学 MDH 模型实现
 */
#include "serial_arm/five_dof_arm_kine.h"

#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 数学模型基座 x 偏移，单位 m
 */
static const float S_BASE_X = -0.01000f - 0.01391f;
/**
 * @brief 数学模型基座 y 偏移，单位 m
 */
static const float S_BASE_Y = 0.00390f;
/**
 * @brief 第一关节基座高度，单位 m
 */
static const float S_H0 = 0.04800f + 0.06028f;
/**
 * @brief 第一段主连杆长度，单位 m
 */
static const float S_L1_LEN = 0.18000003f;
/**
 * @brief 第二段主连杆长度，单位 m
 */
static const float S_L2_LEN = 0.14710f;
/**
 * @brief 第三段主连杆长度，单位 m
 */
static const float S_L3_LEN = 0.14000132f;
/**
 * @brief 末端工具坐标系 x 向偏移，单位 m
 */
static const float S_TCP_LEN = 0.0f;

/**
 * @brief 当前五自由度机械臂纯数学模型缓存
 */
static SerialArmModel s_five_dof_model;
/**
 * @brief 当前模型是否已经完成初始化
 */
static bool s_five_dof_initialized = false;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 将变换矩阵置为单位阵
 * @param T 目标变换矩阵
 */
static void s_tf_identity(SerialArmTransform* T);
/**
 * @brief 构造一个纯平移变换
 * @param T 输出变换矩阵
 * @param x x 平移，单位 m
 * @param y y 平移，单位 m
 * @param z z 平移，单位 m
 */
static void s_tf_transl(SerialArmTransform* T, float x, float y, float z);
/**
 * @brief 设置单个转动关节参数并附带统一限位与方向校验
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
 * @brief 五自由度机械臂纯数学模型接口实例
 */
const FiveDofArmKineInterface five_dof_arm_kine_instance = {
    .build_model = five_dof_arm_build_model,
    .init = five_dof_arm_init,
    .get_model = five_dof_arm_get_model,
    .get_mdh_zero = five_dof_arm_get_mdh_zero,
    .fk = five_dof_arm_fk,
    .fk_matrix = five_dof_arm_fk_matrix,
    .ik = five_dof_arm_ik,
    .all_ik = five_dof_arm_all_ik,
    .status_str = five_dof_arm_status_str,
};

/**
 * @brief 构建五自由度机械臂的纯数学 MDH 模型
 * @param model 输出模型
 * @return FiveDofArmStatus 领域层状态码
 */
FiveDofArmStatus five_dof_arm_build_model(SerialArmModel* model) {
    FiveDofArmStatus ret;

    if(model == NULL) return SERIAL_ARM_STATUS_ERROR;

    ret = serial_arm.model_reset(model, FIVE_DOF_ARM_DOF, SERIAL_ARM_DH_MODIFIED);
    if(ret != SERIAL_ARM_STATUS_SUCCESS) return ret;

    s_tf_transl(&model->base_T, S_BASE_X, S_BASE_Y, 0.0f);
    s_tf_transl(&model->tool_T, S_TCP_LEN, 0.0f, 0.0f);

    ret = s_set_revolute_checked(model, 0u, 0.0f, S_H0, 0.0f, 0.0f, 0.0f, 1.0f);
    if(ret != SERIAL_ARM_STATUS_SUCCESS) return ret;
    ret = s_set_revolute_checked(model, 1u, 0.0f, 0.0f, 0.0f, M_PI * 0.5f, M_PI * 0.5f, 1.0f);
    if(ret != SERIAL_ARM_STATUS_SUCCESS) return ret;
    ret = s_set_revolute_checked(model, 2u, 0.0f, 0.0f, S_L1_LEN, 0.0f, 0.0f, -1.0f);
    if(ret != SERIAL_ARM_STATUS_SUCCESS) return ret;
    ret = s_set_revolute_checked(model, 3u, 0.0f, 0.0f, S_L2_LEN, 0.0f, 0.0f, 1.0f);
    if(ret != SERIAL_ARM_STATUS_SUCCESS) return ret;
    ret = s_set_revolute_checked(model, 4u, 0.0f, 0.0f, S_L3_LEN, 0.0f, 0.0f, -1.0f);
    if(ret != SERIAL_ARM_STATUS_SUCCESS) return ret;

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
 * @brief 构建模型并初始化通用串联机械臂求解器
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
 * @brief 获取当前只读模型指针
 * @return const SerialArmModel* 模型指针
 */
const SerialArmModel* five_dof_arm_get_model(void) {
    return s_five_dof_initialized ? &s_five_dof_model : NULL;
}

/**
 * @brief 获取数学 MDH 零位 `[0,0,0,0,0]`
 * @param joints 输出关节数组
 * @return FiveDofArmStatus 领域层状态码
 */
FiveDofArmStatus five_dof_arm_get_mdh_zero(FiveDofArmJointArray* joints) {
    static const float q[FIVE_DOF_ARM_DOF] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

    if(joints == NULL) return SERIAL_ARM_STATUS_ERROR;
    s_fill_joint_array(joints, q);
    return SERIAL_ARM_STATUS_SUCCESS;
}

/**
 * @brief 计算正运动学并输出位姿
 * @param joints 输入关节数组，单位 rad
 * @param pose 输出位姿
 * @return FiveDofArmStatus 领域层状态码
 */
FiveDofArmStatus five_dof_arm_fk(const FiveDofArmJointArray* joints, FiveDofArmPose* pose) {
    if(!s_five_dof_initialized) return SERIAL_ARM_STATUS_NOT_INITIALIZED;
    return serial_arm.fk(joints, pose);
}

/**
 * @brief 计算正运动学并输出齐次变换矩阵
 * @param joints 输入关节数组，单位 rad
 * @param T 输出 4x4 齐次变换矩阵
 * @return FiveDofArmStatus 领域层状态码
 */
FiveDofArmStatus five_dof_arm_fk_matrix(const FiveDofArmJointArray* joints, FiveDofArmTransform* T) {
    if(!s_five_dof_initialized) return SERIAL_ARM_STATUS_NOT_INITIALIZED;
    return serial_arm.fk_matrix(joints, T);
}

/**
 * @brief 计算单组逆运动学解
 * @param target 目标位姿
 * @param joints 输出关节数组
 * @param seed 逆解初值
 * @return FiveDofArmStatus 领域层状态码
 */
FiveDofArmStatus five_dof_arm_ik(const FiveDofArmPose* target, FiveDofArmJointArray* joints,
    const FiveDofArmJointArray* seed) {
    if(!s_five_dof_initialized) return SERIAL_ARM_STATUS_NOT_INITIALIZED;
    return serial_arm.ik(target, joints, seed);
}

/**
 * @brief 搜索多组可行逆解
 * @param target 目标位姿
 * @param solutions 输出逆解集合
 * @return FiveDofArmStatus 领域层状态码
 */
FiveDofArmStatus five_dof_arm_all_ik(const FiveDofArmPose* target, FiveDofArmJointSolutions* solutions) {
    if(!s_five_dof_initialized) return SERIAL_ARM_STATUS_NOT_INITIALIZED;
    return serial_arm.all_ik(target, solutions);
}

/**
 * @brief 将状态码转换为字符串
 * @param status 状态码
 * @return const char* 静态字符串
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
 * @brief 构造一个纯平移变换
 * @param T 输出变换矩阵
 * @param x x 平移，单位 m
 * @param y y 平移，单位 m
 * @param z z 平移，单位 m
 */
static void s_tf_transl(SerialArmTransform* T, float x, float y, float z) {
    s_tf_identity(T);
    T->m[0][3] = x;
    T->m[1][3] = y;
    T->m[2][3] = z;
}

/**
 * @brief 设置单个转动关节参数并附带统一限位与方向校验
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
        -FIVE_DOF_ARM_Q_LIMIT_RAD,
        FIVE_DOF_ARM_Q_LIMIT_RAD);
    if(ret != SERIAL_ARM_STATUS_SUCCESS) return ret;
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

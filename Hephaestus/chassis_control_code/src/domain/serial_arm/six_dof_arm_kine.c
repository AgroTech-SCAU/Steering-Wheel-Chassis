#include "serial_arm/six_dof_arm_kine.h"

#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "matrix.h"

// ! ========================= 变 量 声 明 ========================= ! //

const struct SixDofArmKineInterface six_dof_arm_kine_instance = {
    .init = s_six_dof_init,
    .fk = s_six_dof_fk,
    .ik = s_six_dof_ik,
    .all_ik = s_six_dof_all_ik,
    .solution_select = s_solution_select,
    .rpy_to_quat = s_rpy_to_quat,
    .quat_to_rpy = s_quat_to_rpy
};

/// @brief 机械臂的 MDH 参数
static ArmMDH arm_mdh = { 0 };
static bool arm_initialized = false;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void get_tf_matrix(Matrix* T, float alpha, float a, float theta, float d);
static void fk_compute(const Matrix* q, Matrix* out);
static void matrix_to_pose(const Matrix* T, Pose* pose);

static void joints_to_array(const SixDofJoint* j, Matrix* q);
static void array_to_joints(const Matrix* q, SixDofJoint* j);

static void extract_rotation(const Matrix* T, float R[3][3]);
static void quat_to_rotation(const Quaternion* q, float R[3][3]);
static void rotation_error(const float Rd[3][3], const float R[3][3], float eo[3]);
static void angular_jacobian_col(const float R[3][3], const float R2[3][3], float omega[3], float eps);

static bool solution_is_unique(const SixDofJointAll* sols, const SixDofJoint* candidate);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化机械臂的 MDH 参数
 * @param mdh 机械臂的 MDH 参数
 * @return ArmStatus 错误码
 */
ArmStatus s_six_dof_init(const ArmMDH* mdh) {
    if(mdh == NULL) return ARM_STATUS_ERROR;
    arm_mdh = *mdh;
    arm_initialized = true;
    return ARM_STATUS_SUCCESS;
}

/**
 * @brief 计算机械臂的正运动学
 * @param joints 机械臂的关节角度
 * @param pose 输出的末端位姿
 * @return ArmStatus 错误码
 */
ArmStatus s_six_dof_fk(const SixDofJoint* joints, Pose* pose) {
    if(arm_initialized == false) return ARM_STATUS_NOT_INITIALIZE;
    if(joints == NULL || pose == NULL) return ARM_STATUS_ERROR;

    matrix_create(q, 6, 1);
    matrix_create(T, 4, 4);

    joints_to_array(joints, &q);
    fk_compute(&q, &T);
    matrix_to_pose(&T, pose);

    return ARM_STATUS_SUCCESS;
}

/**
 * @brief 计算机械臂的逆运动学，使用数值雅可比法
 * @param pose 目标末端位姿
 * @param joints 输出的关节角度解
 * @param current_joints 当前关节角度，作为初始猜测
 * @param mode IK 模式，决定只约束位置、姿态还是全位姿
 * @return ArmStatus 错误码
 */
ArmStatus s_six_dof_ik(const Pose* pose, SixDofJoint* joints, const SixDofJoint* current_joints, IkMode mode) {
    if(arm_initialized == false) return ARM_STATUS_NOT_INITIALIZE;
    if(!pose || !joints || !current_joints)
        return ARM_STATUS_ERROR;

    matrix_create(q, 6, 1);
    matrix_create(T, 4, 4);
    matrix_create(T2, 4, 4);
    matrix_create(q_tmp, 6, 1);
    matrix_create(J, 6, 6);
    matrix_create(JT, 6, 6);
    matrix_create(JJT, 6, 6);
    matrix_create(inv, 6, 6);
    matrix_create(tmp, 6, 6);
    matrix_create(err, 6, 1);
    matrix_create(dq, 6, 1);

    joints_to_array(current_joints, &q);

    const float lambda = 1e-4f;
    const float alpha = 0.5f;
    const float eps = 1e-5f;
    matrix_identity_create(L, 6);
    matrix_scalar_mul(&L, lambda, &L);

    float Rd[3][3];
    if(mode == IK_MODE_ORIENTATION_ONLY || mode == IK_MODE_FULL_POSE) {
        quat_to_rotation(&pose->orientation, Rd);
    }

    for(unsigned int iter = 0; iter < 300; iter++) {
        fk_compute(&q, &T);

        float x = 0, y = 0, z = 0;
        matrix_get(&T, 0, 3, &x);
        matrix_get(&T, 1, 3, &y);
        matrix_get(&T, 2, 3, &z);

        float R[3][3];
        extract_rotation(&T, R);

        float dp[3] = { pose->position.x - x,
                         pose->position.y - y,
                         pose->position.z - z };
        float eo[3] = { 0.0f, 0.0f, 0.0f };

        if(mode == IK_MODE_ORIENTATION_ONLY || mode == IK_MODE_FULL_POSE) {
            rotation_error(Rd, R, eo);
        }

        matrix_set(&err, 0, 0, (mode == IK_MODE_ORIENTATION_ONLY) ? 0.0f : dp[0]);
        matrix_set(&err, 1, 0, (mode == IK_MODE_ORIENTATION_ONLY) ? 0.0f : dp[1]);
        matrix_set(&err, 2, 0, (mode == IK_MODE_ORIENTATION_ONLY) ? 0.0f : dp[2]);
        matrix_set(&err, 3, 0, (mode == IK_MODE_POSITION_ONLY) ? 0.0f : eo[0]);
        matrix_set(&err, 4, 0, (mode == IK_MODE_POSITION_ONLY) ? 0.0f : eo[1]);
        matrix_set(&err, 5, 0, (mode == IK_MODE_POSITION_ONLY) ? 0.0f : eo[2]);

        float norm_pos = sqrtf(dp[0] * dp[0] + dp[1] * dp[1] + dp[2] * dp[2]);
        float norm_ori = sqrtf(eo[0] * eo[0] + eo[1] * eo[1] + eo[2] * eo[2]);

        bool pos_ok = (mode == IK_MODE_ORIENTATION_ONLY) || (norm_pos < 1e-4f);
        bool ori_ok = (mode == IK_MODE_POSITION_ONLY) || (norm_ori < 1e-3f);

        if(pos_ok && ori_ok) {
            array_to_joints(&q, joints);
            return ARM_STATUS_SUCCESS;
        }

        for(unsigned int i = 0; i < 6; i++) {
            matrix_copy(&q, &q_tmp);
            q_tmp.pdata[i] += eps;

            fk_compute(&q_tmp, &T2);

            float x2 = 0, y2 = 0, z2 = 0;
            matrix_get(&T2, 0, 3, &x2);
            matrix_get(&T2, 1, 3, &y2);
            matrix_get(&T2, 2, 3, &z2);

            float R2[3][3];
            extract_rotation(&T2, R2);

            float jp0 = (mode == IK_MODE_ORIENTATION_ONLY) ? 0.0f : (x2 - x) / eps;
            float jp1 = (mode == IK_MODE_ORIENTATION_ONLY) ? 0.0f : (y2 - y) / eps;
            float jp2 = (mode == IK_MODE_ORIENTATION_ONLY) ? 0.0f : (z2 - z) / eps;

            float omega[3] = { 0.0f, 0.0f, 0.0f };
            if(mode == IK_MODE_ORIENTATION_ONLY || mode == IK_MODE_FULL_POSE) {
                angular_jacobian_col(R, R2, omega, eps);
            }

            matrix_set(&J, 0, i, jp0);
            matrix_set(&J, 1, i, jp1);
            matrix_set(&J, 2, i, jp2);
            matrix_set(&J, 3, i, omega[0]);
            matrix_set(&J, 4, i, omega[1]);
            matrix_set(&J, 5, i, omega[2]);
        }

        matrix_transpose(&J, &JT);
        matrix_mul(&JT, &J, &JJT);
        matrix_add(&JJT, &L, &JJT);

        if(matrix_inverse(&JJT, &inv) != MATRIX_SUCCESS)
            return ARM_STATUS_SINGULARITY;

        matrix_mul(&inv, &JT, &tmp);
        matrix_mul(&tmp, &err, &dq);

        for(unsigned int i = 0; i < 6; i++) {
            q.pdata[i] += alpha * dq.pdata[i];
            if(q.pdata[i] < arm_mdh.qmin[i]) q.pdata[i] = arm_mdh.qmin[i];
            if(q.pdata[i] > arm_mdh.qmax[i]) q.pdata[i] = arm_mdh.qmax[i];
        }
    }

    return ARM_STATUS_OUT_OF_REACH;
}

/**
 * @brief 计算机械臂的所有逆运动学解，通过在不同初始猜测下调用 IK 求解函数
 * @param pose 目标末端位姿
 * @param joints 输出的所有关节角度解
 * @param mode IK 模式，决定只约束位置、姿态还是全位姿
 * @return ArmStatus 错误码
 */
ArmStatus s_six_dof_all_ik(const Pose* pose, SixDofJointAll* joints, IkMode mode) {
    if(arm_initialized == false) return ARM_STATUS_NOT_INITIALIZE;
    if(!pose || !joints) return ARM_STATUS_ERROR;

    joints->num_solutions = 0;
    const float ratios[3] = { 0.5f, 0.0f, -0.5f };

    for(int i1 = 0; i1 < 3; i1++) {
        for(int i2 = 0; i2 < 3; i2++) {
            for(int i3 = 0; i3 < 3; i3++) {
                if(joints->num_solutions >= 8) return (joints->num_solutions > 0) ? ARM_STATUS_SUCCESS : ARM_STATUS_OUT_OF_REACH;

                SixDofJoint seed = { 0 };
                seed.joint_1 = ratios[i1] * (ratios[i1] > 0 ? arm_mdh.qmax[0] : -arm_mdh.qmin[0]);
                seed.joint_2 = ratios[i2] * (ratios[i2] > 0 ? arm_mdh.qmax[1] : -arm_mdh.qmin[1]);
                seed.joint_3 = ratios[i3] * (ratios[i3] > 0 ? arm_mdh.qmax[2] : -arm_mdh.qmin[2]);

                SixDofJoint candidate = { 0 };
                ArmStatus ret = s_six_dof_ik(pose, &candidate, &seed, mode);

                if(ret != ARM_STATUS_SUCCESS) continue;
                if(!solution_is_unique(joints, &candidate)) continue;

                *s_solution_select(joints, joints->num_solutions) = candidate;
                joints->num_solutions++;
            }
        }
    }

    return (joints->num_solutions > 0) ? ARM_STATUS_SUCCESS : ARM_STATUS_OUT_OF_REACH;
}

/**
 * @brief 从所有解中选择一个特定索引的解
 * @param sols 所有解的结构体
 * @param idx 要选择的解的索引，范围 0-7
 * @return 指向选择的解的指针，如果索引无效则返回 NULL
 */
SixDofJoint* s_solution_select(SixDofJointAll* sols, uint8_t idx) {
    switch(idx) {
        case 0: return &sols->solution_1;
        case 1: return &sols->solution_2;
        case 2: return &sols->solution_3;
        case 3: return &sols->solution_4;
        case 4: return &sols->solution_5;
        case 5: return &sols->solution_6;
        case 6: return &sols->solution_7;
        case 7: return &sols->solution_8;
        default: return NULL;
    }
}

/**
 * @brief 将欧拉角转换为四元数
 * @param rpy 输入的欧拉角，单位为弧度
 * @param quat 输出的四元数
 * @return ArmStatus 错误码
 */
ArmStatus s_rpy_to_quat(const RPY rpy, Quaternion* quat) {
    if(!quat) return ARM_STATUS_ERROR;

    float cx = cosf(rpy.roll * 0.5f);
    float sx = sinf(rpy.roll * 0.5f);
    float cy = cosf(rpy.pitch * 0.5f);
    float sy = sinf(rpy.pitch * 0.5f);
    float cz = cosf(rpy.yaw * 0.5f);
    float sz = sinf(rpy.yaw * 0.5f);

    quat->w = cx * cy * cz + sx * sy * sz;
    quat->x = sx * cy * cz - cx * sy * sz;
    quat->y = cx * sy * cz + sx * cy * sz;
    quat->z = cx * cy * sz - sx * sy * cz;

    return ARM_STATUS_SUCCESS;
}

/**
 * @brief 将四元数转换为欧拉角
 * @param q 输入的四元数
 * @param rpy 输出的欧拉角
 * @return ArmStatus 错误码
 */
ArmStatus s_quat_to_rpy(const Quaternion q, RPY* rpy) {
    if(!rpy) return ARM_STATUS_ERROR;

    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    rpy->roll = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if(fabsf(sinp) >= 1.0f)
        rpy->pitch = copysignf(M_PI / 2.0f, sinp);
    else
        rpy->pitch = asinf(sinp);

    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    rpy->yaw = atan2f(siny_cosp, cosy_cosp);

    return ARM_STATUS_SUCCESS;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 将关节结构体转换为数组形式，便于计算
 * @param j 关节结构体
 * @param q 输出的关节角度数组
 */
static void joints_to_array(const SixDofJoint* j, Matrix* q) {
    matrix_set(q, 0, 0, j->joint_1);
    matrix_set(q, 1, 0, j->joint_2);
    matrix_set(q, 2, 0, j->joint_3);
    matrix_set(q, 3, 0, j->joint_4);
    matrix_set(q, 4, 0, j->joint_5);
    matrix_set(q, 5, 0, j->joint_6);
}

/**
 * @brief 将关节角度数组转换回结构体形式
 * @param q 关节角度数组
 * @param j 输出的关节结构体
 */
static void array_to_joints(const Matrix* q, SixDofJoint* j) {
    matrix_get(q, 0, 0, &j->joint_1);
    matrix_get(q, 1, 0, &j->joint_2);
    matrix_get(q, 2, 0, &j->joint_3);
    matrix_get(q, 3, 0, &j->joint_4);
    matrix_get(q, 4, 0, &j->joint_5);
    matrix_get(q, 5, 0, &j->joint_6);
}


/**
 * @brief 根据 MDH 参数生成变换矩阵
 * @param T 输出的 4x4 变换矩阵
 * @param alpha 关节 i-1 的扭转角
 * @param a 关节 i-1 的连杆长度
 * @param theta 关节 i 的关节角
 * @param d 关节 i 的连杆偏移
 */
static void get_tf_matrix(Matrix* T, float alpha, float a, float theta, float d) {
    matrix_set(T, 0, 0, cosf(theta));
    matrix_set(T, 0, 1, -sinf(theta));
    matrix_set(T, 0, 2, 0);
    matrix_set(T, 0, 3, a);

    matrix_set(T, 1, 0, sinf(theta) * cosf(alpha));
    matrix_set(T, 1, 1, cosf(theta) * cosf(alpha));
    matrix_set(T, 1, 2, -sinf(alpha));
    matrix_set(T, 1, 3, -d * sinf(alpha));

    matrix_set(T, 2, 0, sinf(theta) * sinf(alpha));
    matrix_set(T, 2, 1, cosf(theta) * sinf(alpha));
    matrix_set(T, 2, 2, cosf(alpha));
    matrix_set(T, 2, 3, d * cosf(alpha));

    matrix_set(T, 3, 0, 0.0f);
    matrix_set(T, 3, 1, 0.0f);
    matrix_set(T, 3, 2, 0.0f);
    matrix_set(T, 3, 3, 1.0f);
}

/**
 * @brief 计算机械臂的正运动学，得到末端的变换矩阵
 * @param q 关节角度数组
 * @param T_out 输出的末端变换矩阵
 */
static void fk_compute(const Matrix* q, Matrix* out) {
    matrix_identity_create(T, 4);
    matrix_identity_create(Ti, 4);
    matrix_identity_create(temp, 4);

    for(int i = 0; i < 6; i++) {
        get_tf_matrix(&Ti,
            arm_mdh.alpha[i],
            arm_mdh.a[i],
            q->pdata[i] + arm_mdh.offset[i],
            arm_mdh.d[i]);

        matrix_mul(&T, &Ti, &temp);
        matrix_copy(&temp, &T);
    }
    matrix_copy(&T, out);
}

/**
 * @brief 将 4x4 变换矩阵转换为位姿结构体
 * @param T 输入的 4x4 变换矩阵
 * @param pose 输出的位姿结构体
 */
static void matrix_to_pose(const Matrix* T, Pose* pose) {
    matrix_get(T, 0, 3, &pose->position.x);
    matrix_get(T, 1, 3, &pose->position.y);
    matrix_get(T, 2, 3, &pose->position.z);

    float r11 = 0, r12 = 0, r13 = 0;
    float r21 = 0, r22 = 0, r23 = 0;
    float r31 = 0, r32 = 0, r33 = 0;
    matrix_get(T, 0, 0, &r11); matrix_get(T, 0, 1, &r12); matrix_get(T, 0, 2, &r13);
    matrix_get(T, 1, 0, &r21); matrix_get(T, 1, 1, &r22); matrix_get(T, 1, 2, &r23);
    matrix_get(T, 2, 0, &r31); matrix_get(T, 2, 1, &r32); matrix_get(T, 2, 2, &r33);

    float trace = r11 + r22 + r33;
    float qw, qx, qy, qz;

    if(trace > 0.0f) {
        float s = 0.5f / sqrtf(trace + 1.0f);
        qw = 0.25f / s;
        qx = (r32 - r23) * s;
        qy = (r13 - r31) * s;
        qz = (r21 - r12) * s;
    }
    else if(r11 > r22 && r11 > r33) {
        float s = 2.0f * sqrtf(1.0f + r11 - r22 - r33);
        qw = (r32 - r23) / s;
        qx = 0.25f * s;
        qy = (r12 + r21) / s;
        qz = (r13 + r31) / s;
    }
    else if(r22 > r33) {
        float s = 2.0f * sqrtf(1.0f + r22 - r11 - r33);
        qw = (r13 - r31) / s;
        qx = (r12 + r21) / s;
        qy = 0.25f * s;
        qz = (r23 + r32) / s;
    }
    else {
        float s = 2.0f * sqrtf(1.0f + r33 - r11 - r22);
        qw = (r21 - r12) / s;
        qx = (r13 + r31) / s;
        qy = (r23 + r32) / s;
        qz = 0.25f * s;
    }

    pose->orientation.w = qw;
    pose->orientation.x = qx;
    pose->orientation.y = qy;
    pose->orientation.z = qz;
}

/**
 * @brief 从变换矩阵中提取旋转部分
 * @param T 输入的 4x4 变换矩阵
 * @param R 输出的 3x3 旋转矩阵
 */
static void extract_rotation(const Matrix* T, float R[3][3]) {
    for(unsigned int i = 0; i < 3; i++)
        for(unsigned int j = 0; j < 3; j++)
            matrix_get(T, i, j, &R[i][j]);
}

/**
 * @brief 将四元数转换为旋转矩阵
 * @param q 输入的四元数
 * @param R 输出的 3x3 旋转矩阵
 */
static void quat_to_rotation(const Quaternion* q, float R[3][3]) {
    float qw = q->w, qx = q->x, qy = q->y, qz = q->z;
    R[0][0] = 1.0f - 2.0f * (qy * qy + qz * qz);
    R[0][1] = 2.0f * (qx * qy - qw * qz);
    R[0][2] = 2.0f * (qx * qz + qw * qy);
    R[1][0] = 2.0f * (qx * qy + qw * qz);
    R[1][1] = 1.0f - 2.0f * (qx * qx + qz * qz);
    R[1][2] = 2.0f * (qy * qz - qw * qx);
    R[2][0] = 2.0f * (qx * qz - qw * qy);
    R[2][1] = 2.0f * (qy * qz + qw * qx);
    R[2][2] = 1.0f - 2.0f * (qx * qx + qy * qy);
}

/**
 * @brief 计算当前旋转与目标旋转之间的误差，返回一个表示误差的向量
 * @param Rd 目标旋转矩阵
 * @param R 当前旋转矩阵
 * @param eo 输出的误差向量，表示绕 x、y、z 轴的误差
 */
static void rotation_error(const float Rd[3][3], const float R[3][3], float eo[3]) {
    float Re[3][3] = { {0} };
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            for(int k = 0; k < 3; k++)
                Re[i][j] += Rd[i][k] * R[j][k];

    eo[0] = (Re[2][1] - Re[1][2]) * 0.5f;
    eo[1] = (Re[0][2] - Re[2][0]) * 0.5f;
    eo[2] = (Re[1][0] - Re[0][1]) * 0.5f;
}

/**
 * @brief 计算数值雅可比矩阵的一列，表示关节 i 的小变化对末端位置和姿态的影响
 * @param R 当前旋转矩阵
 * @param R2 关节 i 增加一个小量后的旋转矩阵
 * @param omega 输出的雅可比列，包含位置部分和姿态部分
 * @param eps 用于数值微分的小量
 */
static void angular_jacobian_col(const float R[3][3], const float R2[3][3],
    float omega[3], float eps) {
    float dR[3][3] = { {0} };
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            for(int k = 0; k < 3; k++)
                dR[i][j] += R[k][i] * R2[k][j];

    float inv2eps = 1.0f / (2.0f * eps);
    omega[0] = (dR[2][1] - dR[1][2]) * inv2eps;
    omega[1] = (dR[0][2] - dR[2][0]) * inv2eps;
    omega[2] = (dR[1][0] - dR[0][1]) * inv2eps;
}

/**
 * @brief 检查一个候选解是否与已找到的解足够不同，以避免重复解
 * @param sols 已找到的所有解的结构体
 * @param candidate 要检查的候选解
 * @return true: 候选解是唯一的；false: 候选解与已存在的某个解过于相似
 */
static bool solution_is_unique(const SixDofJointAll* sols,
    const SixDofJoint* candidate) {
    const float thresh = 0.05f;
    const SixDofJoint* s;

    for(uint8_t k = 0; k < sols->num_solutions; k++) {
        s = s_solution_select((SixDofJointAll*)sols, k);
        if(fabsf(s->joint_1 - candidate->joint_1) < thresh &&
            fabsf(s->joint_2 - candidate->joint_2) < thresh &&
            fabsf(s->joint_3 - candidate->joint_3) < thresh &&
            fabsf(s->joint_4 - candidate->joint_4) < thresh &&
            fabsf(s->joint_5 - candidate->joint_5) < thresh &&
            fabsf(s->joint_6 - candidate->joint_6) < thresh) {
            return false;
        }
    }
    return true;
}

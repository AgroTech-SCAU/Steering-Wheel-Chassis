/**
 * @file serial_arm_kine.c
 * @brief 通用串联机械臂 MDH/FK/IK 实现
 */
#include "serial_arm_kine.h"

#include <math.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 通用串联机械臂统一接口实例
 */
const SerialArmKineInterface serial_arm_kine_instance = {
    .model_reset = s_serial_arm_model_reset,
    .model_set_revolute = s_serial_arm_model_set_revolute,
    .model_set_prismatic = s_serial_arm_model_set_prismatic,
    .model_set_joint_sign = s_serial_arm_model_set_joint_sign,
    .init = s_serial_arm_init,
    .get_task_info = s_serial_arm_get_task_info,
    .set_task_info = s_serial_arm_set_task_info,
    .reset_task_info = s_serial_arm_reset_task_info,
    .task_row_name = s_serial_arm_task_row_name,
    .status_str = s_serial_arm_status_str,
    .fk = s_serial_arm_fk,
    .fk_matrix = s_serial_arm_fk_matrix,
    .ik = s_serial_arm_ik,
    .all_ik = s_serial_arm_all_ik,
    .solution_select = s_serial_arm_solution_select,
    .rpy_to_quat = s_serial_arm_rpy_to_quat,
    .quat_to_rpy = s_serial_arm_quat_to_rpy,
    .pose_from_xyz_rpy = s_serial_arm_pose_from_xyz_rpy,
};

/**
 * @brief 当前已加载的串联机械臂模型
 */
static SerialArmModel s_model;
/**
 * @brief 当前自动推断的 IK 任务行信息
 */
static SerialArmTaskInfo s_task;
/**
 * @brief 模块是否已完成初始化
 */
static bool s_initialized = false;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static float s_clampf(float v, float lo, float hi);
static float s_wrap_pi(float x);
static void s_tf_identity(SerialArmTransform* T);
static void s_tf_mul(const SerialArmTransform* A, const SerialArmTransform* B, SerialArmTransform* out);
static void s_dh_standard_tf(const SerialArmLink* link, float q_user, SerialArmTransform* T);
static void s_dh_modified_tf(const SerialArmLink* link, float q_user, SerialArmTransform* T);
static SerialArmStatus s_fk_compute(const float q[SERIAL_ARM_MAX_DOF], SerialArmTransform* T_out);
static void s_matrix_to_pose(const SerialArmTransform* T, SerialArmPose* pose);
static void s_pose_to_rotation(const SerialArmPose* pose, float R[3][3]);
static void s_extract_rotation(const SerialArmTransform* T, float R[3][3]);
static void s_rotation_error(const float Rd[3][3], const float R[3][3], float eo[3]);
static void s_angular_jacobian_col(const float R[3][3], const float R2[3][3], float omega[3], float eps);
static void s_compute_full_error(const SerialArmPose* target, const SerialArmTransform* current_T, float err6[6]);
static void s_compute_full_jacobian(const float q[SERIAL_ARM_MAX_DOF], float J6[6][SERIAL_ARM_MAX_DOF], float eps);
static void s_auto_select_task_rows(void);
static bool s_validate_model(const SerialArmModel* model);
static bool s_validate_joints(const SerialArmJointArray* joints);
/**
 * @brief 检查外部传入的 IK 任务行配置是否合法
 * @param info 目标任务信息
 * @return bool `true` 表示配置合法
 */
static bool s_validate_task_info(const SerialArmTaskInfo* info);
static bool s_solve_linear(uint8_t n, float A[SERIAL_ARM_TASK_MAX_DIM][SERIAL_ARM_TASK_MAX_DIM],
                           float b[SERIAL_ARM_TASK_MAX_DIM], float x[SERIAL_ARM_TASK_MAX_DIM]);
static bool s_dls_step(const float J[SERIAL_ARM_TASK_MAX_DIM][SERIAL_ARM_MAX_DOF],
                       const float err[SERIAL_ARM_TASK_MAX_DIM], uint8_t rows, uint8_t cols,
                       float damping, float dq[SERIAL_ARM_MAX_DOF]);
static bool s_solution_is_unique(const SerialArmJointSolutions* sols, const SerialArmJointArray* candidate);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SerialArmStatus s_serial_arm_model_reset(SerialArmModel* model, uint8_t dof, SerialArmDhConvention convention) {
    if(model == NULL)
        return SERIAL_ARM_STATUS_ERROR;
    if(dof == 0u || dof > SERIAL_ARM_MAX_DOF)
        return SERIAL_ARM_STATUS_INVALID_MODEL;
    if(convention != SERIAL_ARM_DH_STANDARD && convention != SERIAL_ARM_DH_MODIFIED)
        return SERIAL_ARM_STATUS_INVALID_MODEL;

    memset(model, 0, sizeof(*model));
    model->dof = dof;
    model->convention = convention;
    s_tf_identity(&model->base_T);
    s_tf_identity(&model->tool_T);

    model->ik.max_iterations = 300.0f;
    model->ik.position_tolerance = 1e-4f;
    model->ik.orientation_tolerance = 1e-3f;
    model->ik.step_gain = 0.5f;
    model->ik.damping = 1e-3f;
    model->ik.numeric_eps = 1e-5f;
    model->ik.position_weight = 1.0f;
    model->ik.orientation_weight = 1.0f;

    for(uint8_t i = 0u; i < dof; i++) {
        model->link[i].type = SERIAL_ARM_JOINT_REVOLUTE;
        model->link[i].q_sign = 1.0f;
        model->link[i].q_min = -M_PI;
        model->link[i].q_max = M_PI;
    }
    return SERIAL_ARM_STATUS_SUCCESS;
}

SerialArmStatus s_serial_arm_model_set_revolute(SerialArmModel* model, uint8_t index,
                                                float theta_home, float d, float a, float alpha,
                                                float q_offset, float q_min, float q_max) {
    if(model == NULL)
        return SERIAL_ARM_STATUS_ERROR;
    if(index >= model->dof || index >= SERIAL_ARM_MAX_DOF)
        return SERIAL_ARM_STATUS_INVALID_MODEL;
    if(q_min > q_max)
        return SERIAL_ARM_STATUS_INVALID_MODEL;

    model->link[index].type = SERIAL_ARM_JOINT_REVOLUTE;
    model->link[index].theta = theta_home;
    model->link[index].d = d;
    model->link[index].a = a;
    model->link[index].alpha = alpha;
    model->link[index].q_offset = q_offset;
    model->link[index].q_sign = (model->link[index].q_sign == 0.0f) ? 1.0f : model->link[index].q_sign;
    model->link[index].q_min = q_min;
    model->link[index].q_max = q_max;
    return SERIAL_ARM_STATUS_SUCCESS;
}

SerialArmStatus s_serial_arm_model_set_prismatic(SerialArmModel* model, uint8_t index,
                                                 float theta, float d_home, float a, float alpha,
                                                 float q_offset, float q_min, float q_max) {
    if(model == NULL)
        return SERIAL_ARM_STATUS_ERROR;
    if(index >= model->dof || index >= SERIAL_ARM_MAX_DOF)
        return SERIAL_ARM_STATUS_INVALID_MODEL;
    if(q_min > q_max)
        return SERIAL_ARM_STATUS_INVALID_MODEL;

    model->link[index].type = SERIAL_ARM_JOINT_PRISMATIC;
    model->link[index].theta = theta;
    model->link[index].d = d_home;
    model->link[index].a = a;
    model->link[index].alpha = alpha;
    model->link[index].q_offset = q_offset;
    model->link[index].q_sign = (model->link[index].q_sign == 0.0f) ? 1.0f : model->link[index].q_sign;
    model->link[index].q_min = q_min;
    model->link[index].q_max = q_max;
    return SERIAL_ARM_STATUS_SUCCESS;
}

SerialArmStatus s_serial_arm_model_set_joint_sign(SerialArmModel* model, uint8_t index, float q_sign) {
    if(model == NULL)
        return SERIAL_ARM_STATUS_ERROR;
    if(index >= model->dof || index >= SERIAL_ARM_MAX_DOF)
        return SERIAL_ARM_STATUS_INVALID_MODEL;

    model->link[index].q_sign = (q_sign < 0.0f) ? -1.0f : 1.0f;
    return SERIAL_ARM_STATUS_SUCCESS;
}

SerialArmStatus s_serial_arm_init(const SerialArmModel* model) {
    if(!s_validate_model(model))
        return SERIAL_ARM_STATUS_INVALID_MODEL;
    s_model = *model;
    s_initialized = true;
    s_auto_select_task_rows();
    return SERIAL_ARM_STATUS_SUCCESS;
}

SerialArmStatus s_serial_arm_get_task_info(SerialArmTaskInfo* info) {
    if(!s_initialized)
        return SERIAL_ARM_STATUS_NOT_INITIALIZED;
    if(info == NULL)
        return SERIAL_ARM_STATUS_ERROR;
    *info = s_task;
    return SERIAL_ARM_STATUS_SUCCESS;
}

/**
 * @brief 显式设置当前 IK 任务行配置
 * @param info 目标任务信息
 * @return SerialArmStatus 运动学状态码
 */
SerialArmStatus s_serial_arm_set_task_info(const SerialArmTaskInfo* info) {
    if(!s_initialized)
        return SERIAL_ARM_STATUS_NOT_INITIALIZED;
    if(!s_validate_task_info(info))
        return SERIAL_ARM_STATUS_INVALID_MODEL;

    s_task = *info;
    return SERIAL_ARM_STATUS_SUCCESS;
}

/**
 * @brief 恢复为按当前自由度自动推断的 IK 任务行配置
 * @return SerialArmStatus 运动学状态码
 */
SerialArmStatus s_serial_arm_reset_task_info(void) {
    if(!s_initialized)
        return SERIAL_ARM_STATUS_NOT_INITIALIZED;

    s_auto_select_task_rows();
    return SERIAL_ARM_STATUS_SUCCESS;
}

const char* s_serial_arm_task_row_name(uint8_t row) {
    switch(row) {
        case 0u:
            return "x";
        case 1u:
            return "y";
        case 2u:
            return "z";
        case 3u:
            return "rx";
        case 4u:
            return "ry";
        case 5u:
            return "rz";
        default:
            return "unknown";
    }
}

const char* s_serial_arm_status_str(SerialArmStatus status) {
    switch(status) {
        case SERIAL_ARM_STATUS_SUCCESS:
            return "SERIAL_ARM_STATUS_SUCCESS";
        case SERIAL_ARM_STATUS_ERROR:
            return "SERIAL_ARM_STATUS_ERROR";
        case SERIAL_ARM_STATUS_NOT_INITIALIZED:
            return "SERIAL_ARM_STATUS_NOT_INITIALIZED";
        case SERIAL_ARM_STATUS_INVALID_MODEL:
            return "SERIAL_ARM_STATUS_INVALID_MODEL";
        case SERIAL_ARM_STATUS_INVALID_JOINTS:
            return "SERIAL_ARM_STATUS_INVALID_JOINTS";
        case SERIAL_ARM_STATUS_INVALID_POSE:
            return "SERIAL_ARM_STATUS_INVALID_POSE";
        case SERIAL_ARM_STATUS_SINGULARITY:
            return "SERIAL_ARM_STATUS_SINGULARITY";
        case SERIAL_ARM_STATUS_OUT_OF_REACH:
            return "SERIAL_ARM_STATUS_OUT_OF_REACH";
        case SERIAL_ARM_STATUS_NO_SOLUTION:
            return "SERIAL_ARM_STATUS_NO_SOLUTION";
        default:
            return "SERIAL_ARM_STATUS_UNKNOWN";
    }
}

SerialArmStatus s_serial_arm_fk(const SerialArmJointArray* joints, SerialArmPose* pose) {
    if(!s_initialized)
        return SERIAL_ARM_STATUS_NOT_INITIALIZED;
    if(!s_validate_joints(joints) || pose == NULL)
        return SERIAL_ARM_STATUS_INVALID_JOINTS;

    SerialArmTransform T;
    SerialArmStatus ret = s_fk_compute(joints->q, &T);
    if(ret != SERIAL_ARM_STATUS_SUCCESS)
        return ret;
    s_matrix_to_pose(&T, pose);
    return SERIAL_ARM_STATUS_SUCCESS;
}

SerialArmStatus s_serial_arm_fk_matrix(const SerialArmJointArray* joints, SerialArmTransform* T) {
    if(!s_initialized)
        return SERIAL_ARM_STATUS_NOT_INITIALIZED;
    if(!s_validate_joints(joints) || T == NULL)
        return SERIAL_ARM_STATUS_INVALID_JOINTS;
    return s_fk_compute(joints->q, T);
}

SerialArmStatus s_serial_arm_ik(const SerialArmPose* target, SerialArmJointArray* joints,
                                const SerialArmJointArray* seed) {
    if(!s_initialized)
        return SERIAL_ARM_STATUS_NOT_INITIALIZED;
    if(target == NULL || joints == NULL || seed == NULL)
        return SERIAL_ARM_STATUS_ERROR;
    if(!s_validate_joints(seed))
        return SERIAL_ARM_STATUS_INVALID_JOINTS;
    if(s_task.task_dim == 0u)
        return SERIAL_ARM_STATUS_INVALID_MODEL;

    float q[SERIAL_ARM_MAX_DOF] = { 0.0f };
    for(uint8_t i = 0u; i < s_model.dof; i++) {
        q[i] = s_clampf(seed->q[i], s_model.link[i].q_min, s_model.link[i].q_max);
    }

    const uint16_t max_iter = (s_model.ik.max_iterations > 1.0f) ? (uint16_t)s_model.ik.max_iterations : 300u;
    const float gain = (s_model.ik.step_gain > 0.0f) ? s_model.ik.step_gain : 0.5f;
    const float damping = (s_model.ik.damping > 0.0f) ? s_model.ik.damping : 1e-3f;
    const float eps = (s_model.ik.numeric_eps > 0.0f) ? s_model.ik.numeric_eps : 1e-5f;
    const float pos_tol = (s_model.ik.position_tolerance > 0.0f) ? s_model.ik.position_tolerance : 1e-4f;
    const float ori_tol = (s_model.ik.orientation_tolerance > 0.0f) ? s_model.ik.orientation_tolerance : 1e-3f;
    const float pos_w = (s_model.ik.position_weight > 0.0f) ? s_model.ik.position_weight : 1.0f;
    const float ori_w = (s_model.ik.orientation_weight > 0.0f) ? s_model.ik.orientation_weight : 1.0f;

    for(uint16_t iter = 0u; iter < max_iter; iter++) {
        SerialArmTransform T;
        if(s_fk_compute(q, &T) != SERIAL_ARM_STATUS_SUCCESS)
            return SERIAL_ARM_STATUS_ERROR;

        float err6[6] = { 0.0f };
        s_compute_full_error(target, &T, err6);

        bool need_pos = false;
        bool need_ori = false;
        float pos_sq = 0.0f;
        float ori_sq = 0.0f;
        for(uint8_t r = 0u; r < s_task.task_dim; r++) {
            uint8_t src = s_task.row[r];
            float e = err6[src];
            if(src < 3u) {
                need_pos = true;
                pos_sq += e * e;
            }
            else {
                need_ori = true;
                ori_sq += e * e;
            }
        }

        float pos_norm = sqrtf(pos_sq);
        float ori_norm = sqrtf(ori_sq);

        if((!need_pos || pos_norm < pos_tol) && (!need_ori || ori_norm < ori_tol)) {
            joints->dof = s_model.dof;
            for(uint8_t i = 0u; i < s_model.dof; i++)
                joints->q[i] = q[i];
            return SERIAL_ARM_STATUS_SUCCESS;
        }

        float J6[6][SERIAL_ARM_MAX_DOF] = { { 0.0f } };
        s_compute_full_jacobian(q, J6, eps);

        float J[SERIAL_ARM_TASK_MAX_DIM][SERIAL_ARM_MAX_DOF] = { { 0.0f } };
        float err[SERIAL_ARM_TASK_MAX_DIM] = { 0.0f };
        for(uint8_t r = 0u; r < s_task.task_dim; r++) {
            uint8_t src = s_task.row[r];
            float w = (src < 3u) ? pos_w : ori_w;
            err[r] = err6[src] * w;
            for(uint8_t c = 0u; c < s_model.dof; c++) {
                J[r][c] = J6[src][c] * w;
            }
        }

        float dq[SERIAL_ARM_MAX_DOF] = { 0.0f };
        if(!s_dls_step(J, err, s_task.task_dim, s_model.dof, damping, dq)) {
            return SERIAL_ARM_STATUS_SINGULARITY;
        }

        for(uint8_t i = 0u; i < s_model.dof; i++) {
            q[i] += gain * dq[i];
            q[i] = s_clampf(q[i], s_model.link[i].q_min, s_model.link[i].q_max);
        }
    }

    return SERIAL_ARM_STATUS_OUT_OF_REACH;
}

SerialArmStatus s_serial_arm_all_ik(const SerialArmPose* target, SerialArmJointSolutions* solutions) {
    if(!s_initialized)
        return SERIAL_ARM_STATUS_NOT_INITIALIZED;
    if(target == NULL || solutions == NULL)
        return SERIAL_ARM_STATUS_ERROR;

    memset(solutions, 0, sizeof(*solutions));

    const float ratio[3] = { 0.0f, 0.5f, -0.5f };
    uint32_t trial_count = 1u;
    for(uint8_t i = 0u; i < s_model.dof && i < 5u; i++)
        trial_count *= 3u;

    for(uint32_t t = 0u; t < trial_count; t++) {
        if(solutions->num_solutions >= SERIAL_ARM_MAX_SOLUTIONS)
            break;

        uint32_t code = t;
        SerialArmJointArray seed = { 0 };
        seed.dof = s_model.dof;
        for(uint8_t j = 0u; j < s_model.dof; j++) {
            uint8_t r = (j < 5u) ? (uint8_t)(code % 3u) : 0u;
            if(j < 5u)
                code /= 3u;
            float mid = 0.5f * (s_model.link[j].q_min + s_model.link[j].q_max);
            float span = 0.5f * (s_model.link[j].q_max - s_model.link[j].q_min);
            seed.q[j] = mid + ratio[r] * span;
        }

        SerialArmJointArray candidate = { 0 };
        SerialArmStatus ret = s_serial_arm_ik(target, &candidate, &seed);
        if(ret != SERIAL_ARM_STATUS_SUCCESS)
            continue;
        if(!s_solution_is_unique(solutions, &candidate))
            continue;

        solutions->solution[solutions->num_solutions] = candidate;
        solutions->num_solutions++;
    }

    return (solutions->num_solutions > 0u) ? SERIAL_ARM_STATUS_SUCCESS : SERIAL_ARM_STATUS_NO_SOLUTION;
}

SerialArmJointArray* s_serial_arm_solution_select(SerialArmJointSolutions* solutions, uint8_t index) {
    if(solutions == NULL || index >= solutions->num_solutions || index >= SERIAL_ARM_MAX_SOLUTIONS)
        return NULL;
    return &solutions->solution[index];
}

SerialArmStatus s_serial_arm_rpy_to_quat(const SerialArmRPY rpy, SerialArmQuaternion* quat) {
    if(quat == NULL)
        return SERIAL_ARM_STATUS_ERROR;

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
    return SERIAL_ARM_STATUS_SUCCESS;
}

SerialArmStatus s_serial_arm_quat_to_rpy(const SerialArmQuaternion quat, SerialArmRPY* rpy) {
    if(rpy == NULL)
        return SERIAL_ARM_STATUS_ERROR;

    float sinr_cosp = 2.0f * (quat.w * quat.x + quat.y * quat.z);
    float cosr_cosp = 1.0f - 2.0f * (quat.x * quat.x + quat.y * quat.y);
    rpy->roll = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (quat.w * quat.y - quat.z * quat.x);
    if(fabsf(sinp) >= 1.0f)
        rpy->pitch = copysignf(M_PI * 0.5f, sinp);
    else
        rpy->pitch = asinf(sinp);

    float siny_cosp = 2.0f * (quat.w * quat.z + quat.x * quat.y);
    float cosy_cosp = 1.0f - 2.0f * (quat.y * quat.y + quat.z * quat.z);
    rpy->yaw = atan2f(siny_cosp, cosy_cosp);
    return SERIAL_ARM_STATUS_SUCCESS;
}

SerialArmStatus s_serial_arm_pose_from_xyz_rpy(float x, float y, float z,
                                               float roll, float pitch, float yaw, SerialArmPose* pose) {
    if(pose == NULL)
        return SERIAL_ARM_STATUS_ERROR;
    pose->position.x = x;
    pose->position.y = y;
    pose->position.z = z;
    SerialArmRPY rpy = { roll, pitch, yaw };
    return s_serial_arm_rpy_to_quat(rpy, &pose->orientation);
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static float s_clampf(float v, float lo, float hi) {
    if(v < lo)
        return lo;
    if(v > hi)
        return hi;
    return v;
}

static float s_wrap_pi(float x) {
    while(x > M_PI)
        x -= M_2PI;
    while(x < -M_PI)
        x += M_2PI;
    return x;
}

static void s_tf_identity(SerialArmTransform* T) {
    memset(T, 0, sizeof(*T));
    T->m[0][0] = 1.0f;
    T->m[1][1] = 1.0f;
    T->m[2][2] = 1.0f;
    T->m[3][3] = 1.0f;
}

static void s_tf_mul(const SerialArmTransform* A, const SerialArmTransform* B, SerialArmTransform* out) {
    SerialArmTransform tmp;
    for(uint8_t i = 0u; i < 4u; i++) {
        for(uint8_t j = 0u; j < 4u; j++) {
            tmp.m[i][j] = 0.0f;
            for(uint8_t k = 0u; k < 4u; k++)
                tmp.m[i][j] += A->m[i][k] * B->m[k][j];
        }
    }
    *out = tmp;
}

static void s_dh_standard_tf(const SerialArmLink* link, float q_user, SerialArmTransform* T) {
    float theta = link->theta;
    float d = link->d;
    float q_sign = (link->q_sign < 0.0f) ? -1.0f : 1.0f;
    float q = q_sign * q_user + link->q_offset;
    if(link->type == SERIAL_ARM_JOINT_REVOLUTE)
        theta += q;
    else
        d += q;

    float ct = cosf(theta), st = sinf(theta);
    float ca = cosf(link->alpha), sa = sinf(link->alpha);

    T->m[0][0] = ct;
    T->m[0][1] = -st * ca;
    T->m[0][2] = st * sa;
    T->m[0][3] = link->a * ct;
    T->m[1][0] = st;
    T->m[1][1] = ct * ca;
    T->m[1][2] = -ct * sa;
    T->m[1][3] = link->a * st;
    T->m[2][0] = 0.0f;
    T->m[2][1] = sa;
    T->m[2][2] = ca;
    T->m[2][3] = d;
    T->m[3][0] = 0.0f;
    T->m[3][1] = 0.0f;
    T->m[3][2] = 0.0f;
    T->m[3][3] = 1.0f;
}

static void s_dh_modified_tf(const SerialArmLink* link, float q_user, SerialArmTransform* T) {
    float theta = link->theta;
    float d = link->d;
    float q_sign = (link->q_sign < 0.0f) ? -1.0f : 1.0f;
    float q = q_sign * q_user + link->q_offset;
    if(link->type == SERIAL_ARM_JOINT_REVOLUTE)
        theta += q;
    else
        d += q;

    float ct = cosf(theta), st = sinf(theta);
    float ca = cosf(link->alpha), sa = sinf(link->alpha);

    T->m[0][0] = ct;
    T->m[0][1] = -st;
    T->m[0][2] = 0.0f;
    T->m[0][3] = link->a;
    T->m[1][0] = st * ca;
    T->m[1][1] = ct * ca;
    T->m[1][2] = -sa;
    T->m[1][3] = -d * sa;
    T->m[2][0] = st * sa;
    T->m[2][1] = ct * sa;
    T->m[2][2] = ca;
    T->m[2][3] = d * ca;
    T->m[3][0] = 0.0f;
    T->m[3][1] = 0.0f;
    T->m[3][2] = 0.0f;
    T->m[3][3] = 1.0f;
}

static SerialArmStatus s_fk_compute(const float q[SERIAL_ARM_MAX_DOF], SerialArmTransform* T_out) {
    if(T_out == NULL)
        return SERIAL_ARM_STATUS_ERROR;

    SerialArmTransform T = s_model.base_T;
    for(uint8_t i = 0u; i < s_model.dof; i++) {
        SerialArmTransform A;
        if(s_model.convention == SERIAL_ARM_DH_STANDARD)
            s_dh_standard_tf(&s_model.link[i], q[i], &A);
        else
            s_dh_modified_tf(&s_model.link[i], q[i], &A);
        s_tf_mul(&T, &A, &T);
    }
    s_tf_mul(&T, &s_model.tool_T, &T);
    *T_out = T;
    return SERIAL_ARM_STATUS_SUCCESS;
}

static void s_matrix_to_pose(const SerialArmTransform* T, SerialArmPose* pose) {
    pose->position.x = T->m[0][3];
    pose->position.y = T->m[1][3];
    pose->position.z = T->m[2][3];

    float r11 = T->m[0][0], r12 = T->m[0][1], r13 = T->m[0][2];
    float r21 = T->m[1][0], r22 = T->m[1][1], r23 = T->m[1][2];
    float r31 = T->m[2][0], r32 = T->m[2][1], r33 = T->m[2][2];
    float trace = r11 + r22 + r33;

    if(trace > 0.0f) {
        float s = 0.5f / sqrtf(trace + 1.0f);
        pose->orientation.w = 0.25f / s;
        pose->orientation.x = (r32 - r23) * s;
        pose->orientation.y = (r13 - r31) * s;
        pose->orientation.z = (r21 - r12) * s;
    }
    else if(r11 > r22 && r11 > r33) {
        float s = 2.0f * sqrtf(1.0f + r11 - r22 - r33);
        pose->orientation.w = (r32 - r23) / s;
        pose->orientation.x = 0.25f * s;
        pose->orientation.y = (r12 + r21) / s;
        pose->orientation.z = (r13 + r31) / s;
    }
    else if(r22 > r33) {
        float s = 2.0f * sqrtf(1.0f + r22 - r11 - r33);
        pose->orientation.w = (r13 - r31) / s;
        pose->orientation.x = (r12 + r21) / s;
        pose->orientation.y = 0.25f * s;
        pose->orientation.z = (r23 + r32) / s;
    }
    else {
        float s = 2.0f * sqrtf(1.0f + r33 - r11 - r22);
        pose->orientation.w = (r21 - r12) / s;
        pose->orientation.x = (r13 + r31) / s;
        pose->orientation.y = (r23 + r32) / s;
        pose->orientation.z = 0.25f * s;
    }
}

static void s_pose_to_rotation(const SerialArmPose* pose, float R[3][3]) {
    float qw = pose->orientation.w;
    float qx = pose->orientation.x;
    float qy = pose->orientation.y;
    float qz = pose->orientation.z;

    float n = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
    if(n > 1e-9f) {
        qw /= n;
        qx /= n;
        qy /= n;
        qz /= n;
    }

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

static void s_extract_rotation(const SerialArmTransform* T, float R[3][3]) {
    for(uint8_t i = 0u; i < 3u; i++) {
        for(uint8_t j = 0u; j < 3u; j++)
            R[i][j] = T->m[i][j];
    }
}

static void s_rotation_error(const float Rd[3][3], const float R[3][3], float eo[3]) {
    float Re[3][3] = { { 0.0f } };
    for(uint8_t i = 0u; i < 3u; i++) {
        for(uint8_t j = 0u; j < 3u; j++) {
            for(uint8_t k = 0u; k < 3u; k++)
                Re[i][j] += Rd[i][k] * R[j][k];
        }
    }
    eo[0] = 0.5f * (Re[2][1] - Re[1][2]);
    eo[1] = 0.5f * (Re[0][2] - Re[2][0]);
    eo[2] = 0.5f * (Re[1][0] - Re[0][1]);
}

static void s_angular_jacobian_col(const float R[3][3], const float R2[3][3], float omega[3], float eps) {
    float dR[3][3] = { { 0.0f } };
    for(uint8_t i = 0u; i < 3u; i++) {
        for(uint8_t j = 0u; j < 3u; j++) {
            for(uint8_t k = 0u; k < 3u; k++)
                dR[i][j] += R[k][i] * R2[k][j];
        }
    }
    float inv2eps = 1.0f / (2.0f * eps);
    omega[0] = (dR[2][1] - dR[1][2]) * inv2eps;
    omega[1] = (dR[0][2] - dR[2][0]) * inv2eps;
    omega[2] = (dR[1][0] - dR[0][1]) * inv2eps;
}

static void s_compute_full_error(const SerialArmPose* target, const SerialArmTransform* current_T, float err6[6]) {
    SerialArmPose cur;
    s_matrix_to_pose(current_T, &cur);

    err6[0] = target->position.x - cur.position.x;
    err6[1] = target->position.y - cur.position.y;
    err6[2] = target->position.z - cur.position.z;

    float Rd[3][3], R[3][3];
    s_pose_to_rotation(target, Rd);
    s_extract_rotation(current_T, R);
    s_rotation_error(Rd, R, &err6[3]);
}

static void s_compute_full_jacobian(const float q[SERIAL_ARM_MAX_DOF], float J6[6][SERIAL_ARM_MAX_DOF], float eps) {
    SerialArmTransform T;
    s_fk_compute(q, &T);
    float R[3][3];
    s_extract_rotation(&T, R);

    for(uint8_t c = 0u; c < s_model.dof; c++) {
        float q2[SERIAL_ARM_MAX_DOF] = { 0.0f };
        for(uint8_t i = 0u; i < s_model.dof; i++)
            q2[i] = q[i];
        q2[c] += eps;

        SerialArmTransform T2;
        s_fk_compute(q2, &T2);
        float R2[3][3];
        s_extract_rotation(&T2, R2);

        J6[0][c] = (T2.m[0][3] - T.m[0][3]) / eps;
        J6[1][c] = (T2.m[1][3] - T.m[1][3]) / eps;
        J6[2][c] = (T2.m[2][3] - T.m[2][3]) / eps;

        float omega[3] = { 0.0f };
        s_angular_jacobian_col(R, R2, omega, eps);
        J6[3][c] = omega[0];
        J6[4][c] = omega[1];
        J6[5][c] = omega[2];
    }
}

static void s_auto_select_task_rows(void) {
    memset(&s_task, 0, sizeof(s_task));
    uint8_t dim = s_model.dof;
    if(dim > SERIAL_ARM_TASK_MAX_DIM)
        dim = SERIAL_ARM_TASK_MAX_DIM;
    s_task.task_dim = dim;

    if(dim >= 6u) {
        for(uint8_t i = 0u; i < 6u; i++)
            s_task.row[i] = i;
        return;
    }

    if(dim >= 3u) {
        s_task.row[0] = 0u;
        s_task.row[1] = 1u;
        s_task.row[2] = 2u;

        uint8_t selected = 3u;
        if(selected >= dim)
            return;

        float q[SERIAL_ARM_MAX_DOF] = { 0.0f };
        for(uint8_t i = 0u; i < s_model.dof; i++) {
            if(s_model.link[i].q_min < s_model.link[i].q_max) {
                q[i] = 0.5f * (s_model.link[i].q_min + s_model.link[i].q_max);
            }
        }

        float J6[6][SERIAL_ARM_MAX_DOF] = { { 0.0f } };
        s_compute_full_jacobian(q, J6, (s_model.ik.numeric_eps > 0.0f) ? s_model.ik.numeric_eps : 1e-5f);

        bool used[6] = { true, true, true, false, false, false };
        while(selected < dim) {
            uint8_t best_row = 3u;
            float best_score = -1.0f;
            for(uint8_t r = 3u; r < 6u; r++) {
                if(used[r])
                    continue;
                float score = 0.0f;
                for(uint8_t c = 0u; c < s_model.dof; c++)
                    score += fabsf(J6[r][c]);
                if(score > best_score) {
                    best_score = score;
                    best_row = r;
                }
            }
            used[best_row] = true;
            s_task.row[selected++] = best_row;
        }
        return;
    }

    // 低自由度机械臂：选择最可控的位置坐标行
    float q[SERIAL_ARM_MAX_DOF] = { 0.0f };
    float J6[6][SERIAL_ARM_MAX_DOF] = { { 0.0f } };
    s_compute_full_jacobian(q, J6, (s_model.ik.numeric_eps > 0.0f) ? s_model.ik.numeric_eps : 1e-5f);

    bool used[3] = { false, false, false };
    for(uint8_t k = 0u; k < dim; k++) {
        uint8_t best_row = 0u;
        float best_score = -1.0f;
        for(uint8_t r = 0u; r < 3u; r++) {
            if(used[r])
                continue;
            float score = 0.0f;
            for(uint8_t c = 0u; c < s_model.dof; c++)
                score += fabsf(J6[r][c]);
            if(score > best_score) {
                best_score = score;
                best_row = r;
            }
        }
        used[best_row] = true;
        s_task.row[k] = best_row;
    }
}

static bool s_validate_model(const SerialArmModel* model) {
    if(model == NULL)
        return false;
    if(model->dof == 0u || model->dof > SERIAL_ARM_MAX_DOF)
        return false;
    if(model->convention != SERIAL_ARM_DH_STANDARD && model->convention != SERIAL_ARM_DH_MODIFIED)
        return false;
    for(uint8_t i = 0u; i < model->dof; i++) {
        if(model->link[i].q_min > model->link[i].q_max)
            return false;
        if(model->link[i].type != SERIAL_ARM_JOINT_REVOLUTE && model->link[i].type != SERIAL_ARM_JOINT_PRISMATIC)
            return false;
        if(model->link[i].q_sign == 0.0f)
            return false;
    }
    return true;
}

static bool s_validate_joints(const SerialArmJointArray* joints) {
    if(joints == NULL)
        return false;
    if(joints->dof != s_model.dof)
        return false;
    return true;
}

static bool s_validate_task_info(const SerialArmTaskInfo* info) {
    bool used[SERIAL_ARM_TASK_MAX_DIM] = { false };

    if(info == NULL)
        return false;
    if(info->task_dim == 0u || info->task_dim > SERIAL_ARM_TASK_MAX_DIM)
        return false;
    if(info->task_dim > s_model.dof)
        return false;

    for(uint8_t i = 0u; i < info->task_dim; i++) {
        uint8_t row = info->row[i];
        if(row >= SERIAL_ARM_TASK_MAX_DIM)
            return false;
        if(used[row])
            return false;
        used[row] = true;
    }

    return true;
}

static bool s_solve_linear(uint8_t n, float A[SERIAL_ARM_TASK_MAX_DIM][SERIAL_ARM_TASK_MAX_DIM],
                           float b[SERIAL_ARM_TASK_MAX_DIM], float x[SERIAL_ARM_TASK_MAX_DIM]) {
    for(uint8_t i = 0u; i < n; i++)
        x[i] = 0.0f;

    for(uint8_t k = 0u; k < n; k++) {
        uint8_t pivot = k;
        float max_abs = fabsf(A[k][k]);
        for(uint8_t r = (uint8_t)(k + 1u); r < n; r++) {
            float v = fabsf(A[r][k]);
            if(v > max_abs) {
                max_abs = v;
                pivot = r;
            }
        }
        if(max_abs < 1e-9f)
            return false;

        if(pivot != k) {
            for(uint8_t c = 0u; c < n; c++) {
                float tmp = A[k][c];
                A[k][c] = A[pivot][c];
                A[pivot][c] = tmp;
            }
            float tb = b[k];
            b[k] = b[pivot];
            b[pivot] = tb;
        }

        float diag = A[k][k];
        for(uint8_t c = k; c < n; c++)
            A[k][c] /= diag;
        b[k] /= diag;

        for(uint8_t r = 0u; r < n; r++) {
            if(r == k)
                continue;
            float factor = A[r][k];
            for(uint8_t c = k; c < n; c++)
                A[r][c] -= factor * A[k][c];
            b[r] -= factor * b[k];
        }
    }

    for(uint8_t i = 0u; i < n; i++)
        x[i] = b[i];
    return true;
}

static bool s_dls_step(const float J[SERIAL_ARM_TASK_MAX_DIM][SERIAL_ARM_MAX_DOF],
                       const float err[SERIAL_ARM_TASK_MAX_DIM], uint8_t rows, uint8_t cols,
                       float damping, float dq[SERIAL_ARM_MAX_DOF]) {
    float A[SERIAL_ARM_TASK_MAX_DIM][SERIAL_ARM_TASK_MAX_DIM] = { { 0.0f } };
    float y[SERIAL_ARM_TASK_MAX_DIM] = { 0.0f };

    for(uint8_t r = 0u; r < rows; r++) {
        for(uint8_t c = 0u; c < rows; c++) {
            for(uint8_t k = 0u; k < cols; k++)
                A[r][c] += J[r][k] * J[c][k];
        }
        A[r][r] += damping * damping;
        y[r] = err[r];
    }

    float z[SERIAL_ARM_TASK_MAX_DIM] = { 0.0f };
    if(!s_solve_linear(rows, A, y, z))
        return false;

    for(uint8_t c = 0u; c < cols; c++) {
        dq[c] = 0.0f;
        for(uint8_t r = 0u; r < rows; r++)
            dq[c] += J[r][c] * z[r];
    }
    return true;
}

static bool s_solution_is_unique(const SerialArmJointSolutions* sols, const SerialArmJointArray* candidate) {
    const float thresh = 0.05f;
    for(uint8_t i = 0u; i < sols->num_solutions; i++) {
        bool same = true;
        for(uint8_t j = 0u; j < candidate->dof; j++) {
            if(fabsf(s_wrap_pi(sols->solution[i].q[j] - candidate->q[j])) > thresh) {
                same = false;
                break;
            }
        }
        if(same)
            return false;
    }
    return true;
}

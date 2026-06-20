/**
 * @file arm.c
 * @brief 五自由度舵机机械臂服务层实现
 */
#include "arm.h"

#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 机械臂服务内部可变运行实例
 */
static Arm s_arm = { 0 };
/**
 * @brief 机械臂服务对外只读快照
 */
static Arm s_arm_view = { 0 };

/**
 * @brief 机械臂服务状态刷新失败的循环保护
 *
 * 如果连续多次刷新失败, 则会暂时停止刷新尝试;
 * 直到下一次成功刷新后才会恢复正常刷新
 */
static uint8_t s_refresh_fallback_index = 0u;

/**
 * @brief 机械臂服务状态刷新失败的循环保护阈值
 *
 * 如果连续刷新失败次数超过该阈值, 则会触发循环保护机制
 */
static bool s_refresh_fallback_seen[ARM_DOF] = { false };

/**
 * @brief 机械臂服务统一接口实例
 */
const struct ArmInterface arm_interface = {
#define X(name, str) .name = ARM_##name,
    { ARM_STATUS_TABLE },
#undef X
    .default_config = arm_default_config,
    .init = arm_init,
    .move_joints = arm_move_joints,
    .move_joint = arm_move_joint,
    .move_servo_zero = arm_move_servo_zero,
    .move_mdh_zero = arm_move_mdh_zero,
    .move_pose = arm_move_pose,
    .move_position = arm_move_position,
    .move_orientation = arm_move_orientation,
    .stop = arm_stop,
    .enable = arm_enable,
    .fk = arm_fk,
    .ik = arm_ik,
    .is_ready = arm_is_ready,
    .get_arm = arm_get_arm,
    .refresh_current_state = arm_refresh_current_state,
    .get_current_joints = arm_get_current_joints,
    .get_current_pose = arm_get_current_pose,
    .status_str = arm_status_str,
};

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 检查服务初始化配置是否合法
 * @param config 初始化配置
 * @return bool `true` 表示配置合法
 */
static bool s_config_is_valid(const ArmConfig* config);
/**
 * @brief 检查关节索引是否合法
 * @param joint_index 关节索引
 * @return bool `true` 表示索引合法
 */
static bool s_joint_index_valid(uint8_t joint_index);
/**
 * @brief 检查关节数组是否合法
 * @param joints 关节数组
 * @return bool `true` 表示数组合法
 */
static bool s_joint_array_valid(const FiveDofArmJointArray* joints);
/**
 * @brief 检查指定关节角是否落在数学模型限位内
 * @param index 关节索引
 * @param q 关节角，单位 rad
 * @return bool `true` 表示在限位内
 */
static bool s_joint_in_limit(uint8_t index, float q);
/**
 * @brief 检查实物默认零位关节数组是否合法
 * @param joints 零位关节数组
 * @return bool `true` 表示配置合法
 */
static bool s_servo_zero_valid(const FiveDofArmJointArray* joints);

/**
 * @brief 在临时任务行约束下执行一次 IK，并在退出前恢复原任务配置
 * @param target 目标位姿
 * @param joints 输出关节解
 * @param seed IK 初始种子
 * @param task 临时任务行配置；传入 `NULL` 表示沿用当前配置
 * @return ArmStatus 服务状态码
 */
static ArmStatus s_ik_with_task(const FiveDofArmPose* target, FiveDofArmJointArray* joints,
                                const FiveDofArmJointArray* seed, const SerialArmTaskInfo* task);
/**
 * @brief 向舵机层发送一组关节目标
 * @param joints 目标关节数组，单位 rad
 * @param speed_rad_s 目标速度，单位 rad/s
 * @return ArmStatus 服务状态码
 */
static ArmStatus s_send_joints_to_servo(const FiveDofArmJointArray* joints, float speed_rad_s);
/**
 * @brief 用最新目标关节和位姿结果刷新目标缓存
 * @param joints 最新目标关节数组
 * @param pose 最新目标末端位姿
 * @return ArmStatus 服务状态码
 */
static ArmStatus s_update_target_state(const FiveDofArmJointArray* joints, const FiveDofArmPose* pose);
/**
 * @brief 用最新关节和位姿结果刷新服务缓存
 * @param joints 最新关节数组
 * @param pose 最新末端位姿
 * @return ArmStatus 服务状态码
 */
static ArmStatus s_update_current_state(const FiveDofArmJointArray* joints, const FiveDofArmPose* pose);
/**
 * @brief 同步内部运行实例到对外只读快照
 */
static void s_sync_view(void);
/**
 * @brief 获取当前用于连续规划的参考关节角
 * @param joints 输出参考关节角
 * @return ArmStatus 服务状态码
 */
static ArmStatus s_get_reference_joints(FiveDofArmJointArray* joints);
/**
 * @brief 获取当前用于连续规划的参考末端位姿
 * @param pose 输出参考末端位姿
 * @return ArmStatus 服务状态码
 */
static ArmStatus s_get_reference_pose(FiveDofArmPose* pose);
/**
 * @brief 解析最终使用的运动速度
 * @param speed_rad_s 外部请求速度，单位 rad/s
 * @return float 实际使用速度，单位 rad/s
 */
static float s_resolve_speed(float speed_rad_s);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 获取默认初始化配置
 * @return ArmConfig 默认配置
 */
ArmConfig arm_default_config(void) {
    ArmConfig config;
    memset(&config, 0, sizeof(config));

    config.servo_interface = NULL;
    config.stop_servo = NULL;
    config.enable_servo = NULL;
    config.has_kinematic_model = false;
    for(uint8_t i = 0u; i < ARM_DOF; i++) {
        config.servo_id[i] = i;
        config.servo_zero_joints.q[i] = 0.0f;
    }
    config.servo_zero_joints.dof = ARM_DOF;
    config.default_speed_rad_s = 3.14f;
    config.auto_move_servo_zero = true;
    return config;
}

/**
 * @brief 初始化机械臂服务
 * @param config 初始化配置
 * @return ArmStatus 服务状态码
 */
ArmStatus arm_init(const ArmConfig* config) {
    ArmStatus ret;
    FiveDofArmJointArray mdh_zero;

    if(s_config_is_valid(config) == false) {
        return ARM_INVALID_PARAM;
    }

    memset(&s_arm, 0, sizeof(s_arm));
    memset(s_refresh_fallback_seen, 0, sizeof(s_refresh_fallback_seen));
    s_refresh_fallback_index = 0u;
    s_arm.config = *config;
    if(s_arm.config.default_speed_rad_s <= 0.0f) {
        s_arm.config.default_speed_rad_s = 3.14f;
    }

    if(five_dof_arm.init_with_model(&config->kinematic_model) != SERIAL_ARM_STATUS_SUCCESS) {
        return ARM_KINEMATICS_FAILED;
    }

    if(five_dof_arm.get_mdh_zero(&mdh_zero) != SERIAL_ARM_STATUS_SUCCESS) {
        return ARM_KINEMATICS_FAILED;
    }
    if(five_dof_arm.fk(&mdh_zero, &s_arm.current_pose) != SERIAL_ARM_STATUS_SUCCESS) {
        return ARM_KINEMATICS_FAILED;
    }
    s_arm.target_joints = mdh_zero;
    s_arm.target_pose = s_arm.current_pose;
    s_arm.target_valid = true;
    s_arm.current_joints = mdh_zero;
    s_arm.current_valid = false;
    s_arm.initialized = true;
    s_sync_view();

    if(config->auto_move_servo_zero) {
        ret = arm_move_servo_zero(config->default_speed_rad_s);
        if(ret != ARM_OK) {
            return ret;
        }
    }

    return ARM_OK;
}

/**
 * @brief 按关节数组下发整机目标
 * @param joints 目标关节角，单位 rad
 * @param speed_rad_s 目标速度，单位 rad/s
 * @return ArmStatus 服务状态码
 */
ArmStatus arm_move_joints(const FiveDofArmJointArray* joints, float speed_rad_s) {
    FiveDofArmPose pose;
    ArmStatus ret;

    if(s_arm.initialized == false)
        return ARM_NOT_INITIALIZED;
    if(s_joint_array_valid(joints) == false)
        return ARM_INVALID_PARAM;

    for(uint8_t i = 0u; i < ARM_DOF; i++) {
        if(s_joint_in_limit(i, joints->q[i]) == false) {
            return ARM_OUT_OF_LIMIT;
        }
    }

    if(five_dof_arm.fk(joints, &pose) != SERIAL_ARM_STATUS_SUCCESS) {
        return ARM_KINEMATICS_FAILED;
    }

    ret = s_send_joints_to_servo(joints, s_resolve_speed(speed_rad_s));
    if(ret != ARM_OK)
        return ret;

    return s_update_target_state(joints, &pose);
}

/**
 * @brief 控制单个关节运动
 * @param joint_index 关节索引
 * @param target_rad 目标关节角，单位 rad
 * @param speed_rad_s 目标速度，单位 rad/s
 * @return ArmStatus 服务状态码
 */
ArmStatus arm_move_joint(uint8_t joint_index, float target_rad, float speed_rad_s) {
    FiveDofArmJointArray joints;

    if(s_arm.initialized == false)
        return ARM_NOT_INITIALIZED;
    if(s_joint_index_valid(joint_index) == false)
        return ARM_INVALID_PARAM;
    if(s_joint_in_limit(joint_index, target_rad) == false)
        return ARM_OUT_OF_LIMIT;

    if(s_get_reference_joints(&joints) != ARM_OK)
        return ARM_KINEMATICS_FAILED;

    joints.q[joint_index] = target_rad;
    return arm_move_joints(&joints, speed_rad_s);
}

/**
 * @brief 运动到实物机械臂默认零位
 * @param speed_rad_s 目标速度，单位 rad/s
 * @return ArmStatus 服务状态码
 */
ArmStatus arm_move_servo_zero(float speed_rad_s) {
    if(s_arm.initialized == false)
        return ARM_NOT_INITIALIZED;
    if(s_servo_zero_valid(&s_arm.config.servo_zero_joints) == false)
        return ARM_INVALID_PARAM;

    return arm_move_joints(&s_arm.config.servo_zero_joints, speed_rad_s);
}

/**
 * @brief 运动到数学 MDH 零位
 * @param speed_rad_s 目标速度，单位 rad/s
 * @return ArmStatus 服务状态码
 */
ArmStatus arm_move_mdh_zero(float speed_rad_s) {
    FiveDofArmJointArray joints;

    if(s_arm.initialized == false)
        return ARM_NOT_INITIALIZED;
    if(five_dof_arm.get_mdh_zero(&joints) != SERIAL_ARM_STATUS_SUCCESS) {
        return ARM_KINEMATICS_FAILED;
    }
    return arm_move_joints(&joints, speed_rad_s);
}

/**
 * @brief 按目标位姿求逆解并执行运动
 * @param target 目标位姿
 * @param speed_rad_s 目标速度，单位 rad/s
 * @return ArmStatus 服务状态码
 */
ArmStatus arm_move_pose(const FiveDofArmPose* target, float speed_rad_s) {
    FiveDofArmJointArray seed;
    FiveDofArmJointArray joints;
    ArmStatus ik_ret;

    if(s_arm.initialized == false)
        return ARM_NOT_INITIALIZED;
    if(target == NULL)
        return ARM_INVALID_PARAM;

    if(s_get_reference_joints(&seed) != ARM_OK)
        return ARM_KINEMATICS_FAILED;

    ik_ret = s_ik_with_task(target, &joints, &seed, NULL);
    if(ik_ret != ARM_OK)
        return ik_ret;

    return arm_move_joints(&joints, speed_rad_s);
}

/**
 * @brief 仅修改末端目标位置并保持当前姿态
 * @param x 目标 x，单位 m
 * @param y 目标 y，单位 m
 * @param z 目标 z，单位 m
 * @param speed_rad_s 目标速度，单位 rad/s
 * @return ArmStatus 服务状态码
 */
ArmStatus arm_move_position(float x, float y, float z, float speed_rad_s) {
    FiveDofArmPose target;
    FiveDofArmJointArray seed;
    FiveDofArmJointArray joints;
    SerialArmTaskInfo task = {
        .task_dim = 3u,
        .row = { 0u, 1u, 2u },
    };
    ArmStatus ik_ret;

    if(s_arm.initialized == false)
        return ARM_NOT_INITIALIZED;

    if(s_get_reference_pose(&target) != ARM_OK)
        return ARM_KINEMATICS_FAILED;

    target.position.x = x;
    target.position.y = y;
    target.position.z = z;

    if(s_get_reference_joints(&seed) != ARM_OK)
        return ARM_KINEMATICS_FAILED;

    ik_ret = s_ik_with_task(&target, &joints, &seed, &task);
    if(ik_ret != ARM_OK)
        return ik_ret;

    return arm_move_joints(&joints, speed_rad_s);
}

/**
 * @brief 运动到目标末端姿态，保持当前末端位置
 * @param roll 目标 roll，单位 rad
 * @param pitch 目标 pitch，单位 rad
 * @param yaw 目标 yaw，单位 rad
 * @param speed_rad_s 目标速度，单位 rad/s
 * @return ArmStatus 服务状态码
 */
ArmStatus arm_move_orientation(float roll, float pitch, float yaw, float speed_rad_s) {
    FiveDofArmPose target;
    FiveDofArmJointArray seed;
    FiveDofArmJointArray joints;
    SerialArmTaskInfo task = {
        .task_dim = 3u,
        .row = { 3u, 4u, 5u },
    };
    SerialArmStatus ret;
    ArmStatus ik_ret;

    if(s_arm.initialized == false)
        return ARM_NOT_INITIALIZED;

    if(s_get_reference_pose(&target) != ARM_OK)
        return ARM_KINEMATICS_FAILED;

    ret = serial_arm.pose_from_xyz_rpy(
        target.position.x,
        target.position.y,
        target.position.z,
        roll,
        pitch,
        yaw,
        &target);
    if(ret != SERIAL_ARM_STATUS_SUCCESS) {
        return ARM_KINEMATICS_FAILED;
    }

    if(s_get_reference_joints(&seed) != ARM_OK)
        return ARM_KINEMATICS_FAILED;

    ik_ret = s_ik_with_task(&target, &joints, &seed, &task);
    if(ik_ret != ARM_OK)
        return ik_ret;

    return arm_move_joints(&joints, speed_rad_s);
}

/**
 * @brief 停止全部舵机
 * @return ArmStatus 服务状态码
 */
ArmStatus arm_stop(void) {
    if(s_arm.initialized == false)
        return ARM_NOT_INITIALIZED;

    if(s_arm.config.stop_servo == NULL) {
        return ARM_DEPENDENCY_MISSING;
    }

    for(uint8_t i = 0u; i < ARM_DOF; i++) {
        if(s_arm.config.stop_servo(s_arm.config.servo_id[i]) != SERVO_STATUS_OK) {
            return ARM_SERVO_FAILED;
        }
    }

    return ARM_OK;
}

/**
 * @brief 使能全部舵机扭矩
 * @return ArmStatus 服务状态码
 */
ArmStatus arm_enable(void) {
    if(s_arm.initialized == false)
        return ARM_NOT_INITIALIZED;

    if(s_arm.config.enable_servo == NULL) {
        return ARM_DEPENDENCY_MISSING;
    }

    for(uint8_t i = 0u; i < ARM_DOF; i++) {
        if(s_arm.config.enable_servo(s_arm.config.servo_id[i]) != SERVO_STATUS_OK) {
            return ARM_SERVO_FAILED;
        }
    }

    return ARM_OK;
}

/**
 * @brief 计算正运动学
 * @param joints 输入关节数组，单位 rad
 * @param pose 输出位姿
 * @return ArmStatus 服务状态码
 */
ArmStatus arm_fk(const FiveDofArmJointArray* joints, FiveDofArmPose* pose) {
    if(s_arm.initialized == false)
        return ARM_NOT_INITIALIZED;
    if(s_joint_array_valid(joints) == false || pose == NULL)
        return ARM_INVALID_PARAM;
    return (five_dof_arm.fk(joints, pose) == SERIAL_ARM_STATUS_SUCCESS) ? ARM_OK : ARM_KINEMATICS_FAILED;
}

/**
 * @brief 计算逆运动学
 * @param target 目标位姿
 * @param joints 输出关节数组
 * @param seed 逆解初值
 * @return ArmStatus 服务状态码
 */
ArmStatus arm_ik(const FiveDofArmPose* target, FiveDofArmJointArray* joints, const FiveDofArmJointArray* seed) {
    SerialArmStatus ret;

    if(s_arm.initialized == false)
        return ARM_NOT_INITIALIZED;
    if(target == NULL || joints == NULL || s_joint_array_valid(seed) == false)
        return ARM_INVALID_PARAM;

    ret = five_dof_arm.ik(target, joints, seed);
    if(ret == SERIAL_ARM_STATUS_SUCCESS)
        return ARM_OK;
    if(ret == SERIAL_ARM_STATUS_NO_SOLUTION || ret == SERIAL_ARM_STATUS_OUT_OF_REACH)
        return ARM_NO_SOLUTION;
    return ARM_KINEMATICS_FAILED;
}

/**
 * @brief 查询服务是否已经初始化完成
 * @return bool `true` 表示服务可用
 */
bool arm_is_ready(void) {
    return s_arm.initialized;
}

/**
 * @brief 获取机械臂服务只读视图
 * @return const Arm* 只读视图指针
 */
const Arm* arm_get_arm(void) {
    s_sync_view();
    return &s_arm_view;
}

/**
 * @brief 刷新当前状态缓存
 * @return ArmStatus 服务状态码
 */
ArmStatus arm_refresh_current_state(void) {
    FiveDofArmJointArray joints = { 0 };
    FiveDofArmPose pose;
    BusServoFeedback feedbacks[ARM_DOF];

    if(s_arm.initialized == false)
        return ARM_NOT_INITIALIZED;
    if(s_arm.config.servo_interface == NULL || s_arm.config.servo_interface->update_feedback == NULL)
        return ARM_DEPENDENCY_MISSING;

    joints.dof = ARM_DOF;
    if(s_arm.config.batch_update_feedback != NULL) {
        BusServoStatus ret = s_arm.config.batch_update_feedback(s_arm.config.servo_id, ARM_DOF, feedbacks, ARM_DOF);
        if(ret != SERVO_STATUS_OK)
            return ARM_SERVO_FAILED;
        for(uint8_t i = 0u; i < ARM_DOF; i++) {
            joints.q[i] = feedbacks[i].position;
            s_refresh_fallback_seen[i] = true;
        }
    }
    else {
        BusServoFeedback feedback;
        uint8_t index = s_refresh_fallback_index;
        BusServoStatus ret = s_arm.config.servo_interface->update_feedback(s_arm.config.servo_id[index], &feedback);
        if(ret != SERVO_STATUS_OK)
            return ARM_SERVO_FAILED;

        s_refresh_fallback_seen[index] = true;
        s_refresh_fallback_index = (uint8_t)((index + 1u) % ARM_DOF);

        /*
         * Fallback drivers may still use blocking single-servo reads. Refresh
         * only one servo per call so the 500Hz loop is never held by five
         * serial waits; update FK after every servo has a valid cached value.
         */
        for(uint8_t i = 0u; i < ARM_DOF; i++) {
            if(s_refresh_fallback_seen[i] == false) {
                return ARM_OK;
            }
            joints.q[i] = s_arm.config.servo_interface->get_position(s_arm.config.servo_id[i]);
        }
    }

    if(five_dof_arm.fk(&joints, &pose) != SERIAL_ARM_STATUS_SUCCESS)
        return ARM_KINEMATICS_FAILED;

    return s_update_current_state(&joints, &pose);
}

/**
 * @brief 获取最近一次缓存的关节角
 * @return const FiveDofArmJointArray* 关节缓存指针
 */
const FiveDofArmJointArray* arm_get_current_joints(void) {
    return s_arm.current_valid ? &s_arm.current_joints : NULL;
}

/**
 * @brief 获取最近一次缓存的末端位姿
 * @return const FiveDofArmPose* 位姿缓存指针
 */
const FiveDofArmPose* arm_get_current_pose(void) {
    return s_arm.current_valid ? &s_arm.current_pose : NULL;
}

/**
 * @brief 将服务状态码转换为字符串
 * @param status 状态码
 * @return const char* 静态字符串
 */
const char* arm_status_str(ArmStatus status) {
    switch(status) {
#define X(name, str) \
    case ARM_##name: \
        return str;
        ARM_STATUS_TABLE
#undef X
        default:
            return "Unknown Arm Status";
    }
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 检查服务初始化配置是否合法
 * @param config 初始化配置
 * @return bool `true` 表示配置合法
 */
static bool s_config_is_valid(const ArmConfig* config) {
    if(config == NULL)
        return false;
    if(config->servo_interface == NULL)
        return false;
    if(config->servo_interface->set_pos_spd == NULL)
        return false;
    if(config->servo_interface->get_position == NULL)
        return false;
    if(config->has_kinematic_model == false || config->kinematic_model.dof != ARM_DOF)
        return false;
    if(s_servo_zero_valid(&config->servo_zero_joints) == false)
        return false;
    return true;
}

/**
 * @brief 检查关节索引是否合法
 * @param joint_index 关节索引
 * @return bool `true` 表示索引合法
 */
static bool s_joint_index_valid(uint8_t joint_index) {
    return joint_index < ARM_DOF;
}

/**
 * @brief 检查关节数组是否合法
 * @param joints 关节数组
 * @return bool `true` 表示数组合法
 */
static bool s_joint_array_valid(const FiveDofArmJointArray* joints) {
    return joints != NULL && joints->dof == ARM_DOF;
}

/**
 * @brief 检查指定关节角是否落在数学模型限位内
 * @param index 关节索引
 * @param q 关节角，单位 rad
 * @return bool `true` 表示在限位内
 */
static bool s_joint_in_limit(uint8_t index, float q) {
    const SerialArmModel* model = five_dof_arm.get_model();
    const float eps = 1e-5f;

    if(model == NULL || index >= model->dof)
        return false;
    return (q >= model->link[index].q_min - eps) && (q <= model->link[index].q_max + eps);
}

/**
 * @brief 检查实物默认零位关节数组是否合法
 * @param joints 零位关节数组
 * @return bool `true` 表示配置合法
 */
static bool s_servo_zero_valid(const FiveDofArmJointArray* joints) {
    return s_joint_array_valid(joints);
}

static ArmStatus s_ik_with_task(const FiveDofArmPose* target, FiveDofArmJointArray* joints,
                                const FiveDofArmJointArray* seed, const SerialArmTaskInfo* task) {
    SerialArmStatus ret;
    SerialArmTaskInfo saved_task;
    bool task_switched = false;

    if(target == NULL || joints == NULL || seed == NULL)
        return ARM_INVALID_PARAM;

    if(task != NULL) {
        ret = serial_arm.get_task_info(&saved_task);
        if(ret != SERIAL_ARM_STATUS_SUCCESS)
            return ARM_KINEMATICS_FAILED;

        ret = serial_arm.set_task_info(task);
        if(ret != SERIAL_ARM_STATUS_SUCCESS)
            return ARM_KINEMATICS_FAILED;
        task_switched = true;
    }

    ret = five_dof_arm.ik(target, joints, seed);

    if(task_switched) {
        SerialArmStatus restore_ret = serial_arm.set_task_info(&saved_task);
        if(restore_ret != SERIAL_ARM_STATUS_SUCCESS && ret == SERIAL_ARM_STATUS_SUCCESS) {
            return ARM_KINEMATICS_FAILED;
        }
    }

    if(ret == SERIAL_ARM_STATUS_SUCCESS)
        return ARM_OK;
    if(ret == SERIAL_ARM_STATUS_NO_SOLUTION || ret == SERIAL_ARM_STATUS_OUT_OF_REACH)
        return ARM_NO_SOLUTION;
    return ARM_KINEMATICS_FAILED;
}

/**
 * @brief 向舵机层发送一组关节目标
 * @param joints 目标关节数组，单位 rad
 * @param speed_rad_s 目标速度，单位 rad/s
 * @return ArmStatus 服务状态码
 */
static ArmStatus s_send_joints_to_servo(const FiveDofArmJointArray* joints, float speed_rad_s) {
    if(s_arm.config.batch_set_pos_spd != NULL) {
        BusServoStatus ret = s_arm.config.batch_set_pos_spd(s_arm.config.servo_id, joints->q, ARM_DOF, speed_rad_s);
        return (ret == SERVO_STATUS_OK) ? ARM_OK : ARM_SERVO_FAILED;
    }

    for(uint8_t i = 0u; i < ARM_DOF; i++) {
        BusServoStatus ret = s_arm.config.servo_interface->set_pos_spd(
            s_arm.config.servo_id[i], joints->q[i], speed_rad_s);
        if(ret != SERVO_STATUS_OK)
            return ARM_SERVO_FAILED;
    }

    return ARM_OK;
}

/**
 * @brief 用最新目标关节和位姿结果刷新目标缓存
 * @param joints 最新目标关节数组
 * @param pose 最新目标末端位姿
 * @return ArmStatus 服务状态码
 */
static ArmStatus s_update_target_state(const FiveDofArmJointArray* joints, const FiveDofArmPose* pose) {
    if(joints == NULL || pose == NULL)
        return ARM_INVALID_PARAM;

    s_arm.target_joints = *joints;
    s_arm.target_pose = *pose;
    s_arm.target_valid = true;
    s_sync_view();
    return ARM_OK;
}

/**
 * @brief 用最新关节和位姿结果刷新服务缓存
 * @param joints 最新关节数组
 * @param pose 最新末端位姿
 * @return ArmStatus 服务状态码
 */
static ArmStatus s_update_current_state(const FiveDofArmJointArray* joints, const FiveDofArmPose* pose) {
    if(joints == NULL || pose == NULL)
        return ARM_INVALID_PARAM;

    s_arm.current_joints = *joints;
    s_arm.current_pose = *pose;
    s_arm.current_valid = true;
    s_sync_view();
    return ARM_OK;
}

/**
 * @brief 获取当前用于连续规划的参考关节角
 * @param joints 输出参考关节角
 * @return ArmStatus 服务状态码
 */
static ArmStatus s_get_reference_joints(FiveDofArmJointArray* joints) {
    if(joints == NULL)
        return ARM_INVALID_PARAM;

    if(s_arm.current_valid) {
        *joints = s_arm.current_joints;
        return ARM_OK;
    }

    if(s_arm.target_valid) {
        *joints = s_arm.target_joints;
        return ARM_OK;
    }

    return (five_dof_arm.get_mdh_zero(joints) == SERIAL_ARM_STATUS_SUCCESS) ? ARM_OK : ARM_KINEMATICS_FAILED;
}

/**
 * @brief 获取当前用于连续规划的参考末端位姿
 * @param pose 输出参考末端位姿
 * @return ArmStatus 服务状态码
 */
static ArmStatus s_get_reference_pose(FiveDofArmPose* pose) {
    FiveDofArmJointArray joints;

    if(pose == NULL)
        return ARM_INVALID_PARAM;

    if(s_arm.current_valid) {
        *pose = s_arm.current_pose;
        return ARM_OK;
    }

    if(s_arm.target_valid) {
        *pose = s_arm.target_pose;
        return ARM_OK;
    }

    if(five_dof_arm.get_mdh_zero(&joints) != SERIAL_ARM_STATUS_SUCCESS)
        return ARM_KINEMATICS_FAILED;
    if(five_dof_arm.fk(&joints, pose) != SERIAL_ARM_STATUS_SUCCESS)
        return ARM_KINEMATICS_FAILED;

    return ARM_OK;
}

/**
 * @brief 同步内部运行实例到对外只读快照
 */
static void s_sync_view(void) {
    s_arm_view = s_arm;
}

/**
 * @brief 解析最终使用的运动速度
 * @param speed_rad_s 外部请求速度，单位 rad/s
 * @return float 实际使用速度，单位 rad/s
 */
static float s_resolve_speed(float speed_rad_s) {
    if(speed_rad_s > 0.0f)
        return speed_rad_s;
    if(s_arm.config.default_speed_rad_s > 0.0f)
        return s_arm.config.default_speed_rad_s;
    return 3.14f;
}

#include "chassis.h"

#include "bus_motor/agv_motor.h"
#include "log.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 底盘接口单例的文件内短别名
 *
 * 仅用于本文件内部减少重复书写；
 * 对外仍通过 chassis_interface 暴露接口
 */
#define ch chassis_interface

/**
 * @brief 默认轮角速度到驱动电机角速度的传动比
 *
 * 当前机械结构按一比一处理；
 * 若后续更换传动结构，可通过配置覆盖
 */
#define CHASSIS_DEFAULT_WHEEL_DRIVE_RATIO       1.0f
/**
 * @brief 转向电机 S 曲线规划后的最大跟踪速度
 *
 * 单位为 rad/s；
 * 该值越大转向越快，但也更容易引入抖动
 */
#define CHASSIS_STEER_TRACK_MAX_SPEED_RAD_S     12.56f
/**
 * @brief 转向电机 S 曲线规划使用的最低跟踪速度
 *
 * 单位为 rad/s；
 * 用于避免速度过低导致转向响应迟滞
 */
#define CHASSIS_STEER_TRACK_MIN_SPEED_RAD_S     1.57f
/**
 * @brief 转向跟踪速度从最低值爬升到最高值的时间
 *
 * 单位为秒；
 * 该时间决定 S 曲线加速段的柔和程度
 */
#define CHASSIS_STEER_SPEED_RAMP_TIME_S         0.1f
/**
 * @brief 转向接近目标角时开始降速的角度窗口
 *
 * 单位为 rad；
 * 误差进入该窗口后速度会平滑降低
 */
#define CHASSIS_STEER_SLOWDOWN_ANGLE_RAD        0.628f
/**
 * @brief 底盘控制任务周期
 *
 * 单位为秒；
 * 需与 TIM6 触发周期保持一致
 */
#define CHASSIS_CONTROL_PERIOD_S                0.002f
/**
 * @brief 判定转向目标明显变化的角度阈值
 *
 * 单位为 rad；
 * 超过该阈值会启动 S 曲线速度规划
 */
#define CHASSIS_STEER_TARGET_CHANGE_RAD         0.0628f
/**
 * @brief 允许驱动电机出力的转向角误差阈值
 *
 * 单位为 rad；
 * 未进入该误差范围前，驱动速度会被置零以防漂移
 */
#define CHASSIS_DRIVE_ANGLE_TOL_RAD             0.0628f
/**
 * @brief 驻车刹车流程使用的转向到位误差阈值
 *
 * 单位为 rad；
 * 转向角到位后才会进入电机抱死状态
 */
#define CHASSIS_BRAKE_ANGLE_TOL_RAD             0.0628f
/**
 * @brief 圆周率常量
 *
 * 用于角度归一化和等效角计算；
 * 保持 float 精度即可满足底盘控制需求
 */
#define CHASSIS_PI                              3.14159265358979323846f
/**
 * @brief 转向角完整周期
 *
 * 单位为 rad；
 * 用于选择离当前位置最近的等效转向角
 */
#define CHASSIS_2PI                             (2.0f * CHASSIS_PI)
/**
 * @brief 转向电机位置命令的安全绝对边界
 *
 * 单位为 rad；
 * 目标角会被限制在该范围内
 */
#define CHASSIS_STEER_POS_LIMIT_RAD             12.4f
/**
 * @brief 等效转向角切换滞回
 *
 * 单位为 rad；
 * 用于避免目标角在两个等效解之间来回跳变
 */
#define CHASSIS_EQUIV_ANGLE_HYST_RAD            0.12f
/**
 * @brief 驱动方向等效优化的滞回角
 *
 * 单位为 rad；
 * 用于避免前进/后退等效解在临界角附近抖动
 */
#define CHASSIS_DRIVE_EQUIV_HYST_RAD            0.03f
/**
 * @brief 转向电机上电准备重试间隔
 *
 * 单位为底盘控制周期；
 * 启动未收到反馈时会按该间隔重新发送准备序列
 */
#define CHASSIS_STEER_PREPARE_RETRY_CYCLES      250u
/**
 * @brief 驱动电机上电准备重试间隔
 *
 * 单位为底盘控制周期；
 * 启动未收到反馈时会按该间隔重新发送准备序列
 */
#define CHASSIS_DRIVE_PREPARE_RETRY_CYCLES      250u

/**
 * @brief 逻辑舵轮模块到物理 CAN 电机的映射关系
 *
 * 每个模块包含一个转向电机和一个驱动电机；
 * drive_sign 用于修正驱动电机安装方向
 */
typedef struct {
    ChassisModule module; /**< 舵轮模块逻辑编号 */
    uint8_t steer_id;     /**< 转向电机 CAN ID */
    uint8_t drive_id;     /**< 驱动电机 CAN ID */
    int8_t drive_sign;    /**< 驱动电机安装方向修正符号，取值为 1 或 -1 */
} ChassisModuleMap;

/**
 * @brief 底盘服务内部可变运行实例
 *
 * 控制流程只修改该实例；
 * 对外读取使用同步后的只读快照
 */
static Chassis s_chassis = { 0 };
/**
 * @brief 对外暴露的底盘只读快照
 *
 * 快照中的坐标和状态用于上层查询；
 * 避免直接暴露内部控制过程中的中间状态
 */
static Chassis s_chassis_view = { 0 };
/**
 * @brief 每个舵轮的 S 曲线加速累计时间
 *
 * 单位为秒；
 * 目标角明显变化时会重新计时
 */
static float s_steer_speed_ramp_time[CHASSIS_MODULE_COUNT] = { 0.0f };
/**
 * @brief 每个舵轮上一次用于速度规划的目标转向角
 *
 * 单位为 rad；
 * 用于判断是否需要重启 S 曲线
 */
static float s_steer_speed_last_target[CHASSIS_MODULE_COUNT] = { 0.0f };
/**
 * @brief 每个舵轮速度规划状态的初始化标志
 *
 * 首次进入控制时需要记录当前目标；
 * 后续才可根据目标变化计算 S 曲线
 */
static uint8_t s_steer_speed_initialized[CHASSIS_MODULE_COUNT] = { 0u };
/**
 * @brief 转向电机上电准备重试倒计时
 *
 * 单位为底盘控制周期；
 * 倒计时归零后会再次发送初始化准备序列
 */
static uint16_t s_steer_prepare_retry_countdown = 0u;
/**
 * @brief 驱动电机上电准备重试倒计时
 *
 * 单位为底盘控制周期；
 * 倒计时归零后会再次发送初始化准备序列
 */
static uint16_t s_drive_prepare_retry_countdown = 0u;
/**
 * @brief 上一次记录的转向反馈缺失掩码
 *
 * 每一位对应一个舵轮模块；
 * 仅当掩码变化时才打印日志以避免刷屏
 */
static uint8_t s_last_steer_missing_mask = 0xFFu;
/**
 * @brief 上一次记录的驱动反馈缺失掩码
 *
 * 每一位对应一个舵轮模块；
 * 仅当掩码变化时才打印日志以避免刷屏
 */
static uint8_t s_last_drive_missing_mask = 0xFFu;
/**
 * @brief 展开舵轮模块表生成物理电机映射项
 *
 * 该宏只在 s_module_map 初始化时使用；
 * 展开后立即取消定义
 */
#define X(name, index, steer_id, drive_id) [CHASSIS_MODULE_##name] = { CHASSIS_MODULE_##name, (steer_id), (uint8_t)((drive_id) + 1u), (((drive_id) == 1 || (drive_id) == 2) ? -1 : 1) },
/**
 * @brief 四个舵轮模块的固定电机映射表
 *
 * 表内顺序与 ChassisModule 枚举保持一致；
 * 控制循环按该表访问转向和驱动电机
 */
static const ChassisModuleMap s_module_map[CHASSIS_MODULE_COUNT] = {
    CHASSIS_MODULE_TABLE
};
#undef X

/**
 * @brief 展开底盘状态表到接口结构体常量
 *
 * 该宏用于生成 chassis.OK 等便捷状态码字段；
 * 展开后立即取消定义
 */
#define X(name, str) .name = CHASSIS_##name,
/**
 * @brief 底盘服务对外接口单例
 *
 * 上层通过 chassis.xxx 调用这些函数；
 * 实际实现均位于本文件中
 */
const struct ChassisInterface chassis_interface = {
    {
        CHASSIS_STATUS_TABLE
    },
    .init = chassis_init,
    .set_velocity = chassis_set_velocity,
    .set_steer_then_drive_enabled = chassis_set_steer_then_drive_enabled,
    .process = chassis_process,
    .stop = chassis_stop,
    .brake = chassis_brake,
    .is_ready = chassis_is_ready,
    .get_chassis = chassis_get_chassis,
    .get_state = chassis_get_state,
    .get_control = chassis_get_control,
    .error_code_to_str = chassis_error_code_to_str
};
#undef X

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 将车轮角速度转换为驱动电机角速度
 *
 * 转换会应用当前底盘配置中的传动比；
 * 单位均为 rad/s
 *
 * @param wheel_omega 车轮角速度
 * @return float 驱动电机角速度
 */
static float chassis_wheel_omega_to_drive_omega(float wheel_omega);

/**
 * @brief 将驱动电机角速度转换为车轮角速度
 *
 * 转换会应用当前底盘配置中的传动比；
 * 单位均为 rad/s
 *
 * @param drive_omega 驱动电机角速度
 * @return float 车轮角速度
 */
static float chassis_drive_omega_to_wheel_omega(float drive_omega);

/**
 * @brief 检查底盘初始化配置是否合法
 *
 * 函数会检查抽象电机实例、端口操作表、启动回调、几何尺寸和传动比；
 * 任何无效参数都会阻止底盘初始化
 *
 * @param config 待检查的底盘配置指针
 * @return ChassisErrorCode 配置检查结果
 */
static ChassisErrorCode chassis_check_config(const ChassisConfig* config);

/**
 * @brief 向所有转向电机发送上电准备序列
 *
 * 具体序列由 assemble 层注册的回调决定；
 * 单个电机失败不会阻止后续电机继续尝试
 *
 * @return ChassisErrorCode 所有电机是否都接受了命令
 */
static ChassisErrorCode chassis_prepare_steer_motors(void);
static ChassisErrorCode chassis_prepare_drive_motors(void);

/**
 * @brief 将外部底盘速度命令转换到内部坐标系
 *
 * 遥控和上层接口使用外部约定坐标系；
 * 运动学求解前需要转换为内部坐标系
 *
 * @param vx_ext 外部坐标系 x 方向线速度
 * @param vy_ext 外部坐标系 y 方向线速度
 * @param wz_ext 外部坐标系 z 轴角速度
 * @param vx_int 输出的内部坐标系 x 方向线速度
 * @param vy_int 输出的内部坐标系 y 方向线速度
 * @param wz_int 输出的内部坐标系 z 轴角速度
 */
static void chassis_external_to_internal_twist(float vx_ext, float vy_ext, float wz_ext, float* vx_int, float* vy_int, float* wz_int);

/**
 * @brief 将内部底盘速度反馈转换到外部坐标系
 *
 * 运动学状态在内部坐标系中维护；
 * 对外展示前需要转换为用户约定坐标系
 *
 * @param vx_int 内部坐标系 x 方向线速度
 * @param vy_int 内部坐标系 y 方向线速度
 * @param wz_int 内部坐标系 z 轴角速度
 * @param vx_ext 输出的外部坐标系 x 方向线速度
 * @param vy_ext 输出的外部坐标系 y 方向线速度
 * @param wz_ext 输出的外部坐标系 z 轴角速度
 */
static void chassis_internal_to_external_twist(float vx_int, float vy_int, float wz_int, float* vx_ext, float* vy_ext, float* wz_ext);

/**
 * @brief 将浮点数限制在闭区间内
 *
 * 该工具函数用于限幅角度和归一化参数；
 * 若输入超出边界则返回对应边界值
 *
 * @param value 输入值
 * @param min 下边界
 * @param max 上边界
 * @return float 限幅后的结果
 */
static float chassis_clampf(float value, float min, float max);

/**
 * @brief 将角度归一化到 [-pi, pi] 区间
 *
 * 单位为 rad；
 * 该函数用于计算最短角度误差
 *
 * @param angle 输入角度
 * @return float 归一化后的角度
 */
static float chassis_wrap_pi(float angle);

/**
 * @brief 计算归一化输入的 smoothstep 曲线值
 *
 * 输入会被限制到 [0, 1]；
 * 输出用于生成转向速度 S 曲线
 *
 * @param x 归一化输入值
 * @return float 平滑后的曲线值
 */
static float chassis_smoothstep(float x);

/**
 * @brief 选择距离当前角最近的周期等效目标角
 *
 * 转向电机角度可以跨越多个 2pi 周期；
 * 该函数选择无需绕远路的等效角
 *
 * @param current_angle 当前转向角，单位 rad
 * @param target_angle 期望转向角，单位 rad
 * @return float 最近的周期等效目标角
 */
static float chassis_select_nearest_cyclic_angle(float current_angle, float target_angle);

/**
 * @brief 带滞回地选择最近等效转向目标角
 *
 * 函数会在直接角和反向驱动等效角之间选择；
 * 滞回用于避免临界点附近反复切换
 *
 * @param current_angle 当前转向角，单位 rad
 * @param target_angle 期望转向角，单位 rad
 * @param previous_target_angle 上一次规划的目标角，单位 rad
 * @return float 本周期选定的等效目标角
 */
static float chassis_select_nearest_equivalent_angle(float current_angle, float target_angle, float previous_target_angle);

/**
 * @brief 结合驱动方向反转优化单个舵轮目标
 *
 * 当前进/后退反向能显著减少转向角时；
 * 函数会翻转轮速并选择更近的转向目标
 *
 * @param module 需要优化的舵轮模块
 */
static void chassis_optimize_drive_module_target(ChassisModule module);

/**
 * @brief 重置所有转向速度 S 曲线规划状态
 *
 * 停止、刹车或目标明显变化时调用；
 * 下次控制周期会重新建立速度规划起点
 */
static void chassis_reset_steer_speed_profiles(void);

/**
 * @brief 计算单个舵轮当前转向跟踪速度
 *
 * 速度同时受启动加速曲线和接近目标降速曲线影响；
 * 输出会限制在最小和最大跟踪速度之间
 *
 * @param module 舵轮模块
 * @param target_angle 当前目标转向角，单位 rad
 * @return float 转向跟踪速度命令，单位 rad/s
 */
static float chassis_calc_steer_track_speed(ChassisModule module, float target_angle);

/**
 * @brief 立即停止所有驱动电机
 *
 * 启动未就绪或遥控离线时调用；
 * 只影响驱动电机，不改变转向目标
 */
static void chassis_stop_all_drive_motors(void);

/**
 * @brief 在转向反馈缺失时重试上电准备序列
 *
 * 函数负责维护 ready 状态；
 * 未就绪时会要求底盘保持驱动输出为零
 *
 * @param steer_feedback_ready 四个转向电机是否均有有效反馈
 * @param missing_mask 缺失反馈的舵轮模块位掩码
 * @return true 可以继续正常底盘控制
 * @return false 应保持驱动输出为零
 */
static bool chassis_maintain_motor_startup(
    bool steer_feedback_observed,
    uint8_t steer_missing_mask,
    bool drive_feedback_observed,
    uint8_t drive_missing_mask);

/**
 * @brief 检查转向目标是否已接近到允许驱动输出
 *
 * 该函数实现“先转向再驱动”的门控；
 * 未到位时轮速命令不会下发给驱动电机
 *
 * @return true 所有舵轮均在允许误差内
 * @return false 至少一个舵轮尚未对准
 */
static bool chassis_drive_targets_reached(void);

/**
 * @brief 设置驻车刹车流程使用的固定转向目标
 *
 * 驻车时四个舵轮会转到预设角度；
 * 到位后再抱死转向电机
 */
static void chassis_set_brake_targets(void);

/**
 * @brief 检查驻车刹车转向目标是否已到位
 *
 * 函数根据当前转向角和刹车误差阈值判断；
 * 到位后刹车流程可以进入抱死状态
 *
 * @return true 驻车转向目标已到位
 * @return false 驻车转向目标仍在调整
 */
static bool chassis_brake_targets_reached(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化底盘
 * @param config 底盘配置
 * @return ChassisErrorCode 状态码
 */
ChassisErrorCode chassis_init(const ChassisConfig* config) {
    BusMotorConfig steer_config = {
        .ops = config != NULL ? config->steer_ops : NULL,
        .timeout_ms = 0u,
        .retry_count = 0u,
    };
    BusMotorConfig drive_config = {
        .ops = config != NULL ? config->drive_ops : NULL,
        .timeout_ms = 0u,
        .retry_count = 0u,
    };
    ChassisErrorCode config_status = chassis_check_config(config);

    log_info("CHASSIS init begin");

    if(config_status != ch.OK) {
        log_error("CHASSIS config check failed: %s", chassis_error_code_to_str(config_status));
        return config_status;
    }

    if(config->steer_motor_interface == NULL || config->drive_motor_interface == NULL
        || config->steer_ops == NULL || config->drive_ops == NULL
        || config->prepare_steer_motor == NULL || config->prepare_drive_motor == NULL) {
        log_error("CHASSIS dependency missing before init");
        return ch.DEPENDENCY_MISSING;
    }

    if(steer_motor_set_instance(config->steer_motor_interface) != MOTOR_STATUS_OK
        || drive_motor_set_instance(config->drive_motor_interface) != MOTOR_STATUS_OK) {
        log_error("CHASSIS motor instance bind failed");
        return ch.INVALID_PARAM;
    }

    s_chassis.config = *config;
    s_chassis.brake_requested = 0u;
    s_chassis.brake_latched = 0u;
    s_chassis.steer_then_drive_enabled = 1u;
    s_chassis.initialized = 0u;
    chassis_reset_steer_speed_profiles();
    s_steer_prepare_retry_countdown = 0u;
    s_drive_prepare_retry_countdown = 0u;
    s_last_steer_missing_mask = 0xFFu;
    s_last_drive_missing_mask = 0xFFu;
    s_chassis.steer_motor_ready = 0u;
    s_chassis.drive_motor_ready = 0u;

    if(steer_motor.init(&steer_config) != MOTOR_STATUS_OK) {
        log_error("CHASSIS steer motor device init failed");
        return ch.INVALID_PARAM;
    }
    if(drive_motor.init(&drive_config) != MOTOR_STATUS_OK) {
        log_error("CHASSIS drive motor device init failed");
        return ch.INVALID_PARAM;
    }

    if(swheel.init(&s_chassis.kine, config->model) != swheel.OK) {
        log_error("CHASSIS kinematics init failed");
        return ch.KINEMATICS_FAILED;
    }

    if(chassis_prepare_steer_motors() != ch.OK) {
        log_warn("CHASSIS initial steer prepare incomplete, will retry in process");
    }
    if(chassis_prepare_drive_motors() != ch.OK) {
        log_warn("CHASSIS initial drive prepare incomplete, will retry in process");
    }

    s_chassis.initialized = 1u;
    log_info("CHASSIS init done");
    return ch.OK;
}

/**
 * @brief 设置底盘目标速度
 * @param vx 底盘 x 方向目标线速度，单位 m/s
 * @param vy 底盘 y 方向目标线速度，单位 m/s
 * @param wz 底盘 z 轴目标角速度，单位 rad/s
 * @return ChassisErrorCode 状态码
 */
ChassisErrorCode chassis_set_velocity(float vx, float vy, float wz) {
    if(s_chassis.initialized == 0u) {
        return ch.NOT_INITIALIZED;
    }

    s_chassis.brake_requested = 0u;
    s_chassis.brake_latched = 0u;
    chassis_external_to_internal_twist(vx, vy, wz,
        &s_chassis.kine.control.vx, &s_chassis.kine.control.vy, &s_chassis.kine.control.wz);

    return ch.OK;
}

/**
 * @brief 设置是否启用先转向到位再驱动模式
 * @param enabled true 启用，false 关闭
 * @return ChassisErrorCode 状态码
 */
ChassisErrorCode chassis_set_steer_then_drive_enabled(bool enabled) {
    if(s_chassis.initialized == 0u) {
        return ch.NOT_INITIALIZED;
    }

    s_chassis.steer_then_drive_enabled = enabled ? 1u : 0u;
    return ch.OK;
}

/**
 * @brief 执行一次底盘控制流程
 * @return ChassisErrorCode 状态码
 */
ChassisErrorCode chassis_process(void) {
    uint8_t i;
    bool steer_feedback_observed = true;
    bool drive_feedback_observed = true;
    uint8_t steer_missing_mask = 0u;
    uint8_t drive_missing_mask = 0u;

    if(s_chassis.initialized == 0u) {
        return ch.NOT_INITIALIZED;
    }

    for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
        const ChassisModuleMap* map = &s_module_map[i];

        if(steer_motor.update_feedback(map->steer_id, NULL) != MOTOR_STATUS_OK) {
            steer_feedback_observed = false;
            steer_missing_mask |= (uint8_t)(1u << map->module);
        }
        if(drive_motor.update_feedback(map->drive_id, NULL) != MOTOR_STATUS_OK) {
            drive_feedback_observed = false;
            drive_missing_mask |= (uint8_t)(1u << map->module);
        }
    }

    if(chassis_maintain_motor_startup(steer_feedback_observed, steer_missing_mask,
        drive_feedback_observed, drive_missing_mask) == false) {
        chassis_stop_all_drive_motors();
        s_chassis.kine.state.cur_vx = 0.0f;
        s_chassis.kine.state.cur_vy = 0.0f;
        s_chassis.kine.state.cur_wz = 0.0f;
        s_chassis_view = s_chassis;
        return ch.OK;
    }

    for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
        const ChassisModuleMap* map = &s_module_map[i];

        s_chassis.kine.state.cur_wheels[map->module].wheel_omega =
            chassis_drive_omega_to_wheel_omega((float)map->drive_sign * drive_motor.get_spd(map->drive_id));
        s_chassis.kine.state.cur_wheels[map->module].steer_angle =
            steer_motor.get_pos(map->steer_id);
    }

    if(s_chassis.brake_requested != 0u) {
        chassis_set_brake_targets();

        if(s_chassis.brake_latched == 0u) {
            for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
                const ChassisModuleMap* map = &s_module_map[i];
                float target_angle = s_chassis.kine.control.wheels[map->module].steer_angle;
                float steer_speed = chassis_calc_steer_track_speed(map->module, target_angle);

                (void)drive_motor.stop(map->drive_id);
                (void)steer_motor.set_pos_vel(map->steer_id, target_angle, steer_speed);
            }

            if(chassis_brake_targets_reached()) {
                s_chassis.brake_latched = 1u;
            }
        }

        if(s_chassis.brake_latched != 0u) {
            for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
                const ChassisModuleMap* map = &s_module_map[i];

                (void)steer_motor.brake(map->steer_id);
                (void)drive_motor.brake(map->drive_id);
            }
        }

        for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
            s_chassis.kine.state.cur_wheels[i].wheel_omega = 0.0f;
        }
        s_chassis.kine.state.cur_vx = 0.0f;
        s_chassis.kine.state.cur_vy = 0.0f;
        s_chassis.kine.state.cur_wz = 0.0f;
    }
    else {
        bool drive_ready;

        if(swheel.ik(&s_chassis.kine) != swheel.OK) {
            return ch.KINEMATICS_FAILED;
        }

        for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
            const ChassisModuleMap* map = &s_module_map[i];

            chassis_optimize_drive_module_target(map->module);
        }

        drive_ready = (s_chassis.steer_then_drive_enabled == 0u) || chassis_drive_targets_reached();

        for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
            const ChassisModuleMap* map = &s_module_map[i];
            float target_angle = s_chassis.kine.control.wheels[map->module].steer_angle;
            float target_speed = chassis_wheel_omega_to_drive_omega(s_chassis.kine.control.wheels[map->module].wheel_omega);
            float steer_speed = chassis_calc_steer_track_speed(map->module, target_angle);

            (void)steer_motor.set_pos_vel(map->steer_id, target_angle, steer_speed);
            (void)drive_motor.set_spd(map->drive_id, drive_ready ? ((float)map->drive_sign * target_speed) : 0.0f);
        }

        if(swheel.fk(&s_chassis.kine) != swheel.OK) {
            return ch.KINEMATICS_FAILED;
        }
    }

    s_chassis_view = s_chassis;
    chassis_internal_to_external_twist(s_chassis.kine.state.cur_vx, s_chassis.kine.state.cur_vy, s_chassis.kine.state.cur_wz,
        &s_chassis_view.kine.state.cur_vx, &s_chassis_view.kine.state.cur_vy, &s_chassis_view.kine.state.cur_wz);

    return ch.OK;
}

/**
 * @brief 停止底盘运动
 * @return ChassisErrorCode 状态码
 */
ChassisErrorCode chassis_stop(void) {
    uint8_t i;

    if(s_chassis.initialized == 0u) {
        return ch.NOT_INITIALIZED;
    }

    s_chassis.kine.control.vx = 0.0f;
    s_chassis.kine.control.vy = 0.0f;
    s_chassis.kine.control.wz = 0.0f;
    s_chassis.brake_requested = 0u;
    s_chassis.brake_latched = 0u;
    chassis_reset_steer_speed_profiles();

    for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
        (void)steer_motor.brake(s_module_map[i].steer_id);
        (void)drive_motor.stop(s_module_map[i].drive_id);
    }

    return ch.OK;
}

/**
 * @brief 请求底盘进入驻车刹车状态
 * @return ChassisErrorCode 状态码
 */
ChassisErrorCode chassis_brake(void) {
    if(s_chassis.initialized == 0u) {
        return ch.NOT_INITIALIZED;
    }

    s_chassis.kine.control.vx = 0.0f;
    s_chassis.kine.control.vy = 0.0f;
    s_chassis.kine.control.wz = 0.0f;
    if(s_chassis.brake_requested == 0u) {
        s_chassis.brake_latched = 0u;
        chassis_reset_steer_speed_profiles();
    }
    s_chassis.brake_requested = 1u;

    return ch.OK;
}

/**
 * @brief 获取底盘实例只读视图
 * @return const Chassis* 底盘实例指针
 */
const Chassis* chassis_get_chassis(void) {
    return &s_chassis_view;
}

/**
 * @brief 获取底盘当前状态只读视图
 * @return const SteerWheelState* 当前状态指针
 */
const SteerWheelState* chassis_get_state(void) {
    return &s_chassis_view.kine.state;
}

/**
 * @brief 获取底盘当前控制量只读视图
 * @return const SteerWheelControl* 当前控制量指针
 */
const SteerWheelControl* chassis_get_control(void) {
    return &s_chassis_view.kine.control;
}

/**
 * @brief 检查底盘是否已经就绪
 *
 * 当前只检查转向电机反馈状态；
 * 遥控链路是否在线由上层额外判断
 *
 * @return bool `true` 表示底盘就绪，`false` 表示仍在等待反馈
 */
bool chassis_is_ready(void) {
    return s_chassis.initialized != 0u
        && s_chassis.steer_motor_ready != 0u
        && s_chassis.drive_motor_ready != 0u;
}

/**
 * @brief 将底盘状态码转换为静态字符串
 * @param status 底盘状态码
 * @return const char* 状态码名称
 */
/**
 * @brief 展开底盘状态表为 switch 分支
 *
 * 该宏只在状态码转字符串函数中使用；
 * 展开完成后立即取消定义
 */
#define X(name, str) case CHASSIS_##name: return str;
const char* chassis_error_code_to_str(ChassisErrorCode status) {
    switch(status) {
        CHASSIS_STATUS_TABLE
        default: return "UNKNOWN";
    }
}
#undef X

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static ChassisErrorCode chassis_check_config(const ChassisConfig* config) {
    if(config == NULL) {
        return ch.INVALID_PARAM;
    }
    if(config->steer_motor_interface == NULL || config->drive_motor_interface == NULL
        || config->steer_ops == NULL || config->drive_ops == NULL
        || config->prepare_steer_motor == NULL || config->prepare_drive_motor == NULL
        || config->steer_ops->send == NULL || config->drive_ops->send == NULL) {
        return ch.DEPENDENCY_MISSING;
    }
    if(config->model.length <= 0.0f || config->model.width <= 0.0f
        || config->model.wheel_radius <= 0.0f || config->model.max_wheel_linear_speed < 0.0f
        || config->wheel_drive_ratio <= 0.0f) {
        return ch.INVALID_MODEL;
    }

    return ch.OK;
}

static ChassisErrorCode chassis_prepare_steer_motors(void) {
    bool all_ok = true;
    uint8_t i;

    for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
        uint8_t steer_id = s_module_map[i].steer_id;

        if(s_chassis.config.prepare_steer_motor(steer_id) != MOTOR_STATUS_OK) {
            log_warn("CHASSIS steer_id=%u prepare failed", steer_id);
            all_ok = false;
            continue;
        }

        log_info("CHASSIS steer_id=%u prepare ok", steer_id);
    }

    return all_ok ? ch.OK : ch.STEER_PREPARE_FAILED;
}

static ChassisErrorCode chassis_prepare_drive_motors(void) {
    bool all_ok = true;
    uint8_t i;

    for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
        uint8_t drive_id = s_module_map[i].drive_id;

        if(s_chassis.config.prepare_drive_motor(drive_id) != MOTOR_STATUS_OK) {
            log_warn("CHASSIS drive_id=%u prepare failed", drive_id);
            all_ok = false;
            continue;
        }

        log_info("CHASSIS drive_id=%u prepare ok", drive_id);
    }

    return all_ok ? ch.OK : ch.DRIVE_PREPARE_FAILED;
}

static void chassis_stop_all_drive_motors(void) {
    uint8_t i;

    for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
        (void)drive_motor.stop(s_module_map[i].drive_id);
    }
}

static bool chassis_maintain_motor_startup(
    bool steer_feedback_observed,
    uint8_t steer_missing_mask,
    bool drive_feedback_observed,
    uint8_t drive_missing_mask) {
    if(steer_feedback_observed) {
        if(s_chassis.steer_motor_ready == 0u) {
            log_info("CHASSIS steer motor ready");
        }
        s_steer_prepare_retry_countdown = 0u;
        s_chassis.steer_motor_ready = 1u;
        s_last_steer_missing_mask = 0u;
    }
    else {
        s_chassis.steer_motor_ready = 0u;

        if(steer_missing_mask != s_last_steer_missing_mask) {
            log_warn("CHASSIS steer feedback missing mask=0x%02X", steer_missing_mask);
            s_last_steer_missing_mask = steer_missing_mask;
        }

        if(s_steer_prepare_retry_countdown > 0u) {
            --s_steer_prepare_retry_countdown;
        }
        else {
            log_warn("CHASSIS retry steer prepare, missing mask=0x%02X", steer_missing_mask);
            (void)chassis_prepare_steer_motors();
            s_steer_prepare_retry_countdown = CHASSIS_STEER_PREPARE_RETRY_CYCLES;
        }
    }

    if(drive_feedback_observed) {
        if(s_chassis.drive_motor_ready == 0u) {
            log_info("CHASSIS drive motor ready");
        }
        s_drive_prepare_retry_countdown = 0u;
        s_chassis.drive_motor_ready = 1u;
        s_last_drive_missing_mask = 0u;
    }
    else {
        s_chassis.drive_motor_ready = 0u;

        if(drive_missing_mask != s_last_drive_missing_mask) {
            log_warn("CHASSIS drive feedback missing mask=0x%02X", drive_missing_mask);
            s_last_drive_missing_mask = drive_missing_mask;
        }

        if(s_drive_prepare_retry_countdown > 0u) {
            --s_drive_prepare_retry_countdown;
        }
        else {
            log_warn("CHASSIS retry drive prepare, missing mask=0x%02X", drive_missing_mask);
            (void)chassis_prepare_drive_motors();
            s_drive_prepare_retry_countdown = CHASSIS_DRIVE_PREPARE_RETRY_CYCLES;
        }
    }

    return s_chassis.steer_motor_ready != 0u && s_chassis.drive_motor_ready != 0u;
}

static float chassis_wheel_omega_to_drive_omega(float wheel_omega) {
    return wheel_omega * s_chassis.config.wheel_drive_ratio;
}

static float chassis_drive_omega_to_wheel_omega(float drive_omega) {
    return drive_omega / s_chassis.config.wheel_drive_ratio;
}

static void chassis_external_to_internal_twist(float vx_ext, float vy_ext, float wz_ext, float* vx_int, float* vy_int, float* wz_int) {
    if(vx_int != NULL) *vx_int = vx_ext;
    if(vy_int != NULL) *vy_int = -vy_ext;
    if(wz_int != NULL) *wz_int = -wz_ext;
}

static void chassis_internal_to_external_twist(float vx_int, float vy_int, float wz_int, float* vx_ext, float* vy_ext, float* wz_ext) {
    if(vx_ext != NULL) *vx_ext = vx_int;
    if(vy_ext != NULL) *vy_ext = -vy_int;
    if(wz_ext != NULL) *wz_ext = -wz_int;
}

static float chassis_clampf(float value, float min, float max) {
    if(value < min) {
        return min;
    }
    if(value > max) {
        return max;
    }

    return value;
}

static float chassis_wrap_pi(float angle) {
    while(angle > CHASSIS_PI) {
        angle -= CHASSIS_2PI;
    }
    while(angle <= -CHASSIS_PI) {
        angle += CHASSIS_2PI;
    }

    return angle;
}

static float chassis_select_nearest_cyclic_angle(float current_angle, float target_angle) {
    float best_target = target_angle;
    float best_error = fabsf(target_angle - current_angle);
    int8_t k;

    for(k = -2; k <= 2; ++k) {
        float candidate = target_angle + (float)k * CHASSIS_2PI;
        float error;

        if(candidate < -CHASSIS_STEER_POS_LIMIT_RAD || candidate > CHASSIS_STEER_POS_LIMIT_RAD) {
            continue;
        }

        error = fabsf(candidate - current_angle);
        if(error < best_error) {
            best_error = error;
            best_target = candidate;
        }
    }

    return chassis_clampf(best_target, -CHASSIS_STEER_POS_LIMIT_RAD, CHASSIS_STEER_POS_LIMIT_RAD);
}

static float chassis_smoothstep(float x) {
    if(x <= 0.0f) {
        return 0.0f;
    }
    if(x >= 1.0f) {
        return 1.0f;
    }

    return x * x * (3.0f - 2.0f * x);
}

static float chassis_select_nearest_equivalent_angle(float current_angle, float target_angle, float previous_target_angle) {
    float option_a = chassis_select_nearest_cyclic_angle(current_angle, target_angle);
    float option_b = chassis_select_nearest_cyclic_angle(current_angle, target_angle + CHASSIS_PI);
    float option_prev = chassis_select_nearest_cyclic_angle(current_angle, previous_target_angle);
    float error_a = option_a - current_angle;
    float error_b = option_b - current_angle;
    float error_prev = option_prev - current_angle;
    float prev_equiv_error_a = fabsf(chassis_wrap_pi(option_prev - option_a));
    float prev_equiv_error_b = fabsf(chassis_wrap_pi(option_prev - option_b));

    if((prev_equiv_error_a < CHASSIS_DRIVE_ANGLE_TOL_RAD || prev_equiv_error_b < CHASSIS_DRIVE_ANGLE_TOL_RAD)
        && fabsf(error_prev) <= (fminf(fabsf(error_a), fabsf(error_b)) + CHASSIS_EQUIV_ANGLE_HYST_RAD)) {
        return option_prev;
    }

    if(fabsf(error_b) < fabsf(error_a)) {
        return option_b;
    }

    return option_a;
}

static void chassis_optimize_drive_module_target(ChassisModule module) {
    float current_angle;
    float target_angle;
    float target_omega;
    float option_a;
    float option_b;
    float error_a;
    float error_b;

    if(module >= CHASSIS_MODULE_COUNT) {
        return;
    }

    current_angle = s_chassis.kine.state.cur_wheels[module].steer_angle;
    target_angle = s_chassis.kine.control.wheels[module].steer_angle;
    target_omega = s_chassis.kine.control.wheels[module].wheel_omega;

    option_a = chassis_select_nearest_cyclic_angle(current_angle, target_angle);
    option_b = chassis_select_nearest_cyclic_angle(current_angle, target_angle + CHASSIS_PI);
    error_a = option_a - current_angle;
    error_b = option_b - current_angle;

    if(fabsf(error_b) + CHASSIS_DRIVE_EQUIV_HYST_RAD < fabsf(error_a)) {
        s_chassis.kine.control.wheels[module].steer_angle = option_b;
        s_chassis.kine.control.wheels[module].wheel_omega = -target_omega;
    }
    else {
        s_chassis.kine.control.wheels[module].steer_angle = option_a;
    }
}

static void chassis_reset_steer_speed_profiles(void) {
    uint8_t i;

    for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
        s_steer_speed_ramp_time[i] = 0.0f;
        s_steer_speed_last_target[i] = 0.0f;
        s_steer_speed_initialized[i] = 0u;
    }
}

static float chassis_calc_steer_track_speed(ChassisModule module, float target_angle) {
    float current_angle;
    float angle_error;
    float target_delta;
    float ramp_ratio;
    float slowdown_ratio;
    float speed_ratio;

    if(module >= CHASSIS_MODULE_COUNT) {
        return CHASSIS_STEER_TRACK_MAX_SPEED_RAD_S;
    }

    current_angle = s_chassis.kine.state.cur_wheels[module].steer_angle;
    angle_error = fabsf(chassis_wrap_pi(target_angle - current_angle));

    if(angle_error <= CHASSIS_DRIVE_ANGLE_TOL_RAD) {
        s_steer_speed_ramp_time[module] = 0.0f;
        s_steer_speed_last_target[module] = target_angle;
        s_steer_speed_initialized[module] = 1u;
        return CHASSIS_STEER_TRACK_MIN_SPEED_RAD_S;
    }

    target_delta = chassis_wrap_pi(target_angle - s_steer_speed_last_target[module]);
    if(s_steer_speed_initialized[module] == 0u || fabsf(target_delta) > CHASSIS_STEER_TARGET_CHANGE_RAD) {
        s_steer_speed_ramp_time[module] = 0.0f;
        s_steer_speed_last_target[module] = target_angle;
        s_steer_speed_initialized[module] = 1u;
    }

    s_steer_speed_ramp_time[module] += CHASSIS_CONTROL_PERIOD_S;
    ramp_ratio = s_steer_speed_ramp_time[module] / CHASSIS_STEER_SPEED_RAMP_TIME_S;
    slowdown_ratio = chassis_smoothstep(angle_error / CHASSIS_STEER_SLOWDOWN_ANGLE_RAD);
    speed_ratio = chassis_smoothstep(ramp_ratio);
    if(slowdown_ratio < speed_ratio) {
        speed_ratio = slowdown_ratio;
    }

    return CHASSIS_STEER_TRACK_MIN_SPEED_RAD_S
        + (CHASSIS_STEER_TRACK_MAX_SPEED_RAD_S - CHASSIS_STEER_TRACK_MIN_SPEED_RAD_S) * speed_ratio;
}

static bool chassis_drive_targets_reached(void) {
    uint8_t i;

    for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
        float current_angle = s_chassis.kine.state.cur_wheels[i].steer_angle;
        float target_angle = s_chassis.kine.control.wheels[i].steer_angle;
        float angle_error = chassis_wrap_pi(target_angle - current_angle);

        if(fabsf(angle_error) > CHASSIS_DRIVE_ANGLE_TOL_RAD) {
            return false;
        }
    }

    return true;
}

static void chassis_set_brake_targets(void) {
    const float hx = s_chassis.config.model.length * 0.5f;
    const float hy = s_chassis.config.model.width * 0.5f;
    float target_fl = atan2f(hy, -hx);
    float target_fr = atan2f(-hy, -hx);
    float target_rr = atan2f(-hy, hx);
    float target_rl = atan2f(hy, hx);

    s_chassis.kine.control.wheels[CHASSIS_MODULE_FL].wheel_omega = 0.0f;
    s_chassis.kine.control.wheels[CHASSIS_MODULE_FR].wheel_omega = 0.0f;
    s_chassis.kine.control.wheels[CHASSIS_MODULE_RR].wheel_omega = 0.0f;
    s_chassis.kine.control.wheels[CHASSIS_MODULE_RL].wheel_omega = 0.0f;

    s_chassis.kine.control.wheels[CHASSIS_MODULE_FL].steer_angle =
        chassis_select_nearest_equivalent_angle(s_chassis.kine.state.cur_wheels[CHASSIS_MODULE_FL].steer_angle,
            target_fl,
            s_chassis.kine.control.wheels[CHASSIS_MODULE_FL].steer_angle);
    s_chassis.kine.control.wheels[CHASSIS_MODULE_FR].steer_angle =
        chassis_select_nearest_equivalent_angle(s_chassis.kine.state.cur_wheels[CHASSIS_MODULE_FR].steer_angle,
            target_fr,
            s_chassis.kine.control.wheels[CHASSIS_MODULE_FR].steer_angle);
    s_chassis.kine.control.wheels[CHASSIS_MODULE_RR].steer_angle =
        chassis_select_nearest_equivalent_angle(s_chassis.kine.state.cur_wheels[CHASSIS_MODULE_RR].steer_angle,
            target_rr,
            s_chassis.kine.control.wheels[CHASSIS_MODULE_RR].steer_angle);
    s_chassis.kine.control.wheels[CHASSIS_MODULE_RL].steer_angle =
        chassis_select_nearest_equivalent_angle(s_chassis.kine.state.cur_wheels[CHASSIS_MODULE_RL].steer_angle,
            target_rl,
            s_chassis.kine.control.wheels[CHASSIS_MODULE_RL].steer_angle);
}

static bool chassis_brake_targets_reached(void) {
    uint8_t i;

    for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
        float current_angle = s_chassis.kine.state.cur_wheels[i].steer_angle;
        float target_angle = s_chassis.kine.control.wheels[i].steer_angle;
        float angle_error = chassis_wrap_pi(target_angle - current_angle);

        if(fabsf(angle_error) > CHASSIS_BRAKE_ANGLE_TOL_RAD) {
            return false;
        }
    }

    return true;
}

#ifndef _app_fsm_h_
#define _app_fsm_h_

/**
 * @file app_fsm.h
 * @brief MCU 系统级状态机接口
 */

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 类 型 声 明 ========================= ! //

/**
 * @brief 系统级状态机状态
 * @details 语义上分为三类
 * - Operational: Idle, Manual, AutoPi, Finished
 * - Fault: 可通过 `CLEAR_FAULT` 恢复到 Idle
 * - EStop: 急停锁死态, 不能通过普通 `START` 或 `CLEAR_FAULT` 恢复
 */
typedef enum {
    APP_FSM_STATE_IDLE = 0,
    APP_FSM_STATE_MANUAL,
    APP_FSM_STATE_AUTO_PI,
    APP_FSM_STATE_FAULT,
    APP_FSM_STATE_ESTOP,
    APP_FSM_STATE_FINISHED
} AppFsmStateId;

/**
 * @brief 系统级状态机事件
 */
typedef enum {
    APP_FSM_EVENT_NONE = 0,
    APP_FSM_EVENT_START,
    APP_FSM_EVENT_STOP,
    APP_FSM_EVENT_SWITCH_TO_MANUAL,
    APP_FSM_EVENT_SWITCH_TO_AUTO_PI,
    APP_FSM_EVENT_FAULT,
    APP_FSM_EVENT_CLEAR_FAULT,
    APP_FSM_EVENT_ESTOP,
    APP_FSM_EVENT_FINISHED
} AppFsmEventId;

/**
 * @brief Manual 状态下的控制子模式
 */
typedef enum {
    APP_MANUAL_MODE_CHASSIS_PC_ARM = 0,
    APP_MANUAL_MODE_ARM_FS
} AppManualMode;

/**
 * @brief 系统级故障来源
 */
typedef enum {
    APP_FAULT_SOURCE_NONE = 0,
    APP_FAULT_SOURCE_CHASSIS,
    APP_FAULT_SOURCE_ODOM,
    APP_FAULT_SOURCE_ARM,
    APP_FAULT_SOURCE_REMOTE,
    APP_FAULT_SOURCE_PC_LINK,
    APP_FAULT_SOURCE_PI_LINK,
    APP_FAULT_SOURCE_PI_MISSION,
    APP_FAULT_SOURCE_COMMAND,
    APP_FAULT_SOURCE_SYSTEM
} AppFaultSource;

/**
 * @brief 系统级故障等级
 */
typedef enum {
    APP_FAULT_LEVEL_NONE = 0,
    APP_FAULT_LEVEL_WARN,
    APP_FAULT_LEVEL_DEGRADE,
    APP_FAULT_LEVEL_RECOVERABLE,
    APP_FAULT_LEVEL_FATAL
} AppFaultLevel;

/**
 * @brief 系统级故障快照
 */
typedef struct {
    AppFaultSource source;
    AppFaultLevel level;
    int32_t code;
    uint32_t stamp_ms;
} AppFault;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化 MCU 系统级状态机
 * @details 在 `app_runtime_init()` 中调用一次
 */
void app_fsm_init(void);

/**
 * @brief 处理挂起的系统级状态机事件
 * @details 建议在 500Hz 主循环中调用
 */
void app_fsm_process(void);

/**
 * @brief 投递普通优先级事件
 * @param event_id 事件 ID
 * @return bool `true` 表示投递成功
 */
bool app_fsm_post(AppFsmEventId event_id);

/**
 * @brief 更新 Manual 控制子模式
 * @param manual_mode 当前 Manual 子模式
 */
void app_fsm_set_manual_mode(AppManualMode manual_mode);

/**
 * @brief 锁存故障并以高优先级进入 Fault
 * @details 该接口会先保存故障信息, 再清空旧事件队列, 最后投递 `FAULT` 事件
 * @param fault 故障描述
 * @return bool `true` 表示故障事件投递成功
 */
bool app_fsm_raise_fault(const AppFault* fault);

/**
 * @brief 以高优先级请求进入 EStop
 * @details 该接口会先清空旧事件队列, 再投递 `ESTOP` 事件
 * @return bool `true` 表示急停事件投递成功
 */
bool app_fsm_request_estop(void);

/**
 * @brief 清除当前锁存故障并请求退出 Fault
 * @details 仅当当前状态为 Fault 且故障级别为 recoverable 时才应调用, EStop 不允许通过该接口恢复
 * @return bool `true` 表示清故障事件投递成功
 */
bool app_fsm_clear_fault(void);

/**
 * @brief 获取当前系统级状态
 * @return AppFsmStateId 当前状态
 */
AppFsmStateId app_fsm_get_state(void);

/**
 * @brief 获取当前 Manual 子模式
 * @return AppManualMode 当前 Manual 子模式
 */
AppManualMode app_fsm_get_manual_mode(void);

/**
 * @brief 获取当前锁存故障
 * @return const AppFault* 故障只读指针
 */
const AppFault* app_fsm_get_fault(void);

/**
 * @brief 判断当前是否存在锁存故障
 * @return bool `true` 表示存在锁存故障
 */
bool app_fsm_has_fault(void);

/**
 * @brief 将系统级状态转换为静态字符串
 * @param state 状态 ID
 * @return const char* 状态字符串
 */
const char* app_fsm_state_str(AppFsmStateId state);

/**
 * @brief 将 Manual 子模式转换为静态字符串
 * @param manual_mode Manual 子模式
 * @return const char* 子模式字符串
 */
const char* app_fsm_manual_mode_str(AppManualMode manual_mode);

#endif

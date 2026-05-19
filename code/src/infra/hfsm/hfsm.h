#ifndef _hfsm_h_
#define _hfsm_h_

#include "hfsm_config.h"
#include "hfsm_core.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief HFSM 单例，用户自定义名称
 */
#define hfsm hfsm_api_instance

/**
 * @brief HFSM 状态码表，使用 X-Macro 定义，方便维护和扩展
 */
#define HFSM_STATUS_TABLE \
    SX(OK) \
    SX(INVALID_ARG) \
    SX(NOT_INITIALIZE) \
    SX(NO_INITIAL_STATE) \
    SX(STARTED) \
    SX(NOT_STARTED) \
    SX(NO_SPACE)

/**
 * @brief HFSM 状态码，由 X-Macro 自动生成枚举类型
 * @param HFSM_STATUS_OK 操作成功
 * @param HFSM_STATUS_INVALID_ARG 无效参数
 * @param HFSM_STATUS_NOT_INITIALIZE 状态机未初始化
 * @param HFSM_STATUS_NO_INITIAL_STATE 没有设置初始状态
 * @param HFSM_STATUS_STARTED 状态机已启动
 * @param HFSM_STATUS_NOT_STARTED 状态机未启动
 * @param HFSM_STATUS_NO_SPACE 待处理事件队列没有空间

 */
#define SX(name) HFSM_STATUS_##name,
typedef enum {
    HFSM_STATUS_TABLE
} HfsmStatus;
#undef SX

/**
 * @brief HFSM 类型定义，包含状态机核心数据结构和状态列表
 * @param machine HFSM 核心数据结构，包含当前状态、事件队列等信息
 * @param initial_state 初始状态指针
 * @param states 状态列表数组，最大长度为 HFSM_MAX_STATES
 * @param state_count 当前状态数量
 * @param initialized 是否已初始化
 * @param started 是否已启动
 */
typedef struct {
    HfsmMachine machine;
    const HfsmState* initial_state;
    HfsmState states[HFSM_MAX_STATES];
    uint16_t state_count;
    bool initialized;
    bool started;
} Hfsm;

#define SX(name) const HfsmStatus name;
extern const struct HfsmApi {
    /**
     * @brief HFSM 状态码，由 X-Macro 自动生成枚举类型
     */
    struct {
        HFSM_STATUS_TABLE
    };
    /**
     * @brief 初始化状态机
     * @param fsm 状态机指针
     * @param context 用户上下文指针，状态机事件处理函数中可通过 hfsm.context() 获取
     * @return HfsmStatus 枚举类型，表示操作结果
     */
    HfsmStatus(*init)(Hfsm* fsm, void* context);
    /**
     * @brief 设置状态机上下文，必须在状态机启动前调用
     * @param fsm 状态机指针
     * @param context 用户上下文指针
     * @return HfsmStatus 枚举类型，表示操作结果
     */
    HfsmStatus(*set_context)(Hfsm* fsm, void* context);
    /**
     * @brief 设置状态机初始状态，必须在状态机启动前调用
     * @param fsm 状态机指针
     * @param initial_state 初始状态指针
     * @return HfsmStatus 枚举类型，表示操作结果
     */
    HfsmStatus(*set_initial)(Hfsm* fsm, const HfsmState* initial_state);
    /**
     * @brief 添加状态
     * @param fsm 状态机指针
     * @param name 状态名称，必须唯一
     * @return HfsmState* 指向新添加状态的指针，失败返回 NULL
     */
    HfsmState* (*add_state)(Hfsm* fsm, const char* name);
    /**
     * @brief 添加子状态
     * @param fsm 状态机指针
     * @param parent 父状态指针
     * @param name 状态名称，必须唯一
     * @return HfsmState* 指向新添加状态的指针，失败返回 NULL
     */
    HfsmState* (*add_substate)(Hfsm* fsm, HfsmState* parent, const char* name);
    /**
     * @brief 设置状态的父状态
     * @param s 状态指针
     * @param parent 父状态指针
     * @return HfsmState* 指向状态 s 的指针，失败返回 NULL
     */
    HfsmState* (*set_parent)(HfsmState* s, const HfsmState* parent);
    /**
     * @brief 设置状态的事件处理函数
     * @param s 状态指针
     * @param handle 事件处理函数指针，函数原型为 HfsmResult handle(const Hfsm* fsm, HfsmEventId event_id, const void* data)
     * @return HfsmState* 指向状态 s 的指针，失败返回 NULL
     */
    HfsmState* (*set_handle)(HfsmState* s, HfsmHandleFn handle);
    /**
     * @brief 设置状态的入口函数
     * @param s 状态指针
     * @param entry 入口函数指针，函数原型为 void entry(const Hfsm* fsm)
     * @return HfsmState* 指向状态 s 的指针，失败返回 NULL
     */
    HfsmState* (*set_entry)(HfsmState* s, HfsmHookFn entry);
    /**
     * @brief 设置状态的出口函数
     * @param s 状态指针
     * @param exit 出口函数指针，函数原型为 void exit(const Hfsm* fsm)
     * @return HfsmState* 指向状态 s 的指针，失败返回 NULL
     */
    HfsmState* (*set_exit)(HfsmState* s, HfsmHookFn exit);
    /**
     * @brief 设置状态的动作函数
     * @param s 状态指针
     * @param action 动作函数指针，函数原型为 void action(const Hfsm* fsm)
     * @return HfsmState* 指向状态 s 的指针，失败返回 NULL
     */
    HfsmState* (*set_action)(HfsmState* s, HfsmHookFn action);
    /**
     * @brief 设置状态的用户数据指针，用户可以在事件处理函数中通过 s->user_data 获取
     * @param s 状态指针
     * @param user_data 用户数据指针
     * @return HfsmState* 指向状态 s 的指针，失败返回 NULL
     */
    HfsmState* (*set_user_data)(HfsmState* s, void* user_data);
    /**
     * @brief 启动状态机，进入初始状态并执行入口函数
     * @param fsm 状态机指针
     * @return HfsmStatus 枚举类型，表示操作结果
     */
    HfsmStatus(*start)(Hfsm* fsm);
    /**
     * @brief 暂停状态机，停止处理事件，但保持当前状态不变
     * @param fsm 状态机指针
     * @return HfsmStatus 枚举类型，表示操作结果
     */
    HfsmStatus(*pause)(Hfsm* fsm);
    /**
     * @brief 继续状态机，恢复处理事件
     * @param fsm 状态机指针
     * @return HfsmStatus 枚举类型，表示操作结果
     */
    HfsmStatus(*go_on)(Hfsm* fsm);
    /**
     * @brief 重置状态机，回到初始状态并执行入口函数
     * @param fsm 状态机指针
     * @return HfsmStatus 枚举类型，表示操作结果
     */
    HfsmStatus(*reset)(Hfsm* fsm);
    /**
     * @brief 发送事件到状态机，事件将被加入待处理事件队列
     * @param fsm 状态机指针
     * @param event_id 事件 ID，必须大于 HFSM_EVENT_NONE
     * @param data 事件数据指针，用户自定义数据结构，状态机事件处理函数中可通过 data 参数获取
     * @return HfsmStatus 枚举类型，表示操作结果
     */
    HfsmStatus(*post)(Hfsm* fsm, HfsmEventId event_id, const void* data);
    /**
     * @brief 清空状态机待处理事件队列
     * @param fsm 状态机指针
     * @return HfsmStatus 枚举类型，表示操作结果
     */
    HfsmStatus(*clear)(Hfsm* fsm);
    /**
     * @brief 处理一个待处理事件，执行状态转换和动作函数
     * @param fsm 状态机指针
     * @return HfsmStatus 枚举类型，表示操作结果
     */
    HfsmStatus(*process)(Hfsm* fsm);
    /**
     * @brief 处理所有待处理事件，直到队列为空
     * @param fsm 状态机指针
     * @return HfsmStatus 枚举类型，表示操作结果
     */
    HfsmStatus(*process_all)(Hfsm* fsm);

    struct {
        /**
         * @brief 事件被忽略，状态机不做任何处理
         * @return HfsmResult 结果类型，表示事件被忽略
         */
        HfsmResult(*ignore)(void);
        /**
         * @brief 事件被处理，状态机不进行状态转换
         * @return HfsmResult 结果类型，表示事件被处理
         */
        HfsmResult(*handled)(void);
        /**
         * @brief 状态转换，状态机切换到指定的下一个状态
         * @param next_state 下一个状态指针
         * @return HfsmResult 结果类型，表示状态转换，并包含下一个状态指针
         */
        HfsmResult(*transition)(const HfsmState* next_state);
    } res;

    /**
     * @brief 获取当前状态指针
     * @param fsm 状态机指针
     * @return const HfsmState* 当前状态指针，未启动返回 NULL
     */
    const HfsmState* (*state)(const Hfsm* fsm);
    /**
     * @brief 获取正在处理事件的状态指针
     * @param fsm 状态机指针
     * @return const HfsmState* 正在处理事件的状态指针，未启动或未处理事件返回 NULL
     */
    const HfsmState* (*dispatching)(const Hfsm* fsm);
    /**
     * @brief 获取状态机上下文指针
     * @param fsm 状态机指针
     * @return void* 上下文指针，未初始化返回 NULL
     */
    void* (*context)(Hfsm* fsm);
    /**
     * @brief 获取状态机上下文指针，常量版本
     * @param fsm 状态机指针
     * @return const void* 上下文指针，未初始化返回 NULL
     */
    const void* (*const_context)(const Hfsm* fsm);
} hfsm_api_instance;
#undef SX

// ! ========================= 接 口 函 数 声 明 ========================= ! //

HfsmStatus hfsm_init(Hfsm* fsm, void* context);
HfsmStatus hfsm_set_context(Hfsm* fsm, void* context);
HfsmStatus hfsm_set_initial(Hfsm* fsm, const HfsmState* initial_state);
HfsmState* hfsm_add_state(Hfsm* fsm, const char* name);
HfsmState* hfsm_add_substate(Hfsm* fsm, HfsmState* parent, const char* name);

HfsmState* hfsm_set_parent(HfsmState* s, const HfsmState* parent);
HfsmState* hfsm_set_handle(HfsmState* s, HfsmHandleFn handle);
HfsmState* hfsm_set_entry(HfsmState* s, HfsmHookFn entry);
HfsmState* hfsm_set_exit(HfsmState* s, HfsmHookFn exit);
HfsmState* hfsm_set_action(HfsmState* s, HfsmHookFn action);
HfsmState* hfsm_set_user_data(HfsmState* s, void* user_data);

HfsmStatus hfsm_start(Hfsm* fsm);
HfsmStatus hfsm_pause(Hfsm* fsm);
HfsmStatus hfsm_go_on(Hfsm* fsm);
HfsmStatus hfsm_reset(Hfsm* fsm);

HfsmStatus hfsm_post(Hfsm* fsm, HfsmEventId event_id, const void* data);
HfsmStatus hfsm_clear(Hfsm* fsm);
HfsmStatus hfsm_process(Hfsm* fsm);
HfsmStatus hfsm_process_all(Hfsm* fsm);


HfsmResult hfsm_ignore(void);
HfsmResult hfsm_handled(void);
HfsmResult hfsm_transition(const HfsmState* next_state);

const HfsmState* hfsm_state(const Hfsm* fsm);
const HfsmState* hfsm_dispatching(const Hfsm* fsm);
void* hfsm_context(Hfsm* fsm);
const void* hfsm_const_context(const Hfsm* fsm);

#endif

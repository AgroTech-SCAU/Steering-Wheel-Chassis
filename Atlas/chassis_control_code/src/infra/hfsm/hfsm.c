#include "hfsm.h"
#include "hfsm_core.h"

#include <stddef.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief HFSM 单例，用户自定义名称
 */
#define hfsm hfsm_api_instance

/**
 * @brief HFSM 单例定义表，使用 X-Macro 定义，方便维护和扩展
 */
#define SX(name) .name = HFSM_STATUS_##name,
const struct HfsmApi hfsm_api_instance = {
    { HFSM_STATUS_TABLE },
    .init = hfsm_init,
    .set_context = hfsm_set_context,
    .set_initial = hfsm_set_initial,
    .add_state = hfsm_add_state,
    .add_substate = hfsm_add_substate,
    .set_parent = hfsm_set_parent,
    .set_handle = hfsm_set_handle,
    .set_entry = hfsm_set_entry,
    .set_exit = hfsm_set_exit,
    .set_action = hfsm_set_action,
    .set_user_data = hfsm_set_user_data,
    .start = hfsm_start,
    .pause = hfsm_pause,
    .go_on = hfsm_go_on,
    .reset = hfsm_reset,
    .post = hfsm_post,
    .clear = hfsm_clear,
    .process = hfsm_process,
    .process_all = hfsm_process_all,
    .res = {
        .ignore = hfsm_ignore,
        .handled = hfsm_handled,
        .transition = hfsm_transition },
    .machine = { .context = hfsm_machine_context, .const_context = hfsm_machine_const_context, .post = hfsm_machine_post, .clear = hfsm_machine_clear, .state = hfsm_machine_state },
    .state = hfsm_state,
    .dispatching = hfsm_dispatching,
    .context = hfsm_context,
    .const_context = hfsm_const_context
};
#undef SX

// ! ========================= 私 有 函 数 声 明 ========================= ! //



// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化状态机
 * @param fsm 状态机指针
 * @param context 用户上下文指针，状态机事件处理函数中可通过 hfsm.context() 获取
 * @return HfsmStatus 枚举类型，表示操作结果
 */
HfsmStatus hfsm_init(Hfsm* fsm, void* context) {
    if(fsm == NULL)
        return hfsm.INVALID_ARG;

    memset(fsm, 0, sizeof(Hfsm));
    fsm->machine.context = context;
    fsm->started = false;
    fsm->initialized = true;

    return hfsm.OK;
}

/**
 * @brief 设置状态机上下文
 * @param fsm 状态机指针
 * @param context 用户上下文指针
 * @return HfsmStatus 枚举类型，表示操作结果
 */
HfsmStatus hfsm_set_context(Hfsm* fsm, void* context) {
    if(fsm == NULL)
        return hfsm.INVALID_ARG;
    if(fsm->initialized == false)
        return hfsm.NOT_INITIALIZE;
    if(fsm->started)
        return hfsm.STARTED;

    fsm->machine.context = context;

    return hfsm.OK;
}

/**
 * @brief 设置状态机初始状态
 * @param fsm 状态机指针
 * @param initial_state 初始状态指针
 * @return HfsmStatus 枚举类型，表示操作结果
 */
HfsmStatus hfsm_set_initial(Hfsm* fsm, const HfsmState* initial_state) {
    if(fsm == NULL)
        return hfsm.INVALID_ARG;
    if(fsm->initialized == false)
        return hfsm.NOT_INITIALIZE;
    if(initial_state == NULL)
        return hfsm.INVALID_ARG;
    if(fsm->started)
        return hfsm.STARTED;

    fsm->initial_state = initial_state;

    return hfsm.OK;
}

/**
 * @brief 添加状态
 * @param fsm 状态机指针
 * @param name 状态名称，必须唯一
 * @return HfsmState* 指向新添加状态的指针，失败返回 NULL
 */
HfsmState* hfsm_add_state(Hfsm* fsm, const char* name) {
    if(fsm == NULL || name == NULL)
        return NULL;
    if(fsm->initialized == false)
        return NULL;
    if(fsm->state_count >= HFSM_MAX_STATES)
        return NULL;
    if(fsm->started)
        return NULL;

    HfsmState* s = &fsm->states[fsm->state_count++];
    memset(s, 0, sizeof(HfsmState));
    s->name = name;

    return s;
}

/**
 * @brief 添加子状态
 * @param fsm 状态机指针
 * @param parent 父状态指针
 * @param name 状态名称，必须唯一
 * @return HfsmState* 指向新添加状态的指针，失败返回 NULL
 */
HfsmState* hfsm_add_substate(Hfsm* fsm, HfsmState* parent, const char* name) {
    if(parent == NULL)
        return NULL;

    HfsmState* s = hfsm_add_state(fsm, name);
    if(s == NULL)
        return NULL;

    s->parent = parent;

    return s;
}

/**
 * @brief 设置状态的父状态
 * @param s 状态指针
 * @param parent 父状态指针
 * @return HfsmState* 指向状态 s 的指针，失败返回 NULL
 */
HfsmState* hfsm_set_parent(HfsmState* s, const HfsmState* parent) {
    if(s == NULL)
        return NULL;
    s->parent = parent;

    return s;
}

/**
 * @brief 设置状态的事件处理函数
 * @param s 状态指针
 * @param handle 事件处理函数指针
 * @return HfsmState* 指向状态 s 的指针，失败返回 NULL
 */
HfsmState* hfsm_set_handle(HfsmState* s, HfsmHandleFn handle) {
    if(s == NULL)
        return NULL;
    s->handle = handle;

    return s;
}

/**
 * @brief 设置状态的入口函数
 * @param s 状态指针
 * @param entry 入口函数指针，函数原型为 void entry(const Hfsm* fsm)
 * @return HfsmState* 指向状态 s 的指针，失败返回 NULL
 */
HfsmState* hfsm_set_entry(HfsmState* s, HfsmHookFn entry) {
    if(s == NULL)
        return NULL;
    s->entry = entry;

    return s;
}

/**
 * @brief 设置状态的出口函数
 * @param s 状态指针
 * @param exit 出口函数指针，函数原型为 void exit(const Hfsm* fsm)
 * @return HfsmState* 指向状态 s 的指针，失败返回 NULL
 */
HfsmState* hfsm_set_exit(HfsmState* s, HfsmHookFn exit) {
    if(s == NULL)
        return NULL;
    s->exit = exit;

    return s;
}

/**
 * @brief 设置状态的动作函数
 * @param s 状态指针
 * @param action 动作函数指针，函数原型为 void action(const Hfsm* fsm)
 * @return HfsmState* 指向状态 s 的指针，失败返回 NULL
 */
HfsmState* hfsm_set_action(HfsmState* s, HfsmHookFn action) {
    if(s == NULL)
        return NULL;
    s->action = action;

    return s;
}

/**
 * @brief 设置状态的用户数据指针
 * @param s 状态指针
 * @param user_data 用户数据指针
 * @return HfsmState* 指向状态 s 的指针，失败返回 NULL
 */
HfsmState* hfsm_set_user_data(HfsmState* s, void* user_data) {
    if(s == NULL)
        return NULL;
    s->user_data = user_data;

    return s;
}

/**
 * @brief 启动状态机，进入初始状态并执行入口函数
 * @param fsm 状态机指针
 * @return HfsmStatus 枚举类型，表示操作结果
 */
HfsmStatus hfsm_start(Hfsm* fsm) {
    if(fsm == NULL)
        return hfsm.INVALID_ARG;
    if(fsm->initialized == false)
        return hfsm.NOT_INITIALIZE;
    if(fsm->initial_state == NULL)
        return hfsm.NO_INITIAL_STATE;
    if(fsm->started)
        return hfsm.STARTED;

    hfsm_core.init(&fsm->machine, fsm->initial_state, fsm->machine.context);
    fsm->started = true;

    return hfsm.OK;
}

/**
 * @brief 暂停状态机，停止处理事件但保持当前状态
 * @param fsm 状态机指针
 * @return HfsmStatus 枚举类型，表示操作结果
 */
HfsmStatus hfsm_pause(Hfsm* fsm) {
    if(fsm == NULL)
        return hfsm.INVALID_ARG;
    if(fsm->initialized == false)
        return hfsm.NOT_INITIALIZE;
    if(fsm->started == false)
        return hfsm.NOT_STARTED;

    fsm->started = false;

    return hfsm.OK;
}

/**
 * @brief 继续状态机，恢复处理事件
 * @param fsm 状态机指针
 * @return HfsmStatus 枚举类型，表示操作结果
 */
HfsmStatus hfsm_go_on(Hfsm* fsm) {
    if(fsm == NULL)
        return hfsm.INVALID_ARG;
    if(fsm->initialized == false)
        return hfsm.NOT_INITIALIZE;
    if(fsm->started)
        return hfsm.STARTED;

    fsm->started = true;

    return hfsm.OK;
}

/**
 * @brief 重置状态机，重新进入初始状态并执行入口函数
 * @param fsm 状态机指针
 * @return HfsmStatus 枚举类型，表示操作结果
 */
HfsmStatus hfsm_reset(Hfsm* fsm) {
    if(fsm == NULL)
        return hfsm.INVALID_ARG;
    if(fsm->initialized == false)
        return hfsm.NOT_INITIALIZE;

    void* context = fsm->machine.context;
    hfsm_core.init(&fsm->machine, fsm->initial_state, context);
    fsm->started = true;

    return hfsm.OK;
}

/**
 * @brief 发布事件，事件将被加入待处理事件队列，状态机将在下一次调用 hfsm_process 或 hfsm_process_all 时处理事件
 * @param fsm 状态机指针
 * @param event_id 事件 ID，必须大于 HFSM_EVENT_NONE
 * @param data 事件数据指针，状态机事件处理函数中可通过 event_data 获取
 * @return HfsmStatus 枚举类型，表示操作结果
 */
HfsmStatus hfsm_post(Hfsm* fsm, HfsmEventId event_id, const void* data) {
    if(fsm == NULL)
        return hfsm.INVALID_ARG;
    if(fsm->initialized == false)
        return hfsm.NOT_INITIALIZE;
    if(fsm->started == false)
        return hfsm.NOT_STARTED;
    if(event_id == HFSM_EVENT_NONE)
        return hfsm.INVALID_ARG;

    return hfsm_core.post(&fsm->machine, event_id, data) ? hfsm.OK : hfsm.NO_SPACE;
}

/**
 * @brief 清空状态机待处理事件队列
 * @param fsm 状态机指针
 * @return HfsmStatus 枚举类型，表示操作结果
 */
HfsmStatus hfsm_clear(Hfsm* fsm) {
    if(fsm == NULL)
        return hfsm.INVALID_ARG;
    if(fsm->initialized == false)
        return hfsm.NOT_INITIALIZE;
    if(fsm->started == false)
        return hfsm.NOT_STARTED;

    hfsm_core.clear(&fsm->machine);

    return hfsm.OK;
}

/**
 * @brief 处理一个待处理事件，执行状态转换和动作函数
 * @param fsm 状态机指针
 * @return HfsmStatus 枚举类型，表示操作结果
 */
HfsmStatus hfsm_process(Hfsm* fsm) {
    if(fsm == NULL)
        return hfsm.INVALID_ARG;
    if(fsm->initialized == false)
        return hfsm.NOT_INITIALIZE;
    if(fsm->started == false)
        return hfsm.NOT_STARTED;

    hfsm_core.process(&fsm->machine);

    return hfsm.OK;
}

/**
 * @brief 处理所有待处理事件，直到队列为空
 * @param fsm 状态机指针
 * @return HfsmStatus 枚举类型，表示操作结果
 */
HfsmStatus hfsm_process_all(Hfsm* fsm) {
    if(fsm == NULL)
        return hfsm.INVALID_ARG;
    if(fsm->initialized == false)
        return hfsm.NOT_INITIALIZE;
    if(fsm->started == false)
        return hfsm.NOT_STARTED;

    hfsm_core.process_all(&fsm->machine);

    return hfsm.OK;
}

/**
 * @brief 忽略事件，状态机不处理事件，保持当前状态不变
 * @return HfsmResult 结果类型，表示忽略事件
 */
HfsmResult hfsm_ignore(void) {
    HfsmResult res = {
        .type = hfsm_core.res.IGNORE,
        .next_state = NULL
    };

    return res;
}

/**
 * @brief 事件被处理，状态机不进行状态转换
 * @return HfsmResult 结果类型，表示事件被处理
 */
HfsmResult hfsm_handled(void) {
    HfsmResult res = {
        .type = hfsm_core.res.HANDLED,
        .next_state = NULL
    };

    return res;
}

/**
 * @brief 状态转换，状态机切换到指定的下一个状态
 * @param next_state 下一个状态指针
 * @return HfsmResult 结果类型，表示状态转换，并包含下一个状态指针
 */
HfsmResult hfsm_transition(const HfsmState* next_state) {
    HfsmResult res = {
        .type = hfsm_core.res.TRANSITION,
        .next_state = next_state
    };

    return res;
}

/**
 * @brief 获取当前状态指针
 * @param fsm 状态机指针
 * @return const HfsmState* 当前状态指针，未启动返回 NULL
 */
const HfsmState* hfsm_state(const Hfsm* fsm) {
    if(fsm == NULL)
        return NULL;
    return hfsm_core.state(&fsm->machine);
}

/**
 * @brief 获取正在处理事件的状态指针
 * @param fsm 状态机指针
 * @return const HfsmState* 正在处理事件的状态指针，未启动或未处理事件返回 NULL
 */
const HfsmState* hfsm_dispatching(const Hfsm* fsm) {
    if(fsm == NULL)
        return NULL;
    return hfsm_core.dispatching(&fsm->machine);
}

/**
 * @brief 获取状态机上下文指针
 * @param fsm 状态机指针
 * @return void* 上下文指针，未初始化返回 NULL
 */
void* hfsm_context(Hfsm* fsm) {
    if(fsm == NULL)
        return NULL;
    return hfsm_core.context(&fsm->machine);
}

/**
 * @brief 获取状态机上下文指针，常量版本
 * @param fsm 状态机指针
 * @return const void* 上下文指针，未初始化返回 NULL
 */
const void* hfsm_const_context(const Hfsm* fsm) {
    if(fsm == NULL)
        return NULL;
    return hfsm_core.const_context(&fsm->machine);
}

/**
 * @brief 获取状态回调参数中的用户上下文
 * @param m 状态机内核实例指针，通常来自状态回调参数
 * @return void* 用户上下文指针，参数无效时返回 NULL
 */
void* hfsm_machine_context(HfsmMachine* m) {
    return hfsm_core.context(m);
}

/**
 * @brief 获取状态回调参数中的只读用户上下文
 * @param m 状态机内核实例指针
 * @return const void* 用户上下文指针，参数无效时返回 NULL
 */
const void* hfsm_machine_const_context(const HfsmMachine* m) {
    return hfsm_core.const_context(m);
}

/**
 * @brief 向状态回调参数对应的状态机投递事件
 * @param m 状态机内核实例指针
 * @param event_id 事件 ID
 * @param data 事件数据指针，可为 NULL
 * @return bool `true` 表示事件进入队列成功
 */
bool hfsm_machine_post(HfsmMachine* m, HfsmEventId event_id, const void* data) {
    return hfsm_core.post(m, event_id, data);
}

/**
 * @brief 清空状态回调参数对应的状态机事件队列
 * @param m 状态机内核实例指针
 */
void hfsm_machine_clear(HfsmMachine* m) {
    hfsm_core.clear(m);
}

/**
 * @brief 获取状态回调参数对应的当前状态
 * @param m 状态机内核实例指针
 * @return const HfsmState* 当前状态指针，参数无效时返回 NULL
 */
const HfsmState* hfsm_machine_state(const HfsmMachine* m) {
    return hfsm_core.state(m);
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

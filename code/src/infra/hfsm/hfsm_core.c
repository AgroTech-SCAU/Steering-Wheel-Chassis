#include "hfsm_core.h"
#include "hfsm_config.h"

#include <assert.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 全局 HFSM 实例，提供接口函数和资源函数的访问
 * @param init 初始化状态机实例
 * @param post 向状态机实例发送事件
 * @param clear 清除状态机实例的待处理事件
 * @param process 处理状态机实例的待处理事件
 * @param state 获取状态机实例的当前状态
 * @param dispatching 获取状态机实例正在处理事件的状态指针
 * @param context 获取状态机实例的上下文指针
 * @param const_context 获取状态机实例的上下文指针（只读）
 * @param is_descendant_of 判断一个状态是否是另一个状态的子状态
 * @param transition 从当前状态转换到目标状态
 * @param has_pending 判断状态机实例是否有待处理事件
 * @param res 资源函数结构体，包含 ignore、handled 和 transition 三个函数
 *      - ignore 返回一个表示事件被忽略的结果
 *      - handled 返回一个表示事件已处理但不进行状态转换的结果
 *      - transition 返回一个表示事件已处理并进行状态转换的结果，next_state 字段指向目标状态
 */
#define RX(name) .name = HFSM_RES_##name,
const struct HfsmInstance hfsm_core_instance = {
    .init = hfsm_core_init,
    .post = hfsm_core_post,
    .clear = hfsm_core_clear,
    .process = hfsm_core_process,
    .process_all = hfsm_core_process_all,
    .state = hfsm_core_get_current_state,
    .dispatching = hfsm_core_get_dispatching_state,
    .context = hfsm_core_get_context,
    .const_context = hfsm_core_get_const_context,
    .is_descendant_of = hfsm_core_is_descendant_of,
    .transition = hfsm_core_transition,
    .has_pending = hfsm_core_has_pending_event,
    .res = {
        .ignore = hfsm_core_res_ignore,
        .handled = hfsm_core_res_handled,
        .transition = hfsm_core_res_transition,
        HFSM_RES_TYPE
    }
};
#undef RX
#define hfsm_core hfsm_core_instance

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static bool pop_event(HfsmMachine* m, HfsmEvent* out);
static HfsmResult dispatch(HfsmMachine* m, const HfsmEvent* e);
static const HfsmState* find_lca(const HfsmState* s1, const HfsmState* s2);
static void exit_up_to(const HfsmState* from, const HfsmState* to, HfsmMachine* m);
static void enter_down_to(const HfsmState* from, const HfsmState* to, HfsmMachine* m);
static void execute_action(HfsmMachine* m);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化状态机实例
 * @param m 状态机实例指针
 * @param initial_state 初始状态指针
 * @param context 用户自定义上下文指针
 */
void hfsm_core_init(HfsmMachine* m, const HfsmState* initial_state, void* context) {
    if(m == NULL) return;

    m->current_state = NULL;
    m->dispatching_state = NULL;
    m->queue_head = 0;
    m->queue_tail = 0;
    m->queue_count = 0;
    m->context = context;

    if(initial_state == NULL) return;

    m->current_state = initial_state;
    enter_down_to(NULL, initial_state, m);
}

/**
 * @brief 向状态机实例发送事件
 * @param m 状态机实例指针
 * @param event_id 事件ID
 * @param data 事件数据指针
 * @return true 表示事件发送成功，false 表示失败
 */
bool hfsm_core_post(HfsmMachine* m, HfsmEventId event_id, const void* data) {
    if(m == NULL || event_id == HFSM_EVENT_NONE) return false;
    if(m->queue_count >= HFSM_PENDING_QUEUE_MAX) return false;

    HfsmEvent* dst = &m->queue[m->queue_tail];
    dst->id = event_id;

    if(data != NULL) memcpy(&dst->data, data, sizeof(HFSM_EVENT_DATA_TYPE));
    else memset(&dst->data, 0, sizeof(HFSM_EVENT_DATA_TYPE));

    m->queue_tail = (uint8_t)((m->queue_tail + 1) % HFSM_PENDING_QUEUE_MAX);
    ++m->queue_count;

    return true;
}

/**
 * @brief 获取状态机实例的当前状态
 * @param m 状态机实例指针
 * @return 当前状态指针，若 m 为 NULL 则返回 NULL
 */
void hfsm_core_clear(HfsmMachine* m) {
    if(m == NULL) return;

    m->queue_head = 0;
    m->queue_tail = 0;
    m->queue_count = 0;
}

/**
 * @brief 处理状态机实例的待处理事件
 * @param m 状态机实例指针
 */
void hfsm_core_process(HfsmMachine* m) {
    if(m == NULL || m->current_state == NULL) return;

    const HfsmState* current_state = m->current_state;
    if(m->queue_count > 0) {
        HfsmEvent event;
        if(!pop_event(m, &event)) return;
        HfsmResult result = dispatch(m, &event);

        if(result.type == hfsm_core.res.TRANSITION && result.next_state != NULL && result.next_state != current_state) {
            hfsm_core_transition(m, result.next_state);
        }
    }

    if(m->current_state == NULL) return;

#if HFSM_RUN_PARENT_ACTIONS
    execute_action(m);
#else
    if(m->current_state->action) {
        m->dispatching_state = m->current_state;
        m->current_state->action(m);
        m->dispatching_state = NULL;
    }
#endif
}

/**
 * @brief 一次性处理完所有待处理事件，直到队列为空
 * @param m 状态机实例指针
 */
void hfsm_core_process_all(HfsmMachine* m) {
    if(m == NULL || m->current_state == NULL) return;

    uint8_t processed = 0;
    HfsmEvent event;
    bool state_changed = false;

    while(pop_event(m, &event) && processed++ < HFSM_MAX_CHAIN_LENGTH) {
        const HfsmState* before_dispatch_state = m->current_state;
        HfsmResult result = dispatch(m, &event);

        if(result.type == hfsm_core.res.TRANSITION && result.next_state != NULL) {
            hfsm_core_transition(m, result.next_state);
        }

        if(m->current_state != before_dispatch_state && m->current_state != NULL) {
            state_changed = true;
#if HFSM_RUN_PARENT_ACTIONS
            execute_action(m);
#else
            if(m->current_state->action) {
                m->dispatching_state = m->current_state;
                m->current_state->action(m);
                m->dispatching_state = NULL;
            }
#endif
        }
    }

    if(m->current_state == NULL || state_changed) return;

#if HFSM_RUN_PARENT_ACTIONS
    execute_action(m);
#else
    if(m->current_state->action) {
        m->dispatching_state = m->current_state;
        m->current_state->action(m);
        m->dispatching_state = NULL;
    }
#endif
}

/**
 * @brief 获取状态机实例的当前状态
 * @param m 状态机实例指针
 * @return 当前状态指针，若 m 为 NULL 则返回 NULL
 */
const HfsmState* hfsm_core_get_current_state(const HfsmMachine* m) {
    if(m == NULL) return NULL;

    return m->current_state;
}

/**
 * @brief 获取状态机实例正在处理事件的状态指针
 * @param m 状态机实例指针
 * @return 正在处理事件的状态指针，若 m 为 NULL 则返回 NULL
 */
const HfsmState* hfsm_core_get_dispatching_state(const HfsmMachine* m) {
    if(m == NULL) return NULL;

    return m->dispatching_state;
}

/**
 * @brief 获取状态机实例的上下文指针
 * @param m 状态机实例指针
 * @return 上下文指针，若 m 为 NULL 则返回 NULL
 */
void* hfsm_core_get_context(HfsmMachine* m) {
    if(m == NULL) return NULL;

    return m->context;
}

/**
 * @brief 获取状态机实例的上下文指针（只读）
 * @param m 状态机实例指针
 * @return 上下文指针，若 m 为 NULL 则返回 NULL
 */
const void* hfsm_core_get_const_context(const HfsmMachine* m) {
    if(m == NULL) return NULL;

    return m->context;
}

/**
 * @brief 判断一个状态是否是另一个状态的子状态
 * @param state 待判断状态指针
 * @param ancestor 祖先状态指针
 * @return 若 state 是 ancestor 的子状态返回 true，否则返回 false
 */
bool hfsm_core_is_descendant_of(const HfsmState* state, const HfsmState* ancestor) {
    if(state == NULL || ancestor == NULL) return false;

    const HfsmState* current = state;
    while(current != NULL) {
        if(current == ancestor) return true;
        current = current->parent;
    }

    return false;
}

/**
 * @brief 从当前状态转换到目标状态
 * @param m 状态机实例指针
 * @param target_state 目标状态指针
 * @return 转换成功返回 true，失败返回 false
 */
bool hfsm_core_transition(HfsmMachine* m, const HfsmState* target_state) {
    if(m == NULL || m->current_state == NULL || target_state == NULL) return false;
    if(m->current_state == target_state) return true;

    const HfsmState* lca = find_lca(m->current_state, target_state);
    exit_up_to(m->current_state, lca, m);
    enter_down_to(lca, target_state, m);

    return true;
}

/**
 * @brief 判断状态机实例是否有待处理事件
 * @param m 状态机实例指针
 * @return true 表示有待处理事件，false 表示没有
 */
bool hfsm_core_has_pending_event(const HfsmMachine* m) {
    if(m == NULL) return false;

    return m->queue_count > 0;
}

/**
 * @brief 返回一个表示事件被忽略的结果
 * @return HfsmResult 结果类型为 IGNORE，next_state 字段为 NULL
 */
HfsmResult hfsm_core_res_ignore(void) {
    return (HfsmResult) { .type = hfsm_core.res.IGNORE, .next_state = NULL };
}

/**
 * @brief 返回一个表示事件已处理但不进行状态转换的结果
 * @return HfsmResult 结果类型为 HANDLED，next_state 字段为 NULL
 */
HfsmResult hfsm_core_res_handled(void) {
    return (HfsmResult) { .type = hfsm_core.res.HANDLED, .next_state = NULL };
}

/**
 * @brief 返回一个表示事件已处理并进行状态转换的结果
 * @param next_state 目标状态指针
 * @return HfsmResult 结果类型为 TRANSITION，next_state 字段指向目标状态
 */
HfsmResult hfsm_core_res_transition(const HfsmState* next_state) {
    return (HfsmResult) { .type = hfsm_core.res.TRANSITION, .next_state = next_state };
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 从状态机实例的事件队列中弹出一个事件
 * @param m 状态机实例指针
 * @param out 输出参数，存储弹出的事件
 * @return true 表示成功弹出事件，false 表示队列为空或 m 为 NULL
 */
static bool pop_event(HfsmMachine* m, HfsmEvent* out) {
    if(m == NULL || out == NULL || m->queue_count == 0) return false;

    *out = m->queue[m->queue_head];
    m->queue_head = (uint8_t)((m->queue_head + 1) % HFSM_PENDING_QUEUE_MAX);
    --m->queue_count;

    return true;
}

/**
 * @brief 分发事件到状态机实例的当前状态及其祖先状态
 * @param m 状态机实例指针
 * @param e 事件指针
 * @return HfsmResult 处理结果，可能是 IGNORE、HANDLED 或 TRANSITION
 */
static HfsmResult dispatch(HfsmMachine* m, const HfsmEvent* e) {
    if(m == NULL || e == NULL || m->current_state == NULL) return hfsm_core.res.ignore();

    const HfsmState* state = m->current_state;
    while(state != NULL) {
        if(state->handle != NULL) {
            m->dispatching_state = state;
            HfsmResult result = state->handle(m, e);
            m->dispatching_state = NULL;
            if(result.type == hfsm_core.res.HANDLED) return result;
            if(result.type == hfsm_core.res.TRANSITION && result.next_state != NULL) return result;
        }
        state = state->parent;
    }

    return hfsm_core.res.ignore();
}

/**
 * @brief 查找两个状态的最近公共祖先
 * @param s1 状态指针1
 * @param s2 状态指针2
 * @return 最近公共祖先状态指针，若没有公共祖先则返回 NULL
 */
static const HfsmState* find_lca(const HfsmState* s1, const HfsmState* s2) {
    if(!s1 || !s2) return NULL;

    int depth1 = 0, depth2 = 0;

    const HfsmState* s = s1;
    while(s) { depth1++; s = s->parent; }
    s = s2;
    while(s) { depth2++; s = s->parent; }

    const HfsmState* deeper = depth1 > depth2 ? s1 : s2;
    const HfsmState* shallower = depth1 > depth2 ? s2 : s1;
    int diff = abs(depth1 - depth2);

    while(diff--) { deeper = deeper->parent; }
    while(deeper != shallower) {
        deeper = deeper->parent;
        shallower = shallower->parent;
    }

    return deeper;
}

/**
 * @brief 从状态 from 退出到状态 to（不包括 to），调用每个状态的 exit 钩子函数
 * @param from 起始状态指针
 * @param to 目标状态指针
 * @param m 状态机实例指针
 */
static void exit_up_to(const HfsmState* from, const HfsmState* to, HfsmMachine* m) {
    const HfsmState* s = from;
    while(s && s != to) {
        if(s->exit) {
            m->dispatching_state = s;
            s->exit(m);
            m->dispatching_state = NULL;
        }
        s = s->parent;
    }
}

/**
 * @brief 从状态 from 进入到状态 to（不包括 from），调用每个状态的 entry 钩子函数
 * @param from 起始状态指针
 * @param to 目标状态指针
 * @param m 状态机实例指针
 */
static void enter_down_to(const HfsmState* from, const HfsmState* to, HfsmMachine* m) {
    const HfsmState* path[HFSM_DEPTH];
    int depth = 0;
    const HfsmState* s = to;

    while(s && s != from) {
#if HFSM_ENABLE_ASSERT
        assert(depth < HFSM_DEPTH);
#else
        if(depth >= HFSM_DEPTH) return;
#endif
        path[depth++] = s;
        s = s->parent;
    }

    while(depth--) {
        s = path[depth];
        m->current_state = s;
        if(s->entry) {
            m->dispatching_state = s;
            s->entry(m);
            m->dispatching_state = NULL;
        }
    }
}

/**
 * @brief 执行当前状态及其祖先状态的 action 函数
 * @param m 状态机实例指针
 */
static void execute_action(HfsmMachine* m) {
    const HfsmState* s = m->current_state;
    while(s) {
        if(s->action) {
            m->dispatching_state = s;
            s->action(m);
            m->dispatching_state = NULL;
        }
        s = s->parent;
    }
}

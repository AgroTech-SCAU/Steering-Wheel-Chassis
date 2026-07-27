# HFSM：轻量级层级有限状态机库

`infra/hfsm` 是一个可移植的 C 语言层级有限状态机（Hierarchical Finite State Machine, HFSM）库，适合用于 MCU 裸机、RTOS、机器人任务编排、业务流程控制等场景

---

## 1. 适合解决什么问题

普通 FSM 通常把所有事件都写在同一层状态里，一旦状态数量变多，就容易出现大量重复判断；HFSM 的核心价值是：

```text
子状态处理不了的事件，可以继续交给父状态处理
```

例如一个通用任务流程可以抽象成：

```text
Fault
Manual
Auto
├── Idle
├── Navigate
├── Operate
└── ReturnHome
```

其中：

- `Idle / Navigate / Operate / ReturnHome` 只处理各自阶段的局部事件；
- `Auto` 统一处理自主模式下共有的 `STOP / FAULT / SWITCH_TO_MANUAL`；
- `Manual` 处理接管后的恢复逻辑；
- `Fault` 作为故障收口状态，统一执行安全动作

这样可以避免每个子状态都重复写“停止、故障、接管”等公共事件

---

## 2. 文件组成与移植方式

把以下文件加入目标工程即可：

```text
hfsm/
├── hfsm.h
├── hfsm.c
├── hfsm_core.h
├── hfsm_core.c
├── hfsm_config.h
└── README.md
```

其中：

| 文件 | 作用 |
|---|---|
| `hfsm.h / hfsm.c` | 推荐业务层直接使用的封装层，提供状态池、生命周期检查、状态码和易用 API |
| `hfsm_core.h / hfsm_core.c` | 最小状态机内核，负责事件队列、层级分发、状态切换、`entry/exit/action` 调度 |
| `hfsm_config.h` | 配置状态数量、层级深度、事件队列长度、事件数据类型等 |

---

## 3. 分层设计

```text
业务代码
   ↓
hfsm.h / hfsm.c
   - 状态池 Hfsm.states[]
   - init/start/pause/go_on/reset 生命周期
   - post/clear/process/process_all 事件驱动接口
   - HfsmStatus 状态码
   - hfsm.machine.* 回调辅助接口
   ↓
hfsm_core.h / hfsm_core.c
   - HfsmMachine 内核对象
   - 环形事件队列
   - 子状态到父状态的事件上送
   - LCA 最近公共祖先状态切换
   - entry / exit / action 调度
   ↓
hfsm_config.h
   - 容量与行为裁剪
```

一般使用 `hfsm.h` 即可；只有在你希望完全静态定义 `HfsmState`、绕过封装层状态池，或者需要访问更底层的判断接口时，才直接使用 `hfsm_core.h`

---

## 4. 核心概念

### 4.1 状态：`HfsmState`

```c
struct HfsmState {
    const char* name;
    const HfsmState* parent;

    HfsmHandleFn handle;
    HfsmHookFn entry;
    HfsmHookFn exit;
    HfsmHookFn action;

    void* user_data;
};
```

| 字段 | 含义 |
|---|---|
| `name` | 状态名，主要用于调试；库不强制检查重名，建议业务层保持唯一 |
| `parent` | 父状态指针，用于形成层级结构；根状态的 `parent` 为 `NULL` |
| `handle` | 事件处理函数；返回 `ignore/handled/transition` |
| `entry` | 进入该状态时执行 |
| `exit` | 退出该状态时执行 |
| `action` | `process()` 驱动时执行的周期动作 |
| `user_data` | 状态私有数据，可选 |

回调函数原型为：

```c
typedef HfsmResult (*HfsmHandleFn)(HfsmMachine* m, const HfsmEvent* e);
typedef void (*HfsmHookFn)(HfsmMachine* m);
```

注意：状态回调拿到的是 `HfsmMachine*`，不是 `Hfsm*`；在回调里需要访问上下文或投递事件时，推荐使用 `hfsm.machine.*` 辅助接口

---

### 4.2 事件：`HfsmEvent`

```c
typedef uint16_t HfsmEventId;

typedef struct {
    HfsmEventId id;
    HFSM_EVENT_DATA_TYPE data;
} HfsmEvent;
```

约定：

- `HFSM_EVENT_NONE == 0`，不能作为有效业务事件；
- 业务事件 ID 建议从 `1` 开始；
- `hfsm.post(&fsm, event_id, data)` 会把 `*data` 按 `sizeof(HFSM_EVENT_DATA_TYPE)` 拷贝进事件队列；
- 如果事件不带数据，传 `NULL` 即可

事件枚举示例：

```c
typedef enum {
    APP_EVENT_START = 1,
    APP_EVENT_STOP,
    APP_EVENT_SWITCH_TO_MANUAL,
    APP_EVENT_SWITCH_TO_AUTO,
    APP_EVENT_STEP_FINISHED,
    APP_EVENT_FAULT,
    APP_EVENT_FAULT_CLEAR,
} AppEventId;
```

---

### 4.3 事件处理结果：`HfsmResult`

状态处理事件后必须返回一种结果：

| 返回值 | 含义 |
|---|---|
| `hfsm.res.ignore()` | 当前状态不处理该事件，继续交给父状态 |
| `hfsm.res.handled()` | 事件已处理，不切换状态，也不继续上送 |
| `hfsm.res.transition(target)` | 事件已处理，并切换到目标状态 |

示例：

```c
static HfsmResult idle_handle(HfsmMachine* m, const HfsmEvent* e) {
    AppContext* ctx = (AppContext*)hfsm.machine.context(m);

    if(ctx == NULL)
        return hfsm.res.ignore();

    if(e->id == APP_EVENT_START) {
        ctx->step_index = 0u;
        return hfsm.res.transition(s_run);
    }

    return hfsm.res.ignore();
}
```

---

### 4.4 用户上下文：`context`

`context` 是整台状态机共享的业务上下文，适合保存当前任务状态、子模块上下文、故障信息、计时数据等

```c
typedef struct {
    uint8_t step_index;
    bool fault_latched;
} AppContext;
```

初始化时传入：

```c
static Hfsm s_fsm;
static AppContext s_ctx;

hfsm.init(&s_fsm, &s_ctx);
```

在状态回调中取出：

```c
static AppContext* app_ctx_from_machine(HfsmMachine* m) {
    return (AppContext*)hfsm.machine.context(m);
}
```

`context` 的生命周期必须长于状态机运行周期，不能传入临时局部变量的地址后在函数返回后继续使用

---

### 4.5 状态私有数据：`user_data`

如果某个状态需要绑定少量私有数据，可以使用：

```c
static uint32_t s_run_counter = 0u;
hfsm.set_user_data(s_run, &s_run_counter);
```

在回调里可以通过当前正在分发的状态获取：

```c
static void run_action(HfsmMachine* m) {
    const HfsmState* state = hfsm_core.dispatching(m);

    if(state == NULL || state->user_data == NULL)
        return;

    uint32_t* counter = (uint32_t*)state->user_data;
    ++(*counter);
}
```

一般建议优先使用 `context` 管理业务数据；`user_data` 更适合绑定状态局部计数器、配置表、只读参数等

---

## 5. 最小使用示例

```c
#include "hfsm.h"

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 业务事件 ========================= ! //

typedef enum {
    APP_EVENT_START = 1,
    APP_EVENT_STOP,
    APP_EVENT_TICK,
} AppEventId;

// ! ========================= 业务上下文 ========================= ! //

typedef struct {
    uint32_t tick_count;
    bool running;
} AppContext;

// ! ========================= 状态指针 ========================= ! //

static Hfsm s_fsm;
static AppContext s_ctx;
static HfsmState* s_idle = NULL;
static HfsmState* s_run = NULL;

// ! ========================= 回调函数 ========================= ! //

static AppContext* app_ctx_from_machine(HfsmMachine* m) {
    return (AppContext*)hfsm.machine.context(m);
}

static HfsmResult idle_handle(HfsmMachine* m, const HfsmEvent* e) {
    AppContext* ctx = app_ctx_from_machine(m);

    if(ctx == NULL)
        return hfsm.res.ignore();

    if(e->id == APP_EVENT_START) {
        ctx->tick_count = 0u;
        return hfsm.res.transition(s_run);
    }

    return hfsm.res.ignore();
}

static HfsmResult run_handle(HfsmMachine* m, const HfsmEvent* e) {
    AppContext* ctx = app_ctx_from_machine(m);

    if(ctx == NULL)
        return hfsm.res.ignore();

    if(e->id == APP_EVENT_STOP)
        return hfsm.res.transition(s_idle);

    if(e->id == APP_EVENT_TICK) {
        ++ctx->tick_count;
        return hfsm.res.handled();
    }

    return hfsm.res.ignore();
}

static void idle_entry(HfsmMachine* m) {
    AppContext* ctx = app_ctx_from_machine(m);

    if(ctx != NULL)
        ctx->running = false;
}

static void run_entry(HfsmMachine* m) {
    AppContext* ctx = app_ctx_from_machine(m);

    if(ctx != NULL)
        ctx->running = true;
}

static void run_action(HfsmMachine* m) {
    AppContext* ctx = app_ctx_from_machine(m);

    if(ctx == NULL)
        return;

    // 周期动作：可以在这里轮询子任务，并在条件满足时继续投递事件
    if(ctx->tick_count >= 100u)
        (void)hfsm.machine.post(m, APP_EVENT_STOP, NULL);
}

// ! ========================= 初始化与运行 ========================= ! //

void app_fsm_init(void) {
    hfsm.init(&s_fsm, &s_ctx);

    s_idle = hfsm.add_state(&s_fsm, "Idle");
    s_run = hfsm.add_state(&s_fsm, "Run");

    hfsm.set_handle(s_idle, idle_handle);
    hfsm.set_entry(s_idle, idle_entry);

    hfsm.set_handle(s_run, run_handle);
    hfsm.set_entry(s_run, run_entry);
    hfsm.set_action(s_run, run_action);

    hfsm.set_initial(&s_fsm, s_idle);
    hfsm.start(&s_fsm);
}

void app_fsm_process(void) {
    (void)hfsm.process(&s_fsm);
}

void app_fsm_start(void) {
    (void)hfsm.post(&s_fsm, APP_EVENT_START, NULL);
}
```

典型主循环：

```c
int main(void) {
    app_fsm_init();

    while(1) {
        // 其他外设、通信、控制任务 ...
        app_fsm_process();
    }
}
```

---

## 6. 推荐的应用层封装方式

在较大的工程里，不建议让外部模块直接访问 `Hfsm` 和 `HfsmState*`；推荐在应用层再包一层模块，把 HFSM 作为内部实现细节

### 6.1 内部结构

```c
typedef struct {
    Hfsm fsm;
    AppContext ctx;
} AppTask;

static AppTask s_task;
```

### 6.2 对外接口

```c
void app_task_init(void);
void app_task_process(void);
bool app_task_post(AppEventId event_id);
bool app_task_force_post(AppEventId event_id);
```

外部模块只投递业务事件，不直接操作状态机：

```c
bool app_task_post(AppEventId event_id) {
    return hfsm.post(&s_task.fsm, (HfsmEventId)event_id, NULL) == hfsm.OK;
}

bool app_task_force_post(AppEventId event_id) {
    (void)hfsm.clear(&s_task.fsm);
    return app_task_post(event_id);
}

void app_task_process(void) {
    (void)hfsm.process(&s_task.fsm);
}
```

这种封装方式的好处是：

- HFSM 库保持通用，不和具体业务耦合；
- 外部模块只知道事件，不知道状态树细节；
- 后续重构状态树时，对外接口基本不用改；
- 高优先级事件可以通过 `clear + post` 做“清队列后插入”

---

## 7. 层级状态使用范式

下面是一个通用任务状态树示例：

```text
Fault
Auto
├── Idle
├── Navigate
├── Operate
└── ReturnHome
Manual
```

状态创建：

```c
static HfsmState* s_fault = NULL;
static HfsmState* s_auto = NULL;
static HfsmState* s_idle = NULL;
static HfsmState* s_navigate = NULL;
static HfsmState* s_operate = NULL;
static HfsmState* s_return_home = NULL;
static HfsmState* s_manual = NULL;

void app_task_init(void) {
    hfsm.init(&s_task.fsm, &s_task.ctx);

    s_fault = hfsm.add_state(&s_task.fsm, "Fault");
    s_auto = hfsm.add_state(&s_task.fsm, "Auto");
    s_manual = hfsm.add_state(&s_task.fsm, "Manual");

    s_idle = hfsm.add_substate(&s_task.fsm, s_auto, "Idle");
    s_navigate = hfsm.add_substate(&s_task.fsm, s_auto, "Navigate");
    s_operate = hfsm.add_substate(&s_task.fsm, s_auto, "Operate");
    s_return_home = hfsm.add_substate(&s_task.fsm, s_auto, "ReturnHome");

    hfsm.set_handle(s_auto, auto_handle);
    hfsm.set_handle(s_idle, idle_handle);
    hfsm.set_handle(s_navigate, navigate_handle);
    hfsm.set_handle(s_operate, operate_handle);
    hfsm.set_handle(s_manual, manual_handle);
    hfsm.set_handle(s_fault, fault_handle);

    hfsm.set_action(s_navigate, navigate_action);
    hfsm.set_action(s_operate, operate_action);
    hfsm.set_action(s_return_home, return_home_action);

    hfsm.set_initial(&s_task.fsm, s_idle);
    hfsm.start(&s_task.fsm);
}
```

父状态处理公共事件：

```c
static HfsmResult auto_handle(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    if(e->id == APP_EVENT_SWITCH_TO_MANUAL)
        return hfsm.res.transition(s_manual);

    if(e->id == APP_EVENT_STOP)
        return hfsm.res.transition(s_idle);

    if(e->id == APP_EVENT_FAULT)
        return hfsm.res.transition(s_fault);

    return hfsm.res.ignore();
}
```

子状态只处理局部事件：

```c
static HfsmResult navigate_handle(HfsmMachine* m, const HfsmEvent* e) {
    AppContext* ctx = app_ctx_from_machine(m);

    if(ctx == NULL)
        return hfsm.res.ignore();

    if(e->id == APP_EVENT_NAV_REACHED) {
        if(ctx->need_operate)
            return hfsm.res.transition(s_operate);

        if(ctx->is_last_target)
            return hfsm.res.transition(s_return_home);

        return hfsm.res.handled();
    }

    // STOP / FAULT / SWITCH_TO_MANUAL 不在这里重复处理，交给 Auto
    return hfsm.res.ignore();
}
```

状态动作里轮询子流程，并把结果转换成事件：

```c
static void navigate_action(HfsmMachine* m) {
    AppContext* ctx = app_ctx_from_machine(m);

    if(ctx == NULL || ctx->fault_latched)
        return;

    switch(app_navigation_process(ctx)) {
        case APP_NAV_REACHED:
            (void)hfsm.machine.post(m, APP_EVENT_NAV_REACHED, NULL);
            break;

        case APP_NAV_FAILED:
            ctx->fault_latched = true;
            hfsm.machine.clear(m);
            (void)hfsm.machine.post(m, APP_EVENT_FAULT, NULL);
            break;

        case APP_NAV_RUNNING:
        default:
            break;
    }
}
```

这个模式的重点是：

```text
子流程返回值  ->  状态机事件  ->  状态转移
```

不要让子流程直接修改状态机当前状态，这样业务边界更清楚

---

## 8. 事件队列与运行模型

`post()` 不会立即执行状态转移，只是把事件放入队列；状态机只有在 `process()` 或 `process_all()` 被调用时才运行

```text
hfsm.post()         事件入队
hfsm.clear()        清空待处理事件
hfsm.process()      处理一个事件，并执行一次 action
hfsm.process_all()  批量处理队列事件
```

`hfsm.process()` 的内部过程可以理解为：

```text
1. 如果队列里有事件，取出一个事件
2. 从当前状态开始调用 handle()
3. 如果返回 ignore()，继续交给父状态
4. 如果返回 handled()，事件结束
5. 如果返回 transition(target)，执行状态切换
6. 状态切换时按 LCA 规则执行 exit/entry
7. 执行当前状态 action；若 HFSM_RUN_PARENT_ACTIONS 为 true，也会执行祖先状态 action
```

即使当前没有待处理事件，`process()` 也会执行当前状态的 `action`因此 `action` 适合作为状态内周期轮询函数

---

## 9. `entry / exit / action` 执行规则

### 9.1 初始进入

如果初始状态是一个子状态，例如：

```text
Auto
└── Idle
```

调用 `hfsm.start()` 后会从父到子执行：

```text
entry Auto
entry Idle
```

### 9.2 状态切换

状态切换使用最近公共祖先（LCA, Lowest Common Ancestor）决定退出和进入路径

例如：

```text
Auto
├── Navigate
└── Operate
```

从 `Navigate` 切到 `Operate`：

```text
exit Navigate
entry Operate
```

`Auto` 是最近公共祖先，不会退出，也不会重新进入

再例如：

```text
Auto
└── Navigate
Manual
```

从 `Navigate` 切到 `Manual`：

```text
exit Navigate
exit Auto
entry Manual
```

### 9.3 action 执行

默认配置：

```c
#define HFSM_RUN_PARENT_ACTIONS true
```

此时 `process()` 会从当前状态开始，向父状态逐级执行 `action`：

```text
current.action()
parent.action()
grandparent.action()
```

如果你只想执行当前状态的 `action`，在 `hfsm_config.h` 中设置：

```c
#define HFSM_RUN_PARENT_ACTIONS false
```

---

## 10. 生命周期接口

| 接口 | 作用 | 常见使用时机 |
|---|---|---|
| `hfsm.init(&fsm, context)` | 初始化封装层对象，清空状态池和队列 | 创建状态机时调用一次 |
| `hfsm.add_state()` | 添加根状态 | `start()` 前 |
| `hfsm.add_substate()` | 添加子状态 | `start()` 前 |
| `hfsm.set_handle/entry/exit/action()` | 绑定状态回调 | `start()` 前 |
| `hfsm.set_initial()` | 设置初始状态 | `start()` 前 |
| `hfsm.start()` | 进入初始状态，执行初始路径上的 `entry` | 初始化完成后 |
| `hfsm.post()` | 投递事件 | 运行期 |
| `hfsm.clear()` | 清空待处理事件 | 急停、故障、模式强切换前 |
| `hfsm.process()` | 处理一个事件并执行一次动作 | 主循环或周期任务中 |
| `hfsm.process_all()` | 批量处理所有待处理事件 | 需要快速清空事件队列时 |
| `hfsm.pause()` | 暂停状态机处理 | 临时挂起 |
| `hfsm.go_on()` | 从暂停状态恢复 | 继续运行 |
| `hfsm.reset()` | 重新进入初始状态 | 故障恢复、任务重启 |

注意：状态结构和初始状态应在 `start()` 前配置完成；`start()` 后再添加状态或修改初始状态会失败或返回状态码

---

## 11. 状态码

`hfsm.h` 封装层的多数接口会返回 `HfsmStatus`：

| 状态码 | 含义 |
|---|---|
| `hfsm.OK` | 操作成功 |
| `hfsm.INVALID_ARG` | 参数无效，例如空指针或非法事件 ID |
| `hfsm.NOT_INITIALIZE` | 状态机还没有初始化 |
| `hfsm.NO_INITIAL_STATE` | 启动前没有设置初始状态 |
| `hfsm.STARTED` | 当前状态机已经启动，某些配置操作不允许继续执行 |
| `hfsm.NOT_STARTED` | 当前状态机未启动，不能投递或处理事件 |
| `hfsm.NO_SPACE` | 事件队列已满，事件投递失败 |

推荐业务层把状态码转换成模块自己的 `bool`、错误码或日志，而不要把 HFSM 状态码直接扩散到所有业务模块

---

## 12. 配置项

所有配置项位于 `hfsm_config.h`

| 配置项 | 默认值 | 含义 |
|---|---:|---|
| `HFSM_DEPTH` | `8` | 最大状态层级深度 |
| `HFSM_MAX_STATES` | `16` | `Hfsm` 封装层状态池容量 |
| `HFSM_PENDING_QUEUE_MAX` | `8` | 待处理事件队列长度 |
| `HFSM_MAX_CHAIN_LENGTH` | `2 * HFSM_PENDING_QUEUE_MAX` | `process_all()` 最多连续处理的事件数量，避免事件自投递导致死循环 |
| `HFSM_RUN_PARENT_ACTIONS` | `true` | 是否在执行当前状态 `action` 后继续执行父状态 `action` |
| `HFSM_ENABLE_ASSERT` | `true` | 层级深度越界时是否使用 `assert` |
| `HFSM_EVENT_DATA_TYPE` | `HfsmEventData` | 事件携带数据类型 |

默认事件数据类型：

```c
typedef union {
    void* ptr;
    int32_t i32;
    uint32_t u32;
    float f;
} HfsmEventData;
```

自定义事件数据类型示例：

```c
typedef union {
    void* ptr;
    uint32_t u32;
    float f;
    const MyCommand* command;
} AppEventData;

#define HFSM_EVENT_DATA_TYPE AppEventData
```

更稳妥的做法是直接在移植后的 `hfsm_config.h` 中统一修改，保证 `hfsm.c`、`hfsm_core.c` 和所有业务源文件看到的是同一个 `HFSM_EVENT_DATA_TYPE` 定义

---

## 13. 事件数据传递示例

定义事件数据：

```c
typedef union {
    void* ptr;
    uint32_t u32;
    float f;
} AppEventData;

#define HFSM_EVENT_DATA_TYPE AppEventData
```

投递事件：

```c
AppEventData data = { .u32 = 3u };
(void)hfsm.post(&s_fsm, APP_EVENT_SELECT_TARGET, &data);
```

处理事件：

```c
static HfsmResult idle_handle(HfsmMachine* m, const HfsmEvent* e) {
    AppContext* ctx = app_ctx_from_machine(m);

    if(ctx == NULL)
        return hfsm.res.ignore();

    if(e->id == APP_EVENT_SELECT_TARGET) {
        ctx->target_index = (uint8_t)e->data.u32;
        return hfsm.res.handled();
    }

    return hfsm.res.ignore();
}
```

如果传入 `NULL`：

```c
hfsm.post(&s_fsm, APP_EVENT_START, NULL);
```

库会把事件数据区清零

---

## 14. 常见设计建议

### 14.1 让父状态处理公共事件

适合放在父状态的事件：

- 停止；
- 急停；
- 故障；
- 模式切换；
- 暂停/恢复；
- 全局复位

子状态只处理本阶段真正关心的事件；处理不了就返回 `hfsm.res.ignore()`

---

### 14.2 `action` 里只轮询，不直接阻塞

推荐：

```c
static void operate_action(HfsmMachine* m) {
    switch(app_operate_process()) {
        case APP_OPERATE_FINISHED:
            (void)hfsm.machine.post(m, APP_EVENT_STEP_FINISHED, NULL);
            break;

        case APP_OPERATE_FAILED:
            hfsm.machine.clear(m);
            (void)hfsm.machine.post(m, APP_EVENT_FAULT, NULL);
            break;

        default:
            break;
    }
}
```

不推荐在 `action`、`entry` 或 `handle` 中长时间阻塞等待，因为这会阻塞整个状态机驱动循环

---

### 14.3 高优先级事件可以先清队列

对于急停、遥控接管、故障收口等事件，可以采用：

```c
hfsm.clear(&s_fsm);
hfsm.post(&s_fsm, APP_EVENT_FAULT, NULL);
```

或者在应用层封装成：

```c
bool app_task_force_post(AppEventId event_id) {
    (void)hfsm.clear(&s_task.fsm);
    return app_task_post(event_id);
}
```

这样可以避免旧事件在高优先级事件之前继续被处理

---

### 14.4 状态机库不负责线程安全

HFSM 内部没有加锁，也没有关闭中断保护

如果你在以下场景使用：

- ISR 中投递事件，主循环中处理事件；
- 多线程同时 `post/process`；
- RTOS 多任务共享同一个 `Hfsm`；

需要在业务层自行加临界区、互斥锁或消息桥接，避免事件队列并发读写

---

### 14.5 状态指针生命周期必须稳定

使用 `hfsm.h` 封装层时，状态保存在 `Hfsm.states[]` 内部静态数组中，状态指针在该 `Hfsm` 对象生命周期内稳定

注意不要：

- 在 `hfsm.init()` 之后还继续使用旧状态指针；
- 让 `Hfsm` 对象本身位于会失效的局部栈帧中；
- 超过 `HFSM_MAX_STATES` 状态容量

---

## 15. 何时直接使用 `hfsm_core`

默认不建议业务层直接操作 `hfsm_core`；可以直接使用它的情况包括：

- 想完全静态定义 `HfsmState`，不使用 `Hfsm.states[]` 状态池；
- 想更细地控制状态生命周期；
- 想在回调里读取 `dispatching_state` 或判断父子关系；
- 想把内核集成进另一个封装层

直接使用内核的大致形式：

```c
static HfsmMachine s_machine;

static const HfsmState s_idle = {
    .name = "Idle",
    .parent = NULL,
    .handle = idle_handle,
};

hfsm_core.init(&s_machine, &s_idle, &s_ctx);
hfsm_core.post(&s_machine, APP_EVENT_START, NULL);
hfsm_core.process(&s_machine);
```

但在普通业务工程中，优先使用 `hfsm.h` 封装层更安全，因为它提供了状态池、启动状态检查和统一状态码

---

## 16. 推荐落地模板

一个可维护的业务状态机模块可以按下面结构组织：

```text
app_task.h
  - 对外事件投递接口
  - 对外状态查询接口

app_task_internal.h
  - AppTask 结构体
  - AppContext 定义或引用
  - 仅内部源文件使用

app_task.c
  - Hfsm 实例
  - HfsmState* 状态指针
  - init/process/post/force_post
  - handle/entry/exit/action 回调
  - ctx_from_machine() 辅助函数

sub_task_xxx.c
  - 子流程本身
  - 返回 RUNNING / FINISHED / FAILED
  - 不直接修改 HFSM 当前状态
```

核心边界是：

```text
HFSM 负责状态编排
子流程负责执行具体动作
外部模块只投递事件或查询状态
```

这样 `infra/hfsm` 仍然是独立库，业务工程只是在它之上构建自己的任务状态机

一个面向嵌入式与业务状态编排场景的轻量级层级有限状态机（Hierarchical Finite State Machine）实现

该仓库采用 **core 内核 + API 包装层 + 配置头** 的分层方式：

- `hfsm_core.*`：最小内核，负责事件队列、层级分发、LCA 转换、entry/exit/action 执行
- `hfsm.*`：业务友好包装层，补充状态池、生命周期状态码、更易用接口
- `hfsm_config.h`：可裁剪配置项（深度、状态数、队列长度、事件数据类型等）

---

## 特性

- 纯 C 实现，适合 MCU / Linux 用户态
- 支持层级状态与父状态兜底处理
- 支持 `entry / exit / action / handle`
- 支持事件队列、单步处理与批量处理
- 支持用户上下文 `context` 与状态私有 `user_data`
- 无动态内存分配（静态状态池 + 静态队列）

---

## 目录结构

```text
hfsm/
├── hfsm.h
├── hfsm.c
├── hfsm_core.h
├── hfsm_core.c
├── hfsm_config.h
└── README.md
```

---

## 核心数据结构

### `HfsmState`

- `name`：状态名
- `parent`：父状态
- `handle`：事件处理函数
- `entry`：进入钩子
- `exit`：退出钩子
- `action`：周期动作
- `user_data`：状态私有数据

### `HfsmMachine`（内核层）

- `current_state`：当前状态
- `dispatching_state`：当前正在处理的状态（回调中可读）
- `context`：用户上下文
- `queue[]`：环形事件队列

### `Hfsm`（包装层）

- `machine`：内核实例
- `initial_state`：初始状态
- `states[]`：状态池（容量 `HFSM_MAX_STATES`）
- `initialized / started`：生命周期标记

---

## 运行模型

### 1. 初始化

先初始化对象，再设置状态与初始状态

### 2. 启动

调用 `start()` 后，状态机进入初始状态并沿父链执行 `entry`

### 3. 事件投递

`post()` 仅入队，不立即处理

### 4. 事件分发

从 `current_state` 向父状态逐级分发：

- 返回 `IGNORE`：继续向父状态传递
- 返回 `HANDLED`：事件结束
- 返回 `TRANSITION(next)`：执行切换

### 5. 状态切换

基于最近公共祖先（LCA）：

- 从当前状态向上 `exit` 到 LCA（不含 LCA）
- 从 LCA 向下 `entry` 到目标状态（不含 LCA）

### 6. 动作执行

- `process()`：每次调用最多处理 1 个事件，然后执行 action
- `process_all()`：批量处理队列；若批处理中发生状态切换，会在切换后执行 action
- 当 `HFSM_RUN_PARENT_ACTIONS == true` 时，action 执行顺序为 **当前状态 -> 父状态 -> ... -> 根状态**
- 当 `HFSM_RUN_PARENT_ACTIONS == false` 时，只执行当前状态 action

---

## 返回值语义（`HfsmResult`）

- `hfsm_core.res.ignore()` / `hfsm.res.ignore()`：忽略当前事件
- `hfsm_core.res.handled()` / `hfsm.res.handled()`：事件已处理但不跳转
- `hfsm_core.res.transition(next)` / `hfsm.res.transition(next)`：事件已处理并跳转

---

## 配置项（`hfsm_config.h`）

```c
#ifndef HFSM_DEPTH
#define HFSM_DEPTH 8
#endif

#ifndef HFSM_MAX_STATES
#define HFSM_MAX_STATES 16
#endif

#ifndef HFSM_PENDING_QUEUE_MAX
#define HFSM_PENDING_QUEUE_MAX 8
#endif

#ifndef HFSM_MAX_CHAIN_LENGTH
#define HFSM_MAX_CHAIN_LENGTH 2 * HFSM_PENDING_QUEUE_MAX
#endif
```

自定义事件载荷类型：

```c
typedef union {
    void* ptr;
    const MyMsg* msg;
    uint32_t cmd;
} MissionEventData;
#define HFSM_EVENT_DATA_TYPE MissionEventData
#include "hfsm_core.h"   // 或 #include "hfsm.h"
```

---

## API 总览

### 内核层 `hfsm_core.h`

生命周期与运行：

```c
void hfsm_core.init(HfsmMachine* m, const HfsmState* initial_state, void* context);
bool hfsm_core.post(HfsmMachine* m, HfsmEventId event_id, const void* data);
void hfsm_core.clear(HfsmMachine* m);
void hfsm_core.process(HfsmMachine* m);
void hfsm_core.process_all(HfsmMachine* m);
```

查询与工具：

```c
const HfsmState* hfsm_core.state(const HfsmMachine* m);
const HfsmState* hfsm_core.dispatching(const HfsmMachine* m);
void* hfsm_core.context(HfsmMachine* m);
const void* hfsm_core.const_context(const HfsmMachine* m);
bool hfsm_core.transition(HfsmMachine* m, const HfsmState* target_state);
bool hfsm_core.has_pending(const HfsmMachine* m);
```

结果构造：

```c
hfsm_core.res.ignore();
hfsm_core.res.handled();
hfsm_core.res.transition(next_state);
```

> 说明：内核层不负责“创建状态”；`HfsmState` 由用户自行定义（通常静态定义）

### 封装层 `hfsm.h`

生命周期：

```c
HfsmStatus hfsm.init(Hfsm* fsm, void* context);
HfsmStatus hfsm.start(Hfsm* fsm);
HfsmStatus hfsm.pause(Hfsm* fsm);
HfsmStatus hfsm.go_on(Hfsm* fsm);
HfsmStatus hfsm.reset(Hfsm* fsm);
```

状态构建：

```c
HfsmState* hfsm.add_state(Hfsm* fsm, const char* name);
HfsmState* hfsm.add_substate(Hfsm* fsm, HfsmState* parent, const char* name);
HfsmState* hfsm.set_parent(HfsmState* s, const HfsmState* parent);
HfsmStatus hfsm.set_initial(Hfsm* fsm, const HfsmState* initial_state);
```

状态配置：

```c
HfsmState* hfsm.set_handle(HfsmState* s, HfsmHandleFn handle);
HfsmState* hfsm.set_entry(HfsmState* s, HfsmHookFn entry);
HfsmState* hfsm.set_exit(HfsmState* s, HfsmHookFn exit);
HfsmState* hfsm.set_action(HfsmState* s, HfsmHookFn action);
HfsmState* hfsm.set_user_data(HfsmState* s, void* user_data);
```

事件处理：

```c
HfsmStatus hfsm.post(Hfsm* fsm, HfsmEventId event_id, const void* data);
HfsmStatus hfsm.clear(Hfsm* fsm);
HfsmStatus hfsm.process(Hfsm* fsm);
HfsmStatus hfsm.process_all(Hfsm* fsm);
```

查询：

```c
const HfsmState* hfsm.state(const Hfsm* fsm);
const HfsmState* hfsm.dispatching(const Hfsm* fsm);
void* hfsm.context(Hfsm* fsm);
const void* hfsm.const_context(const Hfsm* fsm);
```

---

## 快速开始

### 直接使用内核

这类用法只用 `hfsm_core.h`，不使用 `hfsm.h` 的业务友好 API

```c
#include <stdint.h>
typedef enum {
    EVT_NONE = 0,
    EVT_START,
    EVT_STOP
} AppEvent;

typedef struct {
    int counter;
} AppContext;

// 可选：自定义事件数据类型；若不需要可直接用默认 HfsmEventData
typedef union {
    void* ptr;
    int32_t i32;
} AppEventData;
#define HFSM_EVENT_DATA_TYPE AppEventData
#include "hfsm_core.h"

static HfsmMachine g_machine;
static AppContext g_ctx;

static HfsmResult handle_idle(HfsmMachine* m, const HfsmEvent* e);
static HfsmResult handle_running(HfsmMachine* m, const HfsmEvent* e);

static const HfsmState state_idle = {
    .name = "idle",
    .parent = NULL,
    .handle = handle_idle,
    .entry = NULL,
    .exit = NULL,
    .action = NULL,
    .user_data = NULL
};

static const HfsmState state_running = {
    .name = "running",
    .parent = NULL,
    .handle = handle_running,
    .entry = NULL,
    .exit = NULL,
    .action = NULL,
    .user_data = NULL
};

static HfsmResult handle_idle(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;
    if((AppEvent)e->id == EVT_START) {
        return hfsm_core.res.transition(&state_running);
    }
    return hfsm_core.res.ignore();
}

static HfsmResult handle_running(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;
    if((AppEvent)e->id == EVT_STOP) {
        return hfsm_core.res.transition(&state_idle);
    }
    return hfsm_core.res.ignore();
}

void app_init(void) {
    g_ctx.counter = 0;
    hfsm_core.init(&g_machine, &state_idle, &g_ctx);
}

void app_step(void) {
    hfsm_core.process(&g_machine);
}

void app_start(void) {
    AppEventData data = {0};
    hfsm_core.post(&g_machine, (HfsmEventId)EVT_START, &data);
}
```

### 使用业务友好 API

```c
// 可选：自定义事件数据类型；若不需要可直接用默认 HfsmEventData
typedef union {
    void* ptr;
    int32_t i32;
} AppEventData;
#define HFSM_EVENT_DATA_TYPE AppEventData
#include "hfsm.h"

enum {
    EVT_START = 1,
    EVT_STOP = 2
};

static HfsmState* s_idle;
static HfsmState* s_running;

static HfsmResult idle_handle(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;
    if(e->id == EVT_START) return hfsm.res.transition(s_running);
    return hfsm.res.ignore();
}

static HfsmResult running_handle(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;
    if(e->id == EVT_STOP) return hfsm.res.transition(s_idle);
    return hfsm.res.ignore();
}

void demo(void) {
    Hfsm fsm;
    hfsm.init(&fsm, NULL);

    s_idle = hfsm.add_state(&fsm, "idle");
    s_running = hfsm.add_state(&fsm, "running");

    hfsm.set_handle(s_idle, idle_handle);
    hfsm.set_handle(s_running, running_handle);
    hfsm.set_initial(&fsm, s_idle);

    hfsm.start(&fsm);
    hfsm.post(&fsm, EVT_START, NULL);
    hfsm.process(&fsm);
}
```

---

## 层级状态示例

```text
Root
├── Idle
└── Work
    ├── Prepare
    ├── Execute
    └── Recover
```

若当前状态是 `Execute`，事件会先到 `Execute.handle()`，忽略后再上送给 `Work`，再到 `Root`

---

## 典型流程

```text
A. 仅内核：
1) 定义静态 HfsmState
2) hfsm_core.init()
3) hfsm_core.post()
4) hfsm_core.process() / process_all()

B. 包装层：
1) hfsm.init()
2) add_state()/add_substate()
3) set_handle()/set_entry()/set_exit()/set_action()
4) set_initial()
5) start()
6) post() + process()
```

---

## 资源限制与工程建议

- `HFSM_MAX_STATES`：包装层状态池上限
- `HFSM_DEPTH`：层级深度上限
- `HFSM_PENDING_QUEUE_MAX`：队列长度上限
- `HFSM_MAX_CHAIN_LENGTH`：`process_all()` 单次处理上限
- 默认不保证线程安全；多线程或中断场景需自行加锁或单线程调度

---

## 编译集成

```cmake
add_library(hfsm
    hfsm.c
    hfsm_core.c
)

target_include_directories(hfsm PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
```

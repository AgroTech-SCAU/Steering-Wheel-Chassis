# HFSM：轻量级层级有限状态机

## 1. 概述

`hfsm` 是一个面向 **嵌入式系统、机器人任务流程、业务状态编排** 的轻量级 C 语言层级有限状态机（Hierarchical Finite State Machine, HFSM）实现

它适合用来描述类似下面这类逻辑：

```text
系统启动
  ├── 空闲
  ├── 工作
  │   ├── 准备
  │   ├── 执行
  │   └── 恢复
  └── 故障
```

与普通 FSM 相比，HFSM 的重点是：**子状态处理不了的事件，可以自动交给父状态兜底处理**；这样可以减少大量重复判断，让状态逻辑更清晰

---

## 2. 特性

- 纯 C 实现，适合 MCU、RTOS、裸机、Linux 用户态程序；
- 不依赖动态内存分配，状态池和事件队列均为静态容量；
- 支持层级状态：事件可从子状态逐级上送给父状态；
- 支持 `entry`、`exit`、`action`、`handle` 四类状态回调；
- 支持事件队列：`post()` 只投递事件，`process()` 才处理事件；
- 支持用户上下文 `context` 和状态私有数据 `user_data`；
- 支持 `process()` 单步处理和 `process_all()` 批量处理；
- 配置项集中在 `hfsm_config.h`，便于裁剪容量

---

## 3. 目录结构

```text
hfsm/
├── hfsm.h              # 业务友好封装层：推荐初学者优先使用
├── hfsm.c
├── hfsm_core.h         # 最小内核层：适合高级用户直接静态定义状态
├── hfsm_core.c
├── hfsm_config.h       # 容量与行为配置
├── examples/
│   └── quick_start.c   # 最小可运行示例
└── README.md
```

分层关系如下：

```text
业务代码
   ↓
hfsm.h / hfsm.c              	推荐入口：状态池、生命周期状态码、易用 API
   ↓
hfsm_core.h / hfsm_core.c       内核：事件队列、层级分发、entry/exit/action、LCA 转换
   ↓
hfsm_config.h                   配置：状态数量、层级深度、队列长度、事件数据类型
```

初学者建议直接使用 `hfsm.h`，暂时不要直接操作 `hfsm_core.h`

---

## 4. 快速开始

### 在 STM32CubeIDE / Keil / 裸机工程中使用

把下面 5 个文件加入工程即可：

```text
hfsm.h
hfsm.c
hfsm_core.h
hfsm_core.c
hfsm_config.h
```

并保证头文件路径能找到 `hfsm.h`，业务代码中：

```c
#include "hfsm.h"
```

---

## 5. 核心类型定义

### 5.1 状态 State

一个状态由 `HfsmState` 表示：

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

每个状态可以有：

| 字段 | 含义 |
|---|---|
| `name` | 状态名称，主要用于调试 |
| `parent` | 父状态，用于形成层级结构 |
| `handle` | 事件处理函数 |
| `entry` | 进入状态时执行 |
| `exit` | 离开状态时执行 |
| `action` | 每次 `process()` 后执行的周期动作 |
| `user_data` | 状态私有数据，可选 |

### 5.2 事件 Event

事件由 `HfsmEvent` 表示：

```c
typedef struct {
    HfsmEventId id;
    HFSM_EVENT_DATA_TYPE data;
} HfsmEvent;
```

其中：

- `id` 是事件编号；
- `data` 是事件携带的数据；
- `HFSM_EVENT_NONE` 等于 0，不能作为有效事件发送

你通常会自己定义事件枚举：

```c
typedef enum {
    EVT_START = 1,
    EVT_TICK,
    EVT_STOP,
    EVT_ERROR,
} AppEvent;
```

### 5.3 事件处理结果 HfsmResult

状态处理事件后，必须返回一种结果：

| 返回值 | 含义 |
|---|---|
| `hfsm.res.ignore()` | 当前状态不处理该事件，继续交给父状态 |
| `hfsm.res.handled()` | 事件已处理，不切换状态 |
| `hfsm.res.transition(target)` | 事件已处理，并切换到目标状态 |

例如：

```c
static HfsmResult idle_handle(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    if(e->id == EVT_START) {
        return hfsm.res.transition(s_work);
    }

    return hfsm.res.ignore();
}
```

### 5.4 用户上下文 context

`context` 是全局业务上下文，常用于保存电机对象、传感器数据、任务参数等：

```c
typedef struct {
    uint32_t tick_count;
    float target_speed;
} AppContext;
```

初始化时传入：

```c
AppContext ctx = {0};
Hfsm fsm;

hfsm.init(&fsm, &ctx);
```

在回调函数里取出：

```c
static void work_action(HfsmMachine* m) {
    AppContext* ctx = (AppContext*)hfsm_core.context(m);
    ctx->tick_count++;
}
```

注意：状态回调函数收到的是 `HfsmMachine*`，所以回调内部使用 `hfsm_core.context(m)` 获取上下文

### 5.5 状态私有数据 user_data

`context` 适合放整个状态机共享的数据；`user_data` 适合放某个状态自己的数据

设置：

```c
static int work_counter = 0;
hfsm.set_user_data(s_work, &work_counter);
```

在回调中获取当前正在执行的状态：

```c
static void work_action(HfsmMachine* m) {
    const HfsmState* s = hfsm_core.dispatching(m);
    int* counter = (int*)s->user_data;
    (*counter)++;
}
```

---

## 6. 流程演示

下面示例代码演示了：

- 创建状态机；
- 添加 `Idle` 和 `Work` 两个状态；
- 绑定事件处理函数；
- 绑定 `entry / exit / action`；
- 投递事件并驱动状态机运行（通常 process 放在主循环中，在需要的地方用 post 触发事件）

```c
#include <stdio.h>
#include <stdint.h>
#include "hfsm.h"

typedef enum {
    EVT_START = 1,
    EVT_TICK,
    EVT_STOP,
} AppEvent;

typedef struct {
    uint32_t tick_count;
} AppContext;

static HfsmState* s_idle = NULL;
static HfsmState* s_work = NULL;

static HfsmResult idle_handle(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    if(e->id == EVT_START) {
        printf("Idle handle EVT_START -> Work\n");
        return hfsm.res.transition(s_work);
    }

    return hfsm.res.ignore();
}

static HfsmResult work_handle(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    if(e->id == EVT_TICK) {
        printf("Work handle EVT_TICK\n");
        return hfsm.res.handled();
    }

    if(e->id == EVT_STOP) {
        printf("Work handle EVT_STOP -> Idle\n");
        return hfsm.res.transition(s_idle);
    }

    return hfsm.res.ignore();
}

static void work_entry(HfsmMachine* m) {
    AppContext* ctx = (AppContext*)hfsm_core.context(m);
    ctx->tick_count = 0;
    printf("enter Work\n");
}

static void work_exit(HfsmMachine* m) {
    AppContext* ctx = (AppContext*)hfsm_core.context(m);
    printf("exit Work, tick_count=%u\n", (unsigned)ctx->tick_count);
}

static void work_action(HfsmMachine* m) {
    AppContext* ctx = (AppContext*)hfsm_core.context(m);
    ctx->tick_count++;
    printf("Work action, tick_count=%u\n", (unsigned)ctx->tick_count);
}

int main(void) {
    Hfsm fsm;
    AppContext ctx = {0};

    hfsm.init(&fsm, &ctx);

    s_idle = hfsm.add_state(&fsm, "Idle");
    s_work = hfsm.add_state(&fsm, "Work");

    hfsm.set_handle(s_idle, idle_handle);
    hfsm.set_handle(s_work, work_handle);
    hfsm.set_entry(s_work, work_entry);
    hfsm.set_exit(s_work, work_exit);
    hfsm.set_action(s_work, work_action);

    hfsm.set_initial(&fsm, s_idle);
    hfsm.start(&fsm);

    printf("current: %s\n", hfsm.state(&fsm)->name);

    hfsm.post(&fsm, EVT_START, NULL);
    hfsm.process(&fsm);
    printf("current: %s\n", hfsm.state(&fsm)->name);

    hfsm.post(&fsm, EVT_TICK, NULL);
    hfsm.process(&fsm);

    hfsm.post(&fsm, EVT_STOP, NULL);
    hfsm.process(&fsm);
    printf("current: %s\n", hfsm.state(&fsm)->name);

    return 0;
}
```

---

## 7. 推荐使用流程

使用 `hfsm.h` 封装层时，推荐按这个顺序写：

```text
1. 定义事件枚举
2. 定义用户上下文结构体
3. 定义 Hfsm 对象
4. 定义状态指针
5. 编写 handle / entry / exit / action 回调
6. hfsm.init()
7. hfsm.add_state() / hfsm.add_substate()
8. hfsm.set_handle() / set_entry() / set_exit() / set_action()
9. hfsm.set_initial()
10. hfsm.start()
11. hfsm.post()
12. hfsm.process() 或 hfsm.process_all()
```

示例代码骨架：

```c
Hfsm fsm;
AppContext ctx;

hfsm.init(&fsm, &ctx);

HfsmState* idle = hfsm.add_state(&fsm, "Idle");
HfsmState* work = hfsm.add_state(&fsm, "Work");

hfsm.set_handle(idle, idle_handle);
hfsm.set_handle(work, work_handle);
hfsm.set_initial(&fsm, idle);

hfsm.start(&fsm);

while(1) {
    // 根据外设、通信、定时器、任务输入投递事件
    // hfsm.post(&fsm, EVT_xxx, &data);

    // 驱动状态机运行
    hfsm.process(&fsm);
}
```

---

## 8. 事件运行模型

`hfsm` 不是在 `post()` 时立即处理事件，而是采用队列模型：

```text
hfsm.post()       只把事件放入队列
hfsm.process()    从队列取出一个事件并处理
hfsm.process_all() 批量处理队列中的事件
```

一次 `process()` 的内部流程可以理解为：

```text
1. 检查当前状态是否有效
2. 如果队列里有事件，取出一个事件
3. 从当前状态开始调用 handle()
4. 如果返回 ignore()，事件交给父状态
5. 如果返回 handled()，事件结束
6. 如果返回 transition(target)，执行状态切换
7. 状态切换时自动执行 exit 和 entry
8. 最后执行 action
```

事件分发规则：

```text
当前状态 handle()
    ↓ ignore
父状态 handle()
    ↓ ignore
祖父状态 handle()
    ↓ ignore
最终事件被忽略
```

只要某一级状态返回 `handled()` 或 `transition()`，事件就不会继续向父状态传递

---

## 9. 层级状态示例

假设状态结构如下：

```text
Root
├── Idle
└── Work
    ├── Prepare
    ├── Execute
    └── Recover
```

如果当前状态是 `Execute`，事件处理顺序是：

```text
Execute.handle()
  -> Work.handle()
  -> Root.handle()
```

这适合处理公共事件，例如：

- `Execute` 只处理执行阶段自己的事件；
- `Work` 统一处理工作模式下的暂停、恢复、故障；
- `Root` 统一处理急停、复位等全局事件

创建层级状态：

```c
HfsmState* root = hfsm.add_state(&fsm, "Root");
HfsmState* idle = hfsm.add_substate(&fsm, root, "Idle");
HfsmState* work = hfsm.add_substate(&fsm, root, "Work");
HfsmState* prepare = hfsm.add_substate(&fsm, work, "Prepare");
HfsmState* execute = hfsm.add_substate(&fsm, work, "Execute");
HfsmState* recover = hfsm.add_substate(&fsm, work, "Recover");
```

父状态兜底处理示例：

```c
static HfsmResult work_handle(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    if(e->id == EVT_PAUSE) {
        return hfsm.res.transition(s_pause);
    }

    if(e->id == EVT_ERROR) {
        return hfsm.res.transition(s_error);
    }

    return hfsm.res.ignore();
}
```

子状态中不需要重复写 `EVT_PAUSE` 和 `EVT_ERROR`，只要子状态返回 `ignore()`，父状态就有机会处理

---

## 10. 状态切换时 entry / exit 的执行顺序

状态切换使用最近公共祖先（LCA, Lowest Common Ancestor）来决定退出和进入路径

例如：

```text
Work
├── Prepare
└── Execute
```

从 `Prepare` 切换到 `Execute`：

```text
exit Prepare
entry Execute
```

因为它们的最近公共祖先是 `Work`，所以 `Work` 不会退出，也不会重新进入

再例如：

```text
Root
├── Idle
└── Work
    └── Execute
```

从 `Execute` 切换到 `Idle`：

```text
exit Execute
exit Work
entry Idle
```

因为最近公共祖先是 `Root`

规则总结：

```text
exit：从当前状态向上执行，直到 LCA，不包含 LCA
entry：从 LCA 向下执行，直到目标状态，不包含 LCA，包含目标状态
```

---

## 11. action 的执行时机

`action` 是状态的周期动作

调用 `hfsm.process(&fsm)` 后：

- 如果有事件，会先处理一个事件；
- 如果发生状态切换，会先完成 `exit / entry`；
- 最后执行当前状态的 `action`

默认配置下：

```c
#define HFSM_RUN_PARENT_ACTIONS true
```

此时 action 执行顺序是：

```text
当前状态 action -> 父状态 action -> 祖父状态 action -> ... -> 根状态 action
```

如果只希望执行当前状态的 action，可以在编译配置中关闭：

```c
#define HFSM_RUN_PARENT_ACTIONS false
```

---

## 12. process() 和 process_all() 的区别

| 函数 | 行为 | 适合场景 |
|---|---|---|
| `hfsm.process(&fsm)` | 每次最多处理 1 个事件，然后执行 action | 主循环周期调用，节奏稳定 |
| `hfsm.process_all(&fsm)` | 一次尽量处理完整个事件队列 | 初始化阶段、批量补处理、非实时业务 |

一般嵌入式主循环建议使用：

```c
while(1) {
    collect_input_and_post_events();
    hfsm.process(&fsm);
}
```

如果你的事件可能在短时间内堆积，并且希望尽快清空队列，可以使用：

```c
hfsm.process_all(&fsm);
```

`process_all()` 受 `HFSM_MAX_CHAIN_LENGTH` 限制，避免事件链条过长导致长时间占用 CPU

---

## 13. API 说明

### 13.1 生命周期

```c
HfsmStatus hfsm.init(Hfsm* fsm, void* context);
HfsmStatus hfsm.set_context(Hfsm* fsm, void* context);
HfsmStatus hfsm.set_initial(Hfsm* fsm, const HfsmState* initial_state);
HfsmStatus hfsm.start(Hfsm* fsm);
HfsmStatus hfsm.pause(Hfsm* fsm);
HfsmStatus hfsm.go_on(Hfsm* fsm);
HfsmStatus hfsm.reset(Hfsm* fsm);
```

说明：

- `init()`：清空并初始化 `Hfsm` 对象；
- `set_context()`：设置用户上下文，必须在启动前调用；
- `set_initial()`：设置初始状态，必须在启动前调用；
- `start()`：启动状态机，进入初始状态；
- `pause()`：暂停处理事件，保持当前状态不变；
- `go_on()`：从暂停状态恢复；
- `reset()`：重新进入初始状态

### 13.2 状态构建

```c
HfsmState* hfsm.add_state(Hfsm* fsm, const char* name);
HfsmState* hfsm.add_substate(Hfsm* fsm, HfsmState* parent, const char* name);
HfsmState* hfsm.set_parent(HfsmState* s, const HfsmState* parent);
```

说明：

- `add_state()`：添加一个顶层状态；
- `add_substate()`：添加一个子状态；
- `set_parent()`：手动设置父状态

注意：状态由 `Hfsm` 内部的静态状态池保存，数量受 `HFSM_MAX_STATES` 限制

### 13.3 状态回调设置

```c
HfsmState* hfsm.set_handle(HfsmState* s, HfsmHandleFn handle);
HfsmState* hfsm.set_entry(HfsmState* s, HfsmHookFn entry);
HfsmState* hfsm.set_exit(HfsmState* s, HfsmHookFn exit);
HfsmState* hfsm.set_action(HfsmState* s, HfsmHookFn action);
HfsmState* hfsm.set_user_data(HfsmState* s, void* user_data);
```

回调函数原型：

```c
typedef HfsmResult(*HfsmHandleFn)(HfsmMachine* m, const HfsmEvent* e);
typedef void(*HfsmHookFn)(HfsmMachine* m);
```

### 13.4 事件处理

```c
HfsmStatus hfsm.post(Hfsm* fsm, HfsmEventId event_id, const void* data);
HfsmStatus hfsm.clear(Hfsm* fsm);
HfsmStatus hfsm.process(Hfsm* fsm);
HfsmStatus hfsm.process_all(Hfsm* fsm);
```

说明：

- `post()`：投递事件到队列；
- `clear()`：清空事件队列；
- `process()`：处理一个事件；
- `process_all()`：批量处理事件

### 13.5 查询接口

```c
const HfsmState* hfsm.state(const Hfsm* fsm);
const HfsmState* hfsm.dispatching(const Hfsm* fsm);
void* hfsm.context(Hfsm* fsm);
const void* hfsm.const_context(const Hfsm* fsm);
```

说明：

- `state()`：获取当前状态；
- `dispatching()`：获取正在执行回调的状态；
- `context()`：获取用户上下文；
- `const_context()`：获取只读上下文

在状态回调内部，你拿到的是 `HfsmMachine* m`，因此更常用的是内核查询接口：

```c
void* ctx = hfsm_core.context(m);
const HfsmState* current = hfsm_core.state(m);
const HfsmState* running = hfsm_core.dispatching(m);
```

### 13.6 事件处理结果

```c
HfsmResult hfsm.res.ignore(void);
HfsmResult hfsm.res.handled(void);
HfsmResult hfsm.res.transition(const HfsmState* next_state);
```

---

## 14. 状态码说明

`hfsm` 封装层的函数通常返回 `HfsmStatus`：

| 状态码 | 含义 | 常见原因 |
|---|---|---|
| `HFSM_STATUS_OK` | 成功 | 操作正常完成 |
| `HFSM_STATUS_INVALID_ARG` | 参数非法 | 传入空指针、事件 ID 为 0 |
| `HFSM_STATUS_NOT_INITIALIZE` | 未初始化 | 没有先调用 `hfsm.init()` |
| `HFSM_STATUS_NO_INITIAL_STATE` | 未设置初始状态 | 没有调用 `hfsm.set_initial()` |
| `HFSM_STATUS_STARTED` | 已启动 | 启动后又尝试修改初始化阶段配置 |
| `HFSM_STATUS_NOT_STARTED` | 未启动 | 没有调用 `hfsm.start()`，或已经 `pause()` |
| `HFSM_STATUS_NO_SPACE` | 队列满 | 事件投递太快，未及时 `process()` |

由于头文件中定义了：

```c
#define hfsm hfsm_api_instance
```

所以也可以用下面这种形式判断：

```c
HfsmStatus st = hfsm.post(&fsm, EVT_START, NULL);
if(st != hfsm.OK) {
    // handle error
}
```

---

## 15. 配置项

配置项集中在 `hfsm_config.h`

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

### 15.1 常用配置解释

| 配置项 | 默认值 | 含义 |
|---|---:|---|
| `HFSM_DEPTH` | 8 | 状态层级最大深度 |
| `HFSM_MAX_STATES` | 16 | 封装层状态池最大状态数 |
| `HFSM_PENDING_QUEUE_MAX` | 8 | 事件队列最大长度 |
| `HFSM_MAX_CHAIN_LENGTH` | `2 * HFSM_PENDING_QUEUE_MAX` | `process_all()` 最多连续处理多少个事件 |
| `HFSM_RUN_PARENT_ACTIONS` | `true` | 是否执行父状态 action |
| `HFSM_ENABLE_ASSERT` | `true` | 是否启用断言检查 |

### 15.2 自定义事件数据类型

默认事件数据类型是：

```c
typedef union {
    void* ptr;
    int32_t i32;
    uint32_t u32;
    float f;
} HfsmEventData;
```

如果需要携带自己的数据，可以自定义：

```c
typedef union {
    void* ptr;
    uint32_t cmd;
    float speed;
    const MyMessage* msg;
} AppEventData;

#define HFSM_EVENT_DATA_TYPE AppEventData
#include "hfsm.h"
```

但必须注意：`HFSM_EVENT_DATA_TYPE` 会影响 `HfsmEvent` 的内存布局，因此所有编译单元必须使用同一个定义；也就是说，不能只在 `main.c` 中定义，而 `hfsm.c` / `hfsm_core.c` 仍然使用默认定义

更稳妥的做法是在工程里维护一个统一的配置头

---

## 16. 封装层和内核层如何选择

### 16.1 推荐初学者使用封装层 `hfsm.h`

封装层提供：

- `Hfsm` 对象；
- 状态池；
- `add_state()` / `add_substate()`；
- 生命周期状态码；
- 更适合业务代码的 API

典型用法：

```c
Hfsm fsm;
hfsm.init(&fsm, &ctx);
HfsmState* idle = hfsm.add_state(&fsm, "Idle");
```

### 16.2 什么时候直接使用内核层 `hfsm_core.h`

如果你希望状态全部静态定义，不需要 `Hfsm` 的状态池，可以直接用内核层并裁剪掉封装层

内核层特点：

- 更小；
- 没有生命周期状态码；
- 不负责创建状态；
- `HfsmState` 通常由用户手动静态定义

示意：

```c
static const HfsmState state_idle = {
    .name = "Idle",
    .parent = NULL,
    .handle = idle_handle,
};

static HfsmMachine machine;
hfsm_core.init(&machine, &state_idle, &ctx);
```

如果只是正常写业务流程，优先使用 `hfsm.h`

---

## 17. 常见问题与排查

### 17.1 为什么 `post()` 之后状态没有变化？

`post()` 只负责把事件放进队列，不会立即处理事件

你还需要调用：

```c
hfsm.process(&fsm);
```

或：

```c
hfsm.process_all(&fsm);
```

### 17.2 为什么 `post()` 返回 `NOT_STARTED`？

常见原因：

- 没有调用 `hfsm.start()`；
- 调用了 `hfsm.pause()` 之后还没有调用 `hfsm.go_on()`

正确顺序：

```c
hfsm.init(&fsm, &ctx);
...
hfsm.set_initial(&fsm, s_idle);
hfsm.start(&fsm);
hfsm.post(&fsm, EVT_START, NULL);
```

### 17.3 为什么事件没有被任何状态处理？

检查以下几点：

- 当前状态是否设置了 `handle`；
- 父状态是否设置了 `handle`；
- 事件 ID 是否写错；
- 子状态是否返回了 `handled()`，导致事件没有继续上传；
- 是否误用了 `HFSM_EVENT_NONE`，它的值为 0，不能作为有效事件

### 17.4 为什么状态创建失败？

`hfsm.add_state()` 返回 `NULL` 时，常见原因是：

- `fsm` 没有初始化；
- `name` 是空指针；
- 状态数量超过 `HFSM_MAX_STATES`；
- 状态机已经启动，启动后不允许继续添加状态

### 17.5 为什么进入深层状态时断言失败？

可能是状态层级超过了 `HFSM_DEPTH`

解决方法：

```c
#define HFSM_DEPTH 12
```

### 17.6 多线程或中断里能直接调用吗？

当前实现默认不提供线程安全保护

如果在中断、RTOS 多任务或多线程里调用 `post()` / `process()`，需要自行保证互斥，例如：

- 只在一个任务中调用 `hfsm.process()`；
- 中断里只置标志，主循环里再 `post()`；
- RTOS 下对 `post()` 和 `process()` 加锁；
- 或使用消息队列把外部事件统一转发到状态机任务

### 17.7 状态名称是否会自动检查重复？

不会，`name` 主要用于调试阅读，当前实现不会强制检查状态名称唯一性

建议你在业务代码中保持名称唯一，便于日志定位

---

## 18. 设计建议

### 18.1 事件命名建议

推荐按业务含义命名，而不是按硬件输入命名：

```c
typedef enum {
    EVT_START = 1,
    EVT_STOP,
    EVT_PAUSE,
    EVT_RESUME,
    EVT_ERROR,
    EVT_TIMEOUT,
} AppEvent;
```

不建议：

```c
typedef enum {
    EVT_KEY1_DOWN = 1,
    EVT_UART_RX_0X03,
} AppEvent;
```

硬件输入可以在外层转换成业务事件

### 18.2 状态划分建议

状态应该表示系统当前处于什么阶段，而不是表示某一个函数是否执行过

推荐：

```text
Idle
Prepare
Running
Recover
Error
```

不推荐：

```text
ReadSensor
CalcPid
SetPwm
Delay10ms
```

后者更像普通流程函数，不适合拆成状态

### 18.3 父状态适合放公共逻辑

例如：

```text
Work
├── Prepare
├── Execute
└── Recover
```

`Work` 可以统一处理：

- 暂停；
- 故障；
- 急停；
- 工作模式下的公共 action

子状态只处理自己的局部事件

---

## 19. 状态结构参考

例如一个简单电机任务可以这样组织：

```text
Root
├── Idle
├── Manual
├── Auto
│   ├── AutoPrepare
│   ├── AutoRun
│   └── AutoFinish
└── Fault
```

事件可以这样定义：

```c
typedef enum {
    EVT_START = 1,
    EVT_STOP,
    EVT_MANUAL_MODE,
    EVT_AUTO_MODE,
    EVT_PREPARE_DONE,
    EVT_TARGET_REACHED,
    EVT_FAULT,
    EVT_RESET,
} AppEvent;
```

公共规则：

- `Root` 处理 `EVT_FAULT`，任何状态都能进入 `Fault`；
- `Fault` 处理 `EVT_RESET`，恢复到 `Idle`；
- `Auto` 处理 `EVT_STOP`，所有自动子状态都能停止；
- `AutoPrepare`、`AutoRun`、`AutoFinish` 只处理本阶段事件

这样可以避免每个子状态都重复写故障和停止逻辑

# infra SDK 基本说明

> `sdks/infra/` 提供与业务和具体芯片无关的基础设施模块

---

## 1. 模块定位

infra 模块用于沉淀通用能力，例如控制器、数学工具、协议解析器、状态机和时间工具；它们应尽量能在 PC/mock 环境下单独编译和测试

---

## 2. 当前模块

| 模块 | 文件 | 说明 |
|---|---|---|
| delay | `delay.h` / `delay.c` | 通用延时或时间辅助接口 |
| matrix | `matrix.h` / `matrix.c` | 矩阵运算工具 |
| PID | `pid.h` / `pid.c` | 多实例 PID 控制器 |
| protocol parser | `protocol_parser.h` / `protocol_parser.c` | 通信协议解析辅助 |
| HFSM | `hfsm/` | 层级状态机基础框架 |
| log | `log.h` / `log.c` | 可替换输出端口的轻量日志接口 |

---

## 3. 设计约束

- public header 不包含 HAL/FSP/CubeMX/CMSIS 头文件
- 不保存平台全局句柄，例如 `hfdcan1`、`huart1`
- 多实例工具优先使用 `Handle* self` 或上下文结构
- 二值配置、状态和查询接口统一使用 `bool` / `true` / `false`，例如 `ring_buf_create(..., bool overwrite)`、`is_full()` 和 `is_empty()`
- 需要时间、锁、输出流等底层能力时，通过 PortOps 接入
- 错误码不直接泄漏平台错误类型
- 能 PC/mock 测试的模块应保留最小测试入口

---

## 4. 推荐接口形态

```c
typedef struct {
    uint32_t (*now_ms)(void);
} InfraPortOps;

typedef struct {
    const InfraPortOps* ops;
    uint32_t timeout_ms;
} InfraConfig;

InfraStatus infra_init(Infra* self, const InfraConfig* config);
InfraStatus infra_update(Infra* self);
```

纯算法工具不需要 PortOps 时，应保持简单函数或多实例对象，不为统一风格强行增加抽象

---

## 5. 后续补充方向

- CRC
- limiter
- low pass filter
- moving average
- error code table helper
- unit test scaffold

## 6. 与 Atlas 协议相关的当前约定

- `binary_frame.h` 维护 Atlas 二进制协议常量
- `PI_CONTROL` 当前固定为 `38 bytes`
- `PI_CONTROL` 中 `control_mask` 与 `arm_mode` 为正交字段
- Pi 机械臂目标共用 `PI_CONTROL(0x31)`，不新增独立目标消息类型
- 机械臂单次消费语义只适用于 `AutoPi` 模式下的 `Pi -> MCU` 机械臂目标

# bus_motor SDK 接口文档

> `sdks/device/bus_motor/` 提供电机设备层统一入口和具体电机实例实现

---

## 1. 模块定位

`bus_motor.*` 是电机统一接口入口，供 service/app 上层调用

`dm_bus_motor.*` 是达妙电机实例，实现达妙协议并提供 `dm_bus_motor_instance`

推荐使用方式：

```text
service init
→ 组装 BusMotorPortOps
→ bus_motor_set_instance(&dm_bus_motor_instance)
→ bus_motor_init(&config)
→ app/service 统一调用 bus_motor.xxx 或 bus_motor_xxx
```

---

## 2. 文件结构

```text
sdks/device/bus_motor/
├── bus_motor.h       # 通用电机接口、状态码、反馈结构、PortOps、入口单例
├── bus_motor.c       # 入口单例转发实现
├── dm_bus_motor.h    # 达妙电机实例声明和协议常量
└── dm_bus_motor.c    # 达妙电机协议实现
```

---

## 3. 统一接口

### 3.1 入口单例

```c
#define bus_motor (*bus_motor_instance)

extern const BusMotorInterface* bus_motor_instance;

BusMotorStatus bus_motor_set_instance(const BusMotorInterface* instance);
```

上层不直接 include 某个具体电机实现，只绑定实例后调用统一入口

```c
bus_motor_set_instance(&dm_bus_motor_instance);
bus_motor.init(&config);
bus_motor.set_spd(1, 3.0f);
```

### 3.2 状态码

```c
typedef enum {
    MOTOR_STATUS_OK = 0,
    MOTOR_STATUS_ERROR,
    MOTOR_STATUS_INVALID_PARAM,
    MOTOR_STATUS_PORT_ERROR,
    MOTOR_STATUS_TIMEOUT,
    MOTOR_STATUS_ID_MISMATCH,
    MOTOR_STATUS_NO_INSTANCE,
    MOTOR_STATUS_NOT_INITIALIZE,
} BusMotorStatus;
```

`MOTOR_STATUS_NOT_INITIALIZE` 表示已经有电机实例，但实例尚未完成 `bus_motor_init()`

### 3.3 PortOps

```c
typedef struct {
    bool (*send)(uint32_t id, const uint8_t* data, uint8_t len);
    bool (*read)(uint32_t* id, uint8_t* data, uint8_t* len);
    uint32_t (*now_ms)(void);
    void (*delay_ms)(uint32_t ms);
} BusMotorPortOps;
```

PortOps 由 service 绑定 platform 或 adapter，电机 SDK 不直接依赖 HAL/FSP/CubeMX

---

## 4. 初始化示例

```c
#include "bus_motor.h"
#include "dm_bus_motor.h"
#include "platform_can.h"
#include "platform_time.h"

static const BusMotorPortOps bus_motor_ops = {
    .send = platform_can_send,
    .read = platform_can_read,
    .now_ms = platform_time_now_ms,
    .delay_ms = platform_time_delay_ms,
};

void chassis_bus_motor_init(void)
{
    BusMotorConfig config = {
        .ops = &bus_motor_ops,
        .feedback_timeout_ms = 50u,
    };

    bus_motor_set_instance(&dm_bus_motor_instance);
    bus_motor_init(&config);
}
```

---

## 5. 调用示例

```c
void chassis_update(void)
{
    BusMotorFeedback feedback;

    bus_motor.set_spd(1, 2.0f);

    if(bus_motor.get_feedback(1, &feedback) == MOTOR_STATUS_OK) {
        /* 使用 feedback.pos_rad / feedback.spd_rad_s / feedback.torque_a */
    }
}
```

---

## 6. 设计约束

- 上层只依赖 `bus_motor.h`
- 具体电机实例只负责协议实现
- 具体实例内部的初始化、在线等二值状态使用 `bool` / `true` / `false`，不再用 `uint8_t` 或 `0/1` 表示
- `dm_bus_motor.*` 不直接 include platform 头文件
- platform 对接统一放在 service init 或 adapter 中
- 新增其他电机时，只需要新增实例文件并提供 `BusMotorInterface`

# bus_motor SDK 接口说明

`src/device/bus_motor/` 提供总线电机设备层统一接口，以及具体电机驱动实例实现

---

## 1. 模块定位

`bus_motor.*` 提供统一抽象接口，供 service / app 上层调用

`dji_motor.*` 当前支持以下 DJI 总线电机组合：

- `M3508 + C620`
- `M2006 + C610`

这两种型号共用同一套 CAN 协议框架，但内部机械减速比和控制器原始电流指令范围不同，因此驱动内部会按型号参数表进行换算与限幅

---

## 2. 通用配置透传

`BusMotorConfig` 提供一个可选的 `driver_config` 指针，用于把具体驱动私有配置透传给实例层：

```c
typedef struct {
    const BusMotorPortOps* ops;
    uint32_t timeout_ms;
    uint8_t retry_count;
    const void* driver_config;
} BusMotorConfig;
```

通用 `bus_motor` 层不会解析 `driver_config`，只负责原样传给具体驱动

---

## 3. DJI 型号配置

DJI 驱动使用 `DjiMotorConfig` 指定型号：

```c
typedef enum {
    DJI_MOTOR_MODEL_M3508 = 0u,
    DJI_MOTOR_MODEL_M2006,
    DJI_MOTOR_MODEL_COUNT
} DjiMotorModel;

typedef struct {
    DjiMotorModel model;
} DjiMotorConfig;
```

长期推荐由上层显式传入 `DjiMotorConfig`；当前工程为了兼容未改动的 service 初始化代码，在 `driver_config == NULL` 时会默认选择 `DJI_MOTOR_MODEL_M2006`

---

## 4. 速度与位置语义

对 DJI 驱动而言：

- `set_spd()` 的输入语义是减速箱输出轴目标角速度，单位 `rad/s`
- `get_spd()` 的输出语义是减速箱输出轴当前角速度，单位 `rad/s`
- `get_pos()` 的输出语义是减速箱输出轴累计角度，单位 `rad`

注意：

- CAN 反馈中的 `rpm` 是转子侧转速
- 驱动内部会根据当前型号减速比，把转子侧 RPM / 角度换算成输出轴速度 / 位置

---

## 5. 型号配置示例

`M2006 + C610`：

```c
static const DjiMotorConfig dji_config = {
    .model = DJI_MOTOR_MODEL_M2006,
};

BusMotorConfig config = {
    .ops = &motor_ops,
    .timeout_ms = 0u,
    .retry_count = 0u,
    .driver_config = &dji_config,
};
```

`M3508 + C620`：

```c
static const DjiMotorConfig dji_config = {
    .model = DJI_MOTOR_MODEL_M3508,
};
```

---

## 6. 当前支持的型号参数

DJI 驱动内部参数表当前包含：

- `M3508`：减速比 `3591 / 187`，原始电流指令范围 `-16384 ~ +16384`
- `M2006`：减速比 `36`，原始电流指令范围 `-10000 ~ +10000`

PID 输出会先按当前型号的原始电流指令范围限幅，再编码进 `0x200` 控制帧；该控制帧维持现有协议格式不变，每个电机占用 2 字节有符号控制值

---

## 7. 初始化建议

推荐初始化流程：

```text
service init
-> 组装 BusMotorPortOps
-> bus_motor_set_instance(&dji_motor_instance)
-> bus_motor_init(&config)
-> app/service 统一调用 bus_motor.xxx
```

如果上层暂未传入 `driver_config`，当前工程仍会以 `M2006` 兼容默认值运行；长期建议显式配置，避免项目后续切换硬件时产生隐式行为

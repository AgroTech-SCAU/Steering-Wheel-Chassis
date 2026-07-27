# servo SDK

`src/device/servo/` 提供舵机通用入口和 FEETECH SCS 协议驱动

## 文件结构

```text
servo/
├── README.md
├── servo.h
├── servo.c
├── ft_scs_servo.h
└── ft_scs_servo.c
```

## 设计原则

`servo.*` 只放所有舵机都应具备的通用能力

- `init`
- `status_str`
- `set_speed`
- `set_pos_spd`
- `set_pos_spd_tor`
- `get_position`
- `get_speed`
- `get_torque`
- `update_feedback`

通用入口统一使用弧度制

- `position` 单位为 `rad`
- `speed` 单位为 `rad/s`
- `torque` 统一使用 `N*m`

如果某类舵机缺少某个通用能力, 驱动应直接返回 `SERVO_STATUS_UNSUPPORTED`

协议专属能力不放进 `servo.*`, 例如 SCS 的 `ping`, `action`, `write_u8`, `read_u16`, 原始指令包发送等, 都放在 `ft_scs_servo` 特色入口里

## 通用入口

```c
#define servo (*servo_instance)
```

可使用两种调用方式

```c
servo.set_speed(1u, 1.0f);
servo_set_speed(1u, 1.0f);
```

### BusServoFeedback

```c
typedef struct {
    uint8_t id;
    uint8_t error_code;
    float position;
    float speed;
    float torque;
} BusServoFeedback;
```

`update_feedback()` 负责主动读取舵机反馈, 解析应答帧, 并更新驱动内部缓存

`get_position()`, `get_speed()`, `get_torque()` 不会访问总线, 只返回最近一次成功解析后的缓存值

## PortOps

```c
typedef struct {
    bool (*write)(const uint8_t* data, uint16_t len);
    int (*read)(uint8_t* data, uint16_t len);
    uint32_t(*now_ms)(void);
    void (*delay_ms)(uint32_t ms);
} BusServoPortOps;
```

约定

- `write()` 成功返回 `true`, 失败返回 `false`
- `read()` 返回实际读到的字节数, 可以小于请求长度
- `now_ms()` 必须提供, 用于应答超时
- `delay_ms()` 可选, 用于半双工或总线方向切换后的短等待

## 初始化

```c
#include "servo.h"
#include "ft_scs_servo.h"

static bool servo_bus_write(const uint8_t* data, uint16_t len);
static int servo_bus_read(uint8_t* data, uint16_t len);
static uint32_t board_now_ms(void);
static void board_delay_ms(uint32_t ms);
static void servo_bus_flush_rx(void);

static const BusServoPortOps servo_ops = {
    .write = servo_bus_write,
    .read = servo_bus_read,
    .now_ms = board_now_ms,
    .delay_ms = board_delay_ms,
    .flush_rx = servo_bus_flush_rx,
};

void device_servo_init(void) {
    FtScsServoConfig config = {
        .ops = &servo_ops,
        .timeout_ms = 100u,
        .retry_count = 0u,
        .endian = SERVO_ENDIAN_LITTLE,
    };

    bus_servo_set_instance(&ft_scs_servo_common_instance);
    bus_servo_init(&config);
}
```

磁编码 SCS 舵机通常使用小端寄存器, 电位器类型舵机可能使用大端寄存器, 以具体型号内存表为准

## 通用使用

```c
BusServoFeedback feedback;

servo.set_speed(1u, 1.5f);
servo.set_pos_spd(1u, 3.14f, 1.0f);
servo.set_pos_spd_tor(1u, 1.57f, 0.8f, 0.5f);

if(servo.update_feedback(1u, &feedback) == SERVO_STATUS_OK) {
    float pos = servo.get_position(1u);
    float speed = servo.get_speed(1u);
    float torque = servo.get_torque(1u);
    (void)pos;
    (void)speed;
    (void)torque;
}
```

## SCS 特色入口

```c
#include "ft_scs_servo.h"

uint8_t found_id;

ft_scs_servo.ping(1u, &found_id);
ft_scs_servo.enable_torque(1u);
ft_scs_servo.write_u8(1u, FT_SCS_SERVO_LOCK, 0u);
ft_scs_servo.action(FT_SCS_SERVO_BROADCAST_ID);
```

`ft_scs_servo_common_instance` 提供通用 `BusServoInterface`, `ft_scs_servo_instance` 提供 SCS 协议特色接口

## FEETECH SCS 映射

当前驱动按 SCS 常用控制表地址实现

- `set_speed()` 写 `MODE = 1`, 然后写 `ACC` 到 `GOAL_SPEED_H`
- `set_pos_spd()` 写 `MODE = 0`, 然后写 `ACC` 到 `GOAL_SPEED_H`
- `set_pos_spd_tor()` 先写 `TORQUE_LIMIT`, 再执行 `set_pos_spd()`
- `update_feedback()` 读取 `PRESENT_POSITION_L` 到 `PRESENT_LOAD_H`

位置原始值按一圈 `4096` count 与 `2*pi` rad 互转

速度原始值按 `4096` count/s 与 `2*pi` rad/s 互转

力矩或负载按 SCS 原始 `1000` 满量程映射到 STS3215 约 `2.94 N*m`

## 安全建议

- 首次上电先确认供电, 共地, TX/RX 或 RS485 方向控制, 波特率和 ID
- 第一次运动先使用小速度和小范围
- 机械结构联调前在 service/app 层加入角度, 速度, 负载和超时保护
- 阻塞式读写接口不建议在中断中调用

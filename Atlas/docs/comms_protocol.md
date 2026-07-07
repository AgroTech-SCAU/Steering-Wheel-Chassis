# MCU / PC / Pi 通信协议说明

> 文档定位：本文件是 `PC <-> MCU` 与 `Pi <-> MCU` 的统一协议及行为约束
>

---

## 1. 协议目标

本协议用于统一 `PC <-> MCU` 与 `Pi <-> MCU` 的通信格式和控制边界，目标如下：

1. 所有串口通信统一使用同一套二进制帧格式
2. `pc_comms` 与 `pi_comms` 只负责通信、校验、解析、缓存与发送，不直接控制底盘、机械臂或状态机
3. MCU 是系统状态机、模式切换和全自动任务启动权的唯一拥有者
4. Pi 不得主动请求 MCU 进入 `AutoPi`；Pi 通过 `MCU_STATUS` 被动获知状态和任务锁存标志
5. 高频连续控制量、一次性动作和跨状态锁存量必须明确区分
6. 通信错误原则上只丢帧、统计和限频告警，不直接进入 `Fault`
7. 所有消息必须明确方向、payload 长度、字节偏移、数据类型、单位、消费语义和权限边界

---

## 2. 总体职责与通信关系

### 2.1 PC -> MCU

PC 负责：

1. 发送 PC 心跳
2. 发送主臂五个关节角和末端开关状态

### 2.2 Pi -> MCU

Pi 负责：

1. 在 `AutoPi` 下发送高频底盘控制
2. 发送机械臂目标和一次性机械臂动作
3. 发送一次性 yaw 动作
4. 上报任务完成或任务失败
5. 发送急停事件
6. 对 MCU 需要 ACK 的一次性事件发送 ACK
7. 持续发送 Pi 心跳

### 2.3 MCU -> Pi

MCU 负责：

1. 周期发送 `MCU_STATUS`
2. 周期发送 IMU、里程计和机械臂状态
3. 在 `MCU_STATUS` 中发送当前状态机状态和 `auto_start_latched`
4. 发送需要 ACK 的一次性事件
5. 发送 ACK
6. 预留故障即时事件

### 2.4 全自动任务启动权

当前阶段仅考虑遥控器启动：

```text
遥控器产生有效自动启动边沿
        ↓
MCU 检查状态、Pi 在线、底盘和里程计 ready
        ↓
MCU 将 auto_start_latched 从 0 置为 1
        ↓
MCU 执行 Idle -> AutoPi
        ↓
MCU 通过 MCU_STATUS 持续通知 Pi
        ↓
Pi 检测到 auto_start_latched=1 且 app_state=AutoPi
        ↓
Pi 只产生一次 START 事件
```

下一轮任务不能通过释放自动挡直接解锁，必须执行遥控器清理手势：

```text
SWC 置高 + VRA 置低 + VRB 置低
        ↓
MCU 将 auto_start_latched 从 1 清为 0
        ↓
MCU 清理自动任务控制上下文
        ↓
Pi 检测到 1 -> 0
        ↓
Pi 清理本地自动任务上下文并产生一次 RESET 事件
```

---

## 3. 通用二进制帧格式

统一帧格式：

```text
SOF0     1 byte   0xA5
SOF1     1 byte   0x5A
LEN_H    1 byte   body length high byte
LEN_L    1 byte   body length low byte
VER      1 byte   protocol version
MSG_ID   1 byte   message id
SEQ      1 byte   sequence number
FLAGS    1 byte   flags
PAYLOAD  N byte   payload
CRC_H    1 byte   CRC16 high byte
CRC_L    1 byte   CRC16 low byte
```

| 帧偏移 | 长度 | 字段 | 说明 |
|---:|---:|---|---|
| 0 | 1 | `SOF0` | 固定 `0xA5` |
| 1 | 1 | `SOF1` | 固定 `0x5A` |
| 2 | 1 | `LEN_H` | body 长度高字节 |
| 3 | 1 | `LEN_L` | body 长度低字节 |
| 4 | 1 | `VER` | 当前固定 `0x01` |
| 5 | 1 | `MSG_ID` | 消息 ID |
| 6 | 1 | `SEQ` | 帧序号 |
| 7 | 1 | `FLAGS` | 标志位 |
| 8 | N | `PAYLOAD` | 业务数据 |
| 8 + N | 1 | `CRC_H` | CRC16 高字节 |
| 9 + N | 1 | `CRC_L` | CRC16 低字节 |

约定：

1. `LEN` 表示 `VER + MSG_ID + SEQ + FLAGS + PAYLOAD` 的总长度
2. `body_len = 4 + payload_len`
3. `frame_len = 10 + payload_len`
4. `LEN` 使用大端格式
5. `CRC` 使用大端格式
6. payload 内多字节字段使用小端格式
7. `SEQ` 为 `uint8_t`，按 `0 -> 255 -> 0` 循环
8. `FLAGS bit0 = NEED_ACK`，其余位保留
9. 高频周期帧和高频控制帧不使用 ACK

### 3.1 CRC 参数

CRC 必须使用以下完整参数：

```text
Name:       CRC-16/CCITT-FALSE
Polynomial: 0x1021
Init:       0xFFFF
RefIn:      false
RefOut:     false
XorOut:     0x0000
Check:      0x29B1 for ASCII "123456789"
```

CRC 覆盖范围：

```text
SOF0 + SOF1 + LEN_H + LEN_L + VER + MSG_ID + SEQ + FLAGS + PAYLOAD
```

CRC 不覆盖末尾的 `CRC_H / CRC_L` 自身

---

## 4. 字节序、类型与单位

### 4.1 字节序

1. 帧级 `LEN` 和 `CRC` 使用大端
2. payload 内 `uint16_t / int16_t / uint32_t / int32_t` 使用小端
3. 不直接传输 IEEE 754 `float`
4. 布尔字段在线协议中使用 `uint8_t`，合法值为 `0` 或 `1`

### 4.2 单位

| 物理量 | 常用类型 | 线单位 | 换算 |
|---|---|---|---|
| 时间戳 | `uint32_t` | `ms` | MCU/Pi/PC 各自单调时钟，不是 Unix 时间 |
| 角度 | `int32_t` | `urad` | `rad = urad × 1e-6` |
| 角速度 | `int32_t` | `urad/s` | `rad/s = urad/s × 1e-6` |
| PI_CONTROL 底盘角速度 | `int16_t` | `mrad/s` | `rad/s = mrad/s × 1e-3` |
| PI_CONTROL 线速度 | `int16_t` | `mm/s` | `m/s = mm/s × 1e-3` |
| ODOM 线速度 | `int32_t` | `mm/s` | `m/s = mm/s × 1e-3` |
| 位置 | `int32_t` | `mm` | `m = mm × 1e-3` |
| 加速度 | `int32_t` | `mm/s²` | `m/s² = mm/s² × 1e-3` |
| 四元数 | `int16_t` | Q15 | `q = q15 / 32767.0` |

---

## 5. 消息 ID 总表

### 5.1 PC -> MCU

| MSG_ID | 名称 | payload 长度 | 语义 |
|---:|---|---:|---|
| `0x10` | `PC_HEARTBEAT` | 0 | PC 心跳 |
| `0x11` | `PC_MASTER_JOINTS` | 25 | 主臂关节角和末端开关 |

### 5.2 MCU -> Pi

| MSG_ID | 名称 | payload 长度 | 语义 |
|---:|---|---:|---|
| `0x21` | `MCU_STATUS` | 16 | MCU 状态、故障和自动任务锁存标志 |
| `0x22` | `MCU_START_SENSOR_EVENT` | 8 | 启动传感器事件，当前阶段保留 |
| `0x23` | `MCU_ACK` | 4 | MCU ACK |
| `0x24` | `MCU_FAULT_EVENT` | 8 | 故障即时事件，当前预留 |
| `0x25` | `MCU_IMU` | 48 | IMU 周期帧，100 Hz |
| `0x26` | `MCU_ODOM` | 32 | 局部里程计周期帧，50 Hz |
| `0x27` | `MCU_ARM_STATE` | 48 | 机械臂状态周期帧，50 Hz |

### 5.3 Pi -> MCU

| MSG_ID | 名称 | payload 长度 | 语义 |
|---:|---|---:|---|
| `0x30` | `PI_HEARTBEAT` | 0 | Pi 心跳 |
| `0x31` | `PI_CONTROL` | 38 | 底盘连续控制和机械臂目标 |
| `0x40` | `PI_ARM_ACTION` | 8 | 一次性机械臂动作 |
| `0x41` | `PI_YAW_ACTION` | 12 | 一次性 yaw 动作 |
| `0x42` | `PI_MISSION_EVENT` | 8 | 任务完成或失败 |
| `0x43` | `PI_ESTOP` | 8 | 急停事件 |
| `0x44` | `PI_ACK` | 4 | Pi ACK |

Pi 不具备切换 MCU 模式的线协议权限

---

## 6. PC -> MCU 消息

### 6.1 PC_HEARTBEAT

```text
Direction: PC -> MCU
MSG_ID:    0x10
Payload:   empty
```

语义：

1. 更新 PC 在线状态
2. 更新最近一次有效接收时间
3. 不修改主臂关节角缓存

推荐频率：

```text
1 Hz
```

当前 PC 工具默认：

```text
MCU baudrate: 921600
Heartbeat:    1 Hz
```

### 6.2 PC_MASTER_JOINTS

```text
Direction:      PC -> MCU
MSG_ID:         0x11
Payload length: 25 bytes
```

| payload 偏移 | 长度 | 类型 | 字段 | 单位 |
|---:|---:|---|---|---|
| 0 | 4 | `uint32_t` | `stamp_ms` | `ms` |
| 4 | 4 | `int32_t` | `q0_urad` | `urad` |
| 8 | 4 | `int32_t` | `q1_urad` | `urad` |
| 12 | 4 | `int32_t` | `q2_urad` | `urad` |
| 16 | 4 | `int32_t` | `q3_urad` | `urad` |
| 20 | 4 | `int32_t` | `q4_urad` | `urad` |
| 24 | 1 | `uint8_t` | `end_switch` | `0` 未触发，`1` 触发 |

语义：

1. `pc_comms` 只解析并缓存
2. MCU 将五个 `urad` 转换为 `rad`
3. `end_switch` 来自主臂末端设备
4. 该消息使用连续值的 `get + fresh timeout` 语义

权限约束：

1. 仅允许在 `Manual + ManualChassisPcArm` 下被 app 层消费
2. 离开 `ManualChassisPcArm` 时必须清除主臂缓存
3. PC 离线不得影响不依赖 PC 的其他手动模式或 `Idle`

推荐频率：

```text
30 Hz ~ 100 Hz
当前 PC 工具默认 50 Hz
```

---

## 7. Pi -> MCU 消息

### 7.1 PI_HEARTBEAT

```text
Direction: Pi -> MCU
MSG_ID:    0x30
Payload:   empty
```

语义：

1. 更新 Pi 在线状态
2. 更新最近一次有效接收时间
3. 不修改普通控制缓存
4. Pi 自动任务上下文复位后仍必须继续发送心跳

推荐频率：

```text
1 Hz
```

MCU 当前在线超时：

```text
3000 ms
```

### 7.2 PI_CONTROL

```text
Direction:      Pi -> MCU
MSG_ID:         0x31
Payload length: 38 bytes
```

该帧同时承载：

1. 底盘连续控制
2. 机械臂离散目标命令

二者消费语义不同：

```text
chassis 部分：get + fresh timeout
arm 部分：command_seq 去重 + take/consume
```

#### 7.2.1 固定头部和底盘字段

| payload 偏移 | 长度 | 类型 | 字段 | 单位 / 说明 |
|---:|---:|---|---|---|
| 0 | 4 | `uint32_t` | `stamp_ms` | Pi 单调时钟 `ms` |
| 4 | 1 | `uint8_t` | `control_mask` | 有效位 |
| 5 | 1 | `uint8_t` | `arm_mode` | 机械臂目标模式 |
| 6 | 2 | `uint16_t` | `command_seq` | 机械臂命令去重序号 |
| 8 | 2 | `int16_t` | `vx_mm_s` | `mm/s` |
| 10 | 2 | `int16_t` | `vy_mm_s` | `mm/s` |
| 12 | 2 | `int16_t` | `wz_mrad_s` | `mrad/s` |
| 14 | 20 | union | `arm_target[5]` | 由 `arm_mode` 决定 |
| 34 | 2 | `uint16_t` | `arm_speed_mrad_s` | `mrad/s` |
| 36 | 2 | `uint16_t` | `reserved2` | 必须填 0 |

`control_mask`：

| bit | 名称 | 说明 |
|---:|---|---|
| bit0 | `chassis_valid` | 本帧更新底盘控制缓存 |
| bit1 | `arm_valid` | 本帧包含机械臂目标 |
| bit2 | reserved | 必须为 0 |
| bit3 | `brake_request` | 请求底盘刹车 |
| bit4 ~ bit7 | reserved | 必须为 0 |

注意：

1. `arm_valid` 不只表示关节角，也可表示末端 Pose、位置或姿态目标
2. 某个 valid bit 为 0，不表示清除该部分旧缓存
3. 旧缓存是否失效由 fresh timeout 或状态切换清理决定

#### 7.2.2 arm_mode

| 值 | 名称 | 说明 |
|---:|---|---|
| 0 | `NONE` | 本帧不发送机械臂目标 |
| 1 | `JOINTS` | 五关节目标 |
| 2 | `POSE_5D` | `x/y/z/pitch/yaw` |
| 3 | `POSITION` | `x/y/z` |
| 4 | `ORIENTATION_2D` | `pitch/yaw` |

#### 7.2.3 JOINTS 布局

`arm_mode = 1`

| 偏移 | 类型 | 字段 | 单位 |
|---:|---|---|---|
| 14 | `int32_t` | `q0_urad` | `urad` |
| 18 | `int32_t` | `q1_urad` | `urad` |
| 22 | `int32_t` | `q2_urad` | `urad` |
| 26 | `int32_t` | `q3_urad` | `urad` |
| 30 | `int32_t` | `q4_urad` | `urad` |

#### 7.2.4 POSE_5D 布局

`arm_mode = 2`

| 偏移 | 类型 | 字段 | 单位 |
|---:|---|---|---|
| 14 | `int32_t` | `x_mm` | `mm` |
| 18 | `int32_t` | `y_mm` | `mm` |
| 22 | `int32_t` | `z_mm` | `mm` |
| 26 | `int32_t` | `pitch_urad` | `urad` |
| 30 | `int32_t` | `yaw_urad` | `urad` |

#### 7.2.5 POSITION 布局

`arm_mode = 3`

| 偏移 | 类型 | 字段 | 单位 / 说明 |
|---:|---|---|---|
| 14 | `int32_t` | `x_mm` | `mm` |
| 18 | `int32_t` | `y_mm` | `mm` |
| 22 | `int32_t` | `z_mm` | `mm` |
| 26 | `int32_t` | reserved | 必须填 0 |
| 30 | `int32_t` | reserved | 必须填 0 |

#### 7.2.6 ORIENTATION_2D 布局

`arm_mode = 4`

| 偏移 | 类型 | 字段 | 单位 / 说明 |
|---:|---|---|---|
| 14 | `int32_t` | `pitch_urad` | `urad` |
| 18 | `int32_t` | `yaw_urad` | `urad` |
| 22 | `int32_t` | reserved | 必须填 0 |
| 26 | `int32_t` | reserved | 必须填 0 |
| 30 | `int32_t` | reserved | 必须填 0 |

#### 7.2.7 command_seq 语义

1. 每个新的机械臂目标分配一个 `uint16_t command_seq`
2. 同一目标的重发帧必须使用相同 `command_seq`
3. MCU 对相同 `command_seq` 只允许消费一次
4. `command_seq` 自然回绕
5. `command_seq` 只用于机械臂目标去重，不用于底盘连续控制
6. 当前 Pi bridge 建议同一机械臂目标重复发送 3 帧，提高串口瞬态丢帧下的到达概率

#### 7.2.8 权限和超时

1. `PI_CONTROL` 仅允许在 `AutoPi` 下执行
2. 底盘控制 fresh timeout 当前为 `200 ms`
3. 机械臂控制 fresh timeout 当前为 `200 ms`
4. 无新鲜底盘命令时，MCU 必须停止或刹车
5. 离开 `AutoPi`、执行遥控器清理手势或 `auto_start_latched` 复位时，必须清除普通 Pi 控制缓存
6. 清普通控制缓存不得清除 pending EStop

推荐发送频率：

```text
20 Hz ~ 50 Hz
当前 Pi bridge 默认 50 Hz
```

### 7.3 PI_ARM_ACTION

```text
Direction:      Pi -> MCU
MSG_ID:         0x40
Payload length: 8 bytes
Consumption:    take/consume
```

| 偏移 | 长度 | 类型 | 字段 | 说明 |
|---:|---:|---|---|---|
| 0 | 4 | `uint32_t` | `stamp_ms` | 时间戳 |
| 4 | 1 | `uint8_t` | `action` | 动作类型 |
| 5 | 1 | `uint8_t` | reserved | 必须填 0 |
| 6 | 2 | `uint16_t` | `sequence_id` | 序列动作编号 |

| action | 名称 |
|---:|---|
| 1 | `ARM_ENABLE` |
| 2 | `ARM_STOP` |
| 3 | `ARM_SEQUENCE` |

合法动作被缓存为 pending action，app 层读取后立即清除

### 7.4 PI_YAW_ACTION

```text
Direction:      Pi -> MCU
MSG_ID:         0x41
Payload length: 12 bytes
Consumption:    take/consume
```

| 偏移 | 长度 | 类型 | 字段 | 单位 / 说明 |
|---:|---:|---|---|---|
| 0 | 4 | `uint32_t` | `stamp_ms` | `ms` |
| 4 | 1 | `uint8_t` | `action` | 动作类型 |
| 5 | 3 | `uint8_t[3]` | reserved | 必须填 0 |
| 8 | 4 | `int32_t` | `target_yaw_urad` | `urad` |

| action | 名称 |
|---:|---|
| 1 | `HOLD_ENABLE` |
| 2 | `HOLD_DISABLE` |
| 3 | `TARGET_SET` |

连续旋转速度仍通过 `PI_CONTROL.wz_mrad_s` 表达

### 7.5 PI_MISSION_EVENT

```text
Direction:      Pi -> MCU
MSG_ID:         0x42
Payload length: 8 bytes
Consumption:    take/consume
```

| 偏移 | 长度 | 类型 | 字段 | 说明 |
|---:|---:|---|---|---|
| 0 | 4 | `uint32_t` | `stamp_ms` | 时间戳 |
| 4 | 1 | `uint8_t` | `event` | 任务事件 |
| 5 | 1 | `uint8_t` | reserved | 必须填 0 |
| 6 | 2 | `int16_t` | `code` | 失败码，DONE 时填 0 |

| event | 名称 | MCU 行为 |
|---:|---|---|
| 1 | `DONE` | `AutoPi -> Finished` |
| 2 | `FAIL` | 进入 recoverable `Fault` |

关键约束：

1. `DONE` 不清除 `auto_start_latched`
2. `FAIL` 不清除 `auto_start_latched`
3. 任务结束后仍需遥控器清理手势，才能允许下一轮启动
4. 非 `AutoPi` 状态下收到任务事件，应忽略并统计或限频告警

### 7.6 PI_ESTOP

```text
Direction:      Pi -> MCU
MSG_ID:         0x43
Payload length: 8 bytes
Consumption:    global take/consume
```

| 偏移 | 长度 | 类型 | 字段 | 说明 |
|---:|---:|---|---|---|
| 0 | 4 | `uint32_t` | `stamp_ms` | 时间戳 |
| 4 | 1 | `uint8_t` | `reason` | 急停原因 |
| 5 | 3 | `uint8_t[3]` | reserved | 必须填 0 |

约束：

1. 任意状态全局生效
2. 收到后 MCU 必须进入 `EStop`
3. 普通清故障不能解除 `EStop`
4. `pi_comms_clear_controls()` 不得清除 pending EStop
5. 遥控器清理手势可将 `auto_start_latched` 清为 0，但 MCU 必须继续保持 `EStop`

### 7.7 PI_ACK

```text
Direction:      Pi -> MCU
MSG_ID:         0x44
Payload length: 4 bytes
```

| 偏移 | 长度 | 类型 | 字段 | 说明 |
|---:|---:|---|---|---|
| 0 | 1 | `uint8_t` | `ack_msg_id` | 被确认的消息 ID |
| 1 | 1 | `uint8_t` | `ack_seq` | 被确认帧 SEQ |
| 2 | 2 | `uint16_t` | `code` | 结果码，0 表示成功 |

ACK 不匹配只丢弃、统计和限频告警，不进入 `Fault`

---

## 8. MCU -> Pi 消息

### 8.1 MCU_STATUS

```text
Direction:      MCU -> Pi
MSG_ID:         0x21
Payload length: 16 bytes
Recommended:    10 Hz
ACK:            no
```

#### 8.1.1 payload 布局

| 偏移 | 长度 | 类型 | 字段 | 说明 |
|---:|---:|---|---|---|
| 0 | 4 | `uint32_t` | `stamp_ms` | MCU 单调时钟 |
| 4 | 1 | `uint8_t` | `app_state` | 状态机状态 |
| 5 | 1 | `uint8_t` | `manual_mode` | Manual 子模式 |
| 6 | 1 | `uint8_t` | `ready_flags` | ready 位图 |
| 7 | 1 | `uint8_t` | `online_flags` | 在线和故障位图 |
| 8 | 1 | `uint8_t` | `fault_source` | 故障来源 |
| 9 | 1 | `uint8_t` | `fault_level` | 故障等级 |
| 10 | 2 | `int16_t` | `fault_code` | 故障码 |
| 12 | 1 | `uint8_t` | `auto_start_latched` | 全自动任务启动锁存，合法值 0/1 |
| 13 | 3 | `uint8_t[3]` | reserved | 必须填 0 |

完整帧长度：

```text
26 bytes
```

#### 8.1.2 app_state

| 值 | 状态 |
|---:|---|
| 0 | `IDLE` |
| 1 | `MANUAL` |
| 2 | `AUTO_PI` |
| 3 | `FAULT` |
| 4 | `ESTOP` |
| 5 | `FINISHED` |

#### 8.1.3 manual_mode

| 值 | 模式 |
|---:|---|
| 0 | `CHASSIS_PC_ARM` |
| 1 | `ARM_FS` |

`manual_mode` 在非 Manual 状态下仅表示最近一次配置，不代表该模式正在执行

#### 8.1.4 ready_flags

| bit | 名称 |
|---:|---|
| bit0 | `chassis_ready` |
| bit1 | `arm_ready` |
| bit2 | `odom_ready` |
| bit3 | `remote_ready` |
| bit4 | `pc_ready` |
| bit5 | `pi_ready` |
| bit6 ~ bit7 | reserved |

#### 8.1.5 online_flags

| bit | 名称 |
|---:|---|
| bit0 | `remote_online` |
| bit1 | `pc_online` |
| bit2 | `pi_online` |
| bit3 | `has_fault` |
| bit4 | `estop_active` |
| bit5 ~ bit7 | reserved |

#### 8.1.6 fault_source

| 值 | 名称 |
|---:|---|
| 0 | `NONE` |
| 1 | `CHASSIS` |
| 2 | `ODOM` |
| 3 | `ARM` |
| 4 | `REMOTE` |
| 5 | `PC_LINK` |
| 6 | `PI_LINK` |
| 7 | `PI_MISSION` |
| 8 | `COMMAND` |
| 9 | `SYSTEM` |

#### 8.1.7 fault_level

| 值 | 名称 |
|---:|---|
| 0 | `NONE` |
| 1 | `WARN` |
| 2 | `DEGRADE` |
| 3 | `RECOVERABLE` |
| 4 | `FATAL` |

#### 8.1.8 auto_start_latched

| 值 | 含义 |
|---:|---|
| 0 | 当前没有已锁存的启动事件，允许准备下一轮任务 |
| 1 | MCU 已接受过一轮全自动启动，清理手势前禁止再次启动 |

要求：

1. MCU 上电初始化为 `0`
2. 只有启动前置条件全部通过，且 MCU 接受一次有效遥控器启动边沿时，才能置为 `1`
3. `DONE`、`FAIL`、Pi 掉线、人工切入 Manual 或自动挡释放均不得自动将其清为 `0`
4. 只有遥控器 clear/reset 手势可以将其清为 `0`
5. 标志清零时，MCU 必须清理自动任务控制上下文
6. Pi 必须检测边沿，不得在每帧看到 0 时反复清理
7. Pi 收到非法值时应增加协议错误统计，并按 `1` 处理，防止误允许新任务

### 8.2 MCU_IMU

```text
Direction:      MCU -> Pi
MSG_ID:         0x25
Payload length: 48 bytes
Recommended:    100 Hz
ACK:            no
```

| 偏移 | 长度 | 类型 | 字段 | 单位 |
|---:|---:|---|---|---|
| 0 | 4 | `uint32_t` | `stamp_ms` | `ms` |
| 4 | 2 | `uint16_t` | `status_flags` | 位图 |
| 6 | 2 | `uint16_t` | `sequence_count` | 递增计数 |
| 8 | 4 | `int32_t` | `acc_x_mm_s2` | `mm/s²` |
| 12 | 4 | `int32_t` | `acc_y_mm_s2` | `mm/s²` |
| 16 | 4 | `int32_t` | `acc_z_mm_s2` | `mm/s²` |
| 20 | 4 | `int32_t` | `gyro_x_urad_s` | `urad/s` |
| 24 | 4 | `int32_t` | `gyro_y_urad_s` | `urad/s` |
| 28 | 4 | `int32_t` | `gyro_z_urad_s` | `urad/s` |
| 32 | 4 | `int32_t` | `roll_urad` | `urad` |
| 36 | 4 | `int32_t` | `pitch_urad` | `urad` |
| 40 | 4 | `int32_t` | `yaw_urad` | `urad` |
| 44 | 4 | `uint32_t` | reserved | 必须填 0 |

`status_flags`：

| bit | 名称 |
|---:|---|
| bit0 | `IMU_READY` |
| bit1 | `ACC_VALID` |
| bit2 | `GYRO_VALID` |
| bit3 | `ANGLE_VALID` |
| bit4 | `YAW_FUSED_WITH_CHASSIS` |
| bit5 | `BIAS_CORRECTED` |
| bit6 ~ bit15 | reserved |

说明：

1. `yaw_urad` 使用融合后的姿态 `angle.z`
2. 与 `MCU_ODOM.yaw_urad` 同源
3. Pi 不应把 MCU `stamp_ms` 直接解释成 ROS epoch

### 8.3 MCU_ODOM

```text
Direction:      MCU -> Pi
MSG_ID:         0x26
Payload length: 32 bytes
Recommended:    50 Hz
ACK:            no
```

| 偏移 | 长度 | 类型 | 字段 | 单位 / 坐标系 |
|---:|---:|---|---|---|
| 0 | 4 | `uint32_t` | `stamp_ms` | `ms` |
| 4 | 2 | `uint16_t` | `status_flags` | 位图 |
| 6 | 2 | `uint16_t` | `reset_counter` | 里程计复位计数 |
| 8 | 4 | `int32_t` | `x_mm` | odom，`mm` |
| 12 | 4 | `int32_t` | `y_mm` | odom，`mm` |
| 16 | 4 | `int32_t` | `yaw_urad` | odom，`urad` |
| 20 | 4 | `int32_t` | `vx_mm_s` | base_link，`mm/s` |
| 24 | 4 | `int32_t` | `vy_mm_s` | base_link，`mm/s` |
| 28 | 4 | `int32_t` | `wz_urad_s` | base_link，`urad/s` |

`status_flags`：

| bit | 名称 |
|---:|---|
| bit0 | `ODOM_READY` |
| bit1 | `POSE_VALID` |
| bit2 | `TWIST_VALID` |
| bit3 | `IMU_FUSED` |
| bit4 | `WHEEL_FUSED` |
| bit5 | `STATIC_DETECTED` |
| bit6 | `SLIP_SUSPECTED` |
| bit7 | `ODOM_RESET` |
| bit8 ~ bit15 | reserved |

### 8.4 MCU_ARM_STATE

```text
Direction:      MCU -> Pi
MSG_ID:         0x27
Payload length: 48 bytes
Recommended:    50 Hz
ACK:            no
```

| 偏移 | 长度 | 类型 | 字段 | 单位 / 说明 |
|---:|---:|---|---|---|
| 0 | 4 | `uint32_t` | `stamp_ms` | `ms` |
| 4 | 2 | `uint16_t` | `status_flags` | 有效位 |
| 6 | 2 | `uint16_t` | `sequence_count` | 递增计数 |
| 8 | 4 | `int32_t` | `q0_urad` | `urad` |
| 12 | 4 | `int32_t` | `q1_urad` | `urad` |
| 16 | 4 | `int32_t` | `q2_urad` | `urad` |
| 20 | 4 | `int32_t` | `q3_urad` | `urad` |
| 24 | 4 | `int32_t` | `q4_urad` | `urad` |
| 28 | 4 | `int32_t` | `x_mm` | `mm` |
| 32 | 4 | `int32_t` | `y_mm` | `mm` |
| 36 | 4 | `int32_t` | `z_mm` | `mm` |
| 40 | 2 | `int16_t` | `quat_x_q15` | `q / 32767.0` |
| 42 | 2 | `int16_t` | `quat_y_q15` | `q / 32767.0` |
| 44 | 2 | `int16_t` | `quat_z_q15` | `q / 32767.0` |
| 46 | 2 | `int16_t` | `quat_w_q15` | `q / 32767.0` |

`status_flags`：

| bit | 名称 |
|---:|---|
| bit0 | `ARM_READY` |
| bit1 | `JOINT_VALID` |
| bit2 | `FK_VALID / POSE_VALID` |
| bit3 ~ bit15 | reserved |

Pose 表示末端坐标系相对于 `arm_base_link` 的位姿，四元数顺序固定为 `x/y/z/w`

### 8.5 MCU_START_SENSOR_EVENT

```text
Direction:      MCU -> Pi
MSG_ID:         0x22
Payload length: 8 bytes
Default flags:  NEED_ACK
```

| 偏移 | 长度 | 类型 | 字段 | 说明 |
|---:|---:|---|---|---|
| 0 | 4 | `uint32_t` | `stamp_ms` | 时间戳 |
| 4 | 1 | `uint8_t` | `sensor_id` | 传感器 ID |
| 5 | 1 | `uint8_t` | `event_type` | 事件类型 |
| 6 | 2 | `uint16_t` | `event_value` | 事件值 |

当前阶段说明：

1. 当前仅使用遥控器启动，不使用该消息表达遥控器自动启动
2. 遥控器任务启动由 `MCU_STATUS.app_state + auto_start_latched` 表达
3. 本消息保留给未来光电传感器或外部 IO 触发源
4. MCU 内部维护单槽 pending event
5. 未收到 ACK 时每 `100 ms` 重发
6. ACK 超时不阻塞主循环，不进入 `Fault`

### 8.6 MCU_ACK

```text
Direction:      MCU -> Pi
MSG_ID:         0x23
Payload length: 4 bytes
```

| 偏移 | 长度 | 类型 | 字段 |
|---:|---:|---|---|
| 0 | 1 | `uint8_t` | `ack_msg_id` |
| 1 | 1 | `uint8_t` | `ack_seq` |
| 2 | 2 | `uint16_t` | `code` |

用于确认 Pi 的一次性动作或事件；当前主要保留统一格式和发送接口

### 8.7 MCU_FAULT_EVENT

```text
Direction:      MCU -> Pi
MSG_ID:         0x24
Payload length: 8 bytes
Status:         reserved
```

| 偏移 | 长度 | 类型 | 字段 |
|---:|---:|---|---|
| 0 | 4 | `uint32_t` | `stamp_ms` |
| 4 | 1 | `uint8_t` | `fault_source` |
| 5 | 1 | `uint8_t` | `fault_level` |
| 6 | 2 | `int16_t` | `fault_code` |

故障的持续真值仍以周期 `MCU_STATUS` 为准

---

## 9. 遥控器启动与 clear/reset 手势

### 9.1 遥控器通道约定

| 通道 | 含义 |
|---:|---|
| CH4 | `SWA` |
| CH5 | `SWB` |
| CH6 | `SWC` |
| CH7 | `SWD` |
| CH8 | `VRA` |
| CH9 | `VRB` |

当前代码中的语义常量：

```text
REMOTE_SW_HIGH = 1000
REMOTE_SW_CENTER = 1500
REMOTE_SW_LOW = 2000
REMOTE_AUTO_THRESHOLD = 1800
REMOTE_VR_LOW_THRESHOLD = 1200
```

注意：`HIGH/LOW` 是软件语义名称，FS-iA10B 原始数值方向可能与直觉相反

### 9.2 自动启动条件

当前遥控器全自动启动条件：

```text
remote_online == true
SWD == REMOTE_SW_HIGH
VRA >= REMOTE_AUTO_THRESHOLD
VRB >= REMOTE_AUTO_THRESHOLD
```

该条件只能生成一次边沿事件，不能作为持续电平命令

边沿规则：

1. MCU 上电时自动启动边沿默认未武装
2. 遥控器重新上线时自动启动边沿默认未武装
3. 必须先观察到一次“非自动启动条件”，才能武装
4. 从非自动条件进入自动条件时产生一次 pending 启动事件
5. 自动条件持续保持时不得重复产生事件
6. 自动条件释放后可以重新形成输入边沿，但若 `auto_start_latched=1`，app 层仍必须拒绝再次启动

### 9.3 clear/reset 条件

全自动任务复位与可恢复故障清理手势：

```text
remote_online == true
SWC == REMOTE_SW_HIGH
VRA <= REMOTE_VR_LOW_THRESHOLD
VRB <= REMOTE_VR_LOW_THRESHOLD
```

本修订不要求 `SWD` 处于特定位置

clear/reset 也使用边沿事件：

1. 第一次从非清理条件进入清理条件时产生一次事件
2. 持续保持该手势不得重复清理或刷日志
3. 离开该手势后重新武装
4. 遥控器离线时清除 pending，并重新进入未武装状态

### 9.4 启动接受条件

MCU 只在以下条件全部满足时接受启动：

```text
auto_start_latched == 0
app_state == IDLE
Pi online == true
chassis_ready == true
odom_ready == true
not ESTOP
没有阻止启动的 Fault
```

机械臂 ready 不作为仅底盘任务的强制启动条件

启动被拒绝时：

1. `auto_start_latched` 保持 0
2. MCU 保持 `Idle`
3. 不因启动条件不足直接进入 `Fault`
4. 输出限频拒绝原因
5. 操作员必须退出自动条件并重新触发边沿后才能重试

### 9.5 启动接受顺序

推荐顺序：

```text
1. app_control_stop_all()
2. pi_comms_clear_controls()
3. chassis_yaw_hold_reset()
4. auto_start_latched = 1
5. post SWITCH_TO_AUTO_PI
6. MCU_STATUS 持续发送 app_state=AutoPi, auto_start_latched=1
```

### 9.6 clear/reset 行为

收到一次有效 clear/reset 边沿后：

```text
1. auto_start_latched = 0
2. 清除 pending auto-start event
3. 清除 Pi 普通底盘控制缓存
4. 清除 Pi 普通机械臂目标和普通一次性动作
5. 清除普通 yaw action 和 yaw hold 上下文
6. 清除 pending mission event
7. 不清除 pending EStop
8. 停止底盘和机械臂
9. 按现有规则清除 recoverable Fault
10. 根据当前状态进入安全状态
```

不同状态下的行为：

| 当前状态 | clear/reset 后状态 | 说明 |
|---|---|---|
| `Idle` | `Idle` | 清上下文，保持安全停止 |
| `Manual` | `Manual` | 保持人工控制模式，仅清自动任务上下文 |
| `AutoPi` | `Idle` | 人工取消当前全自动任务 |
| `Finished` | `Idle` | 为下一轮任务解锁 |
| recoverable `Fault` | `Idle` | 清故障并解锁 |
| fatal `Fault` | `Fault` | 可清任务锁存，但不得清 fatal fault |
| `EStop` | `EStop` | 可清任务锁存和普通上下文，但不得解除急停 |

核心约束：

```text
任务完成 != 允许再次启动
任务失败 != 允许再次启动
释放自动条件 != 允许再次启动
人工切换 Manual != 允许再次启动

只有 clear/reset 手势可以使 auto_start_latched 从 1 变为 0
```

---

## 10. MCU 状态机权限和转换约束

### 10.1 状态所有权

1. MCU 是状态机唯一所有者
2. `pc_comms` 和 `pi_comms` 不判断状态机权限
3. app/runtime 层统一判断消息是否允许执行
4. Pi 不能通过通信协议请求进入 `AutoPi`

### 10.2 主要转换

| 来源 | 条件 / 事件 | 目标 |
|---|---|---|
| `Idle` | 遥控器 Manual 请求 | `Manual` |
| `Idle` | 有效自动启动事件被 MCU 接受 | `AutoPi` |
| `Manual` | 遥控器退出 Manual | `Idle` |
| `Manual` | EStop / Fault | `EStop / Fault` |
| `AutoPi` | 遥控器 Manual 抢占 | `Manual` |
| `AutoPi` | `PI_MISSION_EVENT.DONE` | `Finished` |
| `AutoPi` | `PI_MISSION_EVENT.FAIL` | `Fault` |
| `AutoPi` | Pi 离线或关键依赖失败 | recoverable `Fault` |
| `AutoPi` | clear/reset 手势 | `Idle` |
| `Finished` | clear/reset 手势 | `Idle` |
| recoverable `Fault` | clear/reset 手势 | `Idle` |
| 任意非 EStop | `PI_ESTOP` | `EStop` |

状态机底层即使支持其他转换，app/runtime 也不得绕过本节业务约束直接触发新的自动任务

### 10.3 人工抢占

1. 遥控器 Manual 请求优先于普通自动控制
2. `AutoPi -> Manual` 时必须立即清 Pi 普通控制缓存
3. 人工抢占不自动清除 `auto_start_latched`
4. 重新启动全自动任务前仍需 clear/reset 手势

---

## 11. Pi 端状态同步与 ROS2 映射

本节描述 Pi bridge 迭代后的行为它不新增线协议消息

### 11.1 /mcu/status

建议 ROS2 消息：

```text
mcu_comm_bridge/msg/McuStatus.msg
```

至少包含：

```text
std_msgs/Header header
uint32 mcu_stamp_ms
uint8 app_state
uint8 manual_mode
uint8 ready_flags
uint8 online_flags
uint8 fault_source
uint8 fault_level
int16 fault_code
bool auto_start_latched
```

推荐话题：

```text
/mcu/status
```

推荐 QoS：

```text
Reliable
KeepLast(1)
TransientLocal
```

目的：后启动的任务管理节点可以立即获得最近一次 MCU 状态真值

### 11.2 Pi 端锁存跟踪规则

Pi 必须跟踪 `auto_start_latched` 和 `app_state`：

| 前值 | 新值 | app_state | Pi 行为 |
|---:|---:|---|---|
| 0 | 0 | 任意 | 不重复 RESET |
| 0 | 1 | `AutoPi` | 产生一次 START |
| 0 | 1 | 非 `AutoPi` | 标记 pending，等待 `AutoPi` |
| 1 | 1 | `AutoPi` | 不重复 START |
| 1 | 1 | 非 `AutoPi` | 保持锁存，不启动新任务 |
| 1 | 0 | 任意 | 清理自动任务上下文，产生一次 RESET |

首次收到状态：

1. `flag=0`：建立基线，清理一次本地自动任务上下文，但不持续发布 RESET
2. `flag=1 && state=AutoPi`：产生一次 START，支持 Pi 晚启动恢复
3. `flag=1 && state!=AutoPi`：保持 pending，不启动任务

START 有效条件：

```text
auto_start_latched == 1
app_state == AUTO_PI
本轮 START 尚未消费
```

### 11.3 /mcu/auto_task_event

建议 ROS2 消息：

```text
mcu_comm_bridge/msg/AutoTaskEvent.msg
```

建议内容：

```text
std_msgs/Header header

uint8 EVENT_START=1
uint8 EVENT_RESET=2

uint8 event
uint32 mcu_stamp_ms
uint8 app_state
bool auto_start_latched
```

推荐话题：

```text
/mcu/auto_task_event
```

推荐 QoS：

```text
Reliable
KeepLast(10)
Volatile
```

不能为一次性事件使用 `TransientLocal`，避免新订阅者把旧 START 当成新任务

职责区分：

```text
/mcu/status：持续状态真值
/mcu/auto_task_event：START / RESET 边沿事件
```

### 11.4 Pi RESET 时清理范围

检测到 `1 -> 0` 后，Pi 应统一调用自动任务上下文复位逻辑，清理：

1. 本轮 START 已触发和已消费标志
2. mission active/done/failed 状态
3. pending mission result
4. 待重发的旧自动底盘控制
5. 待重发的旧机械臂目标
6. 待重发的旧 yaw action
7. 上一轮自动任务命令序号和业务去重状态
8. 与本轮任务相关的 timer、retry、action client 和任务管理上下文

不得清理：

1. 串口连接
2. 二进制流解析器
3. 帧统计和基础诊断
4. IMU/ODOM/ARM_STATE 最近数据
5. `MCU_STATUS` 最近状态
6. ROS2 publisher/subscriber 本身
7. Pi 心跳发送器

### 11.5 日志要求

MCU 周期心跳日志至少包含：

```text
state
manual_mode
remote_online
pc_online
pi_online
auto_start_latched
has_fault
fault_source
fault_level
fault_code
```

示例：

```text
Heartbeat state=AutoPi manual=ManualChassisPcArm remote=1 pc=0 pi=1 auto_start=1 fault=0 src=0 level=0 code=0
```

Pi 只在变化时打印：

```text
MCU state changed: Idle -> AutoPi, auto_start=1
MCU auto_start_latched changed: 0 -> 1
MCU auto task START detected: state=AutoPi auto_start=1
MCU auto_start_latched changed: 1 -> 0
MCU auto task RESET detected: local task context cleared
```

禁止按 `MCU_STATUS` 的 10 Hz 周期重复打印相同变化日志

---

## 12. 高频、一次性和锁存数据分类

### 12.1 高频连续数据

包括：

1. `PC_MASTER_JOINTS`
2. `PI_CONTROL.chassis`
3. `MCU_IMU`
4. `MCU_ODOM`
5. `MCU_ARM_STATE`

消费方式：

```text
get + fresh timeout 或周期覆盖
```

### 12.2 一次性动作

包括：

1. `PI_ARM_ACTION`
2. `PI_YAW_ACTION`
3. `PI_MISSION_EVENT`
4. `PI_ESTOP`
5. `PI_CONTROL.arm` 的新 `command_seq`

消费方式：

```text
take/consume
```

同一缓存结果不得在主循环内重复执行

### 12.3 跨状态锁存量

包括：

```text
MCU_STATUS.auto_start_latched
```

特征：

1. 不是瞬时事件
2. 不是 fresh timeout 控制量
3. 跨越 `AutoPi / Finished / Fault / Manual` 持续保持
4. 只有明确的 clear/reset 手势可以清零
5. Pi 通过边沿生成自己的 START/RESET 事件

---

## 13. ACK 与重发

当前 ACK 重点覆盖 `MCU_START_SENSOR_EVENT`：

1. 发送时设置 `FLAGS.NEED_ACK`
2. 首次发送分配固定 `SEQ`
3. 未收到 ACK 时每 `100 ms` 重发
4. 重发必须保持相同 `SEQ`
5. 收到匹配的 `ack_msg_id + ack_seq` 后清 pending
6. ACK 超时只统计和限频告警，不进入 `Fault`
7. ACK 不用于高频控制帧
8. `auto_start_latched` 依靠周期 `MCU_STATUS` 同步，不使用 ACK

---

## 14. 错误处理策略

以下情况只丢帧、统计和限频告警：

1. CRC 错误
2. 长度错误
3. 未知 `MSG_ID`
4. 协议版本错误
5. ACK 不匹配
6. 非法 action 或 arm_mode
7. 非法 `auto_start_latched` 原始值
8. 非有限机械臂目标
9. IK 无解
10. 目标越界

以下情况可能进入 `Fault`：

1. 底盘真实执行失败
2. 机械臂真实执行失败
3. 关键依赖未就绪但当前状态必须使用
4. `AutoPi` 下 Pi 离线
5. Pi 上报 `MISSION FAIL`

特别约束：

1. 非 `AutoPi` 状态下 Pi 离线不影响手动控制
2. 非 `ManualChassisPcArm` 状态下 PC 离线不影响其他模式
3. 启动条件不足时只拒绝本次启动，不应直接进入 `Fault`
4. recoverable Fault 可由 clear/reset 手势清除
5. fatal Fault 和 EStop 不得被普通 clear/reset 解除

---

## 15. 推荐频率和超时

| 消息 / 项目 | 推荐值 / 当前值 |
|---|---|
| `PC_HEARTBEAT` | 1 Hz |
| `PC_MASTER_JOINTS` | 30~100 Hz，当前默认 50 Hz |
| `PI_HEARTBEAT` | 1 Hz |
| `PI_CONTROL` | 20~50 Hz，当前默认 50 Hz |
| `MCU_STATUS` | 10 Hz |
| `MCU_IMU` | 100 Hz |
| `MCU_ODOM` | 50 Hz |
| `MCU_ARM_STATE` | 50 Hz |
| `MCU_START_SENSOR_EVENT` retry | 100 ms |
| Pi 在线超时 | 3000 ms |
| Pi 底盘控制 fresh timeout | 200 ms |
| Pi 机械臂控制 fresh timeout | 200 ms |
| Pi yaw 控制 fresh timeout | 200 ms |
| 遥控器在线超时 | 100 ms |

发送要求：

1. 发送函数不得长时间阻塞
2. ACK 缺失不得阻塞主循环
3. 500 Hz 主循环不得无节制发送大帧
4. 高频周期帧应错峰发送
5. 心跳频率必须显著高于在线超时所需最低频率

当前 MCU 发送槽位设计：

```text
2 ms 基础槽位
50 个槽位构成 100 ms 周期
IMU:       10 次 / 100 ms = 100 Hz
ODOM:       5 次 / 100 ms = 50 Hz
ARM_STATE:  5 次 / 100 ms = 50 Hz
STATUS:     1 次 / 100 ms = 10 Hz
```

---

## 16. 实现边界

1. `remote.c` 只负责遥控器在线状态、通道解析和边沿事件生成
2. `remote.c` 不得直接修改 `auto_start_latched`
3. `remote.c` 不得调用状态机
4. `app_runtime` 负责锁存标志、状态权限和启动条件检查
5. `app_status` 负责组装状态快照
6. `pi_comms` 只负责将状态快照打包为 `MCU_STATUS`
7. `pi_comms_clear_controls()` 只能清普通控制和普通一次性动作，不得清 pending EStop
8. Pi bridge 只负责协议桥接和边沿发布，不直接启动 Nav2
9. 上层任务管理器订阅 `/mcu/auto_task_event`，执行任务启动和取消
10. UART/HAL 绑定由 assemble 层完成
11. 不使用动态内存实现 MCU 协议状态

---

## 17. 兼容性与部署要求

本修订将 `MCU_STATUS` offset 12 从 reserved 改为 `auto_start_latched`：

1. payload 仍为 16 字节
2. 旧 Pi 解析器会忽略该字节，不会发生帧长度错误
3. 新 Pi 配合旧 MCU 时会一直读到 0，无法检测有效 START
4. 旧 Pi 配合新 MCU 时不会发布 START/RESET 边沿
5. MCU 和 Pi 的本次功能应同步部署和验收
6. 如果未来需要强制拒绝旧端，可升级 `VER`；当前阶段保持 `VER=0x01` 以减少线协议破坏

---

## 18. 验收场景

### 18.1 初始状态

```text
MCU state=Idle
auto_start_latched=0
Pi 建立 0 基线
```

### 18.2 正常启动

```text
遥控器先处于非自动条件
再进入自动条件
Pi online，chassis/odom ready
```

预期：

```text
MCU: Idle -> AutoPi
MCU: auto_start_latched 0 -> 1
Pi:  发布一次 START
```

### 18.3 自动条件持续保持

预期：

```text
不重复启动
不重复清理 Pi 控制缓存
不重复发布 START
```

### 18.4 释放后重新进入自动条件，但未 clear/reset

预期：

```text
auto_start_latched 仍为 1
拒绝新任务
```

### 18.5 任务完成

```text
Pi -> MCU: MISSION DONE
```

预期：

```text
MCU: AutoPi -> Finished
auto_start_latched 仍为 1
Pi 不重复 START
```

### 18.6 任务失败

```text
Pi -> MCU: MISSION FAIL
```

预期：

```text
MCU 进入 recoverable Fault
auto_start_latched 仍为 1
```

### 18.7 clear/reset

```text
SWC 高，VRA 低，VRB 低
```

预期：

```text
MCU: auto_start_latched 1 -> 0
MCU 清自动任务上下文
Pi: 清本地任务上下文
Pi: 发布一次 RESET
```

### 18.8 clear/reset 持续保持

预期：

```text
只处理一次
不重复 RESET
不刷日志
```

### 18.9 第二轮任务

```text
clear/reset 完成
离开 clear/reset 手势
重新形成自动启动边沿
```

预期：

```text
MCU: Idle -> AutoPi
MCU: auto_start_latched 0 -> 1
Pi: 再发布一次 START
```

### 18.10 AutoPi 中 clear/reset

预期：

```text
停止执行机构
清普通 Pi 控制
AutoPi -> Idle
auto_start_latched=0
Pi 发布 RESET
```

### 18.11 EStop 中 clear/reset

预期：

```text
auto_start_latched=0
清普通自动任务上下文
继续保持 EStop
```

### 18.12 Pi 晚启动

MCU 已经：

```text
app_state=AutoPi
auto_start_latched=1
```

Pi bridge 启动后首次收到状态，预期产生一次 START

---

## 19. 后续扩展规则

1. 新增消息优先复用现有统一帧格式
2. 需要 ACK 的一次性线消息统一使用 `SEQ + NEED_ACK + ACK`
3. 不在 PC、Pi、MCU 三端分别实现不同 CRC 变体
4. 新 payload 必须补 offset 表、类型、单位和合法值
5. 新状态字段优先从 reserved 区扩展；不足时再增加新消息或升级协议版本
6. 未来增加光电传感器时，可复用统一的 MCU 启动请求仲裁，但不得绕过 `auto_start_latched`
7. 不同启动源可以通过 `MCU_START_SENSOR_EVENT` 告知来源，但任务启动真值仍以 `MCU_STATUS.app_state + auto_start_latched` 为准
8. Pi 上层任务管理器必须以一次性 START/RESET 边沿驱动任务，不得按 10 Hz 状态帧反复启动或清理

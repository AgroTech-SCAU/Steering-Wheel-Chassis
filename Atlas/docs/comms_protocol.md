# 通信协议说明

## 1. 协议目标

本协议用于统一 `PC <-> MCU` 与 `Pi <-> MCU` 的通信格式，目标如下：

1. 所有通信统一使用同一套二进制帧格式
2. `pc_comms` 与 `pi_comms` 只负责通信、解析、缓存与发送，不直接控制底盘或机械臂
3. 高频连续控制量与一次性动作严格区分
4. 通信错误只丢帧和统计，不直接进入 `Fault`
5. 协议字段、单位、偏移和消息方向明确，便于 PC 端与 Pi 端独立实现打包和解析

## 2. 总体通信关系

### 2.1 PC -> MCU

PC 只负责：

1. 心跳
2. 主臂关节角目标

PC 不负责：

1. 启动系统
2. 停止系统
3. 底盘刹车
4. 机械臂使能
5. 机械臂停止
6. 急停
7. 清故障

### 2.2 Pi -> MCU

Pi 负责：

1. 自动模式下的高频控制输入
2. 一次性机械臂动作
3. 一次性 yaw 动作
4. 任务事件
5. 急停事件
6. 对 MCU 一次性事件的 ACK

### 2.3 MCU -> Pi

MCU 负责：

1. 周期发送状态帧
2. 周期发送 IMU / 里程计帧
3. 发送启动传感器事件
4. 发送 ACK
5. 预留发送故障事件

## 3. 通用帧格式

统一帧格式如下：

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
| 0 | 1 | SOF0 | 固定 `0xA5` |
| 1 | 1 | SOF1 | 固定 `0x5A` |
| 2 | 1 | LEN_H | body 长度高字节 |
| 3 | 1 | LEN_L | body 长度低字节 |
| 4 | 1 | VER | 协议版本，当前 `0x01` |
| 5 | 1 | MSG_ID | 消息 ID |
| 6 | 1 | SEQ | 帧序号 |
| 7 | 1 | FLAGS | 标志位 |
| 8 | N | PAYLOAD | 业务数据 |
| 8 + N | 1 | CRC_H | CRC16 高字节 |
| 9 + N | 1 | CRC_L | CRC16 低字节 |

说明：

1. `LEN` 表示 `VER + MSG_ID + SEQ + FLAGS + PAYLOAD` 的总长度
2. 当前协议版本固定为 `0x01`
3. CRC 使用 `CRC16-CCITT`
4. CRC 覆盖范围为 `SOF + LEN + BODY`
5. `LEN` 与 `CRC` 使用大端格式
6. `PAYLOAD` 内部多字节数字使用小端格式
7. `SEQ` 为 `uint8_t`，按 `0 -> 255` 循环递增
8. `FLAGS bit0` 表示 `NEED_ACK`，其余 bit 预留

## 4. 字节序与单位约定

### 4.1 字节序

1. 帧级别长度和 CRC 使用大端
2. 业务 payload 中的 `uint16_t / int16_t / uint32_t / int32_t` 使用小端
3. 协议中不直接传输 `float`，所有物理量使用定点整数表达

### 4.2 单位

| 物理量 | 类型 | 单位 | 换算 |
|---|---|---|---|
| 时间戳 | `uint32_t` | `ms` | 直接使用 |
| 角度 | `int32_t` | `urad` | `rad = urad * 1e-6` |
| 角速度 | `int32_t` | `urad/s` | `rad/s = urad/s * 1e-6` |
| 底盘角速度 | `int16_t` | `mrad/s` | `rad/s = mrad/s * 1e-3` |
| 线速度 | `int16_t` | `mm/s` | `m/s = mm/s * 1e-3` |
| 位置 | `int32_t` | `mm` | `m = mm * 1e-3` |

## 5. 消息 ID 总表

### 5.1 PC -> MCU

| MSG_ID | 名称 | payload 长度 | 语义 |
|---:|---|---:|---|
| `0x10` | `PC_HEARTBEAT` | 0 | PC 心跳 |
| `0x11` | `PC_MASTER_JOINTS` | 25 | 主臂关节角 + 末端开关 |

### 5.2 MCU -> Pi

| MSG_ID | 名称 | payload 长度 | 语义 |
|---:|---|---:|---|
| `0x21` | `MCU_STATUS` | 16 | MCU 状态 |
| `0x22` | `MCU_START_SENSOR_EVENT` | 8 | 启动传感器事件 |
| `0x23` | `MCU_ACK` | 4 | MCU ACK |
| `0x24` | `MCU_FAULT_EVENT` | 8 | 故障事件，当前预留 |
| `0x25` | `MCU_IMU` | 48 | IMU 周期帧，100Hz |
| `0x26` | `MCU_ODOM` | 32 | 底盘局部里程计周期帧，50Hz |
| `0x27` | `MCU_ARM_STATE` | 40 | 机械臂当前关节角与正解末端位置，50Hz |

### 5.3 Pi -> MCU

| MSG_ID | 名称 | payload 长度 | 语义 |
|---:|---|---:|---|
| `0x30` | `PI_HEARTBEAT` | 0 | Pi 心跳 |
| `0x31` | `PI_CONTROL` | 38 | 自动控制帧 |
| `0x40` | `PI_ARM_ACTION` | 8 | 一次性机械臂动作 |
| `0x41` | `PI_YAW_ACTION` | 12 | 一次性 yaw 动作 |
| `0x42` | `PI_MISSION_EVENT` | 8 | 任务事件 |
| `0x43` | `PI_ESTOP` | 8 | 急停事件 |
| `0x44` | `PI_ACK` | 4 | Pi ACK |

## 6. PC -> MCU 消息

### 6.1 PC_HEARTBEAT

方向：

```text
PC -> MCU
```

`MSG_ID = 0x10`

payload：

```text
empty
```

语义：

1. 更新 PC 在线状态
2. 更新最近一次有效接收时间
3. 不影响主臂关节角缓存

推荐频率：

```text
1Hz，最低不建议低于 0.5Hz
```

### 6.2 PC_MASTER_JOINTS

方向：

```text
PC -> MCU
```

`MSG_ID = 0x11`

payload 长度：

```text
25 bytes
```

payload 偏移：

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

1. 使用小端读取
2. 将 `urad` 转换为 `float rad` 后写入 `FiveDofArmJointArray`
3. 更新主臂关节角缓存和本地 fresh 时间戳
4. `end_switch` 来自 PC 主臂 ID7，`pc_comms` 只解析并缓存，不直接执行业务逻辑

权限约束：

1. `pc_comms` 不判断状态机
2. PC 主臂关节角只允许在 `APP_FSM_STATE_MANUAL + APP_MANUAL_MODE_CHASSIS_PC_ARM` 下被 app 层消费
3. 离开 `ManualChassisPcArm` 时必须调用 `pc_comms_clear_master_joints()`

## 7. Pi -> MCU 消息

### 7.1 PI_HEARTBEAT

方向：

```text
Pi -> MCU
```

`MSG_ID = 0x30`

payload：

```text
empty
```

语义：

1. 更新 Pi 在线状态
2. 更新最近一次有效接收时间
3. 不影响控制缓存

推荐频率：

```text
1Hz，最低不建议低于 0.5Hz
```

### 7.2 PI_CONTROL

方向：

```text
Pi -> MCU
```

`MSG_ID = 0x31`

语义：

高频连续控制帧，使用 `get + fresh timeout`

payload 长度：

```text
38 bytes
```

payload 偏移：

| payload 偏移 | 长度 | 类型 | 字段 | 单位 / 说明 |
|---:|---:|---|---|---|
| 0 | 4 | `uint32_t` | `stamp_ms` | `ms` |
| 4 | 1 | `uint8_t` | `control_mask` | 控制有效位 |
| 5 | 1 | `uint8_t` | `arm_mode` | 机械臂控制模式 |
| 6 | 2 | `uint16_t` | `reserved` | 保留 |
| 8 | 2 | `int16_t` | `vx_mm_s` | `mm/s` |
| 10 | 2 | `int16_t` | `vy_mm_s` | `mm/s` |
| 12 | 2 | `int16_t` | `wz_mrad_s` | `mrad/s`，底盘角速度 |
| 14 | 4 | `int32_t` | `q0_urad` | `urad` |
| 18 | 4 | `int32_t` | `q1_urad` | `urad` |
| 22 | 4 | `int32_t` | `q2_urad` | `urad` |
| 26 | 4 | `int32_t` | `q3_urad` | `urad` |
| 30 | 4 | `int32_t` | `q4_urad` | `urad` |
| 34 | 2 | `uint16_t` | `arm_speed_mrad_s` | `mrad/s` |
| 36 | 2 | `uint16_t` | `reserved2` | 保留 |

`control_mask`：

| bit | 名称 | 说明 |
|---:|---|---|
| bit0 | `chassis_valid` | `vx / vy / wz` 有效 |
| bit1 | `arm_joint_valid` | `q0 ~ q4` 有效 |
| bit2 | `reserved` | 保留，当前不使用 |
| bit3 | `brake_request` | 请求底盘刹车 |
| bit4 ~ bit7 | `reserved` | 保留 |

`arm_mode`：

| 值 | 名称 | 说明 |
|---:|---|---|
| 0 | `none` | 不控制机械臂 |
| 1 | `joint_target` | 关节目标控制 |

语义：

1. `vx / vy` 转换为 `m/s`
2. `wz_mrad_s` 转换为 `rad/s`，表示底盘角速度 `wz`
3. `q0 ~ q4` 转换为 `rad`
4. `arm_speed_mrad_s` 转换为 `rad/s`
5. 高频控制量由 app 层通过 fresh timeout 消费
6. 本协议不再定义独立的 `yaw_rate_valid`
7. 连续旋转控制直接使用 `PI_CONTROL.wz_mrad_s`
8. yaw hold 与 yaw target 通过 `PI_YAW_ACTION` 表达

权限约束：

1. `pi_comms` 不判断状态机
2. `PI_CONTROL` 只允许在 `APP_FSM_STATE_AUTO_PI` 下被 app 层执行
3. 离开 `AutoPi` 时必须调用 `pi_comms_clear_controls()`

多频率控制约束：

1. `chassis_valid` 只表示本帧更新底盘控制缓存
2. `arm_joint_valid` 只表示本帧更新机械臂关节缓存
3. 某个 valid bit 为 0 时，不代表要求清除对应缓存
4. 缓存是否失效由 fresh timeout 判断
5. 离开 `AutoPi` 时再统一清除普通控制缓存

### 7.3 PI_ARM_ACTION

方向：

```text
Pi -> MCU
```

`MSG_ID = 0x40`

语义：

一次性机械臂动作，必须使用 `take/consume`

payload 长度：

```text
8 bytes
```

payload 偏移：

| payload 偏移 | 长度 | 类型 | 字段 | 说明 |
|---:|---:|---|---|---|
| 0 | 4 | `uint32_t` | `stamp_ms` | 时间戳 |
| 4 | 1 | `uint8_t` | `action` | 动作类型 |
| 5 | 1 | `uint8_t` | `reserved` | 保留 |
| 6 | 2 | `uint16_t` | `sequence_id` | 序列编号 |

`action`：

| 值 | 名称 |
|---:|---|
| 1 | `ARM_ENABLE` |
| 2 | `ARM_STOP` |
| 3 | `ARM_SEQUENCE` |

语义：

1. 合法动作被缓存为 pending action
2. app 层通过 `pi_comms_take_arm_action()` 读取
3. 读取后立即清除
4. 不支持的 action 只丢弃并限频告警，不进入 `Fault`

### 7.4 PI_YAW_ACTION

方向：

```text
Pi -> MCU
```

`MSG_ID = 0x41`

语义：

一次性 yaw 动作，必须使用 `take/consume`

payload 长度：

```text
12 bytes
```

payload 偏移：

| payload 偏移 | 长度 | 类型 | 字段 | 单位 / 说明 |
|---:|---:|---|---|---|
| 0 | 4 | `uint32_t` | `stamp_ms` | `ms` |
| 4 | 1 | `uint8_t` | `action` | 动作类型 |
| 5 | 3 | `uint8_t[3]` | `reserved` | 保留 |
| 8 | 4 | `int32_t` | `target_yaw_urad` | `urad` |

`action`：

| 值 | 名称 |
|---:|---|
| 1 | `HOLD_ENABLE` |
| 2 | `HOLD_DISABLE` |
| 3 | `TARGET_SET` |

说明：

1. `PI_YAW_ACTION` 只负责 yaw hold enable / disable / target set
2. 连续旋转控制直接使用 `PI_CONTROL.wz_mrad_s`
3. 本协议不再单独定义 yaw rate 控制帧或 `yaw_rate_valid`

### 7.5 PI_MISSION_EVENT

方向：

```text
Pi -> MCU
```

`MSG_ID = 0x42`

语义：

一次性任务事件，必须使用 `take/consume`

payload 长度：

```text
8 bytes
```

payload 偏移：

| payload 偏移 | 长度 | 类型 | 字段 | 说明 |
|---:|---:|---|---|---|
| 0 | 4 | `uint32_t` | `stamp_ms` | 时间戳 |
| 4 | 1 | `uint8_t` | `event` | 事件类型 |
| 5 | 1 | `uint8_t` | `reserved` | 保留 |
| 6 | 2 | `int16_t` | `code` | 事件码 |

`event`：

| 值 | 名称 |
|---:|---|
| 1 | `DONE` |
| 2 | `FAIL` |

### 7.6 PI_ESTOP

方向：

```text
Pi -> MCU
```

`MSG_ID = 0x43`

语义：

全局一次性事件，必须使用 `take/consume`

payload 长度：

```text
8 bytes
```

payload 偏移：

| payload 偏移 | 长度 | 类型 | 字段 | 说明 |
|---:|---:|---|---|---|
| 0 | 4 | `uint32_t` | `stamp_ms` | 时间戳 |
| 4 | 1 | `uint8_t` | `reason` | 急停原因 |
| 5 | 3 | `uint8_t[3]` | `reserved` | 保留 |

约束：

1. `PI_ESTOP` 任意状态全局生效
2. 收到后 app 运行时必须进入 `EStop`
3. 普通 `clear fault` 不能恢复 `EStop`
4. `pi_comms_clear_controls()` 不能清除 pending 的 EStop 事件
5. EStop 只能由 `pi_comms_take_estop()` 消费

### 7.7 PI_ACK

方向：

```text
Pi -> MCU
```

`MSG_ID = 0x44`

payload 长度：

```text
4 bytes
```

payload 偏移：

| payload 偏移 | 长度 | 类型 | 字段 | 说明 |
|---:|---:|---|---|---|
| 0 | 1 | `uint8_t` | `ack_msg_id` | 被确认的消息 ID |
| 1 | 1 | `uint8_t` | `ack_seq` | 被确认的 SEQ |
| 2 | 2 | `uint16_t` | `code` | 确认结果 |

用途：

1. 确认 MCU 发给 Pi 的一次性事件
2. 当前阶段主要用于 `MCU_START_SENSOR_EVENT` 的 ACK
3. ACK 不匹配只丢弃和统计，不进入 `Fault`

## 8. MCU -> Pi 消息

### 8.1 MCU_IMU

方向：
```text
MCU -> Pi
```

`MSG_ID = 0x25`

建议频率：
```text
100Hz
```

payload 长度：
```text
48 bytes
```

说明：
1. 周期发送完整 IMU 信息，包含三轴加速度、三轴角速度和三轴姿态角
2. `yaw_urad` 使用融合后的 `angle.z`，与 `MCU_ODOM.yaw_urad` 同源
3. 该帧是高频周期帧，不使用 `ACK`，避免阻塞主循环

### 8.2 MCU_ODOM

方向：
```text
MCU -> Pi
```

`MSG_ID = 0x26`

建议频率：
```text
50Hz
```

payload 长度：
```text
32 bytes
```

说明：
1. `x_mm / y_mm / yaw_urad` 属于 odom 坐标系下的局部位姿
2. `vx_mm_s / vy_mm_s / wz_urad_s` 属于 base_link 坐标系下的底盘速度
3. `yaw_urad` 同样使用融合后的 `angle.z`，Pi 端不需要再拼接 odom yaw

### 8.3 MCU_ARM_STATE

方向：
```text
MCU -> Pi
```

`MSG_ID = 0x27`

建议频率：
```text
50Hz
```

payload 长度：
```text
40 bytes
```

说明：
1. 周期发送机械臂当前关节角和当前末端位置
2. `q0 ~ q4` 来自 `arm.get_current_joints()`
3. `x / y / z` 来自 `arm.get_current_pose().position`
4. `x / y / z` 是当前关节角正运动学结果
5. 该帧是周期状态帧，不使用 `ACK`
6. Pi 端根据 `status_flags` 判断字段是否有效

payload 偏移：

| payload 偏移 | 长度 | 类型 | 字段 | 单位 / 说明 |
|---:|---:|---|---|---|
| 0 | 4 | `uint32_t` | `stamp_ms` | `ms` |
| 4 | 2 | `uint16_t` | `status_flags` | 状态有效位 |
| 6 | 2 | `uint16_t` | `sequence_count` | 递增计数 |
| 8 | 4 | `int32_t` | `q0_urad` | `urad` |
| 12 | 4 | `int32_t` | `q1_urad` | `urad` |
| 16 | 4 | `int32_t` | `q2_urad` | `urad` |
| 20 | 4 | `int32_t` | `q3_urad` | `urad` |
| 24 | 4 | `int32_t` | `q4_urad` | `urad` |
| 28 | 4 | `int32_t` | `x_mm` | `mm` |
| 32 | 4 | `int32_t` | `y_mm` | `mm` |
| 36 | 4 | `int32_t` | `z_mm` | `mm` |

`status_flags`：

| bit | 名称 | 说明 |
|---:|---|---|
| bit0 | `arm_ready` | 机械臂服务已初始化 |
| bit1 | `joint_valid` | `q0 ~ q4` 有效 |
| bit2 | `fk_valid` | `x / y / z` 正解结果有效 |

### 8.4 MCU_STATUS

方向：

```text
MCU -> Pi
```

`MSG_ID = 0x21`

建议频率：

```text
5Hz ~ 10Hz
```

payload 长度：

```text
16 bytes
```

payload 偏移：

| payload 偏移 | 长度 | 类型 | 字段 | 说明 |
|---:|---:|---|---|---|
| 0 | 4 | `uint32_t` | `stamp_ms` | 时间戳 |
| 4 | 1 | `uint8_t` | `app_state` | 状态机状态 |
| 5 | 1 | `uint8_t` | `manual_mode` | 手动模式 |
| 6 | 1 | `uint8_t` | `ready_flags` | ready 状态 |
| 7 | 1 | `uint8_t` | `online_flags` | 在线状态 |
| 8 | 1 | `uint8_t` | `fault_source` | 故障来源 |
| 9 | 1 | `uint8_t` | `fault_level` | 故障等级 |
| 10 | 2 | `int16_t` | `fault_code` | 故障码 |
| 12 | 4 | `uint8_t[4]` | `reserved` | 保留 |

`ready_flags`：

| bit | 含义 |
|---:|---|
| bit0 | `chassis_ready` |
| bit1 | `arm_ready` |
| bit2 | `odom_ready` |
| bit3 | `remote_ready` |
| bit4 | `pc_ready` |
| bit5 | `pi_ready` |

`online_flags`：

| bit | 含义 |
|---:|---|
| bit0 | `remote_online` |
| bit1 | `pc_online` |
| bit2 | `pi_online` |
| bit3 | `has_fault` |
| bit4 | `estop` |

### 8.5 MCU_START_SENSOR_EVENT

方向：

```text
MCU -> Pi
```

`MSG_ID = 0x22`

语义：

启动传感器事件，一次性消息，默认带 `NEED_ACK`

payload 长度：

```text
8 bytes
```

payload 偏移：

| payload 偏移 | 长度 | 类型 | 字段 | 说明 |
|---:|---:|---|---|---|
| 0 | 4 | `uint32_t` | `stamp_ms` | 时间戳 |
| 4 | 1 | `uint8_t` | `sensor_id` | 传感器 ID |
| 5 | 1 | `uint8_t` | `event_type` | 事件类型 |
| 6 | 2 | `uint16_t` | `event_value` | 事件值 |

当前阶段机制：

1. MCU 内部维护单槽 pending event
2. 发布后每 `100ms` 重发一次
3. 收到匹配 `PI_ACK` 后清除 pending
4. ACK 超时不会阻塞主循环，不会卡死系统

### 8.6 MCU_ACK

方向：

```text
MCU -> Pi
```

`MSG_ID = 0x23`

payload 长度：

```text
4 bytes
```

payload 偏移：

| payload 偏移 | 长度 | 类型 | 字段 | 说明 |
|---:|---:|---|---|---|
| 0 | 1 | `uint8_t` | `ack_msg_id` | 被确认的消息 ID |
| 1 | 1 | `uint8_t` | `ack_seq` | 被确认的 SEQ |
| 2 | 2 | `uint16_t` | `code` | 确认结果 |

用途：

用于确认 Pi 的一次性动作或事件；当前阶段主要预留统一 ACK 格式和发送接口

### 8.7 MCU_FAULT_EVENT

方向：

```text
MCU -> Pi
```

`MSG_ID = 0x24`

用途：

用于故障发生时的即时通知；当前阶段优先完成消息 ID 和协议层预留，后续可根据业务继续接入

payload 长度：

```text
8 bytes
```

payload 偏移：

| payload 偏移 | 长度 | 类型 | 字段 | 说明 |
|---:|---:|---|---|---|
| 0 | 4 | `uint32_t` | `stamp_ms` | 时间戳 |
| 4 | 1 | `uint8_t` | `fault_source` | 故障来源 |
| 5 | 1 | `uint8_t` | `fault_level` | 故障等级 |
| 6 | 2 | `int16_t` | `fault_code` | 故障码 |

## 9. 高频控制与一次性动作的区别

### 9.1 高频控制

高频连续控制量包括：

1. `PI_CONTROL`
2. `PC_MASTER_JOINTS`

消费方式：

1. 使用 `get + fresh timeout`
2. 超时后视为失效
3. 不做 take/consume

### 9.2 一次性动作

一次性事件包括：

1. `PI_ARM_ACTION`
2. `PI_YAW_ACTION`
3. `PI_MISSION_EVENT`
4. `PI_ESTOP`

消费方式：

1. 使用 `take/consume`
2. 读取后立即清除
3. 不允许在 fresh 窗口内重复执行同一个缓存结果

## 10. ACK 与重发机制

当前阶段 ACK 机制重点覆盖 `MCU_START_SENSOR_EVENT`：

1. 发送时带 `FLAGS.NEED_ACK`
2. 使用固定 `SEQ`
3. 未收到 ACK 时每 `100ms` 重发
4. 收到 `ack_msg_id + ack_seq` 匹配的 `PI_ACK` 后清除 pending
5. ACK 超时只统计和限频告警，不进入 `Fault`
6. ACK 不用于高频控制帧

## 11. 状态机权限约束

1. `pc_comms` 与 `pi_comms` 只负责通信，不做状态机权限判断
2. PC 主臂关节角只允许在 `ManualChassisPcArm` 下生效
3. 普通 Pi 控制只允许在 `AutoPi` 下生效
4. `PI_ARM_ACTION / PI_YAW_ACTION / PI_MISSION_EVENT` 只允许在合适的自动状态下被 app 层消费
5. `PI_ESTOP` 任意状态全局生效
6. 所有权限判断仍由 `app_runtime / app_control` 完成

## 12. 错误处理策略

以下情况只丢帧和统计，不进入 `Fault`：

1. CRC 错误
2. 长度错误
3. 未知 `MSG_ID`
4. 协议版本错误
5. ACK 不匹配

以下情况仍可能进入 `Fault`：

1. 底盘真实执行失败
2. 机械臂真实执行失败
3. 关键依赖未就绪但当前状态必须使用
4. `AutoPi` 下 Pi 离线导致的 recoverable fault

需要特别强调：

1. IK 无解不进入 `Fault`
2. 目标越界不进入 `Fault`
3. 非法命令不进入 `Fault`
4. 只拒绝本周期控制
5. 非 `AutoPi` 状态下 Pi 离线不影响手动控制
6. 非 `ManualChassisPcArm` 状态下 PC 离线不影响手动底盘控制

## 13. 推荐发送频率

建议如下：

| 消息 | 推荐频率 |
|---|---|
| `PC_HEARTBEAT` | 1Hz，最低不建议低于 0.5Hz |
| `PC_MASTER_JOINTS` | 30Hz ~ 100Hz |
| `PI_HEARTBEAT` | 1Hz，最低不建议低于 0.5Hz |
| `PI_CONTROL` | 20Hz ~ 50Hz |
| `MCU_IMU` | 100Hz |
| `MCU_ODOM` | 50Hz |
| `MCU_ARM_STATE` | 50Hz |
| `MCU_STATUS` | 5Hz ~ 10Hz |
| `MCU_START_SENSOR_EVENT` pending retry | 100ms |

要求：

1. 不在发送函数中长时间阻塞
2. 不因 ACK 缺失而阻塞主循环
3. 不在 500Hz 主循环里无节制发送大帧
4. 心跳频率需要高于在线超时判断所需频率，避免串口抖动导致误判离线

## 14. 实现注意事项

1. `pc_comms` 与 `pi_comms` 只负责接收、校验、解析、缓存和发送
2. `pc_comms` 与 `pi_comms` 不直接 include app 层、底盘、机械臂、里程计或遥控器模块
3. UART 与 HAL 绑定应由 `assemble_pc_comms` 和 `assemble_pi_comms` 完成
4. `PI_CONTROL` 中某个 valid bit 为 0 时，不应主动清除对应旧缓存
5. 旧缓存是否失效由 fresh timeout 判断
6. 离开对应状态时再统一调用 clear 接口清除普通控制缓存
7. `pi_comms_clear_controls()` 只能清普通控制和普通一次性动作，不能清 pending EStop
8. `PI_ESTOP` 只能由 `pi_comms_take_estop()` 消费
9. `MCU_ARM_STATE` 由 `app_status` 组装快照，`pi_comms` 只负责打包发送
10. `arm.refresh_current_state()` 仍由 `entry.h` 的 50Hz slot 调度，`app_status` 只读取 `arm.get_current_joints()` 和 `arm.get_current_pose()` 缓存

## 15. 后续扩展规则

1. 新增消息优先复用统一帧格式，不再引入第二套协议
2. 需要 ACK 的一次性消息统一使用 `SEQ + NEED_ACK + ACK` 机制
3. `pc_comms` 与 `pi_comms` 内不要再重复实现新的流式解析器
4. 所有新消息都应明确方向、payload 长度、单位和权限边界
5. 所有新 payload 都应补充 offset 表，避免 PC / Pi / MCU 三端实现出现字段错位

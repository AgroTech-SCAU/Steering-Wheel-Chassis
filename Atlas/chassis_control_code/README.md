# chassis_control_code

`chassis_control_code` 是 Atlas 的 STM32 MCU 实时控制代码；它负责底盘、机械臂、IMU/里程计、遥控器、PC/Pi 通信以及本地安全状态机，不负责导航、视觉和整车任务规划

## 1. 目录结构

```text
chassis_control_code/
├── robot.ioc                    # CubeMX 配置
├── arm_description/             # URDF、关节配置、MATLAB 模型
└── src/
    ├── platform/                # STM32 HAL 适配
    ├── device/                  # 电机、舵机、IMU、遥控器、RGB
    ├── domain/                  # 运动学、Kalman、纯数学模型
    ├── infra/                   # 二进制帧、解析器、HFSM、PID、日志、矩阵、时间
    ├── service/                 # chassis/odom/arm/remote/pc_comms/pi_comms
    │   └── assemble/            # 设备与平台装配
    └── app/                     # entry、runtime、FSM、control、status
```

依赖方向建议保持：

```text
app -> service -> domain/device -> infra/platform
```

`domain` 和 `infra` 不应直接依赖 STM32 HAL

## 2. 启动与调度

应用入口位于 `src/app/entry.h``entry_init()` 的装配顺序为：

```text
delay -> log -> rgb -> imu -> odom -> chassis -> arm -> remote
      -> pc_comms -> pi_comms -> app_runtime/app_status -> TIM6 500Hz
```

`entry_loop()` 调度：

| 频率 | 任务 |
|---:|---|
| 500 Hz | `chassis.process()`、`app_runtime_process()` |
| 250 Hz | `odom.process()` |
| 100 Hz | `remote_process()`、`pc_comms_process()`、`pi_comms_process()` |
| 50 Hz | `arm.refresh_current_state()` |
| 后台 | LED、Pi 状态/传感器发送、1 Hz 日志 |

Pi 发送采用 2 ms 槽位错峰：IMU 100 Hz、ODOM 50 Hz、ARM_STATE 50 Hz、STATUS 10 Hz

## 3. 本地状态机

状态定义：

| 值 | 状态 | 说明 |
|---:|---|---|
| 0 | Idle | 安全停止，等待人工输入 |
| 1 | Manual | 遥控手动控制 |
| 2 | AutoPi | 接受 Pi 自动控制 |
| 3 | Fault | 锁存故障，只有 recoverable 可清理 |
| 4 | EStop | 急停锁死，普通清理不能恢复 |
| 5 | Finished | Pi 上报任务完成后的停止态 |

手动子模式：

- `ManualChassisPcArm`：遥控底盘 + PC 主臂跟随
- `ManualArmFs`：遥控机械臂

注意：HFSM 的原始转移表仍允许 `Manual/Finished -> AutoPi`，但 `app_runtime` 只在 Idle 接受遥控自动启动；后续建议收紧底层转移表，形成双重约束

## 4. 遥控器模式与自动任务锁存

遥控器所有模式、手势和摇杆行为都由以下通道组合决定：

| 通道 | 作用 |
|---|---|
| `SWA` | 手动子模式选择 |
| `SWB` | 速度档位 |
| `SWC` | 手动底盘/机械臂局部行为，以及清理/复位手势的一部分 |
| `SWD` | 手动请求 / 自动启动请求 |
| `RIGHT_X` | 底盘横向、机械臂末端 yaw |
| `RIGHT_Y` | 底盘前后、机械臂末端 pitch / 伸缩 |
| `LEFT_X` | 底盘偏航、机械臂基座 yaw |
| `VRA` | 自动启动 / 清理复位门槛 |
| `VRB` | 自动启动 / 清理复位门槛 |

通道名对应关系见 `src/service/remote.h` 中的 `REMOTE_CH_*` 定义

### 4.1 `SWD`：手动请求与自动请求

`SWD` 是主模式开关：

| 条件 | 结果 |
|---|---|
| `SWD == REMOTE_SW_LOW` | 进入手动请求 |
| `SWD == REMOTE_SW_HIGH` 且 `VRA >= REMOTE_AUTO_THRESHOLD` 且 `VRB >= REMOTE_AUTO_THRESHOLD` | 自动启动条件成立 |
| 其他情况 | 不触发手动请求，也不触发自动启动 |

说明：

- `remote.c` 会把持续条件转换为一次性边沿事件
- 上电或遥控重连后，必须先离开对应条件，再重新进入，才会重新武装下一次事件
- `app_runtime` 只在 `Idle` 接受自动启动
- `SWD` 处于手动请求时，自动启动不生效
- 退出手动请求后，如果当前在 Manual，系统会停止底盘/机械臂并返回 Idle

### 4.2 `SWA`：手动子模式

`SWA` 只在手动请求生效时决定进入哪种手动子模式：

| 条件 | 结果 |
|---|---|
| `SWA == REMOTE_SW_HIGH` | `ManualChassisPcArm`：遥控底盘 + PC 主臂跟随 |
| `SWA == REMOTE_SW_LOW` | `ManualArmFs`：遥控机械臂 |
| `SWA == REMOTE_SW_CENTER` | 默认进入 `ManualChassisPcArm` |

说明：

- `remote.c` 里只有高/低两档会被显式识别为手动源，`app_runtime` 会把中位默认当作 `ManualChassisPcArm`
- `SWA` 本身不触发模式切换，只决定手动请求被接受后进入哪种 Manual 子模式

### 4.3 `SWB`：速度档位

`SWB` 不是模式开关，而是速度档位开关，手动底盘和手动机械臂都会受它影响

| 条件 | 底盘速度限制 | 机械臂速度限制 |
|---|---|---|
| `SWB == REMOTE_SW_LOW` | 快档：`2.0 / 2.0 / 8.0` | 快档：`50.24 / 1.5 / 3.0 / 21 / 21 / 50.24` |
| `SWB == REMOTE_SW_HIGH` | 慢档：`0.5 / 0.5 / 2.0` | 慢档：`12.56 / 0.5 / 1.0 / 7 / 7 / 12.56` |
| `SWB` 中位 | 中档：`1.0 / 1.0 / 4.0` | 中档：`25.12 / 1.0 / 2.0 / 14 / 14 / 25.12` |

注：上表数值顺序分别表示底盘的 `vx / vy / wz` 上限，以及机械臂的 `base_end_yaw / reach / z / end_pitch / end_yaw / servo` 速度上限

### 4.4 `SWC`：局部行为与清理/复位

`SWC` 在手动控制里决定底盘是否启用 steer-then-drive，也参与清理/复位手势：

| 条件 | 结果 |
|---|---|
| `SWC == REMOTE_SW_LOW` | 手动底盘关闭 steer-then-drive；手动机械臂按关节模式微调 `q3/q4` |
| `SWC == REMOTE_SW_CENTER` | 手动底盘开启 steer-then-drive；手动机械臂按基座 yaw + 末端位姿模式控制 |
| `SWC == REMOTE_SW_HIGH` | 手动底盘直接刹停；手动机械臂仅在从非高位切入时执行一次 `move_servo_zero()`，并清空吸盘；同时作为清理/复位手势的一部分 |

### 4.4.1 手动底盘摇杆映射

仅在 `ManualChassisPcArm` 且遥控在线时生效

| 输入 | 作用 |
|---|---|
| `RIGHT_Y` | 底盘前后速度 `vx` |
| `RIGHT_X` | 底盘横向速度 `vy` |
| `LEFT_X` | 底盘角速度 `wz` |
| `SWB` | 速度上限档位 |

附加规则：

- `SWC == REMOTE_SW_LOW` 时，steer-then-drive 关闭
- `SWC == REMOTE_SW_CENTER` 时，steer-then-drive 开启
- `SWC == REMOTE_SW_HIGH` 或 `VRA <= REMOTE_VR_LOW_THRESHOLD` 时，底盘直接刹停
- `VRB > REMOTE_VR_LOW_THRESHOLD` 时，底盘不再输出速度，而是把目标置零并重置 yaw hold
- 只有在 `VRA` 和 `VRB` 都满足低位控制窗口时，才会真正把摇杆映射成连续速度控制

### 4.4.2 手动机械臂摇杆映射

仅在 `ManualArmFs` 或 `ManualChassisPcArm` 且遥控在线时生效

| 输入 | `SWC == REMOTE_SW_LOW` 时作用 | `SWC == REMOTE_SW_CENTER` 时作用 |
|---|---|---|
| `LEFT_X` | 不参与机械臂控制 | 基座 yaw 微调 |
| `RIGHT_Y` | 末端 pitch / 关节 3 控制 | 末端沿当前基座方向前后移动 |
| `RIGHT_X` | 末端 yaw / 关节 4 控制 | 末端高度控制 |

附加规则：

- `SWC == REMOTE_SW_HIGH` 时，机械臂只做一次 `move_servo_zero()`，之后保持静止
- `VRB > REMOTE_VR_LOW_THRESHOLD` 时，机械臂不接收连续摇杆控制
- 每次手动机械臂动作都会强制关闭吸盘，即 `suction_set(false)`

清理/复位条件为遥控器持续满足以下组合：

```text
SWC == REMOTE_SW_HIGH
VRA <= REMOTE_VR_LOW_THRESHOLD
VRB <= REMOTE_VR_LOW_THRESHOLD
```

边沿触发后：

- `auto_start_latched = 0`
- 清除 pending 自动启动事件
- 清除 PC 主臂缓存、Pi 普通控制/动作、yaw hold
- 停止底盘和机械臂
- AutoPi/Finished 转 Idle
- recoverable Fault 清除后转 Idle
- Manual 保持 Manual
- EStop 保持 EStop

任务 DONE/FAIL 或遥控器退出自动位置都不会自动解锁，必须执行一次清理/复位手势

### 4.5 自动启动手势

自动启动条件为遥控器持续满足以下组合：

```text
SWD == REMOTE_SW_HIGH
VRA >= REMOTE_AUTO_THRESHOLD
VRB >= REMOTE_AUTO_THRESHOLD
```

MCU 仅在以下条件全部满足时接受：

- 当前为 Idle
- `auto_start_latched == 0`
- 无锁存 Fault、非 EStop
- Pi online
- chassis ready
- odom ready

接受后先停止执行机构、清除旧 Pi 控制和 yaw hold，再执行：

```text
auto_start_latched = 1
Idle -> AutoPi
```

### 4.6 触发语义总述

`remote.c` 不会把“拨到位”直接当作命令，而是把持续条件变成一次性事件：

- 自动启动需要 `SWD` 高位 + `VRA/VRB` 高位，并且先离开再重新进入才会再次触发
- 清理/复位需要 `SWC` 高位 + `VRA/VRB` 低位，并且先离开再重新进入才会再次触发
- 手动请求由 `SWD` 低位触发，进入后由 `SWA` 决定手动子模式
- `SWB` 始终只影响速度档，不改变状态机模式
- `SWC` 高位在手动机械臂中还会触发一次性的零位收拢动作

## 5. 控制权限

| 数据/动作 | 生效状态 |
|---|---|
| `PC_MASTER_JOINTS` | Manual + `ManualChassisPcArm` |
| 遥控底盘 | Manual |
| 遥控机械臂 | Manual + `ManualArmFs` |
| `PI_CONTROL` 底盘 | AutoPi |
| `PI_CONTROL` 机械臂 | AutoPi，`command_seq` 单次消费 |
| `PI_YAW_ACTION` | 合适的 AutoPi 控制周期 |
| `PI_MISSION_EVENT` | 仅 AutoPi |
| `PI_ESTOP` | 任意状态 |

通信 service 只负责解析和缓存，权限判断位于 `app_runtime/app_control`

## 6. 状态和日志

`MCU_STATUS` 16 字节中 offset 12 为 `auto_start_latched`；1 Hz Heartbeat 日志示例：

```text
Heartbeat state=AutoPi manual=ManualChassisPcArm remote=1 pc=1 pi=1 auto_start=1 fault=0 src=0 level=0 code=0
```

RGB 状态：

| 状态 | 颜色 |
|---|---|
| 未 ready | 红 |
| Idle/Finished ready | 绿 |
| Manual | 蓝 |
| AutoPi | 青 |
| Fault | 橙 |
| EStop | 紫 |

## 7. 关键接口

- `src/app/app_runtime.*`：模式仲裁、安全和自动任务锁存
- `src/app/app_fsm.*`：本地 HFSM
- `src/app/app_control.*`：遥控/Pi 命令到执行机构的映射
- `src/app/app_status.*`：状态、传感器发送和日志
- `src/service/pi_comms.*`：Pi 帧解析、控制缓存和 MCU 上行帧
- `src/service/pc_comms.*`：PC 主臂帧
- `src/service/remote.*`：遥控状态与边沿事件
- `src/infra/binary_frame.*`：统一线协议

完整协议见 [`../docs/comms_protocol.md`](../docs/comms_protocol.md)

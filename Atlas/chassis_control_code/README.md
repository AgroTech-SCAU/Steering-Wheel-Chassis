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

### 4.1 自动启动

自动条件：

```text
SWD == REMOTE_SW_HIGH
VRA >= REMOTE_AUTO_THRESHOLD
VRB >= REMOTE_AUTO_THRESHOLD
```

`remote.c` 将持续电平转换为一次性边沿事件；上电或遥控重连后必须先观察到非自动条件，才会武装下一次边沿

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

### 4.2 清理/复位手势

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

任务 DONE/FAIL 或遥控器退出自动位置都不会自动解锁，必须执行清理手势

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

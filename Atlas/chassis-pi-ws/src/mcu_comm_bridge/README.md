# mcu_comm_bridge 说明

本包负责 pi 与 mcu 之间的串口协议桥接

它把 mcu 的二进制协议转换成 ROS2 话题和服务，也把 pi 端控制服务转换成 mcu 能识别的二进制控制帧

## 一，包定位

本包是底层通信桥，不是任务管理器

负责

```text
解析 MCU_STATUS
解析 MCU_IMU
解析 MCU_ODOM
解析 MCU_ARM_STATE
发布 /mcu/status
发布 /mcu/auto_task_event
发布 /odom
发布 /imu
发布 /arm/joint_states
发布 /arm/pose
发布 /arm/pose_position
订阅 /motor_cmd_vel 并发送 PI_CONTROL 底盘部分
提供机械臂目标服务并发送 PI_CONTROL arm 部分
提供任务结果服务并发送 PI_MISSION_EVENT
提供急停服务并发送 PI_ESTOP
发送 PI_HEARTBEAT
```

不负责

```text
不请求 mcu 进入 AutoPi
不实现自动任务流程
不执行导航
不执行视觉
不执行授粉动作序列
不根据业务逻辑判断 DONE 或 FAIL
```

## 二，与协议的关系

本包遵守 `comms_protocol.md` 中的约束

关键约束

```text
mcu 是状态机唯一所有者
pi 不具备切换 mcu 模式的线协议权限
AutoPi 的启动真值来自 MCU_STATUS.app_state 和 auto_start_latched
DONE 和 FAIL 只通过 PI_MISSION_EVENT 上报
clear/reset 只能由遥控器手势触发
```

## 三，发布话题

### /mcu/status

类型

```text
mcu_comm_bridge/msg/McuStatus
```

用途

```text
持续发布 mcu 状态，ready 位，在线位，故障信息，自动启动锁存
```

QoS 建议

```text
可靠
只保留最新一帧
瞬态本地
```

后启动的任务节点可以立即拿到最近一次 mcu 状态

### /mcu/auto_task_event

类型

```text
mcu_comm_bridge/msg/AutoTaskEvent
```

用途

```text
把 auto_start_latched 和 app_state 的边沿变化转换为一次性 START 或 RESET
```

事件

```text
EVENT_START
EVENT_RESET
```

注意

```text
这个话题不能使用瞬态本地
避免新订阅者把已发布的 START 当成新任务
```

### /odom

类型

```text
nav_msgs/msg/Odometry
```

来源

```text
MCU_ODOM
```

用途

```text
伪导航后端使用该话题闭环移动
```

### /imu

类型

```text
sensor_msgs/msg/Imu
```

来源

```text
MCU_IMU
```

### /arm/joint_states

类型

```text
sensor_msgs/msg/JointState
```

来源

```text
MCU_ARM_STATE 中的五个关节角
```

### /arm/pose

类型

```text
geometry_msgs/msg/PoseStamped
```

来源

```text
MCU_ARM_STATE 中末端位姿
```

### /arm/pose_position

类型

```text
geometry_msgs/msg/PointStamped
```

用途

```text
手眼标定工具和视觉授粉后端可使用该话题读取末端位置
```

## 四，订阅话题

### /motor_cmd_vel

类型

```text
geometry_msgs/msg/Twist
```

用途

```text
接收总任务状态机安全门控后的最终底盘速度
```

注意

```text
导航后端不能直接发布 /motor_cmd_vel
导航后端应发布 /atlas/navigation/cmd_vel
由 atlas_mission_manager 决定是否转发
```

## 五，服务接口

### /mcu/set_brake

用途

```text
请求底盘刹车或解除刹车
```

### /mcu/set_arm_joints

用途

```text
发送五关节目标
```

说明

```text
服务成功只表示命令写入串口，不表示 mcu 已执行完成
上层必须通过 /arm/joint_states 判断到位
```

### /mcu/set_arm_position

用途

```text
发送末端位置目标 x，y，z
```

说明

```text
服务成功只表示命令写入串口，不表示 IK 成功或动作完成
上层必须通过 /arm/pose_position 判断是否到位
```

### /mcu/report_mission_result

用途

```text
pi 上报任务 DONE 或 FAIL
```

约束

```text
DONE 后 mcu 进入 Finished
FAIL 后 mcu 进入 recoverable Fault
DONE 和 FAIL 都不会清除 auto_start_latched
下一轮必须通过遥控器 clear/reset 手势解锁
```

### /mcu/estop

用途

```text
pi 主动发送急停事件
```

## 六，配置文件

配置文件

```text
config/mcu_comm_bridge.yaml
```

重点字段

| 字段 | 说明 |
|---|---|
| `port` | 串口设备路径 |
| `baudrate` | 串口波特率 |
| `publish_tf` | 是否发布 odom 到 base_footprint 的坐标变换 |
| `cmd_vel_timeout_ms` | 底盘速度超时时间 |
| `control_rate_hz` | PI_CONTROL 发送频率 |
| `arm_command_repeat_count` | 机械臂目标重复发送次数 |
| `mission_event_repeat_count` | DONE 或 FAIL 重复发送次数 |
| `auto_ack_start_sensor_event` | 是否自动确认未来启动传感器事件 |

## 七，单独启动

```bash
ros2 launch mcu_comm_bridge mcu_comm_bridge.launch.py
```

## 八，联调检查

确认 mcu 状态

```bash
ros2 topic echo /mcu/status \
  --qos-reliability reliable \
  --qos-durability transient_local
```

确认里程计

```bash
ros2 topic hz /odom
ros2 topic echo /odom
```

确认机械臂反馈

```bash
ros2 topic hz /arm/joint_states
ros2 topic echo /arm/pose_position
```

确认刹车服务

```bash
ros2 service call /mcu/set_brake std_srvs/srv/SetBool "{data: true}"
```

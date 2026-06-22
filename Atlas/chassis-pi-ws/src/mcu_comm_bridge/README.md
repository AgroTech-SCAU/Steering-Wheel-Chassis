# mcu_comm_bridge

`mcu_comm_bridge` 是树莓派端的 ROS2 Humble 通信桥接节点，用于连接 ROS2 导航/任务系统与 STM32 MCU；节点通过串口接收 MCU 上报的 IMU、里程计、机械臂状态和系统状态，并将 ROS2 侧的速度控制、yaw 控制、刹车和急停命令打包发送给 MCU

---

## 1. 职责边界

本节点负责：

1. 打开并维护 Pi 与 MCU 的串口连接
2. 使用统一二进制帧协议完成收发、CRC 校验和流式解析
3. 解析 `MCU_STATUS(0x21)`、`MCU_IMU(0x25)`、`MCU_ODOM(0x26)`、`MCU_ARM_STATE(0x27)`
4. 发布 `/odom`、`/imu`、`/arm/joint_states`、`/arm/fk_position`
5. 可选发布 `odom -> base_footprint` TF
6. 订阅 `/motor_cmd_vel`，按固定频率发送 `PI_CONTROL(0x31)`
7. 提供刹车、yaw hold、yaw target、急停等 service
8. 发送 `PI_HEARTBEAT(0x30)`
9. 对 `MCU_START_SENSOR_EVENT(0x22)` 自动回复 `PI_ACK(0x44)`
10. 统计串口帧数、CRC 错误、未知消息、各类消息接收计数

本节点不负责：

1. Nav2 路径规划
2. SLAM 或重定位
3. competition_fsm 任务仲裁
4. MCU 状态机权限判断
5. MCU 底层电机、舵机、IMU 驱动
6. 机械臂任务序列执行

---

## 2. ROS 接口原则

当前节点按以下原则划分 ROS 接口：

```text
周期性控制量 -> topic
一次性动作命令 -> service
长周期且需要进度反馈的任务 -> action
```

因此：

1. `/motor_cmd_vel` 是周期速度控制，使用 topic
2. `/mcu/set_brake`、`/mcu/set_yaw_hold`、`/mcu/set_yaw_target`、`/mcu/estop` 是一次性命令，使用 service
3. 当前包不定义 action

---

## 3. 发布话题

| 话题 | 类型 | 来源 | 说明 |
|---|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | `MCU_ODOM(0x26)` | 底盘局部里程计 |
| `/imu` | `sensor_msgs/msg/Imu` | `MCU_IMU(0x25)` | IMU 姿态、角速度、线加速度 |
| `/arm/joint_states` | `sensor_msgs/msg/JointState` | `MCU_ARM_STATE(0x27)` | 机械臂 q0~q4 当前关节角 |
| `/arm/fk_position` | `geometry_msgs/msg/PointStamped` | `MCU_ARM_STATE(0x27)` | 当前关节角正解得到的末端 xyz |

当参数 `publish_tf=true` 时，节点发布：

```text
odom -> base_footprint
```

---

## 4. 订阅话题

| 话题 | 类型 | 去向 | 说明 |
|---|---|---|---|
| `/motor_cmd_vel` | `geometry_msgs/msg/Twist` | `PI_CONTROL(0x31)` | competition_fsm 仲裁后的底盘周期速度 |

正式系统中，节点默认订阅 `/motor_cmd_vel`；单独调试时可以通过参数把 `cmd_vel_topic` 改为 `/cmd_vel`

---

## 5. 服务

| 服务 | 类型 | 去向 | 说明 |
|---|---|---|---|
| `/mcu/set_brake` | `std_srvs/srv/SetBool` | `PI_CONTROL(0x31)` | `true` 锁存刹车，`false` 解除刹车 |
| `/mcu/set_yaw_hold` | `std_srvs/srv/SetBool` | `PI_YAW_ACTION(0x41)` | 开启或关闭 MCU 侧 yaw hold |
| `/mcu/set_yaw_target` | `mcu_comm_bridge/srv/SetYawTarget` | `PI_YAW_ACTION(0x41)` | 设置目标 yaw，单位 rad |
| `/mcu/estop` | `mcu_comm_bridge/srv/Estop` | `PI_ESTOP(0x43)` | 发送急停事件 |

服务返回成功表示 Pi 端已经完成协议帧写入，不表示 MCU 一定执行；MCU 是否执行由 MCU 状态机、安全逻辑和当前控制模式决定

---

## 6. 参数

配置文件：

```text
config/mcu_comm_bridge.yaml
```

常用参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `port` | `/dev/mcu_uart` | MCU 串口设备 |
| `baudrate` | `921600` | MCU 串口波特率 |
| `odom_topic` | `/odom` | 里程计话题 |
| `imu_topic` | `/imu` | IMU 话题 |
| `cmd_vel_topic` | `/motor_cmd_vel` | 底盘控制输入话题 |
| `arm_joint_state_topic` | `/arm/joint_states` | 机械臂关节角话题 |
| `arm_fk_topic` | `/arm/fk_position` | 机械臂末端位置话题 |
| `odom_frame_id` | `odom` | odom 坐标系 |
| `base_frame_id` | `base_footprint` | 底盘坐标系 |
| `imu_frame_id` | `imu_link` | IMU 坐标系 |
| `arm_frame_id` | `arm_base_link` | 机械臂基坐标系 |
| `publish_tf` | `true` | 是否发布 `odom -> base_footprint` |
| `heartbeat_rate_hz` | `1.0` | Pi 心跳频率 |
| `control_rate_hz` | `50.0` | `PI_CONTROL` 发送频率 |
| `cmd_vel_timeout_ms` | `200` | 速度指令超时时间 |
| `send_brake_on_cmd_timeout` | `true` | 速度超时后是否发送零速刹车 |
| `max_vx_m_s` | `1.5` | x 方向速度限幅 |
| `max_vy_m_s` | `1.5` | y 方向速度限幅 |
| `max_wz_rad_s` | `1.0` | 角速度限幅 |
| `log_stats_period_ms` | `1000` | 统计日志周期 |
| `log_latest_sample` | `false` | 是否打印最近一次样本 |

---

## 7. 坐标系

`/odom` 使用：

```text
header.frame_id = odom
child_frame_id = base_footprint
```

`/imu` 使用：

```text
header.frame_id = imu_link
```

`/arm/joint_states` 和 `/arm/fk_position` 使用：

```text
header.frame_id = arm_base_link
```

完整导航 TF 链路建议为：

```text
map -> odom -> base_footprint -> base_link -> laser_link
```

---

## 8. 控制发送策略

`/motor_cmd_vel` 回调只缓存最新速度，不直接写串口

控制定时器按 `control_rate_hz` 发送 `PI_CONTROL`，默认 50Hz

如果 `/motor_cmd_vel` 超过 `cmd_vel_timeout_ms` 未更新，且 `send_brake_on_cmd_timeout=true`，节点会发送一次零速刹车帧，避免 MCU 继续使用旧速度缓存

当 `/mcu/set_brake` 请求为 `true` 时，节点会锁存刹车状态，控制定时器持续发送零速 + `brake_request`；当 `/mcu/set_brake` 请求为 `false` 时，节点解除刹车锁存并恢复速度控制

---

## 9. MCU_ARM_STATE 解析

`MCU_ARM_STATE(0x27)` 是 MCU 到 Pi 的机械臂状态周期帧，payload 长度为 40 bytes，推荐频率 50Hz

payload 字段：

| 偏移 | 长度 | 类型 | 字段 | 单位 / 说明 |
|---:|---:|---|---|---|
| 0 | 4 | `uint32_t` | `stamp_ms` | ms |
| 4 | 2 | `uint16_t` | `status_flags` | 状态有效位 |
| 6 | 2 | `uint16_t` | `sequence_count` | MCU 递增计数 |
| 8 | 4 | `int32_t` | `q0_urad` | urad |
| 12 | 4 | `int32_t` | `q1_urad` | urad |
| 16 | 4 | `int32_t` | `q2_urad` | urad |
| 20 | 4 | `int32_t` | `q3_urad` | urad |
| 24 | 4 | `int32_t` | `q4_urad` | urad |
| 28 | 4 | `int32_t` | `x_mm` | mm |
| 32 | 4 | `int32_t` | `y_mm` | mm |
| 36 | 4 | `int32_t` | `z_mm` | mm |

`status_flags`：

| bit | 名称 | 说明 |
|---:|---|---|
| bit0 | `arm_ready` | 机械臂服务已初始化 |
| bit1 | `joint_valid` | q0~q4 有效 |
| bit2 | `fk_valid` | x/y/z 正解结果有效 |

单位换算：

```text
q_rad = q_urad * 1e-6
x_m = x_mm * 1e-3
y_m = y_mm * 1e-3
z_m = z_mm * 1e-3
```

当 `joint_valid` 有效时发布 `/arm/joint_states`

当 `fk_valid` 有效时发布 `/arm/fk_position`

---

## 10. 编译

```bash
cd ~/chassis-pi-ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## 11. 启动

使用 launch：

```bash
ros2 launch mcu_comm_bridge mcu_comm_bridge.launch.py
```

临时指定串口：

```bash
ros2 run mcu_comm_bridge mcu_comm_bridge_node --ros-args \
  -p port:=/dev/ttyACM0 \
  -p baudrate:=921600
```

单独调试底盘速度时订阅 `/cmd_vel`：

```bash
ros2 run mcu_comm_bridge mcu_comm_bridge_node --ros-args \
  -p cmd_vel_topic:=/cmd_vel
```

---

## 12. 运行检查

检查节点：

```bash
ros2 node list
ros2 node info /mcu_comm_bridge_node
```

检查话题频率：

```bash
ros2 topic hz /imu
ros2 topic hz /odom
ros2 topic hz /arm/joint_states
ros2 topic hz /arm/fk_position
```

检查 TF：

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

查看数据：

```bash
ros2 topic echo /odom
ros2 topic echo /imu
ros2 topic echo /arm/joint_states
ros2 topic echo /arm/fk_position
```

---

## 13. 服务调用示例

开启刹车：

```bash
ros2 service call /mcu/set_brake std_srvs/srv/SetBool "{data: true}"
```

解除刹车：

```bash
ros2 service call /mcu/set_brake std_srvs/srv/SetBool "{data: false}"
```

开启 yaw hold：

```bash
ros2 service call /mcu/set_yaw_hold std_srvs/srv/SetBool "{data: true}"
```

关闭 yaw hold：

```bash
ros2 service call /mcu/set_yaw_hold std_srvs/srv/SetBool "{data: false}"
```

设置目标 yaw：

```bash
ros2 service call /mcu/set_yaw_target mcu_comm_bridge/srv/SetYawTarget "{yaw_rad: 1.57}"
```

发送急停：

```bash
ros2 service call /mcu/estop mcu_comm_bridge/srv/Estop "{reason: 1}"
```

---

## 14. 协议文档

完整协议见工作区：

```text
docs/comms_protocol.md
```

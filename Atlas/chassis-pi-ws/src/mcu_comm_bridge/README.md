# mcu_comm_bridge

`mcu_comm_bridge` 是树莓派端的 ROS2 Humble 通信桥接节点，面向 `Atlas/navigation_system` 的导航链路使用

## 职责边界

本节点负责：

1. 通过串口和 MCU 通信
2. 解析 MCU 周期发送的 `MCU_IMU(0x25)` 与 `MCU_ODOM(0x26)`
3. 发布导航系统需要的 `/odom`
4. 发布 IMU 话题 `/imu`
5. 可选发布 `odom -> base_footprint` TF
6. 订阅 `competition_fsm` 输出的 `/motor_cmd_vel`
7. 将 `/motor_cmd_vel` 以 50Hz 转换为 `PI_CONTROL(0x31)` 下发给 MCU
8. 提供底盘一次性服务：刹车、yaw hold、yaw target、急停
9. 发送 `PI_HEARTBEAT(0x30)`
10. 对带 `NEED_ACK` 的 `MCU_START_SENSOR_EVENT(0x22)` 自动回复 `PI_ACK(0x44)`

本节点不负责：

1. Nav2 规划
2. competition_fsm 任务仲裁
3. Cartographer 定位
4. MCU 状态机权限判断
5. 机械臂任务逻辑
6. 长周期动作管理

## 接口原则

当前按以下原则划分 ROS 接口：

```text
周期性动作 -> topic
一次性动作 -> service
长期动作序列 -> action，当前暂不需要
```

因此：

```text
/motor_cmd_vel 是周期速度控制，用 topic
set_brake / set_yaw_hold / set_yaw_target / estop 是一次性命令入口，用 service
```

## 话题

### 发布

| 话题 | 类型 | 来源 | 说明 |
|---|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | `MCU_ODOM(0x26)` | 底盘局部里程计 |
| `/imu` | `sensor_msgs/msg/Imu` | `MCU_IMU(0x25)` | IMU 与融合姿态 |

当 `publish_tf=true` 时，本节点发布：

```text
odom -> base_footprint
```

### 订阅

| 话题 | 类型 | 去向 | 说明 |
|---|---|---|---|
| `/motor_cmd_vel` | `geometry_msgs/msg/Twist` | `PI_CONTROL(0x31)` | competition_fsm 仲裁后的底盘周期速度 |

正式系统中，本节点默认订阅 `/motor_cmd_vel`，不要直接绕过 `competition_fsm` 订阅 `/cmd_vel`

单独调试时可以通过参数把 `cmd_vel_topic` 改成 `/cmd_vel`

## 服务

| 服务 | 类型 | 去向 | 说明 |
|---|---|---|---|
| `/mcu/set_brake` | `std_srvs/srv/SetBool` | `PI_CONTROL(0x31)` | `true` 锁存刹车并持续发送零速刹车，`false` 解除刹车锁存 |
| `/mcu/set_yaw_hold` | `std_srvs/srv/SetBool` | `PI_YAW_ACTION(0x41)` | `true` 开启 yaw hold，`false` 关闭 yaw hold |
| `/mcu/set_yaw_target` | `mcu_comm_bridge/srv/SetYawTarget` | `PI_YAW_ACTION(0x41)` | 设置目标 yaw，单位 rad |
| `/mcu/estop` | `mcu_comm_bridge/srv/Estop` | `PI_ESTOP(0x43)` | 发送急停原因码 |

服务只表示 Pi 节点已把对应协议帧写入串口，不代表 MCU 已经完成执行
MCU 是否执行仍由 MCU 状态机、AutoPi 状态和安全逻辑决定

## 坐标系约定

`/odom` 使用：

```text
header.frame_id = odom
child_frame_id = base_footprint
```

这与当前导航 TF 链路一致：

```text
map -> odom -> base_footprint -> base_link -> laser_link
```

`/imu` 使用：

```text
header.frame_id = imu_link
```

## 控制发送策略

`/motor_cmd_vel` 回调只缓存最新速度，不直接写串口
控制定时器以 `control_rate_hz` 周期发送 `PI_CONTROL`，默认 50Hz

如果 `/motor_cmd_vel` 超过 `cmd_vel_timeout_ms` 没有更新，且 `send_brake_on_cmd_timeout=true`，本节点会发送一次零速刹车帧，避免 MCU 继续使用旧速度缓存

`/mcu/set_brake` 为 `true` 时，节点会锁存刹车状态，控制定时器持续发送零速 + `brake_request`，直到服务再次被调用并传入 `false`

## 服务调用示例

开启刹车锁存：

```bash
ros2 service call /mcu/set_brake std_srvs/srv/SetBool "{data: true}"
```

解除刹车锁存：

```bash
ros2 service call /mcu/set_brake std_srvs/srv/SetBool "{data: false}"
```

开启 yaw hold：

```bash
ros2 service call /mcu/set_yaw_hold std_srvs/srv/SetBool "{data: true}"
```

设置目标 yaw：

```bash
ros2 service call /mcu/set_yaw_target mcu_comm_bridge/srv/SetYawTarget "{yaw_rad: 1.57}"
```

发送急停：

```bash
ros2 service call /mcu/estop mcu_comm_bridge/srv/Estop "{reason: 1}"
```

## 编译

```bash
cd ~/chassis-pi-ws
colcon build --symlink-install
source install/setup.bash
```

## 启动

```bash
ros2 launch mcu_comm_bridge mcu_comm_bridge.launch.py
```

临时指定串口：

```bash
ros2 run mcu_comm_bridge mcu_comm_bridge_node --ros-args \
  -p port:=/dev/ttyACM0 \
  -p baudrate:=921600
```

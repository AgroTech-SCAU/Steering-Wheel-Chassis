# chassis-pi-ws

`chassis-pi-ws` 是运行在树莓派 5 上的 ROS2 Humble 工作区，用于连接上层导航/任务系统与 STM32 MCU；它负责把 MCU 上报的底盘、IMU、机械臂状态转换为 ROS2 标准话题，同时把 ROS2 侧的速度控制和一次性控制命令转换为 MCU 二进制协议帧

---

## 1. 系统定位

整车通信关系如下：

```text
PC 主臂 teleop
        ↓ PC_HEARTBEAT / PC_MASTER_JOINTS
STM32 MCU
        ↑↓
chassis-pi-ws
        ↑↓
ROS2 导航 / competition_fsm / 上层任务
```

`chassis-pi-ws` 位于 Pi 端，主要承担三类工作：

1. **MCU 数据上行**：解析 `MCU_STATUS`、`MCU_IMU`、`MCU_ODOM`、`MCU_ARM_STATE` 等帧，并发布 ROS2 话题
2. **Pi 控制下行**：把 `/motor_cmd_vel`、yaw 控制、刹车、急停等指令打包为 `PI_CONTROL`、`PI_YAW_ACTION`、`PI_ESTOP`
3. **通信维护**：发送 `PI_HEARTBEAT`，对 `MCU_START_SENSOR_EVENT` 自动回复 `PI_ACK`，统计协议错误和通信状态

---

## 2. 工作区结构

```text
chassis-pi-ws/
  README.md
  docs/
    comms_protocol.md
  src/
    mcu_comm_bridge/
      CMakeLists.txt
      package.xml
      README.md
      config/
        mcu_comm_bridge.yaml
      include/mcu_comm_bridge/
        binary_frame.hpp
        serial_port.hpp
      launch/
        mcu_comm_bridge.launch.py
      src/
        binary_frame.cpp
        serial_port.cpp
        mcu_comm_bridge_node.cpp
      srv/
        Estop.srv
        SetYawTarget.srv
```

核心包为 `mcu_comm_bridge`

---

## 3. MCU 上行数据

MCU 通过统一二进制协议向 Pi 周期发送状态和传感器数据

| 消息 | MSG_ID | 推荐频率 | Pi 端用途 |
|---|---:|---:|---|
| `MCU_STATUS` | `0x21` | 5Hz ~ 10Hz | MCU 状态、ready、online、fault、estop |
| `MCU_START_SENSOR_EVENT` | `0x22` | pending 100ms 重发 | 通知 Pi 启动传感器，Pi 自动回复 ACK |
| `MCU_ACK` | `0x23` | 事件应答 | 确认 Pi 端一次性消息 |
| `MCU_FAULT_EVENT` | `0x24` | 事件触发 | 故障事件预留 |
| `MCU_IMU` | `0x25` | 100Hz | 发布 `/imu` |
| `MCU_ODOM` | `0x26` | 50Hz | 发布 `/odom` 和可选 TF |
| `MCU_ARM_STATE` | `0x27` | 50Hz | 发布 `/arm/joint_states` 和 `/arm/fk_position` |

---

## 4. ROS2 发布话题

| 话题 | 类型 | 来源 | 说明 |
|---|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | `MCU_ODOM(0x26)` | 底盘局部里程计 |
| `/imu` | `sensor_msgs/msg/Imu` | `MCU_IMU(0x25)` | IMU 姿态、角速度、线加速度 |
| `/arm/joint_states` | `sensor_msgs/msg/JointState` | `MCU_ARM_STATE(0x27)` | 机械臂 q0~q4 当前关节角，单位 rad |
| `/arm/fk_position` | `geometry_msgs/msg/PointStamped` | `MCU_ARM_STATE(0x27)` | 当前关节角正解得到的末端 xyz，单位 m |
| `odom -> base_footprint` | TF | `MCU_ODOM(0x26)` | 当 `publish_tf=true` 时发布 |

### 4.1 `/odom`

`/odom` 表示底盘在局部 odom 坐标系下的位姿和速度

坐标约定：

```text
header.frame_id = odom
child_frame_id = base_footprint
```

它主要供 Nav2、定位模块、状态机和 rosbag 记录使用

### 4.2 `/imu`

`/imu` 表示 MCU 上报的 IMU 数据，包含：

1. orientation
2. angular_velocity
3. linear_acceleration

`MCU_IMU` 中包含 roll、pitch、yaw，Pi 端将其转换为四元数后发布

### 4.3 `/arm/joint_states`

`/arm/joint_states` 来自 `MCU_ARM_STATE`，包含 q0~q4 五个关节角

默认关节名：

```text
joint_0
joint_1
joint_2
joint_3
joint_4
```

当 `MCU_ARM_STATE.status_flags` 中 `joint_valid` 为有效时才发布

### 4.4 `/arm/fk_position`

`/arm/fk_position` 来自 `MCU_ARM_STATE`，表示 MCU 根据当前关节角正运动学计算得到的末端位置

默认坐标系：

```text
arm_base_link
```

当 `MCU_ARM_STATE.status_flags` 中 `fk_valid` 为有效时才发布

---

## 5. ROS2 订阅话题

| 话题 | 类型 | 去向 | 说明 |
|---|---|---|---|
| `/motor_cmd_vel` | `geometry_msgs/msg/Twist` | `PI_CONTROL(0x31)` | competition_fsm 仲裁后的底盘速度指令 |

正式系统中，`mcu_comm_bridge` 默认订阅 `/motor_cmd_vel`，不直接订阅 Nav2 原始 `/cmd_vel`

典型控制链路：

```text
Nav2
  ↓ /cmd_vel
competition_fsm
  ↓ /motor_cmd_vel
mcu_comm_bridge
  ↓ PI_CONTROL
MCU
```

单独调试底盘时，可以通过参数临时把 `cmd_vel_topic` 改为 `/cmd_vel`

---

## 6. ROS2 服务

| 服务 | 类型 | 协议帧 | 说明 |
|---|---|---|---|
| `/mcu/set_brake` | `std_srvs/srv/SetBool` | `PI_CONTROL(0x31)` | 设置或解除刹车锁存 |
| `/mcu/set_yaw_hold` | `std_srvs/srv/SetBool` | `PI_YAW_ACTION(0x41)` | 开启或关闭 MCU 侧 yaw hold |
| `/mcu/set_yaw_target` | `mcu_comm_bridge/srv/SetYawTarget` | `PI_YAW_ACTION(0x41)` | 设置目标 yaw，单位 rad |
| `/mcu/estop` | `mcu_comm_bridge/srv/Estop` | `PI_ESTOP(0x43)` | 发送急停事件 |

服务返回成功只表示 Pi 端已经把协议帧写入串口，不代表 MCU 已经执行完成；最终是否执行由 MCU 状态机、AutoPi 状态和安全逻辑决定

---

## 7. Pi 下行协议

Pi 端向 MCU 发送：

| 消息 | MSG_ID | 推荐频率 / 触发方式 | 用途 |
|---|---:|---|---|
| `PI_HEARTBEAT` | `0x30` | 1Hz | Pi 在线状态 |
| `PI_CONTROL` | `0x31` | 20Hz ~ 50Hz | 底盘速度、机械臂关节目标、刹车请求 |
| `PI_ARM_ACTION` | `0x40` | service / 任务触发 | 一次性机械臂动作 |
| `PI_YAW_ACTION` | `0x41` | service / 任务触发 | yaw hold、yaw target |
| `PI_MISSION_EVENT` | `0x42` | 任务触发 | 任务完成/失败事件 |
| `PI_ESTOP` | `0x43` | service / 安全触发 | 急停 |
| `PI_ACK` | `0x44` | 自动应答 | 确认 MCU 一次性事件 |

`/motor_cmd_vel` 回调只缓存最新速度，不直接写串口；控制定时器按 `control_rate_hz` 周期发送 `PI_CONTROL`

---

## 8. 坐标系约定

导航 TF 链路建议保持：

```text
map -> odom -> base_footprint -> base_link -> laser_link
```

其中：

1. `odom -> base_footprint` 由 `mcu_comm_bridge` 根据 `MCU_ODOM` 发布
2. `base_footprint -> base_link` 由机器人模型或 `robot_state_publisher` 发布
3. `base_link -> laser_link` 由机器人模型或静态 TF 发布
4. `/arm/fk_position` 默认使用 `arm_base_link`，可通过 `arm_frame_id` 参数修改

---

## 9. 环境准备

推荐环境：

```text
硬件：Raspberry Pi 5
系统：Ubuntu 22.04
ROS：ROS2 Humble
通信：USB CDC 或 USB-TTL 串口
```

安装依赖：

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep
```

初始化 rosdep：

```bash
sudo rosdep init
rosdep update
```

如果已经初始化过 rosdep，可以忽略 `sudo rosdep init` 的重复提示

---

## 10. 串口权限与固定设备名

将当前用户加入 `dialout`：

```bash
sudo usermod -aG dialout $USER
sudo reboot
```

检查串口设备：

```bash
ls -l /dev/ttyACM*
ls -l /dev/ttyUSB*
```

为了避免设备名变化，建议使用 udev 固定 MCU 串口名为 `/dev/mcu_uart`

查看设备信息：

```bash
udevadm info -a -n /dev/ttyACM0 | grep -E "idVendor|idProduct|serial" | head
```

创建规则：

```bash
sudo nano /etc/udev/rules.d/99-mcu-uart.rules
```

STM32 USB CDC 可参考：

```text
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="mcu_uart", GROUP="dialout", MODE="0660"
```

重新加载：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

重新插拔 MCU 后检查：

```bash
ls -l /dev/mcu_uart
```

---

## 11. 编译

```bash
cd ~/chassis-pi-ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## 12. 参数配置

配置文件：

```text
src/mcu_comm_bridge/config/mcu_comm_bridge.yaml
```

常用配置：

```yaml
mcu_comm_bridge_node:
  ros__parameters:
    port: "/dev/mcu_uart"
    baudrate: 921600

    odom_topic: "/odom"
    imu_topic: "/imu"
    cmd_vel_topic: "/motor_cmd_vel"
    arm_joint_state_topic: "/arm/joint_states"
    arm_fk_topic: "/arm/fk_position"

    odom_frame_id: "odom"
    base_frame_id: "base_footprint"
    imu_frame_id: "imu_link"
    arm_frame_id: "arm_base_link"

    publish_tf: true

    heartbeat_rate_hz: 1.0
    control_rate_hz: 50.0
    cmd_vel_timeout_ms: 200
    send_brake_on_cmd_timeout: true

    max_vx_m_s: 1.5
    max_vy_m_s: 1.5
    max_wz_rad_s: 1.0
```

---

## 13. 启动

使用 launch 文件启动：

```bash
cd ~/chassis-pi-ws
source install/setup.bash
ros2 launch mcu_comm_bridge mcu_comm_bridge.launch.py
```

临时指定串口：

```bash
ros2 run mcu_comm_bridge mcu_comm_bridge_node --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baudrate:=921600
```

单独调试底盘时订阅 `/cmd_vel`：

```bash
ros2 run mcu_comm_bridge mcu_comm_bridge_node --ros-args \
  -p cmd_vel_topic:=/cmd_vel
```

---

## 14. 运行检查

### 14.1 节点

```bash
ros2 node list
ros2 node info /mcu_comm_bridge_node
```

### 14.2 话题

```bash
ros2 topic list
ros2 topic hz /imu
ros2 topic hz /odom
ros2 topic hz /arm/joint_states
ros2 topic hz /arm/fk_position
```

期望频率：

| 话题 | 期望频率 |
|---|---:|
| `/imu` | 约 100Hz |
| `/odom` | 约 50Hz |
| `/arm/joint_states` | 约 50Hz |
| `/arm/fk_position` | 约 50Hz |

查看内容：

```bash
ros2 topic echo /odom
ros2 topic echo /imu
ros2 topic echo /arm/joint_states
ros2 topic echo /arm/fk_position
```

### 14.3 TF

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_tools view_frames
```

---

## 15. 底盘控制测试

发布前进速度：

```bash
ros2 topic pub /motor_cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

发布横向速度：

```bash
ros2 topic pub /motor_cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

发布旋转速度：

```bash
ros2 topic pub /motor_cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
```

---

## 16. 服务调用示例

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

## 17. 推荐联调顺序

1. 检查 `/dev/mcu_uart` 是否存在
2. 启动 `mcu_comm_bridge`
3. 检查 `/imu`、`/odom`、`/arm/joint_states`、`/arm/fk_position`
4. 检查 `odom -> base_footprint` TF
5. 发布 `/motor_cmd_vel` 验证 `PI_CONTROL`
6. 调用 `/mcu/set_brake`、`/mcu/set_yaw_hold`、`/mcu/set_yaw_target`、`/mcu/estop`
7. 启动 competition_fsm
8. 启动完整导航系统

---

## 18. 常见问题

### 18.1 找不到串口

```bash
ls -l /dev/ttyACM*
ls -l /dev/ttyUSB*
ls -l /dev/mcu_uart
```

如果没有 `/dev/mcu_uart`，检查 udev 规则是否生效，并重新插拔 MCU

### 18.2 串口权限不足

```bash
groups
sudo usermod -aG dialout $USER
sudo reboot
```

### 18.3 有串口数据但没有 ROS 话题

优先检查：

1. 波特率是否一致
2. MCU 是否发送新版二进制帧
3. CRC 是否一致
4. payload 长度是否一致
5. Pi 端是否打开了正确串口

### 18.4 `/odom` 有数据但 TF 不正常

检查：

```yaml
publish_tf: true
odom_frame_id: "odom"
base_frame_id: "base_footprint"
```

同时确认机器人模型发布 `base_footprint -> base_link`

### 18.5 发布 `/motor_cmd_vel` 后底盘不动

可能原因：

1. MCU 不在 AutoPi 状态
2. Pi 未成功发送 `PI_CONTROL`
3. `/mcu/set_brake` 仍处于 true
4. MCU 底盘未 ready
5. MCU 处于 fault 或 estop
6. 速度被 Pi 端限幅为 0

### 18.6 `/arm/joint_states` 或 `/arm/fk_position` 没有数据

检查 MCU 是否发送 `MCU_ARM_STATE(0x27)`，并确认 `status_flags` 中对应有效位为 1

---

## 19. 协议文档

完整通信协议见：

```text
docs/comms_protocol.md
```

该文档是 PC、Pi、MCU 三端实现协议字段、单位、长度和消息 ID 的依据

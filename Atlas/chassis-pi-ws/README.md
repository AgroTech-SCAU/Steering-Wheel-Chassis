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
        SetArmJoints.srv
        SetArmOrientation.srv
        SetArmPose.srv
        SetArmPosition.srv
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
| `/mcu/set_brake` | `std_srvs/srv/SetBool` | `PI_CONTROL(0x31)` | 设置或解除 Pi 侧刹车锁存 |
| `/mcu/set_arm_joints` | `mcu_comm_bridge/srv/SetArmJoints` | `PI_CONTROL(0x31)` | 设置五关节目标角 |
| `/mcu/set_arm_pose` | `mcu_comm_bridge/srv/SetArmPose` | `PI_CONTROL(0x31)` | 设置五维末端目标 `x/y/z/pitch/yaw` |
| `/mcu/set_arm_position` | `mcu_comm_bridge/srv/SetArmPosition` | `PI_CONTROL(0x31)` | 只设置末端位置 `x/y/z` |
| `/mcu/set_arm_orientation` | `mcu_comm_bridge/srv/SetArmOrientation` | `PI_CONTROL(0x31)` | 保持当前位置，设置 `pitch/yaw` |
| `/mcu/set_yaw_hold` | `std_srvs/srv/SetBool` | `PI_YAW_ACTION(0x41)` | 开启或关闭 MCU 侧 yaw hold |
| `/mcu/set_yaw_target` | `mcu_comm_bridge/srv/SetYawTarget` | `PI_YAW_ACTION(0x41)` | 设置目标 yaw，单位 rad |
| `/mcu/estop` | `mcu_comm_bridge/srv/Estop` | `PI_ESTOP(0x43)` | 发送急停事件 |

机械臂服务只在 MCU 处于 `AutoPi` 模式时允许执行；四个机械臂服务会把目标放入 bridge 本地待发送缓存，默认使用同一个 `arm_command_seq` 重发 3 次；MCU 对同一个序号只消费一次

服务返回 `success=true` 的含义需要区分：

- 机械臂服务：命令已经进入 bridge 本地发送队列；不表示 MCU 已收到、IK 已成功或机械臂已到位；
- yaw、急停服务：至少一帧已经成功写入 Pi 串口；不表示 MCU 已完成动作；
- 解除刹车：只解除 bridge 的刹车锁存，不会主动让底盘运动，后续仍需新的 `/motor_cmd_vel`

`/mcu_comm_bridge_node/get_parameters` 等服务是 ROS 2 节点自动提供的参数管理接口，不是底盘或机械臂业务控制接口

---

## 7. Pi 下行协议

Pi 端向 MCU 发送：

| 消息 | MSG_ID | 推荐频率 / 触发方式 | 用途 |
|---|---:|---|---|
| `PI_HEARTBEAT` | `0x30` | 1Hz | Pi 在线状态 |
| `PI_CONTROL` | `0x31` | 20Hz ~ 50Hz | 底盘速度、机械臂 joints/pose/position/orientation 目标、刹车请求 |
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
    port: "/dev/ttyUSB0"
    baudrate: 1000000

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
  -p baudrate:=1000000
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


### 14.4 日志字段说明与状态码约定

节点启动后主要会看到三类日志：启动配置日志、周期统计日志、最近样本日志；`stats_rate_hz` 控制统计日志打印频率，`log_latest_sample=true` 时会在统计日志后继续打印最近一帧 IMU、ODOM、ARM、STATUS 的解析结果

典型日志格式如下：

```text
serial opened: /dev/ttyUSB0 @ 1000000
mcu_comm_bridge started: port=/dev/ttyUSB0 baudrate=1000000 odom_topic=/odom imu_topic=/imu cmd_vel_topic=/motor_cmd_vel
stats: imu=... odom=... arm=... status=... start_evt=... ack_rx=... fault=... unknown=... bad_len=... tx_hb=... tx_ack=... tx_ctrl=... tx_yaw=... tx_estop=... tx_fail=... parser_frames=... crc_err=... len_err=... ver_err=...
latest imu: stamp=... acc=[...]m/s2 gyro=[...]rad/s rpy=[...]rad flags=0x.... seq=...
latest odom: stamp=... pose=[...] vel=[...] flags=0x.... reset=...
latest arm: stamp=... q=[...] xyz=[...] flags=0x.... seq=...
latest status: stamp=... app=... manual=... ready=0x.. online=0x.. fault_src=... fault_level=... fault_code=...
```

#### 启动配置日志

| 字段 | 含义 | 正常判断 |
|---|---|---|
| `port` | Pi 与 MCU 通信使用的串口设备 | 应和实际设备一致，例如 `/dev/ttyACM0` 或 `/dev/mcu_uart` |
| `baudrate` | 串口波特率 | 必须和 MCU 端一致，当前默认 `1000000` |
| `odom_topic` | MCU_ODOM 发布到 ROS2 的话题 | 默认 `/odom` |
| `imu_topic` | MCU_IMU 发布到 ROS2 的话题 | 默认 `/imu` |
| `cmd_vel_topic` | Pi 端订阅的底盘速度指令话题 | 正式系统默认 `/motor_cmd_vel`，单独调试可改为 `/cmd_vel` |

#### `stats:` 周期统计字段

这些字段都是从节点启动后累计的计数，用于判断“有没有收到帧、有没有发出帧、协议有没有错”

| 字段 | 含义 | 异常判断 |
|---|---|---|
| `imu` | 成功解析并发布的 `MCU_IMU(0x25)` 数量 | 一直为 0：MCU 未发 IMU 或协议不匹配 |
| `odom` | 成功解析并发布的 `MCU_ODOM(0x26)` 数量 | 一直为 0：MCU 未发里程计或 payload 长度不对 |
| `arm` | 成功解析的 `MCU_ARM_STATE(0x27)` 数量 | 一直为 0：机械臂状态未上报 |
| `status` | 成功解析的 `MCU_STATUS(0x21)` 数量 | 一直为 0：MCU 状态帧未上报，无法判断模式/故障 |
| `start_evt` | 收到的 `MCU_START_SENSOR_EVENT(0x22)` 数量 | 持续增长：MCU 一直请求 Pi 启动传感器，可能 ACK 未匹配 |
| `ack_rx` | 收到的 `MCU_ACK(0x23)` 数量 | 用于确认 Pi 一次性事件是否被 MCU 应答 |
| `fault` | 收到的 `MCU_FAULT_EVENT(0x24)` 数量 | 非 0 表示 MCU 主动上报过故障事件 |
| `unknown` | 收到未知 `MSG_ID` 的数量 | 非 0 通常表示三端协议版本不一致 |
| `bad_len` | payload 长度不符合当前 Pi 端定义的数量 | 非 0 通常表示 MCU 发送的 payload 长度和 Pi 端代码不一致 |
| `tx_hb` | Pi 已发送的 `PI_HEARTBEAT(0x30)` 数量 | 不增长：心跳定时器异常或串口发送失败 |
| `tx_ack` | Pi 已发送的 `PI_ACK(0x44)` 数量 | 收到 `start_evt` 但不增长：`auto_ack_start_sensor_event` 可能关闭或事件未带 `NEED_ACK` |
| `tx_ctrl` | Pi 已发送的 `PI_CONTROL(0x31)` 数量 | 发布 `/motor_cmd_vel` 后不增长：控制话题未收到、超时或节点未订阅正确话题 |
| `tx_yaw` | Pi 已发送的 `PI_YAW_ACTION(0x41)` 数量 | 调 yaw 服务后不增长：服务未调用成功或串口写失败 |
| `tx_estop` | Pi 已发送的 `PI_ESTOP(0x43)` 数量 | 调急停服务后应增长，默认会重复发送 `repeat_estop_count` 次 |
| `tx_fail` | 串口写失败次数 | 非 0：检查串口断开、权限、线缆和 MCU 复位 |
| `parser_frames` | 二进制解析器成功解析出的完整帧数 | 有串口字节但不增长：帧头、LEN、CRC 或版本不匹配 |
| `crc_err` | CRC 校验失败次数 | 非 0：检查 CRC 算法、覆盖范围、字节序、串口干扰 |
| `len_err` | 帧级 `LEN` 非法次数 | 非 0：检查 body 长度定义和 `max_body_len` |
| `ver_err` | 协议版本不匹配次数 | 非 0：检查 `PROTOCOL_VERSION` 是否都是 `0x01` |

#### `latest imu:` 字段

| 字段 | 含义 | 单位 / 状态码 |
|---|---|---|
| `stamp` | MCU 端采样时间戳 | `ms`，来自 payload 的 `stamp_ms` |
| `acc=[x y z]` | 三轴线加速度 | `m/s2`，由 `mm/s2` 换算 |
| `gyro=[x y z]` | 三轴角速度 | `rad/s`，由 `urad/s` 换算 |
| `rpy=[roll pitch yaw]` | MCU 融合姿态角 | `rad`，由 `urad` 换算 |
| `flags` | IMU 状态标志 | 当前 Pi 端只打印不判定；建议见下表 |
| `seq` | IMU 样本计数 | MCU 端递增，用于观察丢帧或停更 |

建议的 `MCU_IMU.status_flags` 约定如下；当前 Pi 端不会因为这些位为 0 而停止发布 `/imu`，只用于日志诊断

| bit | 名称 | 含义 |
|---:|---|---|
| bit0 | `imu_ready` | IMU 驱动/服务已初始化 |
| bit1 | `acc_valid` | 加速度有效 |
| bit2 | `gyro_valid` | 角速度有效 |
| bit3 | `rpy_valid` | roll/pitch/yaw 姿态有效 |
| bit4 | `yaw_fused_valid` | yaw 已融合底盘/里程计或可信航向源 |
| bit15 | `imu_fault` | IMU 当前存在故障或数据不可用 |

#### `latest odom:` 字段

| 字段 | 含义 | 单位 / 状态码 |
|---|---|---|
| `stamp` | MCU 端里程计时间戳 | `ms` |
| `pose=[x y yaw]` | odom 坐标系下的底盘局部位姿 | `x/y` 为 `m`，`yaw` 为 `rad` |
| `vel=[vx vy wz]` | base_footprint 坐标系下的底盘速度 | `vx/vy` 为 `m/s`，`wz` 为 `rad/s` |
| `flags` | ODOM 状态标志 | 当前 Pi 端只打印不判定；建议见下表 |
| `reset` | 里程计重置计数 | 每发生一次 MCU 侧里程计清零/重置递增 |

建议的 `MCU_ODOM.status_flags` 约定如下

| bit | 名称 | 含义 |
|---:|---|---|
| bit0 | `odom_ready` | 里程计模块已初始化 |
| bit1 | `pose_valid` | `x/y/yaw` 位姿有效 |
| bit2 | `twist_valid` | `vx/vy/wz` 速度有效 |
| bit3 | `yaw_valid` | yaw 角有效 |
| bit4 | `odom_reset` | 本周期附近发生过里程计重置，配合 `reset` 判断 |
| bit15 | `odom_fault` | 里程计模块故障或数据不可用 |

#### `latest arm:` 字段

| 字段 | 含义 | 单位 / 状态码 |
|---|---|---|
| `stamp` | MCU 端机械臂状态时间戳 | `ms` |
| `q=[q0 q1 q2 q3 q4]` | 五个关节当前角度 | `rad`，由 `urad` 换算 |
| `xyz=[x y z]` | MCU 正运动学计算的末端位置 | `m`，由 `mm` 换算 |
| `flags` | 机械臂状态有效位 | Pi 端会根据 bit1/bit2 决定是否发布对应 ROS 话题 |
| `seq` | 机械臂状态计数 | MCU 端递增 |

`MCU_ARM_STATE.status_flags` 当前代码已经明确使用以下位：

| bit | 名称 | 含义 | Pi 端行为 |
|---:|---|---|---|
| bit0 | `arm_ready` | 机械臂服务已初始化 | 仅用于诊断 |
| bit1 | `joint_valid` | q0~q4 有效 | 为 1 才发布 `/arm/joint_states` |
| bit2 | `fk_valid` | xyz 正解结果有效 | 为 1 才发布 `/arm/fk_position` |

#### `latest status:` 字段

| 字段 | 含义 | 状态码 / 判断方式 |
|---|---|---|
| `stamp` | MCU 状态帧时间戳 | `ms` |
| `app` | MCU 应用状态机主状态 | 见 `app_state` 表 |
| `manual` | MCU 手动子模式 | 见 `manual_mode` 表，仅在 `app=MANUAL` 时重点关注 |
| `ready` | 各模块 ready 位图 | 见 `ready_flags` 表 |
| `online` | 外部控制源/故障在线状态位图 | 见 `online_flags` 表 |
| `fault_src` | 故障来源 | 见 `fault_source` 表 |
| `fault_level` | 故障等级 | 见 `fault_level` 表 |
| `fault_code` | 具体故障码 | `0` 表示无故障；非 0 需要结合 `fault_src/fault_level` 查 MCU 端 |

`app_state` 建议固定为：

| 值 | 名称 | 含义 |
|---:|---|---|
| 0 | `IDLE` | 空闲/待机，未进入手动或自动控制 |
| 1 | `MANUAL` | 人工接管/遥控模式 |
| 2 | `AUTO_PI` | Pi 自动控制模式，普通 `PI_CONTROL` 只应在该状态被 MCU 执行 |
| 3 | `FAULT` | 可恢复故障状态 |
| 4 | `ESTOP` | 急停锁死状态，不能用普通清故障恢复 |
| 5 | `FINISHED` | 自动任务完成或结束态 |

`manual_mode` 建议固定为：

| 值 | 名称 | 含义 |
|---:|---|---|
| 0 | `NONE` | 非手动模式或无子模式 |
| 1 | `CHASSIS_FS` | 遥控器控制底盘 |
| 2 | `ARM_FS` | 遥控器控制机械臂 |
| 3 | `CHASSIS_PC_ARM` | 遥控器控制底盘，PC 主臂关节角控制从臂 |

`ready_flags`：

| bit | 名称 | 含义 |
|---:|---|---|
| bit0 | `chassis_ready` | 底盘服务已初始化 |
| bit1 | `arm_ready` | 机械臂服务已初始化 |
| bit2 | `odom_ready` | 里程计服务已初始化 |
| bit3 | `remote_ready` | 遥控器输入服务已初始化 |
| bit4 | `pc_ready` | PC 通信服务已初始化 |
| bit5 | `pi_ready` | Pi 通信服务已初始化 |

`online_flags`：

| bit | 名称 | 含义 |
|---:|---|---|
| bit0 | `remote_online` | 遥控器当前在线 |
| bit1 | `pc_online` | PC 通信当前在线 |
| bit2 | `pi_online` | Pi 心跳当前在线 |
| bit3 | `has_fault` | 当前存在故障 |
| bit4 | `estop` | 当前处于急停或急停锁存 |

`fault_source` 建议固定为：

| 值 | 名称 | 含义 |
|---:|---|---|
| 0 | `NONE` | 无故障来源 |
| 1 | `CHASSIS` | 底盘执行、驱动、电机或运动学相关故障 |
| 2 | `ARM` | 机械臂执行、舵机、电机或 IK/FK 相关故障 |
| 3 | `ODOM_IMU` | 里程计或 IMU 数据异常 |
| 4 | `REMOTE` | 遥控器离线或输入异常 |
| 5 | `PC_COMMS` | PC 通信异常 |
| 6 | `PI_COMMS` | Pi 通信异常，例如 AutoPi 下心跳超时 |
| 7 | `APP_TASK` | 应用层任务流程异常 |
| 8 | `SAFETY` | 安全策略触发 |
| 255 | `UNKNOWN` | 未分类故障 |

`fault_level` 建议固定为：

| 值 | 名称 | 含义 | 是否通常进入 Fault/EStop |
|---:|---|---|---|
| 0 | `NONE` | 无故障 | 否 |
| 1 | `INFO` | 提示信息，不影响控制 | 否 |
| 2 | `WARN` | 可降级/可忽略警告 | 通常否 |
| 3 | `ERROR` | 可恢复错误，需要进入 `FAULT` | 是，进入 `FAULT` |
| 4 | `FATAL` | 严重安全错误 | 是，进入 `ESTOP` 或保持锁死 |

`fault_code` 是 `int16_t`，建议按范围分配，避免不同模块重复：

| 范围 | 建议归属 |
|---:|---|
| `0` | 无故障 |
| `1 ~ 99` | 应用层/参数/状态机通用错误 |
| `100 ~ 199` | 底盘相关错误 |
| `200 ~ 299` | 机械臂相关错误 |
| `300 ~ 399` | ODOM/IMU 相关错误 |
| `400 ~ 499` | 遥控器相关错误 |
| `500 ~ 599` | PC 通信相关错误 |
| `600 ~ 699` | Pi 通信相关错误 |
| `700 ~ 799` | 自动任务相关错误 |
| `-1` | 未分类或临时错误 |

#### `start sensor event:` 字段

| 字段 | 含义 | 状态码 / 判断方式 |
|---|---|---|
| `stamp` | MCU 事件产生时间 | `ms` |
| `sensor` | 传感器 ID | 见 `sensor_id` 表 |
| `type` | 事件类型 | 见 `event_type` 表 |
| `value` | 事件附加值 | 可作为序号、错误码或启动参数 |
| `seq` | 该事件帧序号 | Pi 回 `PI_ACK` 时会带回该序号 |
| `flags` | 帧标志位 | `0x01` 表示 `NEED_ACK`，Pi 会自动回 ACK |

`sensor_id` 建议固定为：

| 值 | 名称 | 含义 |
|---:|---|---|
| 0 | `ALL` | 所有 Pi 侧传感器 |
| 1 | `LIDAR` | 激光雷达，例如 Livox MID360 |
| 2 | `RGB_CAMERA` | RGB 工业相机 |
| 3 | `DEPTH_CAMERA` | 深度相机，例如 Orbbec |
| 4 | `IMU_EXTERNAL` | Pi 侧外接 IMU，若有 |

`event_type` 建议固定为：

| 值 | 名称 | 含义 |
|---:|---|---|
| 1 | `START_REQUIRED` | MCU 请求 Pi 启动对应传感器 |
| 2 | `STOP_REQUIRED` | MCU 请求 Pi 停止对应传感器 |
| 3 | `RESTART_REQUIRED` | MCU 请求 Pi 重启对应传感器 |
| 4 | `CHECK_REQUIRED` | MCU 请求 Pi 检查对应传感器状态 |

#### `MCU_ACK:` 字段与 ACK 状态码

| 字段 | 含义 |
|---|---|
| `ack_msg_id` | MCU 正在确认的 Pi 消息 ID，例如 `0x41` 表示确认 `PI_YAW_ACTION` |
| `ack_seq` | 被确认消息的发送序号 |
| `code` | MCU 执行或接收结果 |

建议的 ACK `code`：

| 值 | 名称 | 含义 |
|---:|---|---|
| 0 | `OK` | 已接收或已执行 |
| 1 | `REJECTED` | MCU 拒绝执行，通常是权限或安全原因 |
| 2 | `BUSY` | MCU 正忙，本次动作未执行 |
| 3 | `BAD_STATE` | 当前状态不允许该命令，例如非 `AUTO_PI` 下收到普通自动控制动作 |
| 4 | `BAD_PAYLOAD` | payload 参数非法或长度不匹配 |
| 5 | `TIMEOUT` | MCU 等待内部执行结果超时 |
| 6 | `UNSUPPORTED` | 当前固件不支持该命令 |
| 7 | `INTERNAL_ERROR` | MCU 内部执行错误 |

#### `fault event:` 字段

`fault event` 是事件式故障通知，字段含义与 `latest status` 中的 `fault_src/fault_level/fault_code` 相同；区别是：`latest status` 是周期快照，`fault event` 是故障发生时的即时事件；调试时应优先看 `fault event` 发生的时间点，再用后续 `latest status` 判断故障是否仍然存在

> 注意：当前 Pi 端代码只对 `MCU_ARM_STATE.status_flags` 的 bit0~bit2 有硬编码判断；`app_state/manual_mode/fault_source/fault_level/ACK code/sensor_id/event_type/IMU flags/ODOM flags` 应与 STM32 MCU 端宏定义保持一致；如果 MCU 端已经有不同枚举，应以 MCU 端宏定义为准，并同步修改本 README

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

调用前先加载工作区环境并确认服务类型：

```bash
cd ~/chassis-pi-ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 service list
ros2 service type /mcu/set_arm_joints
ros2 interface show mcu_comm_bridge/srv/SetArmJoints
```

### 16.1 使用前提

1. `mcu_comm_bridge_node` 已启动并成功打开 MCU 串口；
2. MCU 日志中的 `pi=1`，表示 Pi 心跳在线；
3. 机械臂目标只在 MCU 的 `AutoPi` 模式下执行；
4. 机械臂服务使用 SI 单位：位置为 `m`，角度为 `rad`，速度为 `rad/s`；
5. `speed_rad_s: 0.0` 表示使用 MCU 默认速度，负速度会被拒绝；
6. `success=true` 只表示命令已经排队或协议帧已经写入串口，不表示机械臂已经到位

### 16.2 机械臂关节角控制

五个关节按 `q0 ~ q4` 顺序给出，单位为 rad：

```bash
ros2 service call /mcu/set_arm_joints \
  mcu_comm_bridge/srv/SetArmJoints \
  "{joints_rad: [0.0, 0.20, -0.30, 0.10, 0.0], speed_rad_s: 1.5}"
```

返回示例：

```text
success: true
message: arm joints command queued for transmission
command_seq: 123
```

`command_seq` 是本次离散机械臂目标的序号；bridge 默认重发 3 帧，但 MCU 对同一个序号只执行一次

使用 MCU 默认速度：

```bash
ros2 service call /mcu/set_arm_joints \
  mcu_comm_bridge/srv/SetArmJoints \
  "{joints_rad: [0.0, 0.20, -0.30, 0.10, 0.0], speed_rad_s: 0.0}"
```

### 16.3 机械臂五维 Pose 控制

Pose 固定定义为：

```text
x、y、z、pitch、yaw
```

其中位置单位为 m，姿态角单位为 rad，参考坐标系为 MCU 机械臂基坐标系：

```bash
ros2 service call /mcu/set_arm_pose \
  mcu_comm_bridge/srv/SetArmPose \
  "{x_m: 0.35, y_m: 0.00, z_m: 0.22, pitch_rad: 0.30, yaw_rad: 0.00, speed_rad_s: 1.2}"
```

该接口是五自由度目标，不包含独立 roll 控制；目标是否可达由 MCU 端 IK、关节限位和机械臂状态决定

### 16.4 机械臂位置控制

只修改末端 `x/y/z`，单位为 m：

```bash
ros2 service call /mcu/set_arm_position \
  mcu_comm_bridge/srv/SetArmPosition \
  "{x_m: 0.30, y_m: 0.05, z_m: 0.18, speed_rad_s: 1.0}"
```

### 16.5 机械臂姿态控制

保持当前末端位置，设置 `pitch/yaw`，单位为 rad：

```bash
ros2 service call /mcu/set_arm_orientation \
  mcu_comm_bridge/srv/SetArmOrientation \
  "{pitch_rad: 0.20, yaw_rad: -0.10, speed_rad_s: 0.8}"
```

该接口不是完整三轴 RPY 控制，只控制当前五自由度机械臂可表达的 `pitch/yaw`

### 16.6 刹车控制

开启刹车锁存：

```bash
ros2 service call /mcu/set_brake \
  std_srvs/srv/SetBool \
  "{data: true}"
```

开启后，bridge 的 50 Hz 控制定时器会持续发送零速度和 `BRAKE_REQUEST`

解除刹车锁存：

```bash
ros2 service call /mcu/set_brake \
  std_srvs/srv/SetBool \
  "{data: false}"
```

解除刹车不会自动产生运动指令；需要重新发布 `/motor_cmd_vel` 才会驱动底盘

### 16.7 Yaw hold

开启 yaw hold：

```bash
ros2 service call /mcu/set_yaw_hold \
  std_srvs/srv/SetBool \
  "{data: true}"
```

关闭 yaw hold：

```bash
ros2 service call /mcu/set_yaw_hold \
  std_srvs/srv/SetBool \
  "{data: false}"
```

### 16.8 设置目标 Yaw

目标单位为 rad，例如约 90°：

```bash
ros2 service call /mcu/set_yaw_target \
  mcu_comm_bridge/srv/SetYawTarget \
  "{yaw_rad: 1.5708}"
```

是否需要先开启 yaw hold，以 MCU 端控制逻辑为准

### 16.9 急停

发送急停，`reason` 是 `uint8` 原因码：

```bash
ros2 service call /mcu/estop \
  mcu_comm_bridge/srv/Estop \
  "{reason: 1}"
```

节点默认按 `repeat_estop_count=3` 连续写入 3 帧急停消息；急停是锁存安全状态，不能通过 `/mcu/set_brake false` 解除；恢复方式由 MCU 的故障/急停状态机决定

### 16.10 检查服务定义和调用结果

查看任意服务的类型：

```bash
ros2 service type /mcu/set_arm_pose
```

查看请求和响应字段：

```bash
ros2 interface show mcu_comm_bridge/srv/SetArmPose
ros2 interface show mcu_comm_bridge/srv/SetArmPosition
ros2 interface show mcu_comm_bridge/srv/SetArmOrientation
ros2 interface show mcu_comm_bridge/srv/SetYawTarget
ros2 interface show mcu_comm_bridge/srv/Estop
ros2 interface show std_srvs/srv/SetBool
```

检查 bridge 的发送统计：

```bash
ros2 topic echo /arm/joint_states
ros2 topic echo /arm/fk_position
```

同时观察节点日志中的：

```text
TX ctrl
TX arm
TX arm_retry
arm_accept
arm_reject
tx_yaw
tx_estop
tx_fail
```

机械臂服务返回成功但机械臂未动作时，依次检查：

1. MCU 是否处于 `AutoPi`；
2. MCU 是否收到新的 `arm_command_seq`；
3. 机械臂是否 ready；
4. 目标是否超出工作空间或关节限位；
5. MCU 是否报告 IK 无解或执行错误

---

## 17. 推荐联调顺序

1. 检查 `/dev/mcu_uart` 是否存在
2. 启动 `mcu_comm_bridge`
3. 检查 `/imu`、`/odom`、`/arm/joint_states`、`/arm/fk_position`
4. 检查 `odom -> base_footprint` TF
5. 发布 `/motor_cmd_vel` 验证 `PI_CONTROL`
6. 切换到 `AutoPi` 后，依次测试 `/mcu/set_arm_joints`、`/mcu/set_arm_position`、`/mcu/set_arm_pose`、`/mcu/set_arm_orientation`
7. 调用 `/mcu/set_brake`、`/mcu/set_yaw_hold`、`/mcu/set_yaw_target`、`/mcu/estop`
8. 启动 competition_fsm
9. 启动完整导航系统

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

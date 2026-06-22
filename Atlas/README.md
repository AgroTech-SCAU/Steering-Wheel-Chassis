<div align="center">

# Atlas

</div>

> `Atlas` 是 AgroTech 协会新一代中型轮式机器人平台  
> 当前目录用于统一管理 Atlas 的 MCU 底盘控制代码、树莓派 Pi 端通信桥、ROS2 导航系统、仿真环境和真机启动链路

---

## 1. 平台定位

`Atlas` 面向中型舵轮机器人平台，目标是形成一套可用于比赛、实训和后续研究的整车软件系统

当前软件链路分为三层：

```text
ROS2 导航系统 / 状态机 / 上层任务
        ↓
树莓派 5 chassis-pi-ws 通信桥
        ↓
MCU 底盘实时控制程序
        ↓
舵轮底盘 / IMU / 电机 / 编码器
```

其中：

- MCU 端负责底盘实时控制、IMU 与里程计数据组织、Pi 通信协议解析、AutoPi 状态权限判断
- Pi 端负责 MCU 通信桥接，把 MCU 数据转换为 ROS2 话题，把 ROS2 控制命令转换为 MCU 协议帧
- ROS2 导航端负责雷达驱动、建图、定位、路径规划、速度输出和任务状态机协同

---

## 2. 目录结构

```text
Atlas/
├─ chassis_control_code/
│  ├─ docs/
│  │  └─ comms_protocol.md      # MCU 与 Pi 的通信协议说明
│  └─ src/                      # MCU 端底盘控制代码
│
├─ chassis-pi-ws/
│  ├─ docs/
│  │  └─ comms_protocol.md      # Pi 端使用的协议说明
│  ├─ src/
│  │  └─ mcu_comm_bridge/       # Pi 端 ROS2 通信桥包
│  └─ README.md
│
├─ navigation_system/
│  ├─ at_nav2/                  # Nav2 导航栈与 Cartographer 纯定位启动
│  ├─ lslidar_driver/           # LSLIDAR N10P 雷达驱动
│  ├─ lslidar_msgs/             # 雷达消息与服务接口
│  ├─ robot_cartographer_mapping/# Cartographer 建图包
│  ├─ robot_description/        # URDF 模型与 TF 发布
│  ├─ robot_gazebo/             # Gazebo 仿真环境
│  ├─ robot_startup/            # 真机 / 仿真总启动入口
│  └─ README.md
│
└─ README.md
```

---

## 3. 系统功能

### 3.1 MCU 底盘控制

`chassis_control_code` 是烧录到 MCU 控制板中的底层控制程序

它主要负责：

- 接收 Pi 端心跳与控制帧
- 解析 `PI_CONTROL`、`PI_YAW_ACTION`、`PI_ESTOP` 等命令
- 在 AutoPi 状态下执行底盘速度控制
- 非 AutoPi 状态下拒绝或忽略 Pi 普通控制输入
- 周期性发送 `MCU_IMU`、`MCU_ODOM` 和 `MCU_STATUS`
- 维护通信 fresh timeout、急停、故障和状态机权限边界

当前关键 MCU -> Pi 数据帧：

```text
MCU_IMU   0x25   100Hz   acc_x/y/z + gyro_x/y/z + roll/pitch/yaw
MCU_ODOM  0x26   50Hz    x/y/yaw + vx/vy/wz
MCU_STATUS        5~10Hz  app_state / ready_flags / online_flags / fault
```

当前关键 Pi -> MCU 控制帧：

```text
PI_HEARTBEAT  0x30
PI_CONTROL    0x31
PI_YAW_ACTION 0x41
PI_ESTOP      0x43
PI_ACK        0x44
```

---

### 3.2 Pi 端底盘通信桥

`chassis-pi-ws` 运行在树莓派 5 上，是 ROS2 导航系统与 MCU 底盘控制程序之间的通信桥

它主要负责：

- 通过串口连接 MCU
- 解析 MCU 二进制协议帧
- 将 `MCU_ODOM` 发布为 `/odom`
- 将 `MCU_IMU` 发布为 `/imu`
- 根据 MCU 里程计发布 `odom -> base_footprint` TF
- 订阅状态机仲裁后的 `/motor_cmd_vel`
- 将 `/motor_cmd_vel` 转换为 `PI_CONTROL` 并周期性下发给 MCU
- 通过 service 下发刹车、急停、yaw hold、yaw target 等一次性底盘命令

Pi 端对外提供的主要 ROS2 接口：

| 接口 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/odom` | `nav_msgs/msg/Odometry` | 发布 | 底盘局部里程计，供 Cartographer 和 Nav2 使用 |
| `/imu` | `sensor_msgs/msg/Imu` | 发布 | MCU IMU 与融合姿态数据，供调试和后续融合使用 |
| `odom -> base_footprint` | TF | 发布 | 底盘局部 TF |
| `/motor_cmd_vel` | `geometry_msgs/msg/Twist` | 订阅 | 状态机仲裁后的底盘速度指令 |
| `/mcu/set_brake` | `std_srvs/srv/SetBool` | service | 设置或解除底盘刹车锁存 |
| `/mcu/estop` | `mcu_comm_bridge/srv/Estop` | service | 向 MCU 发送急停事件 |
| `/mcu/set_yaw_hold` | `std_srvs/srv/SetBool` | service | 开启或关闭 MCU 侧 yaw hold |
| `/mcu/set_yaw_target` | `mcu_comm_bridge/srv/SetYawTarget` | service | 设置 MCU 侧目标 yaw |

---

### 3.3 ROS2 导航系统

`navigation_system` 是 Atlas 的 ROS2 导航工作区，基于 ROS2 Humble、Cartographer 2D 和 Nav2 构建

它主要负责：

- 启动 LSLIDAR N10P 雷达驱动并发布 `/scan`
- 启动 URDF 模型和 `robot_state_publisher`
- 使用 Cartographer 进行 2D 建图或纯定位
- 使用 Nav2 完成全局规划和局部控制
- 通过 `competition_fsm` 对 Nav2 输出速度进行任务级仲裁
- 将最终速度指令发布到 `/motor_cmd_vel`

典型导航控制链路：

```text
LSLIDAR
  ↓ /scan
Cartographer / Nav2

MCU
  ↓ MCU_ODOM
chassis-pi-ws
  ↓ /odom + odom -> base_footprint
Cartographer / Nav2

Nav2 controller_server
  ↓ /cmd_vel
competition_fsm
  ↓ /motor_cmd_vel
chassis-pi-ws
  ↓ PI_CONTROL
MCU
```

---

## 4. 环境要求

推荐运行环境：

```text
树莓派 5
Ubuntu 22.04
ROS2 Humble
```

建议安装的 ROS2 组件：

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-desktop \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 \
  libpcap-dev \
  libpcl-dev
```

如果是第一次使用 `rosdep`：

```bash
sudo rosdep init
rosdep update
```

---

## 5. 编译

进入 ROS2 工作区：

```bash
cd ~/atlas_ws
```

安装依赖：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

编译：

```bash
colcon build --symlink-install
```

加载环境：

```bash
source install/setup.bash
```

如果修改了 service 文件，例如 `Estop.srv` 或 `SetYawTarget.srv`，建议清理后重新编译：

```bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

---

## 6. MCU 与 Pi 串口配置

当前建议优先使用 USB 虚拟串口或 USB-TTL 模块，而不是树莓派 GPIO UART

常见设备名：

```text
/dev/ttyACM0   # STM32 USB CDC 常见设备名
/dev/ttyUSB0   # USB-TTL 常见设备名
/dev/mcu_uart  # 建议通过 udev 固定后的设备名
```

建议为 MCU 串口创建固定软链接：

```bash
sudo nano /etc/udev/rules.d/99-mcu-uart.rules
```

示例规则：

```text
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="mcu_uart", GROUP="dialout", MODE="0660"
```

重新加载规则：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

将当前用户加入串口权限组：

```bash
sudo usermod -aG dialout $USER
```

重新登录或重启后检查：

```bash
ls -l /dev/mcu_uart
groups
```

---

## 7. Pi 端桥接节点使用方法

单独启动 MCU 通信桥：

```bash
source ~/atlas_ws/install/setup.bash
ros2 launch mcu_comm_bridge mcu_comm_bridge.launch.py
```

临时指定串口：

```bash
ros2 run mcu_comm_bridge mcu_comm_bridge_node --ros-args \
  -p port:=/dev/ttyACM0 \
  -p baudrate:=921600
```

推荐配置文件位置：

```text
chassis-pi-ws/src/mcu_comm_bridge/config/mcu_comm_bridge.yaml
```

常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `port` | `/dev/ttyACM0` | MCU 串口设备 |
| `baudrate` | `921600` | 串口波特率 |
| `odom_topic` | `/odom` | 里程计话题 |
| `imu_topic` | `/imu` | IMU 话题 |
| `cmd_vel_topic` | `/motor_cmd_vel` | 底盘速度输入话题 |
| `odom_frame_id` | `odom` | odom 坐标系 |
| `base_frame_id` | `base_footprint` | 底盘基坐标系 |
| `imu_frame_id` | `imu_link` | IMU 坐标系 |
| `publish_tf` | `true` | 是否发布 `odom -> base_footprint` |
| `control_rate_hz` | `50.0` | PI_CONTROL 下发频率 |
| `cmd_vel_timeout_ms` | `200` | 底盘速度指令超时时间 |

---

## 8. 导航系统使用方法

### 8.1 仿真启动

启动 Gazebo 仿真：

```bash
source ~/atlas_ws/install/setup.bash
ros2 launch robot_gazebo gazebo_sim.launch.py
```

启动仿真导航：

```bash
source ~/atlas_ws/install/setup.bash
ros2 launch at_nav2 at_nav_gazebo.launch.py
```

### 8.2 建图

启动仿真或真机传感器后，运行 Cartographer 建图：

```bash
source ~/atlas_ws/install/setup.bash
ros2 launch robot_cartographer_mapping robot_cartographer_mapping_gazebo.launch.py
```

保存 Cartographer 状态：

```bash
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '$(pwd)/map.pbstream'}"
```

导出 Nav2 使用的地图：

```bash
ros2 run nav2_map_server map_saver_cli -t map -f map
```

### 8.3 真机启动

真机启动前应确认：

- MCU 已烧录并上电
- Pi 能打开 MCU 串口
- 雷达设备已连接
- 地图文件已放置到 `at_nav2/maps/`
- `competition_fsm` 与 `mission_manager` 已在工作区中可用
- MCU 已进入允许 Pi 控制的 AutoPi 状态

启动整车系统：

```bash
source ~/atlas_ws/install/setup.bash
ros2 launch robot_startup robot_start.launch.py
```

如果总启动文件尚未纳入 `mcu_comm_bridge`，需要单独启动：

```bash
source ~/atlas_ws/install/setup.bash
ros2 launch mcu_comm_bridge mcu_comm_bridge.launch.py
```

---

## 9. 联调检查

### 9.1 检查话题

```bash
ros2 topic list
```

至少应看到：

```text
/scan
/odom
/imu
/cmd_vel
/motor_cmd_vel
/tf
/tf_static
```

检查频率：

```bash
ros2 topic hz /odom
ros2 topic hz /imu
ros2 topic hz /scan
```

期望：

```text
/odom 约 50Hz
/imu 约 100Hz
/scan 按雷达配置输出
```

### 9.2 检查 TF

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link laser
```

完整 TF 链路应为：

```text
map -> odom -> base_footprint -> base_link -> laser
```

### 9.3 检查底盘控制下发

手动发布速度：

```bash
ros2 topic pub /motor_cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

如果 MCU 已进入 AutoPi 状态，底盘应执行对应速度

### 9.4 检查 service

刹车：

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

设置目标 yaw：

```bash
ros2 service call /mcu/set_yaw_target mcu_comm_bridge/srv/SetYawTarget "{yaw_rad: 1.57}"
```

急停：

```bash
ros2 service call /mcu/estop mcu_comm_bridge/srv/Estop "{reason: 1}"
```

---

## 10. 当前注意事项

- `chassis-pi-ws` 默认订阅 `/motor_cmd_vel`，不是 `/cmd_vel`
- `/cmd_vel` 应先由 `competition_fsm` 仲裁，再输出 `/motor_cmd_vel`
- MCU 是否执行 Pi 下发速度，最终由 MCU 状态机决定
- 普通底盘速度属于周期性控制，使用 topic
- 刹车、急停、yaw hold、yaw target 属于一次性命令，使用 service
- 当前底盘部分没有需要持续反馈进度的长期动作，暂不使用 action
- `/imu` 当前主要用于调试和后续融合，当前导航闭环优先依赖 `/odom`
- `odom -> base_footprint` 应由 Pi 端桥接节点发布
- `map -> odom` 应由 Cartographer 纯定位发布

---

## 11. TODO

### 11.1 启动与集成

- [ ] 将 `mcu_comm_bridge` 正式加入 `robot_startup/launch/robot_start.launch.py`
- [ ] 确认 `competition_fsm` 和 `mission_manager` 的源码位置、接口和启动顺序
- [ ] 明确 `/cmd_vel`、`/cmd_vel_smoothed`、`/motor_cmd_vel` 三者的使用关系
- [ ] 确认真机启动时 `/scan`、`/odom`、`/tf` 均已在 Nav2 启动前可用

### 11.2 地图与定位

- [ ] 补齐真实场地 `map.pbstream`
- [ ] 补齐真实场地 `map.yaml` 和 `map.pgm`
- [ ] 修正 `at_nav.launch.py` 中地图文件和 RViz 文件路径
- [ ] 建立真实场地建图、保存、纯定位启动的标准流程

### 11.3 Pi 端通信桥

- [ ] 给 `SerialPort::write_all()` 增加超时保护，避免 USB 串口异常时忙等或阻塞
- [ ] 显式补充 `geometry_msgs` 相关 include
- [ ] 发布 `/mcu/status` 或 `diagnostic_msgs/DiagnosticArray`
- [ ] 将 MCU 时间戳与 ROS 时间戳的同步策略文档化
- [ ] 增加 rosbag 记录建议和通信统计说明

### 11.4 机器人模型与 TF

- [ ] 在 URDF 中补充 `imu_link`
- [ ] 明确 `base_link -> imu_link` 的安装位姿
- [ ] 确认 `laser` / `laser_link` 命名与雷达驱动 `frame_id` 完全一致
- [ ] 检查 `base_footprint -> base_link` 与实际底盘高度是否一致

### 11.5 MCU 端

- [ ] 补充 MCU 固件编译和烧录说明
- [ ] 补充 MCU 状态机说明，尤其是 AutoPi 进入条件
- [ ] 补充 `MCU_IMU`、`MCU_ODOM`、`PI_CONTROL` 的实测频率记录
- [ ] 补充 MCU 故障、急停、Pi 离线处理策略说明

### 11.6 系统联调

- [ ] 建立最小闭环验收表：`/odom`、`/imu`、`/scan`、TF、`/motor_cmd_vel`、MCU 执行
- [ ] 建立真机低速测试流程
- [ ] 建立急停和刹车测试流程
- [ ] 建立 Nav2 目标点导航测试流程
- [ ] 建立常见问题排查文档

---

## 12. 总结

`Atlas` 当前已经具备 MCU 底盘控制、Pi 端通信桥和 ROS2 导航系统三条主线

它的最小目标是打通：

```text
MCU -> /odom -> Cartographer / Nav2 -> /cmd_vel -> /motor_cmd_vel -> MCU
```

在完成启动整合、地图路径修正、FSM 包确认和若干稳定性补强后，Atlas 可以进入真机导航闭环联调阶段

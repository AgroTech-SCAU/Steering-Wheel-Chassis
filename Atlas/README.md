<div align="center">

# Atlas

</div>

> `Atlas` 是 AgroTech 协会新一代中型轮式机器人平台  
> 当前目录用于统一管理 Atlas 的 MCU 底盘控制代码、PC 主臂遥操作脚本、树莓派 Pi 端通信桥、ROS2 导航系统，以及仿真与真机联调链路

---

## 1. 平台定位

`Atlas` 面向中型轮式机器人平台，目标是形成一套可用于比赛、实验和后续研究的整车软件系统

当前软件链路可以分为四层：

```text
PC 主臂 teleop / 上位机调试
        ->
ROS2 导航系统 / 状态机 / 上层任务
        ->
树莓派 5 chassis-pi-ws 通信桥
        ->
MCU 底盘实时控制程序
        ->
轮式底盘 / IMU / 电机 / 编码器 / 机械臂
```

其中：

- MCU 端负责底盘实时控制、IMU 与里程计组织、PC/Pi 协议解析、应用状态机与安全边界控制
- Pi 端负责把 MCU 数据转换为 ROS2 话题，并把 ROS2 控制命令转换为 MCU 协议帧
- ROS2 端负责雷达驱动、建图、定位、路径规划、速度输出以及导航链路编排
- PC 端负责主臂遥操作输入，并通过串口向 MCU 下发 `PC_MASTER_JOINTS` 与 `PC_HEARTBEAT`

---

## 2. 目录结构

```text
Atlas/
├── chassis_control_code/
│   ├── arm_description/           # Atlas 底盘与机械臂模型
│   ├── src/                       # MCU 端底层控制代码
│   └── robot.ioc                  # STM32CubeMX 工程
│
├── chassis-pi-ws/
│   ├── src/
│   │   └── mcu_comm_bridge/       # Pi 端 ROS2 <-> MCU 通信桥
│   └── README.md
│
├── chassis-pc-ws/
│   ├── scripts/
│   │   └── teleop.py              # PC 主臂遥操作脚本
│   └── README.md
│
├── navigation_system/
│   ├── at_nav2/                   # Nav2 导航栈与 Cartographer 纯定位启动
│   ├── lslidar_driver/            # LSLIDAR N10P 雷达驱动
│   ├── lslidar_msgs/              # 雷达消息与服务接口
│   ├── robot_cartographer_mapping/# Cartographer 建图包
│   ├── robot_description/         # URDF 模型与 TF 发布
│   ├── robot_gazebo/              # Gazebo 仿真环境
│   ├── robot_startup/             # 真机 / 仿真总启动入口
│   └── README.md
│
├── docs/
│   └── comms_protocol.md          # PC / Pi / MCU 统一通信协议说明
│
└── README.md
```

---

## 3. 系统功能

### 3.1 MCU 底盘控制

`chassis_control_code` 是烧录到 MCU 控制板中的底层控制程序

它主要负责：

- 接收 PC 端 `PC_HEARTBEAT`、`PC_MASTER_JOINTS`
- 接收 Pi 端 `PI_CONTROL`、`PI_ARM_ACTION`、`PI_YAW_ACTION`、`PI_ESTOP`、`PI_ACK`
- 执行底盘速度控制、机械臂控制、yaw hold 与任务事件消费
- 周期性发布 `MCU_IMU`、`MCU_ODOM`、`MCU_ARM_STATE`、`MCU_STATUS`
- 维护 PC/Pi 在线状态、fresh timeout、急停、故障与应用状态机权限边界

当前关键 MCU -> 上位机数据帧：

```text
MCU_STATUS      0x21   5~10Hz   app_state / ready_flags / online_flags / fault
MCU_IMU         0x25   100Hz    acc_x/y/z + gyro_x/y/z + roll/pitch/yaw
MCU_ODOM        0x26   50Hz     x/y/yaw + vx/vy/wz
MCU_ARM_STATE   0x27   50Hz     q0~q4 + pose(xyz + quat xyzw) + status_flags
```

当前关键上位机 -> MCU 控制帧：

```text
PC_HEARTBEAT     0x10
PC_MASTER_JOINTS 0x11
PI_HEARTBEAT     0x30
PI_CONTROL       0x31
PI_ARM_ACTION    0x40
PI_YAW_ACTION    0x41
PI_MISSION_EVENT 0x42
PI_ESTOP         0x43
PI_ACK           0x44
```

---

### 3.2 Pi 端底盘通信桥

`chassis-pi-ws` 运行在树莓派 5 上，是 ROS2 导航系统和 MCU 底盘控制程序之间的通信桥

它主要负责：

- 通过串口连接 MCU
- 解析 MCU 二进制协议帧
- 将 `MCU_ODOM` 发布为 `/odom`
- 将 `MCU_IMU` 发布为 `/imu`
- 将 `MCU_ARM_STATE` 发布为 `/arm/joint_states`、`/arm/pose` 与 `/arm/pose_position`
- 根据 MCU 里程计发布 `odom -> base_footprint` TF
- 订阅状态机仲裁后的 `/motor_cmd_vel`
- 将 `/motor_cmd_vel` 转换为 `PI_CONTROL` 并周期性下发给 MCU
- 通过 service 下发刹车、急停、yaw hold、yaw target 等一次性命令

Pi 端对外提供的主要 ROS2 接口：

| 接口 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/odom` | `nav_msgs/msg/Odometry` | 发布 | 底盘局部里程计，供 Cartographer 和 Nav2 使用 |
| `/imu` | `sensor_msgs/msg/Imu` | 发布 | MCU IMU 与融合姿态数据 |
| `/arm/joint_states` | `sensor_msgs/msg/JointState` | 发布 | 机械臂 q0~q4 当前关节角 |
| `/arm/pose` | `geometry_msgs/msg/PoseStamped` | 发布 | MCU 正运动学求得的末端位姿，四元数顺序为 `x/y/z/w` |
| `/arm/pose_position` | `geometry_msgs/msg/PointStamped` | 发布 | 机械臂末端位置话题，与 `/arm/pose` 的 `pose.position` 同源 |
| `odom -> base_footprint` | TF | 发布 | 底盘局部 TF |
| `/motor_cmd_vel` | `geometry_msgs/msg/Twist` | 订阅 | 状态机仲裁后的底盘速度指令 |
| `/mcu/set_brake` | `std_srvs/srv/SetBool` | service | 设置或解除底盘刹车 |
| `/mcu/estop` | `mcu_comm_bridge/srv/Estop` | service | 向 MCU 发送急停事件 |
| `/mcu/set_yaw_hold` | `std_srvs/srv/SetBool` | service | 开启或关闭 MCU 侧 yaw hold |
| `/mcu/set_yaw_target` | `mcu_comm_bridge/srv/SetYawTarget` | service | 设置 MCU 侧目标 yaw |

---

### 3.3 PC 主臂遥操作链路

`chassis-pc-ws` 用于运行 PC 端主臂遥操作脚本，目前以单脚本方式工作

它主要负责：

- 连接主臂 Dynamixel 串口
- 周期性读取主臂 q0~q4 与末端开关状态
- 将主臂角度打包为 `PC_MASTER_JOINTS`
- 向 MCU 周期性发送 `PC_HEARTBEAT`
- 为机械臂手动跟随、调试和主从控制提供输入链路

典型 PC -> MCU 遥操作链路：

```text
PC teleop
  -> PC_MASTER_JOINTS / PC_HEARTBEAT
MCU
  -> app_fsm / manual mode permission check
机械臂执行层
```

---

### 3.4 ROS2 导航系统

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
  -> /scan
Cartographer / Nav2

MCU
  -> MCU_ODOM
chassis-pi-ws
  -> /odom + odom -> base_footprint
Cartographer / Nav2

Nav2 controller_server
  -> /cmd_vel
competition_fsm
  -> /motor_cmd_vel
chassis-pi-ws
  -> PI_CONTROL
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

Atlas 当前包含多个子工作区，通常按模块分别编译

### 5.1 Pi 端工作区

```bash
cd ~/chassis-pi-ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

如果修改了 service 文件，例如 `Estop.srv` 或 `SetYawTarget.srv`，建议清理后重新编译：

```bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

### 5.2 导航工作区

```bash
cd ~/AT_Atlas_nav_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 5.3 MCU 工程

`chassis_control_code` 为 STM32 工程，通常通过 STM32CubeMX / EIDE / 对应工具链进行编译与烧录

---

## 6. MCU 串口与协议配置

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

完整协议说明见：

```text
docs/comms_protocol.md
```

---

## 7. Pi 端桥接节点使用方法

单独启动 MCU 通信桥：

```bash
source ~/chassis-pi-ws/install/setup.bash
ros2 launch mcu_comm_bridge mcu_comm_bridge.launch.py
```

临时指定串口：

```bash
ros2 run mcu_comm_bridge mcu_comm_bridge_node --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baudrate:=1000000
```

推荐配置文件位置：

```text
chassis-pi-ws/src/mcu_comm_bridge/config/mcu_comm_bridge.yaml
```

常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `port` | `/dev/ttyUSB0` | MCU 串口设备 |
| `baudrate` | `1000000` | 串口波特率 |
| `odom_topic` | `/odom` | 里程计话题 |
| `imu_topic` | `/imu` | IMU 话题 |
| `cmd_vel_topic` | `/motor_cmd_vel` | 底盘速度输入话题 |
| `arm_joint_state_topic` | `/arm/joint_states` | 机械臂关节状态话题 |
| `arm_pose_topic` | `/arm/pose` | 机械臂末端位姿话题 |
| `arm_pose_position_topic` | `/arm/pose_position` | 机械臂末端位置话题 |
| `odom_frame_id` | `odom` | odom 坐标系 |
| `base_frame_id` | `base_footprint` | 底盘基座坐标系 |
| `imu_frame_id` | `imu_link` | IMU 坐标系 |
| `arm_frame_id` | `arm_base_link` | 机械臂参考坐标系 |
| `publish_tf` | `true` | 是否发布 `odom -> base_footprint` |
| `control_rate_hz` | `50.0` | `PI_CONTROL` 下发频率 |
| `cmd_vel_timeout_ms` | `200` | 底盘速度指令超时时间 |

---

## 8. PC 端遥操作使用方法

PC 端说明见：

```text
chassis-pc-ws/README.md
```

典型运行方式：

```bash
python3 teleop.py \
  --leader-port /dev/ttyUSB0 \
  --mcu-port /dev/ttyUSB1 \
  --freq 50
```

仅调试主臂输入、不向 MCU 发包：

```bash
python3 teleop.py \
  --leader-port /dev/ttyUSB0 \
  --dry-run \
  --print-rate 10
```

使用前建议先确认：

- 主臂 Dynamixel 端口可正常打开
- MCU 串口与波特率匹配
- `PC_MASTER_JOINTS` 的 payload 长度与 MCU 解析实现一致

---

## 9. 导航系统使用方法

### 9.1 仿真启动

启动 Gazebo 仿真：

```bash
source ~/AT_Atlas_nav_ws/install/setup.bash
ros2 launch robot_gazebo gazebo_sim.launch.py
```

启动仿真导航：

```bash
source ~/AT_Atlas_nav_ws/install/setup.bash
ros2 launch at_nav2 at_nav_gazebo.launch.py
```

### 9.2 建图

启动仿真或真机传感器后，运行 Cartographer 建图：

```bash
source ~/AT_Atlas_nav_ws/install/setup.bash
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

### 9.3 真机启动

真机启动前应确认：

- MCU 已烧录并上电
- Pi 能打开 MCU 串口
- 雷达设备已连接
- 地图文件已放到 `at_nav2/maps/`
- `competition_fsm` 与 `mission_manager` 在工作区中可用
- MCU 已进入允许 Pi 控制的 `AutoPi` 状态

启动整车系统：

```bash
source ~/AT_Atlas_nav_ws/install/setup.bash
ros2 launch robot_startup robot_start.launch.py
```

如果总启动文件尚未纳入 `mcu_comm_bridge`，仍需要单独启动：

```bash
source ~/chassis-pi-ws/install/setup.bash
ros2 launch mcu_comm_bridge mcu_comm_bridge.launch.py
```

---

## 10. 联调检查

### 10.1 检查话题

```bash
ros2 topic list
```

至少应看到：

```text
/scan
/odom
/imu
/arm/joint_states
/arm/pose
/arm/pose_position
/cmd_vel
/motor_cmd_vel
/tf
/tf_static
```

检查频率：

```bash
ros2 topic hz /odom
ros2 topic hz /imu
ros2 topic hz /arm/joint_states
ros2 topic hz /arm/pose
ros2 topic hz /arm/pose_position
ros2 topic hz /scan
```

期望：

```text
/odom 约 50Hz
/imu 约 100Hz
/arm/joint_states 约 50Hz
/arm/pose 约 50Hz
/arm/pose_position 约 50Hz
/scan 按雷达配置输出
```

### 10.2 检查 TF

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link laser_link
```

完整 TF 链路应为：

```text
map -> odom -> base_footprint -> base_link -> laser_link
```

### 10.3 检查底盘控制下发

手动发布速度：

```bash
ros2 topic pub /motor_cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

如果 MCU 已进入 `AutoPi` 状态，底盘应执行对应速度

### 10.4 检查机械臂状态

```bash
ros2 topic echo /arm/joint_states
ros2 topic echo /arm/pose
ros2 topic echo /arm/pose_position
```

如果没有数据，优先检查 MCU 是否已发送 `MCU_ARM_STATE(0x27)`

### 10.5 检查 service

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

## 11. 当前注意事项

- `chassis-pi-ws` 默认订阅 `/motor_cmd_vel`，不是 `/cmd_vel`
- `/cmd_vel` 应先由 `competition_fsm` 仲裁，再输出 `/motor_cmd_vel`
- MCU 是否执行 Pi 下发速度，最终由 MCU 状态机决定
- 普通底盘速度属于周期性控制，使用 topic + `PI_CONTROL`
- 刹车、急停、yaw hold、yaw target 属于一次性命令，使用 service
- `MCU_ARM_STATE` 已纳入协议，但真机联调时仍需确认其发布频率与字段有效位
- `odom -> base_footprint` 应由 Pi 端桥接节点发布
- `map -> odom` 应由 Cartographer 纯定位或建图节点发布
- PC 主臂输入是否生效，最终由 MCU 手动模式与权限状态决定

---

## 12. TODO

### 12.1 启动与集成

- [ ] 将 `mcu_comm_bridge` 正式纳入 `robot_startup/launch/robot_start.launch.py`
- [ ] 确认 `competition_fsm` 和 `mission_manager` 的源码位置、接口和启动顺序
- [ ] 明确 `/cmd_vel`、`/motor_cmd_vel` 与底盘执行层之间的完整链路
- [ ] 确认真机启动时 `/scan`、`/odom`、`/tf` 均已在 Nav2 启动前可用

### 12.2 地图与定位

- [ ] 补齐真实场地 `map.pbstream`
- [ ] 补齐真实场地 `map.yaml` 和 `map.pgm`
- [ ] 修正 `at_nav.launch.py` 中地图文件和 RViz 文件路径
- [ ] 建立真实场地建图、保存、纯定位启动的标准流程

### 12.3 Pi 端通信桥

- [ ] 为 `SerialPort::write_all()` 增加超时保护，避免 USB 串口异常时忙等或阻塞
- [ ] 发布 `/mcu/status` 或 `diagnostic_msgs/DiagnosticArray`
- [ ] 补充 `MCU_ACK`、`MCU_STATUS` 的 ROS2 可观测性输出
- [ ] 将 MCU 时间戳与 ROS 时间戳的同步策略文档化
- [ ] 增加 rosbag 记录建议和通信统计说明

### 12.4 PC 端遥操作

- [ ] 明确 `teleop.py` 与文档中的参数、默认端口、协议版本是否完全一致
- [ ] 补充主臂关节方向、零位与夹爪阈值的标定流程
- [ ] 统一 `PC_MASTER_JOINTS` 的 payload 变更说明与 MCU 解析实现

### 12.5 机器人模型与 TF

- [ ] 在 URDF 中补全 `imu_link`
- [ ] 明确 `base_link -> imu_link` 的安装位姿
- [ ] 确认 `laser_link` 命名与雷达驱动 `frame_id` 完全一致
- [ ] 检查 `base_footprint -> base_link` 与实际底盘高度是否一致

### 12.6 MCU 端

- [ ] 补充 MCU 固件编译和烧录说明
- [ ] 补充 MCU 状态机说明，尤其是 `AutoPi` 与手动模式的进入条件
- [ ] 补充 `MCU_IMU`、`MCU_ODOM`、`MCU_ARM_STATE`、`PI_CONTROL` 的实测频率记录
- [ ] 补充 MCU 故障、急停、PC/Pi 离线处理策略说明

### 12.7 系统联调

- [ ] 建立最小闭环验收表：`/odom`、`/imu`、`/scan`、TF、`/motor_cmd_vel`、MCU 执行
- [ ] 建立真机低速测试流程
- [ ] 建立急停和刹车测试流程
- [ ] 建立 Nav2 目标点导航测试流程
- [ ] 建立常见问题排查文档

---

## 13. 总结

`Atlas` 当前已经具备 MCU 底盘控制、PC 主臂遥操作、Pi 端通信桥和 ROS2 导航系统四条主线

它的最小闭环目标是打通：

```text
MCU -> /odom -> Cartographer / Nav2 -> /cmd_vel -> /motor_cmd_vel -> MCU
```

同时补齐：

```text
PC teleop -> PC_MASTER_JOINTS -> MCU -> 机械臂执行
```

在完成启动整合、地图路径修正、FSM 包确认、通信观测性补强以及真机联调后，`Atlas` 就可以进入更稳定的整车闭环验证阶段

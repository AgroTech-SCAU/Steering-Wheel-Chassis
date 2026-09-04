# Atlas

Atlas 是 AgroTech 协会的中型轮式机器人平台；这个目录统一管理整车 MCU 控制程序、Pi 端 ROS2 自主任务工作区、PC 主臂遥操作脚本、ASRPro 语音链路和整车通信协议文档

当前主线是智械争锋全自主区：MCU 侧触发 AutoPi，Pi 端运行 YASMIN 比赛状态机，完成 A/B 场地识别、语义导航、视觉抓取、园区放置和 DONE/FAIL 上报

## Quick Start

### 1. 整车角色分工

| 端 | 目录 | 作用 |
|---|---|---|
| MCU | `chassis_control_code/` | 底盘实时控制、机械臂执行、AutoPi 状态、安全边界和串口协议 |
| Pi | `chassis-pi-ws/` | ROS2 Humble、MCU bridge、导航、视觉、YASMIN 比赛状态机 |
| PC | `chassis-pc-ws/` | 主臂遥操作和上位机调试 |
| ASRPro | `atlas_asrpro/` | 语音触发和播报相关链路 |

正式自主比赛入口在 Pi 端：

```text
Atlas/chassis-pi-ws/src/app/atlas_competition_bringup/config/competition.yaml
Atlas/chassis-pi-ws/src/app/atlas_competition_bringup/launch/competition_stack.launch.py
```

### 2. MCU 侧准备

MCU 工程位于：

```text
Atlas/chassis_control_code/
```

使用 STM32CubeMX、EIDE 或当前工程配置的工具链编译烧录上电后先确认：

- MCU 固件已烧录
- Pi 可以打开 MCU 串口，例如 `/dev/ttyACM0`、`/dev/ttyUSB0` 或固定软链接 `/dev/mcu_uart`
- MCU 能输出状态、里程计、IMU 和机械臂状态帧
- AutoPi 由 MCU 侧条件触发，Pi 端不主动强行切换 MCU 模式

### 3. PC 遥操作准备

PC 端目录：

```text
Atlas/chassis-pc-ws/
```

典型运行方式：

```bash
cd Atlas/chassis-pc-ws/scripts
python3 teleop.py --leader-port /dev/ttyUSB0 --mcu-port /dev/ttyUSB1 --freq 50
```

PC 遥操作用于主臂手动跟随和调试，不是自主比赛的主入口

### 4. Pi 端环境和编译

Pi 端目录：

```text
Atlas/chassis-pi-ws/
```

安装依赖：

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-yaml \
  python3-opencv \
  python3-numpy \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2
```

编译：

```bash
cd Atlas/chassis-pi-ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 5. 放入 A/B 半场地图

需要准备两套独立半场地图资源；导航 backend 会在视觉判断出 A/B 后，只启动对应半场的地图和定位栈

```text
arena_A.yaml
arena_A.pgm 或 arena_A.png
arena_A.pbstream

arena_B.yaml
arena_B.pgm 或 arena_B.png
arena_B.pbstream
```

地图可以放在仓库内，例如：

```text
Atlas/chassis-pi-ws/src/nav_system/at_nav2/maps/
```

也可以放在外部目录，然后在 `competition.yaml` 中使用绝对路径引用

### 6. 配置唯一比赛 YAML

正式比赛优先只改这一份：

```text
Atlas/chassis-pi-ws/src/app/atlas_competition_bringup/config/competition.yaml
```

必须填入并打开对应安全门：

| 配置段 | 内容 |
|---|---|
| `competition.navigation.arenas.A` | A 半场 `map`、`pbstream`、`pickup / park_1 / park_2` 坐标 |
| `competition.navigation.arenas.B` | B 半场 `map`、`pbstream`、`pickup / park_1 / park_2` 坐标 |
| `competition.vision.sorting_scan_a/b` | 机械臂判断 A/B 的两个视觉扫描位姿 |
| `competition.vision.sorting_rule` | `park_1_roi`、`park_2_roi` 分拣标识 ROI |
| `competition.manipulation.placement` | `park_1 / park_2` 放置基准位姿、层高、slot 偏移 |

未实测字段保持：

```yaml
configured: false
enabled: false
```

地图路径为空、waypoint 未配置、扫描位姿未配置、ROI 未启用或放置未启用时，backend 会拒绝执行，不会把 0 默认值当作真实目标

### 7. 启动整场比赛栈

```bash
cd Atlas/chassis-pi-ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch atlas_competition_bringup competition_stack.launch.py
```

使用外部比赛配置：

```bash
ros2 launch atlas_competition_bringup competition_stack.launch.py \
  competition_config:=/path/to/competition.yaml
```

常用联调参数：

```bash
ros2 launch atlas_competition_bringup competition_stack.launch.py --show-args

ros2 launch atlas_competition_bringup competition_stack.launch.py \
  no_preview:=true

ros2 launch atlas_competition_bringup competition_stack.launch.py \
  enable_navigation:=false \
  enable_vision:=false \
  enable_manipulation:=false \
  enable_mission:=false
```

### 8. 状态检查

```bash
ros2 topic echo /mcu/status
ros2 topic echo /atlas/mission/status
ros2 topic echo /atlas/navigation/status
ros2 topic echo /atlas/manipulation/status
ros2 topic hz /odom
ros2 topic hz /scan
```

关键服务：

```bash
ros2 service list | grep atlas
ros2 service list | grep move_to_sorting
ros2 service list | grep mcu
```

### 9. 整车执行链路

```text
ASRPro / 遥控器 / MCU 条件触发
  ↓
MCU 进入 AutoPi 并向 Pi 发布自动任务事件
  ↓
Pi 端 atlas_mission_yasmin 启动比赛任务
  ↓
vision backend 调用 handeye_bridge 扫描 sorting_scan_A / sorting_scan_B
  ↓
识别 arena=A/B 和 park_1/park_2 的货物映射
  ↓
atlas_nav_full_backend 锁定 A/B 并启动对应半场 map/pbstream
  ↓
YASMIN 只发送 pickup / park_1 / park_2 语义点
  ↓
导航到 pickup，视觉定位并抓取
  ↓
按分拣规则导航到 park_1 或 park_2
  ↓
机械臂按 placement 配置放置
  ↓
8 件完成后通过 MCU 上报 DONE；异常时上报 FAIL
```

## 目录结构

```text
Atlas/
├── atlas_asrpro/                  # ASRPro 语音链路
├── chassis_control_code/          # MCU 底盘与机械臂控制工程
├── chassis-pc-ws/                 # PC 主臂遥操作脚本
├── chassis-pi-ws/                 # Pi 端 ROS2 自主任务工作区
├── docs/                          # 整车通信协议和说明文档
└── README.md
```

Pi 端核心目录：

```text
chassis-pi-ws/src/
├── app/
│   ├── atlas_competition_bringup/
│   ├── atlas_competition_config/
│   ├── atlas_competition_manipulation_backend/
│   ├── atlas_competition_vision_backend/
│   ├── atlas_mission_interfaces/
│   └── atlas_mission_yasmin/
├── mcu_comm_bridge/
├── nav_system/
│   ├── at_nav2/
│   ├── atlas_nav_full_backend/
│   ├── atlas_nav_pseudo_backend/
│   └── robot_startup/
└── vision_system/
    ├── handeye_bridge/
    ├── vison_topic/
    └── vison_topic_interfaces/
```

## 关键模块

| 模块 | 作用 |
|---|---|
| `chassis_control_code` | MCU 实时控制、状态机、安全边界和串口协议 |
| `chassis-pc-ws` | PC 主臂遥操作输入和调试 |
| `atlas_asrpro` | 语音触发与播报链路 |
| `mcu_comm_bridge` | MCU 与 ROS2 的串口桥接，发布 `/odom`、`/imu`、机械臂状态并接收控制命令 |
| `atlas_mission_yasmin` | 比赛任务状态机，负责 8 件货物循环、分类规则应用和任务编排 |
| `atlas_competition_bringup` | 正式比赛统一启动入口，安装顶层 `competition.yaml` |
| `atlas_competition_config` | 顶层 YAML 解析、A/B arena 锁定和语义 waypoint 解析 |
| `atlas_nav_full_backend` | Nav2 完整导航后端，按 `arena + waypoint_id` 解析真实 map 坐标 |
| `at_nav2` | Cartographer 纯定位、map_server 和 Nav2 bringup |
| `handeye_bridge` | 检测像素到机械臂坐标转换、抓取目标发布和视觉扫描位姿控制 |
| `vison_topic` | ONNX 目标检测服务，包名保持现有拼写 |
| `atlas_competition_vision_backend` | 识别 A/B 与分拣规则，提供比赛视觉服务 |
| `atlas_competition_manipulation_backend` | 执行观察、抓取和放置动作 |

## 主要接口

MCU 与任务触发：

```text
/mcu/status
/mcu/auto_task_event
/mcu/report_mission_result
/mcu/estop
```

底盘与传感器：

```text
/odom
/imu
/scan
/tf
/tf_static
/motor_cmd_vel
```

导航：

```text
/atlas/navigation/start
/atlas/navigation/cancel
/atlas/navigation/status
/atlas/navigation/cmd_vel
```

视觉和手眼：

```text
/vision_detect
/detection_centers
/pick_target
/move_to_initial_pose
/move_to_sorting_scan_a
/move_to_sorting_scan_b
/initial_pose_ready
/vision_pose_ready
/atlas/vision/classify_sorting_rule
/atlas/vision/detect_target
```

机械臂：

```text
/arm/joint_states
/arm/pose
/arm/pose_position
/atlas/manipulation/start
/atlas/manipulation/cancel
/atlas/manipulation/status
/mcu/set_arm_pose
/mcu/set_arm_position
/mcu/set_suction
```

## A/B 地图与导航配置

A/B 两套半场地图是必须的；`atlas_nav_full_backend` 在第一次有效导航请求时锁定 arena，并用该 arena 的 `map` 和 `pbstream` 启动导航栈

`competition.yaml` 示例：

```yaml
competition:
  navigation:
    coordinate_mode: absolute_map
    arenas:
      A:
        map: "/path/to/arena_A.yaml"
        pbstream: "/path/to/arena_A.pbstream"
        waypoints:
          pickup: {x: 1.0, y: 0.0, yaw: 0.0, configured: true}
          park_1: {x: 2.0, y: 0.4, yaw: 1.57, configured: true}
          park_2: {x: 2.0, y: -0.4, yaw: -1.57, configured: true}
      B:
        map: "/path/to/arena_B.yaml"
        pbstream: "/path/to/arena_B.pbstream"
        waypoints:
          pickup: {x: 1.0, y: 0.0, yaw: 0.0, configured: true}
          park_1: {x: 2.0, y: 0.4, yaw: 1.57, configured: true}
          park_2: {x: 2.0, y: -0.4, yaw: -1.57, configured: true}
```

相对路径会按 `competition.yaml` 所在目录解析；绝对路径原样使用

## MCU 串口配置

Pi 端 MCU 通信配置文件：

```text
Atlas/chassis-pi-ws/src/mcu_comm_bridge/config/mcu_comm_bridge.yaml
```

常见串口设备：

```text
/dev/ttyACM0
/dev/ttyUSB0
/dev/mcu_uart
```

建议通过 udev 固定软链接：

```text
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="mcu_uart", GROUP="dialout", MODE="0660"
```

应用规则并检查权限：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout $USER
ls -l /dev/mcu_uart
```

通信协议说明见：

```text
Atlas/docs/comms_protocol.md
```

## 实车配置顺序

1. 烧录 MCU，并确认 Pi 串口连接正常
2. 启动 `mcu_comm_bridge`，确认 `/mcu/status`、`/odom`、`/imu`、`/arm/pose` 有数据
3. 启动雷达，确认 `/scan` 有数据且 TF 链路正确
4. 建 A/B 两套半场地图，保存各自 `.yaml`、地图图片和 `.pbstream`
5. 在 A/B 地图中实测 `pickup`、`park_1`、`park_2` 的 `x / y / yaw`
6. 实测 `sorting_scan_a`、`sorting_scan_b` 机械臂观察位姿
7. 配置并验证分拣标识 ROI
8. 实测 `park_1`、`park_2` 放置基准位姿、层高和四个 slot 偏移
9. 逐项把对应 `configured` 或 `enabled` 改为 `true`，再启动整场比赛栈

## 验证命令

构建 Pi 端工作区：

```bash
cd Atlas/chassis-pi-ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

构建比赛相关包：

```bash
colcon build --packages-select \
  atlas_competition_config \
  atlas_nav_full_backend \
  atlas_competition_vision_backend \
  atlas_competition_manipulation_backend \
  handeye_bridge \
  atlas_competition_bringup
```

运行核心测试：

```bash
export PYTHONPATH=$PWD/src/app/atlas_competition_config:$PYTHONPATH
export PYTHONPATH=$PWD/src/app/atlas_competition_vision_backend:$PYTHONPATH
export PYTHONPATH=$PWD/src/app/atlas_competition_manipulation_backend:$PYTHONPATH
export PYTHONPATH=$PWD/src/nav_system/atlas_nav_full_backend:$PYTHONPATH
export PYTHONPATH=$PWD/src/vision_system/handeye_bridge:$PYTHONPATH

python3 -m pytest \
  src/app/atlas_competition_config/test/test_config.py \
  src/app/atlas_competition_vision_backend/test/test_backend.py \
  src/app/atlas_competition_manipulation_backend/test/test_manipulation_config.py \
  src/nav_system/atlas_nav_full_backend/test/test_competition_navigation.py \
  src/vision_system/handeye_bridge/test/test_vision_pose_gate.py
```

解析比赛 launch 参数：

```bash
ROS_LOG_DIR=/tmp/atlas_ros_log \
ros2 launch atlas_competition_bringup competition_stack.launch.py --show-args
```

## 安全原则

- Pi 不主动请求 MCU 进入 AutoPi；自动任务由 MCU 侧条件触发
- Nav2 速度先进入 `/atlas/navigation/cmd_vel`，再经任务安全门控输出到 `/motor_cmd_vel`
- 顶层 `competition.yaml` 默认保持安全拒绝状态
- 未确认地图、坐标、机械臂位姿、ROI 或放置点之前，不开启对应 `configured/enabled`
- A/B arena 一场比赛只锁定一次，不允许运行中切换半场地图
- 急停、刹车、串口离线和 MCU 状态异常优先由 MCU/Pi 安全链处理

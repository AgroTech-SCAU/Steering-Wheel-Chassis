# Atlas Pi Workspace

`chassis-pi-ws` 是 Atlas 机器人 Pi 端比赛工作区；当前主线是“智械争锋全自主区”：MCU 触发自动任务，Pi 端运行 YASMIN 任务状态机，完成 A/B 场地识别、语义导航、视觉抓取、园区放置和结果上报

系统的正式配置入口是一份顶层 YAML：

```text
src/app/atlas_competition_bringup/config/competition.yaml
```

实车准备完成后，正常情况下只需要改这份 YAML，就能把 A/B 半场地图、导航点、视觉扫描位姿、分拣 ROI 和放置位姿接入整场任务

## Quick Start

### 1. 进入工作区并加载 ROS

```bash
cd ~/chassis-pi-ws
source /opt/ros/humble/setup.bash
```

### 2. 配置顶层比赛 YAML

正式比赛优先只改：

```text
src/app/atlas_competition_bringup/config/competition.yaml
```

需要填入的实测内容：

| 配置段 | 必填内容 |
|---|---|
| `competition.navigation.arenas.A` | A 半场 `map`、`pbstream`、`pickup / park_1 / park_2` 坐标 |
| `competition.navigation.arenas.B` | B 半场 `map`、`pbstream`、`pickup / park_1 / park_2` 坐标 |
| `competition.vision.sorting_scan_a/b` | 机械臂用于判断 A/B 的两个视觉扫描位姿 |
| `competition.vision.sorting_rule` | `park_1_roi`、`park_2_roi` 分拣标识 ROI |
| `competition.manipulation.placement` | `park_1 / park_2` 放置基准位姿、层高、slot 偏移 |

所有未实测字段保持：

```yaml
configured: false
enabled: false
```

地图路径为空、waypoint 未配置、扫描位姿未配置、ROI 未启用或放置未启用时，backend 会拒绝执行，不会把 0 默认值当作真实目标

### 3. 编译

```bash
colcon build --symlink-install
source install/setup.bash
```

### 4. 启动整场比赛栈

使用包内默认顶层配置：

```bash
ros2 launch atlas_competition_bringup competition_stack.launch.py
```

使用外部配置文件：

```bash
ros2 launch atlas_competition_bringup competition_stack.launch.py \
  competition_config:=/path/to/competition.yaml
```

### 5. 常用联调方式

查看启动参数：

```bash
ros2 launch atlas_competition_bringup competition_stack.launch.py --show-args
```

关闭 OpenCV 预览：

```bash
ros2 launch atlas_competition_bringup competition_stack.launch.py \
  no_preview:=true
```

按模块关闭：

```bash
ros2 launch atlas_competition_bringup competition_stack.launch.py \
  enable_navigation:=false \
  enable_vision:=false \
  enable_manipulation:=false \
  enable_mission:=false
```

只联调导航 backend：

```bash
ros2 launch atlas_nav_full_backend full_nav_backend.launch.py \
  competition_config:=/path/to/competition.yaml
```

只联调视觉和手眼：

```bash
ros2 launch handeye_bridge screw_pick.launch.py \
  competition_config:=/path/to/competition.yaml
```

### 6. 查看运行状态

```bash
ros2 topic echo /mcu/status
ros2 topic echo /atlas/mission/status
ros2 topic echo /atlas/navigation/status
ros2 topic echo /atlas/manipulation/status
```

关键服务：

```bash
ros2 service list | grep atlas
ros2 service list | grep move_to_sorting
```

## 比赛执行流程

```text
遥控器/MCU 触发 AutoPi
  ↓
atlas_mission_yasmin 启动任务
  ↓
视觉 backend 调用 handeye 扫描 sorting_scan_A / sorting_scan_B
  ↓
识别 arena=A/B 和 park_1/park_2 对应货物类型
  ↓
导航 backend 锁定 A/B，并启动对应半场 map/pbstream 的 Nav2 栈
  ↓
YASMIN 只发送 pickup / park_1 / park_2 语义点
  ↓
到 pickup 后观察、抓取
  ↓
按分拣规则到 park_1 或 park_2
  ↓
机械臂按 placement 配置放置
  ↓
循环直到 8 件货物完成
  ↓
通过 MCU 上报 DONE；异常时上报 FAIL
```

YASMIN 不保存物理坐标，不直接管理 YOLO、像素、Cartographer 或 Nav2 进程；物理坐标和地图资源属于 navigation backend；视觉扫描和 ROI 属于 vision/handeye；放置坐标属于 manipulation backend

## 顶层配置说明

### 导航配置

导航使用两套独立半场地图资源：

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

路径可以写绝对路径；如果写相对路径，会按 `competition.yaml` 所在目录解析

首次有效导航请求会锁定本场 arena；锁定后如果又收到另一个 arena 的请求，导航 backend 会拒绝，防止比赛中途切错地图

### 视觉配置

`sorting_scan_a` 和 `sorting_scan_b` 是两套实测机械臂观察位姿，用于判断当前半场：

```yaml
competition:
  vision:
    sorting_scan_a:
      configured: true
      x_m: 0.30
      y_m: 0.10
      z_m: 0.35
      pitch_rad: -3.06
      yaw_rad: -3.11
      speed_rad_s: 0.5
    sorting_scan_b:
      configured: true
      x_m: 0.30
      y_m: -0.10
      z_m: 0.35
      pitch_rad: -3.06
      yaw_rad: -3.11
      speed_rad_s: 0.5
    sorting_rule:
      enabled: true
      park_1_roi: [10, 20, 110, 120]
      park_2_roi: [130, 20, 230, 120]
```

检测流程先尝试 `sorting_scan_a`，能稳定识别分拣标识则认为 arena=A；否则尝试 `sorting_scan_b`，成功则 arena=B

### 机械臂放置配置

放置位姿是“底盘已经导航到对应 park 语义点后”的机械臂基座坐标：

```yaml
competition:
  manipulation:
    placement:
      enabled: true
      approach_m: 0.060
      layer_step_m: 0.050
      park_1:
        x_m: 0.31
        y_m: 0.11
        first_layer_z_m: 0.06
      park_2:
        x_m: 0.42
        y_m: -0.12
        first_layer_z_m: 0.07
      slot_offsets_xy_m: [0.0, 0.0, 0.05, 0.0, 0.05, 0.05, 0.0, 0.05]
```

`layer_step_m` 用于按当前已有层数计算下一层释放高度；`slot_offsets_xy_m` 是四个槽位相对 park 基准点的 xy 偏移，格式为 `[x0,y0,x1,y1,x2,y2,x3,y3]`

## 功能包总览

| 功能包 | 作用 |
|---|---|
| `mcu_comm_bridge` | 串口协议桥接，发布 MCU 状态、里程计和机械臂状态，提供机械臂控制和任务结果上报服务 |
| `atlas_mission_interfaces` | 定义任务层消息和服务 |
| `atlas_mission_yasmin` | 智械争锋比赛任务状态机，负责 8 件货物循环、分类规则应用和任务编排 |
| `atlas_competition_bringup` | 正式比赛统一启动入口，安装顶层 `competition.yaml` |
| `atlas_competition_config` | 共享顶层 YAML 解析、A/B arena 锁定和 semantic waypoint 解析 |
| `atlas_nav_full_backend` | Nav2 完整导航后端，按 `arena + waypoint_id` 解析真实 map 坐标并启动对应半场地图 |
| `atlas_nav_pseudo_backend` | 伪导航后端，用于无完整导航栈时的安全联调 |
| `at_nav2` | Cartographer 纯定位、map_server 和 Nav2 bringup |
| `vison_topic` | ONNX 视觉检测服务，包名保持现有拼写 |
| `handeye_bridge` | 检测像素、手眼变换、抓取目标和视觉扫描位姿控制 |
| `atlas_competition_vision_backend` | 比赛视觉 backend，识别 A/B 和分拣规则，提供目标检测服务 |
| `atlas_competition_manipulation_backend` | 比赛机械臂 backend，执行观察、抓取和放置动作 |

## 主要话题和服务

MCU 与任务触发：

```text
/mcu/status
/mcu/auto_task_event
/mcu/report_mission_result
/mcu/estop
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
/atlas/manipulation/start
/atlas/manipulation/cancel
/atlas/manipulation/status
/mcu/set_arm_pose
/mcu/set_arm_position
/mcu/set_suction
```

## 验证命令

构建受影响比赛栈：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  atlas_competition_config \
  atlas_nav_full_backend \
  atlas_competition_vision_backend \
  atlas_competition_manipulation_backend \
  handeye_bridge \
  atlas_competition_bringup
```

运行核心单元测试：

```bash
source /opt/ros/humble/setup.bash
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

沙箱或无写权限环境下解析 launch 时，ROS 日志目录可能需要指到 `/tmp`：

```bash
ROS_LOG_DIR=/tmp/atlas_ros_log \
ros2 launch atlas_competition_bringup competition_stack.launch.py --show-args
```

## 实车配置顺序

1. 先确认 `mcu_comm_bridge` 能稳定发布 `/mcu/status`、`/odom`、`/arm/pose`
2. 建 A/B 两套半场地图，得到各自的 `.yaml`、地图图片和 `.pbstream`
3. 在 A/B 地图中实测 `pickup`、`park_1`、`park_2` 的 `x / y / yaw`
4. 实测 `sorting_scan_a`、`sorting_scan_b` 机械臂观察位姿
5. 配置并验证 `sorting_rule.park_1_roi` 和 `park_2_roi`
6. 实测 `park_1`、`park_2` 放置基准位姿、层高和四个 slot 偏移
7. 逐项把对应 `configured` 或 `enabled` 改为 `true`，再启动整栈

## 安全原则

- Pi 不主动请求 MCU 进入 AutoPi；自动任务由 MCU 侧条件触发
- 速度输出先进入 `/atlas/navigation/cmd_vel`，再由任务安全门控输出到底盘
- 顶层 YAML 默认值全部是安全拒绝状态
- 未确认地图、坐标、机械臂位姿、ROI 或放置点之前，不开启对应 `configured/enabled`
- A/B arena 一场比赛只锁定一次，不允许运行中切换半场地图

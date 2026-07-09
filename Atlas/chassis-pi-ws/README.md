# chassis-pi-ws

本工作区用于 Pi 端自动任务系统，核心目标是把 MCU 状态，伪导航，视觉识别，手眼变换，机械臂授粉序列连接成一条可配置的自动任务链路

当前默认任务链路为

```text
遥控器触发自动任务
  ↓
MCU 进入 AutoPi 并通过 /mcu/status 通知 Pi
  ↓
atlas_mission_manager 接收 START
  ↓
读取 mission_route.yaml
  ↓
调用伪导航后端移动到点位
  ↓
调用视觉授粉后端执行预识别，视觉识别，预授粉，授粉，回退
  ↓
全部点位完成后上报 DONE
```

当前已实现

```text
总任务状态机
伪导航后端
视觉授粉后端
视觉目标 service
手眼变换
可配置地图点位
可配置预识别动作
可配置工具点偏移
可配置授粉动作序列
```

当前暂未实现

```text
完整导航后端
视觉只选择动作序列后端
动态避障
```

---

## 一，功能包总览

```text
src/
├── mcu_comm_bridge
├── atlas_mission_interfaces
├── atlas_mission_manager
├── atlas_nav_pseudo_backend
├── atlas_vision_pollination_backend
└── handeye_calibration_tool
```

| 功能包 | 作用 |
|---|---|
| `mcu_comm_bridge` | 串口协议桥接，发布 MCU 状态，里程计，机械臂状态，提供机械臂和任务结果服务 |
| `atlas_mission_interfaces` | 定义任务层消息和 service |
| `atlas_mission_manager` | 总任务状态机，后端选择，安全门控，DONE 和 FAIL 上报 |
| `atlas_nav_pseudo_backend` | 伪导航后端，使用 `/odom` 做任务相对点位移动 |
| `atlas_vision_pollination_backend` | 视觉目标 service，手眼变换，预授粉和授粉动作序列 |
| `handeye_calibration_tool` | 手眼标定辅助工具，生成视觉授粉所需的手眼参数 |

---

## 二，总任务流程

### 1，启动条件

Pi 不主动请求 MCU 进入 AutoPi

自动任务启动由 MCU 根据遥控器输入，Pi 在线状态，底盘 ready，odom ready 等条件决定

Pi 端只监听

```text
/mcu/status
/mcu/auto_task_event
```

当看到

```text
app_state = AutoPi
auto_start_latched = true
```

并且 START 尚未消费时，`atlas_mission_manager` 才启动一轮任务

### 2，任务执行

每个点位执行顺序为

```text
读取当前 waypoint
  ↓
调用导航后端移动到 waypoint
  ↓
导航后端到点并稳定
  ↓
如果 arrival_task = noop，进入下一个点
  ↓
如果 arrival_task = visual_pollination，调用视觉授粉后端
  ↓
视觉授粉后端执行完整授粉序列
  ↓
进入下一个点
```

### 3，任务结束

全部点位完成后，`atlas_mission_manager` 调用

```text
/mcu/report_mission_result
```

上报

```text
DONE
```

如果导航，视觉，机械臂任一阶段失败，则上报

```text
FAIL
```

### 4，任务复位

下一轮任务必须等 MCU 清除 `auto_start_latched`

Pi 收到 RESET 后会清理本地任务上下文，取消导航和视觉授粉后端，停止底盘速度输出

---

## 三，编译与启动

安装依赖

```bash
sudo apt update
sudo apt install -y python3-yaml python3-opencv python3-numpy libyaml-cpp-dev
```

编译

```bash
cd ~/chassis-pi-ws
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select \
  mcu_comm_bridge \
  atlas_mission_interfaces \
  atlas_nav_pseudo_backend \
  atlas_vision_pollination_backend \
  atlas_mission_manager \
  handeye_calibration_tool \
  --symlink-install

source install/setup.bash
```

启动串口桥

```bash
ros2 launch mcu_comm_bridge mcu_comm_bridge.launch.py
```

启动完整任务栈

```bash
ros2 launch atlas_mission_manager mission_stack.launch.py
```

查看状态

```bash
ros2 topic echo /atlas/mission/status
ros2 topic echo /atlas/navigation/status
ros2 topic echo /atlas/manipulation/status
```

---

## 四，核心配置文件

| 文件 | 内容 |
|---|---|
| `src/atlas_mission_manager/config/mission_manager.yaml` | 总状态机参数，后端服务名，安全停止参数 |
| `src/atlas_mission_manager/config/mission_route.yaml` | 地图点位，点位顺序，预识别动作名称，到点任务名称 |
| `src/atlas_nav_pseudo_backend/config/pseudo_nav.yaml` | 伪导航控制参数，速度限制，到点阈值，超时 |
| `src/atlas_vision_pollination_backend/config/pollination_actions.yaml` | 预识别关节位姿，工具点偏移，授粉序列 |
| `src/atlas_vision_pollination_backend/config/pollination.yaml` | 视觉授粉后端参数，手眼参数，等待阈值 |
| `src/atlas_vision_pollination_backend/config/camera_target.yaml` | 相机和目标检测参数 |
| `src/mcu_comm_bridge/config/mcu_comm_bridge.yaml` | 串口和 ROS 话题参数 |

---

## 五，如何配置地图点位

地图点位写在

```text
src/atlas_mission_manager/config/mission_route.yaml
```

每个点位格式如下

```yaml
- id: "area_a_02_down"
  x: 0.71
  y: -0.08
  yaw: 0.00
  area: "AREA_A"
  timeout_s: 20.0
  prepare_action: "pre_detect_nav_02"
  arrival_task: "visual_pollination"
```

字段说明

| 字段 | 说明 |
|---|---|
| `id` | 点位名称，需要全局唯一 |
| `x` | 任务相对 x 坐标，单位 m |
| `y` | 任务相对 y 坐标，单位 m |
| `yaw` | 任务相对航向，单位 rad |
| `area` | 区域标签，只用于日志和调试 |
| `timeout_s` | 该点伪导航最大允许时间 |
| `prepare_action` | 到点任务开始前使用的预识别动作名称 |
| `arrival_task` | 到点后任务名称，`noop` 或 `visual_pollination` |

坐标模式默认为

```yaml
coordinate_mode: "task_relative"
```

含义是，任务开始后，伪导航后端把当前 `/odom` 记录为任务原点，后续所有点位都是相对这个原点的坐标

首次测试建议保持

```yaml
max_forward_waypoints: 1
return_home_enabled: false
```

逐步打开方式

```yaml
# 执行前两个前进点
max_forward_waypoints: 2

# 执行全部前进点
max_forward_waypoints: 0

# 开启返航点
return_home_enabled: true
```

过渡点写法

```yaml
prepare_action: "noop"
arrival_task: "noop"
```

作业点写法

```yaml
prepare_action: "pre_detect_nav_02"
arrival_task: "visual_pollination"
```

---

## 六，如何配置预识别动作

预识别动作写在

```text
src/atlas_vision_pollination_backend/config/pollination_actions.yaml
```

格式如下

```yaml
prepare_actions:
  pre_detect_nav_02:
    type: "joints"
    joints_rad: [1.606, 2.315, 5.875, 2.152, 3.141]
    speed_rad_s: 3.14
    timeout_s: 8.0
```

字段说明

| 字段 | 说明 |
|---|---|
| `type` | 当前支持 `noop` 和 `joints` |
| `joints_rad` | 五个关节目标角，单位 rad |
| `speed_rad_s` | 动作速度，单位 rad/s |
| `timeout_s` | 等待到位超时时间，单位 s |

点位如何引用预识别动作

```yaml
prepare_action: "pre_detect_nav_02"
```

视觉授粉后端执行 `visual_pollination` 时，会先执行当前点的 `prepare_action`，然后才触发视觉识别

---

## 七，如何配置工具点和授粉序列

工具点和授粉序列写在

```text
src/atlas_vision_pollination_backend/config/pollination_actions.yaml
```

当前最终授粉工具点为

```yaml
pollination_tool_point_m: [0.05, -0.015, 0.087]
```

它位于

```yaml
arrival_tasks:
  visual_pollination:
    pollination_tool_point_m: [0.05, -0.015, 0.087]
```

预授粉工具点为

```yaml
pre_pollination_tool_point_m: [0.05, -0.015, 0.097]
```

两者含义

| 字段 | 说明 |
|---|---|
| `pre_pollination_tool_point_m` | 预授粉偏移，用于先靠近目标 |
| `pollination_tool_point_m` | 授粉偏移，用于真正接触目标 |

动作序列如下

```yaml
sequence:
  - type: "ensure_prepare_pose"
    name: "到达预识别位姿"

  - type: "visual_position"
    name: "到达预授粉位姿"
    tool_point_ref: "pre_pollination_tool_point_m"

  - type: "visual_position"
    name: "到达授粉位姿"
    tool_point_ref: "pollination_tool_point_m"

  - type: "dwell"
    name: "授粉停留"
    duration_s: 0.3

  - type: "visual_position"
    name: "回到预授粉位姿"
    tool_point_ref: "pre_pollination_tool_point_m"

  - type: "joints_action"
    name: "回到预识别位姿"
    action_ref: "prepare_action"
```

执行逻辑

```text
预识别位姿
  ↓
视觉 service 返回相机坐标目标
  ↓
手眼变换得到 arm_base_link 下的目标点
  ↓
根据 pre_pollination_tool_point_m 计算预授粉末端位置
  ↓
根据 pollination_tool_point_m 计算授粉末端位置
  ↓
停留
  ↓
回到预授粉
  ↓
回到预识别
```

视觉目标的 base 坐标只在预识别位姿下计算一次，后续动作使用同一个目标 base 坐标和检测时刻的工具坐标方向，避免机械臂移动后重复使用过期相机坐标导致误差

---

## 八，如何配置伪导航

伪导航参数写在

```text
src/atlas_nav_pseudo_backend/config/pseudo_nav.yaml
```

常用参数

| 参数 | 说明 |
|---|---|
| `control_rate_hz` | 控制频率 |
| `odom_timeout_s` | `/odom` 超时时间 |
| `waypoint_timeout_s` | 单点最大移动时间 |
| `kp_xy` | 平面位置比例系数 |
| `kp_yaw` | 航向比例系数 |
| `max_linear_speed_m_s` | 最大线速度 |
| `max_angular_speed_rad_s` | 最大角速度 |
| `max_linear_accel_m_s2` | 最大线加速度 |
| `max_angular_accel_rad_s2` | 最大角加速度 |
| `position_tolerance_m` | 到点位置误差阈值 |
| `yaw_tolerance_rad` | 到点航向误差阈值 |
| `settle_duration_s` | 刹车稳定等待时间 |

首次实车建议保持低速

```yaml
max_linear_speed_m_s: 0.20
max_angular_speed_rad_s: 0.40
position_tolerance_m: 0.04
yaw_tolerance_rad: 0.08
```

---

## 九，各功能包说明

### mcu_comm_bridge

负责串口协议桥接，不做任务决策

主要输出

```text
/mcu/status
/mcu/auto_task_event
/odom
/imu
/arm/joint_states
/arm/pose
/arm/pose_position
```

主要输入和服务

```text
/motor_cmd_vel
/mcu/set_arm_joints
/mcu/set_arm_position
/mcu/report_mission_result
/mcu/estop
```

### atlas_mission_interfaces

定义任务层接口

主要接口

```text
MissionStatus.msg
NavigationStatus.msg
ManipulationStatus.msg
StartNavigation.srv
CancelNavigation.srv
StartManipulation.srv
CancelManipulation.srv
DetectCameraTarget.srv
```

### atlas_mission_manager

负责总任务状态机和安全门控

它负责

```text
监听 START 和 RESET
读取 mission_route.yaml
选择 navigation_backend 和 manipulation_backend
按点位调用导航后端
按点位调用视觉授粉后端
发布最终 /motor_cmd_vel
上报 DONE 或 FAIL
```

它不负责

```text
直接计算伪导航速度
直接调用视觉模型
直接计算手眼变换
直接生成授粉末端点
```

### atlas_nav_pseudo_backend

负责伪导航

它负责

```text
接收目标点
读取 /odom
计算任务相对目标
输出 /atlas/navigation/cmd_vel
判断到点和稳定
```

它不直接发布 `/motor_cmd_vel`，最终速度必须经过 `atlas_mission_manager` 安全门控

### atlas_vision_pollination_backend

负责视觉授粉

它负责

```text
提供 /vision/detect_camera_target
调用视觉识别
执行预识别动作
计算手眼变换
计算预授粉和授粉位置
调用 /mcu/set_arm_position
等待机械臂到位
发布 /atlas/manipulation/status
```

### handeye_calibration_tool

负责手眼标定辅助

输出结果用于配置视觉授粉后端中的手眼参数

---

## 十，首次实车联调建议

### 第一步，只验证启动和状态

```yaml
max_forward_waypoints: 1
return_home_enabled: false
```

第一个点保持

```yaml
arrival_task: "noop"
```

观察

```bash
ros2 topic echo /atlas/mission/status
ros2 topic echo /atlas/navigation/status
```

### 第二步，验证伪导航方向

把第一个点设置成小距离移动

```yaml
x: 0.00
y: -0.07
yaw: 0.00
```

确认机器人移动方向和预期一致

### 第三步，打开第一个作业点

```yaml
max_forward_waypoints: 2
```

如果要先验证底盘，不执行授粉，可以把第二个点临时设为

```yaml
arrival_task: "noop"
```

### 第四步，打开视觉授粉

把作业点设置为

```yaml
arrival_task: "visual_pollination"
```

观察

```bash
ros2 topic echo /atlas/manipulation/status
```

### 第五步，逐步打开全部点

```yaml
max_forward_waypoints: 0
```

前进路线稳定后再开启返航

```yaml
return_home_enabled: true
```

---

## 十一，常见问题

### 没有启动任务

检查

```bash
ros2 topic echo /mcu/status
ros2 topic echo /mcu/auto_task_event
```

确认

```text
app_state = AutoPi
auto_start_latched = true
```

### 机器人不动

检查

```bash
ros2 topic echo /atlas/navigation/cmd_vel
ros2 topic echo /motor_cmd_vel
ros2 topic echo /odom
```

如果 `/atlas/navigation/cmd_vel` 有速度但 `/motor_cmd_vel` 没有速度，说明总状态机安全门控没有放行

### 到点后不执行视觉授粉

检查点位配置

```yaml
arrival_task: "visual_pollination"
```

检查服务

```bash
ros2 service list | grep vision
```

### 机械臂不到位

检查

```bash
ros2 topic echo /arm/joint_states
ros2 topic echo /arm/pose_position
```

确认 `pollination_actions.yaml` 中的动作目标在机械臂工作空间内

### 授粉深度不合适

修改

```yaml
pollination_tool_point_m: [0.05, -0.015, 0.087]
```

接触过深就增大 z，接触不到就减小 z

---

## 十二，后续扩展方式

切换完整导航后端时，只需要新增导航后端功能包，并在 `mission_route.yaml` 中修改

```yaml
navigation_backend: "full_nav"
```

切换新的视觉任务后端时，只需要新增 manipulation 后端功能包，并在 `mission_route.yaml` 中修改

```yaml
manipulation_backend: "new_backend_name"
```

总任务状态机只依赖标准 service 和 status 话题，不需要改动主流程

---

## 点位与多任务配置补充

当前路线配置已升级为 `pre_move_action + arrival_jobs` 结构

```text
pre_move_action
  在底盘移动前执行
  用于收臂，安全姿态，或移动到某个预识别位姿

arrival_jobs
  在底盘到点稳定后依次执行
  PASS_BY 点为空列表
  AREA_A 和 AREA_B 每个点默认两个 job
  AREA_C 每个点默认一个 job
```

视觉授粉任务支持 `visual_pollination_multi`

```text
visual_pollination_a_0_3  每个 job 识别 0 到 3 朵雌花
visual_pollination_b_0_3  每个 job 识别 0 到 3 朵雌花
visual_pollination_c_0_2  每个 job 识别 0 到 2 朵雌花
```

每朵花的动作序列在 `pollination_actions.yaml` 的 `per_target_sequence` 中配置

默认顺序为

```text
预识别位姿
→ 预授粉位姿
→ 授粉位姿
→ 授粉停留
→ 回到预授粉位姿
```

每个 job 完成后会执行 `after_all_targets_sequence`，默认回到预识别位姿

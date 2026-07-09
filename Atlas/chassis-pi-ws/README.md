# chassis-pi-ws 工作区说明

本工作区是农业机器人树莓派端任务系统，负责把 mcu 提供的底盘，机械臂，里程计，遥控启动和安全状态，与 pi 端的导航，视觉，手眼变换，动作序列和任务上报连接起来

本说明面向实车联调，重点说明每个包的职责，任务流程，配置方式，启动方式和排错方法

## 一，系统职责边界

mcu 负责

```text
底盘真实执行
机械臂真实执行
遥控器输入和启动仲裁
pc 与 pi 串口通信
系统状态机
急停，故障，安全停车
自动任务启动锁存 auto_start_latched
```

pi 负责

```text
监听 /mcu/status 与 /mcu/auto_task_event
执行总任务状态机
按配置选择导航后端和视觉后端
执行伪导航或未来完整导航
执行视觉识别，手眼变换和授粉动作序列
把任务完成或失败上报给 mcu
```

pc 负责

```text
主臂遥操作
人工示教
人工调试
必要时人工接管
```

重要约束

```text
pi 不主动请求 mcu 进入 AutoPi
全自动启动权只在 mcu
pi 只在 mcu 已进入 AutoPi 且 auto_start_latched=1 时启动本地任务
任务完成或失败后 auto_start_latched 不会自动清零
下一轮任务必须使用遥控器 clear/reset 手势解锁
```

## 二，工作区目录

```text
chassis-pi-ws/
├── README.md
└── src/
    ├── mcu_comm_bridge
    ├── atlas_mission_interfaces
    ├── atlas_mission_manager
    ├── atlas_nav_pseudo_backend
    ├── atlas_vision_pollination_backend
    └── handeye_calibration_tool
```

旧的独立 vision 示例目录已经删除

视觉服务，语音文本，手眼变换和授粉动作序列都放在

```text
src/atlas_vision_pollination_backend
```

## 三，包职责总览

| 包名 | 主要职责 | 不负责的内容 |
|---|---|---|
| `mcu_comm_bridge` | 串口协议桥接，发布 mcu 状态，里程计，机械臂反馈，提供底盘和机械臂控制服务 | 不实现任务流程，不启动导航，不启动视觉 |
| `atlas_mission_interfaces` | 保存任务系统公共消息和服务接口 | 不包含执行逻辑 |
| `atlas_mission_manager` | 总任务状态机，后端选择，速度安全门控，任务结果上报 | 不写具体导航算法，不写视觉模型，不写授粉序列 |
| `atlas_nav_pseudo_backend` | 基于 /odom 的伪导航后端，输出导航速度 | 不直接控制 /motor_cmd_vel，不做地图避障 |
| `atlas_vision_pollination_backend` | 相机目标服务，手眼变换，视觉授粉动作序列 | 不管理 mcu 自动启动，不控制底盘导航 |
| `handeye_calibration_tool` | 单目手眼标定采样，求解，结果保存 | 不参与自动任务运行 |

## 四，完整任务流程

正常自动任务流程如下

```text
遥控器自动启动
  ↓
mcu 接受启动条件
  ↓
mcu 设置 auto_start_latched=1
  ↓
mcu 进入 AutoPi
  ↓
mcu_comm_bridge 发布 /mcu/auto_task_event 的 START
  ↓
atlas_mission_manager 进入 PRECHECK
  ↓
读取 mission_route.yaml
  ↓
按点循环执行
  ↓
调用导航后端 /atlas/navigation/start
  ↓
等待导航后端到点成功
  ↓
调用视觉授粉后端 /atlas/manipulation/start
  ↓
等待视觉授粉后端成功
  ↓
进入下一点或返航点
  ↓
全部完成后调用 /mcu/report_mission_result 上报 DONE
  ↓
mcu 从 AutoPi 进入 Finished
  ↓
等待遥控器 clear/reset 手势
```

中止流程如下

```text
收到 RESET，Manual，Fault，EStop，mcu 状态超时，导航失败，视觉失败
  ↓
atlas_mission_manager 取消导航后端
  ↓
atlas_mission_manager 取消视觉授粉后端
  ↓
atlas_mission_manager 屏蔽非零速度
  ↓
atlas_mission_manager 发布零速并请求刹车
  ↓
必要时上报 FAIL
  ↓
等待 mcu Fault 或等待 RESET
```

## 五，速度链路

底盘速度只有一个最终出口

```text
atlas_nav_pseudo_backend
  ↓ /atlas/navigation/cmd_vel
atlas_mission_manager
  ↓ /motor_cmd_vel
mcu_comm_bridge
  ↓ PI_CONTROL
mcu
```

约束

```text
导航后端只能发布 /atlas/navigation/cmd_vel
只有 atlas_mission_manager 可以发布 /motor_cmd_vel
一旦总状态机不允许运动，manager 会无条件输出零速
RESET，Manual，Fault，EStop 时 manager 会取消后端并请求刹车
```

## 六，视觉授粉链路

当前实现的是完整视觉加部分动作序列后端

```text
atlas_mission_manager
  ↓ /atlas/manipulation/start
atlas_vision_pollination_backend
  ↓ /vision/detect_camera_target
atlas_camera_target_service
  ↓ 返回 camera 坐标目标
atlas_vision_pollination_backend
  ↓ 手眼变换
atlas_vision_pollination_backend
  ↓ /mcu/set_arm_position 与 /mcu/set_arm_joints
mcu_comm_bridge
  ↓ PI_CONTROL.arm
mcu
```

当前动作序列

```text
预识别位姿
  ↓
视觉识别
  ↓
预授粉位姿，工具点 z=0.097
  ↓
授粉位姿
  ↓
停留
  ↓
回到预授粉位姿
  ↓
回到预识别位姿
```

后续如果改为“视觉只负责选择动作序列”，可以新增另一个 manipulation 后端，并复用现有 `/atlas/manipulation/start` 接口

## 七，主要话题和服务

mcu 通信桥输出

```text
/mcu/status
/mcu/auto_task_event
/odom
/imu
/arm/joint_states
/arm/pose
/arm/pose_position
```

mcu 通信桥服务

```text
/mcu/set_brake
/mcu/set_arm_joints
/mcu/set_arm_pose
/mcu/set_arm_position
/mcu/set_arm_orientation
/mcu/set_yaw_hold
/mcu/set_yaw_target
/mcu/report_mission_result
/mcu/estop
```

任务系统接口

```text
/atlas/mission/status
/atlas/navigation/start
/atlas/navigation/cancel
/atlas/navigation/status
/atlas/navigation/cmd_vel
/atlas/manipulation/start
/atlas/manipulation/cancel
/atlas/manipulation/status
/vision/detect_camera_target
```

## 八，配置文件位置

总任务配置

```text
src/atlas_mission_manager/config/mission_manager.yaml
src/atlas_mission_manager/config/mission_route.yaml
```

伪导航配置

```text
src/atlas_nav_pseudo_backend/config/pseudo_nav.yaml
```

视觉授粉配置

```text
src/atlas_vision_pollination_backend/config/camera_target.yaml
src/atlas_vision_pollination_backend/config/pollination.yaml
src/atlas_vision_pollination_backend/config/pollination_actions.yaml
```

通信桥配置

```text
src/mcu_comm_bridge/config/mcu_comm_bridge.yaml
```

手眼标定配置

```text
src/handeye_calibration_tool/config/handeye_tool.yaml
```

## 九，首次实车安全配置

首次联调建议保持

```yaml
max_forward_waypoints: 1
return_home_enabled: false
```

这表示只执行第一个 7 cm 测试点

确认方向，刹车，里程计和遥控 reset 都正常后，再逐步调整

```yaml
max_forward_waypoints: 2
```

最后完整打开全部前进点

```yaml
max_forward_waypoints: 0
```

返航确认安全后再打开

```yaml
return_home_enabled: true
```

## 十，依赖安装

基础依赖

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-base \
  python3-colcon-common-extensions \
  python3-yaml \
  python3-numpy \
  python3-opencv \
  python3-cv-bridge \
  libyaml-cpp-dev
```

视觉模型依赖需要根据实际模型部署方式安装

如果使用 ultralytics

```bash
pip3 install ultralytics
```

语音播报如果使用 espeak

```bash
sudo apt install -y espeak alsa-utils
```

## 十一，编译

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

## 十二，启动

启动 mcu 通信桥

```bash
ros2 launch mcu_comm_bridge mcu_comm_bridge.launch.py
```

启动任务栈

```bash
ros2 launch atlas_mission_manager mission_stack.launch.py
```

单独启动伪导航后端

```bash
ros2 launch atlas_nav_pseudo_backend pseudo_nav.launch.py
```

单独启动视觉授粉后端

```bash
ros2 launch atlas_vision_pollination_backend vision_pollination.launch.py
```

单独启动总任务状态机

```bash
ros2 launch atlas_mission_manager mission_manager.launch.py
```

## 十三，常用查看命令

查看 mcu 状态

```bash
ros2 topic echo /mcu/status \
  --qos-reliability reliable \
  --qos-durability transient_local
```

查看自动任务边沿事件

```bash
ros2 topic echo /mcu/auto_task_event
```

查看总任务状态

```bash
ros2 topic echo /atlas/mission/status
```

查看导航后端状态

```bash
ros2 topic echo /atlas/navigation/status
```

查看视觉授粉后端状态

```bash
ros2 topic echo /atlas/manipulation/status
```

查看导航速度

```bash
ros2 topic echo /atlas/navigation/cmd_vel
```

查看最终给 mcu 的速度

```bash
ros2 topic echo /motor_cmd_vel
```

## 十四，联调顺序

推荐按下面顺序联调

```text
1，启动 mcu_comm_bridge，确认 /mcu/status 正常
2，确认 /odom 频率约 50 Hz
3，确认 /arm/joint_states 和 /arm/pose_position 正常
4，启动 mission_stack
5，先保持 max_forward_waypoints=1，确认 7 cm 测试点方向正确
6，测试移动中 clear/reset，确认立即停车
7，打开 area_a_02_down，确认预识别位姿能执行
8，配置视觉模型路径和相机编号
9，调用 /vision/detect_camera_target 做单次视觉验证
10，打开 visual_pollination，验证手眼变换和授粉序列
11，逐步增加 waypoint 数量
12，最后打开 return_home_enabled
```

## 十五，常见问题

没有 START

```text
检查 mcu 是否 AutoPi
检查 /mcu/status 中 auto_start_latched 是否为 true
检查 pi_online，chassis_ready，odom_ready 是否为 true
检查遥控器启动手势是否先经过非自动状态再进入自动状态
```

机器人不动

```text
检查 /atlas/navigation/cmd_vel 是否有速度
检查 /motor_cmd_vel 是否有速度
如果前者有，后者没有，说明 manager 安全门控不允许运动
检查 mcu 是否仍处于 AutoPi
检查 /mcu/set_brake 是否被持续请求
```

伪导航方向反了

```text
先不要打开多点路线
只用第一个 7 cm 点
检查 /odom 的 x，y，yaw 方向
检查底盘 vx，vy 方向和 mcu 里程计方向是否一致
```

视觉服务失败

```text
检查 camera_target.yaml 中 camera_index 是否正确
检查 model_path 是否存在
检查是否能打开相机
检查 /vision/debug_image 是否有图像
检查 target_camera_m 单位是否为米
```

机械臂到位失败

```text
检查 /arm/joint_states 是否新鲜
检查 /arm/pose_position 是否新鲜
检查目标点是否在机械臂工作空间内
检查 pollination_actions.yaml 中工具点偏移是否合理
检查 mcu 日志中是否有 IK 无解或目标越界
```

## 十一，地图点位配置方法

地图点位统一配置在

```text
src/atlas_mission_manager/config/mission_route.yaml
```

这个文件只描述路线和动作引用，不写具体机械臂关节角，不写视觉模型参数，不写伪导航控制参数

每个前进点使用下面的结构

```yaml
- nav_index: 2
  id: "area_a_02_down"
  x: 0.71
  y: -0.08
  yaw: 0.00
  area: "AREA_A"
  flower_pattern:
    direction: "Y"
    up: false
    mid: false
    down: true
  timeout_s: 20.0
  prepare_action: "pre_detect_nav_02"
  arrival_task: "visual_pollination"
```

字段说明

| 字段 | 作用 |
|---|---|
| `nav_index` | 旧 MCU 路线索引，只用于对照和记录 |
| `id` | 当前 pi 任务系统使用的点位名，必须唯一 |
| `x` | 任务相对坐标 x，单位 m |
| `y` | 任务相对坐标 y，单位 m |
| `yaw` | 任务相对航向角，单位 rad |
| `area` | 区域标签，可写 `PASS_BY`，`AREA_A`，`AREA_B`，`AREA_C` |
| `flower_pattern` | 从旧 MCU 路线迁移过来的目标分布记录，当前主要用于说明和后续视觉策略 |
| `timeout_s` | 伪导航执行该点允许的最长时间 |
| `prepare_action` | 该点到位后视觉识别前使用的预识别机械臂位姿 |
| `arrival_task` | 该点到位后执行的任务 |

点位执行顺序就是 YAML 中 `waypoints` 的书写顺序

重复坐标不能随便合并，例如 `area_a_02_down` 和 `area_a_03_up_mid` 坐标相同，但预识别动作和目标分布不同

首次实车联调建议这样配置

```yaml
max_forward_waypoints: 1
return_home_enabled: false
```

这只执行第一个 7 cm 过渡点，适合确认底盘方向，/odom 方向，刹车和 RESET

验证第一个授粉点时可以改为

```yaml
max_forward_waypoints: 2
return_home_enabled: false
```

完整执行全部前进点时改为

```yaml
max_forward_waypoints: 0
return_home_enabled: false
```

前进点稳定后再打开返航

```yaml
return_home_enabled: true
```

## 十二，预识别动作配置方法

预识别动作统一配置在

```text
src/atlas_vision_pollination_backend/config/pollination_actions.yaml
```

`mission_route.yaml` 中的 `prepare_action` 必须能在 `prepare_actions` 中找到同名动作

示例

```yaml
prepare_actions:
  pre_detect_nav_02:
    type: "joints"
    joints_rad: [1.606, 2.315, 5.875, 2.152, 3.141]
    speed_rad_s: 1.0
    timeout_s: 8.0
```

字段说明

| 字段 | 作用 |
|---|---|
| `type` | 当前支持 `noop` 和 `joints` |
| `joints_rad` | 五个关节目标角，单位 rad |
| `speed_rad_s` | 下发给 mcu 的动作速度，单位 rad/s |
| `timeout_s` | 等待该关节动作到位的最长时间 |

当前已经从 `src(141).zip` 的 `navigation_route.c` 迁移了全部 `pre_detect_joints`

命名规则是

```text
pre_detect_nav_02
pre_detect_nav_03
...
pre_detect_nav_20
```

其中数字对应旧 MCU 的 `nav_index`

如果要修改某个点的预识别位姿，优先只改该点的 `pre_detect_nav_XX`，不要直接改通用视觉授粉序列

## 十三，视觉授粉动作序列配置方法

到位后任务同样配置在

```text
src/atlas_vision_pollination_backend/config/pollination_actions.yaml
```

默认任务名是

```yaml
arrival_task: "visual_pollination"
```

该任务的默认流程是

```text
到达预识别位姿
  ↓
调用 /vision/detect_camera_target
  ↓
用手眼变换计算目标在 arm_base_link 下的位置
  ↓
到达预授粉位姿
  ↓
到达授粉位姿
  ↓
停留
  ↓
回到预授粉位姿
  ↓
回到预识别位姿
```

对应 YAML 是

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

当前支持的步骤类型

| 步骤类型 | 作用 |
|---|---|
| `ensure_prepare_pose` | 执行当前 waypoint 的 `prepare_action` |
| `visual_position` | 根据视觉目标和工具点偏移计算机械臂位置目标 |
| `dwell` | 原地等待一段时间 |
| `joints_action` | 执行某个已定义的关节动作 |

工具点偏移配置

```yaml
pre_pollination_tool_point_m: [0.05, -0.015, 0.097]
pollination_tool_point_m: [0.05, -0.015, 0.087]
```

含义

| 字段 | 作用 |
|---|---|
| `pre_pollination_tool_point_m` | 预授粉工具点偏移，单位 m |
| `pollination_tool_point_m` | 授粉工具点偏移，单位 m |

调参原则

```text
如果授粉动作接触过深，增大 pollination_tool_point_m 的 z
如果授粉动作接触不到，减小 pollination_tool_point_m 的 z
如果预授粉离目标太近，增大 pre_pollination_tool_point_m 的 z
如果预授粉离目标太远，减小 pre_pollination_tool_point_m 的 z
```

重要约束

```text
视觉目标只在预识别位姿下计算一次
预授粉，授粉和回退都复用同一个 target_base
不要让机械臂移动后再拿旧相机坐标重新计算目标
```

## 十四，旧 MCU 纯关节授粉序列配置方法

为了保留 `src(141).zip` 中 `pollen_route.c` 的旧任务流，当前已经把旧关节序列迁移为

```text
legacy_pollination_nav_02
legacy_pollination_nav_03
...
legacy_pollination_nav_20
```

这些任务默认不启用

如果你想让某个点不用视觉伺服，而是临时复刻旧 MCU 纯关节序列，只需要把该点改成

```yaml
arrival_task: "legacy_pollination_nav_02"
```

对应序列会按下面方式执行

```yaml
legacy_pollination_nav_02:
  type: "joint_sequence"
  sequence:
    - type: "joints_action"
      name: "legacy_nav_02_step_01"
      action_ref: "legacy_nav_02_step_01"
```

对应关节动作定义在 `prepare_actions` 里

```yaml
legacy_nav_02_step_01:
  type: "joints"
  joints_rad: [1.485, 3.106, 5.410, 1.930, 3.141]
  speed_rad_s: 1.0
  timeout_s: 8.0
```

建议使用方式

```text
正式路线默认使用 visual_pollination
旧纯关节序列只用于对照，回归测试或视觉暂时不可用时的临时验证
不要同时在一个点中混用 visual_position 和旧固定关节序列，除非已经明确验证安全空间
```


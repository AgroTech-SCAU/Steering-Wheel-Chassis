# atlas_mission_manager 说明

本包是 pi 端总任务状态机

它负责自动任务生命周期，后端选择，速度安全门控，结果上报和恢复等待

它不直接实现伪导航，不直接实现视觉识别，不直接实现授粉动作

## 一，包定位

负责

```text
订阅 /mcu/status
订阅 /mcu/auto_task_event
读取 mission_manager.yaml
读取 mission_route.yaml
根据配置选择导航后端
根据配置选择视觉授粉后端
按 waypoint 顺序调用导航后端
每个点到位后调用视觉授粉后端
把导航速度安全转发到 /motor_cmd_vel
在 RESET，Manual，Fault，EStop，状态超时时取消后端并刹车
通过 /mcu/report_mission_result 上报 DONE 或 FAIL
等待 mcu 进入 Finished 或 Fault
等待遥控器 clear/reset 后进入下一轮
```

不负责

```text
不计算伪导航速度
不做地图导航
不打开相机
不加载视觉模型
不做手眼矩阵计算
不直接生成授粉序列
```

## 二，状态机流程

主要状态

```text
BOOTSTRAP
  ↓
WAIT_MCU_STATUS
  ↓
WAIT_START
  ↓
PRECHECK
  ↓
INITIALIZING
  ↓
RUNNING
  ↓
REPORTING_DONE 或 REPORTING_FAIL
  ↓
WAIT_MCU_FINISHED 或 WAIT_MCU_FAULT
  ↓
WAIT_RESET
```

异常状态

```text
ABORTING
RECOVERY_REQUIRED
SHUTTING_DOWN
```

## 三，启动条件

进入任务前必须满足

```text
mcu 状态新鲜
mcu_app_state 为 AutoPi
auto_start_latched 为 true
pi 已收到 START 或从 /mcu/status 恢复出 START
chassis_ready 为 true
odom_ready 为 true
如果 require_arm_ready_in_common_precheck=true，则 arm_ready 也必须为 true
```

注意

```text
pi 不负责请求 mcu 进入 AutoPi
如果 mcu 没有进入 AutoPi，manager 只等待
```

## 四，路线执行方式

路线文件

```text
config/mission_route.yaml
```

执行顺序

```text
读取 waypoints
根据 max_forward_waypoints 截断前进点
对每个点调用导航后端
导航成功后检查 arrival_task
arrival_task 为 noop 时直接进入下一点
arrival_task 非 noop 时调用视觉授粉后端
前进点完成后根据 return_home_enabled 判断是否执行 return_waypoints
全部完成后上报 DONE
```

点位字段

| 字段 | 说明 |
|---|---|
| `id` | 点位唯一编号 |
| `x` | 任务相对 x，单位米 |
| `y` | 任务相对 y，单位米 |
| `yaw` | 任务相对偏航角，单位弧度 |
| `timeout_s` | 单点导航超时时间 |
| `prepare_action` | 该点对应的预识别动作名称 |
| `arrival_task` | 到点后的任务名称 |

## 五，后端选择

配置文件

```text
config/mission_manager.yaml
```

当前默认

```yaml
navigation_backend: "pseudo"
manipulation_backend: "vision_pollination"
```

当前已经实现

```text
pseudo
  由 atlas_nav_pseudo_backend 提供

vision_pollination
  由 atlas_vision_pollination_backend 提供
```

后续完整导航接入方式

```text
新增导航后端包
实现 StartNavigation，CancelNavigation 和 NavigationStatus
把 navigation_backend 改成新后端名称
把 navigation_start_service，navigation_cancel_service，navigation_status_topic 改成新后端接口
```

后续视觉只选择动作序列的接入方式

```text
新增 manipulation 后端包
实现 StartManipulation，CancelManipulation 和 ManipulationStatus
把 manipulation_backend 改成新后端名称
把 manipulation_start_service，manipulation_cancel_service，manipulation_status_topic 改成新后端接口
```

## 六，速度安全门控

manager 是 `/motor_cmd_vel` 的唯一发布者

输入

```text
/atlas/navigation/cmd_vel
```

输出

```text
/motor_cmd_vel
```

允许运动的条件

```text
任务处于 RUNNING
mcu 状态新鲜
mcu_app_state 为 AutoPi
auto_start_latched 为 true
没有 Fault
没有 EStop
导航后端正在执行
```

不允许运动时

```text
发布零速
请求 /mcu/set_brake
忽略导航后端的非零速度
```

## 七，结果上报

任务成功

```text
调用 /mcu/report_mission_result
result=RESULT_DONE
code=0
等待 mcu_app_state 变为 Finished
```

任务失败

```text
调用 /mcu/report_mission_result
result=RESULT_FAIL
code=错误码
等待 mcu_app_state 变为 Fault
```

如果结果上报成功但 mcu 未在超时内确认

```text
进入 RECOVERY_REQUIRED
不重复执行任务
等待人工处理和 clear/reset
```

## 八，配置文件

### mission_manager.yaml

控制总状态机的接口名，后端名，超时和安全策略

### mission_route.yaml

控制任务路线，前进点，返航点，预识别动作名和到点任务名

## 九，启动

启动整个任务栈

```bash
ros2 launch atlas_mission_manager mission_stack.launch.py
```

只启动总状态机

```bash
ros2 launch atlas_mission_manager mission_manager.launch.py
```

启动时替换配置

```bash
ros2 launch atlas_mission_manager mission_stack.launch.py \
  manager_config:=/home/wheeltec/my_config/mission_manager.yaml \
  route:=/home/wheeltec/my_config/mission_route.yaml
```

## 十，状态查看

```bash
ros2 topic echo /atlas/mission/status
```

常用状态含义

| 状态 | 含义 |
|---|---|
| `WAIT_MCU_STATUS` | 等待 mcu 状态 |
| `WAIT_START` | 等待自动任务 START |
| `PRECHECK` | 检查 ready 和锁存 |
| `RUNNING` | 任务执行中 |
| `REPORTING_DONE` | 上报完成 |
| `WAIT_MCU_FINISHED` | 等待 mcu 进入 Finished |
| `REPORTING_FAIL` | 上报失败 |
| `WAIT_RESET` | 等待遥控器 clear/reset |
| `RECOVERY_REQUIRED` | 需要人工恢复 |

## 十一，联调建议

第一步只开一个点

```yaml
max_forward_waypoints: 1
return_home_enabled: false
```

第二步打开第二个点但保持 arrival_task 为 noop

```yaml
max_forward_waypoints: 2
```

第三步把目标点的 arrival_task 改为

```yaml
arrival_task: "visual_pollination"
```

第四步验证视觉和授粉动作

第五步逐步打开更多点位和返航

## 地图点位配置补充

`mission_route.yaml` 已经按 `src(141).zip` 的路线补齐 20 个前进点和 2 个返航点

`nav_index` 用于对照旧 MCU 代码，`id` 用于当前 ROS2 任务流

`prepare_action` 只写动作名称，具体关节角在 `atlas_vision_pollination_backend/config/pollination_actions.yaml` 中配置

`arrival_task` 默认使用 `visual_pollination`，过渡点使用 `noop`

首次实车联调保持 `max_forward_waypoints: 1`，确认后再逐步增加

# atlas_mission_manager

`atlas_mission_manager` 是 Pi 端自动任务总状态机

它只负责任务生命周期，后端选择，安全门控和任务结果上报

它不直接实现伪导航，不直接实现视觉识别，也不直接计算机械臂授粉目标

---

## 一，职责

本包负责

```text
监听 /mcu/status
监听 /mcu/auto_task_event
读取 mission_route.yaml
按点位调用导航后端
按点位调用视觉授粉后端
统一发布 /motor_cmd_vel
调用 /mcu/report_mission_result 上报 DONE 或 FAIL
处理 RESET，Fault，EStop，Manual 抢占和状态超时
```

本包不负责

```text
串口协议解析
伪导航速度计算
视觉模型推理
手眼变换
机械臂目标生成
```

---

## 二，主流程

```text
WAIT_START
  ↓
PRECHECK
  ↓
LOAD_ROUTE
  ↓
START_NAVIGATION
  ↓
WAIT_NAVIGATION
  ↓
START_MANIPULATION
  ↓
WAIT_MANIPULATION
  ↓
NEXT_WAYPOINT
  ↓
REPORT_DONE
```

异常路径

```text
RESET / Fault / EStop / Manual / 状态超时
  ↓
取消后端任务
  ↓
发布零速
  ↓
请求刹车
  ↓
等待下一轮 RESET 或 START
```

---

## 三，配置文件

```text
config/mission_manager.yaml
config/mission_route.yaml
```

### mission_manager.yaml

用于配置总状态机参数

常用字段

| 字段 | 说明 |
|---|---|
| `status_timeout_s` | MCU 状态超时时间 |
| `safe_stop_publish_hz` | 安全停止速度发布频率 |
| `navigation_start_service` | 导航后端启动服务 |
| `navigation_cancel_service` | 导航后端取消服务 |
| `manipulation_start_service` | 视觉授粉后端启动服务 |
| `manipulation_cancel_service` | 视觉授粉后端取消服务 |

### mission_route.yaml

用于配置点位和任务

点位示例

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
| `timeout_s` | 该点最大导航时间 |
| `prepare_action` | 到点任务使用的预识别动作名称 |
| `arrival_task` | 到点任务名称，支持 `noop` 和 `visual_pollination` |

---

## 四，后端选择

在 `mission_route.yaml` 中配置

```yaml
navigation_backend: "pseudo"
manipulation_backend: "vision_pollination"
```

当前实现

```text
pseudo
vision_pollination
```

后续新增后端时，总状态机只需要读取新的后端名称和服务名

---

## 五，速度安全门控

导航后端只发布

```text
/atlas/navigation/cmd_vel
```

本包接收该速度后，根据当前任务状态决定是否转发到

```text
/motor_cmd_vel
```

只有以下条件满足时才允许非零速度

```text
MCU 状态为 AutoPi
任务处于导航阶段
后端状态正常
没有 RESET，Fault，EStop，Manual 抢占
```

否则本包持续发布零速并请求刹车

---

## 六，启动

通常不单独启动本包，推荐使用总启动文件

```bash
ros2 launch atlas_mission_manager mission_stack.launch.py
```

单独启动

```bash
ros2 launch atlas_mission_manager mission_manager.launch.py
```

查看状态

```bash
ros2 topic echo /atlas/mission/status
```

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

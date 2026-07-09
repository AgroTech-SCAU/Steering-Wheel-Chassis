# atlas_mission_interfaces 说明

本包保存 pi 端任务系统公共消息和服务接口

它只定义接口，不包含任何运行逻辑

## 一，包定位

本包用于解耦以下模块

```text
atlas_mission_manager
atlas_nav_pseudo_backend
atlas_vision_pollination_backend
未来完整导航后端
未来视觉动作序列选择后端
```

约束

```text
接口包不能订阅话题
接口包不能发布话题
接口包不能调用服务
接口包不能读取配置文件
```

## 二，消息接口

### MissionStatus.msg

用途

```text
发布总任务状态机状态
用于查看当前是否等待 mcu，等待 START，运行中，上报结果，等待 RESET，或需要人工恢复
```

关键字段

| 字段 | 含义 |
|---|---|
| `state` | 总状态枚举 |
| `local_run_id` | pi 本地任务轮次编号 |
| `active` | 当前是否有本地任务正在运行 |
| `mcu_status_fresh` | mcu 状态是否新鲜 |
| `auto_start_latched` | mcu 自动启动锁存状态 |
| `mcu_app_state` | mcu 当前应用状态 |
| `result_reported` | 本轮是否已经上报 DONE 或 FAIL |
| `error_code` | 错误码 |
| `state_name` | 中文或简短状态名 |
| `message` | 当前说明 |

状态枚举

```text
STATE_BOOTSTRAP
STATE_WAIT_MCU_STATUS
STATE_WAIT_START
STATE_PRECHECK
STATE_INITIALIZING
STATE_RUNNING
STATE_ABORTING
STATE_REPORTING_DONE
STATE_WAIT_MCU_FINISHED
STATE_REPORTING_FAIL
STATE_WAIT_MCU_FAULT
STATE_WAIT_RESET
STATE_RECOVERY_REQUIRED
STATE_SHUTTING_DOWN
```

### NavigationStatus.msg

用途

```text
导航后端发布当前导航状态
总任务状态机通过该话题判断单个 waypoint 是否成功，失败或取消
```

关键字段

| 字段 | 含义 |
|---|---|
| `state` | 导航状态 |
| `backend` | 后端名称，当前为 pseudo |
| `waypoint_id` | 当前点位编号 |
| `target_x_m` | 目标 x 坐标，单位米 |
| `target_y_m` | 目标 y 坐标，单位米 |
| `target_yaw_rad` | 目标偏航角，单位弧度 |
| `distance_error_m` | 当前位置误差 |
| `yaw_error_rad` | 当前偏航误差 |
| `error_code` | 错误码 |
| `message` | 状态说明 |

状态枚举

```text
STATE_IDLE
STATE_RUNNING
STATE_SUCCEEDED
STATE_FAILED
STATE_CANCELLED
```

### ManipulationStatus.msg

用途

```text
视觉授粉后端发布当前动作状态
总任务状态机通过该话题判断到点任务是否成功，失败或取消
```

关键字段

| 字段 | 含义 |
|---|---|
| `state` | 执行状态 |
| `backend` | 后端名称，当前为 vision_pollination |
| `waypoint_id` | 点位编号 |
| `task_id` | 到点任务编号 |
| `step_name` | 当前动作步骤 |
| `error_code` | 错误码 |
| `message` | 状态说明 |

## 三，服务接口

### StartNavigation.srv

用途

```text
总任务状态机请求导航后端移动到一个点位
```

请求字段

| 字段 | 含义 |
|---|---|
| `backend` | 期望后端名称，当前填 pseudo |
| `waypoint_id` | 点位编号 |
| `x_m` | 任务相对 x，单位米 |
| `y_m` | 任务相对 y，单位米 |
| `yaw_rad` | 任务相对偏航角，单位弧度 |
| `reset_origin` | 是否用当前 /odom 作为任务原点 |
| `timeout_s` | 单点超时时间 |

响应字段

| 字段 | 含义 |
|---|---|
| `success` | 是否接受请求 |
| `message` | 接受或拒绝原因 |

### CancelNavigation.srv

用途

```text
总任务状态机请求导航后端取消当前点位移动
```

### StartManipulation.srv

用途

```text
总任务状态机请求视觉授粉后端执行某个点位的到位任务
```

请求字段

| 字段 | 含义 |
|---|---|
| `backend` | 期望后端名称，当前填 vision_pollination |
| `waypoint_id` | 点位编号 |
| `prepare_action` | 预识别动作名称 |
| `arrival_task` | 到位任务名称 |

### CancelManipulation.srv

用途

```text
总任务状态机请求视觉授粉后端取消当前执行
```

### DetectCameraTarget.srv

用途

```text
视觉服务返回一个相机坐标系下的目标点
```

请求字段

| 字段 | 含义 |
|---|---|
| `waypoint_id` | 当前点位编号 |
| `task_id` | 当前任务编号 |

响应字段

| 字段 | 含义 |
|---|---|
| `success` | 是否识别到需要执行的目标 |
| `message` | 目标类别，NO_TARGET，或错误原因 |
| `target_camera_m` | 相机光学坐标系目标点，单位米 |

约定

```text
success=true 表示 target_camera_m 有效
success=false 且 message=NO_TARGET 表示识别完成但无需执行
success=false 且 message 为其他值表示视觉失败
```

## 四，新增后端时的规则

新增完整导航后端时

```text
实现 /atlas/navigation/start
实现 /atlas/navigation/cancel
发布 /atlas/navigation/status
如果需要输出速度，仍发布 /atlas/navigation/cmd_vel 或由 manager 单独适配
```

新增视觉动作序列选择后端时

```text
实现 /atlas/manipulation/start
实现 /atlas/manipulation/cancel
发布 /atlas/manipulation/status
保持 StartManipulation 中 prepare_action 和 arrival_task 的语义不变
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

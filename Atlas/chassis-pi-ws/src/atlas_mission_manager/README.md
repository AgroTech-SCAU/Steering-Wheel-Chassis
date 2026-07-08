# atlas_mission_manager

Atlas 树莓派端的公共任务生命周期与安全状态机

当前版本只实现前置条件和通用部分，不定义或接入导航、视觉、机械臂任务后端

## 已实现

- 订阅 `/mcu/status`；
- 订阅 `/mcu/auto_task_event`；
- START/RESET 去重；
- 公共 `AutoPi` 前置检查；
- Pi 本地单轮任务上下文；
- Manual/Fault/EStop/RESET/状态超时安全取消；
- 零速与底盘刹车；
- 为后续真实任务流预留 `allow_motion()`，由任务流明确释放刹车；
- `/mcu/report_mission_result` DONE/FAIL 上报；
- 等待 MCU `Finished/Fault` 确认；
- 节点晚启动进入 `RECOVERY_REQUIRED`；
- `/atlas/mission/status` 状态发布；
- dry-run 成功、失败和保持模式

## 未实现

- 导航 Service client；
- 视觉 Service client；
- waypoint 流程；
- 手眼坐标变换；
- 机械臂动作执行与到位判断

## 启动

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch atlas_mission_manager mission_manager.launch.py
```

默认：

```yaml
dry_run_mode: "hold"
```

该模式进入 `RUNNING` 后只等待 RESET 或 MCU 安全状态变化，不会自动上报 DONE/FAIL

验证成功生命周期时可临时设置：

```yaml
dry_run_mode: "success_after_delay"
```

验证失败生命周期时可临时设置：

```yaml
dry_run_mode: "fail_after_delay"
```

验证结束后必须恢复 `hold`

## 状态查看

```bash
ros2 topic echo /atlas/mission/status \
  --qos-reliability reliable \
  --qos-durability transient_local
```

## 安全原则

- 节点晚启动时，如果 MCU 已经是 `AutoPi + auto_start_latched=1`，不会从头重放任务；
- RESET、Manual、Fault、EStop 和 MCU 状态超时都会触发零速和刹车；
- 人工接管、RESET 和 EStop 不会被自动改写成普通任务 FAIL；
- Service 返回成功后仍等待 MCU 状态确认；
- 本包不会主动请求 MCU 进入 AutoPi

## 后续任务流接入边界

后续真实任务流只需要在 `RUNNING` 状态下完成三类结果回传：

```text
TaskFlowSucceeded
TaskFlowFailed(error_code, message)
TaskFlowCancelled(reason)
```

公共状态机继续负责：

```text
START / RESET
MCU 状态检查
安全停止
DONE / FAIL 上报
Finished / Fault 确认
恢复策略
```

导航和视觉 Service 的类型、请求和响应应在后续任务流模块中按既有工作区接口接入，不应写入公共状态机核心

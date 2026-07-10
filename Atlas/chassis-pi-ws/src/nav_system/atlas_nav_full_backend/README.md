# atlas_nav_full_backend

`atlas_nav_full_backend` 是任务状态机和 Nav2 之间的适配层。它不重新实现规划或控制，而是把 `atlas_mission_manager` 的 `/atlas/navigation/start` 请求转换为 Nav2 的 `NavigateToPose` action，并把 Nav2 执行结果转换为 `/atlas/navigation/status`。

## 设计边界

- 总任务状态机只认识统一接口：`StartNavigation`、`CancelNavigation`、`NavigationStatus`。
- 完整导航栈仍由 `at_nav2` 启动，负责定位、全局规划、局部控制和 `/cmd_vel` 输出。
- 本后端只负责接收任务点、发送 Nav2 goal、取消 Nav2 goal、回传运行状态。
- 底盘速度最终仍必须经过 `atlas_mission_manager` 的安全门控，再发布到 `/motor_cmd_vel`，不要让 Nav2 直接控制 MCU。

## 坐标模式

配置项 `coordinate_mode` 支持两种模式：

1. `task_relative_odom`
   - 第一个 waypoint 的 `reset_origin=true` 时记录当前 `/odom` 位姿作为任务原点。
   - 后续 route 中的 `x/y/yaw` 被解释为相对任务原点的坐标。
   - 目标以 `odom` frame 发送给 Nav2，Nav2 需要能够通过 TF 把 `odom` 目标转换到全局 frame。
   - 适合保留原先伪导航的相对点位表，做完整导航联调。

2. `absolute_map`
   - route 中的 `x/y/yaw` 被直接解释为 `map` frame 下的绝对目标。
   - 适合地图已经标定完成、点位已经在 map 坐标中重新标注的正式运行。

## 启动方式

单独启动适配器：

```bash
ros2 launch atlas_nav_full_backend full_nav_backend.launch.py
```

和完整任务栈一起启动：

```bash
ros2 launch atlas_mission_manager mission_stack.launch.py navigation_backend:=full
```

## 关键参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `backend_name` | `full` | 必须与 route 中的 `navigation_backend` 一致。 |
| `action_name` | `navigate_to_pose` | Nav2 的 `NavigateToPose` action 名称。 |
| `coordinate_mode` | `task_relative_odom` | 任务点坐标解释方式。 |
| `odom_topic` | `/odom` | 相对坐标模式下读取任务原点。 |
| `status_topic` | `/atlas/navigation/status` | 任务状态机订阅的导航状态。 |
| `start_service` | `/atlas/navigation/start` | 任务状态机调用的启动服务。 |
| `cancel_service` | `/atlas/navigation/cancel` | 任务状态机调用的取消服务。 |

## 联调检查

1. `ros2 action list | grep navigate_to_pose` 能看到 Nav2 action。
2. `ros2 topic echo /atlas/navigation/status` 能看到 `backend=full`。
3. `ros2 topic echo /atlas/navigation/cmd_vel` 能看到 Nav2 输出被 remap 到任务状态机输入。
4. `ros2 topic echo /motor_cmd_vel` 只有在任务状态机处于允许运动阶段时才会输出速度。


# mcu_comm_bridge

`mcu_comm_bridge` 是 Atlas Pi 端的 ROS 2 通信桥，负责在 ROS 2 接口与 MCU 二进制协议之间做双向转换。

## 1. 负责范围

本包负责：

- 解析 `MCU_IMU`、`MCU_ODOM`、`MCU_ARM_STATE`、`MCU_STATUS`
- 发布 `/imu`、`/odom`、`/arm/joint_states`、`/arm/pose`、`/arm/pose_position`
- 发布 `/mcu/status` 和 `/mcu/auto_task_event`
- 订阅 `/motor_cmd_vel` 并按周期发送 `PI_CONTROL`
- 提供刹车、急停、Yaw、机械臂控制和任务结果上报服务

本包不负责：

- 主动请求 MCU 进入 `AutoPi`
- 直接启动 Nav2
- 直接取消 Nav2 goal
- 实现完整的 mission manager

## 2. MCU 状态所有权

MCU 是以下内容的唯一所有者：

- 本地应用状态机
- `AutoPi` 进入权限
- `Finished` 与 `Fault` 状态切换
- `auto_start_latched`
- 安全边界与急停逻辑

Pi bridge 只负责：

1. 接收 `MCU_STATUS`
2. 解析 `auto_start_latched`
3. 检测 `START` / `RESET` 边沿
4. 清理本地自动任务上下文
5. 向上层发布一次性事件

## 3. 主要话题

### 3.1 `/mcu/status`

消息类型：

```text
mcu_comm_bridge/msg/McuStatus
```

当前状态常量：

```text
STATE_IDLE=0
STATE_MANUAL=1
STATE_AUTO_PI=2
STATE_FAULT=3
STATE_ESTOP=4
STATE_FINISHED=5
```

当 MCU 上报 `app_state=5` 时，bridge 会记录为 `Finished`。

### 3.2 `/mcu/auto_task_event`

该话题只发布一次性边沿事件：

- `EVENT_START`
- `EVENT_RESET`

上层应基于该话题触发自动任务，而不是仅仅因为 `app_state == AutoPi` 就重复启动任务。

## 4. 任务结果上报服务

服务名：

```text
/mcu/report_mission_result
```

服务类型：

```text
mcu_comm_bridge/srv/ReportMissionResult
```

定义：

```text
uint8 RESULT_DONE=1
uint8 RESULT_FAIL=2

uint8 result
int16 code
---
bool success
string message
uint8 sent_count
```

调用示例。

任务完成：

```bash
ros2 service call /mcu/report_mission_result \
  mcu_comm_bridge/srv/ReportMissionResult \
  "{result: 1, code: 0}"
```

任务失败：

```bash
ros2 service call /mcu/report_mission_result \
  mcu_comm_bridge/srv/ReportMissionResult \
  "{result: 2, code: -123}"
```

映射规则：

- `result=1` -> `DONE`
- `result=2` -> `FAIL`
- `DONE` 一律编码为 `event=1, code=0`
- `FAIL` 一律编码为 `event=2, code=request.code`

## 5. 服务接受条件

调用 `/mcu/report_mission_result` 前，bridge 会检查：

- 已收到 `MCU_STATUS`
- MCU 当前处于 `STATE_AUTO_PI`
- MCU `auto_start_latched == true`
- Pi 本地 `mission_active == true`
- 当前任务结果尚未成功上报
- 当前没有并发中的任务结果上报

典型拒绝原因包括：

- `MCU status is not available`
- `MCU is not in AutoPi`
- `auto task is not latched`
- `no active mission`
- `mission result already reported`
- `mission result report is already in progress`
- `unsupported mission result`

## 6. 任务结果发送语义

`PI_MISSION_EVENT` 当前没有 MCU ACK。

因此：

- `success=true` 只表示至少有一帧成功写入 Pi 串口
- 不表示 MCU 已解析
- 不表示 MCU 已接受
- 不表示 MCU 已切换到 `Finished` 或 `Fault`

最终结果必须通过 `/mcu/status` 确认：

- `DONE` 后应观察到 `STATE_FINISHED=5`
- `FAIL` 后应观察到 `STATE_FAULT=3`

## 7. 参数

当前与本次任务相关的关键参数如下：

```yaml
mcu_comm_bridge_node:
  ros__parameters:
    arm_pose_position_topic: "/arm/pose_position"
    mission_result_service: "/mcu/report_mission_result"
    mission_event_repeat_count: 3
```

其中：

- `mission_result_service` 默认为 `/mcu/report_mission_result`
- `mission_event_repeat_count` 默认为 `3`
- `mission_event_repeat_count` 最小值为 `1`
- 当前代码将其限制在 `1..10`
- 非法值会回退到 `3`

## 8. 构建

```bash
source /opt/ros/humble/setup.bash
colcon build \
  --packages-select mcu_comm_bridge \
  --symlink-install
```

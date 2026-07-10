# atlas_autonomous_transport_manager

该功能包实现 Atlas 在“智械争锋”全自主运输区的任务状态机。状态机覆盖中转区播报、语音启动、智能分拣规则识别、待派送区抓取、园区映射、往返运输、安全停止和 MCU 结果上报。

## 状态流程

```text
WAIT_START
  -> PRECHECK
  -> ANNOUNCE_TRANSITION
  -> WAIT_VOICE_START（可配置）
  -> NAVIGATE_SORTING_AREA
  -> CLASSIFY_SORTING_RULE
  -> NAVIGATE_DISPATCH_AREA
  -> SELECT_CARGO
  -> PICK_CARGO
  -> NAVIGATE_TARGET_PARK
  -> PLACE_CARGO
  -> 返回 NAVIGATE_DISPATCH_AREA
  -> COMPLETE / FAILED
  -> REPORTING_DONE / REPORTING_FAIL
  -> WAIT_RESET
```

进入 `RUNNING` 后，状态机不提供返回遥操作区的转移。MCU 离开 `STATE_AUTO_PI`、清除任务锁存、进入手动控制、故障、急停或状态超时时，状态机立即取消导航与机械臂后端，发布零速度并请求制动。

## 主要接口

- `/mcu/status`：MCU 生命周期、就绪标志、故障和急停状态。
- `/mcu/auto_task_event`：自动任务启动与复位事件。
- `/atlas/navigation/start`、`/atlas/navigation/cancel`：导航后端服务。
- `/atlas/navigation/status`：导航执行状态。
- `/atlas/navigation/cmd_vel`：导航后端速度，经过状态机速度门控后转发至 `/motor_cmd_vel`。
- `/atlas/manipulation/start`、`/atlas/manipulation/cancel`：机械臂任务服务。
- `/atlas/manipulation/status`：机械臂任务状态和视觉目标数量。
- `/vision/classify_sorting_rule`：齿轮与 T 型螺栓的园区映射识别服务。
- `/atlas/voice/text`：待播报文本。
- `/atlas/voice/command`：语音识别文本输入。
- `/atlas/autonomous_transport/status`：全自主运输状态、阶段、映射和计数。

## 标定门控

`config/autonomous_transport.yaml` 中的以下数据必须在实际场地测量：

1. 智能分拣区、待派送区、园区 1、园区 2 的 `map` 坐标与停车朝向。
2. `transport_actions.yaml` 中的收拢、抓取观察和园区投放关节位姿。
3. RACOM 像素到相机坐标的深度与比例参数。
4. 智能分拣标识在画面左、右槽位与园区编号的映射。
5. `classification.minimum_confidence` 的现场识别阈值。

未完成上述确认时保持 `calibration_confirmed: false`。此时状态机可以启动并发布诊断，但预检不会允许机器人产生任务运动。

## 构建

```bash
cd ~/Atlas/chassis-pi-ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 启动全自主运输任务栈

```bash
ros2 launch atlas_autonomous_transport_manager autonomous_transport_stack.launch.py \
  navigation_backend:=full \
  manipulation_backend:=racom_vision \
  enable_voice_player:=true
```

整车启动入口：

```bash
ros2 launch robot_startup robot_autonomous_transport.launch.py
```

## 语音启动输入

语音识别节点需要把最终识别文本发布到 `/atlas/voice/command`。接口联调可使用：

```bash
ros2 topic pub --once /atlas/voice/command std_msgs/msg/String \
  "{data: '执行全自主运输任务'}"
```

该命令只用于接口测试。正式比赛应由麦克风语音识别节点自主发布，机器人开始运动后不得通过键盘、鼠标或控制器继续操控。

## 状态观察

```bash
ros2 topic echo /atlas/autonomous_transport/status
ros2 topic echo /atlas/navigation/status
ros2 topic echo /atlas/manipulation/status
```

## 导航坐标模式

全自主运输配置中的停车点是 `map` 坐标系绝对坐标。完整导航启动时默认加载 `config/autonomous_full_nav.yaml`，其中：

```yaml
coordinate_mode: "absolute_map"
```

不要在全自主运输实车配置中使用 `task_relative_odom`，否则停车点会被解释为任务起点附近的相对位移。

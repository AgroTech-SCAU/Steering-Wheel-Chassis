# atlas_autonomous_transport_manager

该功能包实现智械争锋全自主运输区的生命周期；任务编排；速度门控；错误跳过和 MCU 结果上报

## 流程

```text
WAIT_START
→ PRECHECK
→ ANNOUNCE_TRANSITION
→ ANNOUNCE_VOICE_PROMPT
→ WAIT_VOICE_START
→ ANNOUNCE_AUTONOMOUS_START
→ NAVIGATE_SORTING_AREA
→ CLASSIFY_SORTING_RULE
→ NAVIGATE_DISPATCH_AREA
→ PICK_CARGO
→ NAVIGATE_TARGET_PARK
→ PLACE_CARGO
→ COMPLETE
→ REPORTING_DONE
→ WAIT_RESET
```

## 配置

```text
config/autonomous_transport.yaml
config/autonomous_full_nav.yaml
```

`autonomous_transport.yaml` 同时由顶层 launch 读取；其中 `system` 段管理整车启动；`autonomous_transport` 段管理状态机

## 启动

只启动状态机

```bash
ros2 launch atlas_autonomous_transport_manager autonomous_transport.launch.py
```

整车启动

```bash
ros2 launch robot_startup robot_autonomous_transport.launch.py
```

## 关键接口

```text
/mcu/status
/mcu/auto_task_event
/atlas/asrpro/status
/atlas/asrpro/recognized
/atlas/asrpro/speak
/atlas/navigation/start
/atlas/navigation/cancel
/atlas/manipulation/start
/atlas/manipulation/cancel
/vision/classify_sorting_rule
/atlas/autonomous_transport/status
```

## 错误策略

普通播报；导航；识别和抓取错误先重试；达到上限后记录并进入安全后继阶段

Manual；RESET；EStop；Fault；离开 AutoPi 和 MCU 状态超时属于安全中断；立即取消后端并制动

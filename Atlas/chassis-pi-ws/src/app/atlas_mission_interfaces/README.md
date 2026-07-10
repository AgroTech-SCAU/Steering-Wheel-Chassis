# atlas_mission_interfaces

Atlas Pi 端任务层公共消息和服务定义

## 主要消息

```text
NavigationStatus
ManipulationStatus
AutonomousTransportStatus
AsrproStatus
AsrproEvent
```

## 主要服务

```text
StartNavigation
CancelNavigation
StartManipulation
CancelManipulation
ClassifySortingRule
DetectCameraTarget
AsrproSpeak
```

接口包不包含业务状态机；只提供导航；视觉；机械臂；ASRPRO 和全自主运输之间的稳定契约

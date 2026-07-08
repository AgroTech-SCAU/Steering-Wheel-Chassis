# atlas_mission_interfaces

Atlas Pi 端任务生命周期的公共 ROS2 接口包

当前只定义：

```text
atlas_mission_interfaces/msg/MissionStatus
```

该消息描述 Pi 端公共任务状态、MCU 状态摘要、运行编号和错误信息；导航、视觉和机械臂工作区的 Service 接口不放在本包中，也不由本包重新定义

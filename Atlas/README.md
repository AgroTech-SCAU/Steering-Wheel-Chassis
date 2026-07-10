# Atlas

Atlas 工程由 MCU 底盘控制；PC 遥操作；Pi 端 ROS 2 全自主运输和统一协议文档组成

## 目录

```text
Atlas/
├── chassis_control_code/     MCU 实时控制和应用状态机
├── chassis-pc-ws/            PC 主臂遥操作
├── chassis-pi-ws/            Pi 端 MCU 桥；导航；视觉；ASRPRO 和全自主状态机
├── docs/                     文档入口
└── README.md
```

## 任务范围

任务范围为智械争锋全自主运输区；遥操作任务和 PC 端不属于本工作区任务流；MCU 固件保持独立

全自主流程

```text
MCU AutoPi 手势
→ ASRPRO 播报遥操作区任务已完成
→ 等待 Atlas 启动
→ ASRPRO 播报全自主任务启动
→ 智能分拣区识别园区映射
→ 待派送区取货
→ 园区 1 和园区 2 派送
→ 安全收尾和 MCU 结果上报
→ 恢复手势清理本轮锁存
```

## 唯一整车启动入口

```bash
cd ~/Atlas/chassis-pi-ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch robot_startup robot_autonomous_transport.launch.py
```

统一配置

```text
chassis-pi-ws/src/app/atlas_autonomous_transport_manager/config/autonomous_transport.yaml
```

## 文档

```text
chassis-pi-ws/docs/配置指南.md
chassis-pi-ws/docs/使用说明与教程.md
chassis-pi-ws/docs/工作区详细介绍及完整任务流.md
chassis-pi-ws/docs/状态机详细说明及错误处理.md
chassis-pi-ws/docs/ASRPRO_TWEN51适配与烧录说明.md
chassis-pi-ws/docs/串口固定绑定教程.md
chassis-pi-ws/docs/视觉模型配置与标定.md
chassis-pi-ws/docs/comms_protocol.md
```

## 安全门控

默认 `calibration_confirmed: false`；地图点；机械臂动作和视觉标定完成前不会产生任务运动

Manual；EStop；MCU Fault；MCU 状态超时或离开 AutoPi 时；Pi 立即取消后端；发布零速度并请求制动

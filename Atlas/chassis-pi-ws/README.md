# chassis-pi-ws

Pi 工作区负责 MCU ROS 桥；激光导航；最新 ONNX 视觉；ASRPRO TWEN51 和智械争锋全自主运输状态机

## 核心目录

```text
src/app/atlas_mission_interfaces
src/app/atlas_autonomous_transport_manager
src/asrpro/atlas_asrpro_bridge
src/mcu_comm_bridge
src/nav_system
src/vision_system
docs
scripts
```

## 编译

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 启动

```bash
ros2 launch robot_startup robot_autonomous_transport.launch.py
```

统一 YAML

```text
src/app/atlas_autonomous_transport_manager/config/autonomous_transport.yaml
```

视觉模型路径在 `system.paths.vision_model_path` 配置；留空使用最新视觉包内置 `resource/best.onnx`

## 设备固定名称

```text
/dev/atlas_mcu
/dev/atlas_asrpro
/dev/atlas_lidar
/dev/atlas_camera
```

## 状态接口

```bash
ros2 topic echo /mcu/status
ros2 topic echo /atlas/asrpro/status
ros2 topic echo /atlas/autonomous_transport/status
ros2 topic echo /atlas/navigation/status
ros2 topic echo /atlas/manipulation/status
```

## 文档入口

先阅读

```text
docs/配置指南.md
docs/使用说明与教程.md
docs/工作区详细介绍及完整任务流.md
docs/状态机详细说明及错误处理.md
```

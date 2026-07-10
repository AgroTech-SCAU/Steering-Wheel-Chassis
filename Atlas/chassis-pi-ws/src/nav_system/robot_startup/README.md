# robot_startup

该功能包只保留一个正式整车入口

```bash
ros2 launch robot_startup robot_autonomous_transport.launch.py
```

统一配置参数

```bash
ros2 launch robot_startup robot_autonomous_transport.launch.py \
  config:=/absolute/path/autonomous_transport.yaml
```

launch 使用统一 YAML 组织以下顺序

```text
ASRPRO + MCU
→ 雷达 + robot_description
→ Cartographer + Nav2 + 导航后端
→ 最新视觉模型 + 视觉适配
→ 机械臂动作后端
→ 全自主状态机
```

延时只组织启动顺序；状态机仍检查实际设备状态和服务就绪条件

开机自启安装脚本

```bash
sudo ./scripts/install_atlas_autostart.sh "$PWD" wheeltec
```

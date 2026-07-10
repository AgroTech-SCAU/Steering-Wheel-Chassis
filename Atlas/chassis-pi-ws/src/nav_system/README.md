# Atlas 导航系统

全自主运输默认使用 Cartographer 纯定位；Nav2 和 `atlas_nav_full_backend`

## 启动

整车统一启动

```bash
ros2 launch robot_startup robot_autonomous_transport.launch.py
```

导航后端在统一 YAML 中选择

```yaml
system:
  launch:
    navigation_backend: "full"
```

安全联调可改为

```yaml
navigation_backend: "pseudo"
```

## 速度链路

```text
Nav2 /cmd_vel
→ /atlas/navigation/cmd_vel
→ atlas_autonomous_transport_manager 安全门控
→ /motor_cmd_vel
→ mcu_comm_bridge
→ MCU
```

Nav2 不直接控制 MCU

## 主要功能包

```text
at_nav2                     Cartographer 纯定位；地图服务和 Nav2
atlas_nav_full_backend      NavigateToPose 适配
atlas_nav_pseudo_backend    无 Nav2 接口联调
lslidar_driver              LSN10P 雷达
robot_description           URDF 和静态 TF
robot_startup               唯一整车启动入口
```

全自主停车点使用 `map` 绝对坐标；对应后端配置为

```text
src/app/atlas_autonomous_transport_manager/config/autonomous_full_nav.yaml
```

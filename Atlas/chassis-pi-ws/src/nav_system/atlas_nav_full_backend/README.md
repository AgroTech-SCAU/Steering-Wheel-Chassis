# atlas_nav_full_backend

该功能包把 `StartNavigation` 和 `CancelNavigation` 服务转换为 Nav2 `NavigateToPose`

全自主运输配置使用

```yaml
coordinate_mode: "absolute_map"
map_frame: "map"
```

单独启动

```bash
ros2 launch atlas_nav_full_backend full_nav_backend.launch.py \
  config:=/absolute/path/autonomous_full_nav.yaml
```

整车启动

```bash
ros2 launch robot_startup robot_autonomous_transport.launch.py
```

速度由 Nav2 输出到 `/atlas/navigation/cmd_vel`；只有全自主状态机处于已受理导航阶段时才转发到 `/motor_cmd_vel`

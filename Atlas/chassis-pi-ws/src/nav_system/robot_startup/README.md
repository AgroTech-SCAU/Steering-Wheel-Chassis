# robot_startup

`robot_startup` 是 Atlas PI 端整车总启动入口，用于把 MCU 通信桥、雷达、机器人模型、完整导航后端、任务状态机和 RACOM 视觉链路一次性拉起。

---

## 一，当前默认组成

```text
robot_start.launch.py
├── mcu_comm_bridge                # MCU 通信桥，发布 /odom、/imu、机械臂状态，订阅 /motor_cmd_vel
├── lslidar_driver                 # 激光雷达驱动，发布 /scan
├── robot_description              # URDF 与静态 TF
└── atlas_mission_manager/mission_stack.launch.py
    ├── at_nav2                    # Cartographer 纯定位 + Nav2
    ├── atlas_nav_full_backend     # Nav2 NavigateToPose 适配成任务导航后端
    ├── racom_vision               # RACOM/RAICOM ONNX 检测服务
    ├── atlas_racom_vision_backend # RACOM 检测结果适配成 /vision/detect_camera_target
    └── atlas_vision_pollination_backend # 作业动作序列、手眼变换和机械臂控制
```

默认后端：

```text
navigation_backend=full
manipulation_backend=racom_vision
```

---

## 二，速度链路

完整任务链路中，不再让 Nav2 直接控制底盘。

```text
Nav2 controller_server
  -> /cmd_vel
  -> remap 到 /atlas/navigation/cmd_vel
  -> atlas_mission_manager 安全门控
  -> /motor_cmd_vel
  -> mcu_comm_bridge
  -> PI_CONTROL
  -> MCU
```

只有任务状态机处于导航阶段、MCU 处于 `AutoPi`、没有 RESET / Fault / EStop / Manual 抢占时，`atlas_mission_manager` 才会把导航速度转发到 `/motor_cmd_vel`。

---

## 三，运行方式

```bash
source ~/chassis-pi-ws/install/setup.bash
ros2 launch robot_startup robot_start.launch.py
```

只做安全联调、不启动完整导航：

```bash
ros2 launch robot_startup robot_start.launch.py navigation_backend:=pseudo
```

回退旧视觉模型：

```bash
ros2 launch robot_startup robot_start.launch.py manipulation_backend:=vision_pollination
```

---

## 四，分步调试

```bash
# MCU 通信桥
ros2 launch mcu_comm_bridge mcu_comm_bridge.launch.py

# 雷达
ros2 launch lslidar_driver lsn10p_launch.py

# 机器人描述
ros2 launch robot_description robot_description.launch.py

# 任务总栈，默认 full + racom_vision
ros2 launch atlas_mission_manager mission_stack.launch.py
```

如果只想确认完整导航后端是否能调用 Nav2：

```bash
ros2 launch at_nav2 at_nav.launch.py
ros2 launch atlas_nav_full_backend full_nav_backend.launch.py
```

---

## 五，联调检查

```bash
ros2 topic hz /odom
ros2 topic hz /imu
ros2 topic hz /scan
ros2 topic echo /atlas/navigation/status
ros2 topic echo /atlas/mission/status
ros2 service list | grep -E "navigation|manipulation|vision|suction|arm"
```

重点确认：

```text
/odom              约 50 Hz
/imu               约 100 Hz
/scan              按雷达配置输出
/tf                有 map -> odom -> base_footprint 链路
/atlas/navigation/cmd_vel  Nav2 输出经 remap 后的话题
/motor_cmd_vel     任务状态机门控后的实际底盘速度
```

---

## 六，常见问题

### 1. Nav2 有速度但底盘不动

检查：

```bash
ros2 topic echo /atlas/navigation/cmd_vel
ros2 topic echo /motor_cmd_vel
ros2 topic echo /mcu/status
```

如果 `/atlas/navigation/cmd_vel` 有数据而 `/motor_cmd_vel` 没有，通常是任务状态机当前不处于导航阶段，或 MCU 不是 `AutoPi`。

### 2. 完整导航后端启动但目标被拒绝

检查：

```bash
ros2 action list | grep navigate_to_pose
ros2 topic echo /odom
ros2 run tf2_ros tf2_echo map odom
```

`atlas_nav_full_backend` 必须等 Nav2 `NavigateToPose` action server 就绪后才能接受任务目标。

### 3. RACOM 视觉没有结果

检查：

```bash
ros2 service call /vision_detect vison_topic_interfaces/srv/VisionDetect "{start: true}"
ros2 service call /vision_detect vison_topic_interfaces/srv/VisionDetect "{start: false}"
ros2 service call /vision/detect_camera_target atlas_mission_interfaces/srv/DetectCameraTarget "{waypoint_id: test, task_id: test, max_targets: 1, target_class: ''}"
```

如果 `/vision_detect` 有结果但 `/vision/detect_camera_target` 没有结果，优先检查类别名过滤和 `racom_camera_target.yaml` 中的像素到相机坐标近似参数。

### 4. PI 端吸盘服务被拒绝

默认 `require_auto_pi_for_suction=true`。需要 MCU 处于 `AutoPi` 才允许：

```bash
ros2 service call /mcu/set_suction std_srvs/srv/SetBool "{data: true}"
```

台架调试时可临时在 `mcu_comm_bridge.yaml` 中改成：

```yaml
require_auto_pi_for_suction: false
```

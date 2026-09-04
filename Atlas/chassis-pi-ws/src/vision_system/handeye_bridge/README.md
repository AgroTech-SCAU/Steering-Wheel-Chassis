# handeye_bridge — 坐标变换与机械臂桥接

`handeye_bridge` 接收视觉检测中心和机械臂末端位姿，把目标像素转换为机械臂基座坐标，并根据安全配置决定是否调用 MCU 运动服务。

## 1. 坐标链

```text
目标像素 (u,v)
  → 相机内参去畸变
  → 相机坐标中的射线或 PnP 三维点
  → ^gripper T_camera（手眼结果）
  → ^base T_gripper（/arm/pose）
  → base 坐标目标 (x,y,z)
```

bridge 使用检测帧时间戳匹配采集当时的 `/arm/pose`，而不是直接使用收到 `/pick_target` 时的最新位姿。

## 2. 文件

| 文件 | 作用 |
|---|---|
| `handeye_bridge/bridge_node.py` | 主节点 |
| `config/bridge_node.yaml` | 参数与安全门禁 |
| `config/camera_intrinsics.yaml` | 部署用相机内参 |
| `config/samples_result.yaml` | 部署用手眼结果 |
| `launch/bridge_node.launch.py` | 单独启动 bridge |
| `launch/screw_pick.launch.py` | 同时启动视觉和 bridge |

## 3. 构建与启动

标定结果变化后先复制到包内：

```bash
cp calib/camera_intrinsics.yaml handeye_bridge/config/
cp calib/samples_result.yaml handeye_bridge/config/
```

然后从 ROS 2 工作空间根目录构建：

```bash
colcon build --packages-select \
    vison_topic_interfaces handeye_bridge
source install/setup.bash
```

单独启动：

```bash
ros2 launch handeye_bridge bridge_node.launch.py
```

完整启动：

```bash
ros2 launch handeye_bridge screw_pick.launch.py
```

配置文件会安装到 `install/share/handeye_bridge/config/`。源码 YAML 修改后必须重新构建并重新 `source`。

## 4. ROS 接口

| 名称 | 类型 | 方向 | 用途 |
|---|---|---|---|
| `/detection_centers` | `DetectionCenterArray` | 订阅 | 带采集时间戳的检测中心 |
| `/pick_target` | `PickTarget` | 订阅 | 选择角编号和工作层 |
| `/arm/pose` | `PoseStamped` | 订阅 | MCU FK 末端位姿 |
| `/move_to_initial_pose` | `Trigger` | 服务端 | 返回初始观察位 |
| `/move_to_sorting_scan_a` | `Trigger` | 服务端 | 移动到 A 半场分类标识观察位 |
| `/move_to_sorting_scan_b` | `Trigger` | 服务端 | 移动到 B 半场分类标识观察位 |
| `/initial_pose_ready` | `Bool` | 发布 | 初始观察位连续稳定到位后为 true |
| `/vision_pose_ready` | `Bool` | 发布 | initial / sorting_scan_A / sorting_scan_B 任一合法视觉位到位后为 true |
| `/mcu/set_arm_pose` | `SetArmPose` | 服务客户端 | 发送计算后的目标位姿 |

## 5. 配置顺序

编辑 `config/bridge_node.yaml`。新设备或重新标定后，推荐按以下顺序配置。

### 5.1 标定文件

```yaml
intrinsics_file: "camera_intrinsics.yaml"
handeye_result_file: "samples_result.yaml"
```

相对路径以安装后的 `share/handeye_bridge/config/` 为基准，也可使用绝对路径。

### 5.2 安全状态先关闭

首次调试建议：

```yaml
initial_pose_configured: false
plane_heights_configured: false
auto_send: false
```

不要直接沿用旧设备中已经为 `true` 的值。

### 5.3 初始观察位

```yaml
initial_pose_configured: false
auto_move_to_initial_on_start: true
initial_x_m: 0.0
initial_y_m: 0.0
initial_z_m: 0.0
initial_pitch_rad: 0.0
initial_yaw_rad: 0.0
initial_speed_rad_s: 0.3
```

先空载确认坐标和姿态，填写后再将 `initial_pose_configured` 改为 `true`。

bridge 启动后默认延迟发送一次初始位，也可手动调用：

```bash
ros2 service call /move_to_initial_pose std_srvs/srv/Trigger "{}"
ros2 topic echo /initial_pose_ready --once
```

服务响应成功只表示命令已提交；必须等 `/arm/pose` 连续进入容差后，`/initial_pose_ready` 才会变为 `true`。

比赛分类识别位必须由实车 FK 记录后填写。未确认前保持 `configured: false`：

```yaml
sorting_scan_a:
  configured: false
  x_m: 0.0
  y_m: 0.0
  z_m: 0.0
  pitch_rad: 0.0
  yaw_rad: 0.0
  speed_rad_s: 0.5
sorting_scan_b:
  configured: false
  x_m: 0.0
  y_m: 0.0
  z_m: 0.0
  pitch_rad: 0.0
  yaw_rad: 0.0
  speed_rad_s: 0.5
```

`/vision_pose_ready` 覆盖初始观察位和两个 sorting scan 位；`/initial_pose_ready` 仍只表示 default pickup 观察位。

## 6. 深度模式

### 6.1 manual：射线与固定平面求交

```yaml
depth_mode: "manual"
plane1_z_m: -0.03
plane2_z_m: 0.12
plane3_z_m: 0.19
plane_heights_configured: false
target_z_offset_m: 0.03
```

算法：

```text
像素去畸变
  → 相机射线
  → 射线转换到 base
  → 与 Z=plane{layer}_z_m 平面求交
  → x/y 取交点
  → z = plane{layer}_z_m + target_z_offset_m
```

`plane1_z_m` 等参数是基座坐标系中的绝对平面 Z，不是相机到平面的距离。

测量方式：让末端到达工作平面，读取：

```bash
ros2 topic echo /arm/pose --once
```

三层实测并复核后设置：

```yaml
plane_heights_configured: true
```

#### camera_to_plane 参数

```yaml
camera_to_plane1_distance_m: 0.395
camera_to_plane2_distance_m: 0.28
camera_to_plane3_distance_m: 0.21
```

在当前代码中，这三个参数已声明，但 manual 模式的实际坐标计算仍使用“射线与 `planeX_z_m` 求交”，不会读取 `camera_to_planeX_distance_m`。

因此修改 `camera_to_plane1_distance_m` 不会改变当前抓取坐标。它只能作为测量记录和近似像素比例参考：

```text
mm_per_pixel ≈ camera_to_plane_distance_m × 1000 / fx_px
```

### 6.2 pnp：四角矩形求三维位姿

```yaml
depth_mode: "pnp"
pnp_target_width_m: 0.05
pnp_target_height_m: 0.05
pnp_max_reprojection_px: 2.0
pnp_min_depth_m: 0.05
pnp_max_depth_m: 0.80
```

PnP 使用四个中心点和已知矩形宽高，通过 `SOLVEPNP_IPPE` 求目标平面相对相机的位姿。

要求：

- 必须同时存在四个角点；
- 编号必须为 `0 TL、1 TR、2 BR、3 BL`；
- 宽高必须是四个目标中心之间的真实距离；
- 四点必须属于同一个刚性平面矩形；
- 重投影误差和深度必须通过门限。

如果积木可以相互移动、并非固定矩形，不应使用这一 PnP 模式计算共同深度。

## 7. 时间同步与最终最佳帧

实时检测：

1. `/detection_centers.header.stamp` 记录图像采集时间；
2. bridge 在 `/arm/pose` 历史中查找最近位姿；
3. 时间差超过 `max_pose_sync_dt_ms` 时拒绝计算。

停止检测后的最佳帧：

1. 检测节点发布 `is_final_best: true`；
2. 消息仍保留原始图像时间；
3. bridge 从缓存中取该帧对应的原始机械臂位姿；
4. 超过 `final_best_valid_s` 或缓存缺失时拒绝抓取。

相关参数：

```yaml
max_detection_age_s: 0.5
final_best_valid_s: 60.0
max_pose_sync_dt_ms: 80.0
pose_history_size: 200
detection_pose_cache_size: 2000
```

## 8. 选择目标

四目标编号：

```text
0 左上 ───────── 1 右上
  │                 │
3 左下 ───────── 2 右下
```

发布 `/pick_target`：

```bash
# 左上目标，第 1 层
ros2 topic pub /pick_target \
    vison_topic_interfaces/msg/PickTarget \
    "{corner_index: 0, layer: 1}" --once

# 右下目标，第 3 层
ros2 topic pub /pick_target \
    vison_topic_interfaces/msg/PickTarget \
    "{corner_index: 2, layer: 3}" --once
```

bridge 会检查：

- 角编号是否存在；
- 层号是否有效；
- 检测是否过期；
- 检测帧能否匹配机械臂位姿；
- 标定文件是否有效；
- 结果是否在 workspace 内。

## 9. 手动偏置与 workspace

```yaml
manual_offset_x_m: 0.0
manual_offset_y_m: 0.0
manual_offset_z_m: 0.0

workspace_max_xy_m: 0.50
workspace_z_min_m: -0.20
workspace_z_max_m: 0.50
```

手动偏置只适合补偿稳定的整体偏差。若误差随画面位置、深度或第二天状态改变，不要靠偏置掩盖，应重新检查内参、手眼旋转、相机固定和深度模型。

workspace 校验失败时不会调用 MCU 服务。

## 10. 从只计算切换到自动发送

第一阶段：

```yaml
auto_send: false
```

分别选择中心、四角和不同层目标，检查日志中的：

- 像素坐标；
- 同步时间差；
- 深度或平面 Z；
- base 坐标；
- workspace 判断。

完成空载验证后再设置：

```yaml
auto_send: true
```

manual 模式还要求：

```yaml
plane_heights_configured: true
```

## 11. 常见问题

### 改 camera_to_plane 深度但坐标不变

这是当前代码的预期行为。manual 模式使用 `planeX_z_m` 射线求交，未使用 `camera_to_planeX_distance_m`。

### 一个目标正确，四个目标误差不同

优先检查内参畸变、手眼旋转误差、相机固定、检测中心定义和工作平面倾斜。这类误差通常不是统一 XY 偏置。

### 第二天相同参数效果不同

检查相机或末端支架是否松动、机械臂零位是否变化、焦距/自动对焦是否变化、工作台是否移动。

### PnP 深度变化很大

核对四点排序、真实中心距和重投影误差。如果目标不是刚性矩形，改用 manual 固定平面方式。

### 找不到标定文件

确认文件已复制到 `handeye_bridge/config/`，重新 `colcon build` 并 `source install/setup.bash`。不要只修改源码目录而继续使用旧安装空间。

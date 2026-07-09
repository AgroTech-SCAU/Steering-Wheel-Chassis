# atlas_vision_pollination_backend

`atlas_vision_pollination_backend` 负责视觉授粉任务

它包含视觉目标 service，预识别动作执行，手眼变换，工具点偏移计算和机械臂授粉序列

---

## 一，职责

本包负责

```text
提供 /vision/detect_camera_target
接收 /atlas/manipulation/start
执行 prepare_action
调用视觉 service 获取目标相机坐标
把目标转换到 arm_base_link
根据工具点偏移计算预授粉和授粉位置
调用 /mcu/set_arm_position
等待机械臂到位
发布 /atlas/manipulation/status
```

本包不负责

```text
底盘导航
总任务 START 和 RESET
最终 DONE 和 FAIL 上报
串口协议解析
```

---

## 二，主要节点

### atlas_camera_target_service

提供

```text
/vision/detect_camera_target
```

服务类型

```text
atlas_mission_interfaces/srv/DetectCameraTarget
```

响应中的 `target_camera_m` 是相机坐标系下目标点，单位 m

### atlas_vision_pollination_backend

提供

```text
/atlas/manipulation/start
/atlas/manipulation/cancel
```

发布

```text
/atlas/manipulation/status
```

调用

```text
/vision/detect_camera_target
/mcu/set_arm_joints
/mcu/set_arm_position
```

订阅

```text
/arm/joint_states
/arm/pose_position
```

---

## 三，配置文件

```text
config/camera_target.yaml
config/pollination.yaml
config/pollination_actions.yaml
```

### camera_target.yaml

配置相机和目标检测参数

常用字段

| 字段 | 说明 |
|---|---|
| `camera.index` | 相机编号 |
| `camera.width` | 图像宽度 |
| `camera.height` | 图像高度 |
| `camera.buffer_size` | 相机缓存数量 |
| `detect.timeout_s` | 单次检测超时时间 |
| `detect.warmup_frame_count` | 检测前丢弃的缓存帧数量 |

### pollination.yaml

配置后端运行参数和手眼参数

常用字段

| 字段 | 说明 |
|---|---|
| `actions_file` | 动作配置文件路径 |
| `joint_tolerance_rad` | 关节到位容差 |
| `position_tolerance_m` | 末端位置到位容差 |
| `motion_timeout_s` | 单次动作超时时间 |
| `handeye.rotation` | gripper 到 camera 的旋转矩阵 |
| `handeye.translation_m` | gripper 到 camera 的平移 |

### pollination_actions.yaml

配置预识别动作和授粉序列

预识别动作示例

```yaml
prepare_actions:
  pre_detect_nav_02:
    type: "joints"
    joints_rad: [1.606, 2.315, 5.875, 2.152, 3.141]
    speed_rad_s: 3.14
    timeout_s: 8.0
```

授粉任务示例

```yaml
arrival_tasks:
  visual_pollination:
    type: "visual_pollination"
    pre_pollination_tool_point_m: [0.05, -0.015, 0.097]
    pollination_tool_point_m: [0.05, -0.015, 0.087]
```

---

## 四，授粉动作序列

当前默认序列

```text
预识别位姿
  ↓
视觉识别
  ↓
预授粉位姿
  ↓
授粉位姿
  ↓
停留
  ↓
回到预授粉位姿
  ↓
回到预识别位姿
```

对应 YAML

```yaml
sequence:
  - type: "ensure_prepare_pose"
    name: "到达预识别位姿"

  - type: "visual_position"
    name: "到达预授粉位姿"
    tool_point_ref: "pre_pollination_tool_point_m"

  - type: "visual_position"
    name: "到达授粉位姿"
    tool_point_ref: "pollination_tool_point_m"

  - type: "dwell"
    name: "授粉停留"
    duration_s: 0.3

  - type: "visual_position"
    name: "回到预授粉位姿"
    tool_point_ref: "pre_pollination_tool_point_m"

  - type: "joints_action"
    name: "回到预识别位姿"
    action_ref: "prepare_action"
```

---

## 五，工具点配置

工具点偏移写在 tool0 坐标系下

当前最终授粉工具点

```yaml
pollination_tool_point_m: [0.05, -0.015, 0.087]
```

预授粉工具点

```yaml
pre_pollination_tool_point_m: [0.05, -0.015, 0.097]
```

调参规则

```text
接触过深，增大 z
接触不到，减小 z
左右偏差，调整 x 或 y
```

---

## 六，手眼计算逻辑

视觉目标只在预识别位姿下计算一次

```text
相机坐标目标
  ↓
gripper 到 camera 手眼变换
  ↓
arm_base_link 下目标点
  ↓
根据工具点偏移反推末端目标
```

后续预授粉，授粉，回退动作都使用同一个检测目标，避免重复使用过期相机坐标

---

## 七，启动

通常由总启动文件启动

```bash
ros2 launch atlas_mission_manager mission_stack.launch.py
```

单独启动

```bash
ros2 launch atlas_vision_pollination_backend vision_pollination.launch.py
```

查看状态

```bash
ros2 topic echo /atlas/manipulation/status
```

---

## 点位与多任务配置补充

当前路线配置已升级为 `pre_move_action + arrival_jobs` 结构

```text
pre_move_action
  在底盘移动前执行
  用于收臂，安全姿态，或移动到某个预识别位姿

arrival_jobs
  在底盘到点稳定后依次执行
  PASS_BY 点为空列表
  AREA_A 和 AREA_B 每个点默认两个 job
  AREA_C 每个点默认一个 job
```

视觉授粉任务支持 `visual_pollination_multi`

```text
visual_pollination_a_0_3  每个 job 识别 0 到 3 朵雌花
visual_pollination_b_0_3  每个 job 识别 0 到 3 朵雌花
visual_pollination_c_0_2  每个 job 识别 0 到 2 朵雌花
```

每朵花的动作序列在 `pollination_actions.yaml` 的 `per_target_sequence` 中配置

默认顺序为

```text
预识别位姿
→ 预授粉位姿
→ 授粉位姿
→ 授粉停留
→ 回到预授粉位姿
```

每个 job 完成后会执行 `after_all_targets_sequence`，默认回到预识别位姿

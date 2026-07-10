# Atlas PI 端视觉系统

本目录包含 Atlas 在树莓派 PI 端使用的视觉检测、视觉目标适配、手眼标定和作业动作后端。

当前默认链路已经切换为 `racom_vision`，旧视觉模型只作为回退方案保留。

---

## 一，当前默认链路

```text
racom_vision/vison_topic
  -> /vision_detect
  -> 输出目标类别与 u/v 像素点

atlas_racom_vision_backend
  -> 调用 /vision_detect
  -> 转换为 /vision/detect_camera_target
  -> 兼容 atlas_mission_manager 原有接口

atlas_vision_pollination_backend
  -> 接收 /atlas/manipulation/start
  -> 执行 prepare_action、视觉识别、手眼变换和机械臂动作序列
  -> 调用 /mcu/set_arm_position、/mcu/set_arm_joints
  -> 可通过 /mcu/set_suction 或机械臂服务附带字段控制末端吸盘
```

这样做的目的不是重写整套作业状态机，而是只替换视觉模型入口。任务 YAML、手眼变换、机械臂动作模板和状态发布接口都尽量保持兼容。

---

## 二，目录说明

| 目录 | 说明 |
|---|---|
| `racom_vision/atlas_racom_vision_backend` | 新增适配器，把 RACOM 检测服务转换成 Atlas 任务系统需要的视觉目标服务。 |
| `raicom_vsion/vison_topic` | 原 RACOM/RAICOM 检测服务源码目录，包名和目录名沿用历史拼写。 |
| `raicom_vsion/vison_topic_interfaces` | `VisionDetect.srv` 接口包。 |
| `atlas_vision_pollination_backend` | 原视觉作业后端，当前继续复用动作序列、手眼变换和机械臂控制逻辑。 |
| `handeye_calibration_tool` | 手眼标定相关工具。 |

---

## 三，总启动方式

推荐从任务系统总启动文件启动：

```bash
source ~/chassis-pi-ws/install/setup.bash
ros2 launch atlas_mission_manager mission_stack.launch.py
```

默认等价于：

```bash
ros2 launch atlas_mission_manager mission_stack.launch.py \
  navigation_backend:=full \
  manipulation_backend:=racom_vision
```

只回退旧视觉模型：

```bash
ros2 launch atlas_mission_manager mission_stack.launch.py manipulation_backend:=vision_pollination
```

---

## 四，RACOM 视觉接口

原始检测服务：

```text
/vision_detect
vison_topic_interfaces/srv/VisionDetect
```

适配后的任务视觉服务：

```text
/vision/detect_camera_target
atlas_mission_interfaces/srv/DetectCameraTarget
```

`racom_vision` 当前只输出目标类别和像素坐标，适配器使用 `default_depth_m`、`pixel_to_meter_x`、`pixel_to_meter_y` 做临时相机坐标近似转换。正式实车使用前，建议至少完成下面一种修正：

```text
1. 让 RACOM 模型或后处理直接输出相机坐标系三维点；
2. 接入深度相机或双目深度；
3. 通过固定工作平面和相机内参反投影计算真实三维目标点。
```

---

## 五，PI 端吸盘控制

PI 端现在有两种方式控制末端吸盘。

### 1. 独立服务控制

```bash
ros2 service call /mcu/set_suction std_srvs/srv/SetBool "{data: true}"
ros2 service call /mcu/set_suction std_srvs/srv/SetBool "{data: false}"
```

默认 `require_auto_pi_for_suction=true`，即 MCU 必须处于 `AutoPi`，避免手动模式下误触发。

### 2. 动作序列控制

独立动作：

```yaml
- type: suction
  enable: true
```

随机械臂目标一起下发：

```yaml
- type: visual_position
  name: "到达目标并打开吸盘"
  tool_point_ref: "pollination_tool_point_m"
  suction_enable: true
```

没有显式写 `suction_enable` 或 `suction` 时，不改变当前吸盘状态。

---

## 六，联调检查

```bash
ros2 service list | grep vision
ros2 service call /vision_detect vison_topic_interfaces/srv/VisionDetect "{start: true}"
ros2 service call /vision_detect vison_topic_interfaces/srv/VisionDetect "{start: false}"
ros2 service call /vision/detect_camera_target atlas_mission_interfaces/srv/DetectCameraTarget "{waypoint_id: test, task_id: test, max_targets: 1, target_class: ''}"
ros2 topic echo /atlas/manipulation/status
```

如果 `/vision/detect_camera_target` 能返回目标，但机械臂动作不执行，优先检查：

```text
/mcu_comm_bridge 是否启动
/arm/joint_states 是否有数据
/arm/pose_position 是否有数据
MCU 是否处于 AutoPi
/mcu/set_arm_position 服务是否存在
```

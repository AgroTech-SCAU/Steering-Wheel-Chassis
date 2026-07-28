# atlas_racom_vision_backend

本包用于把 `racom_vision` 的 `VisionDetect` 服务适配成 Atlas 任务系统已经使用的 `/vision/detect_camera_target` 服务。

## 为什么需要这个适配层

原来的 `atlas_vision_pollination_backend` 分成两层：

1. `camera_target_service`：模型推理并返回相机坐标目标。
2. `vision_pollination_backend`：根据视觉目标、手眼外参和动作序列控制机械臂。

现在新增的 `racom_vision` 已经能输出检测类别和像素坐标，但它的接口是 `vison_topic_interfaces/srv/VisionDetect`，返回的是 `u/v` 像素点，不是任务系统需要的 `DetectCameraTarget`。因此本包只做接口转换，保留原有任务状态机和机械臂动作流程。

## 数据流

```text
racom_vision/vision_detect_server
    └── /vision_detect  (VisionDetect: cls_id, cls_name, u_px, v_px)
          ↓
atlas_racom_camera_target_service
    └── /vision/detect_camera_target  (DetectCameraTarget: target_camera_m)
          ↓
atlas_vision_pollination_backend
    └── /atlas/manipulation/start / status
```

## 当前限制

`racom_vision` 目前只输出二维像素坐标，没有深度或真实三维坐标。本适配器使用 `default_depth_m + pixel_to_meter_x/y` 做临时近似转换：

```text
x = (u - image_center_u_px) * pixel_to_meter_x
y = (v - image_center_v_px) * pixel_to_meter_y
z = default_depth_m
```

这可以让任务链路先跑通，但要实现可靠实车作业，仍需要补充以下任一方案：

- 用深度相机或双目恢复目标深度；
- 通过固定工作距离和标定板实测 `pixel_to_meter_x/y/default_depth_m`；
- 让 `racom_vision` 直接输出相机坐标系三维点。

## 启动

单独启动适配器：

```bash
ros2 launch atlas_racom_vision_backend racom_camera_target.launch.py
```

完整任务栈中使用：

```bash
ros2 launch atlas_mission_manager mission_stack.launch.py manipulation_backend:=racom_vision
```

## 关键参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `service_name` | `/vision/detect_camera_target` | 对外兼容旧任务视觉接口。 |
| `vision_detect_service` | `/vision_detect` | racom_vision 原始检测服务。 |
| `scan_duration_s` | `1.5` | 单次识别持续时间。 |
| `default_depth_m` | `0.30` | 没有深度时使用的默认 z 值。 |
| `pixel_to_meter_x/y` | `0.00050` | 像素到米的近似比例。 |
| `target_order` | `center_first` | 多目标返回排序策略。 |


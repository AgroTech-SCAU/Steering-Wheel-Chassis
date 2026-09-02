# vison_topic — YOLO 视觉检测节点

`vison_topic` 从 USB 相机读取 `640×480` 图像，通过包内 `best.onnx` 检测目标，并发布检测框中心、类别、置信度、时间戳和角编号。

## 1. 节点与文件

| 项目 | 名称 |
|---|---|
| 服务端可执行程序 | `vision_detect_server` |
| 调试客户端 | `vision_detect_client` |
| launch 文件 | `vision_detect.launch.py` |
| 模型 | `resource/best.onnx` |
| 主代码 | `vison_topic/detect_and_send.py` |
| 相机公共配置 | `vison_topic/camera_utils.py` |

## 2. 构建

从 ROS 2 工作空间根目录执行：

```bash
colcon build --packages-select \
    vison_topic_interfaces vison_topic
source install/setup.bash
```

## 3. 启动

```bash
# 部署：默认关闭预览
ros2 launch vison_topic vision_detect.launch.py

# 调试：显示 OpenCV 预览
ros2 launch vison_topic vision_detect.launch.py no_preview:=false
```

直接运行：

```bash
ros2 run vison_topic vision_detect_server --no-preview
```

可用参数：

| 参数 | 说明 | 默认值 |
|---|---|---:|
| `--camera` | 相机编号 | `0` |
| `--conf` | 置信度阈值 | 配置中的 `0.55` |
| `--process-every-n` | 每 N 帧推理一次 | `2` |
| `--rate-hz` | 检测定时器频率 | `15` |
| `--service-name` | 检测服务名 | `vision_detect` |
| `--topic-name` | 原始检测话题名 | `vision_detections` |
| `--no-preview` | 禁用 OpenCV 窗口 | 关闭 |
| `--allow-unprepared` | 绕过初始位门禁，仅相机调试 | 关闭 |

`--allow-unprepared` 只能用于不连接机械臂的相机/算法调试，完整抓取流程禁止使用。

## 4. 相机要求

在线检测、相机标定和手眼采样共用 `camera_utils.py`：

```text
Linux 后端：V4L2
编码：MJPG
分辨率：640×480
帧率：30 FPS
缓存：1 帧
```

节点会读取真实帧检查分辨率。相机打不开、持续读帧失败或实际分辨率不一致时会报错，避免错误内参与图像混用。

YOLO 的 `640×640` 是 letterbox 后的模型输入；发布的 `u/v` 坐标始终映射回原始 `640×480` 图像。

## 5. ROS 接口

| 名称 | 类型 | 方向 | 用途 |
|---|---|---|---|
| `/vision_detect` | `VisionDetect` | 服务 | 开始或停止检测 |
| `/vision_detections` | `Float32MultiArray` | 发布 | 兼容用的原始检测数组 |
| `/detection_centers` | `DetectionCenterArray` | 发布 | 带时间戳和角编号的检测中心 |
| `/initial_pose_ready` | `Bool` | 订阅 | 机械臂初始观察位门禁 |

查看接口：

```bash
ros2 interface show vison_topic_interfaces/srv/VisionDetect
ros2 interface show vison_topic_interfaces/msg/DetectionCenterArray
ros2 interface show vison_topic_interfaces/msg/DetectionCenter
```

单个 `DetectionCenter` 包含：

| 字段 | 含义 |
|---|---|
| `cls_id` | 类别 ID |
| `cls_name` | 类别名 |
| `u`, `v` | 原始图像中的检测框中心像素 |
| `conf` | 置信度 |
| `corner_index` | 角编号 |

`DetectionCenterArray.header.stamp` 是该帧采集后的 ROS 时间，用于 bridge 匹配同一时刻的机械臂位姿。

## 6. 开始与停止检测

完整应用先确认机械臂已经到初始观察位：

```bash
ros2 topic echo /initial_pose_ready --once
```

输出 `data: true` 后开始检测：

```bash
ros2 service call /vision_detect \
    vison_topic_interfaces/srv/VisionDetect \
    "{start: true}"
```

查看实时结果：

```bash
ros2 topic echo /detection_centers
```

停止检测：

```bash
ros2 service call /vision_detect \
    vison_topic_interfaces/srv/VisionDetect \
    "{start: false}"
```

行为说明：

- `start=true` 只启动持续检测，响应中的 `count` 为 0；
- `start=false` 停止检测并返回本轮缓存的最佳检测组；
- 停止后的最佳帧会再次发布到 `/detection_centers`，并设置 `is_final_best: true`；
- 最佳帧保留原始采集时间，而不是停止服务的时间；
- 初始位门禁变成 `false` 时，检测会自动停止。

## 7. 排序与角编号

四角约定：

```text
0 左上 ───────── 1 右上
  │                 │
  │                 │
3 左下 ───────── 2 右下
```

当前代码最多取置信度最高的 4 个检测参与角分配。排序函数会根据当前点集的质心、象限和概念角位置分配编号，再按编号输出。

重要限制：

- 四个目标位置接近标准四角布局时，编号才具有明确的 TL/TR/BR/BL 含义；
- 两三个目标时，缺失角身份无法只靠当前点集可靠恢复；
- PnP 模式必须得到完整、顺序正确的四个角点；
- 目标严重旋转、挤在同一象限或混入误检时，角编号可能不稳定。

如要判断排序问题，先查看：

```bash
ros2 topic echo /detection_centers
```

同时打开预览，核对 `corner_index`、中心点和实际画面位置，不要只看服务数组的先后。

## 8. 最佳帧规则

当前实现用一帧中所有有效检测的置信度总和作为分数，缓存得分最高的一帧。

这意味着目标数量较多的帧可能因置信度总和更高而胜出。若实际应用严格要求四个目标，建议后续将规则改为：优先检测数量恰好为 4，再比较前四个目标的置信度。

## 9. 模型和检测配置

主要配置位于 `detect_and_send.py` 顶部的 `CONFIG`：

```python
"yolo": {
    "conf_threshold": 0.55,
    "class_names": ["luosi", "chilun"],
    "plane_area_threshold": 1500.0,
}
```

修改类别时，必须保证 `class_names` 的顺序与 ONNX 模型训练类别一致。

`plane_area_threshold` 只用于预览中的面积分类提示；实际抓取层由 `/pick_target.layer` 指定。

## 10. 常见问题

### 检测坐标跳动

检查光照、曝光、目标反光、检测框本身是否稳定，以及相机是否固定。中心点来自检测框中心，不一定等于螺丝几何中心。

### 排序每帧变化

检查是否有误检、漏检、目标跨越质心象限、多个目标中心过近。两三个目标不能可靠恢复缺失的固定角身份。

### 停止后选择了错误帧

确认 `is_final_best: true` 的消息内容，并注意当前最佳帧按置信度总和选取，不是按“最接近四个目标”选取。

### SSH 下无法显示预览

使用 `no_preview:=true`，或参照项目主 README 的 X11 转发说明。

